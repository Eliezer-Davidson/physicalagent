"""Tests for the AiHand codec — pure framing and clamping, no I/O, no hardware.

The AiHand's wire protocol is the ONE thing about this robot that is NOT yet
verified against the physical hand, and this module is built to say so out loud.
Two candidate framings are implemented side by side:

    LOBOT   b"\\x55\\x55" length cmd ...   the binary HiWonder servo-controller
            frame the fleet spec records for the AiHand ("USB-TTL 0x55 0x55")
    ASCII   b"L|n|time|id|pos...&"        the MiniHexa/ROSPug-style text frame

Neither is confirmed for THIS hand's micro:bit V2 firmware. The codec therefore
encodes both and the operator PROBES to find out which one moves fingers; the
probe result is what promotes a framing from candidate to verified. Tests pin
the SHAPE of each framing (so a probe result is interpretable), never the claim
that a given framing is the right one.
"""
import pytest

from physicalagent.robots import aihand_codec as codec
from physicalagent.robots.tonypi_servo import ServoTier


# ---------------------------------------------------------------------------
# Finger naming — the model speaks fingers, the wire speaks servo ids
# ---------------------------------------------------------------------------
class TestFingerMap:
    def test_five_named_fingers_plus_a_base_rotation(self):
        assert set(codec.FINGER_IDS) == {
            "thumb", "index", "middle", "ring", "pinky", "base"}

    def test_ids_are_distinct(self):
        ids = list(codec.FINGER_IDS.values())
        assert len(ids) == len(set(ids))

    def test_unknown_finger_is_rejected_by_name(self):
        with pytest.raises(ValueError, match="unknown finger"):
            codec.finger_id("toe")

    def test_known_finger_resolves(self):
        assert codec.finger_id("thumb") == codec.FINGER_IDS["thumb"]

    def test_finger_names_are_case_insensitive(self):
        assert codec.finger_id("THUMB") == codec.FINGER_IDS["thumb"]


# ---------------------------------------------------------------------------
# Clamping — the hand has NO feedback, so the clamp is the only protection
# ---------------------------------------------------------------------------
class TestClamp:
    def test_locked_is_the_narrowest_band(self):
        lo, hi = codec.tier_band(ServoTier.LOCKED)
        nlo, nhi = codec.tier_band(ServoTier.NORMAL)
        assert lo > nlo and hi < nhi

    def test_normal_is_narrower_than_full(self):
        nlo, nhi = codec.tier_band(ServoTier.NORMAL)
        flo, fhi = codec.tier_band(ServoTier.FULL)
        assert nlo > flo and nhi < fhi

    def test_full_is_the_electrical_range_and_no_wider(self):
        assert codec.tier_band(ServoTier.FULL) == (
            codec.PULSE_MIN, codec.PULSE_MAX)

    def test_a_high_value_is_clamped_down_to_the_tier_ceiling(self):
        _, hi = codec.tier_band(ServoTier.LOCKED)
        assert codec.clamp_finger(99999, ServoTier.LOCKED)[0] == hi

    def test_a_low_value_is_clamped_up_to_the_tier_floor(self):
        lo, _ = codec.tier_band(ServoTier.LOCKED)
        assert codec.clamp_finger(-500, ServoTier.LOCKED)[0] == lo

    def test_move_time_is_raised_to_the_tier_floor(self):
        _, t = codec.clamp_finger(1500, ServoTier.LOCKED, move_time_ms=1)
        assert t == codec.TIER_MIN_MOVE_MS[ServoTier.LOCKED]

    def test_move_time_none_takes_the_tier_default(self):
        _, t = codec.clamp_finger(1500, ServoTier.NORMAL, move_time_ms=None)
        assert t == codec.TIER_MIN_MOVE_MS[ServoTier.NORMAL]

    def test_a_generous_move_time_is_kept(self):
        _, t = codec.clamp_finger(1500, ServoTier.FULL, move_time_ms=3000)
        assert t == 3000

    def test_move_time_cannot_wrap_the_two_byte_field(self):
        _, t = codec.clamp_finger(1500, ServoTier.FULL, move_time_ms=10**9)
        assert t <= 0xFFFF


# ---------------------------------------------------------------------------
# ASCII framing — shape only
# ---------------------------------------------------------------------------
class TestAsciiFrame:
    def test_single_servo_frame_shape(self):
        assert codec.encode_ascii({1: 1500}, 1000) == b"L|1|1000|1|1500&"

    def test_count_matches_the_number_of_servos(self):
        frame = codec.encode_ascii({1: 500, 2: 600, 3: 700}, 800)
        assert frame.startswith(b"L|3|800|")

    def test_servos_are_emitted_in_ascending_id_order(self):
        # dict order must NOT leak into the wire frame: same command, same bytes.
        a = codec.encode_ascii({3: 700, 1: 500, 2: 600}, 800)
        b = codec.encode_ascii({1: 500, 2: 600, 3: 700}, 800)
        assert a == b

    def test_frame_is_terminated(self):
        assert codec.encode_ascii({1: 1500}, 1000).endswith(b"&")

    def test_empty_command_is_refused_rather_than_sent_as_a_noop(self):
        with pytest.raises(ValueError, match="no servos"):
            codec.encode_ascii({}, 1000)


# ---------------------------------------------------------------------------
# LOBOT binary framing — shape only
# ---------------------------------------------------------------------------
class TestLobotFrame:
    def test_frame_starts_with_the_double_header(self):
        assert codec.encode_lobot({1: 1500}, 1000).startswith(b"\x55\x55")

    def test_command_byte_is_move_servo(self):
        frame = codec.encode_lobot({1: 1500}, 1000)
        assert frame[3] == codec.CMD_SERVO_MOVE

    def test_length_byte_counts_every_byte_after_the_header(self):
        frame = codec.encode_lobot({1: 1500, 2: 600}, 1000)
        assert frame[2] == len(frame) - 2

    def test_servo_count_is_carried(self):
        frame = codec.encode_lobot({1: 1500, 2: 600}, 1000)
        assert frame[4] == 2

    def test_time_is_little_endian_two_bytes(self):
        frame = codec.encode_lobot({1: 1500}, 0x0102)
        assert frame[5] == 0x02 and frame[6] == 0x01

    def test_position_is_little_endian_two_bytes(self):
        frame = codec.encode_lobot({7: 0x0304}, 1000)
        assert frame[7] == 7            # servo id
        assert frame[8] == 0x04 and frame[9] == 0x03

    def test_servos_are_emitted_in_ascending_id_order(self):
        a = codec.encode_lobot({3: 700, 1: 500}, 800)
        b = codec.encode_lobot({1: 500, 3: 700}, 800)
        assert a == b

    def test_empty_command_is_refused(self):
        with pytest.raises(ValueError, match="no servos"):
            codec.encode_lobot({}, 1000)

    def test_a_value_too_wide_for_the_field_is_refused_not_truncated(self):
        # Silent truncation would move a finger somewhere nobody asked for.
        with pytest.raises(ValueError, match="out of range"):
            codec.encode_lobot({1: 70000}, 1000)


# ---------------------------------------------------------------------------
# Framing selection — the seam the probe resolves
# ---------------------------------------------------------------------------
class TestFramingSelection:
    def test_both_candidate_framings_are_registered(self):
        assert set(codec.FRAMINGS) == {"lobot", "ascii"}

    def test_encode_dispatches_on_framing_name(self):
        assert codec.encode({1: 1500}, 1000, framing="ascii") == \
            codec.encode_ascii({1: 1500}, 1000)

    def test_unknown_framing_raises_rather_than_defaulting(self):
        with pytest.raises(ValueError, match="unknown framing"):
            codec.encode({1: 1500}, 1000, framing="morse")

    def test_no_framing_is_marked_verified_until_hardware_says_so(self):
        # This assertion is the honesty gate. It flips ONLY when a probe has
        # moved a real finger, and flipping it is a human act with a date.
        assert codec.VERIFIED_FRAMING is None
