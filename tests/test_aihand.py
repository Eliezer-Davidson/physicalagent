"""Tests for the AiHand controller and its tools on the TonyPi adapter.

The AiHand is not a robot of its own here — it is a hand BOLTED TO a TonyPi arm,
so its tools ride on the TonyPi's tool surface and inherit that robot's guard,
e-stop and tier. What this file pins:

  * the hand tools appear only when a hand is actually attached, and their
    absence is loud, not silent;
  * every finger command is clamped by tier before it reaches the wire;
  * a refused command writes NOTHING to the hand;
  * the hand is included in stop/e-stop, because a closing fist that ignores an
    emergency stop is the whole reason this layer exists.
"""
import pytest

from physicalagent.robots import aihand_codec as codec
from physicalagent.robots.aihand import AiHand
from physicalagent.robots.aihand_link import FakeAiHandLink
from physicalagent.robots.tonypi import TonyPiAdapter
from physicalagent.robots.tonypi_link import FakeTonyLink
from physicalagent.robots.tonypi_servo import ServoTier
from physicalagent.robots.transport import ScriptedClock
from physicalagent.safety import TONYPI_SAFETY, SafetyError, SafetyGuard


def make_hand(tier=ServoTier.LOCKED, framing="lobot", fail=False):
    link = FakeAiHandLink(fail_on_send=fail)
    return AiHand(link, tier=tier, framing=framing), link


def make_adapter(tier=ServoTier.LOCKED, with_hand=True, unlocked=False):
    link = FakeTonyLink()
    guard = SafetyGuard(TONYPI_SAFETY, motion_unlocked=unlocked)
    hand, hand_link = (make_hand(tier=tier) if with_hand else (None, None))
    a = TonyPiAdapter(link, guard, clock=ScriptedClock(), tier=tier, hand=hand)
    return a, link, hand_link


# ---------------------------------------------------------------------------
# The controller
# ---------------------------------------------------------------------------
class TestSetFingers:
    def test_a_named_finger_reaches_the_wire_as_its_servo_id(self):
        hand, link = make_hand(tier=ServoTier.FULL)
        hand.set_fingers({"thumb": 1500})
        assert link.sent == [codec.encode_lobot(
            {codec.FINGER_IDS["thumb"]: 1500}, codec.TIER_MIN_MOVE_MS[ServoTier.FULL])]

    def test_several_fingers_go_out_in_one_frame(self):
        hand, link = make_hand(tier=ServoTier.FULL)
        hand.set_fingers({"thumb": 1500, "index": 1400})
        assert len(link.sent) == 1

    def test_an_unnamed_finger_is_left_alone(self):
        hand, link = make_hand(tier=ServoTier.FULL)
        hand.set_fingers({"thumb": 1500})
        frame = link.sent[0]
        assert frame[4] == 1          # servo count byte: exactly one

    def test_an_unknown_finger_raises_and_sends_nothing(self):
        hand, link = make_hand()
        with pytest.raises(ValueError, match="unknown finger"):
            hand.set_fingers({"toe": 1500})
        assert link.sent == []

    def test_an_empty_command_sends_nothing(self):
        hand, link = make_hand()
        hand.set_fingers({})
        assert link.sent == []

    def test_the_ascii_framing_can_be_selected(self):
        hand, link = make_hand(tier=ServoTier.FULL, framing="ascii")
        hand.set_fingers({"thumb": 1500})
        assert link.sent[0].startswith(b"L|1|")


class TestClampIsAppliedBeforeTheWire:
    def test_a_wild_value_is_clamped_to_the_locked_band(self):
        hand, link = make_hand(tier=ServoTier.LOCKED)
        hand.set_fingers({"thumb": 2500})
        _, hi = codec.tier_band(ServoTier.LOCKED)
        assert link.sent == [codec.encode_lobot(
            {1: hi}, codec.TIER_MIN_MOVE_MS[ServoTier.LOCKED])]

    def test_a_fast_move_is_slowed_to_the_tier_floor(self):
        hand, link = make_hand(tier=ServoTier.LOCKED)
        hand.set_fingers({"thumb": 1500}, move_time_ms=1)
        floor = codec.TIER_MIN_MOVE_MS[ServoTier.LOCKED]
        assert link.sent == [codec.encode_lobot({1: 1500}, floor)]

    def test_the_full_tier_reaches_the_electrical_limit(self):
        hand, link = make_hand(tier=ServoTier.FULL)
        hand.set_fingers({"thumb": 9999})
        assert link.sent == [codec.encode_lobot(
            {1: codec.PULSE_MAX}, codec.TIER_MIN_MOVE_MS[ServoTier.FULL])]


class TestRest:
    def test_rest_opens_every_finger(self):
        hand, link = make_hand(tier=ServoTier.FULL)
        hand.rest()
        assert len(link.sent) == 1
        assert link.sent[0][4] == len(codec.FINGER_IDS)

    def test_rest_is_clamped_like_any_other_command(self):
        hand, link = make_hand(tier=ServoTier.LOCKED)
        hand.rest()
        lo, hi = codec.tier_band(ServoTier.LOCKED)
        # every position in the frame sits inside the locked band
        frame = link.sent[0]
        for i in range(frame[4]):
            pos = int.from_bytes(frame[8 + i * 3: 10 + i * 3], "little")
            assert lo <= pos <= hi


class TestFailuresAreLoud:
    def test_an_unreachable_hand_raises_rather_than_pretending(self):
        hand, _ = make_hand(fail=True)
        with pytest.raises(OSError):
            hand.set_fingers({"thumb": 1500})


# ---------------------------------------------------------------------------
# The tool surface on the TonyPi
# ---------------------------------------------------------------------------
class TestHandToolSurface:
    def test_hand_tools_appear_when_a_hand_is_attached(self):
        a, _, _ = make_adapter(with_hand=True)
        names = {s.name for s in a.capabilities()}
        assert {"set_hand", "open_hand", "close_hand"} <= names

    def test_no_hand_tools_when_no_hand_is_attached(self):
        a, _, _ = make_adapter(with_hand=False)
        names = {s.name for s in a.capabilities()}
        assert not ({"set_hand", "open_hand", "close_hand"} & names)

    def test_the_base_tonypi_tools_are_unchanged_by_the_hand(self):
        a, _, _ = make_adapter(with_hand=True)
        names = {s.name for s in a.capabilities()}
        assert {"get_status", "look", "do_action", "set_joint", "set_joints",
                "set_head", "stop", "emergency_stop"} <= names

    def test_hand_tools_are_motion_tools(self):
        a, _, _ = make_adapter(with_hand=True)
        assert {"set_hand", "open_hand", "close_hand"} <= a._motion_tool_names()

    def test_every_hand_declaration_is_wellformed(self):
        a, _, _ = make_adapter(with_hand=True)
        for d in a.gemini_declarations():
            assert set(d) == {"name", "description", "parameters"}
            assert d["description"].strip()


class TestHandToolsGoThroughTheGuard:
    def test_set_hand_moves_a_finger(self):
        a, _, hand_link = make_adapter(tier=ServoTier.FULL)
        a.execute("set_hand", {"thumb": 1500})
        assert len(hand_link.sent) == 1

    def test_an_estopped_robot_refuses_and_sends_nothing(self):
        a, _, hand_link = make_adapter(with_hand=True)
        a._guard.estop()
        result = a.execute("set_hand", {"thumb": 1500})
        assert "stop" in str(result).lower() or "refus" in str(result).lower()
        assert hand_link.sent == []

    def test_close_hand_then_open_hand_both_reach_the_wire(self):
        a, _, hand_link = make_adapter(tier=ServoTier.FULL)
        a.execute("close_hand", {})
        a.execute("open_hand", {})
        assert len(hand_link.sent) == 2

    def test_an_unknown_finger_is_a_refusal_not_a_crash(self):
        a, _, hand_link = make_adapter(tier=ServoTier.FULL)
        result = a.execute("set_hand", {"toe": 1500})
        assert "error" in str(result).lower() or "refus" in str(result).lower()
        assert hand_link.sent == []


class TestStopIncludesTheHand:
    def test_stop_all_rests_the_hand(self):
        a, _, hand_link = make_adapter(tier=ServoTier.FULL)
        a.execute("set_hand", {"thumb": 1500})
        hand_link.sent.clear()
        a.stop_all()
        assert hand_link.sent, "stop must reach the hand, not just the body"

    def test_a_dead_hand_does_not_stop_the_body_from_settling(self):
        # stop_all is best-effort by contract: one failed link must never
        # prevent the biped from settling to a stance.
        link = FakeTonyLink()
        guard = SafetyGuard(TONYPI_SAFETY, motion_unlocked=False)
        hand = AiHand(FakeAiHandLink(fail_on_send=True), tier=ServoTier.LOCKED)
        a = TonyPiAdapter(link, guard, clock=ScriptedClock(), hand=hand)
        a.stop_all()          # must not raise
        assert link.calls, "the body still received its settle"

    def test_emergency_stop_reaches_the_hand(self):
        a, _, hand_link = make_adapter(tier=ServoTier.FULL)
        hand_link.sent.clear()
        a.execute("emergency_stop", {})
        assert hand_link.sent
