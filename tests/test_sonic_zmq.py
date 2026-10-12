"""Wire-format tests for the SONIC ZMQ link. No socket, no policy, no robot —
these pin the BYTES, because a malformed message here produces no error and no
motion, which is the hardest kind of bug to find on a robot.

The 1280-byte header is the specific trap: two example senders shipped inside
gear_sonic_deploy pad to 1024 and are silently never parsed by the production
subscriber. That constant gets a test of its own.
"""
from __future__ import annotations

import json
import struct

import pytest

from physicalagent.robots import sonic_zmq as z


def split(msg: bytes, topic: bytes):
    """(header_dict, payload) from a packed message."""
    assert msg.startswith(topic)
    rest = msg[len(topic):]
    header, payload = rest[:z.HEADER_SIZE], rest[z.HEADER_SIZE:]
    return json.loads(header.rstrip(b"\x00").decode()), payload


class TestHeader:
    def test_header_slot_is_1280_not_1024(self):
        """gear_sonic_deploy/include/.../zmq_packed_message_subscriber.hpp:99."""
        assert z.HEADER_SIZE == 1280

    def test_header_is_padded_to_exactly_the_slot(self):
        msg = z.encode_command(True, False, True)
        rest = msg[len(b"command"):]
        assert len(rest) - 3 == z.HEADER_SIZE   # 3 payload bytes

    def test_oversized_header_raises_rather_than_truncating(self):
        fields = [{"name": "x" * 200, "dtype": "f32", "shape": [3]} for _ in range(20)]
        with pytest.raises(ValueError, match="exceeds"):
            z._pack(b"planner", fields, b"")


class TestCommandTopic:
    def test_start_command_fields_and_values(self):
        h, p = split(z.encode_command(True, False, True), b"command")
        assert [f["name"] for f in h["fields"]] == ["start", "stop", "planner"]
        assert all(f["dtype"] == "u8" for f in h["fields"])
        assert struct.unpack("BBB", p) == (1, 0, 1)

    def test_emergency_stop_sets_stop_bit(self):
        _, p = split(z.encode_command(False, True, True), b"command")
        assert struct.unpack("BBB", p) == (0, 1, 1)


class TestPlannerTopic:
    def test_field_order_and_dtypes_match_the_subscriber(self):
        h, _ = split(z.encode_planner(2, (1, 0, 0), (1, 0, 0), 0.5, -1.0), b"planner")
        assert [(f["name"], f["dtype"], f["shape"]) for f in h["fields"]] == [
            ("mode", "i32", [1]), ("movement", "f32", [3]),
            ("facing", "f32", [3]), ("speed", "f32", [1]), ("height", "f32", [1])]

    def test_payload_is_little_endian_and_correctly_sized(self):
        _, p = split(z.encode_planner(2, (1, 0, 0), (0, 1, 0), 0.4, -1.0), b"planner")
        assert len(p) == 4 + 12 + 12 + 4 + 4
        mode, = struct.unpack_from("<i", p, 0)
        mv = struct.unpack_from("<fff", p, 4)
        fc = struct.unpack_from("<fff", p, 16)
        speed, height = struct.unpack_from("<ff", p, 28)
        assert (mode, mv, fc) == (2, (1.0, 0.0, 0.0), (0.0, 1.0, 0.0))
        assert speed == pytest.approx(0.4) and height == pytest.approx(-1.0)

    def test_minus_one_means_mode_default_not_a_real_value(self):
        """-1.0 is the sentinel for speed and height (input_command.hpp)."""
        _, p = split(z.encode_planner(1, (1, 0, 0), (1, 0, 0), -1.0, -1.0), b"planner")
        speed, height = struct.unpack_from("<ff", p, 28)
        assert speed == -1.0 and height == -1.0


class TestModeMapping:
    def test_speed_selects_slow_walk_below_the_threshold(self):
        assert z.SLOW_WALK_MAX_M_S == 0.8
        assert z.MODE_SLOW_WALK == 1 and z.MODE_WALK == 2

    def test_pose_names_map_to_documented_mode_ids(self):
        """The old POSE_MODES dict held three names; the mode table holds all 27.
        The mapping those three names resolved to must not have moved."""
        from physicalagent.robots import sonic_modes

        assert sonic_modes.resolve("stand").value == z.MODE_IDLE == 0
        assert sonic_modes.resolve("squat").value == z.MODE_SQUAT == 4
        assert sonic_modes.resolve("sit").value == z.MODE_SQUAT
        # damp is an e-stop, not a mode, and must not have acquired one
        with pytest.raises(ValueError):
            sonic_modes.resolve("damp")


class TestSelectModeSpeed:
    """What speed a named mode actually puts on the wire.

    This class exists because it did not, and the gap cost a capability: every
    set_mode call that did not name a speed sent 0.0, which the planner reads as
    "stand still" (localmotion_kplanner.hpp:64). Mode 17 under that command took
    one step and stopped, and the obvious reading -- that SONIC refuses flight
    phases -- was wrong. The mode was told not to go anywhere.

    Driven through the internals rather than a socket, the same way the damper
    tests are, because the question is arithmetic and not transport.
    """

    class Bare:
        """select_mode's state without ZmqSonicLink's constructor."""

        def __init__(self):
            import threading
            self._lock = threading.Lock()
            self._mode = z.MODE_IDLE
            self._speed = 0.0
            self._body_vel = (0.0, 0.0)
            self._yaw_rate = 0.0
            self._height = -1.0
            self._hold = False
            self._expires_at = 0.0

        select_mode = z.ZmqSonicLink.select_mode
        emergency_stop = staticmethod(lambda: {"kind": "emergency_stop"})

    def call(self, name, **kw):
        bare = self.Bare()
        return bare, bare.select_mode(name, **kw)

    def test_jump_forward_is_not_told_to_stand_still(self):
        """The regression this whole change is for."""
        bare, out = self.call("jump_forward")
        assert out["mode_id"] == 17
        assert out["speed"] == -1.0, "speed 0.0 is the bug: mode 17 stays put"
        assert bare._speed == -1.0

    @pytest.mark.parametrize("name", [
        "jump_forward", "dance_walk", "zombie_walk", "stealth_walk", "gun_walk",
        "scare_walk", "ledge_walk", "carry", "injured_walk", "stealth_walk_2",
        "walk", "boxing_walk"])
    def test_no_styled_walk_is_commanded_stationary(self, name):
        _, out = self.call(name)
        assert abs(out["speed"]) > 1e-9, f"{name} was told to stand still"

    def test_a_named_speed_still_wins_and_is_still_clamped(self):
        _, out = self.call("jump_forward", speed=99.0)
        assert out["speed"] == z.MAX_VX_M_S
        _, out = self.call("dance_walk", speed=0.3)
        assert out["speed"] == pytest.approx(0.3)

    def test_an_explicit_zero_is_honoured_and_holds(self):
        """Naming zero is a real request -- 'be in this mode but do not travel'
        -- and must stay distinguishable from naming nothing."""
        _, out = self.call("dance_walk", speed=0.0)
        assert out["speed"] == 0.0 and out["held"] is True

    def test_a_mode_default_speed_still_expires(self):
        """-1 travels, so it must be bounded. `abs(spd) > 0` rather than
        `spd > 0` is the whole reason this passes."""
        bare, out = self.call("dance_walk", duration_s=3.0)
        assert out["speed"] == -1.0
        assert out["held"] is False and bare._expires_at > 0.0

    def test_minus_one_is_not_dead_reckoned_as_reverse_motion(self):
        bare, _ = self.call("jump_forward")
        assert bare._body_vel == (0.0, 0.0)

    def test_grounded_modes_get_the_vendor_height_not_the_standing_floor(self):
        for name, want in (("kneel_two_legs", 0.4), ("crawl", 0.4),
                           ("elbow_crawl", 0.3), ("squat", 0.4)):
            _, out = self.call(name)
            assert out["height_m"] == pytest.approx(want), name

    def test_damp_is_still_an_estop_and_not_a_mode(self):
        _, out = self.call("damp")
        assert out["kind"] == "emergency_stop"


class TestUpperBodyDamper:
    """The arm reference is RAMPED, not stepped.

    SONIC applies no rate limit to `upper_body_position` -- the only slew clamp
    in the deploy binary is on the Dex3 hands -- so a step change is chased as
    hard as the policy's gains allow, which reads as a flinch. These exercise
    the ramp arithmetic without a socket by driving the internals directly.
    """

    class Bare:
        """ZmqSonicLink's ramp logic without its constructor, which binds a
        socket and starts a thread. Only the fields _advance_upper_body and
        upper_body_travel_time touch."""

        def __init__(self, cur, target, slew):
            import threading
            self._lock = threading.Lock()
            self._upper_body = list(cur)
            self._ub_target = None if target is None else list(target)
            self._ub_slew = slew

        advance = z.ZmqSonicLink._advance_upper_body
        travel = z.ZmqSonicLink.upper_body_travel_time

    def test_a_step_command_becomes_a_ramp(self):
        b = self.Bare([0.0] * 17, [1.0] + [0.0] * 16, 1.2)
        b.advance(0.05)                       # one 20 Hz tick
        assert b._upper_body[0] == pytest.approx(0.06)
        assert b._upper_body[0] < 1.0, "a tick must not jump to the target"

    def test_the_ramp_reaches_the_target_and_stops_there(self):
        b = self.Bare([0.0] * 17, [1.0] + [0.0] * 16, 1.2)
        for _ in range(100):
            b.advance(0.05)
        assert b._upper_body[0] == pytest.approx(1.0)
        b.advance(0.05)
        assert b._upper_body[0] == pytest.approx(1.0), "must not overshoot"

    def test_every_joint_moves_at_once_rather_than_in_sequence(self):
        b = self.Bare([0.0] * 17, [1.0] * 17, 1.2)
        b.advance(0.05)
        assert all(v == pytest.approx(0.06) for v in b._upper_body)

    def test_a_joint_already_at_target_does_not_drift(self):
        b = self.Bare([0.5] + [0.0] * 16, [0.5] + [0.0] * 16, 1.2)
        b.advance(0.05)
        assert b._upper_body[0] == pytest.approx(0.5)

    def test_ramp_goes_the_right_way_when_the_target_is_lower(self):
        b = self.Bare([1.0] + [0.0] * 16, [0.0] * 17, 1.2)
        b.advance(0.05)
        assert b._upper_body[0] == pytest.approx(0.94)

    def test_travel_time_is_the_worst_joint_not_the_first(self):
        cur = [0.0] * 17
        target = [0.1] * 17
        target[9] = 1.2                        # one joint travels much further
        b = self.Bare(cur, target, 1.2)
        assert b.travel() == pytest.approx(1.0)

    def test_travel_time_is_zero_when_the_policy_owns_the_arms(self):
        b = self.Bare([0.0] * 17, None, 1.2)
        assert b.travel() == 0.0
        b.advance(0.05)                        # and advancing is a no-op

    def test_released_arms_stop_ramping(self):
        b = self.Bare([0.4] * 17, None, 1.2)
        b.advance(0.05)
        assert b._upper_body == [0.4] * 17

    def test_the_default_rate_is_gentle_enough_to_look_deliberate(self):
        """A 70-degree shoulder swing should take about a second, not a blink.
        This is the number the whole change exists to set."""
        seconds = 1.22 / z.UPPER_BODY_SLEW_RAD_S
        assert 0.7 <= seconds <= 2.0, f"{seconds:.2f}s reads wrong"


class TestThePublishThreadOutlivesItsOwnFailures:
    """This 20 Hz thread is the ONLY continuous command channel to the robot.
    Unguarded, one raise inside it ended the stream permanently while every
    link call kept returning a success dict — and the planner drops to IDLE
    after ~1 s of silence, so the robot simply stops obeying while the console
    reports success. Driven through the internals, like the damper tests: the
    question is control flow, not transport.
    """

    class Bare:
        def __init__(self, explode_times):
            import threading
            self._stop_flag = threading.Event()
            self._publish_error_reported = False
            self.beats = 0
            self._explode_times = explode_times

        def _publish_beat(self, dt):
            self.beats += 1
            if self.beats <= self._explode_times:
                raise RuntimeError(f"beat {self.beats} exploded")

        def run_for(self, beats):
            import threading
            import time as _t
            loop = z.ZmqSonicLink._publish_loop.__get__(self)
            t = threading.Thread(target=loop, daemon=True)
            t.start()
            deadline = _t.monotonic() + 3.0
            while self.beats < beats and _t.monotonic() < deadline:
                _t.sleep(0.01)
            self._stop_flag.set()
            t.join(timeout=1.0)
            return t.is_alive()

    def test_a_raising_beat_does_not_end_the_command_stream(self, capsys):
        b = self.Bare(explode_times=1)
        still_alive = b.run_for(5)
        assert b.beats >= 5, "the publish loop stopped after the first raise"
        assert not still_alive, "the loop ignored its stop flag"
        assert "SONIC PUBLISH BEAT FAILED" in capsys.readouterr().out

    def test_a_permanently_broken_beat_reports_once_not_20_times_a_second(self, capsys):
        b = self.Bare(explode_times=10_000)
        b.run_for(8)
        assert capsys.readouterr().out.count("SONIC PUBLISH BEAT FAILED") == 1
