"""Tests for physicalagent/robots/minihexa.py — the MiniHexa hexapod adapter.

Everything runs offline through FakeMiniHexaLink + an injected recording sleep.
The focus is the SAFETY-CRITICAL logic driven by the hardware facts:

  * The firmware has NO watchdog and its walk LATCHES a velocity, so walk MUST be
    a bounded burst that ALWAYS ends with a stop frame (invariant I2).
  * No motion runs on an unreadable / too-low battery (invariant I1).
  * Per-joint (L) and action-stop (K|2) exist over USB only, never BLE (I3).
  * stop / emergency_stop are ALWAYS allowed and never blocked by e-stop or a low
    battery (I4).

Exact frame bytes are asserted for walk (a move then a stop), pose, and per-servo,
using the independently-tested codec so the assertions are exact without manual
byte arithmetic. execute() must turn every failure into a refusal, never raise.
"""
import pytest

from physicalagent.robots import minihexa_codec as codec
from physicalagent.robots.minihexa import MiniHexaAdapter
from physicalagent.robots.minihexa_link import FakeMiniHexaLink
from physicalagent.robots.minihexa_motion import MotionTier
from physicalagent.safety import MINIHEXA_SAFETY, SafetyGuard


def make_adapter(tier=MotionTier.LOCKED, battery_mv=8100, supports_per_joint=True,
                 actions=None):
    link = FakeMiniHexaLink(battery_mv=battery_mv,
                            supports_per_joint=supports_per_joint)
    guard = SafetyGuard(MINIHEXA_SAFETY)
    slept: list[float] = []
    a = MiniHexaAdapter(link, guard, tier=tier, sleep=slept.append,
                        action_groups=actions)
    return a, link, slept


# ---------------------------------------------------------------------------
# Tool surface
# ---------------------------------------------------------------------------
class TestToolSurface:
    def test_exposes_expected_tools(self):
        a, _, _ = make_adapter()
        names = {s.name for s in a.capabilities()}
        assert names == {"get_status", "walk", "set_pose", "do_action",
                         "set_servo", "set_servos", "stop", "emergency_stop"}

    def test_every_declaration_is_wellformed(self):
        a, _, _ = make_adapter()
        for d in a.gemini_declarations():
            assert set(d) == {"name", "description", "parameters"}
            assert d["description"].strip()

    def test_motion_tool_names_are_exactly_the_five_movers(self):
        a, _, _ = make_adapter()
        assert a._motion_tool_names() == {"walk", "set_pose", "do_action",
                                          "set_servo", "set_servos"}

    def test_read_and_halt_tools_are_not_motion_tools(self):
        a, _, _ = make_adapter()
        movers = a._motion_tool_names()
        for safe in ("get_status", "stop", "emergency_stop"):
            assert safe not in movers

    def test_always_allowed_tools(self):
        a, _, _ = make_adapter()
        assert a._always_allowed_tools() == {"get_status", "stop", "emergency_stop"}


# ---------------------------------------------------------------------------
# get_status — read-only, never refused
# ---------------------------------------------------------------------------
class TestGetStatus:
    def test_reports_battery_state_tier_kind_and_capability(self):
        a, _, _ = make_adapter(tier=MotionTier.NORMAL, battery_mv=8100)
        r = a.execute("get_status", {})
        assert r.ok
        assert r.payload["battery_mv"] == 8100
        assert r.payload["battery_state"] == "ok"
        assert r.payload["tier"] == "normal"
        assert r.payload["link_kind"] == "fake"
        assert r.payload["supports_per_joint"] is True

    def test_low_battery_is_reported_not_refused(self):
        a, _, _ = make_adapter(battery_mv=6900)
        r = a.execute("get_status", {})
        assert r.ok
        assert r.payload["battery_state"] == "warn"

    def test_unreadable_battery_is_reported_not_refused(self):
        a, _, _ = make_adapter(battery_mv=None)
        r = a.execute("get_status", {})
        assert r.ok
        assert r.payload["battery_mv"] is None
        assert r.payload["battery_state"] == "unreadable"

    def test_is_never_refused_even_while_estopped(self):
        a, _, _ = make_adapter()
        a._guard.estop()
        assert a.execute("get_status", {}).ok

    def test_swallows_a_link_read_error_into_a_flag(self):
        class _AngryLink(FakeMiniHexaLink):
            def read_battery_mv(self):
                raise RuntimeError("link down")
        a = MiniHexaAdapter(_AngryLink(), SafetyGuard(MINIHEXA_SAFETY),
                            sleep=lambda s: None)
        r = a.execute("get_status", {})
        assert r.ok
        assert r.payload["battery_read_ok"] is False


# ---------------------------------------------------------------------------
# walk — bounded burst that ALWAYS ends with a stop
# ---------------------------------------------------------------------------
class TestWalk:
    def test_locked_clamps_velocity_then_bursts_and_stops(self):
        a, link, slept = make_adapter(tier=MotionTier.LOCKED)
        r = a.execute("walk", {"vx": 50, "vy": 0, "turn": 0})
        assert r.ok, r.error
        # LOCKED cap 15: 50 -> 15. Move THEN stop, in that order.
        assert link.sent == [codec.encode_move(15, 0, 0), codec.encode_stop()]
        assert slept == [0.4]

    def test_full_reaches_the_move_byte_limit(self):
        a, link, slept = make_adapter(tier=MotionTier.FULL)
        a.execute("walk", {"vx": 999, "vy": -999, "turn": 2})
        assert link.sent == [codec.encode_move(50, -50, 2), codec.encode_stop()]
        assert slept == [1.5]

    def test_return_payload_reports_clamped_command_and_burst(self):
        a, _, _ = make_adapter(tier=MotionTier.NORMAL)
        r = a.execute("walk", {"vx": 100, "vy": 5, "turn": 1})
        assert r.payload["vx"] == 30 and r.payload["vy"] == 5
        assert r.payload["turn"] == 1
        assert r.payload["burst_s"] == 0.8
        assert r.payload["tier"] == "normal"

    def test_last_frame_is_always_the_stop(self):
        a, link, _ = make_adapter(tier=MotionTier.FULL)
        a.execute("walk", {"vx": 40, "vy": 0, "turn": 0})
        assert link.last_frame() == codec.encode_stop()

    def test_walk_is_refused_while_estopped_and_sends_nothing(self):
        a, link, _ = make_adapter()
        a._guard.estop()
        r = a.execute("walk", {"vx": 10, "vy": 0, "turn": 0})
        assert not r.ok
        assert link.sent == []


# ---------------------------------------------------------------------------
# set_pose — F-frame body pose, byte-clamped per tier
# ---------------------------------------------------------------------------
class TestSetPose:
    def test_locked_clamps_pose_bytes(self):
        a, link, _ = make_adapter(tier=MotionTier.LOCKED)
        r = a.execute("set_pose", {"yaw": 50, "pitch": -50, "roll": 50,
                                   "x": -50, "y": 50, "height": 99})
        assert r.ok, r.error
        assert link.sent == [codec.encode_pose(15, -15, 15, -15, 15, 30)]

    def test_full_reaches_the_pose_byte_limit(self):
        a, link, _ = make_adapter(tier=MotionTier.FULL)
        a.execute("set_pose", {"yaw": 99, "pitch": 0, "roll": 0,
                               "x": 0, "y": 0, "height": 10})
        assert link.sent == [codec.encode_pose(50, 0, 0, 0, 0, 10)]

    def test_missing_a_field_is_refused_by_schema(self):
        a, link, _ = make_adapter()
        r = a.execute("set_pose", {"yaw": 0, "pitch": 0, "roll": 0, "x": 0, "y": 0})
        assert not r.ok
        assert "height" in r.error
        assert link.sent == []

    def test_set_pose_is_refused_while_estopped(self):
        a, link, _ = make_adapter()
        a._guard.estop()
        r = a.execute("set_pose", {"yaw": 0, "pitch": 0, "roll": 0,
                                   "x": 0, "y": 0, "height": 10})
        assert not r.ok
        assert link.sent == []


# ---------------------------------------------------------------------------
# do_action — the EMPTY whitelist refuses every group
# ---------------------------------------------------------------------------
class TestDoAction:
    def test_every_group_is_refused_by_the_empty_whitelist_and_sends_nothing(self):
        a, link, _ = make_adapter()
        for gid in (1, 2, 5, 99):
            r = a.execute("do_action", {"group_id": gid})
            assert not r.ok, f"group {gid} was allowed"
        assert link.sent == []

    def test_a_group_not_in_the_adapter_list_is_refused_first(self):
        # even if a curated list were provided, a group outside it is refused
        # before the guard, and writes nothing.
        a, link, _ = make_adapter(actions=("7",))
        r = a.execute("do_action", {"group_id": 3})
        assert not r.ok
        assert link.sent == []

    def test_do_action_is_refused_while_estopped(self):
        a, link, _ = make_adapter()
        a._guard.estop()
        r = a.execute("do_action", {"group_id": 1})
        assert not r.ok
        assert link.sent == []


# ---------------------------------------------------------------------------
# set_servo / set_servos — USB-only per-joint control
# ---------------------------------------------------------------------------
class TestSetServo:
    def test_serial_clamps_duty_and_time_then_sends_L_frame(self):
        a, link, _ = make_adapter(tier=MotionTier.LOCKED)
        r = a.execute("set_servo", {"servo_id": 5, "duty": 2500, "move_time_ms": 10})
        assert r.ok, r.error
        # LOCKED band 1200-1800, floor 800 ms: 2500 -> 1800, 10 -> 800
        assert link.last_frame() == codec.encode_servo([(5, 1800)], 800)

    def test_full_reaches_electrical_max(self):
        a, link, _ = make_adapter(tier=MotionTier.FULL)
        a.execute("set_servo", {"servo_id": 5, "duty": 9999, "move_time_ms": 10})
        assert link.last_frame() == codec.encode_servo([(5, 2500)], 120)

    def test_missing_move_time_uses_the_tier_floor(self):
        a, link, _ = make_adapter(tier=MotionTier.NORMAL)
        a.execute("set_servo", {"servo_id": 2, "duty": 1500})
        assert link.last_frame() == codec.encode_servo([(2, 1500)], 400)

    def test_ble_refuses_per_joint_and_sends_nothing(self):
        a, link, _ = make_adapter(supports_per_joint=False)
        r = a.execute("set_servo", {"servo_id": 5, "duty": 1500})
        assert not r.ok
        assert "USB-only" in r.error or "USB only" in r.error
        assert link.sent == []

    def test_bad_servo_id_is_refused_and_sends_nothing(self):
        a, link, _ = make_adapter()
        r = a.execute("set_servo", {"servo_id": 99, "duty": 1500})
        assert not r.ok
        assert link.sent == []

    def test_set_servo_is_refused_while_estopped(self):
        a, link, _ = make_adapter()
        a._guard.estop()
        r = a.execute("set_servo", {"servo_id": 1, "duty": 1500})
        assert not r.ok
        assert link.sent == []


class TestSetServos:
    def test_batch_clamps_every_entry_and_sends_one_L_frame(self):
        a, link, _ = make_adapter(tier=MotionTier.LOCKED)
        r = a.execute("set_servos", {"servos": [
            {"servo_id": 1, "duty": 2500},   # -> 1800
            {"servo_id": 2, "duty": 0},      # -> 1200
        ], "move_time_ms": 200})             # -> 800 floor
        assert r.ok, r.error
        assert link.last_frame() == codec.encode_servo([(1, 1800), (2, 1200)], 800)

    def test_one_bad_id_refuses_the_whole_batch_and_sends_nothing(self):
        a, link, _ = make_adapter()
        r = a.execute("set_servos", {"servos": [
            {"servo_id": 1, "duty": 1500},
            {"servo_id": 99, "duty": 1500},
        ]})
        assert not r.ok
        assert link.sent == []            # all-or-nothing

    def test_empty_batch_is_refused(self):
        a, link, _ = make_adapter()
        r = a.execute("set_servos", {"servos": []})
        assert not r.ok
        assert link.sent == []

    def test_ble_refuses_the_batch_and_sends_nothing(self):
        a, link, _ = make_adapter(supports_per_joint=False)
        r = a.execute("set_servos", {"servos": [{"servo_id": 1, "duty": 1500}]})
        assert not r.ok
        assert link.sent == []


# ---------------------------------------------------------------------------
# stop / emergency_stop — always allowed, K|2 only on serial
# ---------------------------------------------------------------------------
class TestStop:
    def test_serial_stop_sends_move_stop_then_action_stop(self):
        a, link, _ = make_adapter(supports_per_joint=True)
        r = a.execute("stop", {})
        assert r.ok
        assert link.sent == [codec.encode_stop(), codec.encode_action_stop()]

    def test_ble_stop_sends_only_the_move_stop(self):
        a, link, _ = make_adapter(supports_per_joint=False)
        r = a.execute("stop", {})
        assert r.ok
        assert link.sent == [codec.encode_stop()]

    def test_emergency_stop_behaves_like_stop(self):
        a, link, _ = make_adapter(supports_per_joint=True)
        assert a.execute("emergency_stop", {}).ok
        assert link.sent == [codec.encode_stop(), codec.encode_action_stop()]

    def test_stop_works_while_estopped(self):
        a, link, _ = make_adapter()
        a._guard.estop()
        r = a.execute("stop", {})
        assert r.ok
        assert codec.encode_stop() in link.sent

    def test_stop_all_is_best_effort_when_a_send_raises(self):
        class _AngryLink(FakeMiniHexaLink):
            def send(self, frame):
                if frame == codec.encode_stop():
                    raise RuntimeError("link down")
                return super().send(frame)
        link = _AngryLink()
        a = MiniHexaAdapter(link, SafetyGuard(MINIHEXA_SAFETY), sleep=lambda s: None)
        a.stop_all()   # must not raise even though the first send blew up
        # the action-stop still went out despite the move-stop failing
        assert link.sent == [codec.encode_action_stop()]

    def test_stop_all_never_raises(self):
        a, _, _ = make_adapter()
        a.stop_all()  # no assertion needed — the point is it returns


# ---------------------------------------------------------------------------
# Tier selection
# ---------------------------------------------------------------------------
class TestTierSelection:
    def test_default_adapter_tier_is_locked(self):
        link = FakeMiniHexaLink()
        a = MiniHexaAdapter(link, SafetyGuard(MINIHEXA_SAFETY), sleep=lambda s: None)
        assert a._tier is MotionTier.LOCKED

    def test_escalating_to_full_widens_the_walk_clamp(self):
        locked, l_link, _ = make_adapter(tier=MotionTier.LOCKED)
        full, f_link, _ = make_adapter(tier=MotionTier.FULL)
        locked.execute("walk", {"vx": 40, "vy": 0, "turn": 0})
        full.execute("walk", {"vx": 40, "vy": 0, "turn": 0})
        assert l_link.sent[0] == codec.encode_move(15, 0, 0)   # LOCKED caps at 15
        assert f_link.sent[0] == codec.encode_move(40, 0, 0)   # FULL passes 40


# ---------------------------------------------------------------------------
# RobotAdapter required surface
# ---------------------------------------------------------------------------
class TestRequiredSurface:
    def test_get_frame_returns_no_camera_frame(self):
        a, _, _ = make_adapter()
        assert a.get_frame() in (None, b"")

    def test_telemetry_matches_status(self):
        a, _, _ = make_adapter()
        t = a.telemetry()
        assert "battery_mv" in t and "tier" in t


# ---------------------------------------------------------------------------
# Sensing-only profile withholds every MiniHexa motion tool (mirror TonyPi)
# ---------------------------------------------------------------------------
class TestSensingOnlyWithholdsMotion:
    MOTION = {"walk", "set_pose", "do_action", "set_servo", "set_servos"}
    SENSING = {"get_status", "stop", "emergency_stop"}

    def _sensing_only_profile(self):
        from physicalagent.profiles.base import (
            LinkSpec, RobotProfile, SafeStatePolicy,
        )
        return RobotProfile(
            name="minihexa_bench", embodiment="hexapod",
            supervision=None,               # not proven -> no motion tools
            safe_state=SafeStatePolicy.FREEZE,
            safety=MINIHEXA_SAFETY,
            link=LinkSpec(kind="serial", device="/dev/ttyUSB0", baud=115200),
            embodiment_prompt="x",
            motion_tools_enabled=False,
        )

    def test_no_motion_tool_is_declared(self):
        a = MiniHexaAdapter(FakeMiniHexaLink(), SafetyGuard(MINIHEXA_SAFETY),
                            robot_profile=self._sensing_only_profile(),
                            sleep=lambda s: None)
        declared = {d["name"] for d in a.gemini_declarations()}
        assert declared & self.MOTION == set()
        assert declared == self.SENSING

    def test_every_motion_tool_is_refused_and_sends_nothing(self):
        link = FakeMiniHexaLink()
        a = MiniHexaAdapter(link, SafetyGuard(MINIHEXA_SAFETY),
                            robot_profile=self._sensing_only_profile(),
                            sleep=lambda s: None)
        for tool, args in (("walk", {"vx": 5, "vy": 0, "turn": 0}),
                           ("set_pose", {"yaw": 0, "pitch": 0, "roll": 0,
                                         "x": 0, "y": 0, "height": 10}),
                           ("do_action", {"group_id": 1}),
                           ("set_servo", {"servo_id": 1, "duty": 1500}),
                           ("set_servos", {"servos": [{"servo_id": 1, "duty": 1500}]})):
            r = a.execute(tool, args)
            assert r.ok is False, f"{tool} ran on a sensing-only robot"
            assert "sensing-only" in r.error
        assert link.sent == []

    def test_stop_still_works_on_a_sensing_only_robot(self):
        link = FakeMiniHexaLink()
        a = MiniHexaAdapter(link, SafetyGuard(MINIHEXA_SAFETY),
                            robot_profile=self._sensing_only_profile(),
                            sleep=lambda s: None)
        assert a.execute("stop", {}).ok
        assert codec.encode_stop() in link.sent


# ===========================================================================
# THE FOUR NON-NEGOTIABLE SAFETY INVARIANTS — each its own dedicated test
# ===========================================================================
class TestInvariantI1_NoMotionOnBadBattery:
    """I1. No motion tool executes on an unreadable or too-low battery: the frame
    buffer stays EMPTY and the call is refused."""

    @pytest.mark.parametrize("battery_mv", [6999, 6600, 100, None])
    @pytest.mark.parametrize("tool,args", [
        ("walk", {"vx": 10, "vy": 0, "turn": 0}),
        ("set_pose", {"yaw": 0, "pitch": 0, "roll": 0, "x": 0, "y": 0, "height": 10}),
        ("do_action", {"group_id": 1}),
        ("set_servo", {"servo_id": 1, "duty": 1500}),
        ("set_servos", {"servos": [{"servo_id": 1, "duty": 1500}]}),
    ])
    def test_motion_refused_and_nothing_sent(self, battery_mv, tool, args):
        a, link, _ = make_adapter(battery_mv=battery_mv)
        r = a.execute(tool, args)
        assert r.ok is False, f"{tool} ran at {battery_mv} mV"
        assert link.sent == [], f"{tool} sent a frame at {battery_mv} mV"


class TestInvariantI2_WalkNeverLeavesRobotMoving:
    """I2. After a successful walk, the LAST frame is the stop, and a move frame
    preceded it — the velocity-latched walk is never left running."""

    def test_last_frame_is_stop_and_a_move_came_first(self):
        a, link, _ = make_adapter(tier=MotionTier.FULL, battery_mv=8100)
        r = a.execute("walk", {"vx": 30, "vy": 10, "turn": 1})
        assert r.ok, r.error
        assert link.sent[-1] == codec.encode_stop()
        assert link.sent[-2] == codec.encode_move(30, 10, 1)
        assert len(link.sent) == 2


class TestInvariantI3_PerJointAndActionStopAreBleRefused:
    """I3. Per-joint (L) and action-stop (K|2) are USB-only. Over a BLE-shaped
    link they must be refused / withheld and NEVER emitted."""

    def test_set_servo_and_set_servos_refused_over_ble_send_nothing(self):
        a, link, _ = make_adapter(supports_per_joint=False)
        for tool, args in (("set_servo", {"servo_id": 1, "duty": 1500}),
                           ("set_servos", {"servos": [{"servo_id": 1, "duty": 1500}]})):
            r = a.execute(tool, args)
            assert r.ok is False
            assert link.sent == []

    def test_stop_over_ble_sends_move_stop_but_not_action_stop(self):
        a, link, _ = make_adapter(supports_per_joint=False)
        a.execute("stop", {})
        assert codec.encode_stop() in link.sent
        assert codec.encode_action_stop() not in link.sent

    def test_stop_over_serial_does_send_action_stop(self):
        a, link, _ = make_adapter(supports_per_joint=True)
        a.execute("stop", {})
        assert codec.encode_action_stop() in link.sent


class TestInvariantI4_StopAlwaysAllowed:
    """I4. stop / emergency_stop are always allowed — even after e-stop, even on a
    dead-flat battery — and a low battery never blocks a stop."""

    def test_stop_works_after_estop(self):
        a, link, _ = make_adapter(supports_per_joint=True)
        a._guard.estop()
        r = a.execute("stop", {})
        assert r.ok
        assert link.sent == [codec.encode_stop(), codec.encode_action_stop()]

    def test_emergency_stop_works_after_estop(self):
        a, link, _ = make_adapter(supports_per_joint=True)
        a._guard.estop()
        assert a.execute("emergency_stop", {}).ok

    def test_low_battery_does_not_block_stop(self):
        a, link, _ = make_adapter(battery_mv=100)   # flat pack
        r = a.execute("stop", {})
        assert r.ok
        assert codec.encode_stop() in link.sent

    def test_unreadable_battery_does_not_block_stop(self):
        a, link, _ = make_adapter(battery_mv=None)
        assert a.execute("stop", {}).ok
