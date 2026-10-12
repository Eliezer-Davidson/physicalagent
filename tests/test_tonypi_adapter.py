"""Tests for physicalagent/robots/tonypi.py — the TonyPi humanoid adapter.

Everything runs offline through FakeTonyLink + ScriptedClock. The focus is the
SAFETY-CRITICAL logic: the tier clamp on every servo command (exact pulse AND
time asserted at the link), the two-gate action whitelist, that a refusal writes
NOTHING to the link, that stop/emergency_stop always work (even while e-stopped)
and settle the biped to a stance rather than freezing it, and that execute()
turns every failure into a refusal the model can read rather than an exception.
"""
import pytest

from physicalagent.robots.tonypi import TonyPiAdapter
from physicalagent.robots.tonypi_link import FakeTonyLink, rpc_result
from physicalagent.robots.tonypi_servo import ServoTier, PROTECTION_OVERRIDE_SUPPORTED
from physicalagent.robots.transport import ScriptedClock
from physicalagent.safety import TONYPI_SAFETY, SafetyGuard


def make_adapter(tier=ServoTier.LOCKED, results=None, snapshot=b"\xff\xd8jpeg",
                 unlocked=False, actions=None):
    link = FakeTonyLink(results=results, snapshot=snapshot)
    guard = SafetyGuard(TONYPI_SAFETY, motion_unlocked=unlocked)
    a = TonyPiAdapter(link, guard, clock=ScriptedClock(), tier=tier,
                      action_groups=actions)
    return a, link


# ---------------------------------------------------------------------------
# Tool surface
# ---------------------------------------------------------------------------
class TestToolSurface:
    def test_exposes_expected_tools(self):
        a, _ = make_adapter()
        names = {s.name for s in a.capabilities()}
        assert names == {"get_status", "look", "do_action", "set_joint",
                         "set_joints", "set_head", "stop", "emergency_stop"}

    def test_every_tool_declaration_is_wellformed(self):
        a, _ = make_adapter()
        for d in a.gemini_declarations():
            assert set(d) == {"name", "description", "parameters"}
            assert d["description"].strip()

    def test_motion_tool_names_are_exactly_the_four_movers(self):
        a, _ = make_adapter()
        assert a._motion_tool_names() == {"do_action", "set_joint",
                                          "set_joints", "set_head"}

    def test_read_and_halt_tools_are_not_motion_tools(self):
        a, _ = make_adapter()
        movers = a._motion_tool_names()
        for safe in ("get_status", "look", "stop", "emergency_stop"):
            assert safe not in movers


# ---------------------------------------------------------------------------
# Sensing
# ---------------------------------------------------------------------------
class TestSensing:
    def test_get_status_reports_servos_running_func_and_tier(self):
        a, _ = make_adapter(tier=ServoTier.NORMAL, results={
            "GetBusServosPulse": [True, [500] * 15, "GetBusServosPulse"],
            "GetRunningFunc": [True, [0], "GetRunningFunc"]})
        r = a.execute("get_status", {})
        assert r.ok
        assert r.payload["servo_positions"] == [500] * 15
        assert r.payload["running_func"] == 0
        assert r.payload["tier"] == "normal"

    def test_get_status_is_robust_when_the_servo_read_fails(self):
        # GetBusServosPulse can return E04 (a servo didn't answer); status must
        # still return, never crash into a refusal.
        a, _ = make_adapter(results={
            "GetBusServosPulse": [False, "E04 - Operation timeout!", "GetBusServosPulse"]})
        r = a.execute("get_status", {})
        assert r.ok
        assert r.payload["servo_read_ok"] is False

    def test_get_status_is_never_refused_even_while_estopped(self):
        a, _ = make_adapter()
        a._guard.estop()
        assert a.execute("get_status", {}).ok

    def test_look_attaches_the_camera_frame(self):
        a, _ = make_adapter(snapshot=b"\xff\xd8xyz")
        r = a.execute("look", {})
        assert r.ok
        assert r.payload.get("frame_captured") is True
        assert a.last_frame == b"\xff\xd8xyz"

    def test_look_works_while_estopped(self):
        a, _ = make_adapter()
        a._guard.estop()
        assert a.execute("look", {}).ok


# ---------------------------------------------------------------------------
# do_action — two-gate whitelist
# ---------------------------------------------------------------------------
class TestDoAction:
    def test_whitelisted_action_runs_runaction_with_name_and_times(self):
        a, link = make_adapter()
        r = a.execute("do_action", {"name": "wave"})
        assert r.ok, r.error
        assert link.last_call("RunAction") == ("RunAction", ["wave", 1])

    def test_non_whitelisted_action_is_refused_and_writes_nothing(self):
        a, link = make_adapter()
        r = a.execute("do_action", {"name": "left_kick"})
        assert not r.ok
        assert "left_kick" in r.error
        assert link.calls == []          # never reached the robot

    def test_action_not_in_the_adapters_own_list_is_refused(self):
        # defense in depth: even if the guard whitelist were opened, the
        # adapter's curated list is checked first.
        a, link = make_adapter(actions=("stand", "wave"))
        r = a.execute("do_action", {"name": "bow"})
        assert not r.ok
        assert link.calls == []

    def test_do_action_is_refused_while_estopped(self):
        a, link = make_adapter()
        a._guard.estop()
        r = a.execute("do_action", {"name": "wave"})
        assert not r.ok
        assert link.calls == []


# ---------------------------------------------------------------------------
# set_joint — the per-joint bus-servo control, clamped by the active tier
# ---------------------------------------------------------------------------
class TestSetJoint:
    def test_locked_clamps_pulse_into_band_and_raises_time_to_floor(self):
        a, link = make_adapter(tier=ServoTier.LOCKED)
        r = a.execute("set_joint", {"servo_id": 5, "pulse": 2500, "move_time_ms": 100})
        assert r.ok, r.error
        # BUS LOCKED band 350-650, floor 800 ms: 2500 -> 650, 100 -> 800. (2500 is
        # a PWM-scale value; on a bus servo it MUST clamp to 650, never pass through.)
        assert link.last_call("SetBusServoPulse") == ("SetBusServoPulse", [800, 1, 5, 650])

    def test_locked_clamps_a_low_pulse_up_to_350(self):
        a, link = make_adapter(tier=ServoTier.LOCKED)
        a.execute("set_joint", {"servo_id": 3, "pulse": 0, "move_time_ms": 1000})
        # BUS LOCKED floor is 350
        assert link.last_call("SetBusServoPulse") == ("SetBusServoPulse", [1000, 1, 3, 350])

    def test_full_tier_reaches_the_true_electrical_maximum(self):
        a, link = make_adapter(tier=ServoTier.FULL)
        a.execute("set_joint", {"servo_id": 5, "pulse": 9999, "move_time_ms": 10})
        # BUS FULL band 0-1000, floor 120 ms: 9999 -> 1000, 10 -> 120
        assert link.last_call("SetBusServoPulse") == ("SetBusServoPulse", [120, 1, 5, 1000])

    def test_full_tier_reaches_the_true_electrical_minimum(self):
        a, link = make_adapter(tier=ServoTier.FULL)
        a.execute("set_joint", {"servo_id": 5, "pulse": -50, "move_time_ms": 500})
        # BUS FULL floor is 0
        assert link.last_call("SetBusServoPulse") == ("SetBusServoPulse", [500, 1, 5, 0])

    def test_missing_move_time_uses_the_tier_floor(self):
        a, link = make_adapter(tier=ServoTier.NORMAL)
        # 500 is inside BUS NORMAL's 200-800 band, so the pulse passes through
        a.execute("set_joint", {"servo_id": 2, "pulse": 500})
        assert link.last_call("SetBusServoPulse") == ("SetBusServoPulse", [400, 1, 2, 500])

    def test_out_of_range_servo_id_is_refused_and_writes_nothing(self):
        a, link = make_adapter()
        r = a.execute("set_joint", {"servo_id": 99, "pulse": 1500})
        assert not r.ok
        assert link.calls == []

    def test_missing_pulse_is_refused_by_schema_validation(self):
        a, link = make_adapter()
        r = a.execute("set_joint", {"servo_id": 5})
        assert not r.ok
        assert "pulse" in r.error
        assert link.calls == []

    def test_set_joint_is_refused_while_estopped(self):
        a, link = make_adapter()
        a._guard.estop()
        r = a.execute("set_joint", {"servo_id": 5, "pulse": 1500})
        assert not r.ok
        assert link.calls == []


# ---------------------------------------------------------------------------
# set_joints — batched bus-servo control
# ---------------------------------------------------------------------------
class TestSetJoints:
    def test_batches_into_one_call_with_each_pulse_clamped(self):
        a, link = make_adapter(tier=ServoTier.LOCKED)
        r = a.execute("set_joints", {"joints": [
            {"servo_id": 1, "pulse": 2500},   # BUS LOCKED -> 650 (high clamp)
            {"servo_id": 2, "pulse": 0},      # BUS LOCKED -> 350 (low clamp)
        ], "move_time_ms": 200})              # -> 800 floor
        assert r.ok, r.error
        assert link.last_call("SetBusServoPulse") == (
            "SetBusServoPulse", [800, 2, 1, 650, 2, 350])

    def test_full_tier_batch_reaches_full_range(self):
        a, link = make_adapter(tier=ServoTier.FULL)
        a.execute("set_joints", {"joints": [
            {"servo_id": 4, "pulse": 9999},   # BUS FULL -> 1000
            {"servo_id": 7, "pulse": -50},    # BUS FULL -> 0
        ], "move_time_ms": 130})
        assert link.last_call("SetBusServoPulse") == (
            "SetBusServoPulse", [130, 2, 4, 1000, 7, 0])

    def test_a_bad_servo_id_in_the_batch_refuses_the_whole_batch(self):
        a, link = make_adapter()
        r = a.execute("set_joints", {"joints": [
            {"servo_id": 1, "pulse": 1500},
            {"servo_id": 99, "pulse": 1500},
        ]})
        assert not r.ok
        assert link.calls == []     # all-or-nothing: nothing partially sent

    def test_empty_joint_list_is_refused(self):
        a, link = make_adapter()
        r = a.execute("set_joints", {"joints": []})
        assert not r.ok
        assert link.calls == []


# ---------------------------------------------------------------------------
# set_head — pan/tilt PWM servos
# ---------------------------------------------------------------------------
class TestSetHead:
    def test_pan_drives_pwm_servo_id_2_clamped(self):
        a, link = make_adapter(tier=ServoTier.LOCKED)
        a.execute("set_head", {"pan": 2500, "move_time_ms": 100})
        # pan is head servo id 2 (logical centre 1500); LOCKED clamps 2500 -> 1800
        assert link.last_call("SetPWMServo") == ("SetPWMServo", [800, 1, 2, 1800])

    def test_tilt_drives_pwm_servo_id_1_clamped(self):
        a, link = make_adapter(tier=ServoTier.FULL)
        a.execute("set_head", {"tilt": 2500, "move_time_ms": 200})
        assert link.last_call("SetPWMServo") == ("SetPWMServo", [200, 1, 1, 2500])

    def test_both_pan_and_tilt_go_in_one_call_tilt_first(self):
        a, link = make_adapter(tier=ServoTier.FULL)
        a.execute("set_head", {"tilt": 1500, "pan": 1500, "move_time_ms": 300})
        assert link.last_call("SetPWMServo") == (
            "SetPWMServo", [300, 2, 1, 1500, 2, 1500])

    def test_neither_pan_nor_tilt_is_refused_and_writes_nothing(self):
        a, link = make_adapter()
        r = a.execute("set_head", {})
        assert not r.ok
        assert link.calls == []

    def test_set_head_is_refused_while_estopped(self):
        a, link = make_adapter()
        a._guard.estop()
        r = a.execute("set_head", {"pan": 1500})
        assert not r.ok
        assert link.calls == []


# ---------------------------------------------------------------------------
# Tier selection & the protection-override seam
# ---------------------------------------------------------------------------
class TestTierSelection:
    def test_default_adapter_tier_is_locked(self):
        a, link = make_adapter()   # no tier passed to TonyPiAdapter's default
        a.execute("set_joint", {"servo_id": 1, "pulse": 2500, "move_time_ms": 1000})
        # BUS LOCKED band caps at 650
        assert link.last_call("SetBusServoPulse")[1][3] == 650

    def test_constructing_without_a_tier_argument_defaults_to_locked(self):
        from physicalagent.robots.tonypi import TonyPiAdapter
        link = FakeTonyLink()
        a = TonyPiAdapter(link, SafetyGuard(TONYPI_SAFETY), clock=ScriptedClock())
        assert a._tier is ServoTier.LOCKED

    def test_escalating_to_full_changes_the_clamp(self):
        locked, l_link = make_adapter(tier=ServoTier.LOCKED)
        full, f_link = make_adapter(tier=ServoTier.FULL)
        locked.execute("set_joint", {"servo_id": 1, "pulse": 2500, "move_time_ms": 1000})
        full.execute("set_joint", {"servo_id": 1, "pulse": 2500, "move_time_ms": 1000})
        # BUS domain: LOCKED caps at 650, FULL reaches the 1000 electrical max
        assert l_link.last_call("SetBusServoPulse")[1][3] == 650
        assert f_link.last_call("SetBusServoPulse")[1][3] == 1000

    def test_protection_override_is_off_and_unreachable_through_normal_tools(self):
        # No tool disables servo protection; the seam constant is OFF, and
        # nothing in a servo move references it.
        assert PROTECTION_OVERRIDE_SUPPORTED is False
        a, link = make_adapter(tier=ServoTier.FULL)
        a.execute("set_joint", {"servo_id": 1, "pulse": 2500, "move_time_ms": 120})
        # only the servo-move call was issued; no protection-toggling method
        for method, _ in link.calls:
            assert "rotection" not in method and "nload" not in method


# ---------------------------------------------------------------------------
# stop / emergency_stop — always allowed, settle to a stance
# ---------------------------------------------------------------------------
class TestStop:
    def _stance_settled(self, link):
        # stop must halt then command the stable stance (RunAction 'stand')
        return link.last_call("RunAction") == ("RunAction", ["stand", 1])

    def test_stop_halts_both_ways_then_settles_to_stance(self):
        a, link = make_adapter()
        r = a.execute("stop", {})
        assert r.ok
        methods = [m for m, _ in link.calls]
        assert "StopActionGroup" in methods
        assert "StopBusServo" in methods
        assert self._stance_settled(link)

    def test_stop_order_is_halt_before_settle(self):
        a, link = make_adapter()
        a.execute("stop", {})
        methods = [m for m, _ in link.calls]
        assert methods.index("StopActionGroup") < methods.index("RunAction")
        assert methods.index("StopBusServo") < methods.index("RunAction")

    def test_stop_works_while_estopped(self):
        a, link = make_adapter()
        a._guard.estop()
        r = a.execute("stop", {})
        assert r.ok
        assert self._stance_settled(link)

    def test_emergency_stop_works_while_estopped(self):
        a, link = make_adapter()
        a._guard.estop()
        r = a.execute("emergency_stop", {})
        assert r.ok
        assert self._stance_settled(link)

    def test_stop_works_mid_motion_after_a_servo_move(self):
        a, link = make_adapter(tier=ServoTier.FULL)
        a.execute("set_joint", {"servo_id": 1, "pulse": 2500, "move_time_ms": 120})
        r = a.execute("stop", {})
        assert r.ok
        assert self._stance_settled(link)

    def test_stop_all_is_robust_when_the_halt_calls_raise(self):
        # a link that throws on the stop calls must still not raise out of stop_all
        class _AngryLink(FakeTonyLink):
            def call(self, method, params):
                if method in ("StopActionGroup", "StopBusServo"):
                    raise RuntimeError("link down")
                return super().call(method, params)
        link = _AngryLink()
        a = TonyPiAdapter(link, SafetyGuard(TONYPI_SAFETY), clock=ScriptedClock())
        a.stop_all()   # must not raise
        # it still attempted the settle stance despite the halt failures
        assert link.last_call("RunAction") == ("RunAction", ["stand", 1])


# ---------------------------------------------------------------------------
# Camera keepalive — offline, deterministic (no real thread)
# ---------------------------------------------------------------------------
class TestCameraKeepalive:
    def test_start_camera_loads_function_9_and_starts_it(self):
        a, link = make_adapter()
        a.start_camera()
        assert link.last_call("LoadFunc") == ("LoadFunc", [9])
        assert link.last_call("StartFunc") == ("StartFunc", [])

    def test_heartbeat_pings_the_deadman(self):
        a, link = make_adapter()
        a.heartbeat()
        assert link.last_call("Heartbeat") == ("Heartbeat", [])

    def test_look_works_without_starting_the_camera(self):
        a, link = make_adapter(snapshot=b"\xff\xd8frame")
        assert a.execute("look", {}).ok
        assert a.last_frame == b"\xff\xd8frame"


# ---------------------------------------------------------------------------
# Sensing-only profile withholds every TonyPi motion tool
# ---------------------------------------------------------------------------
class TestSensingOnlyWithholdsMotion:
    MOTION = {"do_action", "set_joint", "set_joints", "set_head"}
    SENSING = {"get_status", "look", "stop", "emergency_stop"}

    def _sensing_only_profile(self):
        from physicalagent.profiles.base import (
            LinkSpec, RobotProfile, SafeStatePolicy,
        )
        return RobotProfile(
            name="tonypi_bench", embodiment="humanoid",
            supervision=None,               # not proven -> no motion tools
            safe_state=SafeStatePolicy.SETTLE_TO_STANCE,
            safety=TONYPI_SAFETY,
            link=LinkSpec(kind="http_rpc", host="x", port=9030),
            embodiment_prompt="x",
            motion_tools_enabled=False,
        )

    def test_no_motion_tool_is_declared(self):
        link = FakeTonyLink()
        a = TonyPiAdapter(link, SafetyGuard(TONYPI_SAFETY), clock=ScriptedClock(),
                          robot_profile=self._sensing_only_profile())
        declared = {d["name"] for d in a.gemini_declarations()}
        assert declared & self.MOTION == set()
        assert declared == self.SENSING

    def test_every_motion_tool_is_refused_and_writes_nothing(self):
        link = FakeTonyLink()
        a = TonyPiAdapter(link, SafetyGuard(TONYPI_SAFETY), clock=ScriptedClock(),
                          robot_profile=self._sensing_only_profile())
        for tool, args in (("do_action", {"name": "wave"}),
                           ("set_joint", {"servo_id": 1, "pulse": 1500}),
                           ("set_joints", {"joints": [{"servo_id": 1, "pulse": 1500}]}),
                           ("set_head", {"pan": 1500})):
            r = a.execute(tool, args)
            assert r.ok is False, f"{tool} ran on a sensing-only robot"
            assert "sensing-only" in r.error
        assert link.calls == []

    def test_stop_still_works_on_a_sensing_only_robot(self):
        link = FakeTonyLink()
        a = TonyPiAdapter(link, SafetyGuard(TONYPI_SAFETY), clock=ScriptedClock(),
                          robot_profile=self._sensing_only_profile())
        assert a.execute("stop", {}).ok
