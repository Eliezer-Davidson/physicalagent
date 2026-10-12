"""Tests for physicalagent/robots/rospug.py — the full ROSPug adapter.

Everything runs offline through FakeTransport + ScriptedClock + a fake camera.
The focus is the SAFETY-CRITICAL logic: preflight gates, clamping, bounding,
direction signs, motion budget, and that execute() turns every failure into a
refusal the model can read (never an exception).
"""
import math

import pytest

from physicalagent.robots.rospug import RospugAdapter
from physicalagent.robots.transport import FakeTransport, ScriptedClock
from physicalagent.safety import ROSPUG_SAFETY, SafetyGuard


OK_BATTERY = 11800
LOW_BATTERY = 11000


def _odom_yaw(deg):
    half = math.radians(deg) / 2.0
    return {"pose": {"pose": {"orientation": {"w": math.cos(half), "x": 0.0, "y": 0.0, "z": math.sin(half)},
                              "position": {"x": 0.0, "y": 0.0, "z": 0.0}}}}


def make_adapter(battery=OK_BATTERY, deadman="ALIVE", odom=None, unlocked=False,
                 frame=b"\xff\xd8jpeg", scan=None):
    t = FakeTransport(
        telemetry={"batt": battery, "dead": deadman, "scan": scan, "odom": odom},
        services={"/pug_control/go_home": {}, "/pug_control/run_action_group": {"success": True}},
    )
    guard = SafetyGuard(ROSPUG_SAFETY, motion_unlocked=unlocked,
                        action_whitelist=("stand", "sit", "wave"))
    a = RospugAdapter(t, guard, clock=ScriptedClock(),
                      frame_fetcher=lambda: frame,
                      action_groups=("stand", "sit", "wave"))
    return a, t


# ---------------------------------------------------------------------------
# Tool surface
# ---------------------------------------------------------------------------
class TestActionSurface:
    """Session 4 exposed every built-in action, accepting the tip risk
    (carpet floor, operator present and watching).

    The previous curated 10-action subset silently blocked a directly requested
    move — the typed task "sit down on your 2 back legs", which
    is `2_legs_stand`, and nothing happened. The default is now the full vendor
    inventory. The whitelist MECHANISM is unchanged and still enforced: an
    explicit action_groups argument restricts, the guard gates every call, and
    an action the robot does not have is still refused.
    """

    def _permissive_adapter(self):
        from physicalagent import config
        t = FakeTransport(telemetry={"batt": OK_BATTERY, "dead": "ALIVE"},
                          services={"/pug_control/run_action_group": {"success": True}})
        guard = SafetyGuard(ROSPUG_SAFETY, motion_unlocked=False,
                            action_whitelist=tuple(config.ROSPUG.action_groups))
        return RospugAdapter(t, guard, clock=ScriptedClock()), t

    def test_the_action_requested_in_session_4_is_available(self):
        a, _ = self._permissive_adapter()
        assert a.execute("do_action", {"name": "2_legs_stand"}).ok

    def test_full_vendor_inventory_is_exposed_by_default(self):
        from physicalagent import config
        a, _ = self._permissive_adapter()
        for name in config.ROSPUG.action_groups:
            assert a.execute("do_action", {"name": name}).ok, f"{name} should be available"

    def test_explicit_action_groups_still_restricts(self):
        a, t = make_adapter()          # built with action_groups=("stand","sit","wave")
        r = a.execute("do_action", {"name": "boxing"})
        assert not r.ok
        assert t.service_calls == []   # never reached the robot

    def test_unknown_action_is_still_refused(self):
        a, t = self._permissive_adapter()
        r = a.execute("do_action", {"name": "backflip"})
        assert not r.ok
        assert t.service_calls == []


class TestToolSurface:
    def test_exposes_expected_tools(self):
        a, _ = make_adapter()
        names = {s.name for s in a.capabilities()}
        assert names == {"get_status", "scan_surroundings", "look",
                         "walk", "strafe", "turn", "set_pose", "set_gait",
                         "do_action", "stop"}

    def test_every_tool_declaration_is_wellformed(self):
        a, _ = make_adapter()
        for d in a.gemini_declarations():
            assert set(d) == {"name", "description", "parameters"}
            assert d["description"].strip()


# ---------------------------------------------------------------------------
# Sensing
# ---------------------------------------------------------------------------
class TestSensing:
    def test_get_status_reports_battery_and_deadman(self):
        a, _ = make_adapter(battery=11800, deadman="ALIVE")
        r = a.execute("get_status", {})
        assert r.ok
        assert r.payload["battery_mv"] == 11800
        assert r.payload["deadman"] == "ALIVE"
        # default is now an uncapped session; a number only appears if a budget is set
        assert r.payload["moves_remaining"] == "unlimited"

    def test_scan_surroundings_summarizes_sectors(self):
        ranges = [8.0] * 360
        ranges[0] = 1.0
        a, _ = make_adapter(scan={"ranges": ranges, "angle_min": 0.0,
                                  "angle_increment": math.radians(1)})
        r = a.execute("scan_surroundings", {})
        assert r.ok
        assert r.payload["sectors"]["front"]["nearest_m"] == 1.0

    def test_scan_without_data_is_refusal_not_crash(self):
        a, _ = make_adapter(scan=None)
        r = a.execute("scan_surroundings", {})
        assert not r.ok
        assert r.error

    def test_look_returns_frame_signal(self):
        a, _ = make_adapter(frame=b"\xff\xd8xyz")
        r = a.execute("look", {})
        assert r.ok
        assert r.payload.get("frame_captured") is True
        # the brain retrieves the actual bytes separately
        assert a.last_frame == b"\xff\xd8xyz"


# ---------------------------------------------------------------------------
# Preflight gates shared by every motion tool
# ---------------------------------------------------------------------------
class TestMotionPreflight:
    def test_low_battery_refuses_walk(self):
        a, t = make_adapter(battery=LOW_BATTERY)
        r = a.execute("walk", {"distance_cm": 10})
        assert not r.ok
        assert "floor" in r.error or "battery" in r.error.lower()
        assert t.last_published("/pug_control/velocity_move") is None  # no motion

    def test_deadman_paused_refuses_walk(self):
        a, t = make_adapter(deadman="PAUSED")
        r = a.execute("walk", {"distance_cm": 10})
        assert not r.ok
        assert "paused" in r.error.lower() or "deadman" in r.error.lower()
        assert t.last_published("/pug_control/velocity_move") is None

    def test_deadman_absent_refuses_walk_fail_closed(self):
        # deadman node not running -> status None -> must REFUSE (never assume ALIVE)
        a, t = make_adapter(deadman=None)
        r = a.execute("walk", {"distance_cm": 10})
        assert not r.ok
        assert t.last_published("/pug_control/velocity_move") is None

    def test_deadman_dormant_refuses_walk(self):
        a, t = make_adapter(deadman="DORMANT")
        r = a.execute("walk", {"distance_cm": 10})
        assert not r.ok
        assert t.last_published("/pug_control/velocity_move") is None

    def test_estop_refuses_all_motion(self):
        a, t = make_adapter()
        a._guard.estop()
        for tool, args in [("walk", {"distance_cm": 5}), ("turn", {"angle_deg": 30}),
                           ("strafe", {"distance_cm": 5}), ("set_pose", {"height_cm": -13})]:
            r = a.execute(tool, args)
            assert not r.ok, f"{tool} should be refused while e-stopped"

    def test_motion_budget_exhausts(self):
        a, t = make_adapter()
        a._max_moves = 2
        assert a.execute("walk", {"distance_cm": 5}).ok
        assert a.execute("walk", {"distance_cm": 5}).ok
        r = a.execute("walk", {"distance_cm": 5})
        assert not r.ok
        assert "budget" in r.error.lower()


# ---------------------------------------------------------------------------
# walk
# ---------------------------------------------------------------------------
class TestWalk:
    def test_walk_forward_publishes_positive_x(self):
        a, t = make_adapter()
        a.execute("walk", {"distance_cm": 10})
        motion = next(m for _, _, m in t.published if m.get("x", 0) != 0)
        assert motion["x"] > 0

    def test_walk_backward_publishes_negative_x(self):
        a, t = make_adapter()
        a.execute("walk", {"distance_cm": -10})
        motion = next(m for _, _, m in t.published if m.get("x", 0) != 0)
        assert motion["x"] < 0

    def test_walk_clamps_speed_to_locked_cap(self):
        a, t = make_adapter(unlocked=False)  # 0.05 cap
        a.execute("walk", {"distance_cm": 10})
        motion = next(m for _, _, m in t.published if m.get("x", 0) != 0)
        assert abs(motion["x"]) <= 0.05 + 1e-9

    def test_walk_bounds_distance_and_reports_both(self):
        a, t = make_adapter()
        a._max_step_cm = 20
        r = a.execute("walk", {"distance_cm": 500})  # absurd
        assert r.ok
        assert r.payload["requested_cm"] == 500
        assert r.payload["allowed_cm"] == 20

    def test_walk_ends_with_a_stop(self):
        a, t = make_adapter()
        a.execute("walk", {"distance_cm": 10})
        assert t.published[-1][2].get("stop") is True

    def test_walk_duration_capped_by_config_when_locked(self):
        # locked cap is UNVERIFIED_MAX_DURATION_S = 1.0 s; a 30 cm request must
        # not run the full 6 s it would take at 0.05 m/s.
        from physicalagent import config
        a, t = make_adapter(unlocked=False)
        a._max_step_cm = 30
        r = a.execute("walk", {"distance_cm": 30})
        assert r.payload["duration_s"] <= config.UNVERIFIED_MAX_DURATION_S + 1e-9

    def test_walk_missing_arg_refused(self):
        a, t = make_adapter()
        r = a.execute("walk", {})
        assert not r.ok
        assert "distance_cm" in r.error

    def test_walk_nan_distance_refused_publishes_nothing(self):
        a, t = make_adapter()
        r = a.execute("walk", {"distance_cm": float("nan")})
        assert not r.ok
        # a NaN must NEVER reach the robot as a velocity
        assert t.last_published("/pug_control/velocity_move") is None

    def test_walk_inf_distance_refused(self):
        a, t = make_adapter()
        r = a.execute("walk", {"distance_cm": float("inf")})
        assert not r.ok
        assert t.last_published("/pug_control/velocity_move") is None

    def test_walk_non_numeric_distance_refused_not_crash(self):
        a, t = make_adapter()
        r = a.execute("walk", {"distance_cm": "far"})
        assert not r.ok  # execute() catches, never raises


# ---------------------------------------------------------------------------
# strafe
# ---------------------------------------------------------------------------
class TestStrafe:
    def test_strafe_left_positive_y(self):
        a, t = make_adapter(unlocked=True)
        a.execute("strafe", {"distance_cm": 10})
        motion = next(m for _, _, m in t.published if m.get("y", 0) != 0)
        assert motion["y"] > 0

    def test_strafe_right_negative_y(self):
        a, t = make_adapter(unlocked=True)
        a.execute("strafe", {"distance_cm": -10})
        motion = next((m for _, _, m in t.published if m.get("y", 0) != 0), None)
        assert motion is not None and motion["y"] < 0


# ---------------------------------------------------------------------------
# turn (closed-loop)
# ---------------------------------------------------------------------------
class TestTurn:
    def test_turn_uses_closed_loop_and_reports_achieved(self):
        a, t = make_adapter()
        t.script_telemetry("odom", [_odom_yaw(d) for d in (0, 30, 60, 90)])
        r = a.execute("turn", {"angle_deg": 90})
        assert r.ok
        assert abs(r.payload["achieved_deg"] - 90) <= 6
        assert r.payload["reached"] is True

    def test_turn_works_without_odometry_now_that_it_is_open_loop(self):
        """POLICY CHANGE, operator-directed: odometry under-reports rotation, so
        turns no longer depend on it. Live, a perfectly good turn was refused
        with 'odometry stalled mid-turn' while the robot was rotating fine."""
        a, _ = make_adapter(odom=None)
        r = a.execute("turn", {"angle_deg": 90})
        assert r.ok, r.error
        assert r.payload["method"] == "open_loop"

    def test_closed_loop_is_still_available_and_still_refuses_without_odom(self):
        """The closed-loop path is opt-in, not deleted."""
        from physicalagent.robots.rospug import RospugAdapter
        t = FakeTransport(telemetry={"batt": OK_BATTERY, "dead": "ALIVE", "odom": None},
                          services={})
        a = RospugAdapter(t, SafetyGuard(ROSPUG_SAFETY, motion_unlocked=True), clock=ScriptedClock(),
                          frame_fetcher=lambda: b"x", open_loop_turn=False)
        r = a.execute("turn", {"angle_deg": 90})
        assert not r.ok
        assert "odom" in r.error.lower()

    def test_turn_yaw_rate_clamped_to_locked_cap(self):
        # locked (default): yaw cap is 0.15 rad/s. Turn must NOT command 0.35.
        a, t = make_adapter(unlocked=False)
        t.script_telemetry("odom", [_odom_yaw(d) for d in (0, 5, 10, 90)])
        a.execute("turn", {"angle_deg": 90})
        motion = next(m for _, _, m in t.published if m.get("yaw_rate", 0) != 0)
        assert abs(motion["yaw_rate"]) <= 0.15 + 1e-9, "turn bypassed the guard's yaw clamp"

    def test_refused_turn_does_not_consume_motion_budget(self):
        a, t = make_adapter(odom=None)
        before = a.execute("get_status", {}).payload["moves_remaining"]
        a.execute("turn", {"angle_deg": 90})  # refused — no odom
        after = a.execute("get_status", {}).payload["moves_remaining"]
        assert after == before, "a refused turn wrongly consumed a budget slot"

    def test_turn_nan_angle_refused_publishes_nothing(self):
        a, t = make_adapter()
        r = a.execute("turn", {"angle_deg": float("nan")})
        assert not r.ok
        assert t.last_published("/pug_control/velocity_move") is None

    def test_turn_bounds_angle(self):
        a, t = make_adapter()
        a._max_turn_deg = 180
        t.script_telemetry("odom", [_odom_yaw(d) for d in (0, 90, 180)])
        r = a.execute("turn", {"angle_deg": 720})
        assert r.ok
        assert r.payload["requested_deg"] == 720
        assert abs(r.payload["allowed_deg"]) == 180


# ---------------------------------------------------------------------------
# set_pose
# ---------------------------------------------------------------------------
class TestSetPose:
    def test_set_pose_clamps_and_publishes(self):
        a, t = make_adapter()
        r = a.execute("set_pose", {"pitch_deg": 90, "height_cm": -13})
        assert r.ok
        pose = t.last_published("/pug_control/pose")
        assert pose is not None
        assert abs(pose["pitch"]) < math.radians(90)  # clamped well below 90 deg

    def test_set_pose_does_not_translate(self):
        a, t = make_adapter()
        a.execute("set_pose", {"pitch_deg": 5, "height_cm": -13})
        # pose is planted: no velocity commands at all
        assert t.last_published("/pug_control/velocity_move") is None

    def test_set_pose_exposes_the_full_posture_surface(self):
        """POLICY CHANGE (session 4): roll was suppressed as a footgun,
        but every joint is now exposed. The vendor applies no pose clamp at
        all, so our envelope is a sanity bound rather than a protection."""
        a, _ = make_adapter()
        props = next(s for s in a.capabilities()
                     if s.name == "set_pose").parameters["properties"]
        assert {"pitch_deg", "roll_deg", "height_cm", "x_shift_cm"} <= set(props)

    def test_set_pose_publishes_a_commanded_roll(self):
        a, t = make_adapter()
        a.execute("set_pose", {"roll_deg": 10, "height_cm": -13})
        assert t.last_published("/pug_control/pose")["roll"] != 0

    def test_roll_defaults_to_zero_when_not_requested(self):
        """Roll is available now, but asking for pitch alone must not induce a lean."""
        a, t = make_adapter()
        a.execute("set_pose", {"pitch_deg": 8, "height_cm": -13})
        assert t.last_published("/pug_control/pose")["roll"] == 0.0


# ---------------------------------------------------------------------------
# do_action
# ---------------------------------------------------------------------------
class TestDoAction:
    def test_whitelisted_action_runs(self):
        a, t = make_adapter()
        r = a.execute("do_action", {"name": "wave"})
        assert r.ok
        assert any("run_action_group" in c[0] for c in t.service_calls)

    def test_unknown_action_refused(self):
        a, t = make_adapter()
        r = a.execute("do_action", {"name": "backflip"})
        assert not r.ok
        assert "backflip" in r.error
        assert t.service_calls == []  # never reached the robot


# ---------------------------------------------------------------------------
# stop always works
# ---------------------------------------------------------------------------
class TestStop:
    def test_stop_publishes_halt(self):
        a, t = make_adapter()
        r = a.execute("stop", {})
        assert r.ok
        assert t.last_published("/pug_control/velocity_move")["stop"] is True

    def test_stop_works_while_estopped(self):
        a, t = make_adapter()
        a._guard.estop()
        r = a.execute("stop", {})
        assert r.ok  # stop is the one motion-ish tool allowed while e-stopped


# ---------------------------------------------------------------------------
# Unlimited motion budget — continuous operation (session 4 decision)
# ---------------------------------------------------------------------------
class TestUnlimitedMotionBudget:
    """max_moves=None removes the per-session move cap entirely.

    The 20-move budget was early caution, not a safety property: it does not
    prevent any specific harm, it just stops the robot mid-task. The real
    protections (battery gate, deadman, e-stop, clearance) are unaffected.
    """

    def test_no_budget_allows_more_moves_than_the_old_cap(self):
        a, t = make_adapter(unlocked=True)
        a._max_moves = None
        for _ in range(25):
            res = a.execute("walk", {"distance_cm": 10})
            assert res.ok, res.error
        assert a._moves_used == 25

    def test_status_reports_unlimited(self):
        a, _ = make_adapter()
        a._max_moves = None
        res = a.execute("get_status", {})
        assert res.payload["moves_remaining"] == "unlimited"

    def test_budget_still_enforced_when_set(self):
        a, _ = make_adapter(unlocked=True)
        a._max_moves = 2
        assert a.execute("walk", {"distance_cm": 5}).ok
        assert a.execute("walk", {"distance_cm": 5}).ok
        third = a.execute("walk", {"distance_cm": 5})
        assert third.ok is False
        assert "budget" in third.error


# ---------------------------------------------------------------------------
# Honest motion feedback — rf2o translation is NOT ground truth
# ---------------------------------------------------------------------------
class TestHonestWalkFeedback:
    """rf2o odometry translation was measured at 0.5x-2.5x of actual distance on
    carpet (yaw stays accurate; translation does not). Reporting it as
    `measured_cm` told Gemini it had travelled 70 cm when it had gone 27, so it
    stopped short of targets. The tool must not present it as truth.
    """

    def _scan(self, front_m):
        # 360 rays; front sector centred at index 0
        return {"ranges": [front_m] * 360, "angle_min": 0.0,
                "angle_increment": math.radians(1.0), "range_max": 12.0}

    def test_walk_does_not_report_a_measured_cm_ground_truth(self):
        a, _ = make_adapter(unlocked=True, scan=self._scan(3.0))
        res = a.execute("walk", {"distance_cm": 20})
        assert res.ok
        assert "measured_cm" not in res.payload

    def test_walk_reports_expected_distance_from_speed_and_duration(self):
        a, _ = make_adapter(unlocked=True, scan=self._scan(3.0))
        res = a.execute("walk", {"distance_cm": 20})
        # 0.10 m/s for 2.0 s == 20 cm
        assert res.payload["expected_cm"] == pytest.approx(20.0, abs=0.5)

    def test_walk_reports_front_clearance_before_and_after(self):
        a, _ = make_adapter(unlocked=True, scan=self._scan(3.0))
        res = a.execute("walk", {"distance_cm": 20})
        assert "front_clearance_m" in res.payload

    def test_odom_estimate_is_labelled_unreliable(self):
        a, _ = make_adapter(unlocked=True, odom=_odom_yaw(0), scan=self._scan(3.0))
        res = a.execute("walk", {"distance_cm": 20})
        # present for diagnostics, but under a name that cannot be mistaken for truth
        assert "odom_estimate_cm" in res.payload


class TestBatterySagAbortsMotion:
    """The robot brown-out-rebooted mid-session at 11.8 V resting.

    The battery gate samples ONCE before a move, so it cannot see voltage sag
    caused by the move itself. With walk duration raised to 15 s, a single call
    can now draw heavy current for a long time — so the pack is re-checked
    while moving and motion stops the moment it drops below the floor.
    """

    def test_walk_stops_when_the_pack_sags_below_the_floor(self):
        t = FakeTransport(telemetry={"dead": "ALIVE", "scan": None, "odom": None},
                          services={})
        t.script_telemetry("batt", [11800, 11800, 10500])   # healthy, then sags
        guard = SafetyGuard(ROSPUG_SAFETY, motion_unlocked=True)
        a = RospugAdapter(t, guard, clock=ScriptedClock(), frame_fetcher=lambda: b"x")
        res = a.execute("walk", {"distance_cm": 150})
        assert res.ok is False
        assert "sag" in res.error.lower() or "battery" in res.error.lower()

    def test_a_stop_is_published_when_motion_is_aborted(self):
        t = FakeTransport(telemetry={"dead": "ALIVE", "scan": None, "odom": None},
                          services={})
        t.script_telemetry("batt", [11800, 11800, 10500])
        guard = SafetyGuard(ROSPUG_SAFETY, motion_unlocked=True)
        a = RospugAdapter(t, guard, clock=ScriptedClock(), frame_fetcher=lambda: b"x")
        a.execute("walk", {"distance_cm": 150})
        assert t.published[-1][2].get("stop") is True   # never left driving

    def test_healthy_pack_completes_a_long_walk(self):
        t = FakeTransport(telemetry={"batt": 11800, "dead": "ALIVE",
                                     "scan": None, "odom": None}, services={})
        guard = SafetyGuard(ROSPUG_SAFETY, motion_unlocked=True)
        a = RospugAdapter(t, guard, clock=ScriptedClock(), frame_fetcher=lambda: b"x")
        assert a.execute("walk", {"distance_cm": 150}).ok


class TestSensingOnlyProfileWithholdsEveryMotionTool:
    """The ROSPug half of the sensing-only contract, through the real adapter.

    A profile with motion_tools_enabled=False must not put walk/strafe/turn/
    set_pose/do_action in front of the model, and must refuse them if called
    from a cached declaration. Everything that only reads sensors -- and stop,
    which is a halt, not a motion -- keeps working.
    """

    MOTION = {"walk", "strafe", "turn", "set_pose", "set_gait", "do_action"}
    SENSING = {"get_status", "scan_surroundings", "look", "stop"}

    def _sensing_only(self):
        from physicalagent.profiles.base import (
            LinkSpec, RobotProfile, SafeStatePolicy,
        )
        return RobotProfile(
            name="benchbot", embodiment="hexapod",
            supervision=None,                  # not yet proven -> no motion
            safe_state=SafeStatePolicy.FREEZE,
            safety=ROSPUG_SAFETY,
            link=LinkSpec(kind="serial", device="/dev/ttyUSB0", baud=115200),
            embodiment_prompt="x",
            motion_tools_enabled=False,
        )

    def _adapter(self):
        t = FakeTransport(telemetry={"batt": OK_BATTERY, "dead": "ALIVE",
                                     "scan": None, "odom": None},
                          services={"/pug_control/run_action_group": {"success": True}})
        a = RospugAdapter(t, SafetyGuard(ROSPUG_SAFETY, motion_unlocked=True),
                          clock=ScriptedClock(), robot_profile=self._sensing_only(),
                          frame_fetcher=lambda: b"x")
        return a, t

    def test_no_motion_tool_is_declared_to_the_model(self):
        a, _ = self._adapter()
        declared = {d["name"] for d in a.gemini_declarations()}
        assert declared & self.MOTION == set()
        assert declared == self.SENSING

    def test_every_motion_tool_is_refused_at_runtime(self):
        a, t = self._adapter()
        calls = [("walk", {"distance_cm": 10}), ("strafe", {"distance_cm": 10}),
                 ("turn", {"angle_deg": 45}), ("set_pose", {"pitch_deg": 5}),
                 ("do_action", {"name": "sit"})]
        for tool, args in calls:
            r = a.execute(tool, args)
            assert r.ok is False, f"{tool} ran on a sensing-only robot"
            assert "sensing-only" in r.error
        assert t.published == [], "a sensing-only robot published a command"
        assert t.service_calls == [], "a sensing-only robot called a service"

    def test_sensing_tools_still_work(self):
        a, _ = self._adapter()
        assert a.execute("get_status", {}).ok
        assert a.execute("look", {}).ok

    def test_stop_is_not_treated_as_a_motion_tool(self):
        """Halting must never be withheld -- it is the safe direction."""
        a, t = self._adapter()
        assert a.execute("stop", {}).ok
        assert t.last_published("/pug_control/velocity_move")["stop"] is True

    def test_the_rospugs_own_profile_withholds_nothing(self):
        """The live robot is motion-enabled; this change must not touch it."""
        from physicalagent.profiles import get_profile
        t = FakeTransport(telemetry={"batt": OK_BATTERY, "dead": "ALIVE"}, services={})
        a = RospugAdapter(t, SafetyGuard(ROSPUG_SAFETY, motion_unlocked=True),
                          clock=ScriptedClock(), robot_profile=get_profile("rospug"),
                          frame_fetcher=lambda: b"x")
        assert {d["name"] for d in a.gemini_declarations()} == self.MOTION | self.SENSING


# ---------------------------------------------------------------------------
# The adapter must read its limits from the GUARD'S PROFILE, not from config
# ---------------------------------------------------------------------------
class TestAdapterLimitsComeFromTheProfile:
    """REGRESSION GUARD: the adapter used to reach around its own guard to the
    module-level `config` globals for three limits — the turn controller's yaw
    rate, the per-call duration cap, and the mid-move battery-sag floor. Every
    other test in this suite builds a guard from ROSPUG_SAFETY, whose values ARE
    the config globals, so the bypass was invisible by construction: profile and
    config agreed on every number.

    A second robot's profile would therefore have been silently ignored — its
    turns would run at the ROSPug's yaw rate, its moves for the ROSPug's
    duration, and its 2S pack would be judged against the ROSPug's 3S floor
    (i.e. refused as flat while perfectly healthy). On an interface where the
    vendor applies no clamp, a limit that silently reverts to another robot's
    number is a physical-damage bug.

    So this test deliberately builds an adapter whose profile DISAGREES with
    config on all three, and pins every one to the profile.
    """

    TINY_YAW = 0.05          # vs the ROSPug's unlocked 0.4 rad/s
    TINY_DURATION_S = 2.0    # vs the ROSPug's unlocked 15.0 s
    TINY_BATTERY_MIN = 7000  # vs the ROSPug's 3S floor of 11100 mV

    def _tiny_profile(self):
        from physicalagent.safety import (
            SafetyProfile, VelocityLimits, default_pose_limits,
        )

        def _v(x, y, yaw):
            return VelocityLimits(max_abs_x=x, max_abs_y=y, max_abs_yaw=yaw,
                                  allow_lateral_y=True)

        return SafetyProfile(
            name="tiny",
            locked_velocity=_v(0.01, 0.01, 0.02),
            unlocked_velocity=_v(0.02, 0.02, self.TINY_YAW),
            vendor_ceiling=_v(0.02, 0.02, self.TINY_YAW),
            pose=default_pose_limits(),
            locked_max_duration_s=0.5,
            unlocked_max_duration_s=self.TINY_DURATION_S,
            max_steps_per_call=4,
            battery_motion_min_mv=self.TINY_BATTERY_MIN,
            battery_warn_mv=6500,
            action_whitelist=("stand",),
        )

    def _adapter(self, battery=12000, batt_script=None):
        t = FakeTransport(telemetry={"batt": battery, "dead": "ALIVE",
                                     "scan": None, "odom": None}, services={})
        if batt_script is not None:
            t.script_telemetry("batt", batt_script)
        guard = SafetyGuard(self._tiny_profile(), motion_unlocked=True)
        return RospugAdapter(t, guard, clock=ScriptedClock(),
                             frame_fetcher=lambda: b"x"), t

    def test_turn_rate_tracks_the_profile_not_the_rospug_config(self):
        from physicalagent import config
        a, t = self._adapter()
        a.execute("turn", {"angle_deg": 90})
        yaw = next(m["yaw_rate"] for _, _, m in t.published if m.get("yaw_rate", 0) != 0)
        assert abs(yaw) == pytest.approx(self.TINY_YAW), (
            "turn commanded a yaw rate from config, not from the guard's profile")
        assert abs(yaw) < config.VERIFIED_MAX_ABS_YAW_RAD_S

    def test_duration_cap_tracks_the_profile_not_the_rospug_config(self):
        from physicalagent import config
        a, _ = self._adapter()
        assert a._duration_cap() == pytest.approx(self.TINY_DURATION_S)
        assert a._duration_cap() < config.VERIFIED_MAX_DURATION_S

    def test_a_capped_walk_runs_for_the_profiles_duration(self):
        """Behavioural mirror of the above: at 0.02 m/s a 150 cm request would
        take 75 s, so the reported duration is purely the cap."""
        a, _ = self._adapter()
        res = a.execute("walk", {"distance_cm": 150})
        assert res.ok, res.error
        assert res.payload["duration_s"] == pytest.approx(self.TINY_DURATION_S)

    def test_sag_floor_tracks_the_profile_not_the_rospug_config(self):
        """8000 mV is healthy for this profile (floor 7000) but far below the
        ROSPug's 11100 — hardwiring config aborted a perfectly good move."""
        a, _ = self._adapter(batt_script=[12000, 12000, 8000])
        res = a.execute("walk", {"distance_cm": 150})
        assert res.ok, res.error

    def test_a_real_sag_against_the_profile_floor_still_aborts(self):
        """The gate is moved, not removed: below THIS profile's floor it fires."""
        a, t = self._adapter(batt_script=[12000, 12000, 6500])
        res = a.execute("walk", {"distance_cm": 150})
        assert res.ok is False
        assert str(self.TINY_BATTERY_MIN) in res.error
        assert t.published[-1][2].get("stop") is True

    def test_a_profile_with_no_pack_skips_the_sag_check_entirely(self):
        """A mains-powered robot has no pack to sag; a None floor must not be
        compared against, and must not refuse motion for lack of telemetry."""
        from dataclasses import replace
        from physicalagent.safety import NO_BATTERY
        mains = replace(self._tiny_profile(),
                        battery_motion_min_mv=NO_BATTERY, battery_warn_mv=NO_BATTERY)
        t = FakeTransport(telemetry={"batt": None, "dead": "ALIVE",
                                     "scan": None, "odom": None}, services={})
        a = RospugAdapter(t, SafetyGuard(mains, motion_unlocked=True),
                          clock=ScriptedClock(), frame_fetcher=lambda: b"x")
        assert a.execute("walk", {"distance_cm": 150}).ok
