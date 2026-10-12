"""Tests for the closed-loop turn controller (physicalagent/robots/turn_control.py).

The robot over-turns ~122% open-loop and IMU yaw is unusable, so turns close
the loop on rf2o /odom_raw. This is the interesting pure logic: command yaw,
poll odometry, stop at target (within tolerance) or on timeout, report the
achieved angle. Fully offline via FakeTransport (scripted odom) + ScriptedClock.
"""
import math

import pytest

from physicalagent.robots.transport import FakeTransport, ScriptedClock
from physicalagent.robots.turn_control import TurnController, TurnResult


def _yaw_msg(deg):
    """A nav_msgs/Odometry-shaped dict carrying `deg` of yaw about Z."""
    half = math.radians(deg) / 2.0
    return {"pose": {"pose": {"orientation": {"w": math.cos(half), "x": 0.0,
                                              "y": 0.0, "z": math.sin(half)}}}}


class TestTurnControllerReachesTarget:
    def test_stops_when_target_reached(self):
        t = FakeTransport()
        # odom yaw climbs 0 -> 30 -> 60 -> 90 across polls
        t.script_telemetry("odom", [_yaw_msg(d) for d in (0, 30, 60, 90)])
        ctrl = TurnController(t, ScriptedClock(),
                              vel_topic="/pug_control/velocity_move",
                              vel_type="pug_control/Velocity",
                              odom_key="odom", yaw_rate=0.35, tol_deg=5.0, timeout_s=20.0)
        res = ctrl.turn(90.0)
        assert res.reached is True
        assert abs(res.achieved_deg - 90.0) <= 5.0

    def test_commands_positive_yaw_for_left_turn(self):
        t = FakeTransport()
        t.script_telemetry("odom", [_yaw_msg(d) for d in (0, 45, 90)])
        ctrl = TurnController(t, ScriptedClock(), vel_topic="/v", vel_type="V",
                              odom_key="odom", yaw_rate=0.35, tol_deg=5.0, timeout_s=20.0)
        ctrl.turn(90.0)
        # first motion command must be positive yaw_rate (left)
        first_motion = next(m for _, _, m in t.published if m.get("yaw_rate", 0) != 0)
        assert first_motion["yaw_rate"] > 0

    def test_commands_negative_yaw_for_right_turn(self):
        t = FakeTransport()
        t.script_telemetry("odom", [_yaw_msg(d) for d in (0, -45, -90)])
        ctrl = TurnController(t, ScriptedClock(), vel_topic="/v", vel_type="V",
                              odom_key="odom", yaw_rate=0.35, tol_deg=5.0, timeout_s=20.0)
        res = ctrl.turn(-90.0)
        first_motion = next(m for _, _, m in t.published if m.get("yaw_rate", 0) != 0)
        assert first_motion["yaw_rate"] < 0
        assert res.reached is True


class TestTurnControllerStopsCleanly:
    def test_final_command_is_a_stop(self):
        t = FakeTransport()
        t.script_telemetry("odom", [_yaw_msg(d) for d in (0, 45, 90)])
        ctrl = TurnController(t, ScriptedClock(), vel_topic="/v", vel_type="V",
                              odom_key="odom", yaw_rate=0.35, tol_deg=5.0, timeout_s=20.0)
        ctrl.turn(90.0)
        last = t.published[-1][2]
        assert last.get("stop") is True
        assert last.get("yaw_rate", 0) == 0

    def test_zero_angle_makes_no_motion(self):
        t = FakeTransport()
        t.script_telemetry("odom", [_yaw_msg(0)])
        ctrl = TurnController(t, ScriptedClock(), vel_topic="/v", vel_type="V",
                              odom_key="odom", yaw_rate=0.35, tol_deg=5.0, timeout_s=20.0)
        res = ctrl.turn(0.0)
        assert res.achieved_deg == 0.0
        # no non-zero yaw was ever commanded
        assert all(m.get("yaw_rate", 0) == 0 for _, _, m in t.published)


class TestTurnControllerTimeout:
    def test_times_out_if_target_never_reached_despite_progress(self):
        # odom advances continuously (so it is NOT a stall) but too slowly to
        # reach 90 deg before the hard timeout.
        t = FakeTransport()
        t.script_telemetry("odom", [_yaw_msg(i) for i in range(0, 60)])  # +1 deg/poll
        clock = ScriptedClock()
        ctrl = TurnController(t, clock, vel_topic="/v", vel_type="V",
                              odom_key="odom", yaw_rate=0.35, tol_deg=5.0,
                              timeout_s=2.0, stall_s=1.0, poll_dt=0.1)
        res = ctrl.turn(90.0)
        assert res.reached is False
        assert res.timed_out is True
        assert clock.now() >= 2.0
        assert t.published[-1][2].get("stop") is True

    def test_reports_partial_progress_on_timeout(self):
        t = FakeTransport()
        t.script_telemetry("odom", [_yaw_msg(i * 2) for i in range(0, 60)])  # +2 deg/poll
        ctrl = TurnController(t, ScriptedClock(), vel_topic="/v", vel_type="V",
                              odom_key="odom", yaw_rate=0.35, tol_deg=5.0,
                              timeout_s=2.0, stall_s=1.0, poll_dt=0.1)
        res = ctrl.turn(180.0)
        assert res.reached is False
        assert res.achieved_deg > 10  # got partway before the timeout


class TestTurnControllerStallDetection:
    def test_stops_when_odometry_dies_mid_turn(self):
        t = FakeTransport()
        # valid at entry, then None forever (rf2o froze/crashed after the turn began)
        t.script_telemetry("odom", [_yaw_msg(0), _yaw_msg(8), None])
        clock = ScriptedClock()
        ctrl = TurnController(t, clock, vel_topic="/v", vel_type="V",
                              odom_key="odom", yaw_rate=0.35, tol_deg=4.0,
                              timeout_s=12.0, stall_s=1.0)
        res = ctrl.turn(90.0)
        assert res.reached is False
        assert res.error and "stall" in res.error.lower()
        # must stop WELL before the 12 s hard timeout — no 240-degree blind spin
        assert clock.now() < 4.0
        assert t.published[-1][2].get("stop") is True

    def test_frozen_nonzero_odometry_also_detected_as_stall(self):
        t = FakeTransport()
        # odom holds 8 deg forever — robot commanded to turn but not rotating
        t.script_telemetry("odom", [_yaw_msg(8)])
        clock = ScriptedClock()
        ctrl = TurnController(t, clock, vel_topic="/v", vel_type="V",
                              odom_key="odom", yaw_rate=0.35, tol_deg=4.0,
                              timeout_s=12.0, stall_s=1.0)
        res = ctrl.turn(90.0)
        assert res.reached is False
        assert clock.now() < 4.0  # stall stop, not the full timeout


class TestTurnControllerOdometryUnavailable:
    def test_refuses_when_no_odometry(self):
        t = FakeTransport()  # no odom scripted -> latest returns None
        ctrl = TurnController(t, ScriptedClock(), vel_topic="/v", vel_type="V",
                              odom_key="odom", yaw_rate=0.35, tol_deg=5.0, timeout_s=6.0)
        res = ctrl.turn(90.0)
        assert res.reached is False
        assert res.error is not None
        assert "odom" in res.error.lower()
        # must NOT have commanded blind motion
        assert all(m.get("yaw_rate", 0) == 0 for _, _, m in t.published)


class TestTurnResultWrapping:
    def test_wraps_across_180(self):
        """Yaw crossing +/-180 must not read as a huge negative jump."""
        t = FakeTransport()
        t.script_telemetry("odom", [_yaw_msg(d) for d in (170, -175, -160)])
        ctrl = TurnController(t, ScriptedClock(), vel_topic="/v", vel_type="V",
                              odom_key="odom", yaw_rate=0.35, tol_deg=5.0, timeout_s=20.0)
        res = ctrl.turn(20.0)  # 170 -> -175 is +15, -> -160 is +30 total
        assert res.reached is True
        assert 15 <= res.achieved_deg <= 25


class TestTurnTimeoutScalesWithAngle:
    """A failed turn must not spin for the WALK duration cap.

    turn's timeout was max_duration_s + 4. Raising max_duration_s 8 -> 20 for
    longer walks silently raised the turn timeout 12 s -> 24 s, which at
    0.4 rad/s is ~550 deg of blind rotation on a stuck turn. Observed live as
    'it spun and spun'. The budget must come from the ANGLE, not from how far
    the robot is allowed to walk.
    """

    def test_small_turn_gets_a_small_time_budget(self):
        t = FakeTransport()
        # creeps at 0.05 deg/poll: never reaches 30 deg, so the TIMEOUT decides
        t.script_telemetry("odom", [_yaw_msg(i * 0.05) for i in range(4000)])
        clock = ScriptedClock()
        ctrl = TurnController(t, clock, vel_topic="/v", vel_type="V", odom_key="odom",
                              yaw_rate=0.4, tol_deg=4.0, timeout_s=24.0, stall_s=99.0)
        ctrl.turn(30.0)
        # 30 deg at 0.4 rad/s (~23 deg/s) needs ~1.3 s; a generous budget is still
        # far below the 24 s walk-derived cap
        assert clock.now() < 10.0

    def test_large_turn_still_gets_enough_time(self):
        t = FakeTransport()
        t.script_telemetry("odom", [_yaw_msg(d) for d in (0, 60, 120, 180)])
        ctrl = TurnController(t, ScriptedClock(), vel_topic="/v", vel_type="V",
                              odom_key="odom", yaw_rate=0.4, tol_deg=5.0, timeout_s=24.0)
        assert ctrl.turn(180.0).reached is True

    def test_never_exceeds_the_absolute_cap(self):
        t = FakeTransport()
        t.script_telemetry("odom", [_yaw_msg(0.1 * i) for i in range(2000)])
        clock = ScriptedClock()
        ctrl = TurnController(t, clock, vel_topic="/v", vel_type="V", odom_key="odom",
                              yaw_rate=0.4, tol_deg=4.0, timeout_s=12.0, stall_s=99.0)
        ctrl.turn(180.0)
        assert clock.now() <= 12.5


class TestTurnAbortsOnWrongDirection:
    """Observed live: commanded +45, odometry reported -27.8 and it kept driving.

    If the measured rotation is heading AWAY from the target, something is wrong
    (odometry lost tracking, or the robot is being pushed). Continuing to drive
    on that reading is how a turn becomes an uncontrolled spin.
    """

    def test_stops_when_rotation_goes_the_wrong_way(self):
        t = FakeTransport()
        # commanded left (+), but odom reports increasingly negative yaw
        t.script_telemetry("odom", [_yaw_msg(d) for d in (0, -8, -16, -24, -32)])
        clock = ScriptedClock()
        ctrl = TurnController(t, clock, vel_topic="/v", vel_type="V", odom_key="odom",
                              yaw_rate=0.4, tol_deg=4.0, timeout_s=24.0, stall_s=99.0)
        res = ctrl.turn(45.0)
        assert res.reached is False
        assert res.error and "direction" in res.error.lower()
        assert t.published[-1][2].get("stop") is True
        assert clock.now() < 6.0        # caught early, not at the timeout

    def test_small_wrong_way_jitter_is_tolerated(self):
        """Odometry noise must not abort a legitimate turn."""
        t = FakeTransport()
        t.script_telemetry("odom", [_yaw_msg(d) for d in (0, -1.5, 10, 25, 45)])
        ctrl = TurnController(t, ScriptedClock(), vel_topic="/v", vel_type="V",
                              odom_key="odom", yaw_rate=0.4, tol_deg=5.0, timeout_s=20.0)
        assert ctrl.turn(45.0).reached is True


class TestOpenLoopTurn:
    """Odometry under-reports rotation, so closing the loop on it made the robot
    over-rotate: it 'spun and spun', convinced it had not turned when it had.
    Operator-confirmed on hardware.

    Open-loop drives a calibrated duration instead (the robot over-turns ~122%
    of commanded yaw, measured), and uses odometry ONLY as a gross-overshoot
    monitor — safe in that role precisely because it under-reports: if it still
    claims a huge rotation, the rotation is certainly real.
    """

    def _ctrl(self, t, clock, **kw):
        return TurnController(t, clock, vel_topic="/v", vel_type="V", odom_key="odom",
                              yaw_rate=0.4, tol_deg=4.0, timeout_s=24.0,
                              open_loop=True, over_turn=1.22, **kw)

    def test_drives_a_calibrated_duration_not_until_odom_agrees(self):
        t = FakeTransport()
        t.script_telemetry("odom", [_yaw_msg(0)])       # odom frozen: irrelevant now
        clock = ScriptedClock()
        res = self._ctrl(t, clock).turn(90.0)
        assert res.reached is True                       # no longer hostage to the sensor
        # 90 deg at 0.4 rad/s (22.9 deg/s), corrected for 122% over-turn
        expected = (90.0 / 22.918) / 1.22
        assert clock.now() == pytest.approx(expected, rel=0.35)

    def test_frozen_odometry_no_longer_blocks_a_turn(self):
        """The exact live failure: 'odometry stalled mid-turn' refusing to turn."""
        t = FakeTransport()
        t.script_telemetry("odom", [_yaw_msg(5)])        # never changes
        res = self._ctrl(t, ScriptedClock()).turn(60.0)
        assert res.reached is True
        assert res.error is None

    def test_turns_without_any_odometry_at_all(self):
        t = FakeTransport()                              # no odom whatsoever
        res = self._ctrl(t, ScriptedClock()).turn(45.0)
        assert res.reached is True

    def test_direction_sign_is_preserved(self):
        for angle, sign in ((90.0, 1), (-90.0, -1)):
            t = FakeTransport()
            self._ctrl(t, ScriptedClock()).turn(angle)
            first = next(m for _, _, m in t.published if m.get("yaw_rate", 0) != 0)
            assert (first["yaw_rate"] > 0) == (sign > 0)

    def test_always_stops_at_the_end(self):
        t = FakeTransport()
        self._ctrl(t, ScriptedClock()).turn(90.0)
        assert t.published[-1][2].get("stop") is True

    def test_gross_overshoot_seen_by_odometry_stops_early(self):
        """Odometry under-reports, so a LARGE reading means real over-rotation."""
        t = FakeTransport()
        t.script_telemetry("odom", [_yaw_msg(d) for d in (0, 40, 80, 120, 160, 200)])
        clock = ScriptedClock()
        res = self._ctrl(t, clock).turn(45.0)
        # odom is sampled in 40 deg steps here, so it stops at the first sample
        # past the 90 deg cutoff — the point is it stopped instead of running on
        assert res.error is not None and "past" in res.error
        assert abs(res.achieved_deg) <= 130
        assert t.published[-1][2].get("stop") is True

    def test_reports_the_odometry_reading_as_a_cross_check(self):
        t = FakeTransport()
        t.script_telemetry("odom", [_yaw_msg(d) for d in (0, 20, 40)])
        res = self._ctrl(t, ScriptedClock()).turn(90.0)
        assert res.odom_observed_deg is not None
