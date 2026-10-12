"""Closed-loop navigation: the control law, and every way it is allowed to stop.

The law is a pure function, so these drive it to convergence over thousands of
cycles against a simulated robot with no simulator running. What that CANNOT
show is whether the real robot tracks a commanded course -- only the live run
answers that, which is why scripts/verify_goto.py exists too.

The frame-offset tests are the ones that matter most. A navigator that assumes
the policy's heading frame equals odometry converges perfectly on a robot that
spawned at yaw 0 and spirals on one that did not, and the difference is
invisible offline unless a test deliberately introduces the offset.
"""
from __future__ import annotations

import math

import pytest

from physicalagent.robots import sonic_navigate as nav
from physicalagent.robots.sonic_link import ScriptedClock


class FakeTelemetry:
    """A robot that goes exactly where it is told, in ODOMETRY frame.

    `frame_offset` is the offset the robot really has: a course commanded in
    policy frame arrives here rotated by -offset. A Navigator configured with
    the matching offset cancels it; one configured with 0.0 does not, which is
    exactly the bug being guarded against.
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, *, frame_offset=0.0,
                 height=0.78, fresh=True, available=True, blocked=False):
        self.x, self.y, self.yaw = x, y, yaw
        self.frame_offset = frame_offset
        self.height = height
        self.fresh = fresh
        self.available = available
        self.blocked = blocked
        self.commands = []

    # -- link side ---------------------------------------------------------
    def set_course(self, move_heading, facing_heading, speed, duration_s):
        self.commands.append((move_heading, facing_heading, speed))
        true_heading = move_heading - self.frame_offset
        self.yaw = facing_heading - self.frame_offset
        if not self.blocked:
            self.x += speed * math.cos(true_heading) * nav.Navigator.CYCLE_S
            self.y += speed * math.sin(true_heading) * nav.Navigator.CYCLE_S
        return {}

    def stop(self):
        self.commands.append("stop")
        return {}

    # -- telemetry side ----------------------------------------------------
    def pose(self):
        if not self.available:
            return {"available": False, "note": "no rt/odostate"}
        return {"available": True, "fresh": self.fresh, "height_m": self.height,
                "position_m": {"x": self.x, "y": self.y, "z": self.height},
                "heading_deg": math.degrees(self.yaw)}


def navigator(bot, **kw):
    return nav.Navigator(bot, bot, ScriptedClock(), **kw)


class TestWrap:
    def test_wraps_into_pi_range(self):
        for a in (0.0, 3.0, -3.0, 7.0, -7.0, math.pi, 100.0):
            assert -math.pi - 1e-9 <= nav.wrap(a) <= math.pi + 1e-9

    def test_the_near_pi_case_that_makes_a_robot_spin_the_long_way(self):
        """Two headings 2 degrees apart either side of pi differ by ~2pi raw."""
        a, b = math.radians(179), math.radians(-179)
        assert abs(b - a) > 6.0
        assert abs(nav.wrap(b - a)) == pytest.approx(math.radians(2), abs=1e-6)


class TestControlLaw:
    def test_inside_tolerance_is_done(self):
        out = nav.step((0.0, 0.0, 0.0), nav.Goal(0.05, 0.05))
        assert out["done"] and out["reason"] == "arrived"

    def test_far_away_walks_toward_the_goal(self):
        out = nav.step((0.0, 0.0, 0.0), nav.Goal(5.0, 0.0))
        assert not out["done"]
        assert out["speed"] > 0
        assert out["move_heading"] == pytest.approx(0.0)

    def test_goal_behind_turns_in_place_rather_than_walking_backwards(self):
        out = nav.step((0.0, 0.0, 0.0), nav.Goal(-5.0, 0.0))
        assert out["speed"] == 0.0
        assert out["reason"] == "turning toward goal"
        assert abs(nav.wrap(out["facing_heading"] - math.pi)) < 1e-6

    def test_slows_down_on_approach(self):
        far = nav.step((0.0, 0.0, 0.0), nav.Goal(5.0, 0.0))["speed"]
        near = nav.step((0.0, 0.0, 0.0), nav.Goal(0.3, 0.0))["speed"]
        assert near < far

    def test_speed_never_exceeds_the_cap(self):
        for d in (0.2, 1.0, 5.0, 50.0):
            assert nav.step((0, 0, 0), nav.Goal(d, 0))["speed"] <= nav.MAX_SPEED_M_S

    def test_final_heading_is_pursued_once_in_position(self):
        out = nav.step((0.0, 0.0, 0.0), nav.Goal(0.0, 0.0, yaw=math.pi / 2))
        assert not out["done"]
        assert out["speed"] == 0.0
        assert out["facing_heading"] == pytest.approx(math.pi / 2)

    def test_in_position_and_facing_right_is_done(self):
        out = nav.step((0.0, 0.0, 0.1), nav.Goal(0.0, 0.0, yaw=0.0))
        assert out["done"]

    def test_frame_offset_rotates_what_goes_on_the_wire_only(self):
        plain = nav.step((0.0, 0.0, 0.0), nav.Goal(5.0, 0.0))
        offset = nav.step((0.0, 0.0, 0.0), nav.Goal(5.0, 0.0),
                          frame_offset=math.pi / 2)
        assert offset["move_heading"] == pytest.approx(plain["move_heading"]
                                                       + math.pi / 2)
        # the measured error is in odometry frame and must NOT be rotated
        assert offset["heading_error_rad"] == pytest.approx(
            plain["heading_error_rad"])


class TestConvergence:
    @pytest.mark.parametrize("gx,gy", [(2.0, 0.0), (0.0, 2.0), (-2.0, 0.0),
                                       (-1.5, -1.5), (3.0, 1.0), (0.4, -0.4)])
    def test_reaches_the_goal_from_any_direction(self, gx, gy):
        bot = FakeTelemetry()
        out = navigator(bot).go(nav.Goal(gx, gy), timeout_s=300)
        assert out["ok"], out
        assert out["final_error_m"] <= nav.POS_TOL_M

    def test_reaches_a_goal_pose_including_heading(self):
        bot = FakeTelemetry()
        out = navigator(bot).go(nav.Goal(1.5, 1.5, yaw=-math.pi / 2),
                                timeout_s=300)
        assert out["ok"], out
        assert abs(out["final_heading_error_deg"]) <= 12.0

    def test_converges_when_the_frame_offset_is_known(self):
        """The robot's frame is rotated 90 degrees and the navigator knows."""
        bot = FakeTelemetry(frame_offset=math.pi / 2)
        out = navigator(bot, frame_offset=math.pi / 2, calibrated=True).go(
            nav.Goal(2.0, 0.0), timeout_s=300)
        assert out["ok"], out

    def test_does_NOT_converge_when_the_frame_offset_is_wrong(self):
        """The failure this design exists to prevent. Left uncorrected the robot
        walks off at an angle; it must stop and SAY so, not wander until
        timeout with a cheerful result."""
        bot = FakeTelemetry(frame_offset=math.pi / 2)
        out = navigator(bot, frame_offset=0.0).go(nav.Goal(2.0, 0.0),
                                                  timeout_s=300)
        assert not out["ok"]
        assert "no progress" in out["reason"] or "timed out" in out["reason"]
        assert out["warning"].startswith("frame offset was assumed")


class TestAborts:
    def test_no_pose_refuses_rather_than_walking_blind(self):
        bot = FakeTelemetry(available=False)
        out = navigator(bot).go(nav.Goal(1.0, 0.0))
        assert not out["ok"] and "cannot navigate blind" in out["reason"]
        assert bot.commands == []

    def test_stale_pose_mid_run_stops_the_robot(self):
        bot = FakeTelemetry()
        n = navigator(bot)
        real_pose = bot.pose

        calls = {"n": 0}

        def flaky():
            calls["n"] += 1
            if calls["n"] > 3:
                bot.fresh = False
            return real_pose()

        bot.pose = flaky
        out = n.go(nav.Goal(3.0, 0.0), timeout_s=300)
        assert not out["ok"]
        assert "stale" in out["reason"]
        assert "stop" in bot.commands

    def test_a_fallen_robot_stops_the_run(self):
        bot = FakeTelemetry(height=0.24)
        out = navigator(bot).go(nav.Goal(2.0, 0.0), timeout_s=300)
        assert not out["ok"] and "went down" in out["reason"]
        assert "stop" in bot.commands

    def test_blocked_robot_reports_no_progress_instead_of_walking_forever(self):
        bot = FakeTelemetry(blocked=True)
        out = navigator(bot).go(nav.Goal(3.0, 0.0), timeout_s=300)
        assert not out["ok"] and "no progress" in out["reason"]

    def test_timeout_is_reported_as_a_timeout(self):
        bot = FakeTelemetry()
        out = navigator(bot).go(nav.Goal(7.0, 0.0), timeout_s=1.0)
        assert not out["ok"] and "timed out" in out["reason"]

    def test_absurd_range_is_refused_before_moving(self):
        bot = FakeTelemetry()
        out = navigator(bot).go(nav.Goal(50.0, 0.0))
        assert not out["ok"]
        assert "obstacle avoidance" in out["reason"]
        assert bot.commands == []

    def test_every_run_ends_stopped(self):
        for bot in (FakeTelemetry(), FakeTelemetry(blocked=True),
                    FakeTelemetry(height=0.2)):
            navigator(bot).go(nav.Goal(2.0, 0.0), timeout_s=50)
            assert bot.commands[-1] == "stop"


class TestResultHonesty:
    def test_result_reports_measured_start_and_end_not_the_goal(self):
        bot = FakeTelemetry(blocked=True)
        out = navigator(bot).go(nav.Goal(3.0, 0.0), timeout_s=300)
        assert out["end_m"] == {"x": 0.0, "y": 0.0}
        assert out["final_error_m"] == pytest.approx(3.0)
        assert out["ok"] is False

    def test_uncalibrated_runs_carry_a_warning(self):
        bot = FakeTelemetry()
        out = navigator(bot).go(nav.Goal(1.0, 0.0), timeout_s=300)
        assert out["frame_offset_measured"] is False
        assert "warning" in out

    def test_calibrated_runs_do_not(self):
        bot = FakeTelemetry()
        out = navigator(bot, calibrated=True).go(nav.Goal(1.0, 0.0),
                                                 timeout_s=300)
        assert out["frame_offset_measured"] is True
        assert "warning" not in out


class TestCalibration:
    @pytest.mark.parametrize("true_offset", [0.0, math.pi / 2, -math.pi / 4, 2.5])
    def test_recovers_the_offset_it_was_given(self, true_offset):
        bot = FakeTelemetry(frame_offset=true_offset)
        n = navigator(bot)
        out = n.calibrate()
        assert out["ok"], out
        assert nav.wrap(n.frame_offset - true_offset) == pytest.approx(0, abs=1e-6)
        assert n.calibrated

    def test_calibrating_then_navigating_converges(self):
        bot = FakeTelemetry(frame_offset=-1.1)
        n = navigator(bot)
        assert n.calibrate()["ok"]
        assert n.go(nav.Goal(2.0, 1.0), timeout_s=300)["ok"]

    def test_a_robot_that_did_not_move_is_not_a_calibration(self):
        """Direction is meaningless over zero displacement, and an offset
        derived from it would be noise dressed as a measurement."""
        bot = FakeTelemetry(blocked=True)
        n = navigator(bot)
        out = n.calibrate()
        assert not out["ok"]
        assert "barely moved" in out["reason"]
        assert not n.calibrated
