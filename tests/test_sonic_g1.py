"""Tests for the SONIC-driven G1 adapter. All offline: no simulator, no DDS, no
network, no Gemini. FakeSonicLink integrates the commands so a test can assert
the robot actually went somewhere, not merely that a call was made.

These pin the things a demo can get wrong quietly: an unclamped velocity, a
latched walk that never stops, telemetry that invents a pose, and a motion tool
that still fires after an e-stop.
"""
from __future__ import annotations

import math

import pytest

from physicalagent.robots import sonic_arm_kin
from physicalagent.robots import sonic_link as link_mod
from physicalagent.robots import sonic_upper_body as upper_body
from physicalagent.robots.sonic_g1 import SAFE_POSES, SonicG1Adapter
from physicalagent.robots.sonic_link import (
    FakeSonicLink,
    ScriptedClock,
    clamp_duration,
    clamp_velocity,
)


class StubGuard:
    """Minimal stand-in: RobotAdapter only consults `is_estopped`."""

    def __init__(self, estopped=False):
        self.is_estopped = estopped


def build(estopped=False, view=None):
    link = FakeSonicLink()
    clock = ScriptedClock()
    return link, SonicG1Adapter(link, StubGuard(estopped), view=view, clock=clock)


# --- clamps -------------------------------------------------------------------
class TestClamps:
    def test_velocity_clamped_to_envelope(self):
        vx, vy, wz = clamp_velocity(99.0, -99.0, 99.0)
        assert vx == link_mod.MAX_VX_M_S
        assert vy == -link_mod.MAX_VY_M_S
        assert wz == link_mod.MAX_WZ_RAD_S

    def test_velocity_passes_through_inside_envelope(self):
        assert clamp_velocity(0.2, 0.1, 0.3) == (0.2, 0.1, 0.3)

    def test_duration_none_is_default_not_forever(self):
        assert clamp_duration(None) == link_mod.DEFAULT_DURATION_S

    def test_duration_capped(self):
        assert clamp_duration(10_000) == link_mod.MAX_DURATION_S

    def test_negative_duration_floors_to_zero(self):
        assert clamp_duration(-5) == 0.0


# --- motion -------------------------------------------------------------------
class TestMotion:
    def test_walk_moves_the_robot_and_reports_what_was_commanded(self):
        link, a = build()
        r = a.execute("walk", {"vx": 0.3, "duration_s": 2.0})
        assert r.ok, r.error
        assert r.payload["vx"] == 0.3
        assert link.position[0] == pytest.approx(0.6)

    def test_walk_is_clamped_and_the_result_says_so(self):
        link, a = build()
        r = a.execute("walk", {"vx": 50.0, "duration_s": 1.0})
        assert r.ok
        assert r.payload["vx"] == link_mod.MAX_VX_M_S

    def test_every_walk_ends_stopped(self):
        """No latched motion: the last thing on the wire is always a stop."""
        link, a = build()
        a.execute("walk", {"vx": 0.3})
        assert link.commands[-1]["kind"] == "stop"
        assert a.execute("walk", {"vx": 0.3}).payload["stopped"] is True

    def test_turn_changes_yaw_only(self):
        link, a = build()
        a.execute("turn", {"yaw_rate": 0.5, "duration_s": 2.0})
        assert link.yaw == pytest.approx(1.0)
        assert link.position == (pytest.approx(0.0), pytest.approx(0.0))

    def test_walk_duration_is_bounded(self):
        link, a = build()
        r = a.execute("walk", {"vx": 0.1, "duration_s": 9999})
        assert r.payload["duration_s"] == link_mod.MAX_DURATION_S


# --- poses --------------------------------------------------------------------
class TestPoses:
    def test_known_pose_selected(self):
        link, a = build()
        assert a.execute("set_pose", {"name": "sit"}).ok
        assert link.mode == "sit"

    def test_unknown_pose_refused_as_a_result_not_an_exception(self):
        _, a = build()
        r = a.execute("set_pose", {"name": "backflip"})
        assert not r.ok
        assert "backflip" in r.error

    def test_damp_is_offered_but_described_as_a_last_resort(self):
        spec = next(s for s in a_caps() if s.name == "set_pose")
        assert "damp" in SAFE_POSES
        assert "sinks to the ground" in spec.description


def a_caps():
    _, a = build()
    return a.capabilities()


# --- e-stop and refusals ------------------------------------------------------
class TestSafetyBoundary:
    def test_motion_refused_while_estopped(self):
        link, a = build(estopped=True)
        r = a.execute("walk", {"vx": 0.3})
        assert not r.ok
        assert "e-stop" in r.error
        assert link.commands == []

    def test_stop_and_reads_still_work_while_estopped(self):
        _, a = build(estopped=True)
        assert a.execute("stop", {}).ok
        assert a.execute("get_status", {}).ok

    def test_execute_never_raises_on_malformed_args(self):
        _, a = build()
        for name, args in [("walk", {"nope": 1}), ("turn", {}), ("set_pose", {}),
                           ("walk", {"vx": "fast"}), ("nosuchtool", {})]:
            r = a.execute(name, args)
            assert r.ok is False and r.error

    def test_look_without_a_camera_is_a_refusal_not_a_crash(self):
        _, a = build()
        r = a.execute("look", {})
        assert not r.ok and "camera" in r.error


# --- telemetry honesty --------------------------------------------------------
class TestTelemetryHonesty:
    def test_unreadable_state_says_so_rather_than_inventing_a_pose(self):
        class DeadLink(FakeSonicLink):
            def read_state(self):
                raise OSError("link down")

        a = SonicG1Adapter(DeadLink(), StubGuard(), clock=ScriptedClock())
        t = a.telemetry()
        assert t["state_available"] is False
        assert "link down" in t["note"]
        # the fields a fall detector would read must be absent, not defaulted
        assert "root_height_m" not in t or t.get("root_height_m") is None

    def test_command_path_is_reported_unverified_until_promoted(self):
        _, a = build()
        assert a.telemetry()["command_path_verified"] is False
        assert link_mod.VERIFIED_COMMAND_PATH is None


# --- tool surface -------------------------------------------------------------
class TestToolSurface:
    def test_NO_LEG_JOINT_can_ever_be_commanded(self):
        """The invariant that actually matters, stated directly.

        Upper-body joints ARE commandable -- they ride on the planner topic as
        `upper_body_position` while the legs stay under the balance controller,
        which is the entire reason it is safe. Leg joints are not, and must never
        be: writing one bypasses the only thing keeping the robot upright.

        This test used to assert that no motion tool had 'joint' in its name.
        That was a proxy, and the proxy broke the moment real joint control
        arrived -- correctly flagging that something had changed, but unable to
        say whether the change was safe. This checks the property itself.
        """
        from physicalagent.robots import sonic_upper_body as ub

        leg_joints = {"hip", "knee", "ankle"}
        for name in ub.JOINT_NAMES:
            assert not any(part in name for part in leg_joints), (
                f"{name} is a leg joint and must not be in the commandable set")
        assert len(ub.JOINT_NAMES) == ub.NUM_UPPER_BODY_JOINTS == 17

        # ...and the tool refuses a leg joint by name rather than ignoring it
        link, a = build()
        r = a.execute("set_joints", {"joints": {"left_knee": 0.5}})
        assert r.ok
        assert "left_knee" in r.payload["ignored_unknown_joints"]
        commanded = link.commands[-1]["values"]
        assert commanded == list(ub.NEUTRAL), "a leg joint changed the reference"

    def test_reads_stay_available_and_are_not_motion(self):
        _, a = build()
        assert "get_joints" in a._always_allowed_tools()
        assert "get_joints" not in a._motion_tool_names()

    def test_fingers_are_the_one_joint_level_command_and_are_not_body_joints(self):
        """set_grasp does write joint targets. That is deliberate and safe: the
        Inspire fingers are on a channel of their own and cannot fight the
        balance controller, which is why they are offered when the 29 body
        joints are not."""
        names = {s.name for s in a_caps()}
        assert "set_grasp" in names
        spec = next(s for s in a_caps() if s.name == "set_grasp")
        assert "get_contacts" in spec.description, (
            "the grasp tool must point at the only tool that can confirm a hold")

    def test_goto_is_offered_as_motion_and_warns_it_cannot_avoid_obstacles(self):
        names = {s.name for s in a_caps()}
        assert "goto" in names
        _, a = build()
        assert "goto" in a._motion_tool_names()
        spec = next(s for s in a_caps() if s.name == "goto")
        assert "DOES NOT AVOID OBSTACLES" in spec.description

    def test_goto_without_telemetry_refuses_instead_of_walking_open_loop(self):
        """Degrading to open-loop walking would report an arrival nothing
        measured — the exact failure goto exists to remove."""
        link, a = build()
        r = a.execute("goto", {"x": 2.0, "y": 0.0})
        assert not r.ok
        assert "no telemetry" in r.error
        assert link.commands == []

    def test_goto_refused_while_estopped(self):
        link, a = build(estopped=True)
        r = a.execute("goto", {"x": 1.0, "y": 0.0})
        assert not r.ok and "e-stop" in r.error
        assert link.commands == []

    def test_goto_arrives_and_reports_a_MEASURED_end_position(self):
        from test_sonic_navigate import FakeTelemetry

        bot = FakeTelemetry()
        a = SonicG1Adapter(bot, StubGuard(), clock=ScriptedClock(), telemetry=bot)
        r = a.execute("goto", {"x": 1.5, "y": 1.0, "timeout_s": 300})
        assert r.ok, r.error
        assert r.payload["ok"] is True
        assert r.payload["final_error_m"] <= 0.15
        # the end position is read back, not echoed from the goal
        assert r.payload["end_m"] != r.payload["goal_m"]

    def test_calibrate_heading_is_offered_and_is_motion(self):
        """It walks 0.8 m, so it must be gated like any other motion."""
        from test_sonic_navigate import FakeTelemetry

        _, a = build()
        assert "calibrate_heading" in {s.name for s in a_caps()}
        assert "calibrate_heading" in a._motion_tool_names()
        assert a.execute("calibrate_heading", {}).ok is False   # no telemetry

        estopped = SonicG1Adapter(FakeTelemetry(), StubGuard(estopped=True),
                                  clock=ScriptedClock(), telemetry=FakeTelemetry())
        assert "e-stop" in estopped.execute("calibrate_heading", {}).error

    def test_the_goto_warning_names_the_tool_that_fixes_it(self):
        """A warning telling the model to call something that does not exist is
        worse than no warning: it invents a tool name and the call fails."""
        from test_sonic_navigate import FakeTelemetry

        bot = FakeTelemetry()
        a = SonicG1Adapter(bot, StubGuard(), clock=ScriptedClock(), telemetry=bot)
        warning = a.execute("goto", {"x": 1.0, "y": 0.0, "timeout_s": 300}
                            ).payload["warning"]
        named = [s.name for s in a.capabilities() if s.name + "()" in warning]
        assert named == ["calibrate_heading"], warning

    def test_calibration_persists_across_goto_calls(self):
        """One measurement per policy start — the frame is fixed at init, so
        re-measuring per goto would walk 0.8 m of detour every time."""
        from test_sonic_navigate import FakeTelemetry

        bot = FakeTelemetry(frame_offset=0.7)
        a = SonicG1Adapter(bot, StubGuard(), clock=ScriptedClock(), telemetry=bot)
        assert a.execute("calibrate_heading", {}).ok
        r = a.execute("goto", {"x": 1.5, "y": 0.5, "timeout_s": 300})
        assert r.payload["ok"] is True
        assert r.payload["frame_offset_measured"] is True
        assert "warning" not in r.payload

    def test_where_are_my_hands_works_before_any_arm_command(self):
        """A model may well ask this first. _last_upper_body used to be created
        by the first arm command, so reading it before one raised
        AttributeError -- a crash, not a refusal."""
        _, a = build()
        r = a.execute("where_are_my_hands", {})
        assert r.ok, r.error
        assert "NOT measured" in r.payload["left"]["from"]
        for side in ("left", "right"):
            assert len(r.payload[side]["palm_m"]) == 3

    def test_where_are_my_hands_is_a_read_and_survives_an_estop(self):
        _, a = build(estopped=True)
        assert a.execute("where_are_my_hands", {}).ok

    def test_reach_to_commands_joints_that_put_the_hand_on_the_target(self):
        link, a = build()
        r = a.execute("reach_to", {"x": 0.35, "y": 0.25, "z": 0.30,
                                   "hand": "left"})
        assert r.ok, r.error
        assert r.payload["solution_error_m"] < 0.02
        assert r.payload["hand"] == "left"
        sent = link.commands[-1]["values"]
        # ...and the commanded pose really does solve to the target
        chain = [sent[upper_body.INDEX[l.joint]]
                 for l in sonic_arm_kin.LINKS["left"]]
        palm = sonic_arm_kin.fk(chain, "left")["palm_pos"]
        assert math.dist(palm, (0.35, 0.25, 0.30)) < 0.02

    def test_reach_to_leaves_the_other_arm_and_the_legs_alone(self):
        link, a = build()
        a.execute("reach_to", {"x": 0.35, "y": 0.25, "z": 0.3, "hand": "left"})
        sent = link.commands[-1]["values"]
        for name in upper_body.JOINT_NAMES:
            if name.startswith("right_"):
                assert sent[upper_body.INDEX[name]] == pytest.approx(
                    upper_body.NEUTRAL[upper_body.INDEX[name]])
        assert len(sent) == 17          # legs are not in this vector at all

    def test_unreachable_target_refuses_and_commands_NOTHING(self):
        """Stretching at an impossible point and calling it done is the failure
        mode; so is moving the arm and then reporting failure."""
        link, a = build()
        r = a.execute("reach_to", {"x": 3.0, "y": 0.0, "z": 0.0})
        assert r.ok                      # a refusal is a result, not an error
        assert r.payload["ok"] is False
        assert r.payload["nothing_was_commanded"] is True
        assert "closest" in r.payload["reason"] or "cannot reach" in r.payload["reason"]
        assert link.commands == []

    def test_either_hand_picks_the_one_that_reaches(self):
        _, a = build()
        r = a.execute("reach_to", {"x": 0.3, "y": -0.3, "z": 0.1,
                                   "hand": "either"})
        assert r.ok and r.payload["hand"] == "right", r.payload

    def test_reach_to_refused_while_estopped(self):
        link, a = build(estopped=True)
        r = a.execute("reach_to", {"x": 0.3, "y": 0.2, "z": 0.1})
        assert not r.ok and "e-stop" in r.error
        assert link.commands == []

    def test_bad_hand_name_is_a_result_not_a_crash(self):
        _, a = build()
        r = a.execute("reach_to", {"x": 0.3, "y": 0.2, "z": 0.1, "hand": "foot"})
        assert not r.ok and "foot" in r.error

    def test_reach_to_declares_the_frame_and_says_it_differs_from_goto(self):
        """The single most likely way for a model to get this wrong is to pass
        goto's world coordinates to reach_to."""
        spec = next(s for s in a_caps() if s.name == "reach_to")
        assert "PELVIS" in spec.description
        assert "NOT the frame goto() uses" in spec.description

    def test_motion_tools_withheld_when_profile_says_sensing_only(self):
        class SensingOnly:
            motion_tools_enabled = False

        link = FakeSonicLink()
        a = SonicG1Adapter(link, StubGuard(), clock=ScriptedClock(),
                           robot_profile=SensingOnly())
        declared = {d["name"] for d in a.gemini_declarations()}
        assert "walk" not in declared
        assert a.execute("walk", {"vx": 0.3}).ok is False
        assert link.commands == []


class TestPostureIsAttachedToEveryMotionResult:
    """A command that returns success while the robot lies on the floor is
    indistinguishable from one that worked.

    Observed 2026-08-01: the model believed it had stood the robot up while the
    pelvis was at 0.09 m with the torso on the ground. get_status() had the
    answer the whole time -- but it is a tool the model must choose to call, and
    believing everything is fine is exactly the state in which nothing prompts
    you to check.
    """

    class Pose:
        def __init__(self, height, tilt):
            self._h, self._t = height, tilt

        def pose(self):
            return {"available": True, "height_m": self._h, "tilt_deg": self._t}

        def imu(self):
            return {"available": False}

        def controller(self):
            return {"available": False}

        def joints(self):
            return {"available": False}

    def build(self, height, tilt):
        return SonicG1Adapter(FakeSonicLink(), StubGuard(), clock=ScriptedClock(),
                              telemetry=self.Pose(height, tilt))

    def test_a_fallen_robot_says_so_on_a_motion_result(self):
        out = self.build(0.09, 84.0).execute("set_mode", {"name": "idle"})
        assert out.ok
        body = out.payload["body"]
        assert body["standing"] is False
        assert "NOT STANDING" in body["note"]

    def test_it_says_ordinary_modes_cannot_get_up_and_names_the_one_that_tries(self):
        """The single most important sentence: retrying ORDINARY modes cannot
        work. There is now exactly one thing that tries — recover_from_fall —
        and the note must name it without promising it succeeds, or the model
        goes back to retrying walk at a robot on the floor."""
        body = self.build(0.09, 84.0).execute("walk", {"vx": 0.2}).payload["body"]
        assert "will NOT stand it back up" in body["note"]
        assert "recover_from_fall" in body["note"]
        assert "operator" in body["note"]

    def test_a_standing_robot_is_not_alarmed_about(self):
        body = self.build(0.78, 2.0).execute("set_mode", {"name": "idle"}).payload["body"]
        assert body["standing"] is True and body["note"] == "standing"

    def test_a_low_pelvis_alone_is_not_a_fall(self):
        """A crouch gait reaches 0.533 m while walking perfectly. Height without
        tilt is the bound that has already been wrong five times."""
        body = self.build(0.53, 4.0).execute("walk", {"vx": 0.2}).payload["body"]
        assert body["standing"] is False
        assert "NOT STANDING" in body["note"]
        assert body["tilt_deg"] == 4.0     # the model can see it is upright

    def test_sensing_tools_are_not_annotated(self):
        """get_status already reports pose; doubling it wastes context."""
        out = self.build(0.78, 2.0).execute("get_joints", {})
        assert "body" not in out.payload

    def test_missing_telemetry_does_not_invent_a_posture(self):
        adapter = SonicG1Adapter(FakeSonicLink(), StubGuard(), clock=ScriptedClock(),
                                 telemetry=None)
        assert "body" not in adapter.execute("set_mode", {"name": "idle"}).payload
