"""Tests for physicalagent/robots/base.py — the RobotAdapter ABC + ToolSpec.

This is the function-calling boundary: Gemini emits typed tool calls, the
adapter translates them to robot commands. Like safety.py, base.py must be
PURE (no roslibpy, no network) so this whole file runs offline.

Design rules under test:
  * ToolSpec serialises to a Gemini FunctionDeclaration dict.
  * execute() NEVER raises — every failure path returns an error ToolResult,
    because an exception in the control loop kills the heartbeat thread and
    (correctly, but disruptively) pauses the robot via the deadman.
  * Unknown tools, bad args, handler crashes, and e-stop are all refusals.
  * stop_all() must remain callable even when e-stopped.
"""

import pytest

from physicalagent.robots.base import RobotAdapter, ToolResult, ToolSpec
from physicalagent.safety import ROSPUG_SAFETY, SafetyGuard


# ---------------------------------------------------------------------------
# ToolSpec
# ---------------------------------------------------------------------------

class TestToolSpec:
    def test_to_gemini_produces_function_declaration(self):
        spec = ToolSpec(
            name="walk",
            description="Walk forward/backward at a bounded speed.",
            parameters={
                "type": "object",
                "properties": {"speed": {"type": "number"}},
                "required": ["speed"],
            },
        )
        decl = spec.to_gemini()
        assert decl == {
            "name": "walk",
            "description": "Walk forward/backward at a bounded speed.",
            "parameters": {
                "type": "object",
                "properties": {"speed": {"type": "number"}},
                "required": ["speed"],
            },
        }

    def test_toolspec_is_immutable(self):
        spec = ToolSpec(name="x", description="d", parameters={})
        with pytest.raises(Exception):
            spec.name = "y"


# ---------------------------------------------------------------------------
# ToolResult
# ---------------------------------------------------------------------------

class TestToolResult:
    def test_success_carries_payload(self):
        r = ToolResult.success(distance_cm=28.9)
        assert r.ok is True
        assert r.payload == {"distance_cm": 28.9}
        assert r.error is None

    def test_failure_carries_message(self):
        r = ToolResult.failure("battery below floor")
        assert r.ok is False
        assert r.error == "battery below floor"
        assert r.payload == {}

    def test_to_gemini_response_success(self):
        r = ToolResult.success(state="paused")
        assert r.to_gemini() == {"ok": True, "state": "paused"}

    def test_to_gemini_response_failure(self):
        r = ToolResult.failure("unknown tool")
        assert r.to_gemini() == {"ok": False, "error": "unknown tool"}


# ---------------------------------------------------------------------------
# A minimal concrete adapter for exercising the base dispatch logic
# ---------------------------------------------------------------------------

class FakeAdapter(RobotAdapter):
    """Records calls; no hardware."""

    def __init__(self, guard=None):
        super().__init__(guard=guard)
        self.stopped = False
        self.walked_with = None

    def capabilities(self):
        return (
            ToolSpec(
                name="walk",
                description="walk",
                parameters={
                    "type": "object",
                    "properties": {"speed": {"type": "number"}},
                    "required": ["speed"],
                },
            ),
            ToolSpec(name="stop", description="stop", parameters={"type": "object", "properties": {}}),
            ToolSpec(name="boom", description="always crashes", parameters={"type": "object", "properties": {}}),
        )

    def _handlers(self):
        return {"walk": self._walk, "stop": self._stop, "boom": self._boom}

    def _walk(self, speed):
        self.walked_with = speed
        return {"commanded": speed}

    def _stop(self):
        self.stopped = True
        return {}

    def _boom(self):
        raise RuntimeError("servo exploded")

    def get_frame(self):
        return b"\xff\xd8fake-jpeg"

    def telemetry(self):
        return {"battery_mv": 11378}

    def stop_all(self):
        self.stopped = True


class TestRobotAdapterABC:
    def test_cannot_instantiate_abstract_base(self):
        with pytest.raises(TypeError):
            RobotAdapter()

    def test_gemini_declarations_lists_every_capability(self):
        a = FakeAdapter()
        decls = a.gemini_declarations()
        assert [d["name"] for d in decls] == ["walk", "stop", "boom"]
        assert all(set(d) == {"name", "description", "parameters"} for d in decls)


class TestExecuteDispatch:
    def test_execute_routes_to_handler_and_wraps_result(self):
        a = FakeAdapter()
        r = a.execute("walk", {"speed": 0.1})
        assert r.ok is True
        assert r.payload == {"commanded": 0.1}
        assert a.walked_with == 0.1

    def test_unknown_tool_is_refusal_not_exception(self):
        a = FakeAdapter()
        r = a.execute("fly", {})
        assert r.ok is False
        assert "fly" in r.error

    def test_missing_required_arg_refused_before_handler_runs(self):
        a = FakeAdapter()
        r = a.execute("walk", {})
        assert r.ok is False
        assert "speed" in r.error
        assert a.walked_with is None  # handler never invoked

    def test_unexpected_arg_refused_before_handler_runs(self):
        a = FakeAdapter()
        r = a.execute("walk", {"speed": 0.1, "warp_factor": 9})
        assert r.ok is False
        assert "warp_factor" in r.error
        assert a.walked_with is None

    def test_handler_exception_becomes_failure_result(self):
        a = FakeAdapter()
        r = a.execute("boom", {})
        assert r.ok is False
        assert "servo exploded" in r.error


class TestEstopIntegration:
    def test_estopped_guard_refuses_every_tool(self):
        guard = SafetyGuard(ROSPUG_SAFETY, motion_unlocked=False)
        guard.estop()
        a = FakeAdapter(guard=guard)
        r = a.execute("walk", {"speed": 0.05})
        assert r.ok is False
        assert "stop" in r.error.lower()  # mentions e-stop
        assert a.walked_with is None

    def test_stop_all_still_works_while_estopped(self):
        guard = SafetyGuard(ROSPUG_SAFETY, motion_unlocked=False)
        guard.estop()
        a = FakeAdapter(guard=guard)
        a.stop_all()
        assert a.stopped is True

    def test_guard_reset_restores_execution(self):
        guard = SafetyGuard(ROSPUG_SAFETY, motion_unlocked=False)
        guard.estop()
        a = FakeAdapter(guard=guard)
        guard.reset()
        r = a.execute("walk", {"speed": 0.05})
        assert r.ok is True

    def test_always_allowed_tool_runs_while_estopped(self):
        """A tool named in _always_allowed_tools() (e.g. 'stop') must execute
        even while e-stopped — halting is never something to refuse."""
        guard = SafetyGuard(ROSPUG_SAFETY, motion_unlocked=False)
        guard.estop()
        a = AllowStopAdapter(guard=guard)
        r = a.execute("stop", {})
        assert r.ok is True
        assert a.stopped is True


class MotionAwareAdapter(FakeAdapter):
    """Declares 'walk' as its one motion tool; 'stop' and 'boom' are not."""

    def _motion_tool_names(self):
        return {"walk"}


class _Profile:
    """Minimal stand-in for a RobotProfile -- only the flag is consulted."""

    def __init__(self, motion_tools_enabled):
        self.motion_tools_enabled = motion_tools_enabled


class TestMotionToolsEnabledIsActuallyEnforced:
    """REGRESSION GUARD: RobotProfile.motion_tools_enabled was validated at
    construction and then consulted by nothing. Nothing filtered the tool
    surface and nothing refused a motion call, so a profile declaring itself
    sensing-only -- the state a hexapod sits in precisely because its failure
    mode is walking until the pack is flat -- still handed the model every
    motion tool it had.

    Enforcement is deliberately doubled. Withholding the declaration is not
    sufficient on its own: the model keeps earlier turns in its conversation
    history and can call a tool whose declaration has since been withdrawn.
    """

    def test_motion_tools_are_absent_from_the_declared_surface(self):
        a = MotionAwareAdapter()
        a._robot_profile = _Profile(motion_tools_enabled=False)
        assert [d["name"] for d in a.gemini_declarations()] == ["stop", "boom"]

    def test_sensing_tools_are_still_declared(self):
        a = MotionAwareAdapter()
        a._robot_profile = _Profile(motion_tools_enabled=False)
        assert "stop" in [d["name"] for d in a.gemini_declarations()]

    def test_a_withheld_motion_tool_is_refused_if_called_anyway(self):
        """Defence in depth: the model may have it cached from an earlier turn."""
        a = MotionAwareAdapter()
        a._robot_profile = _Profile(motion_tools_enabled=False)
        r = a.execute("walk", {"speed": 0.05})
        assert r.ok is False
        assert "sensing-only" in r.error
        assert a.walked_with is None, "handler ran despite motion being disabled"

    def test_refusal_happens_even_with_invalid_args(self):
        """The refusal must not depend on passing schema validation first."""
        a = MotionAwareAdapter()
        a._robot_profile = _Profile(motion_tools_enabled=False)
        assert a.execute("walk", {}).ok is False
        assert a.walked_with is None

    def test_non_motion_tools_still_execute(self):
        a = MotionAwareAdapter()
        a._robot_profile = _Profile(motion_tools_enabled=False)
        assert a.execute("stop", {}).ok is True

    def test_an_enabled_profile_changes_nothing(self):
        a = MotionAwareAdapter()
        a._robot_profile = _Profile(motion_tools_enabled=True)
        assert [d["name"] for d in a.gemini_declarations()] == ["walk", "stop", "boom"]
        assert a.execute("walk", {"speed": 0.05}).ok is True

    def test_no_profile_means_motion_enabled(self):
        """Scripts and tests build adapters with no profile; the guard, battery,
        deadman and e-stop gates are all still in force for them."""
        a = MotionAwareAdapter()
        assert a.motion_tools_enabled is True
        assert a.execute("walk", {"speed": 0.05}).ok is True


class AllowStopAdapter(FakeAdapter):
    def _always_allowed_tools(self):
        return {"stop"}

    def capabilities(self):
        return super().capabilities() + (
            ToolSpec(name="stop", description="stop", parameters={"type": "object", "properties": {}}),
        )

    def _handlers(self):
        h = dict(super()._handlers())
        h["stop"] = self._do_stop
        return h

    def _do_stop(self):
        self.stopped = True
        return {}
