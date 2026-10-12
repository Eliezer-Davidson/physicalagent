"""RobotAdapter ABC + ToolSpec — the function-calling boundary.

Gemini emits typed tool calls; an adapter translates them into robot commands.
This module is PURE (no roslibpy, no network, no clocks) so it is fully
testable offline — the same discipline as physicalagent/safety.py.

Why execute() never raises
    The control loop that calls execute() is the same process keeping the
    heartbeat alive. An uncaught exception kills the loop, the heartbeat
    stops, and the robot-side deadman pauses the robot. That is the correct
    *last resort*, but a malformed tool call from the model should be an
    ordinary refusal the model can read and correct — not a robot pause.
    So every failure path returns ToolResult.failure(...); exceptions stop
    at this boundary.

Validation before execution
    Tool args are validated against the ToolSpec's JSON-schema fragment
    (required keys present, no unexpected keys) BEFORE the handler runs.
    The model gets a precise error naming the offending parameter, and a
    half-valid call never reaches hardware.

E-stop
    If constructed with a SafetyGuard and the guard is e-stopped, every
    tool call is refused. stop_all() is deliberately NOT routed through
    execute() so it keeps working while e-stopped.

Motion tools
    A RobotProfile may declare motion_tools_enabled=False -- the spec's
    "sensing-only until proven" state, e.g. a hexapod whose failure mode is
    walking until the battery is flat. That flag is enforced in TWO places
    here, deliberately: motion tools are absent from gemini_declarations(),
    AND refused by execute() if called anyway. The second is not redundant.
    The model keeps prior turns in its conversation history, so it can call a
    tool whose declaration has since disappeared; filtering the surface alone
    would leave that path wide open.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Callable, Mapping


@dataclass(frozen=True)
class ToolSpec:
    """One tool the model may call. `parameters` is a JSON-schema fragment."""

    name: str
    description: str
    parameters: dict

    def to_gemini(self) -> dict:
        """Gemini FunctionDeclaration form."""
        return {
            "name": self.name,
            "description": self.description,
            "parameters": self.parameters,
        }


@dataclass(frozen=True)
class ToolResult:
    """Outcome of one tool call, in a shape the model can read back."""

    ok: bool
    payload: dict = field(default_factory=dict)
    error: str | None = None

    @classmethod
    def success(cls, **payload: Any) -> "ToolResult":
        return cls(ok=True, payload=payload)

    @classmethod
    def failure(cls, message: str) -> "ToolResult":
        return cls(ok=False, error=message)

    def to_gemini(self) -> dict:
        """FunctionResponse body."""
        if self.ok:
            return {"ok": True, **self.payload}
        return {"ok": False, "error": self.error}


class RobotAdapter(ABC):
    """Base class for all robot adapters (ROSPug, TonyPi, ...)."""

    def __init__(self, guard=None, robot_profile=None):
        self._guard = guard
        self._robot_profile = robot_profile

    # -- what subclasses must provide --------------------------------------
    @abstractmethod
    def capabilities(self) -> tuple[ToolSpec, ...]:
        """Every tool this robot exposes to the model."""

    @abstractmethod
    def _handlers(self) -> Mapping[str, Callable]:
        """Tool name -> callable. Handlers take the tool args as kwargs and
        return a dict payload (or raise; execute() converts to a refusal)."""

    @abstractmethod
    def get_frame(self) -> bytes:
        """Latest camera frame as JPEG bytes."""

    @abstractmethod
    def telemetry(self) -> dict:
        """Current robot state (battery, deadman, pose...)."""

    @abstractmethod
    def stop_all(self) -> None:
        """Physically halt the robot. Must work even while e-stopped."""

    # -- concrete behaviour shared by every adapter ------------------------
    def _always_allowed_tools(self) -> set[str]:
        """Tools that run even while e-stopped. Halting must never be refused;
        subclasses add 'stop' (and any other fail-safe read/halt tool) here."""
        return set()

    def _motion_tool_names(self) -> set[str]:
        """Tools that physically move the robot, and are therefore withheld
        when the profile says motion_tools_enabled=False. Subclasses override;
        the base names none, because the base declares none."""
        return set()

    @property
    def motion_tools_enabled(self) -> bool:
        """Whether this robot is allowed to expose motion tools at all.

        Sourced from the RobotProfile, which is where the fleet-level decision
        lives. An adapter built WITHOUT a profile (tests, and the scripts in
        scripts/) is treated as motion-enabled: the base class has no way to
        know otherwise, and the real gates -- guard, battery, deadman, e-stop
        -- are all still in force. Production wires the profile through in
        cli.py, so the sensing-only case is covered where it matters.
        """
        if self._robot_profile is None:
            return True
        return bool(getattr(self._robot_profile, "motion_tools_enabled", True))

    def _motion_tool_withheld(self, name: str) -> bool:
        return not self.motion_tools_enabled and name in self._motion_tool_names()

    def gemini_declarations(self) -> list[dict]:
        return [spec.to_gemini() for spec in self.capabilities()
                if not self._motion_tool_withheld(spec.name)]

    def execute(self, name: str, args: dict) -> ToolResult:
        """Run one tool call. Never raises."""
        # Checked BEFORE the e-stop branch and before handler lookup: a robot
        # that must not move must not move for any reason, including a tool
        # the model remembers from a conversation turn taken before the
        # declaration was withdrawn.
        if self._motion_tool_withheld(name):
            return ToolResult.failure(
                f"'{name}' is a motion tool and this robot is configured "
                f"sensing-only (motion_tools_enabled=False): refused. "
                f"Sensing tools remain available."
            )

        if (self._guard is not None and self._guard.is_estopped
                and name not in self._always_allowed_tools()):
            return ToolResult.failure(
                "e-stop is engaged: all tool calls are refused until reset"
            )

        handler = self._handlers().get(name)
        if handler is None:
            return ToolResult.failure(f"unknown tool '{name}'")

        spec = next((s for s in self.capabilities() if s.name == name), None)
        if spec is not None:
            err = _validate_args(spec.parameters, args)
            if err is not None:
                return ToolResult.failure(err)

        try:
            payload = handler(**args)
        except Exception as exc:  # boundary: model error, not robot pause
            return ToolResult.failure(f"{name} failed: {exc}")
        return ToolResult.success(**self._annotate(name, payload or {}))

    def _annotate(self, name: str, payload: dict) -> dict:
        """Hook to attach measured context to a tool result before it reaches
        the model. Default is a no-op.

        This exists because a tool result is the ONLY thing the model reliably
        reads. Anything it has to remember to ask for separately, it will
        eventually not ask for -- and a command that returns success while the
        robot lies on the floor is indistinguishable from one that worked.
        """
        return payload


def _validate_args(schema: dict, args: dict) -> str | None:
    """Check `args` against a JSON-schema fragment. Returns an error message
    naming the offending parameter, or None if valid."""
    required = schema.get("required", [])
    properties = schema.get("properties", {})

    for key in required:
        if key not in args:
            return f"missing required parameter '{key}'"
    for key in args:
        if key not in properties:
            return f"unexpected parameter '{key}'"
    return None
