"""Unitree G1 adapter — DESIGN SKETCH. Not wired to anything, and not registered.

Read this before believing any of it. Every handler below returns a dict and
touches no robot: the tool SHAPE is a proposal for what the model should be
offered on a G1, nothing more. It is kept because that shape is a useful
starting point for the real design, not because any of it works.

Three things in here would be dangerous if this were reachable, and they are the
reason the profile is not in the registry:

  * `telemetry()` originally returned a hardcoded {battery 24000 mV, pitch 0.0,
    roll 0.0, state "standing"}. Fed to a fall detector, that is a sensor which
    reports "upright and healthy" while the robot is on its face. It now raises.
  * the `groot_*` tools name a policy stack that does not exist here — no
    checkpoint, no runtime, no interface. A tool the model can call and that
    silently returns success is worse than no tool.
  * a 35 kg biped's supervision story is not a dataclass field. Until there is
    one that means something, no adapter should offer it motion.
"""
from __future__ import annotations

from typing import Callable, Mapping

from physicalagent.robots.base import RobotAdapter, ToolResult, ToolSpec


class UnitreeG1Adapter(RobotAdapter):
    """Adapter for the Unitree G1 dynamic humanoid.
    
    This robot operates via DDS to its sport_request endpoints. We do not
    control leg joints natively; we send velocity commands and high-level
    actions (stand, sit). Arm joints are controlled via IK.
    """

    def capabilities(self) -> tuple[ToolSpec, ...]:
        return (
            ToolSpec(
                name="walk_velocity",
                description="Move the robot base with a specific velocity.",
                parameters={
                    "type": "object",
                    "properties": {
                        "vx": {"type": "number", "description": "Forward velocity m/s"},
                        "vy": {"type": "number", "description": "Lateral velocity m/s"},
                        "yaw_rate": {"type": "number", "description": "Turn rate rad/s"},
                    },
                    "required": ["vx", "vy", "yaw_rate"],
                },
            ),
            ToolSpec(
                name="sit_down",
                description="Command the robot to sit down and settle.",
                parameters={"type": "object"},
            ),
            ToolSpec(
                name="move_arm_ik",
                description="Move a specific arm to a Cartesian coordinate.",
                parameters={
                    "type": "object",
                    "properties": {
                        "arm": {"type": "string", "enum": ["left", "right"]},
                        "x": {"type": "number"},
                        "y": {"type": "number"},
                        "z": {"type": "number"},
                    },
                    "required": ["arm", "x", "y", "z"],
                },
            ),
            ToolSpec(
                name="groot_grasp",
                description="Invoke the Isaac GR00T grasping policy to pick up an object at a 3D coordinate.",
                parameters={
                    "type": "object",
                    "properties": {
                        "x": {"type": "number"},
                        "y": {"type": "number"},
                        "z": {"type": "number"},
                        "object_type": {"type": "string"},
                    },
                    "required": ["x", "y", "z", "object_type"],
                },
            ),
            ToolSpec(
                name="groot_place",
                description="Invoke the Isaac GR00T placement policy to put down an object at a 3D coordinate.",
                parameters={
                    "type": "object",
                    "properties": {
                        "x": {"type": "number"},
                        "y": {"type": "number"},
                        "z": {"type": "number"},
                    },
                    "required": ["x", "y", "z"],
                },
            ),
            ToolSpec(
                name="groot_protomotion",
                description="Playback or retarget a specific GR00T protomotion primitive.",
                parameters={
                    "type": "object",
                    "properties": {
                        "motion_name": {"type": "string", "enum": ["wave", "handover", "point"]},
                    },
                    "required": ["motion_name"],
                },
            ),
        )

    def _handlers(self) -> Mapping[str, Callable]:
        return {
            "walk_velocity": self.cmd_walk_velocity,
            "sit_down": self.cmd_sit_down,
            "move_arm_ik": self.cmd_move_arm_ik,
            "groot_grasp": self.cmd_groot_grasp,
            "groot_place": self.cmd_groot_place,
            "groot_protomotion": self.cmd_groot_protomotion,
            "stop": self.stop_all,
        }

    def get_frame(self) -> bytes:
        # Stub for returning the Realsense RGB-D frame
        return b""

    def telemetry(self) -> dict:
        # NEVER return placeholder telemetry from a robot adapter. This used to
        # answer {battery 24000 mV, pitch 0.0, roll 0.0, "standing"} whatever the
        # robot was actually doing — a fall detector reading that would conclude
        # the robot is level and healthy while it is lying on the floor. A sensor
        # that cannot read must say so.
        raise NotImplementedError(
            "G1 telemetry is not implemented — no DDS link exists yet. "
            "Placeholder telemetry is deliberately not returned: it would feed "
            "the stability check permanently-safe values.")

    def stop_all(self) -> dict:
        """Issue a safe stop. Must be allowed during e-stop."""
        # For G1, we would send a zero velocity command and transition to damping
        return {"status": "stopped"}

    def _always_allowed_tools(self) -> set[str]:
        return {"stop"}

    def _motion_tool_names(self) -> set[str]:
        return {"walk_velocity", "sit_down", "move_arm_ik", "groot_grasp", "groot_place", "groot_protomotion"}

    def cmd_walk_velocity(self, vx: float, vy: float, yaw_rate: float) -> dict:
        return {"action": "walking", "vx": vx, "vy": vy, "yaw_rate": yaw_rate}

    def cmd_sit_down(self) -> dict:
        return {"action": "sitting"}

    def cmd_move_arm_ik(self, arm: str, x: float, y: float, z: float) -> dict:
        return {"action": "moving_arm", "arm": arm, "target": [x, y, z]}

    def cmd_groot_grasp(self, x: float, y: float, z: float, object_type: str) -> dict:
        return {"action": "groot_grasp", "target": [x, y, z], "object": object_type}

    def cmd_groot_place(self, x: float, y: float, z: float) -> dict:
        return {"action": "groot_place", "target": [x, y, z]}

    def cmd_groot_protomotion(self, motion_name: str) -> dict:
        return {"action": "groot_protomotion", "motion": motion_name}
