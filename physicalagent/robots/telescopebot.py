"""TelescopeBot Robot Adapter (Draft).

This is a scaffold for the mid-sized humanoid with a telescoping spine.
Hardware: Jetson + RealSense + Mix of Hobby/Bus Servos + Continuous Rotation Spine.
"""
from typing import Any, Callable, Mapping

from physicalagent.robots.base import RobotAdapter, ToolSpec

# Spine drive physics. The spine is a pair of M8 x 1.25 threaded rods driven
# by FS5103R continuous-rotation servos. An M8 rod advances its nut by the
# THREAD PITCH per revolution -- 1.25 mm -- not by 8 mm; "M8" names a
# diameter, not a lead. (A T8x8 printer lead screw IS 8 mm/rev; if the spine
# is ever upgraded to T8, change SPINE_LEAD_MM_PER_REV to 8.0 and nothing
# else.) At ~60 RPM under load that is ~1.25 mm/s of travel, so a full 25 cm
# extension takes ~200 s open loop.
SPINE_LEAD_MM_PER_REV = 1.25
SPINE_SERVO_REV_PER_S = 1.0  # FS5103R ~60 RPM under load -- characterize on hardware
SPINE_TRAVEL_CM_PER_S = SPINE_LEAD_MM_PER_REV * SPINE_SERVO_REV_PER_S / 10.0
SPINE_MAX_HEIGHT_CM = 25.0


class TelescopeBotAdapter(RobotAdapter):
    """Adapter for the Telescoping Humanoid."""

    def __init__(self, guard=None, robot_profile=None):
        super().__init__(guard, robot_profile)
        # Mock state for the scaffold
        self._spine_height_cm = 0.0
        self._battery_voltage = 12.0
    
    def capabilities(self) -> tuple[ToolSpec, ...]:
        return (
            ToolSpec(
                name="set_spine_height",
                description="Extend or retract the central twin-screw spine to reach higher or lower.",
                parameters={
                    "type": "object",
                    "properties": {
                        "height_cm": {
                            "type": "number",
                            "description": f"Height to extend the spine in cm (0 to {SPINE_MAX_HEIGHT_CM})"
                        }
                    },
                    "required": ["height_cm"],
                }
            ),
            ToolSpec(
                name="do_action",
                description="Execute a predefined kinematic action (e.g., walk, wave, reach).",
                parameters={
                    "type": "object",
                    "properties": {
                        "action_name": {
                            "type": "string",
                            "description": "Name of the action to perform."
                        }
                    },
                    "required": ["action_name"],
                }
            ),
            ToolSpec(
                name="look",
                description="Capture an image from the RealSense camera.",
                parameters={
                    "type": "object",
                    "properties": {},
                    "required": [],
                }
            ),
        )

    def _handlers(self) -> Mapping[str, Callable]:
        return {
            "set_spine_height": self._handle_set_spine_height,
            "do_action": self._handle_do_action,
            "look": self._handle_look,
        }

    def _motion_tool_names(self) -> set[str]:
        return {"set_spine_height", "do_action"}
        
    def _always_allowed_tools(self) -> set[str]:
        return {"stop"}

    def _handle_set_spine_height(self, height_cm: float) -> dict[str, Any]:
        """Convert a height request into a run duration for the twin FS5103R
        continuous-rotation servos.

        Both servos MUST be driven in the SAME rotational direction at the
        same speed: the parallel screws are also the chest box's
        anti-rotation guides, so any differential rotation racks the carriage
        until it binds. Open-loop timing on continuous servos WILL drift;
        the duration returned here is a plan, and homing against the bottom
        hard stop is the only re-zero.
        """
        target = max(0.0, min(SPINE_MAX_HEIGHT_CM, height_cm))
        travel_cm = target - self._spine_height_cm
        duration_s = abs(travel_cm) / SPINE_TRAVEL_CM_PER_S

        # Placeholder for actual hardware PWM/serial bus calls:
        #   set_servo_speed(LEFT_SPINE_SERVO, direction)
        #   set_servo_speed(RIGHT_SPINE_SERVO, direction)
        #   time.sleep(duration_s)
        #   set_servo_speed(LEFT_SPINE_SERVO, 0); set_servo_speed(RIGHT_SPINE_SERVO, 0)

        self._spine_height_cm = target
        return {
            "result": f"Twin-Spine set to {target:.1f} cm",
            "travel_cm": round(travel_cm, 2),
            "duration_s": round(duration_s, 1),
        }

    def _handle_do_action(self, action_name: str) -> dict[str, Any]:
        """Placeholder for walking/reaching kinematics."""
        return {"result": f"Action '{action_name}' executed"}

    def _handle_look(self) -> dict[str, Any]:
        """No-op for the logic handler; the system will call get_frame() directly."""
        return {"result": "Looked"}

    def get_frame(self) -> bytes:
        """Fetch RGB frame from RealSense."""
        # Return a dummy JPEG for the scaffold
        return b"\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x01\x00H\x00H\x00\x00\xff\xd9"

    def telemetry(self) -> dict:
        return {
            "battery_voltage": self._battery_voltage,
            "spine_height_cm": self._spine_height_cm,
            "state": "idle"
        }

    def stop_all(self) -> None:
        """E-stop all servos."""
        # E.g., send torque-off to all bus servos and stop the continuous spine servos
        pass
