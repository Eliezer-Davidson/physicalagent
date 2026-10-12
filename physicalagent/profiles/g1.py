from __future__ import annotations

from physicalagent.profiles.base import (
    LinkSpec,
    RobotProfile,
    SafeStatePolicy,
    SupervisionClass,
)
from physicalagent.safety import (
    NO_BATTERY,
    PoseLimits,
    SafetyProfile,
    VelocityLimits,
)

_G1_SAFETY = SafetyProfile(
    name="g1",
    locked_velocity=VelocityLimits(max_abs_x=0.1, max_abs_yaw=0.1, allow_lateral_y=True, max_abs_y=0.1),
    unlocked_velocity=VelocityLimits(max_abs_x=0.5, max_abs_yaw=0.5, allow_lateral_y=True, max_abs_y=0.3),
    vendor_ceiling=VelocityLimits(max_abs_x=2.0, max_abs_yaw=2.0, allow_lateral_y=True, max_abs_y=1.0),
    pose=PoseLimits(
        max_abs_roll=0.5,
        max_abs_pitch=0.5,
        height_min=0.3,
        height_max=0.8,
        max_abs_x_shift=0.1,
        max_abs_stance=0.1,
    ),
    locked_max_duration_s=1.0,
    unlocked_max_duration_s=5.0,
    max_steps_per_call=10,
    battery_motion_min_mv=NO_BATTERY,  # We will get this via DDS later, mock for now
    battery_warn_mv=NO_BATTERY,
    action_whitelist=("stand_up", "sit_down", "damping"),
)

G1 = RobotProfile(
    name="g1",
    embodiment="humanoid",
    supervision=SupervisionClass.DYNAMIC_HUMANOID, # Needs to be added to SupervisionClass
    safe_state=SafeStatePolicy.SETTLE_TO_STANCE,
    safety=_G1_SAFETY,
    link=LinkSpec(kind="dds", host="192.168.123.161"),
    embodiment_prompt="I am a Unitree G1 dynamic humanoid robot. My leg balance is controlled automatically. I can walk by velocity and move my arms in 3D Cartesian coordinates.",
    motion_tools_enabled=True,
)
