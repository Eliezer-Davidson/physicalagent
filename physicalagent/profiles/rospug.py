"""The ROSPug — profile #1, and the reference for every profile after it."""
from __future__ import annotations

from physicalagent import config
from physicalagent.profiles.base import (
    LinkSpec, RobotProfile, SafeStatePolicy, SupervisionClass,
)
from physicalagent.safety import ROSPUG_SAFETY

EMBODIMENT_PROMPT = (
    "You are the reasoning brain of a small four-legged robot (a HiWonder "
    "ROSPug) on a carpeted floor. You perceive through a forward camera and a "
    "360-degree lidar, and you act through movement tools."
)


def _make_adapter(link, guard, *, clock=None, robot_profile=None,
                  frame_fetcher=None, tier=None):
    """Build a RospugAdapter from a Transport + guard. Imported lazily so the
    registry stays importable without pulling in roslibpy. `tier` is accepted
    for a uniform factory signature but unused — tiers are a TonyPi concept."""
    from physicalagent.robots.rospug import RospugAdapter
    return RospugAdapter(link, guard, clock=clock, robot_profile=robot_profile,
                         frame_fetcher=frame_fetcher)

ROSPUG = RobotProfile(
    name="rospug",
    embodiment="quadruped",
    # VERIFIED: robot_nodes/physicalagent_deadman.py runs on the Jetson.
    supervision=SupervisionClass.DEADMAN,
    # VERIFIED reasoning: four legs hold it statically stable when the gait
    # halts. Does NOT generalise to the biped.
    safe_state=SafeStatePolicy.FREEZE,
    safety=ROSPUG_SAFETY,
    link=LinkSpec(kind="rosbridge",
                  host=config.ROSPUG.host,
                  port=config.ROSPUG.rosbridge_port),
    embodiment_prompt=EMBODIMENT_PROMPT,
    adapter_factory=_make_adapter,
    # VERIFIED: a human has watched THIS robot move under our control across
    # several sessions (first_motion_test, walk/turn calibration, the live
    # Gemini runs). It has earned the vendor's own limits; nothing else has.
    motion_unlocked=True,
)
