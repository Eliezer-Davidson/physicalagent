"""MiniHexa — profile #3, and the first hexapod in the registry.

Three facts about this body drive its profile. It is STATICALLY STABLE, so its
safe state FREEZES (freezing the legs where they are is safe — there is no
settle-to-stance to command, unlike the biped). Its supervision is
OPERATOR_BOUNDED: the firmware has NO watchdog and its walk LATCHES a velocity,
so the robot never self-stops — every walk must be a host-bounded burst followed
by a stop, with a human present and able to cut power. And it links over a raw
wire protocol (CH340 serial, the full-control default; BLE as an alternative with
a reduced command set), not ROS or JSON-RPC.
"""
from __future__ import annotations

from physicalagent import config
from physicalagent.profiles.base import (
    LinkSpec, RobotProfile, SafeStatePolicy, SupervisionClass,
)
from physicalagent.safety import MINIHEXA_SAFETY

EMBODIMENT_PROMPT = (
    "You are the reasoning brain of a small six-legged walking robot — a hexapod "
    "(a HiWonder MiniHexa). You walk in short bounded bursts — each walk step moves you for a "
    "fraction of a second and then stops on its own, so there is NO continuous "
    "walking; re-issue walk to keep going. You can hold body poses (tilt, shift, "
    "raise and lower the body) while standing, and — ONLY when tethered over USB "
    "— move individual leg joints. Every move self-terminates. Because the "
    "firmware will keep walking until told to stop, a human operator is present "
    "and watching; keep your bursts short and deliberate."
)


def _make_adapter(link, guard, *, clock=None, robot_profile=None,
                  frame_fetcher=None, tier=None):
    """Build a MiniHexaAdapter from a MiniHexaLink + guard.

    Imported lazily so the profile registry stays importable without pulling in
    the adapter (or pyserial / bleak). `frame_fetcher` is accepted for a uniform
    factory signature but unused — the MiniHexa has no camera in this path.
    `tier` defaults to LOCKED, the timid motion band, when not explicitly selected.
    """
    from physicalagent.robots.minihexa import MiniHexaAdapter
    from physicalagent.robots.minihexa_motion import MotionTier
    return MiniHexaAdapter(link, guard, clock=clock, robot_profile=robot_profile,
                           tier=tier or MotionTier.LOCKED)


MINIHEXA = RobotProfile(
    name="minihexa",
    embodiment="hexapod",
    # No watchdog + velocity-latched walk + mobile: none of DEADMAN,
    # COMMAND_EXPIRY or TETHERED is honest. The host bounds every burst.
    supervision=SupervisionClass.OPERATOR_BOUNDED,
    # Statically stable: cutting motion and holding the legs is safe. No settle.
    safe_state=SafeStatePolicy.FREEZE,
    safety=MINIHEXA_SAFETY,
    # Serial (CH340) is the full-control default: the BLE firmware cannot do
    # per-joint (L) or action-stop (K|2). Choose BLE explicitly via --link.
    link=LinkSpec(kind="serial",
                  device=config.MINIHEXA.device,
                  baud=config.MINIHEXA.baud),
    embodiment_prompt=EMBODIMENT_PROMPT,
    adapter_factory=_make_adapter,
    # Nothing has watched this robot move under our control yet: stays locked, and
    # the motion tier stays LOCKED, until a human decides otherwise.
    motion_unlocked=False,
)
