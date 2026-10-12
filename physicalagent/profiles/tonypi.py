"""TonyPi — profile #2, and the first humanoid in the registry.

A biped, so two facts differ from the ROSPug: its safe state SETTLES TO A STANCE
(freezing mid-stride is a fall), and its supervision is COMMAND_EXPIRY (the tool
surface is discrete self-terminating commands, not continuous walk). It links
over JSON-RPC/HTTP, not rosbridge, and it carries a real adapter_factory so the
CLI can build it without hard-coding an adapter class.
"""
from __future__ import annotations

from physicalagent import config
from physicalagent.profiles.base import (
    LinkSpec, RobotProfile, SafeStatePolicy, SupervisionClass,
)
from physicalagent.safety import TONYPI_SAFETY

EMBODIMENT_PROMPT = (
    "You are the reasoning brain of a small humanoid robot (a HiWonder TonyPi), "
    "a bipedal figure that stands and balances on two legs. You perceive through "
    "a forward camera on a pan/tilt head, and you act through discrete, "
    "self-terminating commands: named gesture routines that keep you standing, "
    "per-joint servo moves, and head look commands. Because you balance on two "
    "legs, every action must leave you in a stable, upright stance — there is no "
    "continuous walking, and a command that ended mid-motion would risk a fall."
)


def _make_adapter(link, guard, *, clock=None, robot_profile=None,
                  frame_fetcher=None, tier=None):
    """Build a TonyPiAdapter from a TonyLink + guard.

    Imported lazily so the profile registry stays importable without pulling in
    the adapter (and urllib). `frame_fetcher` is accepted for a uniform factory
    signature but unused — TonyPi frames come from link.snapshot(). `tier`
    defaults to LOCKED, the timid servo band, when not explicitly selected.
    """
    from physicalagent.robots.tonypi import TonyPiAdapter
    from physicalagent.robots.tonypi_servo import ServoTier

    resolved = tier or ServoTier.LOCKED
    return TonyPiAdapter(link, guard, clock=clock, robot_profile=robot_profile,
                         tier=resolved, hand=make_hand(resolved))


def make_hand(tier):
    """Build the AiHand on the arm, or None when no hand is configured.

    The hand is OPT-IN through config.AIHAND.enabled (env $AIHAND). It is not
    autodetected, and a hand that is configured but unreachable is NOT silently
    downgraded to no-hand: the link opens lazily, so an unreachable hand shows
    up as a loud error on the first finger command rather than as a tool that
    quietly went missing. An earlier draft dialled a TCP bridge here inside a
    bare `except` — with no bridge running, every session started handless and
    said nothing about it.
    """
    if not config.AIHAND.enabled:
        return None
    from physicalagent.robots.aihand import AiHand
    from physicalagent.robots.aihand_link import (
        SerialAiHandLink, TcpAiHandLink,
    )
    if config.AIHAND.transport == "serial":
        link = SerialAiHandLink(device=config.AIHAND.device,
                                baud=config.AIHAND.baud)
    else:
        link = TcpAiHandLink(host=config.AIHAND.host, port=config.AIHAND.port)
    return AiHand(link, tier=tier, framing=config.AIHAND.framing)


TONYPI = RobotProfile(
    name="tonypi",
    embodiment="humanoid",
    # The exposed tool surface is DISCRETE bounded commands — action groups that
    # self-terminate and servo position moves — with no continuous-walk function,
    # so each command stops by default: COMMAND_EXPIRY, not DEADMAN.
    supervision=SupervisionClass.COMMAND_EXPIRY,
    # A biped frozen mid-stride is a fall, so stop must settle to a stable stance
    # rather than freezing the servos wherever they happen to be.
    safe_state=SafeStatePolicy.SETTLE_TO_STANCE,
    safety=TONYPI_SAFETY,
    link=LinkSpec(kind="http_rpc",
                  host=config.TONYPI.host,
                  port=config.TONYPI.rpc_port),
    embodiment_prompt=EMBODIMENT_PROMPT,
    adapter_factory=_make_adapter,
    # Not yet watched moving under our control on hardware: stays locked, and the
    # servo tier stays LOCKED, until a human decides otherwise.
    motion_unlocked=False,
)
