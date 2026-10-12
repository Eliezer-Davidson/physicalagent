"""RobotProfile — what the rest of the system needs to know about one robot.

This is the seam that makes `./robot --robot <name>` a one-word change. A
profile is pure data plus two factories; it imports no transport and touches
no hardware, so the registry can be read without connecting to anything.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Callable

from physicalagent.safety import SafetyError, SafetyProfile


class SupervisionClass(str, Enum):
    """How we guarantee the robot stops when we stop talking to it.

    DEADMAN           robot runs our watchdog; PC heartbeats; robot self-stops
                      on silence. Continuous motion is allowed.
    COMMAND_EXPIRY    firmware accepts only bounded commands and stops by
                      default. Safe by construction; no continuous motion.
    TETHERED          non-mobile; cannot run away. Unplug = dead.
    OPERATOR_BOUNDED  firmware has NO watchdog and its walk command LATCHES a
                      velocity, so the robot never self-stops — not on silence,
                      not on link loss. Safe motion is therefore the HOST's
                      responsibility: every walk is issued as a short bounded
                      burst immediately followed by a stop, with a human operator
                      present and able to cut power. This class is only sound
                      under active supervision — a wedged host or a dropped link
                      leaves a latched walk running until power is removed. It is
                      the honest label for hardware that none of DEADMAN (no
                      watchdog), COMMAND_EXPIRY (commands do not expire — they
                      latch) or TETHERED (it is mobile) describes.
    """

    DEADMAN = "deadman"
    COMMAND_EXPIRY = "command_expiry"
    TETHERED = "tethered"
    OPERATOR_BOUNDED = "operator_bounded"
    DYNAMIC_HUMANOID = "dynamic_humanoid"


class SafeStatePolicy(str, Enum):
    """What "safe" physically means for this body.

    FREEZE             cut motion, hold servo angles. Correct for a statically
                       stable quadruped/hexapod.
    SETTLE_TO_STANCE   stop the action group, then command a stable stance.
                       Required for a biped: freezing mid-stride is a fall.
    LOWER_AND_RELEASE  move to rest pose, then disable torque. For arms/hands.
    """

    FREEZE = "freeze"
    SETTLE_TO_STANCE = "settle_to_stance"
    LOWER_AND_RELEASE = "lower_and_release"


@dataclass(frozen=True)
class LinkSpec:
    """How to reach the robot. Deliberately NOT a Transport.

    The Transport ABC is ROS-shaped; forcing JSON-RPC or a Feetech servo bus
    into that shape would be a lie. LinkSpec is just the address -- each
    adapter owns its own link type.
    """

    kind: str                      # "rosbridge" | "http_rpc" | "serial"
    host: str | None = None
    port: int | None = None
    device: str | None = None
    baud: int | None = None


@dataclass(frozen=True)
class RobotProfile:
    name: str
    embodiment: str                # quadruped|hexapod|humanoid|arm|hand
    supervision: SupervisionClass | None
    safe_state: SafeStatePolicy
    safety: SafetyProfile
    link: LinkSpec
    embodiment_prompt: str
    adapter_factory: Callable | None = None
    doctor: Callable | None = None
    motion_tools_enabled: bool = True
    # "Stays False until a human has watched THIS robot move under our
    # control." That was config.MOTION_UNLOCKED's own docstring, but it was a
    # single global: the moment a second profile was registered, its first ever
    # run inherited the ROSPug's field-testing and started unlocked. The
    # decision is per-body, so it lives on the body. Locked is the default, and
    # a new profile has to earn its way out of it.
    motion_unlocked: bool = False

    # motion_tools_enabled=False means SENSING-ONLY. It is enforced twice:
    # here at construction (a robot with no supervision story may not claim
    # motion tools), and at the tool boundary by RobotAdapter, which both
    # withholds the motion declarations and refuses the calls. The second half
    # used to be missing entirely -- the flag validated a profile that nothing
    # downstream consulted, so a sensing-only robot still handed walk/turn/
    # do_action to the model.
    def __post_init__(self):
        # Spec 4.1, verbatim: "A robot with no supervision story gets no
        # motion tools. Not negotiable." Enforced here so it cannot be
        # forgotten when a new robot is added in a hurry.
        if self.supervision is None and self.motion_tools_enabled:
            raise SafetyError(
                f"profile {self.name!r} declares no supervision class, so "
                f"motion_tools_enabled must be False — refusing to build"
            )
