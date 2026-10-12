"""The robot registry. `./robot --robot <name>` resolves through here."""
from __future__ import annotations

from physicalagent.profiles.base import (
    LinkSpec, RobotProfile, SafeStatePolicy, SupervisionClass,
)
from physicalagent.profiles.minihexa import MINIHEXA
from physicalagent.profiles.rospug import ROSPUG
from physicalagent.profiles.tonypi import TONYPI

# physicalagent.profiles.g1 is a DRAFT and is deliberately NOT registered.
# A registered profile is a promise the CLI can build that robot, and this one
# cannot be built: it has no adapter_factory, and link_for() has no builder for
# its "dds" link kind, so `./robot --robot g1` died with
#   ValueError: profile 'g1': no link builder for kind 'dds'
# after listing g1 as available. An unbuildable entry in the registry is worse
# than an absent one — it advertises a robot the fleet cannot reach. It gets
# registered when it has a link, an adapter and a supervision story that means
# something on a 35 kg biped.
_REGISTRY: dict[str, RobotProfile] = {
    ROSPUG.name: ROSPUG,
    TONYPI.name: TONYPI,
    MINIHEXA.name: MINIHEXA,
}

__all__ = [
    "LinkSpec", "RobotProfile", "SafeStatePolicy", "SupervisionClass",
    "UnknownRobotError", "get_profile", "list_profiles",
]


class UnknownRobotError(KeyError):
    """Raised when a robot name is not in the registry.

    Subclasses KeyError (rather than being a fresh Exception type) so every
    existing `pytest.raises(KeyError)` / `except KeyError` caller keeps
    working unchanged. The point of having a dedicated type at all is so
    call sites that specifically want "bad --robot name" can catch *this*
    narrowly, instead of catching every bare KeyError that might be raised
    for an unrelated reason during real command work.
    """


def list_profiles() -> tuple[str, ...]:
    """Every registered robot name, sorted."""
    return tuple(sorted(_REGISTRY))


def get_profile(name: str) -> RobotProfile:
    """Look up a robot by name, case-insensitively.

    Raises UnknownRobotError (a KeyError subclass) naming what IS available
    -- a typo at the CLI should tell you the answer, not just that you were
    wrong.
    """
    try:
        return _REGISTRY[name.strip().lower()]
    except KeyError:
        raise UnknownRobotError(
            f"unknown robot {name!r}; available: {', '.join(list_profiles())}"
        ) from None
