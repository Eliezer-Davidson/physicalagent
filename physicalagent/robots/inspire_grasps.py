"""Named grasps for the Inspire hands, expanded to joint targets.

REUSES WORK THAT ALREADY EXISTS
    The library is gear_sonic_deploy/reference/hand_primitives/hand_primitives.json,
    built 2026-07-28 and proven kinematically in a viewer. It is not re-derived
    here. This module is the adapter between that file and the sidecar: it
    expands one named grasp into the 12 joint angles for a hand, and nothing else.

WHY NAMED GRASPS AND NOT TWELVE NUMBERS
    A model asked for twelve finger angles will produce twelve plausible numbers
    and no grasp. The library's own framing -- style x size x strength -- is the
    right control surface for a reasoner: the three axes are things you can
    reason about from a photograph ("it's a thin flat card, grip it lightly")
    whereas thumb_proximal_pitch is not.

WHAT A GRASP IS AND IS NOT
    A primitive is a finger POSE. It is not a successful grasp, and the project
    has an explicit claim on file that no grasp and no lift has ever been
    achieved here. Closing a hand on nothing looks identical, in joint angles,
    to closing it on a bottle. Whether anything is actually held is answered by
    the contact query, never by the fact that this function returned.
    `strength` likewise carries a force TARGET for a controller that does not
    exist yet; today it is a small extra curl. Both are said in the tool result
    rather than left for the model to assume.
"""
from __future__ import annotations

import functools
import json
import pathlib

from physicalagent.paths import WBC_DEPLOY

LIBRARY_PATH = WBC_DEPLOY / "reference" / "hand_primitives" / "hand_primitives.json"

SIDE_PREFIX = {"left": "L_", "right": "R_"}


@functools.lru_cache(maxsize=1)
def library() -> dict:
    with open(LIBRARY_PATH) as fh:
        return json.load(fh)


def grasp_names() -> list[str]:
    return sorted(library()["primitives"])


def size_names() -> list[str]:
    return [k for k in library()["sizes"] if not k.startswith("_")]


def strength_names() -> list[str]:
    return [k for k in library()["strengths"] if not k.startswith("_")]


def describe() -> str:
    """The menu, with each grasp's own tags as the hint for when to use it."""
    prims = library()["primitives"]
    rows = []
    for name in sorted(prims):
        p = prims[name]
        rows.append(f"{name} — for {', '.join(p['tags'])}"
                    + ("" if p["sizeable"] else " (size has no effect)"))
    return "\n".join(rows)


def expand(name: str, *, size: str = "medium", strength: str = "firm",
           side: str = "right") -> dict[str, float]:
    """One named grasp -> {model joint name: target radians} for one hand.

    Raises with the full menu on an unknown name: a refusal listing the
    alternatives is a turn the model can recover from.
    """
    lib = library()
    prims = lib["primitives"]
    key = str(name).strip().lower().replace(" ", "_").replace("-", "_")
    if key not in prims:
        raise ValueError(f"{name!r} is not a grasp. Available: "
                         + ", ".join(sorted(prims)))
    if side not in SIDE_PREFIX:
        raise ValueError(f"side must be 'left' or 'right', not {side!r}")
    sizes, strengths = lib["sizes"], lib["strengths"]
    if size not in sizes or size.startswith("_"):
        raise ValueError(f"size must be one of {', '.join(size_names())}")
    if strength not in strengths or strength.startswith("_"):
        raise ValueError(f"strength must be one of {', '.join(strength_names())}")

    prim = prims[key]
    scale = sizes[size] if prim["sizeable"] else 1.0
    overclose = strengths[strength]["overclose"]
    prefix = SIDE_PREFIX[side]
    joint_map = lib["meta"]["joint_map"]

    targets: dict[str, float] = {}
    for closure, dof in zip(prim["vec"], lib["meta"]["dof_order"]):
        c = closure * scale
        # Over-curl only applies where the finger is already closing. Adding it
        # to an open hand would quietly turn 'release' into a light grip.
        if c > 1e-6:
            c = min(1.0, c + overclose)
        for spec in joint_map[dof].values():
            joint = prefix + spec[0]
            if len(spec) == 2:                       # [joint, closed_angle]
                targets[joint] = c * spec[1]
            else:                                    # [joint, open, closed]
                lo, hi = spec[1], spec[2]
                targets[joint] = lo + c * (hi - lo)
    return targets


def grasp_result(name: str, size: str, strength: str, side: str) -> dict:
    """What the tool reports back. The caveat travels with the result rather
    than living only in a docstring the model never sees."""
    lib = library()
    return {
        "grasp": name, "size": size, "strength": strength, "hand": side,
        "force_target": lib["strengths"][strength]["force"],
        "note": "COMMAND ONLY. The numbers below are the targets just sent; the "
                "fingers have not moved yet and nothing here measures them. Call "
                "get_hands() to see where they actually got to, and "
                "get_contacts() to find out whether they are touching anything. "
                "Grip force is not closed-loop yet; 'strength' is a target plus a "
                "small extra curl.",
    }
