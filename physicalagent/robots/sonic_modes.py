"""The 27 SONIC locomotion modes, as a table a language model can choose from.

SOURCE OF TRUTH
    `LocomotionMode` in gear_sonic_deploy/.../include/localmotion_kplanner.hpp:78-106.
    The integer values are the wire values on the ZMQ `planner` topic; they are
    an enum ordinal, so they are pinned by test rather than re-derived here.

WHY ALL 27 AND NOT 4
    The planner binary validates the mode against a range that depends on which
    ONNX it loaded: V0 -> 4 modes, V1 -> 20, V2 -> 27
    (localmotion_kplanner.hpp:291-303, selected by substring match on the planner
    path at g1_deploy_onnx_ref.cpp:2399-2411). We load
    `planner/target_vel/V2/planner_sonic.onnx`, so all 27 are in range. Exposing
    four of them was a limit we imposed, not one SONIC imposes.

WHAT `kind` MEANS, and why the model is told
    static   — the robot holds a pose and does not travel. Commanding speed is
               meaningless; the planner ignores movement_direction.
    standing — upright; travel and styled walks live here. speed applies.
    ground   — the robot is on the floor (crawling, lying). Getting back up is
               NOT guaranteed by any of these, which is why they are flagged.

    The classification mirrors is_static_motion_mode / is_standing_motion_mode /
    is_squat_motion_mode (same header, lines 108-149). A mode can be in more than
    one of the vendor's predicates; `kind` records the one that matters for
    deciding whether it is safe to ask for speed, so it is our summary of theirs
    rather than a copy.

UNTESTED IS SAID OUT LOUD
    Only IDLE, SLOW_WALK, WALK and IDEL_SQUAT have been observed driving this
    robot. The other 23 are in range and correctly encoded, and that is a
    different claim from "they work". `tested` carries the difference, and it
    reaches the model in the tool description so it can hedge the same way.

`speed` AND `height` ARE THE VENDOR'S OPERATING POINT, NOT A DEFAULT WE CHOSE
    Every mode is trained at a particular speed and body height, and asking for
    it at the wrong one does not produce a slower version of the mode — it
    produces a mode that appears not to work. We learned this the expensive way:
    `jump_forward` was commanded with speed 0.0 and "kind of took a big step
    forward and stopped", which reads as "SONIC blocks flight phases" and is
    instead what mode 17 looks like when told to stay where it is.

    The numbers are copied from the vendor's OWN reference input,
    `GamepadManager::applySpeedAndHeight` (gamepad_manager.hpp:429-467) — the
    driver NVIDIA ships and demos with. Two facts from it matter:

      * `-1.0` means "use the mode's own default", and it is a real wire value,
        distinct from 0.0 which means "stand still" (localmotion_kplanner.hpp:64).
      * the gamepad NEVER sends speed 0.0 to a standing mode. When the stick
        centres it switches the mode to IDLE and sets speed back to -1
        (gamepad.hpp:697-700). Mode 17 at speed 0.0 is a state the vendor's own
        driver cannot produce, and we were producing it on every set_mode call
        that did not name a speed.

    So an unspecified speed now resolves to this table, not to zero. A speed the
    caller DID name still wins, and is still clamped by the link.
"""
from __future__ import annotations

from dataclasses import dataclass

STATIC = "static"
STANDING = "standing"
GROUND = "ground"


@dataclass(frozen=True)
class Mode:
    value: int
    name: str
    kind: str
    blurb: str
    tested: bool = False
    speed: float = -1.0
    height: float = -1.0

    @property
    def takes_speed(self) -> bool:
        return self.kind == STANDING


# Ordered by wire value. Names are ours (lowercase, guessable by a model); the
# vendor's enum identifier is in the comment where the two differ.
MODES: tuple[Mode, ...] = (
    Mode(0, "idle", STATIC, "stand still and balance", tested=True),
    Mode(1, "slow_walk", STANDING, "walk at 0.1-0.8 m/s; the default for careful travel", tested=True, speed=0.4),
    Mode(2, "walk", STANDING, "walk at 0.8-2.5 m/s", tested=True),
    Mode(3, "run", STANDING, "run at 2.5-7.5 m/s; violent, and clamped well below its range here", speed=1.5),
    Mode(4, "squat", STATIC, "squat down and hold it", tested=True, height=0.4),          # IDEL_SQUAT
    Mode(5, "kneel_two_legs", STATIC, "kneel on both knees", height=0.4),                 # IDEL_KNEEL_TWO_LEGS
    Mode(6, "kneel", STATIC, "kneel on one knee", height=0.4),                            # IDEL_KNEEL
    Mode(7, "lie_face_down", GROUND, "lie face down on the floor"),           # IDEL_LYING_FACE_DOWN
    Mode(8, "crawl", GROUND, "crawl forward on hands and knees", speed=0.7, height=0.4),  # CRAWLING
    Mode(9, "boxing_stance", STANDING, "hold a boxing guard while standing", speed=0.7),  # IDEL_BOXING
    Mode(10, "boxing_walk", STANDING, "advance while holding a boxing guard", speed=0.7),
    Mode(11, "punch_left", STANDING, "throw a straight left", speed=0.7),
    Mode(12, "punch_right", STANDING, "throw a straight right", speed=0.7),
    Mode(13, "punch_random", STANDING, "throw punches with either arm", speed=0.7),
    Mode(14, "elbow_crawl", GROUND, "crawl on elbows, body low", speed=0.7, height=0.3),
    Mode(15, "hook_left", STANDING, "throw a left hook", speed=0.7),
    Mode(16, "hook_right", STANDING, "throw a right hook", speed=0.7),
    Mode(17, "jump_forward", STANDING, "jump forward; the most aggressive mode here"),
    Mode(18, "stealth_walk", STANDING, "walk low and quietly"),
    Mode(19, "injured_walk", STANDING, "walk with a limp"),
    Mode(20, "ledge_walk", STANDING, "walk carefully as if along a narrow ledge"),
    Mode(21, "carry", STANDING, "walk holding an object in both arms"),       # OBJECT_CARRYING
    Mode(22, "stealth_walk_2", STANDING, "a second, lower stealth gait"),
    Mode(23, "dance_walk", STANDING, "walk with a happy, bouncing gait"),     # HAPPY_DANCE_WALK
    Mode(24, "zombie_walk", STANDING, "walk stiff-armed like a zombie"),
    Mode(25, "gun_walk", STANDING, "walk holding a two-handed weapon pose"),
    Mode(26, "scare_walk", STANDING, "walk in a scared, cowering posture"),
)

BY_NAME: dict[str, Mode] = {m.name: m for m in MODES}
BY_VALUE: dict[int, Mode] = {m.value: m for m in MODES}

# Aliases for the words a model reaches for first. Kept small and one-way: this
# maps synonyms onto real modes, it never invents a mode that does not exist.
ALIASES = {
    "stand": "idle",
    "stop": "idle",
    "still": "idle",
    "sit": "squat",
    "crouch": "squat",
    "walking": "slow_walk",
    "jog": "walk",
    "sprint": "run",
    "punch": "punch_random",
    "box": "boxing_stance",
    "dance": "dance_walk",
    "sneak": "stealth_walk",
    "prone": "lie_face_down",
    "lie_down": "lie_face_down",
}

# Height, in metres, of the pelvis the planner is asked to hold. -1 means "use
# whatever this mode's default is" (localmotion_kplanner.hpp:65,216 — the
# default is 0.7887).
#
# The floor is per-kind, because a single floor cannot be right for both. 0.50
# is ours and applies to STANDING: below it the robot is folded up and cannot
# walk. But the vendor commands 0.4 for squat and kneel and 0.3 for the elbow
# crawl, and a flat 0.50 floor silently raised those to a height those modes are
# not trained at — so STATIC and GROUND get the vendor's own band instead
# (gamepad.hpp:709-723 clamps squat/kneel to [0.1, 0.8]). Lying face down at 0.1
# is not a mistake, it is the mode.
HEIGHT_DEFAULT = -1.0
HEIGHT_MIN_M = 0.50            # STANDING only
HEIGHT_MIN_GROUNDED_M = 0.10   # STATIC and GROUND — the vendor's floor, not ours
HEIGHT_MAX_M = 0.85


def resolve(name: str) -> Mode:
    """Map a name the model produced onto a real mode, or raise with the full
    menu. A refusal that lists the alternatives is a turn the model can recover
    from; a bare KeyError is not."""
    key = str(name).strip().lower().replace(" ", "_").replace("-", "_")
    key = ALIASES.get(key, key)
    mode = BY_NAME.get(key)
    if mode is None:
        raise ValueError(
            f"{name!r} is not a SONIC mode. Available: " + ", ".join(BY_NAME))
    return mode


def clamp_height(height_m: float | None, mode: Mode | None = None) -> float:
    """Bound a requested body height. None keeps the mode's own default rather
    than silently picking one for it.

    `mode` selects which floor applies. Passing None keeps the STANDING floor,
    which is the conservative answer for a caller that does not know what mode
    it is in — set_height() is exactly that caller.
    """
    if height_m is None:
        return HEIGHT_DEFAULT
    h = float(height_m)
    if h < 0:
        return HEIGHT_DEFAULT
    floor = (HEIGHT_MIN_GROUNDED_M if mode is not None and mode.kind in (STATIC, GROUND)
             else HEIGHT_MIN_M)
    return max(floor, min(HEIGHT_MAX_M, h))


def menu(tested_only: bool = False) -> str:
    """The mode list as the model sees it, marked so it knows what is proven."""
    rows = []
    for m in MODES:
        if tested_only and not m.tested:
            continue
        mark = "" if m.tested else " [untested]"
        rows.append(f"{m.name} ({m.kind}) — {m.blurb}{mark}")
    return "\n".join(rows)
