"""The 17 upper-body joints SONIC will take a reference for, and safe poses.

WHY 17 AND WHY THESE
    `upper_body_position` on the planner topic is 17 f32 (zmq_manager.hpp:885).

THE WIRE ORDER IS NOT OUR ORDER  (fixed 2026-08-01, was live and wrong)
    Our table below is SERIAL: waist (3), then the whole left arm (7), then the
    whole right arm (7). That matches rt/lowstate, it matches JOINT_NAMES[12:29]
    in sonic_telemetry.py, and it matches the block comments on
    `loco_upper_body_dof_pos` in the wbc yaml. It is the order a human reads.

    It is NOT what goes on the wire. The deploy binary scatters the 17 received
    values through `upper_body_joint_isaaclab_order_in_isaaclab_index`
    (policy_parameters.hpp:80, applied at g1_deploy_onnx_ref.cpp:786), which is
    IsaacLab order: waist (3), then left/right INTERLEAVED --
    left_shoulder_pitch, right_shoulder_pitch, left_shoulder_roll,
    right_shoulder_roll, and so on. The vendor states the same permutation twice,
    independently, and the two agree (policy_parameters.hpp:80-81).

    So everything from wire slot 4 on has to be permuted. See WIRE_JOINT_NAMES.
    Sending serial order put right_shoulder_pitch's value on right_elbow, left
    the right shoulder tracking a number nobody commanded, and range-checked
    every joint past slot 3 against a DIFFERENT joint's limits.

    HOW THIS SURVIVED REVIEW, because the next person will be tempted the same
    way: the original test asserted that this table matched the telemetry table.
    Both were ours, both were serial, so it passed. Internal consistency is not
    correctness. The vendor header was the ground truth the whole time and
    nothing checked against it -- test_wire_order_matches_the_vendors_own_map
    now does, by parsing policy_parameters.hpp.

WHAT THIS BUYS, AND WHAT IT DOES NOT
    The legs are untouched. SONIC keeps balancing and all 27 locomotion modes
    keep working, because this rides on the planner topic rather than replacing
    it. What the model gets is the upper body: reach, point, carry, hold a pose
    while walking.

    It is a REFERENCE, not a servo command. The policy is free to deviate to stay
    upright, and it will -- lean far enough and the arms become a balance
    strategy, not a manipulator. Read the joints back with get_joints() rather
    than assuming the reference was achieved. That is the same discipline the
    grasp tool needs and for the same reason.

LIMITS ARE THE ROBOT'S OWN
    Taken from motor_pos_lower/upper_limit_list in the wbc yaml, entries 12-28.
    Out-of-range values are CLAMPED rather than refused: a model that asked for
    too much should get the nearest reachable pose and be told what it actually
    got, not an exception that ends the turn.
"""
from __future__ import annotations

NUM_UPPER_BODY_JOINTS = 17

# OUR order: serial, and the same order rt/lowstate reports. Every public value
# in this module -- NEUTRAL, POSES, LOWER/UPPER, from_named_joints -- is in this
# order, so a commanded list can be compared elementwise against a measured one
# without a second permutation. Do not sort, do not regroup.
JOINT_NAMES: tuple[str, ...] = (
    "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
    "left_elbow", "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
    "right_elbow", "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
)
INDEX = {name: i for i, name in enumerate(JOINT_NAMES)}

# THE order that goes on the wire: IsaacLab, left/right interleaved after the
# waist. Transcribed from upper_body_joint_isaaclab_order_in_mujoco_index
# (policy_parameters.hpp:81) = {12,13,14,15,22,16,23,17,24,18,25,19,26,20,27,21,28},
# read through sonic_telemetry.JOINT_NAMES to turn those motor indices into names.
WIRE_JOINT_NAMES: tuple[str, ...] = (
    "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "right_shoulder_pitch",
    "left_shoulder_roll", "right_shoulder_roll",
    "left_shoulder_yaw", "right_shoulder_yaw",
    "left_elbow", "right_elbow",
    "left_wrist_roll", "right_wrist_roll",
    "left_wrist_pitch", "right_wrist_pitch",
    "left_wrist_yaw", "right_wrist_yaw",
)
# wire slot k carries our index TO_WIRE[k]; FROM_WIRE undoes it.
TO_WIRE: tuple[int, ...] = tuple(INDEX[n] for n in WIRE_JOINT_NAMES)
FROM_WIRE: tuple[int, ...] = tuple(WIRE_JOINT_NAMES.index(n) for n in JOINT_NAMES)

# A duplicated or missing name would make TO_WIRE a silent mis-mapping rather
# than an error, which is precisely the failure being fixed here. Cheap to rule
# out at import, and it fires in production, not only under pytest.
assert sorted(TO_WIRE) == list(range(NUM_UPPER_BODY_JOINTS)), "TO_WIRE is not a permutation"
assert len(set(WIRE_JOINT_NAMES)) == NUM_UPPER_BODY_JOINTS
assert set(WIRE_JOINT_NAMES) == set(JOINT_NAMES)


def to_wire(values) -> list[float]:
    """Our serial order -> the IsaacLab order the policy scatters. Call this at
    the wire boundary and nowhere else, so there is exactly one place to be
    wrong."""
    return [float(values[i]) for i in TO_WIRE]


def from_wire(values) -> list[float]:
    """The inverse, for decoding anything that arrives in policy order."""
    return [float(values[i]) for i in FROM_WIRE]

# wbc yaml motor_pos_lower_limit_list / motor_pos_upper_limit_list, entries 12-28.
LOWER = (-2.618, -0.52, -0.52,
         -3.0892, -1.5882, -2.618, -1.0472, -1.972222054, -1.61443, -1.61443,
         -3.0892, -2.2515, -2.618, -1.0472, -1.972222054, -1.61443, -1.61443)
UPPER = (2.618, 0.52, 0.52,
         2.6704, 2.2515, 2.618, 2.0944, 1.972222054, 1.61443, 1.61443,
         2.6704, 1.5882, 2.618, 2.0944, 1.972222054, 1.61443, 1.61443)

# The policy's own resting reference (`loco_upper_body_dof_pos` in the yaml).
# Used as the base for every partial command: a model that names two joints must
# not have the other fifteen default to zero, which is a straight-armed posture
# the policy never chose and has to fight.
NEUTRAL = (0.0, 0.0, 0.0,
           0.0, 0.3, 0.0, 1.0, 0.0, 0.0, 0.0,
           0.0, -0.3, 0.0, 1.0, 0.0, 0.0, 0.0)

LIMBS = {"waist": range(0, 3), "left_arm": range(3, 10), "right_arm": range(10, 17)}

# Named poses. Deliberately a short curated menu next to the free-form joint
# tool: a name the model cannot mistype beats seventeen numbers it can.
# Shoulder pitch is NEGATIVE forward/up on this robot, and the elbow is
# positive-flexion -- the two sign conventions most likely to be guessed wrong.
POSES: dict[str, tuple[float, ...]] = {
    "neutral": NEUTRAL,
    "arms_down": (0.0, 0.0, 0.0,
                  0.0, 0.15, 0.0, 0.15, 0.0, 0.0, 0.0,
                  0.0, -0.15, 0.0, 0.15, 0.0, 0.0, 0.0),
    "arms_forward": (0.0, 0.0, 0.0,
                     -1.4, 0.15, 0.0, 0.5, 0.0, 0.0, 0.0,
                     -1.4, -0.15, 0.0, 0.5, 0.0, 0.0, 0.0),
    "arms_out": (0.0, 0.0, 0.0,
                 0.0, 1.3, 0.0, 0.2, 0.0, 0.0, 0.0,
                 0.0, -1.3, 0.0, 0.2, 0.0, 0.0, 0.0),
    "arms_up": (0.0, 0.0, 0.0,
                -2.5, 0.3, 0.0, 0.3, 0.0, 0.0, 0.0,
                -2.5, -0.3, 0.0, 0.3, 0.0, 0.0, 0.0),
    "reach_left": (0.3, 0.0, 0.0,
                   -1.5, 0.4, 0.0, 0.3, 0.0, 0.0, 0.0,
                   0.0, -0.3, 0.0, 1.0, 0.0, 0.0, 0.0),
    "reach_right": (-0.3, 0.0, 0.0,
                    0.0, 0.3, 0.0, 1.0, 0.0, 0.0, 0.0,
                    -1.5, -0.4, 0.0, 0.3, 0.0, 0.0, 0.0),
    "carry": (0.0, 0.0, 0.0,
              -1.2, 0.5, 0.0, 1.4, 0.0, 0.0, 0.0,
              -1.2, -0.5, 0.0, 1.4, 0.0, 0.0, 0.0),
    "wave_right": (0.0, 0.0, 0.0,
                   0.0, 0.3, 0.0, 1.0, 0.0, 0.0, 0.0,
                   -2.2, -0.6, 0.0, 0.9, 0.0, 0.0, 0.0),
}


def clamp(values) -> list[float]:
    """Bound every joint to the robot's own range. Clamps, never raises."""
    return [max(LOWER[i], min(UPPER[i], float(v))) for i, v in enumerate(values)]


def pose(name: str) -> list[float]:
    key = str(name).strip().lower().replace(" ", "_").replace("-", "_")
    if key not in POSES:
        raise ValueError(f"{name!r} is not an arm pose. Available: "
                         + ", ".join(sorted(POSES)))
    return list(POSES[key])


def from_named_joints(joints: dict, base=None) -> tuple[list[float], list[str]]:
    """Merge a partial {joint_name: radians} onto a base pose.

    Returns the full 17 AND the list of names that were not recognised, so the
    caller can tell the model exactly what it ignored. Silently dropping a
    misspelled joint is how a model concludes an arm is broken.
    """
    out = list(base if base is not None else NEUTRAL)
    unknown = []
    for name, value in (joints or {}).items():
        key = str(name).strip().lower()
        idx = INDEX.get(key)
        if idx is None:
            unknown.append(str(name))
            continue
        out[idx] = float(value)
    return clamp(out), unknown


def describe() -> str:
    return (
        "17 joints, in three groups:\n"
        "  waist:     waist_yaw (+left), waist_roll, waist_pitch (+forward)\n"
        "  left_arm:  left_shoulder_pitch (NEGATIVE raises the arm forward/up), "
        "left_shoulder_roll (+ moves it out to the side), left_shoulder_yaw, "
        "left_elbow (+ bends it), left_wrist_roll/pitch/yaw\n"
        "  right_arm: same seven, mirrored — right_shoulder_roll is NEGATIVE "
        "to move that arm out to the side\n"
        "Angles are radians. Values outside the robot's range are clamped and "
        "the result tells you what was actually used."
    )


def as_degrees(values) -> dict:
    """Readable form for a tool result — radians are hard to sanity-check."""
    import math
    return {JOINT_NAMES[i]: round(math.degrees(v), 1) for i, v in enumerate(values)}
