"""Where the G1's hands are, and what joint angles put one somewhere.

WHY THIS EXISTS
    Everything above this file commands JOINTS. A model that wants to touch a
    thing knows where the thing is, not what seven angles put a wrist on it.
    This closes that gap: forward kinematics to say where a hand IS, and inverse
    kinematics to say what angles would put it somewhere.

    The result goes out through the same `upper_body_position` reference the arm
    tools already use, so the legs never leave the balance controller. This adds
    no new authority over the robot -- only a better way to choose the numbers.

PURE STDLIB, ON PURPOSE
    No numpy, no scipy, no mujoco. The offline suite runs with zero third-party
    packages installed and that is worth keeping: it is why the whole tool
    surface can be exercised with the robot switched off. Position-only IK needs
    a 3x3 solve, which is twenty lines, rather than a linear-algebra dependency.

THE TABLE IS GENERATED, NOT TRANSCRIBED
    LINKS was emitted by a script reading the vendor MJCF, and
    test_link_table_matches_the_vendor_mjcf re-derives it from that XML with
    ElementTree. This project has already shipped one hand-copied kinematic
    table that was wrong in a way every test agreed with (see
    sonic_upper_body.WIRE_JOINT_NAMES), so a table typed by hand is not
    acceptable here even when it is short.

FRAME
    Everything is in the PELVIS frame: x forward, y left, z up, origin at the
    pelvis body. That is the frame the model can reason about without knowing
    where the robot is standing, and it is what `vr_position` uses too. Convert
    to world with the pose from rt/odostate if you need to.

WHAT IT DOES NOT MODEL
    Self-collision. The IK will happily fold an arm through the torso. Targets
    are checked for REACH, not for whether the path or the pose is sane, and the
    policy is the thing that ultimately decides what it will do with the
    reference anyway.
"""
from __future__ import annotations

import math


class Link:
    """One joint and the fixed transform from its parent body to it.

    MJCF semantics: the child body frame is the parent frame translated by
    `pos`, rotated by `quat`, and THEN rotated about `axis` by the joint value.
    Getting that order wrong produces a plausible arm that is wrong everywhere
    except the zero pose.
    """

    __slots__ = ("joint", "body", "pos", "quat", "axis", "range")

    def __init__(self, joint, body, pos, quat, axis, range_):
        self.joint, self.body = joint, body
        self.pos, self.quat, self.axis, self.range = pos, quat, axis, range_


# Generated from gear_sonic/data/robot_model/model_data/g1/
# g1_29dof_with_inspire_actuated.xml. Do not edit by hand -- regenerate.
LINKS: dict[str, tuple[Link, ...]] = {
    "left": (
        Link("waist_yaw", "waist_yaw_link",
             (0.0, 0.0, 0.0),
             (1.0, 0.0, 0.0, 0.0),
             (0.0, 0.0, 1.0), (-2.618, 2.618)),
        Link("waist_roll", "waist_roll_link",
             (-0.0039635, 0.0, 0.035),
             (1.0, 0.0, 0.0, 0.0),
             (1.0, 0.0, 0.0), (-0.52, 0.52)),
        Link("waist_pitch", "torso_link",
             (0.0, 0.0, 0.019),
             (1.0, 0.0, 0.0, 0.0),
             (0.0, 1.0, 0.0), (-0.52, 0.52)),
        Link("left_shoulder_pitch", "left_shoulder_pitch_link",
             (0.0039563, 0.10022, 0.23778),
             (0.990264, 0.139201, 1.38722e-05, -9.86868e-05),
             (0.0, 1.0, 0.0), (-3.0892, 2.6704)),
        Link("left_shoulder_roll", "left_shoulder_roll_link",
             (0.0, 0.038, -0.013831),
             (0.990268, -0.139172, 0.0, 0.0),
             (1.0, 0.0, 0.0), (-1.5882, 2.2515)),
        Link("left_shoulder_yaw", "left_shoulder_yaw_link",
             (0.0, 0.00624, -0.1032),
             (1.0, 0.0, 0.0, 0.0),
             (0.0, 0.0, 1.0), (-2.618, 2.618)),
        Link("left_elbow", "left_elbow_link",
             (0.015783, 0.0, -0.080518),
             (1.0, 0.0, 0.0, 0.0),
             (0.0, 1.0, 0.0), (-1.0472, 2.0944)),
        Link("left_wrist_roll", "left_wrist_roll_link",
             (0.1, 0.00188791, -0.01),
             (1.0, 0.0, 0.0, 0.0),
             (1.0, 0.0, 0.0), (-1.97222, 1.97222)),
        Link("left_wrist_pitch", "left_wrist_pitch_link",
             (0.038, 0.0, 0.0),
             (1.0, 0.0, 0.0, 0.0),
             (0.0, 1.0, 0.0), (-1.61443, 1.61443)),
        Link("left_wrist_yaw", "left_wrist_yaw_link",
             (0.046, 0.0, 0.0),
             (1.0, 0.0, 0.0, 0.0),
             (0.0, 0.0, 1.0), (-1.61443, 1.61443)),
    ),
    "right": (
        Link("waist_yaw", "waist_yaw_link",
             (0.0, 0.0, 0.0),
             (1.0, 0.0, 0.0, 0.0),
             (0.0, 0.0, 1.0), (-2.618, 2.618)),
        Link("waist_roll", "waist_roll_link",
             (-0.0039635, 0.0, 0.035),
             (1.0, 0.0, 0.0, 0.0),
             (1.0, 0.0, 0.0), (-0.52, 0.52)),
        Link("waist_pitch", "torso_link",
             (0.0, 0.0, 0.019),
             (1.0, 0.0, 0.0, 0.0),
             (0.0, 1.0, 0.0), (-0.52, 0.52)),
        Link("right_shoulder_pitch", "right_shoulder_pitch_link",
             (0.0039563, -0.10021, 0.23778),
             (0.990264, -0.139201, 1.38722e-05, 9.86868e-05),
             (0.0, 1.0, 0.0), (-3.0892, 2.6704)),
        Link("right_shoulder_roll", "right_shoulder_roll_link",
             (0.0, -0.038, -0.013831),
             (0.990268, 0.139172, 0.0, 0.0),
             (1.0, 0.0, 0.0), (-2.2515, 1.5882)),
        Link("right_shoulder_yaw", "right_shoulder_yaw_link",
             (0.0, -0.00624, -0.1032),
             (1.0, 0.0, 0.0, 0.0),
             (0.0, 0.0, 1.0), (-2.618, 2.618)),
        Link("right_elbow", "right_elbow_link",
             (0.015783, 0.0, -0.080518),
             (1.0, 0.0, 0.0, 0.0),
             (0.0, 1.0, 0.0), (-1.0472, 2.0944)),
        Link("right_wrist_roll", "right_wrist_roll_link",
             (0.1, -0.00188791, -0.01),
             (1.0, 0.0, 0.0, 0.0),
             (1.0, 0.0, 0.0), (-1.97222, 1.97222)),
        Link("right_wrist_pitch", "right_wrist_pitch_link",
             (0.038, 0.0, 0.0),
             (1.0, 0.0, 0.0, 0.0),
             (0.0, 1.0, 0.0), (-1.61443, 1.61443)),
        Link("right_wrist_yaw", "right_wrist_yaw_link",
             (0.046, 0.0, 0.0),
             (1.0, 0.0, 0.0, 0.0),
             (0.0, 0.0, 1.0), (-1.61443, 1.61443)),
    ),
}

# The first three entries are the waist, shared by both arms and NOT solved for.
WAIST_LINKS = 3
ARM_JOINTS = 7

# Where an object sits when the hand closes on it, in the wrist_yaw body frame:
# the mean of the four finger roots (index/middle/ring/pinky proximal),
# composed through the intermediate `*_hand_base_link`. That composition is the
# point -- reading the finger `pos` attributes directly gives (0, -0.137, 0),
# which is in the HAND BASE frame, not the wrist frame, and using it put the
# left palm inside the torso and the right palm out in the air. The mirror test
# is what caught it.
#
# Per side, because the two are mirrored in y. Derived from the vendor MJCF and
# checked against it by test_palm_offset_matches_the_vendor_finger_roots.
PALM_OFFSET = {
    "left": (0.17656, 0.00329, 0.00336),
    "right": (0.17656, -0.00329, 0.00336),
}


# --- small linear algebra, all pure python ----------------------------------
def quat_to_mat(q):
    """(w,x,y,z) -> row-major 3x3 as a 9-tuple. Unitree and MJCF both put w
    first; scipy puts it last, and swapping them yields a rotation that looks
    reasonable and is wrong."""
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
            2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
            2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y))


def axis_angle_to_mat(axis, angle):
    """Rodrigues. The joint axes here are all principal, but writing it general
    means the table can carry a non-principal axis without silently breaking."""
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z) or 1.0
    x, y, z = x / n, y / n, z / n
    c, s = math.cos(angle), math.sin(angle)
    C = 1 - c
    return (c + x * x * C, x * y * C - z * s, x * z * C + y * s,
            y * x * C + z * s, c + y * y * C, y * z * C - x * s,
            z * x * C - y * s, z * y * C + x * s, c + z * z * C)


def mat_mul(a, b):
    return tuple(
        sum(a[r * 3 + k] * b[k * 3 + c] for k in range(3))
        for r in range(3) for c in range(3))


def mat_vec(m, v):
    return tuple(sum(m[r * 3 + c] * v[c] for c in range(3)) for r in range(3))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def solve3(m, v):
    """Solve a 3x3 system by Gaussian elimination with partial pivoting.

    Returns None if singular. The damping in the IK is chosen so this should not
    happen, but returning None beats returning a huge step built from a
    near-zero pivot, which is how a solver throws an arm across the room.
    """
    a = [list(m[0:3]) + [v[0]], list(m[3:6]) + [v[1]], list(m[6:9]) + [v[2]]]
    for i in range(3):
        p = max(range(i, 3), key=lambda r: abs(a[r][i]))
        if abs(a[p][i]) < 1e-12:
            return None
        a[i], a[p] = a[p], a[i]
        for r in range(i + 1, 3):
            f = a[r][i] / a[i][i]
            for c in range(i, 4):
                a[r][c] -= f * a[i][c]
    out = [0.0, 0.0, 0.0]
    for i in (2, 1, 0):
        out[i] = (a[i][3] - sum(a[i][c] * out[c] for c in range(i + 1, 3))) / a[i][i]
    return tuple(out)


# --- kinematics -------------------------------------------------------------
def fk(values, side: str = "left", *, tool=None) -> dict:
    """Forward kinematics in the PELVIS frame.

    `values` is the 10 chain angles (3 waist + 7 arm) in the LINKS order for
    that side. Returns the wrist body origin, the palm point, and per-joint
    axes and anchors, which is everything the Jacobian needs.
    """
    chain = LINKS[side]
    if len(values) != len(chain):
        raise ValueError(f"{side} chain takes {len(chain)} values, got {len(values)}")
    if tool is None:
        tool = PALM_OFFSET[side]

    rot = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    pos = (0.0, 0.0, 0.0)
    axes, anchors = [], []
    for link, q in zip(chain, values):
        off = mat_vec(rot, link.pos)
        pos = (pos[0] + off[0], pos[1] + off[1], pos[2] + off[2])
        rot = mat_mul(rot, quat_to_mat(link.quat))
        # The joint turns about its axis expressed AFTER the fixed quat, anchored
        # at the body origin (every joint in this chain has pos="0 0 0").
        axes.append(mat_vec(rot, link.axis))
        anchors.append(pos)
        rot = mat_mul(rot, axis_angle_to_mat(link.axis, q))

    palm_off = mat_vec(rot, tool)
    return {"wrist_pos": pos, "wrist_rot": rot,
            "palm_pos": (pos[0] + palm_off[0], pos[1] + palm_off[1],
                         pos[2] + palm_off[2]),
            "axes": axes, "anchors": anchors}


def reach_limits(side: str = "left") -> tuple[float, float]:
    """Roughly how far the palm can get from the shoulder. Used to reject an
    out-of-range target BEFORE iterating, so the refusal names a distance
    instead of reporting 'did not converge'."""
    chain = LINKS[side]
    span = 0.0
    for link in chain[WAIST_LINKS + 1:]:
        span += math.sqrt(sum(c * c for c in link.pos))
    span += math.sqrt(sum(c * c for c in PALM_OFFSET[side]))
    return 0.05, span


# Restarts for when a descent stalls. A 7-DOF arm has local minima, and against
# joint limits the damped step can point straight into a bound and stop. These
# are fixed rather than random so the same target always gives the same answer:
# a solver that returns a different arm each call is one nobody can debug, and
# on a real robot it means the same instruction produces a different motion.
_RESTARTS = (
    (0.0, 0.0, 0.0, 0.6, 0.0, 0.0, 0.0),
    (-0.8, 0.3, 0.0, 1.2, 0.0, 0.0, 0.0),
    (-1.6, 0.6, 0.0, 0.9, 0.0, 0.0, 0.0),
    (0.5, 0.2, -0.5, 1.5, 0.0, 0.0, 0.0),
    (-0.4, 1.2, 0.8, 0.4, 0.0, 0.0, 0.0),
    (-2.2, 0.2, -0.8, 1.8, 0.0, 0.0, 0.0),
)


def ik(target, side: str = "left", *, waist=(0.0, 0.0, 0.0), seed=None,
       tool=None, iters: int = 200, tol: float = 0.005,
       damping: float = 0.08) -> dict:
    """Damped least squares for the 7 arm joints. Position only.

    The waist is held at `waist` rather than solved for: it is shared with the
    other arm and it moves the whole torso, so spending it on a reach is a
    decision the caller should make deliberately, not one the solver makes to
    shave a centimetre.

    Damped rather than plain pseudo-inverse because the arm passes through
    singular configurations (straight elbow) where the pseudo-inverse asks for
    an unbounded joint step. Damping trades a little accuracy for never doing
    that, which is the right trade when the output is going to a real robot.
    """
    chain = LINKS[side]
    arm = chain[WAIST_LINKS:]
    lo = [l.range[0] for l in arm]
    hi = [l.range[1] for l in arm]
    if tool is None:
        tool = PALM_OFFSET[side]

    def descend(q0):
        q = [max(lo[i], min(hi[i], float(v))) for i, v in enumerate(q0)]
        bq, be = list(q), float("inf")
        for _ in range(iters):
            state = fk(list(waist) + q, side, tool=tool)
            p = state["palm_pos"]
            e = (target[0] - p[0], target[1] - p[1], target[2] - p[2])
            err = math.sqrt(e[0] ** 2 + e[1] ** 2 + e[2] ** 2)
            if err < be:
                be, bq = err, list(q)
            if err < tol:
                break

            # Geometric Jacobian: column i is axis_i x (p_ee - anchor_i).
            cols = []
            for i in range(ARM_JOINTS):
                a = state["axes"][WAIST_LINKS + i]
                r = state["anchors"][WAIST_LINKS + i]
                cols.append(cross(a, (p[0] - r[0], p[1] - r[1], p[2] - r[2])))

            # dq = J^T (J J^T + lambda^2 I)^-1 e -- the 3x3 form, so no 7x7
            # inverse and no linear-algebra dependency.
            jjt = []
            for r in range(3):
                for c in range(3):
                    s = sum(cols[i][r] * cols[i][c] for i in range(ARM_JOINTS))
                    jjt.append(s + (damping ** 2 if r == c else 0.0))
            y = solve3(tuple(jjt), e)
            if y is None:
                break
            for i in range(ARM_JOINTS):
                q[i] = max(lo[i], min(hi[i],
                                      q[i] + sum(cols[i][k] * y[k]
                                                 for k in range(3))))
        return bq, be

    starts = ([seed] if seed is not None else []) + list(_RESTARTS)
    best_q, best_err = None, float("inf")
    for s in starts:
        q, err = descend(s)
        if err < best_err:
            best_q, best_err = q, err
        if best_err < tol:
            break

    _, span = reach_limits(side)
    state = fk(list(waist) + best_q, side, tool=tool)
    p = state["palm_pos"]
    err = math.dist(p, target)
    return {"ok": err <= max(tol, 0.02),
            "joints": {arm[i].joint: best_q[i] for i in range(ARM_JOINTS)},
            "values": best_q,
            "error_m": err,
            "palm_pos": p,
            "wrist_pos": state["wrist_pos"],
            "reach_span_m": span}
