"""Turn an Isaac Lab root state into the fields rt/odostate carries.

Deliberately free of isaac, torch, numpy and unitree imports: rt/odostate is the
only source of pelvis height in this stack, so its rules are worth testing in the
stdlib venv rather than only on a machine with a GPU.

Returning None is a first-class answer and means "publish NOTHING". A consumer
that receives no message says "no rt/odostate received"; a consumer that receives
zeros believes the robot is at the origin, on the floor.

Frame conventions (2026-08-07, from the cross-backend review): the reference
publisher (GR00T's MuJoCo bridge) writes angular velocity in the BODY-LOCAL
frame (mj qvel[3:6]) while Isaac Lab's root_state_w carries it in the WORLD
frame. Same topic, so the Isaac caller must convert — world_ang_vel_to_body()
below is that conversion, kept here so it is pinned offline like everything
else on this topic.
"""
from __future__ import annotations

import math

# Isaac Lab emits near-unit quaternions; float32 rounding moves the norm by
# ~1e-3 at most. The window is DELIBERATELY far wider than that: inside it a
# non-unit norm is treated as rounding and normalised away; only a norm this
# grossly wrong -- the kind produced by corrupted readback (prior art: the
# numpy-2 wrong-stride buffers) or by sum-of-squares overflow -- is refused.
# Mid-window corruption (say norm 0.6) IS repaired, not refused; the window
# buys refusal of the unambiguous cases without risking false refusals of
# healthy state, and that trade is the intent, not an accident.
_NORM_MIN, _NORM_MAX = 0.5, 2.0


def _finite_seq(seq, n):
    if seq is None:
        return None
    try:
        vals = [float(seq[i]) for i in range(n)]
    except (TypeError, KeyError, IndexError, ValueError, OverflowError):
        # OverflowError: float(10**400) -- a huge python int is untrustworthy
        # state like any other and must answer None, not raise.
        return None
    if not all(math.isfinite(v) for v in vals):
        return None
    return vals


def odostate_fields(pos_xyz, quat_wxyz, lin_vel_xyz=None, ang_vel_xyz=None):
    """Validated odometry fields, or None when the state cannot be trusted.

    pos_xyz      world position, metres. Index 2 is the pelvis height.
    quat_wxyz    world orientation, W FIRST -- Isaac Lab's root_state_w order,
                 which is also what sonic_telemetry reads.
    lin_vel_xyz  optional world-frame linear velocity, m/s.
    ang_vel_xyz  optional BODY-FRAME angular velocity, rad/s (the reference
                 publisher's convention -- convert world-frame input with
                 world_ang_vel_to_body() first).

    One gate for the whole message: if ANY provided field is untrustworthy,
    the answer is None and nothing is published. A message with a validated
    pose and NaN velocities is still an invented message.
    """
    pos = _finite_seq(pos_xyz, 3)
    quat = _finite_seq(quat_wxyz, 4)
    if pos is None or quat is None:
        return None
    norm = math.sqrt(sum(c * c for c in quat))
    # not-isfinite guards the overflow path explicitly even though the window
    # comparison below would also be False for inf/nan.
    if not math.isfinite(norm) or not (_NORM_MIN < norm < _NORM_MAX):
        return None
    out = {"position": pos, "quaternion_wxyz": [c / norm for c in quat]}
    if lin_vel_xyz is not None:
        lin = _finite_seq(lin_vel_xyz, 3)
        if lin is None:
            return None
        out["linear_velocity"] = lin
    if ang_vel_xyz is not None:
        ang = _finite_seq(ang_vel_xyz, 3)
        if ang is None:
            return None
        out["angular_velocity"] = ang
    return out


def unit_quat_wxyz(quat_wxyz):
    """The quaternion normalised, or None when it cannot be trusted -- same
    finite/sanity-window gate as odostate_fields(). Used by the secondary-IMU
    publisher, which must never put an invented orientation on a real topic."""
    quat = _finite_seq(quat_wxyz, 4)
    if quat is None:
        return None
    norm = math.sqrt(sum(c * c for c in quat))
    if not math.isfinite(norm) or not (_NORM_MIN < norm < _NORM_MAX):
        return None
    return [c / norm for c in quat]


def world_ang_vel_to_body(quat_wxyz, ang_vel_world_xyz):
    """Rotate a world-frame angular velocity into the body frame.

    omega_body = R(q)^T . omega_world, with q the body's world orientation,
    w first. Returns None when either input is untrustworthy -- same contract
    as odostate_fields(). The quaternion is normalised (within the same sanity
    window) before use.
    """
    quat = _finite_seq(quat_wxyz, 4)
    w_vec = _finite_seq(ang_vel_world_xyz, 3)
    if quat is None or w_vec is None:
        return None
    norm = math.sqrt(sum(c * c for c in quat))
    if not math.isfinite(norm) or not (_NORM_MIN < norm < _NORM_MAX):
        return None
    w, x, y, z = (c / norm for c in quat)
    # Rows of R^T (= columns of R) applied to omega_world.
    ox, oy, oz = w_vec
    return [
        (1 - 2 * (y * y + z * z)) * ox + 2 * (x * y + w * z) * oy + 2 * (x * z - w * y) * oz,
        2 * (x * y - w * z) * ox + (1 - 2 * (x * x + z * z)) * oy + 2 * (y * z + w * x) * oz,
        2 * (x * z + w * y) * ox + 2 * (y * z - w * x) * oy + (1 - 2 * (x * x + y * y)) * oz,
    ]
