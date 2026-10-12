"""Bridge between describe_scene's robot-relative answers and goto's world frame.

The sidecar reports each object as (range_m, bearing_deg) relative to the robot
(bearing 0 straight ahead, positive LEFT — sidecar._op_objects), while goto()
takes world-frame metres. Without this bridge the model has to do trigonometry
unaided, which the 2026-08-12 mapping ranked among the top gaps for scene tasks.
Pure stdlib: this module must import under the test venv with no dependencies.
"""
from __future__ import annotations

import math

MIN_STANDOFF_M = 0.4   # never resolve to a point inside furniture


def object_world_xy(robot_xy, heading_deg, bearing_deg, range_m):
    th = math.radians(float(heading_deg) + float(bearing_deg))
    return (robot_xy[0] + float(range_m) * math.cos(th),
            robot_xy[1] + float(range_m) * math.sin(th))


def standoff_point(robot_xy, object_xy, standoff_m=0.6):
    dx = object_xy[0] - robot_xy[0]
    dy = object_xy[1] - robot_xy[1]
    dist = math.hypot(dx, dy)
    yaw_deg = math.degrees(math.atan2(dy, dx))
    standoff = max(MIN_STANDOFF_M, float(standoff_m))
    if dist <= standoff:
        # Already close enough: hold position, just face it. Walking backwards
        # away from the object would surprise the model more than staying put.
        return (robot_xy[0], robot_xy[1], yaw_deg)
    f = (dist - standoff) / dist
    return (robot_xy[0] + f * dx, robot_xy[1] + f * dy, yaw_deg)


def augment_objects(reply, pose):
    """Add world_xy to each object when the robot's own pose is fresh. Mutates
    nothing: returns a shallow-copied reply so a cached sidecar answer is safe.

    A malformed sidecar object (missing bearing_deg/range_m) is skipped, not
    fatal: one bad entry must not KeyError describe_scene/goto_object for the
    whole scene. Nothing is rounded HERE -- resolve_goto_object() consumes this
    exact value on the control path, so rounding it would be a navigation
    precision loss, not just a display nicety. It is not full precision
    though, and saying so used to be wrong: the sidecar already rounded
    range_m to 2 dp and bearing_deg to 1 dp before this function saw them,
    which costs <=5 mm radially plus R*8.7e-4 laterally, i.e. <=9 mm at 5 m --
    an order of magnitude inside MIN_STANDOFF_M."""
    out = dict(reply)
    out["objects"] = [dict(o) for o in reply.get("objects", [])]
    note = str(reply.get("note", ""))
    omitted = []
    if pose and pose.get("available") and pose.get("fresh"):
        rx = pose["position_m"]["x"]
        ry = pose["position_m"]["y"]
        hd = pose["heading_deg"]
        for o in out["objects"]:
            if "bearing_deg" not in o or "range_m" not in o:
                omitted.append(o.get("name") or "<unnamed>")
                continue
            wx, wy = object_world_xy((rx, ry), hd, o["bearing_deg"], o["range_m"])
            o["world_xy"] = [wx, wy]
        out["note"] = note + ("; " if note else "") + \
            "world_xy is the world-frame point goto() accepts"
    else:
        out["note"] = note + ("; " if note else "") + \
            "world_xy unavailable (no fresh odometry) — use get_status() + trigonometry"
    if omitted:
        out["note"] += ("; " if out["note"] else "") + \
            "world_xy omitted for malformed object(s) (missing bearing_deg/" \
            f"range_m): {', '.join(omitted)}"
    return out


def resolve_goto_object(reply, name, standoff_m, *, robot_xy):
    objs = {o.get("name"): o for o in reply.get("objects", [])}
    if name not in objs:
        present = ", ".join(sorted(n for n in objs if n)) or "nothing"
        raise ValueError(
            f"no object named '{name}' in the scene. describe_scene() sees: "
            f"{present}. Names must match exactly.")
    o = objs[name]
    if "world_xy" not in o:
        raise RuntimeError(
            "cannot resolve the object's world position: odometry is not fresh. "
            "Read get_status() and use goto(x, y) instead.")
    return standoff_point(robot_xy, tuple(o["world_xy"]), standoff_m)
