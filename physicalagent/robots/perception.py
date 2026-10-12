"""Pure lidar interpretation — no ROS, no network.

summarize_scan() turns a raw LaserScan dict (from /scan_raw: full 360 deg,
CCW, metres) into a compact per-sector summary the model reasons over. Returns
at or below the robot's body radius are dropped, as are NaN/inf/out-of-range.
"""
from __future__ import annotations

import math
import statistics

# sector centre angle (rad) -> half-width; front/left/right/back, 90 deg each
_SECTORS = {
    "front": (0.0, math.pi / 4),
    "left": (math.pi / 2, math.pi / 4),
    "right": (-math.pi / 2, math.pi / 4),
    "back": (math.pi, math.pi / 4),
}

SELF_BODY_M = 0.12   # returns nearer than this are the robot's own chassis
MAX_RANGE_M = 12.0


def _ang_diff(a: float, b: float) -> float:
    d = abs(a - b)
    return d if d <= math.pi else (2 * math.pi - d)


def summarize_scan(scan: dict | None, self_body_m: float = SELF_BODY_M) -> dict:
    """Per-sector {nearest_m, median_m} (or "no returns"). Raises ValueError
    on a missing scan."""
    if not scan:
        raise ValueError("no scan data")
    ranges = scan.get("ranges", [])
    amin = scan.get("angle_min", 0.0)
    inc = scan.get("angle_increment", 0.0)
    if not ranges or not inc:
        raise ValueError("scan has no ranges/angle_increment")

    buckets: dict[str, list[float]] = {name: [] for name in _SECTORS}
    for i, d in enumerate(ranges):
        if not isinstance(d, (int, float)) or d != d or d <= self_body_m or d > MAX_RANGE_M:
            continue
        a = amin + i * inc
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        for name, (centre, half) in _SECTORS.items():
            if _ang_diff(a, centre) <= half:
                buckets[name].append(d)

    out = {}
    for name, vals in buckets.items():
        if vals:
            out[name] = {"nearest_m": round(min(vals), 2),
                         "median_m": round(statistics.median(vals), 2)}
        else:
            out[name] = "no returns"
    return out


def min_front_clearance(scan: dict) -> float | None:
    """Nearest valid return in the front sector, or None. Convenience for a
    pre-walk safety check."""
    summary = summarize_scan(scan)
    front = summary.get("front")
    return front["nearest_m"] if isinstance(front, dict) else None
