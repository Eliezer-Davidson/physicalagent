#!/usr/bin/env python
"""Turn the A/B run into a verdict about whether the prompt moved the robot.

Reads the simulator's ground-truth pelvis log and the phase marks written by
sonic_prove.sh, and reports how far the robot actually travelled in each phase.

The comparison is the point. Translation during the Gemini phase alone proves
nothing -- the policy could be walking of its own accord. Translation in the
Gemini phase AND near-zero translation in the control phase, same policy, same
session, is what distinguishes "the prompt drove it" from "it was doing that
anyway".
"""
from __future__ import annotations

import csv
import math
import os
import pathlib
import sys


def read_pose(out):
    # the simulator suffixes its pid, so a stray second writer is visible
    paths = sorted(out.glob("pose.csv*"))
    if len(paths) > 1:
        print(f"WARNING: {len(paths)} pose logs -- more than one simulator wrote here: "
              f"{[p.name for p in paths]}")
    rows = []
    for p in paths:
        with open(p) as fh:
            for r in csv.DictReader(fh):
                rows.append((float(r["wall_s"]), float(r["sim_s"]), float(r["x"]),
                             float(r["y"]), float(r["z"]), r.get("event", "")))
    rows.sort()
    return rows


def read_marks(path):
    marks = {}
    with open(path) as fh:
        for line in fh:
            name, t = line.strip().split(",")
            marks[name] = float(t)
    return marks


def phase(rows, t0, t1):
    return [r for r in rows if t0 <= r[0] <= t1]


JUMP_M = 0.10   # >10 cm between 20 Hz samples is not walking, it is a discontinuity


def summarise(name, seg):
    if len(seg) < 2:
        return f"{name:8s}  no samples"
    x0, y0 = seg[0][2], seg[0][3]
    x1, y1 = seg[-1][2], seg[-1][3]
    net = math.hypot(x1 - x0, y1 - y0)
    path_len = sum(math.hypot(b[2] - a[2], b[3] - a[3]) for a, b in zip(seg, seg[1:]))
    dur = seg[-1][0] - seg[0][0]          # WALL duration; sim time may stall
    zmin = min(r[4] for r in seg)
    speed = net / dur if dur > 0 else 0.0
    resets = sum(1 for r in seg if r[5] == "RESET")
    jumps = [(a[0], math.hypot(b[2] - a[2], b[3] - a[3]))
             for a, b in zip(seg, seg[1:]) if math.hypot(b[2] - a[2], b[3] - a[3]) > JUMP_M]
    line = (f"{name:8s}  {dur:5.1f}s  net {net:5.2f} m "
            f"(dx {x1-x0:+.2f}, dy {y1-y0:+.2f})  path {path_len:5.2f} m  "
            f"mean {speed:.2f} m/s  min height {zmin:.2f} m")
    if resets:
        line += f"\n          {resets} SIM RESET(S) in this phase -- the robot fell and respawned"
    if jumps:
        line += (f"\n          {len(jumps)} position discontinuit(y/ies) > {JUMP_M} m "
                 f"(largest {max(j for _, j in jumps):.2f} m) -- NOT physical motion")
    return line, net, path_len, resets, len(jumps)


def default_artifacts() -> pathlib.Path:
    """$PA_ARTIFACTS, or this repository's own artifacts/ -- never a home dir."""
    env = os.environ.get("PA_ARTIFACTS")
    if env:
        return pathlib.Path(env).expanduser()
    return pathlib.Path(__file__).resolve().parents[1] / "artifacts"


def main() -> int:
    out = (pathlib.Path(sys.argv[1]) if len(sys.argv) > 1
           else default_artifacts() / "sonic_prove")
    rows = read_pose(out)
    marks = read_marks(out / "marks.csv")
    print(f"\n=== pelvis ground truth ({len(rows)} samples) ===")

    res = {}
    for name, a, b in (("CONTROL", "control_start", "control_end"),
                       ("GEMINI", "gemini_start", "gemini_end")):
        if a not in marks or b not in marks:
            print(f"{name}: marks missing")
            continue
        r = summarise(name, phase(rows, marks[a], marks[b]))
        if isinstance(r, tuple):
            print(r[0])
            res[name] = r[1:]
        else:
            print(r)

    print()
    if "CONTROL" in res and "GEMINI" in res:
        c_net, _, c_rst, c_jmp = res["CONTROL"]
        g_net, _, g_rst, g_jmp = res["GEMINI"]
        if c_rst or g_rst or c_jmp or g_jmp:
            print("VERDICT: VOID -- this run contains a sim reset or a position "
                  "discontinuity, so distance travelled is not attributable to the "
                  "prompt. Re-run; do not quote these numbers.")
        elif g_net > 0.5 and g_net > 4 * max(c_net, 0.02):
            print(f"VERDICT: the prompt moved the robot. {g_net:.2f} m commanded "
                  f"vs {c_net:.2f} m uncommanded.")
        elif g_net > 0.5:
            print(f"VERDICT: AMBIGUOUS -- moved {g_net:.2f} m under command, but "
                  f"also {c_net:.2f} m with no command. Not attributable.")
        else:
            print(f"VERDICT: NO. Only {g_net:.2f} m under command. The prompt "
                  f"reached the adapter but the robot did not translate.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
