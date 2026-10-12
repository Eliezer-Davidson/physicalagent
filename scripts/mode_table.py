"""Score EVERY mode in the logs at once, one row per mode.

    .venv_sim/bin/python scripts/mode_table.py

Pairs each planner transition from wire_tap.jsonl with the body samples that
fall inside it, so a batch of a dozen modes run from a single prompt comes back
as a table instead of a dozen hand-sliced windows. The wire timestamps are the
only reliable segmentation: body data alone cannot say where one mode ended and
the next began, and a mode that did nothing leaves no seam at all.

Columns are the ones that turned out to discriminate:
  speed / pelvis    the legs
  lift              peak foot clearance
  swing             CYCLIC arm amplitude, median range over 1 s windows -- NOT
                    the whole-bout range, which is dominated by the one-time ramp
                    into the mode's pose (see mode_probe.py)
  stance L/R        fraction of time each foot carries the robot; the limp
                    detector, and invisible to every height-based statistic
  air               samples with ZERO ground contact. Five or more at 50 Hz is a
                    real 0.1 s flight phase; two or three is contact chatter and
                    is reported as `?`, never promoted to flight
  FELL              a non-foot body touching the floor. Pelvis height cannot say
                    this -- the stealth gait crouches to 0.533 m while walking
"""
from __future__ import annotations

import json
import pathlib
import sys

LIVE = pathlib.Path(__file__).resolve().parents[1] / "artifacts" / "live"
HZ = 50.0
FLIGHT_MIN = 5

# What touching the floor MEANS the robot has fallen. Knees and elbows are not
# on this list on purpose: kneel_two_legs puts both knees down and kneel puts one
# down, and the elbow crawl is named after the contact it makes. "Anything that
# is not a foot" flagged all three of those as falls on the run they worked
# perfectly -- the same mistake as a standing-height fall threshold applied to a
# crouch gait, one level further in.
# NOT hands and NOT elbows: crawl (8) goes on hands and knees and elbow_crawl
# (14) is named after the contact it makes, so both were flagged as falls on the
# run where crawl worked perfectly. Only bodies that mean "toppled over".
FALL_BODIES = ("pelvis", "torso", "waist", "shoulder", "head")

# A single-sample pelvis translation larger than this is a teleport, not motion.
# 0.5 m in 0.02 s would be 25 m/s.
RESET_JUMP_M = 0.5


def load():
    tap = [json.loads(x) for x in (LIVE / "wire_tap.jsonl").read_text().splitlines() if x.strip()]
    lines = (LIVE / "mode_probe.jsonl").read_text().splitlines()
    base = json.loads(lines[0])["baseline"]
    # Skip unparseable lines rather than dying on them. A recorder killed
    # mid-write leaves one torn line, and refusing to read 384,000 good samples
    # because of it throws away the whole session's evidence.
    rows, bad = [], 0
    for x in lines[1:]:
        if not x.strip():
            continue
        try:
            rows.append(json.loads(x))
        except ValueError:
            bad += 1
    if bad:
        print(f"note: skipped {bad} torn line(s) of {len(lines) - 1}", file=sys.stderr)
    rows.sort(key=lambda r: r["wall"])
    return tap, base, rows


def spans(tap, rows):
    """Each non-idle mode, from when it went on the wire to the next transition."""
    out = []
    for i, e in enumerate(tap):
        if e["mode"] == 0:
            continue
        end = tap[i + 1]["wall"] if i + 1 < len(tap) else rows[-1]["wall"]
        out.append((e, e["wall"], end))
    return out


def count_resets(win):
    """How many times the simulator teleported the robot back to the origin.

    base_sim.check_fall() resets the episode when the pelvis drops below
    SONIC_FALL_RESET_M, and a reset is a POSITION DISCONTINUITY, not motion.
    Averaged in blindly it produced `elbow_crawl: 91.57 m at 12.04 m/s` and 157
    "airborne" samples -- the robot was mid-teleport with no contacts. Every
    number spanning a reset is fiction, so they are counted and the row is
    marked rather than quietly reported.
    """
    n = 0
    for i in range(1, len(win)):
        a, b = win[i - 1]["pelvis"], win[i]["pelvis"]
        jump = ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5
        if jump > RESET_JUMP_M:
            n += 1
    return n


def score(win, base):
    floor_l, floor_r = base["lfoot"], base["rfoot"]
    dt = win[-1]["wall"] - win[0]["wall"]
    dx = win[-1]["pelvis"][0] - win[0]["pelvis"][0]
    dy = win[-1]["pelvis"][1] - win[0]["pelvis"][1]
    dist = (dx * dx + dy * dy) ** 0.5

    pz = [r["pelvis"][2] for r in win]
    lift = max(max(r["lfoot"] - floor_l, r["rfoot"] - floor_r) for r in win)

    fwd = [max(r["lhand"][0], r["rhand"][0]) for r in win]
    cyc = max(1, int(HZ))
    rr = sorted(max(fwd[i:i + cyc]) - min(fwd[i:i + cyc])
                for i in range(0, max(1, len(fwd) - cyc), max(1, cyc // 4)))
    swing = rr[len(rr) // 2] if rr else 0.0

    have = any(r.get("ground") is not None for r in win)
    st = {}
    air = 0
    fell = []
    if have:
        air = sum(1 for r in win if not r.get("ground"))
        for side, key in (("l", "left_ankle"), ("r", "right_ankle")):
            st[side] = 100.0 * sum(
                any(key in p for p in (r.get("ground") or [])) for r in win) / len(win)
        fell = sorted({p for r in win for p in (r.get("ground") or [])
                       if any(k in p for k in FALL_BODIES)})

    return {"resets": count_resets(win),
            "dt": dt, "dist": dist, "speed": dist / dt if dt else 0.0,
            "pmean": sum(pz) / len(pz), "pmin": min(pz), "pmax": max(pz),
            "lift": lift, "swing": swing, "st": st, "air": air,
            "n": len(win), "fell": fell, "have": have}


def main() -> int:
    tap, base, rows = load()
    print(f"{'mode':<17}{'id':>3} {'s':>5} {'dist':>6} {'m/s':>5} "
          f"{'pelvis mean/min':>16} {'lift':>6} {'swing':>6} "
          f"{'stance L/R':>11} {'air':>5}  flag")
    print("-" * 104)
    for e, t0, t1 in spans(tap, rows):
        win = [r for r in rows if t0 <= r["wall"] <= t1]
        if len(win) < 25:
            print(f"{e['name']:<17}{e['mode']:>3}  -- too few samples "
                  f"({len(win)}); mode may have been superseded immediately")
            continue
        s = score(win, base)
        st = (f"{s['st']['l']:.0f}/{s['st']['r']:.0f}" if s["st"] else "--")
        if not s["have"]:
            air = "n/a"
        elif s["fell"]:
            # A toppling robot loses contact too. Flight during a collapse is
            # not flight, and reporting it as such is how a fall becomes a
            # capability.
            air = "fall"
        elif s["air"] >= FLIGHT_MIN:
            air = f"{s['air']}!"
        elif s["air"] >= 2:
            air = f"{s['air']}?"
        else:
            air = "0"
        flag = ""
        if s["resets"]:
            flag = f"*** {s['resets']} SIM RESETS -- every number in this row is fiction"
        elif s["fell"]:
            flag = "FELL: " + ",".join(x.replace("_link", "") for x in s["fell"][:3])
        elif s["dist"] < 0.05 and s["lift"] < 0.02:
            flag = "NO MOTION"
        print(f"{e['name']:<17}{e['mode']:>3} {s['dt']:5.1f} {s['dist']:6.2f} "
              f"{s['speed']:5.2f} {s['pmean']:8.3f}/{s['pmin']:.3f}   "
              f"{s['lift']:6.3f} {s['swing']:6.3f} {st:>11} {air:>5}  {flag}")
    print("-" * 104)
    print(f"air: '!' = >={FLIGHT_MIN} samples, a real flight phase.  "
          f"'?' = 2-4 samples, contact chatter, NOT flight.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
