"""Record what the robot's body actually did, so a mode can be judged on numbers.

    # once, in the background, for the whole session
    .venv_sim/bin/python scripts/mode_probe.py record

    # after each mode, summarise the window that just went past
    .venv_sim/bin/python scripts/mode_probe.py report --last 8

WHY THIS EXISTS
    23 of SONIC's 27 modes have never been observed on this robot, and the whole
    point of tonight is to stop asserting and start measuring. "Looked like a
    zombie" is not a result. Distance travelled, pelvis height and foot clearance
    are, and they are the three that separate a mode that ran from a mode that
    was silently ignored.

THE FLIGHT TEST, WHICH IS THE ONE THAT MATTERS
    A jump is not "the pelvis went up" -- the pelvis rises on any hard push-off
    while a foot stays planted. A jump is BOTH FEET OFF THE FLOOR AT THE SAME
    INSTANT, so the statistic is `max over time of min(left_z, right_z)`. Taking
    each foot's own maximum instead would score a normal walking stride as
    flight, which is exactly how you talk yourself into a capability you do not
    have.

    Ankle bodies do not sit on the floor, so the floor is calibrated from the
    robot's own standing pose rather than assumed to be zero. `record` writes the
    baseline it saw at start-up into the first line of the log.
"""
from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from physicalagent.robots.sonic_sim_client import SimClient  # noqa: E402
from physicalagent.paths import PA_ARTIFACTS  # noqa: E402

LOG = PA_ARTIFACTS / "live" / "mode_probe.jsonl"
BODIES = ["pelvis", "left_ankle_roll_link", "right_ankle_roll_link",
          "left_wrist_yaw_link", "right_wrist_yaw_link"]
# 50 Hz, matching the control loop. 20 Hz was too coarse for the one measurement
# that matters: a running flight phase lasts ~0.1 s, which is TWO samples at
# 20 Hz -- exactly the threshold, so a real flight and a sampling artifact were
# indistinguishable. At 50 Hz the same phase is five samples.
HZ = 50.0
FALLEN_PELVIS_M = 0.55

# A foot is "off the floor" once it is this far above where it rested while the
# robot stood still. Ankle-body height is not sole height, so the number is a
# margin over a measured baseline, never an absolute.
CLEARANCE_M = 0.02


def sample(client: SimClient) -> dict | None:
    try:
        r = client.body_pose(BODIES)
    except Exception:
        return None
    if not r.get("ok"):
        return None
    b = r["bodies"]
    return {
        "t": r.get("sim_time_s"),
        "wall": time.time(),
        "pelvis": b["pelvis"]["world_m"],
        "lfoot": b["left_ankle_roll_link"]["world_m"][2],
        "rfoot": b["right_ankle_roll_link"]["world_m"][2],
        "lhand": b["left_wrist_yaw_link"]["pelvis_m"],
        "rhand": b["right_wrist_yaw_link"]["pelvis_m"],
        "ground": r.get("ground_contact"),
    }


def lock_or_die(path: pathlib.Path, what: str):
    """Refuse to start if another instance already owns this log.

    Two recorders writing the same file interleave mid-line, and the corruption
    shows up much later as a JSONDecodeError on a file that looks fine -- after
    a restart had already left a second copy running. An exclusive flock makes
    the second one say so and exit instead.
    """
    import fcntl
    path.parent.mkdir(parents=True, exist_ok=True)
    fh = open(str(path) + ".lock", "w")
    try:
        fcntl.flock(fh, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        print(f"another {what} already owns {path} -- refusing to start a second",
              file=sys.stderr)
        raise SystemExit(3)
    return fh          # held for the process lifetime; released on exit


def record() -> int:
    _lock = lock_or_die(LOG, "recorder")  # noqa: F841 -- must stay alive
    client = SimClient()
    LOG.parent.mkdir(parents=True, exist_ok=True)

    first = None
    for _ in range(40):
        first = sample(client)
        if first:
            break
        time.sleep(0.25)
    if first is None:
        print("no sidecar -- nothing to record", file=sys.stderr)
        return 2

    with LOG.open("w") as fh:
        fh.write(json.dumps({"baseline": first}) + "\n")
        fh.flush()
        print(f"baseline: pelvis {first['pelvis'][2]:.3f} m, "
              f"feet {first['lfoot']:.3f}/{first['rfoot']:.3f} m -> {LOG}")
        period = 1.0 / HZ
        while True:
            t0 = time.monotonic()
            row = sample(client)
            if row:
                fh.write(json.dumps(row) + "\n")
                fh.flush()
            time.sleep(max(0.0, period - (time.monotonic() - t0)))


# A sample counts as motion if the robot is translating or has picked a foot up.
# Both are needed: a mode can march on the spot, and a mode can slide forward
# with no foot clearance at all.
MOVING_M_S = 0.03
MOVING_LIFT_M = 0.015
SPEED_WINDOW_S = 0.2    # baseline for the speed estimate; NOT the sample period
BOUT_GAP_S = 1.5        # quiet longer than this ends the bout
BOUT_PAD_S = 0.7        # keep a little stillness either side, to see entry and exit


def find_last_bout(rows: list[dict], floor_l: float, floor_r: float) -> list[dict]:
    """The last stretch of time the robot was actually doing something.

    A fixed `--last N` window is only right if the human reports promptly, and
    the first time this was used the walk had happened a minute earlier and the
    window showed a flat zero -- which reads exactly like a mode that did
    nothing. Detecting the bout removes the operator from the measurement.
    """
    # Speed is measured over SPEED_WINDOW_S, never between adjacent samples.
    # Adjacent-sample speed is (position noise / dt), and dt shrank when the
    # sampler went from 20 Hz to 50 Hz: sub-millimetre jitter on a stationary
    # robot then reads as 0.05 m/s and the detector locks onto a "bout" of pure
    # noise AFTER the real motion. That is how a 4.85 m stealth walk got
    # reported as 0.000 m travelled -- a false negative that looks exactly like
    # a mode that does nothing.
    span = max(1, int(SPEED_WINDOW_S * HZ))
    moving = [False] * len(rows)
    for i in range(span, len(rows)):
        j = i - span
        dt = rows[i]["wall"] - rows[j]["wall"]
        if dt <= 0:
            continue
        dx = rows[i]["pelvis"][0] - rows[j]["pelvis"][0]
        dy = rows[i]["pelvis"][1] - rows[j]["pelvis"][1]
        speed = (dx * dx + dy * dy) ** 0.5 / dt
        lift = max(rows[i]["lfoot"] - floor_l, rows[i]["rfoot"] - floor_r)
        moving[i] = speed > MOVING_M_S or lift > MOVING_LIFT_M

    end = next((i for i in range(len(rows) - 1, -1, -1) if moving[i]), None)
    if end is None:
        return []
    start = end
    for i in range(end, 0, -1):
        if moving[i]:
            start = i
        elif rows[start]["wall"] - rows[i]["wall"] > BOUT_GAP_S:
            break
    lo, hi = rows[start]["wall"] - BOUT_PAD_S, rows[end]["wall"] + BOUT_PAD_S
    return [r for r in rows if lo <= r["wall"] <= hi]


def report(last_s: float | None) -> int:
    if not LOG.exists():
        print("no log -- is `mode_probe.py record` running?", file=sys.stderr)
        return 2
    lines = LOG.read_text().splitlines()
    if len(lines) < 2:
        print("log has no samples yet", file=sys.stderr)
        return 2

    baseline = json.loads(lines[0])["baseline"]
    rows = [json.loads(x) for x in lines[1:] if x.strip()]

    if last_s is None:
        win = find_last_bout(rows, baseline["lfoot"], baseline["rfoot"])
        if not win:
            print("THE ROBOT NEVER MOVED in this whole recording.", file=sys.stderr)
            return 1
        ago = rows[-1]["wall"] - win[-1]["wall"]
        print(f"last motion bout, ended {ago:.0f}s ago")
    else:
        cutoff = rows[-1]["wall"] - last_s
        win = [r for r in rows if r["wall"] >= cutoff]
    if len(win) < 5:
        print(f"only {len(win)} samples in the window", file=sys.stderr)
        return 2

    floor_l, floor_r = baseline["lfoot"], baseline["rfoot"]
    x0, y0 = win[0]["pelvis"][0], win[0]["pelvis"][1]
    x1, y1 = win[-1]["pelvis"][0], win[-1]["pelvis"][1]
    dist = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
    span = win[-1]["wall"] - win[0]["wall"]

    pz = [r["pelvis"][2] for r in win]
    both = [min(r["lfoot"] - floor_l, r["rfoot"] - floor_r) for r in win]
    lift_l = max(r["lfoot"] - floor_l for r in win)
    lift_r = max(r["rfoot"] - floor_r for r in win)
    flight = max(both)

    # Flight is decided by CONTACT, not by height. See the sidecar's
    # _ground_contact: the ankle body is the joint, not the sole, so a lifted
    # heel raises it several cm with the toe still planted. Height-based
    # detection called a deep crouch a flight phase.
    have_contact = any(r.get("ground") is not None for r in win)
    if have_contact:
        airborne = [not r.get("ground") for r in win]
        flight_frames = sum(airborne)
    else:
        flight_frames = sum(1 for v in both if v > CLEARANCE_M)

    fwd = [max(r["lhand"][0], r["rhand"][0]) for r in win]
    hands_fwd = max(fwd)
    hands_up = max(max(r["lhand"][2], r["rhand"][2]) for r in win)
    # Arm SWING, not arm position. Peak forward reach says almost nothing about
    # upper-body style -- every gait peaks between +0.24 and +0.40 simply from
    # swinging. Amplitude is what separates them: zombie_walk is "stiff-armed",
    # meaning the arms do not swing rather than that they are held out in front.
    #
    # Measured as the MEDIAN range over 1 s windows, not the range over the whole
    # bout. Total range also contains the one-time ramp from neutral into the
    # mode's pose, which for a mode that parks the arms somewhere new is larger
    # than the gait swing it is supposed to measure -- gun_walk scored the widest
    # "swing" of any mode that way, purely from raising the arms once.
    cyc = max(1, int(1.0 * HZ))
    ranges = [max(fwd[i:i + cyc]) - min(fwd[i:i + cyc])
              for i in range(0, max(1, len(fwd) - cyc), max(1, cyc // 4))]
    ranges.sort()
    hands_swing = ranges[len(ranges) // 2] if ranges else 0.0

    # Stance fraction per foot -- how long each leg carries the robot. This is
    # what separates a limp from a walk, and peak foot lift does NOT: a limp and
    # a stealth gait have similar lift asymmetry while their stance split is
    # completely different. Only measurable now that contacts are reported.
    stance = {}
    if have_contact:
        for side, key in (("left", "left_ankle"), ("right", "right_ankle")):
            down = [any(key in p for p in (r.get("ground") or [])) for r in win]
            edges = sum(1 for i in range(1, len(down)) if down[i] and not down[i - 1])
            stance[side] = (100.0 * sum(down) / len(down), edges)

    print(f"window {span:.1f}s, {len(win)} samples")
    print(f"  travelled        {dist:.3f} m   ({dist / span:.2f} m/s mean)")
    print(f"  pelvis z         mean {sum(pz) / len(pz):.3f}  "
          f"min {min(pz):.3f}  max {max(pz):.3f}   (stood at {baseline['pelvis'][2]:.3f})")
    print(f"  foot lift        left {lift_l:.3f} m   right {lift_r:.3f} m")
    print(f"  hands vs pelvis  max forward {hands_fwd:+.3f} m   max up {hands_up:+.3f} m"
          f"   swing {hands_swing:.3f} m")
    if stance:
        l, r_ = stance["left"], stance["right"]
        print(f"  stance           left {l[0]:.1f}% ({l[1]} steps)   "
              f"right {r_[0]:.1f}% ({r_[1]} steps)   "
              f"asymmetry {abs(l[0] - r_[0]):.1f} pts")
    if have_contact:
        print(f"  NO GROUND CONTACT {flight_frames} of {len(win)} samples "
              f"(MuJoCo contacts; ankle peak {flight:+.3f} m)")
    else:
        print(f"  BOTH FEET CLEAR  {flight:+.3f} m peak, {flight_frames} of {len(win)} "
              f"samples above {CLEARANCE_M:.2f} m   [height-inferred, no contact data]")

    # A fall is a fall only if something that is not a foot is on the floor.
    # Pelvis height alone cannot say: the stealth gait crouches to 0.533 m, well
    # under any sensible standing threshold, while walking perfectly.
    down = sorted({p for r in win for p in (r.get("ground") or [])
                   if "ankle" not in p and "foot" not in p})
    if down:
        print(f"  *** NON-FOOT GROUND CONTACT: {', '.join(down[:6])} "
              f"(pelvis min {min(pz):.3f} m) ***")
    elif min(pz) < FALLEN_PELVIS_M and not have_contact:
        print(f"  pelvis reached {min(pz):.3f} m -- low, but no contact data to "
              f"say whether that is a crouch or a fall")
    # A real flight phase lasts ~0.1 s, so at 50 Hz it is about five samples.
    # Two or three is the length of a sampling artifact and is reported as
    # inconclusive rather than promoted to a capability.
    flight_s = flight_frames / HZ
    if flight_frames >= 5:
        verdict = f"FLIGHT -- both feet left the floor for {flight_s:.2f}s"
    elif flight_frames >= 2:
        verdict = (f"INCONCLUSIVE -- {flight_frames} samples ({flight_s:.2f}s) with "
                   f"both feet clear. Too short to call; re-run before believing it")
    else:
        verdict = "no flight -- at least one foot was down at all times"
    print("  VERDICT: " + verdict)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("record")
    rep = sub.add_parser("report")
    rep.add_argument("--last", type=float, default=None,
                     help="seconds to summarise; default finds the last motion bout")
    args = ap.parse_args()
    return record() if args.cmd == "record" else report(args.last)


if __name__ == "__main__":
    sys.exit(main())
