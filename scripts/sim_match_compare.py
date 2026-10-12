#!/usr/bin/env python3
"""Compare two hw_recorder captures segment-by-segment: sim-to-real as a number.

WHY

    After 2026-08-21 the physical G1 is gone. The only way a simulator claim
    stays falsifiable is to run the same battery script against the MuJoCo G1,
    record it with the same hw_recorder, and diff the two captures. This tool
    is that diff: it joins each side's verb log (mark events bracket named
    segments) against its rt/lowstate stream and reports per-segment,
    per-joint RMSE/correlation -- how far the simulator is from the robot.

TIER: below LOCKED -- this never opens DDS at all. It reads recordings.

STREAMING, NOT LISTS

    Real captures are ~3.6M records (~1 GB) per stream. hw_timebase_probe's
    analyse() once built a list out of read_stream() and the mandatory
    end-of-day replay check stopped finishing. Same law here: every stream is
    read exactly ONCE, front to back, and only the records inside a named
    segment are retained (downsampled) -- everything else is looked at and
    dropped.

STDLIB ONLY

    No numpy, no matplotlib. This must run off-robot on any machine with a bare
    python3, against files recorded on a robot nobody has any more.

USAGE

    python3 scripts/sim_match_compare.py \
        --a artifacts/hw/20260819-...-battery --a-log hw_verbs.jsonl --a-label hw \
        --b artifacts/sim/20260901-...-battery --b-log sim_verbs.jsonl --b-label sim \
        --out artifacts/sim_match/day1 [--normalize-time] [--grid-hz 100]
"""
from __future__ import annotations

import argparse
import array
import bisect
import json
import math
import pathlib
import sys

# read_stream() is the corpus contract: reconstructing records from .bin +
# .schema.json belongs to hw_recorder alone, so a format change there cannot
# silently diverge from what this tool expects.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from hw_recorder import read_stream  # noqa: E402


# ------------------------------------------------------------------ verb log

def parse_verb_log(path):
    """{"meta": dict|None, "segments": [{"label", "t_start", "t_end"}]}.

    A segment is opened by a mark and closed by the next mark; the final mark
    closes the last segment and opens nothing (anything after it -- shutdown,
    damping -- is deliberately outside the comparison). Unknown kinds are
    skipped: the battery runner is allowed to grow new event types without
    breaking every old comparison script.
    """
    meta = None
    marks = []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                ev = json.loads(line)
            except ValueError:
                continue                      # half-written trailing line
            kind = ev.get("kind")
            if kind == "meta" and meta is None:
                meta = ev
            elif kind == "mark":
                marks.append(ev)
    segments = []
    for opener, closer in zip(marks, marks[1:]):
        segments.append({"label": opener["label"],
                         "t_start": float(opener["t_wall"]),
                         "t_end": float(closer["t_wall"])})
    return {"meta": meta, "segments": segments}


def meta_warnings(meta_a, meta_b):
    """Cross-side sanity of the two verb logs' meta lines. A sha mismatch
    means the two batteries ran DIFFERENT scripts -- the comparison is still
    computable segment-by-segment, so warn loudly and continue."""
    warnings = []
    sha_a = (meta_a or {}).get("sha256")
    sha_b = (meta_b or {}).get("sha256")
    if sha_a != sha_b:
        warnings.append(
            f"verb-log meta.sha256 differs (a={sha_a!r} b={sha_b!r}): the two "
            f"sides ran DIFFERENT battery scripts; matching segment labels "
            f"are still compared, but treat cross-segment conclusions with "
            f"suspicion")
    return warnings


def match_segments(segs_a, segs_b):
    """(pairs, only_a_labels, only_b_labels) via longest common subsequence
    of the label sequences.

    LCS rather than zip-by-index: one side aborting early, or an operator
    inserting a recovery segment mid-battery, must not misalign every
    segment after it. In the intended case both sequences are identical and
    LCS degenerates to the full pairing.
    """
    la = [s["label"] for s in segs_a]
    lb = [s["label"] for s in segs_b]
    n, m = len(la), len(lb)
    # Classic O(n*m) table; battery scripts have tens of segments, not 3.6M.
    dp = [[0] * (m + 1) for _ in range(n + 1)]
    for i in range(n - 1, -1, -1):
        for j in range(m - 1, -1, -1):
            if la[i] == lb[j]:
                dp[i][j] = dp[i + 1][j + 1] + 1
            else:
                dp[i][j] = max(dp[i + 1][j], dp[i][j + 1])
    pairs = []
    matched_a, matched_b = set(), set()
    i = j = 0
    while i < n and j < m:
        if la[i] == lb[j]:
            pairs.append((segs_a[i], segs_b[j]))
            matched_a.add(i); matched_b.add(j)
            i += 1; j += 1
        elif dp[i + 1][j] >= dp[i][j + 1]:
            i += 1
        else:
            j += 1
    only_a = [la[i] for i in range(n) if i not in matched_a]
    only_b = [lb[j] for j in range(m) if j not in matched_b]
    return pairs, only_a, only_b


# ------------------------------------------------------------ stream slicing

# In-segment samples are kept at no more than INGEST_OVERSAMPLE x the output
# grid rate. The grid is where every metric is computed, so keeping ~4 samples
# per grid cell loses nothing measurable while bounding memory by grid_hz and
# segment length instead of by the capture's native rate (1052 Hz on hardware,
# and whatever a future simulator decides to publish at).
INGEST_OVERSAMPLE = 4.0


def _motor_prefix(record):
    """Contiguous m{i}_q prefix length. The sidecar's n_motor is authoritative
    for files, but the slicer works on any record iterable, so it reads the
    count off the first record instead of trusting a caller to pass it."""
    n = 0
    while f"m{n}_q" in record:
        n += 1
    return n


def slice_stream(records, segments, grid_hz=100.0):
    """Route a time-ordered record iterable into time-ordered segments.

    ONE forward pass, no rewind, no len(): `records` is typically
    hw_recorder.read_stream(), a generator over ~3.6M records, and
    materialising it is the exact bug that once made hw_timebase_probe
    unusable on real captures. A simple cursor works because both the records
    and the segments are time-ordered.

    Returns ([seg_data per segment], n_motor). seg_data holds only
    downsampled in-segment joint samples: t (t_rel seconds), and per-motor
    q/dq/tau_est arrays.
    """
    min_dt = 1.0 / (INGEST_OVERSAMPLE * float(grid_hz))
    n_motor = None
    out = []
    for seg in segments:
        out.append({"label": seg["label"], "t": array.array("d"),
                    "q": [], "dq": [], "tau": [], "records_seen": 0,
                    "_last_kept": None})
    cursor = 0
    for r in records:
        if cursor >= len(segments):
            break                    # past the final mark: nothing left to fill
        t = r["t_wall"]
        while cursor < len(segments) and t >= segments[cursor]["t_end"]:
            cursor += 1
        if cursor >= len(segments):
            break
        seg = segments[cursor]
        if t < seg["t_start"]:
            continue                 # between segments / before the first mark
        if n_motor is None:
            n_motor = _motor_prefix(r)
        d = out[cursor]
        d["records_seen"] += 1
        if not d["q"]:
            for _ in range(n_motor):
                d["q"].append(array.array("d"))
                d["dq"].append(array.array("d"))
                d["tau"].append(array.array("d"))
        t_rel = t - seg["t_start"]
        if d["_last_kept"] is not None and (t_rel - d["_last_kept"]) < min_dt:
            continue                 # downsample: memory scales with the grid
        d["_last_kept"] = t_rel
        d["t"].append(t_rel)
        for j in range(n_motor):
            d["q"][j].append(float(r.get(f"m{j}_q", 0.0)))
            d["dq"][j].append(float(r.get(f"m{j}_dq", 0.0)))
            d["tau"][j].append(float(r.get(f"m{j}_tau_est", 0.0)))
    for d in out:
        d.pop("_last_kept", None)
    return out, n_motor


# ----------------------------------------------------------------- metrics

# Motor index -> joint name, in LowState_.motor_state order. Copied from
# physicalagent/robots/sonic_telemetry.py (JOINT_NAMES) rather than imported:
# this tool must stay runnable from a bare checkout of just scripts/ next to a
# capture, with no package on the path. Same order as JOINT_KP in the wbc yaml.
G1_JOINT_NAMES = (
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee",
    "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee",
    "right_ankle_pitch", "right_ankle_roll",
    "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
    "left_elbow", "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
    "right_elbow", "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
)


def joint_name(i):
    """A motor beyond the known table keeps its index name -- inventing a
    name for a motor this table has never seen would be a lie with a label."""
    return G1_JOINT_NAMES[i] if i < len(G1_JOINT_NAMES) else f"m{i}"


def _interp(ts, vs, t):
    """Linear interpolation with clamped ends (numpy.interp semantics).
    Clamping, not extrapolating: a grid point microseconds past the last
    sample must not be answered with an invented slope."""
    if t <= ts[0]:
        return vs[0]
    if t >= ts[-1]:
        return vs[-1]
    hi = bisect.bisect_right(ts, t)
    lo = hi - 1
    span = ts[hi] - ts[lo]
    if span <= 0.0:
        return vs[lo]
    w = (t - ts[lo]) / span
    return vs[lo] * (1.0 - w) + vs[hi] * w


def _pearson(xs, ys):
    """Pearson r, or None when either side is (numerically) constant. An idle
    joint has no signal to correlate; None is the honest answer and it must
    never surface as a divide-by-zero."""
    n = len(xs)
    if n < 2:
        return None
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    syy = sum((y - my) ** 2 for y in ys)
    if sxx <= 1e-24 or syy <= 1e-24:
        return None
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    return sxy / math.sqrt(sxx * syy)


def compare_segment(da, db, dur_a, dur_b, grid_hz, normalize, n_motor,
                    joint_names=None, label_a="a", label_b="b"):
    """Per-joint error between the two sides of one segment.

    Default mode compares wall-clock aligned time (t_rel from each side's own
    opening mark). --normalize-time instead maps each side onto u in [0,1],
    which answers "same MOTION?" even when one side ran slower -- at the cost
    of hiding exactly that slowdown, which is why it is not the default.
    """
    warnings = []
    label = da.get("label", "?")
    dmax = max(dur_a, dur_b)
    result = {"label": label,
              f"duration_{label_a}_s": dur_a, f"duration_{label_b}_s": dur_b,
              "duration_ratio": (dur_b / dur_a) if dur_a > 0 else None,
              "joints": {}, "mean_rmse_q": None, "worst_joint": None,
              "warnings": warnings}
    if dmax > 0 and abs(dur_a - dur_b) > 0.10 * dmax and not normalize:
        warnings.append(
            f"segment {label!r}: time-warp, durations differ by >10% "
            f"({label_a}={dur_a:.2f}s {label_b}={dur_b:.2f}s) -- the sim ran "
            f"at a different rate; consider --normalize-time")
    if not len(da["t"]) or not len(db["t"]):
        empty = [n for n, d in ((label_a, da), (label_b, db)) if not len(d["t"])]
        warnings.append(
            f"segment {label!r}: no rt/lowstate records inside the segment on "
            f"side(s) {empty} -- nothing to compare (was the recorder running "
            f"during this verb?)")
        return result

    if normalize:
        # u-grid: N points over [0,1], dense enough for the LONGER side.
        n_pts = max(2, int(math.ceil(grid_hz * dmax)))
        grid_a = [(k / (n_pts - 1)) * dur_a for k in range(n_pts)]
        grid_b = [(k / (n_pts - 1)) * dur_b for k in range(n_pts)]
    else:
        dur = min(dur_a, dur_b)
        n_pts = max(2, int(dur * grid_hz))
        grid_a = grid_b = [k / grid_hz for k in range(n_pts)]

    rmses = []
    for j in range(n_motor):
        name = joint_names[j] if joint_names else joint_name(j)
        qa = [_interp(da["t"], da["q"][j], t) for t in grid_a]
        qb = [_interp(db["t"], db["q"][j], t) for t in grid_b]
        dqa = [_interp(da["t"], da["dq"][j], t) for t in grid_a]
        dqb = [_interp(db["t"], db["dq"][j], t) for t in grid_b]
        dq_err = [x - y for x, y in zip(qa, qb)]
        rmse_q = math.sqrt(sum(e * e for e in dq_err) / len(dq_err))
        rmses.append((rmse_q, name))
        result["joints"][name] = {
            "motor": j,
            "rmse_q": rmse_q,
            "corr_q": _pearson(qa, qb),
            "rmse_dq": math.sqrt(sum((x - y) ** 2
                                     for x, y in zip(dqa, dqb)) / len(dqa)),
            "max_abs_delta_q": max(abs(e) for e in dq_err),
        }
    result["mean_rmse_q"] = sum(r for r, _ in rmses) / len(rmses)
    worst = max(rmses)
    result["worst_joint"] = {"name": worst[1], "rmse_q": worst[0]}
    return result


# ----------------------------------------------------------------- thermal

def slice_thermal(records, segments):
    """Hottest motor per segment, single pass, O(1) memory per segment.

    Temperature is max(temp_winding, temp_driver): on 2026-08-19 SONIC's
    overheat warning fired on the max of the two, so reporting only one
    sensor could hide the very trigger this stream exists to answer.
    """
    out = [None] * len(segments)
    cursor = 0
    n_motor = None
    for r in records:
        if cursor >= len(segments):
            break
        t = r["t_wall"]
        while cursor < len(segments) and t >= segments[cursor]["t_end"]:
            cursor += 1
        if cursor >= len(segments) or t < segments[cursor]["t_start"]:
            continue
        if n_motor is None:
            n_motor = 0
            while f"m{n_motor}_temp_winding" in r:
                n_motor += 1
        best = out[cursor]
        for j in range(n_motor):
            temp = max(r[f"m{j}_temp_winding"], r[f"m{j}_temp_driver"])
            if best is None or temp > best["max_temp_c"]:
                best = {"hottest_motor": joint_name(j), "max_temp_c": temp}
        out[cursor] = best
    return out


# -------------------------------------------------------------------- svg

# House SVG style follows scripts/lidar_capture.py: dark ground, monospace
# text, hand-rolled markup, no plotting library anywhere near this repo's
# analyzers.
_SVG_W, _SVG_H = 860, 420
_SVG_ML, _SVG_MR, _SVG_MT, _SVG_MB = 64, 16, 46, 42
_COLOR_A, _COLOR_B = "#4da3ff", "#ff9d4d"


def _polyline(ts, qs, x0, y0, w, h, t_max, q_min, q_max, color):
    q_span = (q_max - q_min) or 1.0
    t_span = t_max or 1.0
    # ~2 points per output pixel is visually lossless; more is just bytes.
    step = max(1, len(ts) // (2 * w))
    pts = []
    for i in range(0, len(ts), step):
        x = x0 + (ts[i] / t_span) * w
        y = y0 + h - ((qs[i] - q_min) / q_span) * h
        pts.append(f"{x:.1f},{y:.1f}")
    return (f'<polyline fill="none" stroke="{color}" stroke-width="1.6" '
            f'points="{" ".join(pts)}"/>')


def render_overlay_svg(seg_label, jname, ta, qa, tb, qb, rmse,
                       label_a, label_b):
    """Both sides' q vs t_rel as two polylines with a legend, so a human can
    see WHERE in the motion the error lives -- an RMSE alone cannot say
    whether the sim lags, overshoots, or drifts."""
    x0, y0 = _SVG_ML, _SVG_MT
    w = _SVG_W - _SVG_ML - _SVG_MR
    h = _SVG_H - _SVG_MT - _SVG_MB
    all_q = list(qa) + list(qb) or [0.0]
    q_min, q_max = min(all_q), max(all_q)
    if q_max - q_min < 1e-9:
        q_min, q_max = q_min - 0.5, q_max + 0.5
    t_max = max(list(ta)[-1] if len(ta) else 0.0,
                list(tb)[-1] if len(tb) else 0.0)
    title = (f"{seg_label} / {jname} -- RMSE(q) {rmse:.4f} rad"
             if rmse is not None else f"{seg_label} / {jname}")
    grid = []
    for k in range(5):
        gy = y0 + h * k / 4
        qv = q_max - (q_max - q_min) * k / 4
        grid.append(f'<line x1="{x0}" y1="{gy:.1f}" x2="{x0 + w}" '
                    f'y2="{gy:.1f}" stroke="#2a3a4a" stroke-width="1" '
                    f'stroke-dasharray="3,4"/>')
        grid.append(f'<text x="{x0 - 6}" y="{gy + 4:.1f}" fill="#5b7a99" '
                    f'font-size="11" font-family="monospace" '
                    f'text-anchor="end">{qv:.2f}</text>')
    for k in range(5):
        gx = x0 + w * k / 4
        tv = t_max * k / 4
        grid.append(f'<text x="{gx:.1f}" y="{y0 + h + 16}" fill="#5b7a99" '
                    f'font-size="11" font-family="monospace" '
                    f'text-anchor="middle">{tv:.1f}</text>')
    return f'''<svg xmlns="http://www.w3.org/2000/svg" width="{_SVG_W}" height="{_SVG_H}" viewBox="0 0 {_SVG_W} {_SVG_H}">
<rect width="{_SVG_W}" height="{_SVG_H}" fill="#0d1520"/>
<text x="14" y="24" fill="#c8dbef" font-size="15" font-family="monospace">{title}</text>
{''.join(grid)}
<rect x="{x0}" y="{y0}" width="{w}" height="{h}" fill="none" stroke="#3d5a7a" stroke-width="1"/>
{_polyline(ta, qa, x0, y0, w, h, t_max, q_min, q_max, _COLOR_A)}
{_polyline(tb, qb, x0, y0, w, h, t_max, q_min, q_max, _COLOR_B)}
<rect x="{x0 + w - 150}" y="{y0 + 8}" width="140" height="40" fill="#0d1520" opacity="0.75"/>
<line x1="{x0 + w - 142}" y1="{y0 + 20}" x2="{x0 + w - 114}" y2="{y0 + 20}" stroke="{_COLOR_A}" stroke-width="2"/>
<text x="{x0 + w - 108}" y="{y0 + 24}" fill="#c8dbef" font-size="12" font-family="monospace">{label_a}</text>
<line x1="{x0 + w - 142}" y1="{y0 + 38}" x2="{x0 + w - 114}" y2="{y0 + 38}" stroke="{_COLOR_B}" stroke-width="2"/>
<text x="{x0 + w - 108}" y="{y0 + 42}" fill="#c8dbef" font-size="12" font-family="monospace">{label_b}</text>
<text x="{x0 + w / 2:.0f}" y="{_SVG_H - 8}" fill="#7da9d4" font-size="12" font-family="monospace" text-anchor="middle">t_rel [s]</text>
<text x="16" y="{y0 + h / 2:.0f}" fill="#7da9d4" font-size="12" font-family="monospace" transform="rotate(-90 16 {y0 + h / 2:.0f})" text-anchor="middle">q [rad]</text>
</svg>'''


def _safe_name(s):
    return "".join(c if c.isalnum() or c in "-_" else "_" for c in s)


# ------------------------------------------------------------------- main

def _find_stream(cap_dir, name):
    """(bin_path, schema_path) or (None, reason). A zero-byte .bin is treated
    as MISSING, because this repo has real ones: on 2026-08-19 a dead robot
    link let DDS bind to Wi-Fi and write six zero-byte files that look like a
    capture. A file with no records is a claim, not data."""
    binp = pathlib.Path(cap_dir) / f"{name}.bin"
    schema = pathlib.Path(cap_dir) / f"{name}.schema.json"
    if not binp.exists():
        return None, f"{binp} does not exist -- was this stream recorded?"
    if binp.stat().st_size == 0:
        return None, (f"{binp} is zero-byte -- treated as missing. A "
                      f"zero-byte capture is the dead-link signature "
                      f"(recorder bound to the wrong interface), not a "
                      f"quiet robot.")
    if not schema.exists():
        return None, (f"{schema} missing -- the .bin is unreadable without "
                      f"its sidecar (the sidecar IS the format)")
    return binp, schema


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Compare two hw_recorder captures segment-by-segment "
                    "using their verb logs; report per-joint sim-vs-real "
                    "error.")
    ap.add_argument("--a", required=True, help="first capture dir (e.g. hw)")
    ap.add_argument("--a-log", required=True, help="first verb log (JSONL)")
    ap.add_argument("--a-label", default="a")
    ap.add_argument("--b", required=True, help="second capture dir (e.g. sim)")
    ap.add_argument("--b-log", required=True, help="second verb log (JSONL)")
    ap.add_argument("--b-label", default="b")
    ap.add_argument("--out", required=True, help="report directory")
    ap.add_argument("--normalize-time", action="store_true",
                    help="map each segment to u in [0,1]: compares the "
                         "MOTION even when one side ran slower, at the cost "
                         "of hiding that slowdown")
    ap.add_argument("--grid-hz", type=float, default=100.0)
    ap.add_argument("--top-joints", type=int, default=6,
                    help="worst-N joints per segment to plot")
    args = ap.parse_args(argv)

    warnings = []

    def warn(msg):
        warnings.append(msg)
        print(f"WARNING: {msg}", file=sys.stderr)

    la, lb = args.a_label, args.b_label
    log_a = parse_verb_log(args.a_log)
    log_b = parse_verb_log(args.b_log)
    for w in meta_warnings(log_a["meta"], log_b["meta"]):
        warn(w)

    pairs, only_a, only_b = match_segments(log_a["segments"],
                                           log_b["segments"])
    if only_a:
        warn(f"segments only in {la}: {only_a} -- present on one side only, "
             f"not compared")
    if only_b:
        warn(f"segments only in {lb}: {only_b} -- present on one side only, "
             f"not compared")
    if not pairs:
        print(f"ERROR: zero common segments between the two verb logs "
              f"({la}: {[s['label'] for s in log_a['segments']]}, "
              f"{lb}: {[s['label'] for s in log_b['segments']]}) -- nothing "
              f"to compare. Did both runs use the same battery script?",
              file=sys.stderr)
        return 2

    sides = {}
    for tag, cap in ((la, args.a), (lb, args.b)):
        binp, schema = _find_stream(cap, "rt_lowstate")
        if binp is None:
            print(f"ERROR: side {tag!r}: {schema}", file=sys.stderr)
            return 2
        sides[tag] = (binp, schema)

    out_dir = pathlib.Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    # ONE pass per capture: only the paired (= to-be-compared) segments are
    # sliced, in time order, and only downsampled in-segment data survives.
    segs_a = [pa for pa, _ in pairs]
    segs_b = [pb for _, pb in pairs]
    data_a, n_a = slice_stream(read_stream(*sides[la]), segs_a, args.grid_hz)
    data_b, n_b = slice_stream(read_stream(*sides[lb]), segs_b, args.grid_hz)
    if n_a is None or n_b is None:
        empty = [t for t, n in ((la, n_a), (lb, n_b)) if n is None]
        print(f"ERROR: side(s) {empty}: rt_lowstate has no records inside "
              f"any common segment -- the recorder and the battery did not "
              f"overlap in time.", file=sys.stderr)
        return 2
    n_motor = min(n_a, n_b)
    if n_a != n_b:
        warn(f"motor counts differ ({la}={n_a} {lb}={n_b}) -- comparing the "
             f"common prefix of {n_motor} motors (normal for a sim that "
             f"models fewer joints)")

    # Thermal is optional per side: hardware has it, the MuJoCo capture will
    # not. Both present -> report; otherwise one info line and move on.
    th_a = _find_stream(args.a, "rt_lowstate__thermal")
    th_b = _find_stream(args.b, "rt_lowstate__thermal")
    thermal_a = thermal_b = None
    if th_a[0] is not None and th_b[0] is not None:
        thermal_a = slice_thermal(read_stream(*th_a), segs_a)
        thermal_b = slice_thermal(read_stream(*th_b), segs_b)
    else:
        lacking = [t for t, th in ((la, th_a), (lb, th_b)) if th[0] is None]
        print(f"[info] no thermal stream on side(s) {lacking} (normal for "
              f"sim) -- skipping temperature comparison")

    segments_report = {}
    seg_count = {}
    for idx, (pa, pb) in enumerate(pairs):
        dur_a = pa["t_end"] - pa["t_start"]
        dur_b = pb["t_end"] - pb["t_start"]
        m = compare_segment(data_a[idx], data_b[idx], dur_a, dur_b,
                            grid_hz=args.grid_hz,
                            normalize=args.normalize_time,
                            n_motor=n_motor, label_a=la, label_b=lb)
        for w in m["warnings"]:
            warn(w)
        # A battery may repeat a label ("stand" between every verb); keys in
        # report.json must stay unique or the second run silently eats the
        # first.
        label = pa["label"]
        seg_count[label] = seg_count.get(label, 0) + 1
        key = label if seg_count[label] == 1 else f"{label}#{seg_count[label]}"
        if thermal_a is not None:
            th = {}
            if thermal_a[idx]:
                th[la] = thermal_a[idx]
            if thermal_b[idx]:
                th[lb] = thermal_b[idx]
            if th:
                m["thermal"] = th
        segments_report[key] = m

        # Overlay plots for the worst joints: the picture is what tells a
        # human WHICH failure mode the number is.
        ranked = sorted(((jm["rmse_q"], name, jm["motor"])
                         for name, jm in m["joints"].items()), reverse=True)
        for rmse_q, name, j in ranked[:max(0, args.top_joints)]:
            svg = render_overlay_svg(
                key, name, data_a[idx]["t"], data_a[idx]["q"][j],
                data_b[idx]["t"], data_b[idx]["q"][j], rmse_q, la, lb)
            (out_dir / f"seg_{_safe_name(key)}_{_safe_name(name)}.svg"
             ).write_text(svg)

    report = {
        "tool": "sim_match_compare",
        "a": {"capture": str(args.a), "log": str(args.a_log), "label": la,
              "n_motor": n_a},
        "b": {"capture": str(args.b), "log": str(args.b_log), "label": lb,
              "n_motor": n_b},
        "grid_hz": args.grid_hz,
        "normalize_time": args.normalize_time,
        "n_motor_compared": n_motor,
        "warnings": warnings,
        "segments": segments_report,
    }
    (out_dir / "report.json").write_text(json.dumps(report, indent=2))
    (out_dir / "report.md").write_text(_render_md(report))

    for key, m in segments_report.items():
        worst = m["worst_joint"]
        wtxt = (f"worst {worst['name']} rmse_q={worst['rmse_q']:.4f} rad"
                if worst else "no data")
        print(f"[seg] {key:>20}  {la} {m[f'duration_{la}_s']:.2f}s / "
              f"{lb} {m[f'duration_{lb}_s']:.2f}s  {wtxt}")
    print(f"[out] {out_dir}/report.json  report.md  "
          f"{len(list(out_dir.glob('seg_*.svg')))} svg plots")
    return 0


def _render_md(report):
    la = report["a"]["label"]
    lb = report["b"]["label"]
    lines = [f"# sim_match_compare -- {la} vs {lb}", "",
             f"- {la}: `{report['a']['capture']}` "
             f"({report['a']['n_motor']} motors)",
             f"- {lb}: `{report['b']['capture']}` "
             f"({report['b']['n_motor']} motors)",
             f"- grid: {report['grid_hz']} Hz, "
             f"normalize_time={report['normalize_time']}, "
             f"comparing {report['n_motor_compared']} motors", "",
             "## Warnings", ""]
    if report["warnings"]:
        lines += [f"- {w}" for w in report["warnings"]]
    else:
        lines.append("- none")
    for key, m in report["segments"].items():
        lines += ["", f"## Segment: {key}", "",
                  f"- duration: {la} {m[f'duration_{la}_s']:.2f} s, "
                  f"{lb} {m[f'duration_{lb}_s']:.2f} s "
                  f"(ratio {m['duration_ratio']:.3f})"
                  if m["duration_ratio"] is not None else
                  f"- duration: {la} {m[f'duration_{la}_s']:.2f} s, "
                  f"{lb} {m[f'duration_{lb}_s']:.2f} s",
                  f"- mean RMSE(q): {m['mean_rmse_q']:.4f} rad"
                  if m["mean_rmse_q"] is not None else "- mean RMSE(q): n/a"]
        if "thermal" in m:
            for side, th in m["thermal"].items():
                lines.append(f"- thermal {side}: hottest "
                             f"{th['hottest_motor']} at "
                             f"{th['max_temp_c']} C")
        if m["joints"]:
            lines += ["", "| joint | RMSE(q) [rad] | corr(q) | RMSE(dq) "
                          "[rad/s] | max abs delta-q [rad] |",
                      "|---|---|---|---|---|"]
            ranked = sorted(m["joints"].items(),
                            key=lambda kv: kv[1]["rmse_q"], reverse=True)
            for name, jm in ranked[:6]:
                corr = ("n/a (idle)" if jm["corr_q"] is None
                        else f"{jm['corr_q']:.3f}")
                lines.append(f"| {name} | {jm['rmse_q']:.4f} | {corr} | "
                             f"{jm['rmse_dq']:.4f} | "
                             f"{jm['max_abs_delta_q']:.4f} |")
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    raise SystemExit(main())
