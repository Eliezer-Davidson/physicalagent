#!/usr/bin/env python3
"""What does the REAL G1's telemetry timing actually look like?

WHY THIS IS WORTH A DEDICATED PROBE

    Neither simulator currently knows what it is supposed to imitate. On
    2026-08-12 this project found that the Isaac backend republished
    rt/lowstate at a steady 100 Hz with an incrementing tick and a valid CRC,
    from a shared-memory blob that was only refreshed at ~10 Hz. To any
    consumer -- including SONIC -- that stream was indistinguishable from a
    healthy one. A stalled simulation kept "publishing" frozen state.

    That defect was only findable because someone compared tick advance against
    arrival timing. This probe does that comparison on purpose, and on the real
    robot it produces the reference numbers both bridges must match.

    The measurement that matters is the TAIL, not the mean. A controller with a
    500 ms staleness watchdog does not care that the median gap is 2 ms; it
    cares how often the gap is 400 ms.

TIER: LOCKED, and in fact lower -- this script never opens DDS at all. It reads
a recording produced by hw_recorder.py. That makes it fully testable with no
robot, and it means the same command that characterises the timebase is the
one used for the mandatory end-of-day replay check.

USAGE

    python3 scripts/hw_timebase_probe.py artifacts/hw/20260819-090000-day1
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from hw_recorder import read_stream  # noqa: E402

# A gap this many times the median counts as a stall. 3x is deliberately loose:
# the point is to catch 100 ms holes in a 2 ms stream, not to flag jitter.
STALL_FACTOR = 3.0


def pct(sorted_vals, p):
    if not sorted_vals:
        return None
    k = (len(sorted_vals) - 1) * (p / 100.0)
    lo, hi = math.floor(k), math.ceil(k)
    if lo == hi:
        return sorted_vals[int(k)]
    return sorted_vals[lo] * (hi - k) + sorted_vals[hi] * (k - lo)


def analyse_stream(rows, name):
    """Single pass over an ITERABLE of records.

    It used to take a list, and analyse() built that list eagerly from the
    record reader. On short runs that is invisible; on a real one it is fatal. An hour of lowstate is 3.6M records of ~130 fields, and materialising
    them as dicts meant the mandatory end-of-day replay check did not finish in
    three minutes on the only recording size it actually has to handle.

    Every accumulator below is bounded by the number of DISTINCT values rather
    than the number of records, except the gap array, which is 8 bytes per
    sample because percentiles genuinely need every value -- and the tail is the
    measurement, so it must not be sampled away.
    """
    import array as _array
    import collections as _collections

    it = iter(rows)
    first = next(it, None)
    if first is None:
        return {"stream": name, "records": 0,
                "verdict": "NO DATA -- a silent stream is not a quiet robot; "
                           "check the topic name, the interface, and whether "
                           "the robot was actually running."}

    keys = list(first.keys())
    has_tick = "tick" in first
    NOT_PAYLOAD = ("t_mono", "t_wall", "tick", "crc")
    data_keys = [k for k in keys if k not in NOT_PAYLOAD]
    kp_k = [k for k in keys if k.endswith("_kp")]
    kd_k = [k for k in keys if k.endswith("_kd")]
    tau_k = [k for k in keys if k.endswith("_tau")]

    gaps = _array.array("d")
    n = 1
    t_first = t_prev = first["t_mono"]
    tick_first = tick_prev = first.get("tick")
    tick_repeats = tick_regressions = 0
    tick_incs = _collections.Counter()
    prev_payload = tuple(first[k] for k in data_keys)
    run_count, cur_run, max_run, dup = 0, 1, 1, 0

    def _maxabs(row, ks, sofar):
        for k in ks:
            v = row[k]
            if isinstance(v, (int, float)):
                a = abs(v)
                if a > sofar:
                    sofar = a
        return sofar

    mkp = _maxabs(first, kp_k, 0.0)
    mkd = _maxabs(first, kd_k, 0.0)
    mtau = _maxabs(first, tau_k, 0.0)

    for r in it:
        n += 1
        t = r["t_mono"]
        if t > t_prev:
            gaps.append(t - t_prev)
        t_prev = t

        if has_tick:
            tk = r["tick"]
            d = tk - tick_prev
            tick_incs[d] += 1
            if d == 0:
                tick_repeats += 1
            elif d < 0:
                tick_regressions += 1
            tick_prev = tk

        p = tuple(r[k] for k in data_keys)
        if p == prev_payload:
            cur_run += 1
            dup += 1
        else:
            run_count += 1
            if cur_run > max_run:
                max_run = cur_run
            cur_run = 1
            prev_payload = p

        if kp_k:
            mkp = _maxabs(r, kp_k, mkp)
        if kd_k:
            mkd = _maxabs(r, kd_k, mkd)
        if tau_k:
            mtau = _maxabs(r, tau_k, mtau)

    run_count += 1
    if cur_run > max_run:
        max_run = cur_run

    if n < 2:
        return {"stream": name, "records": n,
                "verdict": "NO DATA -- a single record cannot show a rate or a "
                           "gap; a silent stream is not a quiet robot."}

    span = t_prev - t_first
    gaps_sorted = sorted(gaps)
    med = pct(gaps_sorted, 50) or 0.0
    stall_count, worst_stall = 0, 0.0
    if med > 0:
        thresh = STALL_FACTOR * med
        for g in gaps:
            if g > thresh:
                stall_count += 1
                if g > worst_stall:
                    worst_stall = g

    out = {
        "stream": name,
        "records": n,
        "duration_s": round(span, 3),
        "mean_hz": round((n - 1) / span, 2) if span > 0 else None,
        "gap_ms": {
            "p50": round((pct(gaps_sorted, 50) or 0) * 1e3, 3),
            "p90": round((pct(gaps_sorted, 90) or 0) * 1e3, 3),
            "p99": round((pct(gaps_sorted, 99) or 0) * 1e3, 3),
            "p99_9": round((pct(gaps_sorted, 99.9) or 0) * 1e3, 3),
            "max": round((gaps_sorted[-1] if gaps_sorted else 0.0) * 1e3, 3),
        },
        "stalls_over_3x_median": stall_count,
        "worst_stall_ms": round(worst_stall * 1e3, 2),
    }

    if has_tick:
        fwd = sorted(k for k in tick_incs if k > 0)
        fwd_total = sum(tick_incs[k] for k in fwd)
        median_inc = None
        if fwd_total:
            half, seen = fwd_total // 2, 0
            for k in fwd:
                seen += tick_incs[k]
                if seen > half:
                    median_inc = k
                    break
        out["tick"] = {
            "first": tick_first, "last": tick_prev,
            "repeats": tick_repeats,
            "regressions": tick_regressions,
            "median_increment": median_inc,
            "distinct_increments": len(tick_incs),
            "advance_per_second": (round((tick_prev - tick_first) / span, 2)
                                   if span > 0 else 0.0),
            "arrival_hz": round((n - 1) / span, 2) if span > 0 else None,
            "note": "advance_per_second is reported raw because the counter's "
                    "units are one of the things this session is here to "
                    "measure. The republisher test is on the PAYLOAD -- see "
                    "freshness.",
            "frozen": not fwd_total,
        }
        if not fwd_total:
            out["tick"]["note"] = (
                f"TICK NEVER ADVANCED across {n} messages. The publisher is "
                "emitting, the robot's counter is not moving. Treat every "
                "value in this stream as unverified.")

    # THE republisher test. A real robot never emits two bit-identical payloads
    # -- sensor noise guarantees the low bits move. A republisher emits RUNS of
    # identical payloads, one per source refresh, so the honest sample rate is
    # the arrival rate divided by the mean run length. tick and crc are excluded
    # on purpose: regenerating exactly those is what made the Isaac stream
    # indistinguishable from a healthy one.
    if data_keys:
        mean_run = n / run_count
        frac = dup / (n - 1)
        out["freshness"] = {
            "duplicate_fraction": round(frac, 4),
            "mean_identical_run": round(mean_run, 2),
            "max_identical_run": max_run,
            "arrival_hz": out["mean_hz"],
            "effective_sample_hz": (round(out["mean_hz"] / mean_run, 2)
                                    if out["mean_hz"] and mean_run else None),
            "republished": frac > 0.10,
            "note": "republished=true means consecutive messages carried "
                    "IDENTICAL payloads, so the stream is being emitted faster "
                    "than the robot is being sampled. effective_sample_hz is "
                    "the rate that actually carries information; arrival_hz is "
                    "the rate a naive consumer would report.",
        }

    # A command stream can be published fast, forever, and command nothing.
    if kp_k or kd_k or tau_k:
        out["command"] = {
            "max_kp": round(mkp, 6), "max_kd": round(mkd, 6),
            "max_tau": round(mtau, 6),
            "has_authority": bool(mkp or mkd or mtau),
            "note": "has_authority is false when every gain and every "
                    "feedforward torque is zero for the whole recording -- "
                    "the stream exists but nothing it says can move a joint.",
        }
    return out


def analyse(run_dir):
    run_dir = pathlib.Path(run_dir)
    session = {}
    sj = run_dir / "session.json"
    if sj.exists():
        session = json.loads(sj.read_text())
    result = {"run_dir": str(run_dir), "session": session, "streams": []}
    for schema in sorted(run_dir.glob("*.schema.json")):
        binp = run_dir / (schema.name[:-len(".schema.json")] + ".bin")
        if not binp.exists():
            continue
        meta = json.loads(schema.read_text())
        rows = read_stream(binp, schema)   # generator: never materialised
        result["streams"].append(analyse_stream(rows, meta.get("topic", binp.stem)))

    # Cross-stream alignment. Streams that do not overlap in time cannot be
    # compared, which silently invalidates every later sim-vs-real diff.
    spans = [(s["stream"], s) for s in result["streams"] if s.get("records", 0) > 1]
    if len(spans) > 1:
        result["alignment"] = {
            "note": "all streams should cover the same window; a stream that "
                    "starts late or ends early cannot be diffed against the "
                    "others over the missing interval.",
            "durations_s": {n: s["duration_s"] for n, s in spans},
        }
    return analyse_result_flags(result)


def analyse_result_flags(result):
    """Cross-stream verdicts. Separated from analyse() so the rule that decides
    'is a controller live' is testable without a recording on disk."""
    names = {s["stream"] for s in result["streams"]}
    got = {s["stream"] for s in result["streams"] if s.get("records", 0) > 0}
    result["missing_streams"] = sorted(names - got)

    live = [s for s in result["streams"]
            if s["stream"] == "rt/lowcmd" and s.get("records", 0) > 0]
    result["controller_active"] = any(
        s.get("command", {}).get("has_authority") for s in live)
    if not live:
        result["controller_note"] = "no rt/lowcmd recorded"
    elif result["controller_active"]:
        c = live[0]["command"]
        result["controller_note"] = (
            f"commanding: max kp {c['max_kp']}, kd {c['max_kd']}, "
            f"tau {c['max_tau']}")
    else:
        result["controller_note"] = (
            "rt/lowcmd IS being published, and every gain and feedforward "
            "torque in it is zero for the entire recording -- an idle "
            "heartbeat. Nothing is holding this robot up.")
    return result


def render_lines(r):
    """The report as a list of lines. Separated from main() because the bug
    this exists to prevent was a RENDERING bug: freshness was computed for
    every stream and only displayed for streams carrying a tick, so the two
    streams where it fired hardest printed nothing. A renderer that cannot be
    tested is a place for measurements to go and quietly not come back."""
    out = [f"run: {r['run_dir']}"]
    if r.get("session"):
        out.append(f"label: {r['session'].get('label')}   "
                   f"iface: {r['session'].get('iface')}")
    out.append("")
    for s in r["streams"]:
        out.append(f"--- {s['stream']}")
        if s.get("verdict"):
            out.append(f"    {s['verdict']}")
            continue
        g = s["gap_ms"]
        out.append(f"    {s['records']} rec over {s['duration_s']} s = "
                   f"{s['mean_hz']} Hz")
        out.append(f"    gap ms: p50 {g['p50']}  p90 {g['p90']}  "
                   f"p99 {g['p99']}  p99.9 {g['p99_9']}  MAX {g['max']}")
        out.append(f"    stalls >3x median: {s['stalls_over_3x_median']}"
                   + (f" (worst {s['worst_stall_ms']} ms)"
                      if s["stalls_over_3x_median"] else ""))

        fr = s.get("freshness")          # every stream, ticked or not
        if fr:
            if fr["republished"]:
                out.append(f"    FRESHNESS: REPUBLISHED -- "
                           f"{fr['duplicate_fraction']*100:.0f}% of messages "
                           f"repeat the previous payload (runs up to "
                           f"{fr['max_identical_run']}). Arrives at "
                           f"{fr['arrival_hz']} Hz, informs at "
                           f"{fr['effective_sample_hz']} Hz.")
            else:
                out.append(f"    freshness: OK -- "
                           f"{fr['duplicate_fraction']*100:.1f}% duplicate "
                           f"payloads, effective {fr['effective_sample_hz']} Hz")

        t = s.get("tick")
        if t:
            if t.get("frozen"):
                out.append(f"    tick: FROZEN at {t['first']} for "
                           f"{s['records']} messages -- publisher alive, "
                           f"counter dead")
            else:
                adv = t["advance_per_second"]
                out.append(f"    tick: {adv:+g}/s, "
                           f"median step {t['median_increment']}, "
                           f"{t['repeats']} repeats, {t['regressions']} "
                           f"regressions")
                if t["regressions"]:
                    out.append(
                        f"    TICK RESET x{t['regressions']} -- the counter "
                        f"went BACKWARDS. The robot restarted mid-recording, "
                        f"so this file spans more than one boot and the tick "
                        f"is not a single continuous timebase across it.")

        c = s.get("command")
        if c:
            if c["has_authority"]:
                out.append(f"    command: AUTHORITY -- max kp {c['max_kp']}, "
                           f"kd {c['max_kd']}, tau {c['max_tau']}")
            else:
                out.append("    command: NO AUTHORITY -- every kp, kd and tau "
                           "is zero for the whole recording; this is a "
                           "heartbeat, not a controller")
    out.append("")
    if r.get("missing_streams"):
        out.append(f"MISSING (zero records): {r['missing_streams']}")
    out.append(f"controller holding the robot: {r.get('controller_active')}"
               + (f"  ({r['controller_note']})" if r.get("controller_note")
                  else ""))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--json-out", default="")
    a = ap.parse_args()
    r = analyse(a.run_dir)
    print("\n".join(render_lines(r)))
    if a.json_out:
        pathlib.Path(a.json_out).write_text(json.dumps(r, indent=2))
        print(f"json -> {a.json_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
