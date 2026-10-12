#!/usr/bin/env python
"""Does recover_from_fall actually stand the robot up? MEASURE it, n per geometry.

    SONIC_FRONTEND=scripts/getup_verb_probe.py SONIC_FALL_RESET_M=0.05 \
        ./scripts/sonic_live.sh --trials 10

WHAT THIS IS, AND WHAT getup_probe.py IS NOT
    getup_probe answered an ACTION-SPACE question — "can any sequence of the 27
    modes stand a fallen robot up?" — by driving the link directly, n=1 per
    route. It found one route that works.

    This probe answers a DIFFERENT question about the SHIPPED VERB:
    `SonicG1Adapter._recover_from_fall()`. It calls the adapter method the model
    actually gets, with the telemetry, the fall latch and the stage waits all
    live, and it repeats. n=1 is an existence proof; a verb the safety layer
    leans on needs a rate.

THE THREE QUESTIONS
    prone      Deliberate staged descent to the floor, then the verb. The robot
               barely travels, so the band can reset it between trials and a
               full n is reachable.
    real_fall  A genuine shoulder-drop (the crawl -> elbow_crawl transition
               drops one 4 times in 5), then the verb. Each fall DISPLACES the
               robot, and the band refuses to engage beyond 0.5 m of the origin
               (kp=10000 catapult guard), so this geometry runs until the robot
               drifts out of reach. However many trials that is, is reported —
               a truncated run says so rather than quietly reporting a rate
               over fewer samples than asked for.
    after_damp Is the verb even REACHABLE once the real safety layer has acted?
               A confirmed fall auto-damps, and auto-damp sends
               command{start:0,stop:1} to the deploy binary. If that stops the
               controller for good, then the get-up verb cannot run in the one
               situation it exists for, and the fix is a re-arm inside the verb
               — not a better route. Measured separately and LAST because it
               may end the session.

WHY AUTO-DAMP IS OFF FOR THE FIRST TWO
    One variable at a time. prone/real_fall measure the ROUTE; auto-damp would
    inject the controller-stop question into every trial and confound both. The
    fall DETECTOR still runs and still latches, so the verb sees exactly the
    `fallen` state it gates on — only the damp command is withheld.

WHAT COUNTS AS SUCCESS
    Whatever the shipped verb itself claims, checked against an independent
    sustained standing test from the sidecar (pelvis >= 0.70 m, feet-only
    contact, held 3 s). Both are recorded. A trial where the verb says
    recovered=True and the body says otherwise is the most interesting outcome
    here and must not be averaged away.
"""
from __future__ import annotations

import json
import os
import pathlib
import sys
import time

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "scripts"))

from getup_probe import Probe  # noqa: E402  (measured criteria, band, logging)
from physicalagent.robots.sonic_g1 import SonicG1Adapter  # noqa: E402
from physicalagent.robots.sonic_sim_client import SimClient  # noqa: E402
from physicalagent.robots.sonic_zmq import (  # noqa: E402
    MODE_IDLE,
    ZmqSonicLink,
    encode_planner,
)

ART = REPO / "artifacts" / "getup_verb"
LOG = ART / "trials.jsonl"

# The band anchors at the origin with kp=10000 and is refused beyond 0.5 m.
# Stop a geometry before that, not at it: a trial needs room to travel too.
BAND_REACH_M = 0.40


class _Guard:
    is_estopped = False


def _displacement(p: Probe) -> float | None:
    """Distance from the band's anchor at the origin.

    Reuses Probe.sample() rather than re-reading body_pose: the first version
    of this reached for r["bodies"]["pelvis"]["pos"], but the sidecar's key is
    "world_m". The KeyError was swallowed by the except, the guard returned
    None on every call, and the run burned 8 trials it could never have reset.
    A guard that fails closed to "no opinion" is a guard that is off."""
    s = p.sample()
    if s is None:
        return None
    x, y = s["xy"]
    return float(x * x + y * y) ** 0.5


def _induce_prone(p: Probe) -> bool:
    """Fall geometry A: the deliberate staged descent. Repeatable and it does
    not travel, which is the only reason a full n is reachable here."""
    return p.descend_to_lie()


def _induce_real_fall(p: Probe, rounds: int = 3) -> bool:
    """Fall geometry B: a genuine shoulder-drop out of the crawl -> elbow_crawl
    transition, at VENDOR speeds because that is where the 4-in-5 drops were
    observed. Pinning speed to 0 is the thing that avoids the fall, so it would
    defeat the purpose here."""
    p.command("kneel_two_legs")
    p.wait_for(lambda s: s["z"] < 0.50, 12.0, "fall_kneel")
    p.command("crawl")
    time.sleep(5.0)
    for round_ in range(1, rounds + 1):
        p.command("elbow_crawl")
        fell = p.wait_for(
            lambda s: any(("shoulder" in c or "torso" in c or "head" in c)
                          for c in s["nonfoot"]) and s["z"] < 0.35,
            15.0, f"fall_watch_{round_}")
        if fell:
            return True
        p.say(f"elbow_crawl round {round_}/{rounds} did not drop a shoulder")
        if round_ < rounds:
            p.command("crawl")
            time.sleep(4.0)
    return False


def run_trial(p: Probe, adapter: SonicG1Adapter, geometry: str, n: int) -> dict:
    """One trial: reset to standing, induce the fall, run the VERB, judge."""
    t0 = time.monotonic()
    p.say(f"=== {geometry} trial {n}: reset -> induce -> recover_from_fall ===")
    if not p.ensure_standing():
        return {"geometry": geometry, "trial": n, "aborted": "no standing start"}

    induced = (_induce_prone(p) if geometry == "prone"
               else _induce_real_fall(p))
    if not induced:
        return {"geometry": geometry, "trial": n, "aborted": "fall not induced"}
    p.shot(f"{geometry}_{n}_0_down.png")
    down = p.sample()
    # The descent wait latches on a single z<0.20 sample; a body that bounced
    # or was flung passes it while not actually being down at verb start. A
    # "prone" trial whose subject is not prone must not be scored as one.
    if down is None or down["z"] >= 0.25:
        return {"geometry": geometry, "trial": n,
                "aborted": "fall not induced (not down at verb start)",
                "down_pelvis_m": None if down is None else down["z"]}

    # Let the detector see the floor and latch, exactly as it would live.
    latch_deadline = time.monotonic() + 6.0
    while time.monotonic() < latch_deadline and not adapter._fall.fallen:
        time.sleep(0.2)
    latched = adapter._fall.fallen
    if not latched:
        p.say("detector did NOT latch a fall; the verb will refuse unless the "
              "posture read says down — recording that as the result")

    verb_t0 = time.monotonic()
    try:
        out = adapter._recover_from_fall()
        error = None
    except Exception as exc:                     # a refusal is a result, a crash is not
        out, error = None, f"{type(exc).__name__}: {exc}"
    verb_s = round(time.monotonic() - verb_t0, 2)

    stood = p.standing_now()                      # independent of what the verb claims
    p.shot(f"{geometry}_{n}_1_end.png")
    end = p.sample()
    trial = {
        "geometry": geometry, "trial": n,
        "latched_before_verb": latched,
        "verb": out, "verb_error": error, "verb_seconds": verb_s,
        "body_says_standing": stood,
        "agree": (bool(out and out.get("recovered")) == stood),
        "down_pelvis_m": round(down["z"], 3) if down else None,
        "end_pelvis_m": round(end["z"], 3) if end else None,
        "displacement_m": _displacement(p),
        "trial_seconds": round(time.monotonic() - t0, 1),
    }
    p.say(f"{geometry} trial {n}: verb says "
          f"{bool(out and out.get('recovered'))}, body says {stood}"
          + ("" if trial["agree"] else "   <-- THEY DISAGREE"))
    p.log("trial", **trial)
    return trial


def run_geometry(p: Probe, adapter: SonicG1Adapter, geometry: str,
                 trials: int) -> dict:
    out, stopped = [], None
    for n in range(1, trials + 1):
        d = _displacement(p)
        # The reach limit is the BAND's; the Isaac backend resets by scene
        # reset (rt/reset_pose/cmd), which works from anywhere.
        if p.backend() != "isaac" and d is not None and d > BAND_REACH_M:
            stopped = (f"robot is {d:.2f} m from the origin and the band is "
                       "refused beyond 0.5 m — the harness cannot reset it. "
                       "Restart the simulator to continue this geometry.")
            break
        t = run_trial(p, adapter, geometry, n)
        out.append(t)
        if t.get("aborted") == "no standing start":
            # The reset itself failed. Every later trial would abort the same
            # way, and 8 of them did on the first run — noise that reads like
            # evidence. Stop and say why.
            stopped = ("could not reset the robot to standing; the band "
                       "recovery failed. Restart the simulator to continue.")
            break
    if stopped:
        p.say(f"STOPPING {geometry} after {len(out)}/{trials}: {stopped}")
    done = [t for t in out if "aborted" not in t]
    ok = [t for t in done if t.get("body_says_standing")]
    return {"geometry": geometry, "asked": trials, "attempted": len(out),
            "scored": len(done), "stood_up": len(ok),
            "rate": (f"{len(ok)}/{len(done)}" if done else "0/0"),
            "truncated": stopped,
            "aborts": [t.get("aborted") for t in out if "aborted" in t],
            "disagreements": [t["trial"] for t in done if not t["agree"]],
            "trials": out}


def run_after_damp(p: Probe, adapter: SonicG1Adapter) -> dict:
    """Is the verb reachable once the real safety layer has acted? Auto-damp
    sends command{start:0,stop:1}. If the deploy binary stops for good, the
    get-up verb is unreachable in the exact case it was written for."""
    p.say("=== after_damp: does the verb still work once auto-damp has fired? ===")
    if not p.ensure_standing():
        return {"geometry": "after_damp", "aborted": "no standing start"}
    if not _induce_prone(p):
        return {"geometry": "after_damp", "aborted": "fall not induced"}

    adapter._auto_damp = True
    adapter._fall_alarm_fired = False
    damped = False
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        adapter._fall_poll_once()
        if adapter._fall_alarm_fired:
            damped = True
            break
        time.sleep(0.2)
    p.say(f"auto-damp fired: {damped}")
    time.sleep(2.0)

    out = adapter._recover_from_fall()
    stood = p.standing_now()
    p.shot("after_damp_end.png")
    # If it failed, is a re-arm all that was missing? That distinguishes
    # "the route is wrong" from "the controller was switched off".
    rearm = None
    if not stood:
        p.say("re-arming the controller (start_control) and retrying ONCE — "
              "this separates a dead controller from a bad route")
        try:
            adapter._link.start_control()
            time.sleep(2.0)
            rearm_out = adapter._recover_from_fall()
            rearm = {"verb": rearm_out, "body_says_standing": p.standing_now()}
        except Exception as exc:
            rearm = {"error": f"{type(exc).__name__}: {exc}"}
    res = {"geometry": "after_damp", "damp_fired": damped, "verb": out,
           "body_says_standing": stood, "after_rearm": rearm}
    adapter._auto_damp = False
    p.log("after_damp", **res)
    return res


def main() -> int:
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=10)
    ap.add_argument("--geometries", default="prone,real_fall",
                    help="comma list; order matters. real_fall DISPLACES the "
                         "robot and eventually puts it out of the band's "
                         "reach, so it runs last. after_damp is NOT in the "
                         "default set: it needs a standing start that "
                         "real_fall has already spent, and it may stop the "
                         "deploy binary outright — give it its own session.")
    args = ap.parse_args()

    ART.mkdir(parents=True, exist_ok=True)
    reset_m = os.environ.get("SONIC_FALL_RESET_M")
    if reset_m is None or float(reset_m) > 0.06:
        print("[verb] WARNING: SONIC_FALL_RESET_M not lowered — the sim will "
              "teleport-reset any robot below 0.20 m, which is above the prone "
              "pelvis (~0.15 m). Every trial would be void. Set it to 0.05.",
              flush=True)

    link = ZmqSonicLink(port=5556, settle_s=1.0, auto_start=False)
    link.start_control()
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        link._pub.send(encode_planner(MODE_IDLE, (1.0, 0.0, 0.0),
                                      (1.0, 0.0, 0.0), 0.0, -1.0))
        time.sleep(0.05)
    print("[verb] planner parked in IDLE", flush=True)

    sim = SimClient(port=int(os.environ.get("SONIC_SIDECAR_PORT", "5570")))

    telemetry = None
    try:
        from physicalagent.robots.sonic_telemetry import UnitreeTelemetry
        telemetry = UnitreeTelemetry(init_channel=True, interface="lo")
        if not telemetry.wait_for_data(timeout_s=5.0):
            print("[verb] no rt/lowstate within 5 s — the verb waits on the "
                  "body, so without telemetry every stage reads 'blind' and "
                  "this measurement is NOT the one we want.", flush=True)
    except Exception as exc:
        print(f"[verb] telemetry unavailable: {exc}", flush=True)

    # auto_damp OFF: measure the route, not the controller-stop interaction.
    adapter = SonicG1Adapter(link, _Guard(), telemetry=telemetry, sim=sim,
                             auto_damp=False)
    adapter.start_fall_watchdog()

    summary = []
    with LOG.open("w") as fh:
        p = Probe(link, sim, fh)
        try:
            if not p.bring_down_from_band():
                p.say("bring-up failed; aborting")
                return 2
            for geometry in (w.strip() for w in args.geometries.split(",")):
                if geometry == "after_damp":
                    summary.append(run_after_damp(p, adapter))
                elif geometry in ("prone", "real_fall"):
                    summary.append(run_geometry(p, adapter, geometry,
                                                args.trials))
                elif geometry:
                    p.say(f"unknown geometry {geometry!r} skipped")
        finally:
            adapter.stop_fall_watchdog()
            try:
                p.command("idle")
            except Exception:
                pass
            p.log("summary", results=summary)
            print("\n[verb] SUMMARY", flush=True)
            for r in summary:
                thin = {k: v for k, v in r.items() if k != "trials"}
                print(f"  {json.dumps(thin, default=str)}", flush=True)
            print(f"[verb] per-trial detail: {LOG}", flush=True)
            link.close()
            sim.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
