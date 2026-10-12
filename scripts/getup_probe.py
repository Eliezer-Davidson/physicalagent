#!/usr/bin/env python
"""Can SONIC's own mode vocabulary get the G1 off the floor? MEASURE it.

    SONIC_FRONTEND=scripts/getup_probe.py SONIC_FALL_RESET_M=0.05 ./scripts/sonic_live.sh

THE QUESTION THIS ANSWERS
    "Can't Gemini just reason and find the right poses SONIC allows and get
    itself up?" That is an empirical claim about the ACTION SPACE, not about
    reasoning: if no sequence of the 27 modes stands a fallen robot up, no
    amount of cleverness in choosing sequences will either. So this script
    tries the sequences a reasoning agent would try, from both a deliberate
    lie-down and a genuine fall, and records what the body actually did.

    Runs as the SONIC_FRONTEND so it inherits the one bring-up that works
    (start handshake -> park IDLE -> band release) instead of re-deriving it.

WHAT COUNTS
    STANDING  = pelvis >= 0.70 m AND nothing but feet touching the floor,
                sustained for 3 s. (Idle stands at 0.786; stealth_walk_2
                crouches to 0.533, so the height alone is not the test.)
    DOWN      = pelvis <= 0.25 m.
    Every sample is appended to artifacts/getup/getup_log.jsonl; screenshots
    are taken at each phase boundary. The wire tap (separate process) records
    what was actually commanded.

ALSO MEASURED, while the stack is up: does jump_forward ever produce flight
    (zero ground contacts)? Flight is contact-based per mode_probe's lesson --
    a real flight phase is ~0.1 s, i.e. >=5 samples at 50 Hz; fewer is a
    sampling artifact and is reported as inconclusive, not promoted.
"""
from __future__ import annotations

import json
import os
import pathlib
import subprocess
import sys
import time

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "scripts"))

from physicalagent.robots.sonic_sim_client import SimClient  # noqa: E402
from physicalagent.robots.sonic_zmq import (  # noqa: E402
    MODE_IDLE,
    ZmqSonicLink,
    encode_planner,
)

ART = REPO / "artifacts" / "getup"
LOG = ART / "getup_log.jsonl"

FOOT_MARKERS = ("ankle", "foot")
STAND_Z = 0.70
DOWN_Z = 0.25


class Probe:
    def __init__(self, link: ZmqSonicLink, sim: SimClient, fh):
        self.link, self.sim, self.fh = link, sim, fh
        self.have_torso = True
        self._backend_name = None
        self._reset_pub = None
        try:
            r = sim.body_pose(["pelvis", "torso_link"])
            self.have_torso = bool(r.get("ok"))
        except Exception:
            self.have_torso = False

    def backend(self) -> str:
        """'isaac' or 'mujoco', from the sidecar's own ping. Cached; defaults
        to 'mujoco' (whose ping predates the backend field) when unsure."""
        if self._backend_name is None:
            try:
                self._backend_name = (self.sim.ping() or {}).get("backend",
                                                                 "mujoco")
            except Exception:
                self._backend_name = "mujoco"
        return self._backend_name

    # -- evidence ----------------------------------------------------------
    def log(self, kind: str, **kw):
        row = {"kind": kind, "wall": round(time.time(), 3), **kw}
        self.fh.write(json.dumps(row) + "\n")
        self.fh.flush()

    def say(self, msg: str):
        print(f"[getup] {msg}", flush=True)
        self.log("say", msg=msg)

    def shot(self, name: str):
        """Screenshot, VERIFIED before it is logged as evidence.

        The first version logged a success-shaped row unconditionally; on the
        2026-08-07 Isaac run every grab silently failed (wrong window needle,
        swallowed exit code) and the trial record cited day-old MuJoCo pixels
        as that day's evidence -- the same stale-evidence class as the
        2026-08-02 pose-CSV incident below. A screenshot row now means "this
        file was freshly written by this call"; anything else logs a named
        failure row."""
        out = ART / name
        needle = "isaac" if self.backend() == "isaac" else "mujoco"
        t0 = time.time()
        try:
            r = subprocess.run(["grab-window", needle, str(out)],
                               timeout=20, check=False,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            fresh = out.exists() and out.stat().st_mtime >= t0
            if r.returncode == 0 and fresh:
                self.log("screenshot", path=str(out))
            else:
                self.log("screenshot_failed", path=str(out),
                         returncode=r.returncode, fresh_file=fresh)
                self.say(f"screenshot {name} FAILED (rc={r.returncode}, "
                         f"fresh={fresh}) -- no image evidence for this moment")
        except Exception as exc:
            self.log("screenshot_failed", path=str(out), error=repr(exc))
            self.say(f"screenshot {name} failed: {exc}")

    # -- sensing -----------------------------------------------------------
    def sample(self) -> dict | None:
        bodies = ["pelvis", "torso_link"] if self.have_torso else ["pelvis"]
        try:
            r = self.sim.body_pose(bodies)
        except Exception as exc:
            # Name the failure: a silent None here once hid a KeyError for 8
            # trials, and on 2026-08-07 it hid whatever aborted a whole run.
            self.say(f"sample failed: {type(exc).__name__}: {exc}")
            return None
        if not r.get("ok"):
            self.say(f"sample refused: {r.get('error', r)}")
            return None
        ground = r.get("ground_contact") or []
        nonfoot = sorted({p for p in ground
                          if not any(m in p for m in FOOT_MARKERS)})
        pos = r["bodies"]["pelvis"]["world_m"]
        # x/y matter as much as z: the failed direct-idle attempt THRASHED the
        # robot 1.99 m from the origin, past the band guard, ending the run.
        return {"t": r.get("sim_time_s"), "z": pos[2],
                "xy": [round(pos[0], 3), round(pos[1], 3)],
                "ground": ground, "nonfoot": nonfoot}

    def observe(self, label: str, seconds: float, hz: float = 10.0) -> dict:
        """Watch the body for a while; return an honest trajectory summary."""
        period = 1.0 / hz
        zs, nonfoot_seen = [], set()
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            t0 = time.monotonic()
            s = self.sample()
            if s:
                self.log("sample", label=label, **s)
                zs.append(s["z"])
                nonfoot_seen.update(s["nonfoot"])
            time.sleep(max(0.0, period - (time.monotonic() - t0)))
        if not zs:
            return {"label": label, "no_data": True}
        out = {"label": label, "z_start": round(zs[0], 3),
               "z_end": round(zs[-1], 3), "z_max": round(max(zs), 3),
               "z_min": round(min(zs), 3),
               "nonfoot_contacts": sorted(nonfoot_seen)}
        self.log("observe", **out)
        self.say(f"{label}: pelvis {out['z_start']:.3f} -> {out['z_end']:.3f} m "
                 f"(min {out['z_min']:.3f}, max {out['z_max']:.3f})"
                 + (f", non-foot contact: {', '.join(out['nonfoot_contacts'][:4])}"
                    if nonfoot_seen else ""))
        return out

    def wait_for(self, pred, timeout_s: float, label: str) -> bool:
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            s = self.sample()
            if s:
                self.log("sample", label=label, **s)
                if pred(s):
                    return True
            time.sleep(0.1)
        return False

    def standing_now(self, hold_s: float = 3.0) -> bool:
        """Sustained THREE-sided check: tall enough, feet-only contact, and
        feet actually ON the floor. The third clause is what catches a robot
        hanging on the band: it is tall and touches nothing, which passed a
        two-sided check while the robot flailed in mid air."""
        end = time.monotonic() + hold_s
        while time.monotonic() < end:
            s = self.sample()
            if s is None:
                return False
            self.log("sample", label="standing_check", **s)
            if s["z"] < STAND_Z - 0.02 or s["nonfoot"] or not s["ground"]:
                return False
            time.sleep(0.1)
        return True

    # -- acting ------------------------------------------------------------
    def command(self, mode: str, **kw) -> dict:
        # Crawls TRAVEL at their default speed, and the band's origin guard
        # (engage refused >0.5 m out) is the only reset mechanism between
        # attempts. Ground modes hold their posture at speed 0 -- that is the
        # mode-aware stop_all fact -- so pin the crawls in place.
        if mode in ("crawl", "elbow_crawl") and "speed" not in kw:
            kw["speed"] = 0.0
        r = self.link.select_mode(mode, **kw)
        self.log("command", mode=mode, kwargs=kw, result=r)
        self.say(f"-> mode {mode} (speed {r.get('speed')}, held {r.get('held')})")
        return r

    def band(self, **kw) -> dict:
        try:
            r = self.sim.request("band", **kw)
        except Exception as exc:
            r = {"ok": False, "error": str(exc)}
        self.log("band", kwargs=kw, result=r)
        return r

    # -- compound moves ----------------------------------------------------
    def recover_by_band(self) -> bool:
        """Reset mechanism BETWEEN attempts (sim-only, and says so): hoist on
        the elastic band, lower to the floor, release. Not part of any answer
        about the action space -- the band does not exist on hardware."""
        self.say("band recovery: hoisting (this is the harness, not the robot)")
        self.command("idle")
        r = self.band(engage=True)
        if not r.get("ok", False) and r.get("error"):
            self.say(f"band engage refused/failed: {r}")
            return False
        self.wait_for(lambda s: s["z"] > 0.90, 8.0, "band_hoist")
        self.band(length_m=-0.2)
        self.wait_for(lambda s: 0.65 < s["z"] < 0.90 and s["ground"],
                      10.0, "band_lower")
        self.band(release=True)
        time.sleep(2.0)
        ok = self.standing_now()
        self.say(f"band recovery -> {'standing' if ok else 'NOT standing'}")
        return ok

    def bring_down_from_band(self) -> bool:
        """Get off the band, verified by the SIDECAR and never by a pose CSV.

        The console's release check tails SONIC_POSE_LOG, and on 2026-08-02 it
        tailed a PREVIOUS session's file and declared a hanging robot 'already
        free' — pelvis 0.96 m, no contacts, flailing. The sidecar reads the
        live MuJoCo state, so it cannot be stale. Deterministic sequence:
        shorten the band until the feet touch, then release — a zero-height
        drop instead of a 0.2 m fall."""
        s = self.sample()
        if s is None:
            self.say("no sidecar sample -- cannot verify the band; aborting")
            return False
        if s["z"] < 0.25:
            self.say(f"robot is DOWN at bring-up (pelvis {s['z']:.3f} m)")
            if self.backend() == "isaac":
                return self.recover_by_sim_reset()
            return self.recover_by_band()
        if s["z"] < 0.88 and s["ground"]:
            self.say(f"band already off: pelvis {s['z']:.3f} m with foot contact")
            return True
        self.say(f"pelvis {s['z']:.3f} m, contacts {s['ground'] or 'NONE'} -- "
                 "still ON THE BAND; lowering deterministically")
        self.command("idle")
        self.band(length_m=-0.2)
        self.wait_for(lambda s: s["z"] < 0.86 and s["ground"], 12.0, "band_lower")
        self.band(release=True)
        time.sleep(2.0)
        ok = self.standing_now()
        self.say(f"band bring-down -> {'standing' if ok else 'NOT standing'}")
        return ok

    def descend_to_lie(self) -> bool:
        """Deliberate, staged descent to the floor -- the entry the survey
        showed works. Returns True when the robot is genuinely down."""
        self.command("idle")
        time.sleep(2.0)
        self.command("kneel_two_legs")
        if not self.wait_for(lambda s: s["z"] < 0.50, 12.0, "descend_kneel"):
            self.say("kneel descent did not happen within 12 s")
        self.command("lie_face_down")
        down = self.wait_for(lambda s: s["z"] < 0.20, 20.0, "descend_lie")
        time.sleep(3.0)
        s = self.sample()
        self.say(f"on the floor: pelvis {s['z']:.3f} m, contacts "
                 f"{', '.join(s['nonfoot'][:5]) or 'feet only'}" if s else
                 "on the floor: no telemetry")
        return down

    def recover_by_sim_reset(self, attempts: int = 2) -> bool:
        """Reset mechanism BETWEEN attempts on backends with no band (Isaac):
        ask the vendor sim for a full scene reset over rt/reset_pose/cmd
        (category '2' -> reset_all, a 04-patch addition), then let the policy
        re-stabilise. Harness, not robot -- same disclaimer as the band.

        idle is commanded BEFORE the teleport and AGAIN AFTER it: a failed
        get-up leaves SONIC wedged mid-transition (e.g. idle:timeout from
        kneel with a prone body), and after the teleport it drove the freshly
        standing robot straight back down toward the pose its transition
        graph was stuck in (measured 2026-08-07: standing for ~0.3 s, then
        folded). From a standing body the idle transition can actually
        complete -- it needs re-asking and a moment, not a better teleport."""
        try:
            if self._reset_pub is None:
                from unitree_sdk2py.core.channel import (
                    ChannelFactoryInitialize, ChannelPublisher)
                from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
                try:
                    # Idempotent-by-intent: the frontends init the factory via
                    # UnitreeTelemetry, but a bare getup_probe run does not --
                    # without this the publisher constructor dies on a None
                    # participant.
                    ChannelFactoryInitialize(0, "lo")
                except Exception:
                    pass                      # already initialised
                self._reset_msg = String_
                pub = ChannelPublisher("rt/reset_pose/cmd", String_)
                pub.Init()                    # only a FULLY built publisher is
                self._reset_pub = pub         # kept -- a half-built one would
                                              # dead-end every retry
        except Exception as exc:
            self.say(f"sim reset publisher init failed: {exc!r}")
            return False
        for i in range(1, attempts + 1):
            self.say(f"sim reset recovery {i}/{attempts}: rt/reset_pose/cmd "
                     "'2' (this is the harness, not the robot)")
            self.command("idle")
            try:
                self._reset_pub.Write(self._reset_msg(data="2"))
            except Exception as exc:
                self.say(f"sim reset publish failed: {exc!r}")
                return False
            self.wait_for(lambda s: s["z"] > 0.70, 10.0, "sim_reset_teleport")
            self.command("idle")          # re-ask now that the body can comply
            self.wait_for(lambda s: s["z"] > 0.70 and s["ground"], 8.0,
                          "sim_reset_settle")
            ok = self.standing_now()
            self.say(f"sim reset recovery -> {'standing' if ok else 'NOT standing'}")
            if ok:
                return True
        return False

    def ensure_standing(self) -> bool:
        if self.standing_now(hold_s=1.0):
            return True
        if self.backend() == "isaac":
            return self.recover_by_sim_reset()
        return self.recover_by_band()


def ascent_attempt(p: Probe, name: str, stages: list[tuple[str, float]]) -> dict:
    """One fresh lie-down, then the given mode sequence. The verdict is the
    sustained standing check, never 'the last command was idle'."""
    p.say(f"=== ATTEMPT {name}: {' -> '.join(m for m, _ in stages)} ===")
    if not p.ensure_standing():
        return {"attempt": name, "aborted": "could not reach standing to start"}
    if not p.descend_to_lie():
        p.say("could not reach the floor deliberately; aborting this attempt")
        return {"attempt": name, "aborted": "descent failed"}
    p.shot(f"{name}_0_down.png")
    stage_out = []
    for mode, wait_s in stages:
        p.command(mode)
        stage_out.append(p.observe(f"{name}:{mode}", wait_s))
    stood = p.standing_now()
    p.shot(f"{name}_1_end.png")
    p.say(f"ATTEMPT {name}: {'STOOD UP' if stood else 'did NOT stand up'}")
    out = {"attempt": name, "stages": stage_out, "stood_up": stood}
    p.log("attempt", **out)
    return out


def real_fall_attempt(p: Probe, unpinned: bool = False,
                      rounds: int = 2) -> dict:
    """Induce a genuine fall (the crawl->elbow_crawl transition drops a
    shoulder 4 times in 5), then try the recovery ladder from THAT posture.

    unpinned=True uses the modes' own vendor speeds instead of pinning them to
    0 -- the 4-in-5 falls were observed at default speeds, and the pinned
    transition survived 2/2 on 2026-08-02. Displacement then makes the band
    unusable, so run this as the LAST phase only."""
    p.say(f"=== REAL FALL: crawl -> elbow_crawl transition "
          f"({'vendor speeds' if unpinned else 'pinned to 0'}) ===")
    spd = None if unpinned else 0.0
    if not p.ensure_standing():
        return {"attempt": "real_fall", "aborted": "could not reach standing"}
    p.command("kneel_two_legs")
    p.wait_for(lambda s: s["z"] < 0.50, 12.0, "fall_kneel")
    p.command("crawl", speed=spd)
    time.sleep(6.0)
    fell = False
    for round_ in range(1, rounds + 1):
        p.command("elbow_crawl", speed=spd)
        fell = p.wait_for(
            lambda s: any(("shoulder" in c or "torso" in c or "head" in c)
                          for c in s["nonfoot"]) and s["z"] < 0.35,
            15.0, f"fall_watch_{round_}")
        if fell:
            break
        p.say(f"elbow_crawl round {round_}/{rounds} did not fall; "
              + ("returning to crawl to try again" if round_ < rounds else
                 f"no fall induced in {rounds} rounds"))
        if round_ < rounds:
            p.command("crawl", speed=spd)
            time.sleep(4.0)
    if not fell:
        out = {"attempt": "real_fall", "induced": False}
        p.log("attempt", **out)
        return out
    s = p.sample()
    p.shot("realfall_0_down.png")
    p.say(f"FELL for real: pelvis {s['z']:.3f} m, contacts "
          f"{', '.join(s['nonfoot'][:5])}" if s else "FELL (no telemetry)")
    time.sleep(2.0)
    stage_out = []
    for mode, wait_s in [("lie_face_down", 8.0), ("crawl", 10.0),
                         ("kneel_two_legs", 15.0), ("idle", 15.0)]:
        p.command(mode)
        stage_out.append(p.observe(f"realfall:{mode}", wait_s))
    stood = p.standing_now()
    p.shot("realfall_1_end.png")
    p.say(f"REAL-FALL recovery: {'STOOD UP' if stood else 'did NOT stand up'}")
    out = {"attempt": "real_fall", "induced": True, "stages": stage_out,
           "stood_up": stood}
    p.log("attempt", **out)
    return out


def jump_attempt(p: Probe) -> dict:
    """Flight = ZERO ground contacts, >=5 consecutive samples at 50 Hz."""
    p.say("=== JUMP: does jump_forward ever leave the floor? ===")
    if not p.ensure_standing():
        return {"attempt": "jump", "aborted": "could not reach standing"}
    p.command("jump_forward", duration_s=8.0)
    period = 1.0 / 50.0
    streak = best = airborne = total = 0
    end = time.monotonic() + 9.0
    while time.monotonic() < end:
        t0 = time.monotonic()
        s = p.sample()
        if s:
            total += 1
            p.log("sample", label="jump", **s)
            if not s["ground"]:
                streak += 1
                airborne += 1
                best = max(best, streak)
            else:
                streak = 0
        time.sleep(max(0.0, period - (time.monotonic() - t0)))
    p.command("idle")
    p.shot("jump_end.png")
    if best >= 5:
        verdict = f"FLIGHT: {best} consecutive contact-free samples ({best / 50.0:.2f}s)"
    elif best >= 2:
        verdict = (f"INCONCLUSIVE: best streak {best} samples ({best / 50.0:.2f}s) "
                   "-- artifact-length, not a demonstrated jump")
    else:
        verdict = "NO FLIGHT: a foot was always on the floor"
    p.say(f"jump verdict: {verdict} ({airborne}/{total} samples contact-free)")
    out = {"attempt": "jump", "best_streak": best, "airborne": airborne,
           "total": total, "verdict": verdict}
    p.log("attempt", **out)
    return out


def main() -> int:
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--phases",
                    default="ladder,direct,crawl_route,real_fall,jump",
                    help="comma list; order matters -- an attempt that strands "
                         "the robot far from the origin ends the whole run "
                         "(band guard), so put the risky ones LAST")
    args = ap.parse_args()

    ART.mkdir(parents=True, exist_ok=True)
    reset_m = os.environ.get("SONIC_FALL_RESET_M")
    if reset_m is None or float(reset_m) > 0.06:
        print("[getup] WARNING: SONIC_FALL_RESET_M not lowered -- the sim will "
              "teleport-reset any robot below 0.20 m and the whole experiment "
              "is void. Set SONIC_FALL_RESET_M=0.05.", flush=True)

    # The proven handshake, verbatim from the console.
    link = ZmqSonicLink(port=5556, settle_s=1.0, auto_start=False)
    print("[getup] publisher bound on tcp://*:5556", flush=True)
    link.start_control()
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        link._pub.send(encode_planner(MODE_IDLE, (1.0, 0.0, 0.0),
                                      (1.0, 0.0, 0.0), 0.0, -1.0))
        time.sleep(0.05)
    print("[getup] planner parked in IDLE", flush=True)

    sim = SimClient(port=int(os.environ.get("SONIC_SIDECAR_PORT", "5570")))
    results = []
    with LOG.open("w") as fh:
        p = Probe(link, sim, fh)
        try:
            if not p.bring_down_from_band():
                p.say("bring-up failed; aborting")
                return 2
            p.observe("baseline_idle", 2.0)
            for phase in (w.strip() for w in args.phases.split(",")):
                if phase == "ladder":
                    results.append(ascent_attempt(
                        p, "ladder",
                        [("kneel_two_legs", 15.0), ("idle", 15.0)]))
                elif phase == "direct":
                    results.append(ascent_attempt(p, "direct",
                                                  [("idle", 15.0)]))
                elif phase == "crawl_route":
                    results.append(ascent_attempt(
                        p, "crawl_route",
                        [("crawl", 10.0), ("kneel_two_legs", 15.0),
                         ("idle", 15.0)]))
                elif phase == "real_fall":
                    results.append(real_fall_attempt(p))
                elif phase == "real_fall_unpinned":
                    results.append(real_fall_attempt(p, unpinned=True,
                                                     rounds=4))
                elif phase == "jump":
                    results.append(jump_attempt(p))
                elif phase:
                    p.say(f"unknown phase {phase!r} skipped")
        finally:
            try:
                p.command("idle")
            except Exception:
                pass
            p.log("summary", results=results)
            print("\n[getup] SUMMARY", flush=True)
            for r in results:
                print(f"  {json.dumps(r, default=str)[:400]}", flush=True)
            link.close()
            sim.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
