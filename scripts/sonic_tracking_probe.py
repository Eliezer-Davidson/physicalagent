#!/usr/bin/env python
"""Does SONIC actually DRIVE this backend's legs? MEASURE it, don't infer it.

    SONIC_FRONTEND=scripts/sonic_tracking_probe.py ./scripts/sonic_isaac_live.sh
    SONIC_FRONTEND=scripts/sonic_tracking_probe.py ./scripts/sonic_live.sh   (baseline)

WHY THIS EXISTS (2026-08-12): the Isaac wholebody mode routed only SONIC's 14
arm targets; the legs were the vendor's own in-sim policy the whole time, and
every outcome gate (upright, lowcmd flowing) passed anyway. This probe is the
causality check those gates lacked: it records rt/lowcmd targets against
rt/lowstate positions for the SAME joints while commanding a distinctive leg
motion (kneel). Wired legs track the sweep; unwired legs do whatever the
local policy does, and the correlation says which -- no eyeballing required.

WHAT IT MEASURES
  wiring   per-joint Pearson corr + RMS error (lowcmd q vs lowstate q) on the
           knees during the kneel sweep. It only judges when the COMMAND
           actually swept (>= 0.25 rad) -- a planner that never moved is
           reported as inconclusive, not as pass or fail.
  jitter   stand-phase leg joint-speed RMS + pelvis-height std: the number
           for "it wiggles". Run the same probe on MuJoCo for the baseline.
  RTF      sidecar sim_time vs wall clock, when the sidecar reports sim time.

Evidence: verdict lines, JSONL + head-camera JPEG under artifacts/tracking/.
Exit codes: 0 = legs WIRED, 1 = inconclusive, 2 = legs NOT wired or failure.
Read-only on every rt/* topic except rt/reset_pose/cmd (harness reset, Isaac
only, after the kneel).
"""
from __future__ import annotations

import json
import math
import os
import pathlib
import sys
import threading
import time

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "scripts"))

from physicalagent.robots.sonic_sim_client import SimClient  # noqa: E402
from physicalagent.robots.sonic_telemetry import tilt_from_quat  # noqa: E402
from physicalagent.robots.sonic_zmq import (  # noqa: E402
    MODE_IDLE,
    ZmqSonicLink,
    encode_planner,
)

ART = REPO / "artifacts" / "tracking"

# hw-order indices (LowCmd_/LowState_ motor index). Knees carry the kneel
# verdict; the others give context in the record.
TRACK = {
    0: "left_hip_pitch",
    3: "left_knee",
    4: "left_ankle_pitch",
    9: "right_knee",
    15: "left_shoulder_pitch",
}
VERDICT_JOINTS = (3, 9)          # both knees must agree
LEG_RANGE = range(0, 12)
CMD_SWEEP_MIN_RAD = 0.25         # below this the kneel never happened
WIRED_CORR = 0.80
WIRED_RMS_RAD = 0.30             # generous: staircase lag during the sweep
UNWIRED_CORR = 0.30


class Recorder:
    """Append-only (t, q...) series from rt/lowcmd and rt/lowstate.

    Subscribing is read-only; nothing here can move a robot. Series stay in
    memory (500 Hz x ~40 s x 5 floats is trivial) and are dumped to JSONL."""

    def __init__(self):
        self.lock = threading.Lock()
        self.cmd: list[tuple[float, tuple[float, ...]]] = []
        self.state: list[tuple[float, tuple[float, ...]]] = []
        self.dq: list[tuple[float, tuple[float, ...]]] = []
        self.tilt: list[tuple[float, float]] = []
        self._track = sorted(TRACK)

    def start(self):
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize, ChannelSubscriber)
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        try:
            # The frontends normally init the factory via UnitreeTelemetry; a
            # bare run does not. Second init raises -- that is fine.
            ChannelFactoryInitialize(0, "lo")
        except Exception:
            pass
        # Subscribers are bound to self so they cannot be garbage-collected --
        # the M0 probe's unreferenced subscriber froze its counter at t=4.8 s
        # and produced the false "79 Hz" figure.
        self._cmd_sub = ChannelSubscriber("rt/lowcmd", LowCmd_)
        self._cmd_sub.Init(self._on_cmd, 10)
        self._state_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self._state_sub.Init(self._on_state, 10)

    def _on_cmd(self, msg):
        row = tuple(float(msg.motor_cmd[i].q) for i in self._track)
        with self.lock:
            self.cmd.append((time.monotonic(), row))

    def _on_state(self, msg):
        row = tuple(float(msg.motor_state[i].q) for i in self._track)
        dqrow = tuple(float(msg.motor_state[i].dq) for i in LEG_RANGE)
        # Pelvis height alone is NOT standing. This project's definition is
        # height AND tilt AND nothing but feet on the floor -- a robot pitched
        # 40 deg back holds a perfectly respectable 0.85 m.
        try:
            q = msg.imu_state.quaternion
            # tilt_from_quat returns RADIANS. Labelling it "deg" made a
            # face-down robot (1.6 rad = 92 deg) read as a 1.6 deg lean and
            # nearly bought a "the IMU is broken" conclusion, 2026-08-18.
            tilt = math.degrees(tilt_from_quat(
                float(q[0]), float(q[1]), float(q[2]), float(q[3])))
        except Exception:
            tilt = None
        with self.lock:
            t = time.monotonic()
            self.state.append((t, row))
            self.dq.append((t, dqrow))
            if tilt is not None:
                self.tilt.append((t, tilt))

    def window(self, series, t0, t1):
        with self.lock:
            return [(t, r) for t, r in series if t0 <= t <= t1]


def col(rows, track_idx):
    j = sorted(TRACK).index(track_idx)
    return [(t, r[j]) for t, r in rows]


def interp(series, t):
    """Linear interpolation of (t, v) rows at time t; None outside the span."""
    if not series or t < series[0][0] or t > series[-1][0]:
        return None
    lo, hi = 0, len(series) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if series[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    (ta, va), (tb, vb) = series[lo], series[hi]
    if tb == ta:
        return va
    return va + (vb - va) * (t - ta) / (tb - ta)


def pearson(xs, ys):
    n = len(xs)
    if n < 8:
        return None
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    syy = sum((y - my) ** 2 for y in ys)
    if sxx <= 1e-12 or syy <= 1e-12:
        return None
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    return sxy / (sxx * syy) ** 0.5


def joint_metrics(cmd_rows, state_rows, idx):
    """corr + rms of state vs cmd, evaluated on state timestamps (the slower,
    honester clock). cmd_sweep_rad comes from the RAW command column -- a dead
    state stream must read as "state stream too thin", never as "the planner
    never moved"."""
    c, s = col(cmd_rows, idx), col(state_rows, idx)
    out = {}
    if c:
        raw = [v for _, v in c]
        out["cmd_sweep_rad"] = round(max(raw) - min(raw), 3)
    pairs = [(interp(c, t), v) for t, v in s]
    pairs = [(a, b) for a, b in pairs if a is not None]
    out["n"] = len(pairs)
    if len(pairs) < 8:
        return out
    cmd_vals = [a for a, _ in pairs]
    err = [(a - b) for a, b in pairs]
    r = pearson(cmd_vals, [b for _, b in pairs])
    out["corr"] = None if r is None else round(r, 3)
    out["rms_rad"] = round((sum(e * e for e in err) / len(err)) ** 0.5, 3)
    return out


class Run:
    def __init__(self):
        ART.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self.stamp = stamp
        self.fh = (ART / f"tracking-{stamp}.jsonl").open("w")
        self.rec = Recorder()
        self.sim = SimClient(port=int(os.environ.get("SONIC_SIDECAR_PORT",
                                                     "5570")))
        self.link = None
        self.backend = "unknown"
        self._reset_pub = None
        self._reset_msg = None

    def log(self, kind, **kw):
        row = {"kind": kind, "wall": round(time.time(), 3), **kw}
        self.fh.write(json.dumps(row) + "\n")
        self.fh.flush()

    def say(self, msg):
        print(f"[tracking] {msg}", flush=True)
        self.log("say", msg=msg)

    # -- sidecar helpers ---------------------------------------------------
    def sidecar_times(self):
        """(wall, sim_time_s) pair, or None when the sidecar has no sim time
        (the MuJoCo sidecar predates the field)."""
        try:
            r = self.sim.ping()
            st = r.get("sim_time_s")
            return (time.time(), float(st)) if st is not None else None
        except Exception:
            return None

    def nonfoot_contacts(self):
        """Ground contacts that are not feet. A knee or a hand on the floor
        is not a stand, however tall the pelvis reads."""
        try:
            r = self.sim.body_pose(["pelvis"])
            ground = r.get("ground_contact")
            if not r.get("ok") or ground is None:
                return None
            return sorted({p for p in ground
                           if "ankle" not in p and "foot" not in p})
        except Exception:
            return None

    def release_band(self):
        """MuJoCo hangs the robot from an elastic band during bring-up and
        the CONSOLE is what normally releases it -- a probe that skips this
        measures the BAND's steadiness, not the robot's. Caught 2026-08-12
        when the MuJoCo baseline read pelvis 0.96 m (band height) with a
        pelvis std of 0.0037. Deterministic sequence borrowed from
        getup_probe: shorten until the feet touch, then release, so it is a
        zero-height step-down instead of a 0.2 m drop. Isaac has no band."""
        if self.backend != "mujoco":
            return True
        z, nf = self.pelvis_z(), self.nonfoot_contacts()
        if z is not None and z < 0.88 and nf is not None:
            self.say(f"band already off (pelvis {z:.3f} m)")
            return True
        self.say(f"pelvis {z} m -- ON THE BAND; lowering then releasing")
        self.mode("idle")
        try:
            self.sim.request("band", length_m=-0.2)
            t_end = time.monotonic() + 12.0
            while time.monotonic() < t_end:
                zz = self.pelvis_z()
                if zz is not None and zz < 0.86:
                    break
                time.sleep(0.3)
            self.sim.request("band", release=True)
        except Exception as exc:
            self.say(f"band release FAILED: {exc!r} -- baseline is band-held")
            return False
        time.sleep(3.0)
        z2 = self.pelvis_z()
        self.say(f"after band release: pelvis {z2} m")
        self.log("band_released", pelvis_m=z2)
        return True

    def pelvis_z(self):
        try:
            r = self.sim.body_pose(["pelvis"])
            if r.get("ok"):
                return float(r["bodies"]["pelvis"]["world_m"][2])
        except Exception:
            pass
        return None

    def camera_frame(self, label):
        """Head-camera JPEG, verified fresh-by-this-call before it is logged
        as evidence (the 2026-08-07 lesson: a silent failure once logged
        day-old pixels as the day's proof). The camera NAME is per-backend:
        Isaac's image server calls it "head"; the MuJoCo model's camera is
        "head_camera" and the sidecar passes the name straight to the
        renderer.

        The file is RUN-STAMPED. Writing a fixed "stand.jpg" made every run
        delete the previous run's proof: on 2026-08-18 eight runs produced
        eight image pairs and five pairs -- including the MuJoCo reference
        row -- were gone by the end of the day, overwritten in place. Evidence
        that a later run can silently destroy is not evidence."""
        name = f"{label}-{self.stamp}.jpg"
        out = ART / name
        cam = "head" if self.backend == "isaac" else "head_camera"
        try:
            r = self.sim.request("camera", camera=cam)
            data = r.get("jpeg_b64") or r.get("jpeg")
            if r.get("ok") and data:
                import base64
                blob = base64.b64decode(data) if isinstance(data, str) else data
                if blob[-2:] == b"\xff\xd9":
                    out.write_bytes(blob)
                    self.log("camera_frame", path=str(out), bytes=len(blob))
                    return True
            self.log("camera_frame_failed", reply_keys=sorted(r or {}))
        except Exception as exc:
            self.log("camera_frame_failed", error=repr(exc))
        self.say(f"camera frame {name} unavailable -- no image evidence")
        return False

    # -- control -----------------------------------------------------------
    def handshake(self):
        """The proven bring-up, verbatim from the console/getup probes."""
        self.link = ZmqSonicLink(port=5556, settle_s=1.0, auto_start=False)
        self.say("publisher bound on tcp://*:5556")
        self.link.start_control()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            self.link._pub.send(encode_planner(MODE_IDLE, (1.0, 0.0, 0.0),
                                               (1.0, 0.0, 0.0), 0.0, -1.0))
            time.sleep(0.05)
        self.say("planner parked in IDLE")
        # The takeover is the one moment MuJoCo protects with an elastic band
        # and Isaac does not; before patch 05 the vendor's own leg policy
        # covered it, and full-body routing removed that. Sample across it.
        for label, t_end in (("attached", time.monotonic() + 12.0),):
            while time.monotonic() < t_end:
                z = self.pelvis_z()
                self.log("takeover_z", label=label, z=z,
                         t=round(time.monotonic(), 2))
                time.sleep(0.5)
            self.say(f"pelvis right after takeover: {self.pelvis_z()}")
        # One timed-out ping must not disable backend-specific recovery for
        # the whole run (Isaac's first reply can lag a slow render).
        for attempt in range(5):
            try:
                self.backend = (self.sim.ping() or {}).get("backend", "mujoco")
                break
            except Exception:
                self.backend = "unknown"
                time.sleep(1.0)
        self.say(f"backend: {self.backend}")
        if not self.controller_is_live():
            self.say("ABORT: SONIC is not driving -- no rt/lowcmd. Check "
                     "policy.log for 'Safety check failed' / 'Lost LowState'. "
                     "Measuring a dead controller and calling the result a "
                     "backend divergence is the exact error this probe exists "
                     "to prevent.")
            self.log("abort", reason="controller not driving")
            return False
        return True

    def controller_is_live(self, need_hz=50.0, timeout_s=15.0):
        """rt/lowcmd actually flowing. SONIC refuses to start when lowstate
        is stale at the moment of the start command ('Safety check failed,
        cannot start control') and then publishes NOTHING -- a silence that
        looks exactly like a backend that cannot balance. Caught 2026-08-12
        on a MuJoCo baseline run whose robot simply hung on the band."""
        t_end = time.monotonic() + timeout_s
        while time.monotonic() < t_end:
            t0 = time.monotonic()
            n0 = len(self.rec.cmd)
            time.sleep(1.0)
            hz = (len(self.rec.cmd) - n0) / max(time.monotonic() - t0, 1e-6)
            if hz >= need_hz:
                self.say(f"controller is driving ({hz:.0f} Hz rt/lowcmd)")
                return True
        self.say(f"rt/lowcmd below {need_hz:.0f} Hz for {timeout_s:.0f}s")
        return False

    def mode(self, name, **kw):
        r = self.link.select_mode(name, **kw)
        self.log("command", mode=name, kwargs=kw, result=r)
        self.say(f"-> mode {name}")
        return r

    def sim_reset(self):
        """Harness reset (Isaac only): rt/reset_pose/cmd '2' = reset all."""
        if self.backend == "unknown":
            # The handshake ping may have raced a slow render; ask again
            # before writing off the only reset mechanism this backend has.
            try:
                self.backend = (self.sim.ping() or {}).get("backend", "unknown")
            except Exception:
                pass
        if self.backend != "isaac":
            return False
        try:
            if self._reset_pub is None:
                from unitree_sdk2py.core.channel import ChannelPublisher
                from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
                self._reset_msg = String_
                pub = ChannelPublisher("rt/reset_pose/cmd", String_)
                pub.Init()               # only a fully built publisher is kept
                self._reset_pub = pub
            self.mode("idle")
            self._reset_pub.Write(self._reset_msg(data="2"))
            self.say("rt/reset_pose/cmd '2' sent (harness, not robot)")
            time.sleep(4.0)
            self.mode("idle")
            time.sleep(4.0)
            z = self.pelvis_z()
            self.say(f"after reset: pelvis {z if z is None else round(z, 3)} m")
            return z is not None and z > 0.70
        except Exception as exc:
            self.say(f"sim reset failed: {exc!r}")
            return False

    # -- phases --------------------------------------------------------------
    def record_phase(self, label, seconds):
        t0 = time.monotonic()
        rtf0 = self.sidecar_times()
        zs, nonfoot_seen = [], set()
        while time.monotonic() - t0 < seconds:
            z = self.pelvis_z()
            if z is not None:
                zs.append(z)
            nf = self.nonfoot_contacts()
            if nf:
                nonfoot_seen.update(nf)
            time.sleep(0.2)
        t1 = time.monotonic()
        rtf1 = self.sidecar_times()
        rtf = None
        if rtf0 and rtf1 and rtf1[0] > rtf0[0]:
            rtf = round((rtf1[1] - rtf0[1]) / (rtf1[0] - rtf0[0]), 3)
        cmd = self.rec.window(self.rec.cmd, t0, t1)
        state = self.rec.window(self.rec.state, t0, t1)
        dq = self.rec.window(self.rec.dq, t0, t1)
        leg_speed_rms = None
        if dq:
            flat = [v for _, row in dq for v in row]
            leg_speed_rms = round((sum(v * v for v in flat) / len(flat)) ** 0.5, 4)
        pelvis_std = None
        pelvis_first = pelvis_last = pelvis_min = pelvis_max = None
        if len(zs) >= 5:
            m = sum(zs) / len(zs)
            pelvis_std = round((sum((z - m) ** 2 for z in zs) / len(zs)) ** 0.5, 4)
            # std alone conflates "oscillating in place" with "falling over":
            # a monotonic 0.5 m drop and a violent wobble score alike.
            pelvis_first, pelvis_last = round(zs[0], 3), round(zs[-1], 3)
            pelvis_min, pelvis_max = round(min(zs), 3), round(max(zs), 3)
        span = max(t1 - t0, 1e-6)      # the real window, not the nominal one:
        tilts = [v for tt, v in self.rec.tilt if t0 <= tt <= t1]
        out = {                        # the loop overruns by the final
            "label": label,            # pelvis_z round trip
            "seconds": round(span, 2),
            "tilt_mean_deg": round(sum(tilts) / len(tilts), 1) if tilts else None,
            "tilt_max_deg": round(max(tilts), 1) if tilts else None,
            "nonfoot_ground_contact": sorted(nonfoot_seen),
            "cmd_msgs": len(cmd), "state_msgs": len(state),
            "cmd_hz": round(len(cmd) / span, 1),
            "state_hz": round(len(state) / span, 1),
            "rtf": rtf, "leg_speed_rms_rad_s": leg_speed_rms,
            "pelvis_std_m": pelvis_std,
            "pelvis_first_m": pelvis_first, "pelvis_last_m": pelvis_last,
            "pelvis_min_m": pelvis_min, "pelvis_max_m": pelvis_max,
            "pelvis_series_m": [round(z, 3) for z in zs],
            "joints": {TRACK[i]: joint_metrics(cmd, state, i)
                       for i in sorted(TRACK)},
        }
        self.log("phase", **out)
        return out


def main() -> int:
    run = Run()
    run.rec.start()
    if not run.handshake():
        return 2

    # Record the knobs that decide the outcome, ALWAYS -- not only when they
    # disagree. Today's runs logged SONIC_TIME_SCALE only on a mismatch, so
    # the scale of every run had to be reconstructed afterwards by dividing
    # SONIC's 500 Hz publish rate by the measured cmd_hz. That reconstruction
    # happened to be possible; a run's configuration should never depend on
    # it being possible.
    run.log("run_config",
            stamp=run.stamp,
            backend=run.backend,
            time_scale=os.environ.get("SONIC_TIME_SCALE"),
            target_rtf=os.environ.get("SONIC_WB_TARGET_RTF"),
            full_body=os.environ.get("SONIC_WB_FULL_BODY"),
            render_every=os.environ.get("SONIC_WB_RENDER_EVERY"))

    run.release_band()
    run.say("phase A: quiet stand, 10 s (jitter + RTF baseline)")
    a = run.record_phase("stand", 10.0)
    run.say(f"stand: cmd {a['cmd_hz']} Hz, state {a['state_hz']} Hz, "
            f"RTF {a['rtf']}, leg speed RMS {a['leg_speed_rms_rad_s']} rad/s, "
            f"pelvis {a['pelvis_first_m']} -> {a['pelvis_last_m']} m "
            f"(min {a['pelvis_min_m']}, max {a['pelvis_max_m']}, "
            f"std {a['pelvis_std_m']}), tilt {a['tilt_mean_deg']}/{a['tilt_max_deg']} deg "
            f"mean/max, non-foot contact {a['nonfoot_ground_contact'] or 'none'}")
    scale = os.environ.get("SONIC_TIME_SCALE")
    if scale and a.get("rtf"):
        needed = 1.0 / a["rtf"] if a["rtf"] > 0 else float("inf")
        if float(scale) < needed * 0.9:
            run.say(f"WARNING: SONIC_TIME_SCALE={scale} but this run measured "
                    f"RTF {a['rtf']}, which needs ~{needed:.1f}. The "
                    f"controller is outrunning the physics and any instability "
                    f"below is EXPECTED, not a backend defect.")
            run.log("scale_mismatch", scale=float(scale), rtf=a["rtf"],
                    needed=round(needed, 2))
    run.camera_frame("stand")

    run.say("phase B: kneel_two_legs -- the leg motion the wiring verdict "
            "rides on")
    run.mode("kneel_two_legs")
    b = run.record_phase("kneel", 8.0)
    for name, m in b["joints"].items():
        run.say(f"  {name}: sweep {m.get('cmd_sweep_rad')} rad, "
                f"corr {m.get('corr')}, rms {m.get('rms_rad')} rad")
    run.camera_frame("kneel")

    run.say("recovering to stand")
    run.mode("idle")
    time.sleep(6.0)
    z = run.pelvis_z()
    if z is None or z <= 0.70:
        if run.backend == "isaac":
            run.sim_reset()
        else:
            run.say(f"not standing after idle (pelvis {z}); leaving as-is")

    # -- verdict, on the knees only ----------------------------------------
    verdicts, reasons = [], []
    for idx in VERDICT_JOINTS:
        name = TRACK[idx]
        m = b["joints"][name]
        sweep, corr, rms = m.get("cmd_sweep_rad"), m.get("corr"), m.get("rms_rad")
        if sweep is None or sweep < CMD_SWEEP_MIN_RAD:
            verdicts.append("inconclusive")
            reasons.append(f"{name}: command never swept ({sweep} rad -- "
                           "planner down?)")
        elif corr is None:
            verdicts.append("inconclusive")
            reasons.append(f"{name}: command swept {sweep} rad but only "
                           f"{m.get('n')} state samples -- state stream too "
                           "thin to judge")
        elif corr >= WIRED_CORR and rms <= WIRED_RMS_RAD:
            verdicts.append("wired")
            reasons.append(f"{name}: tracks (corr {corr}, rms {rms} rad)")
        elif corr < UNWIRED_CORR:
            verdicts.append("not_wired")
            reasons.append(f"{name}: command swept {sweep} rad, state ignored "
                           f"it (corr {corr})")
        else:
            verdicts.append("inconclusive")
            reasons.append(f"{name}: borderline (corr {corr}, rms {rms} rad) "
                           "-- neither tracking nor deaf")
    detail = "; ".join(reasons)
    if all(v == "wired" for v in verdicts):
        verdict, code = f"LEGS WIRED -- {detail}", 0
    elif "not_wired" in verdicts:
        verdict, code = f"LEGS NOT WIRED -- {detail}", 2
    else:
        verdict, code = f"INCONCLUSIVE -- {detail}", 1
    run.log("verdict", verdict=verdict, per_joint=dict(zip(
        (TRACK[i] for i in VERDICT_JOINTS), verdicts)), exit_code=code)
    run.say(f"VERDICT: {verdict}")
    run.say(f"evidence: {run.fh.name}")
    return code


if __name__ == "__main__":
    # A crash must never wear the exit code of a measured verdict: 1 is
    # reserved for an evidence-backed "inconclusive". Failure is 2, as the
    # docstring promises.
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        raise
    except Exception:
        import traceback
        traceback.print_exc()
        print("[tracking] probe crashed before producing a verdict -- "
              "exit 2 (failure), not inconclusive", flush=True)
        sys.exit(2)
