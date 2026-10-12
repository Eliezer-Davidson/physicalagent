#!/usr/bin/env python
"""OPEN-LOOP routing check: does joint N of rt/lowcmd move joint N of the sim?

    # sim only, NO SONIC policy running:
    .venv_sim/bin/python scripts/isaac_routing_probe.py

WHY OPEN LOOP (2026-08-12). scripts/sonic_tracking_probe.py asks the same
question through SONIC's own feedback loop, and that cannot separate "the
legs are not wired" from "the legs are wired and the controller is
diverging" -- in the live run both knees showed corr ~0.3 while the robot
thrashed at IDLE (leg speed RMS 7.65 rad/s, RTF 0.179). Correlation needs a
plant that is tracking something; a diverging plant makes it meaningless.

So: no controller at all. This script IS the controller for ~30 s. It holds
every joint rigid at its measured rest pose, then adds a SIGNATURE -- a
distinct displacement per joint, all magnitudes different -- and reports
which joint actually moved and by how much. Three outcomes are
distinguishable, which is the whole point:

    every signature joint moves its own amount   -> routing CORRECT
    nothing below the waist moves                -> legs NOT routed
    joint A moves by joint B's amount            -> order PERMUTED

SAFETY. This publishes rt/lowcmd, so it must never meet a robot or a second
controller:
  * refuses if any g1_deploy_onnx_ref process is alive (it would fight us
    for the same topic at 500 Hz),
  * requires the simulator sidecar to answer on 5570 -- hardware has no
    sidecar, so this is the hardware interlock,
  * commands only small offsets from the pose the robot is ALREADY in.
"""
from __future__ import annotations

import argparse
import json
import pathlib
import subprocess
import sys
import threading
import time

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from physicalagent.robots.sonic_sim_client import SimClient  # noqa: E402

ART = REPO / "artifacts" / "routing"

# 29 body joints in G1 hardware order = rt/lowcmd motor index. Same list the
# patch maps from; pinned against policy_parameters.hpp:143-176.
HW = [
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee",
    "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee",
    "right_ankle_pitch", "right_ankle_roll",
    "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
    "left_elbow", "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
    "right_elbow", "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
]
N = len(HW)

# hw index -> commanded offset (rad). Every magnitude DIFFERENT so a permuted
# mapping shows up as the wrong amplitude on the wrong joint, and signs mixed
# so a sign flip shows up too. Small: this is a nudge, not a motion.
SIGNATURE = {
    3: +0.30,    # left_knee
    9: -0.20,    # right_knee
    0: -0.25,    # left_hip_pitch
    10: +0.15,   # right_ankle_pitch
    12: +0.35,   # waist_yaw
    15: -0.40,   # left_shoulder_pitch
    25: +0.45,   # right_elbow
    21: +0.50,   # left_wrist_yaw
}
# Rigid hold: we are the only controller, so stiffness is ours to choose.
# Stiffer than SONIC's own gains on purpose -- a statue makes the signature
# unambiguous. Legs/waist first, then arms.
HOLD_KP = [200.0] * 15 + [80.0] * 14
HOLD_KD = [5.0] * 15 + [2.0] * 14

MOVED_RAD = 0.05             # below this a joint did not move
TRACK_LO, TRACK_HI = 0.4, 1.6  # measured/commanded ratio counted as tracking


class Bus:
    def __init__(self):
        self.lock = threading.Lock()
        self.q = None
        self.n_state = 0
        self._stop = threading.Event()
        self._pub_thread = None
        self.target = None
        self.n_pub = 0

    def start(self):
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber)
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.utils.crc import CRC
        try:
            ChannelFactoryInitialize(0, "lo")
        except Exception:
            pass
        # Bound to self: an unreferenced subscriber gets collected mid-run
        # (that is what froze the M0 probe's counter and minted "79 Hz").
        self._sub = ChannelSubscriber("rt/lowstate", LowState_)
        self._sub.Init(self._on_state, 10)
        self._pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self._pub.Init()
        self._mk, self._crc = unitree_hg_msg_dds__LowCmd_, CRC()

    def _on_state(self, msg):
        with self.lock:
            self.q = [float(msg.motor_state[i].q) for i in range(N)]
            self.n_state += 1

    def read_q(self, timeout_s=10.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            with self.lock:
                if self.q is not None:
                    return list(self.q)
            time.sleep(0.05)
        return None

    def _publish_loop(self):
        while not self._stop.is_set():
            tgt = self.target
            if tgt is not None:
                cmd = self._mk()
                for i in range(N):
                    m = cmd.motor_cmd[i]
                    m.q, m.dq, m.tau = tgt[i], 0.0, 0.0
                    m.kp, m.kd, m.mode = HOLD_KP[i], HOLD_KD[i], 1
                # The vendor sim drops any lowcmd whose CRC does not match.
                cmd.crc = self._crc.Crc(cmd)
                self._pub.Write(cmd)
                self.n_pub += 1
            time.sleep(0.005)          # 200 Hz; sim samples latest-wins

    def command(self, target):
        self.target = list(target)
        if self._pub_thread is None:
            self._pub_thread = threading.Thread(target=self._publish_loop,
                                                daemon=True)
            self._pub_thread.start()

    def settle_mean(self, seconds):
        """Mean joint vector over the LAST part of a hold -- the settled
        pose, not a peak. Peaks measure overshoot; we want where it ended."""
        t_end = time.monotonic() + seconds
        acc, n = [0.0] * N, 0
        while time.monotonic() < t_end:
            with self.lock:
                q = None if self.q is None else list(self.q)
            if q is not None:
                acc = [a + v for a, v in zip(acc, q)]
                n += 1
            time.sleep(0.02)
        return None if n == 0 else [a / n for a in acc]

    def stop(self):
        self._stop.set()


def refuse_if_unsafe():
    r = subprocess.run(["pgrep", "-f", "[g]1_deploy_onnx_ref"],
                       capture_output=True, text=True)
    if r.stdout.strip():
        print("REFUSING: a SONIC deploy binary is running (pids "
              f"{r.stdout.split()}). It publishes rt/lowcmd at 500 Hz and "
              "would fight this probe for the same topic. Stop it first.")
        return False
    try:
        ping = SimClient(port=5570).ping()
    except Exception as exc:
        print(f"REFUSING: no simulator sidecar on 5570 ({exc}). This probe "
              "drives rt/lowcmd directly and must never run against anything "
              "but a simulator.")
        return False
    print(f"[routing] sidecar backend: {ping.get('backend')} -- simulator "
          "confirmed")
    return True


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--hold-s", type=float, default=14.0,
                    help="wall seconds per phase (RTF<1 needs generous holds)")
    ap.add_argument("--tag", default="", help="label for the evidence file")
    args = ap.parse_args()

    if not refuse_if_unsafe():
        return 2

    ART.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    name = f"routing-{stamp}{'-' + args.tag if args.tag else ''}.jsonl"
    fh = (ART / name).open("w")

    def log(kind, **kw):
        fh.write(json.dumps({"kind": kind, "wall": round(time.time(), 3),
                             **kw}) + "\n")
        fh.flush()

    bus = Bus()
    bus.start()
    rest = bus.read_q()
    if rest is None:
        print("no rt/lowstate -- is the sim running?")
        return 2
    print(f"[routing] rest pose read ({bus.n_state} lowstate msgs)")

    print(f"[routing] phase 1: hold rigid at rest for {args.hold_s:.0f}s")
    bus.command(rest)
    base = bus.settle_mean(args.hold_s)
    log("baseline", q=[round(v, 4) for v in base], published=bus.n_pub)

    target = list(base)
    for i, d in SIGNATURE.items():
        target[i] = base[i] + d
    print(f"[routing] phase 2: signature on {len(SIGNATURE)} joints, "
          f"{args.hold_s:.0f}s")
    bus.command(target)
    after = bus.settle_mean(args.hold_s)
    bus.stop()

    rows = []
    for i in range(N):
        cmd_d = SIGNATURE.get(i, 0.0)
        got_d = after[i] - base[i]
        ratio = (got_d / cmd_d) if cmd_d else None
        rows.append({"hw": i, "joint": HW[i], "cmd_rad": round(cmd_d, 3),
                     "measured_rad": round(got_d, 3),
                     "ratio": None if ratio is None else round(ratio, 2)})
    log("response", published=bus.n_pub, state_msgs=bus.n_state, rows=rows)

    print(f"\n{'joint':<24}{'commanded':>10}{'measured':>10}{'ratio':>8}")
    for r in rows:
        mark = ""
        if r["cmd_rad"]:
            mark = ("  <-- TRACKS" if r["ratio"] is not None
                    and TRACK_LO <= r["ratio"] <= TRACK_HI
                    else "  <-- NO RESPONSE"
                    if abs(r["measured_rad"]) < MOVED_RAD else "  <-- OFF")
        elif abs(r["measured_rad"]) >= MOVED_RAD:
            mark = "  (crosstalk)"
        print(f"{r['joint']:<24}{r['cmd_rad']:>10.3f}"
              f"{r['measured_rad']:>10.3f}"
              f"{'' if r['ratio'] is None else format(r['ratio'], '.2f'):>8}{mark}")

    legs = [r for r in rows if r["cmd_rad"] and r["hw"] < 15]
    arms = [r for r in rows if r["cmd_rad"] and r["hw"] >= 15]

    def tracking(rs):
        return [r for r in rs if r["ratio"] is not None
                and TRACK_LO <= r["ratio"] <= TRACK_HI]

    leg_ok, arm_ok = tracking(legs), tracking(arms)
    verdict = (f"legs+waist {len(leg_ok)}/{len(legs)} tracking, "
               f"arms {len(arm_ok)}/{len(arms)} tracking")
    if len(leg_ok) == len(legs) and len(arm_ok) == len(arms):
        line, code = f"ROUTING CORRECT -- {verdict}", 0
    elif not leg_ok and arm_ok:
        line, code = (f"LEGS NOT ROUTED (arms-only teleop mode?) -- {verdict}",
                      2)
    else:
        line, code = f"PARTIAL / SUSPECT -- {verdict}", 1
    log("verdict", verdict=line, exit_code=code)
    print(f"\n[routing] VERDICT: {line}")
    print(f"[routing] evidence: {fh.name}")
    fh.close()
    return code


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        raise
    except Exception:
        import traceback
        traceback.print_exc()
        print("[routing] probe crashed before a verdict -- exit 2", flush=True)
        sys.exit(2)
