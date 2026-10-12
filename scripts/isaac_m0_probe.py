#!/usr/bin/env python
"""M0 GATE: can SONIC balance the G1 inside Unitree's Isaac sim at all?

Run with the Isaac sim and the SONIC deploy binary already up, under the ISAAC
interpreter (this imports unitree_sdk2py, which the stdlib-only project venv
does not have):

    "$ISAAC_PY" scripts/isaac_m0_probe.py     # ISAAC_PY = the Isaac-side interpreter

WHAT IT ASKS
    Every task shipped in unitree_sim_isaaclab is manipulation. No locomotion
    policy has ever driven that asset there. This answers whether SONIC's
    whole-body policy holds the robot up in it -- before anything is built on
    the assumption that it does.

WHAT IT MEASURES, AND WHAT IT DELIBERATELY DOES NOT
    UPRIGHT   pelvis IMU tilt from rt/lowstate, which the vendor sim publishes.
    COMMANDED rt/lowcmd rate: proof SONIC is actually driving the joints rather
              than the robot merely happening to stand in its spawn pose.

    It does NOT measure pelvis HEIGHT. Height comes from rt/odostate, which the
    vendor sim does not publish yet (that is Task 4). So this gate can say
    "upright and commanded" and must not say "standing" -- a robot on its knees
    is upright too. Height is verified after Task 4, by re-running this probe.

    The distinction is the point. Reporting "standing" from tilt alone would be
    exactly the kind of plausible-but-unfounded claim the fall detector work on
    2026-08-06 was spent removing.
"""
from __future__ import annotations

import json
import pathlib
import subprocess
import sys
import time

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from physicalagent.robots.sonic_telemetry import UnitreeTelemetry  # noqa: E402
from physicalagent.robots.sonic_zmq import (  # noqa: E402
    MODE_IDLE, ZmqSonicLink, encode_planner)

ART = REPO / "artifacts" / "isaac_m0"
HOLD_S = 30.0
UPRIGHT_DEG = 30.0
MIN_CMD_HZ = 50.0          # SONIC runs its loop far faster; this only asks "is it driving?"


def shot(name: str) -> None:
    try:
        subprocess.run(["grab-window", "Isaac", str(ART / name)],
                       timeout=25, check=False,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass


def main() -> int:
    ART.mkdir(parents=True, exist_ok=True)
    link = ZmqSonicLink(port=5556, settle_s=1.0, auto_start=False)
    link.start_control()

    tel = UnitreeTelemetry(init_channel=True, interface="lo")
    if not tel.wait_for_data(timeout_s=10.0):
        print("[m0] FAIL: no rt/lowstate within 10 s. The sim is not publishing "
              "on domain 0 / lo -- check the dds_master domain patch.")
        return 2

    # Count rt/lowcmd ourselves: it is the only proof the policy is driving.
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
    cmds = {"n": 0}
    ChannelSubscriber("rt/lowcmd", LowCmd_).Init(lambda _m: cmds.__setitem__("n", cmds["n"] + 1), 10)

    shot("m0_0_start.png")
    samples, t0 = [], time.monotonic()
    with (ART / "m0.jsonl").open("w") as fh:
        while time.monotonic() - t0 < HOLD_S:
            link._pub.send(encode_planner(MODE_IDLE, (1.0, 0.0, 0.0),
                                          (1.0, 0.0, 0.0), 0.0, -1.0))
            imu = tel.imu()
            pose = tel.pose()
            row = {"t": round(time.monotonic() - t0, 3),
                   "tilt_deg": imu.get("tilt_deg") if imu.get("available") else None,
                   "imu_fresh": imu.get("fresh"),
                   "height_available": bool(pose.get("available")),
                   "height_m": pose.get("height_m"),
                   "lowcmd_count": cmds["n"]}
            fh.write(json.dumps(row) + "\n")
            samples.append(row)
            time.sleep(0.1)
    shot("m0_1_end.png")
    link.close()

    elapsed = time.monotonic() - t0
    cmd_hz = cmds["n"] / elapsed
    tilts = [s["tilt_deg"] for s in samples if s["tilt_deg"] is not None]
    heights = [s["height_m"] for s in samples if s["height_available"] and s["height_m"] is not None]

    if not tilts:
        print("[m0] INCONCLUSIVE: no usable IMU sample in 30 s.")
        return 3

    upright = max(tilts) <= UPRIGHT_DEG
    commanded = cmd_hz >= MIN_CMD_HZ
    ok = upright and commanded

    print(f"[m0] tilt: max {max(tilts):.1f} deg, final {tilts[-1]:.1f} deg "
          f"(limit {UPRIGHT_DEG:.0f}) -> {'UPRIGHT' if upright else 'TOPPLED'}")
    print(f"[m0] rt/lowcmd: {cmds['n']} in {elapsed:.0f}s = {cmd_hz:.0f} Hz "
          f"-> {'SONIC IS DRIVING' if commanded else 'NOT COMMANDED'}")
    if heights:
        print(f"[m0] pelvis height IS available: min {min(heights):.3f} m "
              f"(rt/odostate is publishing)")
    else:
        print("[m0] pelvis height NOT available (rt/odostate absent, expected "
              "before Task 4). This gate therefore claims UPRIGHT + COMMANDED, "
              "NOT 'standing' -- a kneeling robot is upright too.")
    print(f"[m0] {'PASS' if ok else 'FAIL'}  (evidence: {ART}/m0.jsonl)")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
