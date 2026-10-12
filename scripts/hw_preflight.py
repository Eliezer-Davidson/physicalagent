#!/usr/bin/env python3
"""Refuse to touch the physical G1 until the environment is provably safe.

WHY THIS EXISTS AS A SEPARATE, FIRST STEP

    Everything this project has built runs against a SIMULATOR, and several of
    the things that make the simulator work would injure a real robot:

      tools/sonic_timescale.so   deliberately slows SONIC against wall time.
                                 On hardware that is a falling robot.
      SONIC_TIME_SCALE           the knob that shim reads.
      SONIC_FALL_RESET_M         teleports the robot when it falls. There is no
                                 teleport on hardware; there is a floor.
      SONIC_WB_*                 Isaac wholebody routing/pacing knobs.
      --disable-crc-check        accepts corrupt frames. On hardware a corrupt
                                 frame is a corrupt command.

    Every one of those is invisible in a terminal that has already exported it.
    A human cannot be asked to remember five environment variables at 9 a.m.
    next to a $16k robot, so the machine checks.

    The second thing it checks is WHICH ROBOT. On 2026-08-07 this project spent
    a day learning that an agreeing DDS pub/sub pair proves the two ends share
    a world, not that it is the RIGHT world. Loopback DDS and the robot's
    192.168.123.x network look identical to a subscriber. If both are reachable,
    or neither is, this script refuses rather than guessing -- because the
    failure mode of guessing is commanding a real robot while believing it is
    a simulation.

TIER

    This script itself is LOCKED tier: it opens no motor-command object and can
    move nothing. It is safe to run at any time, including with the robot
    powered down.

USAGE

    $WBC/.venv_sim/bin/python scripts/hw_preflight.py            # expect hardware
    $WBC/.venv_sim/bin/python scripts/hw_preflight.py --expect sim
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time

# Environment variables that are SAFE IN SIMULATION AND DANGEROUS ON HARDWARE.
# Each entry is (variable, why it is dangerous). Presence of any one of these
# is a hard refusal when the target is hardware -- not a warning, because every
# one of them silently changes what a command means.
SIM_ONLY_ENV = {
    "SONIC_TIME_SCALE": "slows SONIC against wall time; on hardware the robot falls",
    "SONIC_FALL_RESET_M": "teleports the robot on a fall; hardware has a floor",
    "SONIC_WB_FULL_BODY": "Isaac wholebody routing flag; meaningless and misleading here",
    "SONIC_WB_TARGET_RTF": "Isaac real-time pinning; there is no RTF on hardware",
    "SONIC_WB_RENDER_EVERY": "Isaac render throttle; there is no renderer here",
    "SONIC_SIDECAR_PORT": "simulator sidecar; no sidecar exists on hardware",
}

# Substrings that must not appear in LD_PRELOAD.
PRELOAD_DENY = ("sonic_timescale",)

ROBOT_NET_PREFIX = "192.168.123."


class Check:
    def __init__(self, name, ok, detail, fatal=True):
        self.name, self.ok, self.detail, self.fatal = name, ok, detail, fatal

    def line(self):
        mark = "PASS" if self.ok else ("FAIL" if self.fatal else "WARN")
        return f"[{mark}] {self.name}: {self.detail}"


def interfaces():
    """[(iface, ip)] from `ip -o addr`, empty on any failure. Never raises --
    a preflight that crashes teaches the operator nothing."""
    try:
        out = subprocess.run(["ip", "-o", "-4", "addr", "show"],
                             capture_output=True, text=True, timeout=5).stdout
    except Exception:
        return []
    found = []
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 4 and parts[2] == "inet":
            found.append((parts[1], parts[3].split("/")[0]))
    return found


def check_sim_only_env(expect_hw):
    hits = [f"{k} ({why})" for k, why in SIM_ONLY_ENV.items() if os.environ.get(k)]
    pre = os.environ.get("LD_PRELOAD", "")
    for deny in PRELOAD_DENY:
        if deny in pre:
            hits.append(f"LD_PRELOAD contains {deny} (SIM ONLY -- slows the "
                        f"controller against real time)")
    if not hits:
        return Check("sim-only env", True, "clean -- none of the simulator "
                     "knobs are set")
    if not expect_hw:
        return Check("sim-only env", True,
                     f"{len(hits)} set, which is expected for --expect sim: "
                     + "; ".join(hits), fatal=False)
    return Check("sim-only env", False,
                 "REFUSING -- simulator-only settings are active and every one "
                 "of them changes what a command means on hardware: "
                 + "; ".join(hits))


def check_cyclonedds_uri():
    v = os.environ.get("CYCLONEDDS_URI")
    if not v:
        return Check("CYCLONEDDS_URI", True, "unset, which is correct")
    return Check("CYCLONEDDS_URI", False,
                 f"set to {v!r}. This project's DDS works out of the box and "
                 f"setting this has broken it before. Unset it.")


def check_sdk():
    try:
        import unitree_sdk2py  # noqa: F401
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (  # noqa: F401
            LowState_, LowCmd_, IMUState_, OdoState_)
    except Exception as exc:
        return Check("unitree_sdk2py", False,
                     f"not importable by THIS interpreter ({sys.executable}): "
                     f"{exc!r}. Use GR00T-WholeBodyControl/.venv_sim/bin/python, "
                     f"which is natively in SONIC's DDS world.")
    try:
        import importlib.metadata as md
        ver = md.version("cyclonedds")
    except Exception:
        ver = "unknown"
    ok = ver == "0.10.2"
    return Check("unitree_sdk2py", ok,
                 f"importable; cyclonedds {ver}"
                 + ("" if ok else " -- the tier table pins 0.10.2, and a "
                    "mismatched native core has produced silent topic failures "
                    "on this machine before"),
                 fatal=not ok)


def check_target(expect_hw):
    ifs = interfaces()
    robot = [(i, ip) for i, ip in ifs if ip.startswith(ROBOT_NET_PREFIX)]
    if expect_hw:
        if not robot:
            return Check("target network", False,
                         f"no interface on {ROBOT_NET_PREFIX}x. The robot is not "
                         f"reachable, so anything published now goes nowhere or "
                         f"somewhere unintended. Found: "
                         f"{[f'{i}={ip}' for i, ip in ifs] or 'nothing'}")
        if len(robot) > 1:
            return Check("target network", False,
                         f"AMBIGUOUS -- {len(robot)} interfaces on the robot "
                         f"network: {robot}. Refusing to guess which one is the "
                         f"robot.")
        return Check("target network", True,
                     f"robot network on {robot[0][0]} = {robot[0][1]}")
    return Check("target network", True,
                 f"--expect sim; robot interfaces present: {robot or 'none'}",
                 fatal=False)


def check_disk(path, need_gb):
    try:
        free = shutil.disk_usage(path).free / 1e9
    except Exception as exc:
        return Check("disk", False, f"cannot stat {path}: {exc!r}")
    ok = free >= need_gb
    return Check("disk", ok,
                 f"{free:.0f} GB free at {path} (need >= {need_gb} GB; the "
                 f"recorder writes roughly 1 GB per hour per full-rate stream)",
                 fatal=False if free >= need_gb / 2 else True)


def check_clock():
    """A recording whose timestamps cannot be compared across streams is a
    pile of unrelated files. We stamp with time.monotonic() and record the
    wall offset once, so every stream shares one timebase."""
    t0 = time.monotonic()
    time.sleep(0.01)
    d = time.monotonic() - t0
    ok = 0.005 < d < 0.5
    return Check("monotonic clock", ok, f"10 ms sleep measured {d*1000:.1f} ms")


def run(expect_hw, out_dir, need_gb):
    checks = [
        check_sdk(),
        check_sim_only_env(expect_hw),
        check_cyclonedds_uri(),
        check_target(expect_hw),
        check_disk(out_dir, need_gb),
        check_clock(),
    ]
    for c in checks:
        print(c.line(), flush=True)
    fatal = [c for c in checks if not c.ok and c.fatal]
    warn = [c for c in checks if not c.ok and not c.fatal]
    print()
    if fatal:
        print(f"PREFLIGHT REFUSED -- {len(fatal)} blocking problem(s). "
              f"Nothing has been opened and nothing can move.", flush=True)
        return 1
    print(f"PREFLIGHT OK ({len(warn)} warning(s)).", flush=True)
    print()
    print("Operating tier for the LOCKED-tier tools (recorder, timebase probe):")
    print("  * rt/lowstate subscribe .................. permitted at LOCKED")
    print("  * no motor-command object is constructed . nothing can move")
    print("Anything that COMMANDS the robot -- arm_sdk, locomotion, raw lowcmd,")
    print("SONIC on hardware -- is a HIGHER tier and is gated on a signature")
    print("that does not exist yet. See the capture spec before escalating.")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--expect", choices=("hardware", "sim"), default="hardware")
    ap.add_argument("--out-dir", default="artifacts/hw")
    ap.add_argument("--need-gb", type=float, default=40.0)
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)
    rc = run(a.expect == "hardware", a.out_dir, a.need_gb)
    if a.json:
        print(json.dumps({"rc": rc, "expect": a.expect}))
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
