#!/usr/bin/env python
"""Is the SONIC + MuJoCo stack actually healthy? Counts real DDS traffic.

    python scripts/sonic_check.py [seconds]

This exists because "the robot fell over" is a SYMPTOM with several possible
causes, and the fastest way to tell them apart is to look at which of the two
message streams is missing:

    lowstate == 0  ->  the SIMULATOR is not publishing robot state.
                       Usually the on-screen viewer failed to start (GLFW picks
                       the Wayland backend and MuJoCo reports OpenGL 0x502).
                       Run headless.
    lowcmd   == 0  ->  the sim is fine but the POLICY is not commanding.
                       Check policy.log for "Safety check failed, cannot start
                       control" — that means it never saw lowstate — or for the
                       TensorRT engine still loading (~25 s).
    both > 0       ->  the stack is healthy. If the robot still misbehaves the
                       problem is the commands being sent, not the plumbing.

Healthy reference measured 2026-07-31: ~1800 lowstate and ~5400 lowcmd per 10 s.
"""
from __future__ import annotations

import sys
import time

WINDOW = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0


def main() -> int:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_

    ChannelFactoryInitialize(0, "lo")
    n = {"state": 0, "cmd": 0, "q": None}

    def on_state(msg):
        n["state"] += 1
        n["q"] = [round(msg.motor_state[i].q, 3) for i in (0, 3, 9)]

    def on_cmd(_):
        n["cmd"] += 1

    ChannelSubscriber("rt/lowstate", LowState_).Init(on_state, 10)
    ChannelSubscriber("rt/lowcmd", LowCmd_).Init(on_cmd, 10)

    print(f"listening {WINDOW:.0f}s on rt/lowstate and rt/lowcmd ...")
    time.sleep(WINDOW)

    print(f"  lowstate : {n['state']:6d}   (simulator -> policy)")
    print(f"  lowcmd   : {n['cmd']:6d}   (policy -> simulator)")
    print(f"  sample joints (L-hip-pitch, L-knee, R-knee): {n['q']}")

    if not n["state"]:
        print("\nUNHEALTHY: no robot state. The simulator is not publishing —"
              "\n  run it headless (--no-enable-onscreen, MUJOCO_GL=egl); the"
              "\n  on-screen viewer cannot create a GL context on this machine.")
        return 1
    if not n["cmd"]:
        print("\nUNHEALTHY: state flows but nothing is commanding the joints."
              "\n  grep 'Safety check failed' in the policy log; also allow ~25 s"
              "\n  for the TensorRT engine to load before judging.")
        return 1
    print("\nHEALTHY: both directions are flowing; the policy is driving the robot.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
