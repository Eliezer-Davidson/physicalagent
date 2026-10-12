"""Live proof that goto() puts the robot where it was told.

    SONIC_FRONTEND=scripts/verify_goto.py ./scripts/sonic_live.sh

WHAT ONLY THIS CAN ANSWER
    The offline tests drive the control law to convergence against a robot that
    goes exactly where it is pointed. Two things they cannot check:

      1. the OFFSET between the policy's heading frame and odometry. The policy
         anchors its frame to the base quaternion at init; odometry is world.
         Whether they coincide is an empirical fact about this spawn pose, and
         assuming it is how you get a navigator that works only here.
      2. whether the real robot tracks a commanded course at all.

    So this calibrates first, prints the measured offset, and only then drives
    to targets and measures how close it got.

TARGETS ARE RELATIVE TO WHEREVER IT STARTS
    Absolute coordinates would encode this spawn point into the test.
"""
from __future__ import annotations

import math
import sys
import time

import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from physicalagent.robots import sonic_navigate as nav  # noqa: E402
from physicalagent.robots.sonic_zmq import MODE_IDLE, ZmqSonicLink, encode_planner  # noqa: E402
from scripts.sonic_console import _open_telemetry, release_band  # noqa: E402

# (label, forward, left) in metres from the start pose. Short legs: without
# obstacle avoidance a long one is a bet on an empty floor, and the thing under
# test is closed-loop convergence, which a 1.5 m leg shows as well as a 5 m one.
LEGS = (("forward 1.5 m", 1.5, 0.0),
        ("left 1.0 m", 1.5, 1.0),
        ("back to start", 0.0, 0.0))


def main() -> int:
    link = ZmqSonicLink(port=5556, settle_s=1.0, auto_start=False)
    link.start_control()
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        link._pub.send(encode_planner(MODE_IDLE, (1.0, 0.0, 0.0),
                                      (1.0, 0.0, 0.0), 0.0, -1.0))
        time.sleep(0.05)
    if not release_band():
        print("bring-up failed -- nothing measured")
        return 2

    telemetry = _open_telemetry()
    if telemetry is None:
        print("no telemetry -- goto cannot run, and neither can this check")
        return 2
    time.sleep(1.5)

    navigator = nav.Navigator(link, telemetry)
    origin, _ = navigator._pose()
    if origin is None:
        print("no pose at start -- nothing measured")
        return 2
    print(f"\nstart pose  x={origin[0]:+.2f} y={origin[1]:+.2f} "
          f"heading={math.degrees(origin[2]):+.0f}d")

    print("\ncalibrating the policy-frame / odometry offset "
          "(walks ~0.8 m straight)...")
    cal = navigator.calibrate()
    for k, v in cal.items():
        print(f"    {k}: {v}")
    if not cal.get("ok"):
        print("\nCALIBRATION FAILED -- every goto below would be steering in an "
              "unknown frame, so they are not run.")
        link.stop()
        link.close()
        return 1

    print(f"\n{'leg':<18}{'target':>16}{'reached':>16}{'error':>8}  result")
    print("-" * 78)
    failures = []
    for label, fwd, left in LEGS:
        # legs are expressed in the START frame, so rotate them into odometry
        c, s = math.cos(origin[2]), math.sin(origin[2])
        gx = origin[0] + fwd * c - left * s
        gy = origin[1] + fwd * s + left * c
        out = navigator.go(nav.Goal(gx, gy), timeout_s=90)
        end = out.get("end_m") or {}
        err = out.get("final_error_m")
        ok = bool(out.get("ok"))
        if not ok:
            failures.append(f"{label} ({out['reason']})")
        target_s = f"{gx:+.2f},{gy:+.2f}"
        reached_s = (f"{end['x']:+.2f},{end['y']:+.2f}" if end else "unreadable")
        err_s = f"{err:.2f}" if err is not None else "?"
        print(f"{label:<18}{target_s:>16}{reached_s:>16}{err_s:>8}  "
              f"{'OK' if ok else out['reason']}")
        time.sleep(1.0)

    link.stop()
    link.close()
    print("-" * 78)
    if failures:
        print(f"FAILED {len(failures)}/{len(LEGS)}:")
        for f in failures:
            print(f"    {f}")
        return 1
    print(f"All {len(LEGS)} legs arrived within {nav.POS_TOL_M} m, "
          f"frame offset {cal['frame_offset_deg']}d (measured).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
