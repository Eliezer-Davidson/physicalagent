"""Live A/B: does ramping the arm reference actually slow the arm down?

    SONIC_FRONTEND=scripts/verify_arm_damping.py ./scripts/sonic_live.sh

"The movements are very fast and sharp" is a claim about JOINT SPEED, so this
measures joint speed. Same pose, same robot, same bring-up; the only thing that
changes is the slew rate on the reference. rt/lowstate reports dq per joint, so
the peak of |dq| over the move is the number that says whether it worked.

A/B rather than a single number, because "1.9 rad/s peak" means nothing on its
own -- the question is whether it is lower than it was, and by how much. The
undamped arm is reproduced by asking for a slew far above anything the policy
can track, which is exactly what the old code did by sending a step.
"""
from __future__ import annotations

import sys
import time

import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from physicalagent.robots import sonic_upper_body as ub  # noqa: E402
from physicalagent.robots import sonic_zmq as z  # noqa: E402
from physicalagent.robots.sonic_zmq import MODE_IDLE, ZmqSonicLink, encode_planner  # noqa: E402
from scripts.sonic_console import _open_telemetry, release_band  # noqa: E402

ARM_JOINTS = [n for n in ub.JOINT_NAMES if "shoulder" in n or "elbow" in n]
SAMPLE_S = 0.02
WATCH_S = 4.0
SETTLE_S = 4.0

# 60 rad/s is unreachable for the policy, so the reference arrives in one tick:
# byte-for-byte what the old step-change code put on the wire.
UNDAMPED = 60.0


def peak_speed(telemetry, link, values, slew) -> dict:
    """Command a pose and watch how fast the arm joints actually move."""
    link.set_upper_body(None)
    time.sleep(SETTLE_S)
    link.set_upper_body(values, slew_rad_s=slew)

    peak = {n: 0.0 for n in ARM_JOINTS}
    t_end = time.monotonic() + WATCH_S
    samples = 0
    while time.monotonic() < t_end:
        rows = telemetry.joints()
        if rows.get("available") and rows.get("fresh"):
            samples += 1
            for n in ARM_JOINTS:
                peak[n] = max(peak[n], abs(rows["joints"][n]["speed_rad_s"]))
        time.sleep(SAMPLE_S)
    return {"peak": peak, "worst": max(peak.values()), "samples": samples}


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
        print("no telemetry -- joint speed cannot be measured")
        return 2
    time.sleep(1.5)

    pose = ub.pose("arms_forward")
    runs = []
    for label, slew in (("undamped (step, as before)", UNDAMPED),
                        (f"damped ({z.UPPER_BODY_SLEW_RAD_S} rad/s)",
                         z.UPPER_BODY_SLEW_RAD_S)):
        out = peak_speed(telemetry, link, pose, slew)
        runs.append((label, out))
        print(f"\n{label}: {out['samples']} samples over {WATCH_S:.0f}s")
        for n in ARM_JOINTS:
            print(f"    {n:<24} peak |dq| {out['peak'][n]:>6.2f} rad/s")
        print(f"    {'WORST':<24}           {out['worst']:>6.2f} rad/s")

    link.set_upper_body(None)
    link.stop()
    link.close()

    before, after = runs[0][1]["worst"], runs[1][1]["worst"]
    print("\n" + "-" * 66)
    print(f"peak joint speed  {before:.2f} -> {after:.2f} rad/s "
          f"({(1 - after / before) * 100:.0f}% slower)" if before > 0 else
          "no motion measured")
    if after >= before:
        print("THE DAMPER DID NOTHING. Do not claim the arms are smoother.")
        return 1
    print("Same pose, same robot; only the reference slew changed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
