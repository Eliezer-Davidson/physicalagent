"""Live proof that our arm kinematics match the simulator, and that reach_to
puts the hand where it was asked.

    SONIC_FRONTEND=scripts/verify_reach.py ./scripts/sonic_live.sh

TWO SEPARATE CLAIMS, CHECKED SEPARATELY
    A. Our FK is right. sonic_arm_kin recomputes the arm chain in pure python
       from the MJCF; MuJoCo computes the same thing from the same file. They
       must agree to millimetres for the SAME measured joint angles. This is a
       pure maths check and has nothing to do with the controller.

    B. reach_to arrives. Solve for a target, command it, wait, and measure where
       the hand actually went. This one involves the balance policy, which
       tracks a reference rather than servoing to it, so the bar is looser and
       the number is reported rather than merely passed.

    Reporting them together would let a tracking miss look like a kinematics
    bug, or worse, let correct kinematics paper over a controller that never
    moved.
"""
from __future__ import annotations

import math
import sys
import time

import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from physicalagent.robots import sonic_arm_kin as kin  # noqa: E402
from physicalagent.robots import sonic_upper_body as ub  # noqa: E402
from physicalagent.robots.sonic_zmq import MODE_IDLE, ZmqSonicLink, encode_planner  # noqa: E402
from scripts.sonic_console import _open_sim, _open_telemetry, release_band  # noqa: E402

FK_TOL_M = 0.005          # pure maths: millimetres or it is wrong
REACH_TOL_M = 0.10        # through the policy: centimetres, and reported

# Pelvis-relative targets, chosen inside the arm's ~0.59 m span and away from
# the torso so a miss is a miss and not a self-collision.
TARGETS = (("left", (0.35, 0.22, 0.25)),
           ("right", (0.35, -0.22, 0.25)),
           ("left", (0.40, 0.10, -0.05)),
           ("right", (0.30, -0.30, 0.35)))


def measured_chain(telemetry, side):
    rows = telemetry.joints()
    if not rows.get("available") or not rows.get("fresh"):
        return None
    j = rows["joints"]
    return [math.radians(j[l.joint]["angle_deg"]) for l in kin.LINKS[side]]


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
    sim = _open_sim(5570)
    if telemetry is None or sim is None:
        print("need BOTH telemetry (joint angles) and the sidecar (MuJoCo's own "
              "answer) to check kinematics -- nothing measured")
        return 2
    time.sleep(1.5)
    failures = []

    # --- A. our FK against MuJoCo's, for the same measured angles ------------
    print("\nA. pure kinematics: our FK vs MuJoCo, same measured joint angles")
    print(f"   {'body':<26}{'ours (pelvis)':>26}{'mujoco':>26}{'err':>8}")
    print("   " + "-" * 84)
    bodies = ["left_wrist_yaw_link", "right_wrist_yaw_link"]
    truth = sim.body_pose(bodies).get("bodies", {})
    for side in ("left", "right"):
        chain = measured_chain(telemetry, side)
        if chain is None:
            print("   telemetry stale -- no measurement")
            failures.append("fk: no telemetry")
            continue
        ours = kin.fk(chain, side)["wrist_pos"]
        body = f"{side}_wrist_yaw_link"
        got = truth.get(body)
        if not got:
            print(f"   {body:<26} sidecar did not return it")
            failures.append(f"fk: {body} missing")
            continue
        theirs = got["pelvis_m"]
        err = math.dist(ours, theirs)
        ok = err <= FK_TOL_M
        if not ok:
            failures.append(f"fk {side} off by {err*100:.1f} cm")
        o = f"{ours[0]:+.3f},{ours[1]:+.3f},{ours[2]:+.3f}"
        t = f"{theirs[0]:+.3f},{theirs[1]:+.3f},{theirs[2]:+.3f}"
        print(f"   {body:<26}{o:>26}{t:>26}{err*100:>7.1f}cm  "
              f"{'OK' if ok else 'MISMATCH'}")

    # --- B. reach_to, through the policy ------------------------------------
    print("\nB. reach_to: solve, command, then measure where the hand went")
    print(f"   {'hand':<7}{'target (pelvis)':>22}{'solved':>8}{'measured':>22}{'err':>8}")
    print("   " + "-" * 74)
    for side, target in TARGETS:
        chain = measured_chain(telemetry, side) or [0.0] * 10
        waist = tuple(chain[:kin.WAIST_LINKS])
        sol = kin.ik(target, side, waist=waist, seed=chain[kin.WAIST_LINKS:])
        if not sol["ok"]:
            print(f"   {side:<7}{str(target):>22}  UNREACHABLE "
                  f"(closest {sol['error_m']*100:.0f} cm)")
            failures.append(f"{side} {target}: ik failed")
            continue

        values, unknown = ub.from_named_joints(sol["joints"])
        assert not unknown, unknown
        link.set_upper_body(values)
        time.sleep(3.0)

        after = measured_chain(telemetry, side)
        if after is None:
            print(f"   {side:<7}{str(target):>22}  telemetry stale")
            failures.append(f"{side} {target}: no measurement")
            continue
        actual = kin.fk(after, side)["palm_pos"]
        err = math.dist(actual, target)
        ok = err <= REACH_TOL_M
        if not ok:
            failures.append(f"{side} {target}: off by {err*100:.0f} cm")
        tgt = f"{target[0]:+.2f},{target[1]:+.2f},{target[2]:+.2f}"
        act = f"{actual[0]:+.2f},{actual[1]:+.2f},{actual[2]:+.2f}"
        print(f"   {side:<7}{tgt:>22}{sol['error_m']*100:>7.1f}cm{act:>22}"
              f"{err*100:>7.1f}cm  {'OK' if ok else 'MISSED'}")

    link.set_upper_body(None)
    link.stop()
    link.close()
    sim.close()

    print("\n" + "-" * 90)
    if failures:
        print(f"FAILED ({len(failures)}):")
        for f in failures:
            print(f"    {f}")
        return 1
    print(f"Kinematics agree with MuJoCo within {FK_TOL_M*100:.1f} cm, and every "
          f"reach landed within {REACH_TOL_M*100:.0f} cm.")
    print("The residual in B is the policy tracking a reference while it also "
          "balances — not a kinematics error; A is what proves that.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
