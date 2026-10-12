"""Live proof that a named joint command moves THAT joint on the real robot.

WHY THIS EXISTS
    The 17 upper-body values are permuted between our order and the policy's
    (sonic_upper_body.WIRE_JOINT_NAMES). Every offline test of that permutation
    can only show it is self-consistent -- the shipped bug passed a full suite.
    The only thing that settles it is commanding one joint by name and watching
    which motor moves.

WHAT IT DOES
    Runs the same bring-up as the console, then for each probe joint: command it
    alone, wait for it to settle, read all 29 motors back over rt/lowstate, and
    report which joint moved furthest. If the name in matches the name out, the
    map is right.

    Run it through the launcher so the sim and policy come up the usual way:

        SONIC_FRONTEND=scripts/verify_joint_map.py ./scripts/sonic_live.sh

NOT A PASS/FAIL ORACLE FOR THE POLICY
    The policy is free to deviate from the reference to stay upright, so a joint
    reaching only part way is normal and is reported as a gap, not a failure.
    What is being tested is IDENTITY -- which joint moved -- not tracking error.
"""
from __future__ import annotations

import math
import sys
import time

import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from physicalagent.robots import sonic_upper_body as ub  # noqa: E402
from physicalagent.robots.sonic_zmq import MODE_IDLE, ZmqSonicLink, encode_planner  # noqa: E402
from scripts.sonic_console import _open_telemetry, release_band  # noqa: E402

SETTLE_S = 3.0
RELEASE_S = 3.0

# TWO SEPARATE QUESTIONS, AND ONLY THE FIRST IS ABOUT THE MAP.
#
#   IDENTITY  -- did the joint I named move? That is what this script exists to
#                answer, and it is pass/fail.
#   TRACKING  -- how close did it get? That is a property of the POLICY, which
#                weighs balance against the reference and deliberately does not
#                land on it. It is reported, never failed on.
#
# An earlier version failed a run on tracking. It reported "the name-to-motor map
# is still wrong" over four joints that had each moved the correct motor 56-99%
# of the way, purely because the gap exceeded a threshold picked by guess. A
# measurement that answers a question nobody asked, in the voice of the question
# they did ask, is worse than no measurement.
MIN_TRAVEL_FRAC = 0.5
# The original symptom was one arm tracking far worse than its mirror. Same
# command, mirrored: the two sides should travel the same fraction.
MAX_ASYMMETRY = 0.20

# (joint, target). Each target is chosen FAR from that joint's own neutral --
# the first version of this probe used one value for every joint, and since
# NEUTRAL already holds both elbows at 1.0 rad, commanding 1.2 produced a 0.2 rad
# displacement that lost to incidental motion and read as a wrong-joint failure
# on a joint that had tracked correctly.
#
# Signs are side-aware: shoulder roll is +outward on the left and -outward on the
# right, so a single sign drives one arm out and the other INTO the torso, where
# it stops on self-collision and looks like a mapping failure.
PROBES = (
    ("left_shoulder_pitch", -1.2), ("right_shoulder_pitch", -1.2),
    ("left_shoulder_roll", 1.2), ("right_shoulder_roll", -1.2),
    ("left_shoulder_yaw", 1.0), ("right_shoulder_yaw", -1.0),
    ("left_elbow", 1.9), ("right_elbow", 1.9),
)


def measured_upper(telemetry) -> list[float] | None:
    """The 17 upper-body motors in radians, in OUR order, from rt/lowstate.

    joints() reports degrees keyed by name; going back through ub.JOINT_NAMES
    rather than by position means this cannot quietly re-introduce an ordering
    assumption of its own.
    """
    snap = telemetry.joints()
    if not snap.get("available") or not snap.get("fresh"):
        return None
    rows = snap["joints"]
    return [math.radians(rows[name]["angle_deg"]) for name in ub.JOINT_NAMES]


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
        print("no telemetry -- cannot measure, so cannot claim anything")
        return 2
    time.sleep(1.5)

    print(f"\n{'commanded':<22}{'want':>6}{'got':>6}{'travel':>8}   "
          f"{'joint furthest from rest':<24} identity")
    print("-" * 82)
    failures = []
    travel: dict[str, float] = {}
    for name, target in PROBES:
        link.set_upper_body(None)          # hand back, let it return to neutral
        time.sleep(RELEASE_S)

        values, unknown = ub.from_named_joints({name: target})
        assert not unknown, unknown
        link.set_upper_body(values)
        time.sleep(SETTLE_S)
        after = measured_upper(telemetry)
        if after is None:
            print(f"{name:<22} (telemetry went stale -- no measurement)")
            failures.append(name)
            continue

        # Displacement from NEUTRAL, which is the value every OTHER joint was
        # just commanded to hold -- not from a base reading that may still have
        # been in transit from the previous probe. The named joint's target is
        # ~1 rad off its own neutral, so if the map is right it is unambiguously
        # the furthest-displaced joint.
        disp = [abs(after[i] - ub.NEUTRAL[i]) for i in range(17)]
        moved = ub.JOINT_NAMES[max(range(17), key=lambda i: disp[i])]
        got = after[ub.INDEX[name]]

        # Fraction of the commanded displacement actually travelled. Measured
        # from the joint's OWN neutral, because a joint that rests at 1.0 rad
        # commanded to 1.9 has travelled 0.9, not 1.9.
        rest = ub.NEUTRAL[ub.INDEX[name]]
        frac = (got - rest) / (target - rest) if target != rest else 0.0
        travel[name] = frac
        ok = moved == name and frac >= MIN_TRAVEL_FRAC
        if not ok:
            failures.append(name)
        note = "OK" if ok else ("WRONG JOINT" if moved != name else "barely moved")
        print(f"{name:<22}{math.degrees(target):>6.0f}{math.degrees(got):>6.0f}"
              f"{frac:>7.0%}   {moved:<24} {note}")

    link.set_upper_body(None)
    link.stop()
    link.close()

    print("-" * 82)

    # The original symptom, checked directly: same command mirrored, do the two
    # sides behave the same? This is what "the right arm tracks 59 degrees worse
    # than the left" would have caught.
    print("\nleft/right symmetry — same command mirrored, travel should match")
    worst_pair = 0.0
    for joint in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow"):
        lt, rt = travel.get(f"left_{joint}"), travel.get(f"right_{joint}")
        if lt is None or rt is None:
            continue
        gap = abs(lt - rt)
        worst_pair = max(worst_pair, gap)
        print(f"  {joint:<18} left {lt:>5.0%}   right {rt:>5.0%}   "
              f"differ {gap:>5.0%}  {'ok' if gap <= MAX_ASYMMETRY else 'ASYMMETRIC'}")

    if failures:
        print(f"\nIDENTITY FAILED for {len(failures)}/{len(PROBES)}: "
              f"{', '.join(failures)}")
        print("A named joint did not move the motor of that name. Do not trust "
              "arm commands.")
        return 1
    print(f"\nIDENTITY OK — all {len(PROBES)} probes moved the motor that was named.")
    if worst_pair > MAX_ASYMMETRY:
        print(f"but the two sides differ by up to {worst_pair:.0%} — mirrored "
              "commands are not producing mirrored motion.")
        return 1
    print(f"SYMMETRY OK — mirrored pairs agree within {worst_pair:.0%}.")
    print("Under-travel is the policy weighing balance against the reference, "
          "not a mapping error; judge arm results by measured angle, never by "
          "what was asked for.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
