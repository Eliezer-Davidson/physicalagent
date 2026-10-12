#!/usr/bin/env python3
"""AiHand protocol probe — the one experiment that turns a guess into a fact.

The hand's wire framing is UNVERIFIED (see aihand_codec). Two candidates are
implemented; this sends a single small, clamped move in each, one at a time,
with a pause between them, and asks the operator which one moved a finger.
Whatever moves is the answer. Nothing else can settle it: the hand has no
feedback, so an accepted frame and an ignored frame look identical from here.

    # hand plugged into the Pi, bridge running there
    python3 tools/aihand_probe.py --transport tcp --host 192.168.149.1

    # hand plugged into this laptop
    python3 tools/aihand_probe.py --transport serial --device /dev/ttyACM0

Every move is clamped at the LOCKED tier by default: a small excursion either
side of centre. Watch the hand. Nothing here is safe to run unattended.
"""
import argparse
import sys
import time

sys.path.insert(0, __file__.rsplit("/tools/", 1)[0])

from physicalagent.robots import aihand_codec as codec        # noqa: E402
from physicalagent.robots.aihand import AiHand                # noqa: E402
from physicalagent.robots.aihand_link import (                # noqa: E402
    SerialAiHandLink, TcpAiHandLink,
)
from physicalagent.robots.tonypi_servo import ServoTier       # noqa: E402


def build_link(args):
    if args.transport == "serial":
        return SerialAiHandLink(device=args.device, baud=args.baud)
    return TcpAiHandLink(host=args.host, port=args.port)


def probe_framing(link, framing: str, finger: str, tier: ServoTier) -> bool:
    hand = AiHand(link, tier=tier, framing=framing)
    lo, hi = codec.tier_band(tier)
    print(f"\n=== framing '{framing}' — watch the {finger} ===")
    try:
        for label, pulse in (("closing", hi), ("opening", lo)):
            frame = codec.encode({codec.finger_id(finger): pulse},
                                 codec.TIER_MIN_MOVE_MS[tier], framing=framing)
            print(f"  {label} to {pulse}: {frame!r}")
            hand.set_fingers({finger: pulse})
            time.sleep(1.5)
    except OSError as exc:
        print(f"  LINK FAILED: {exc}")
        print("  (tcp: is the bridge running on the Pi? "
              "serial: is the device there, and did brltty grab it?)")
        return False
    answer = input(f"  did the {finger} move? [y/N] ").strip().lower()
    return answer.startswith("y")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--transport", choices=("tcp", "serial"), default="tcp")
    p.add_argument("--host", default="192.168.149.1")
    p.add_argument("--port", type=int, default=9031)
    p.add_argument("--device", default="/dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--finger", default="index",
                   help=f"which finger to test ({', '.join(sorted(codec.FINGER_IDS))})")
    p.add_argument("--tier", default="locked",
                   help="locked (default) | normal | full")
    args = p.parse_args()

    tier = ServoTier.resolve(args.tier)
    codec.finger_id(args.finger)          # fail now on a bad name, not mid-probe

    print(__doc__.split("\n\n")[0])
    print(f"transport={args.transport} tier={tier.name.lower()} "
          f"finger={args.finger}")
    print("The hand has no sensors. YOU are the instrument here — watch it.")
    input("press enter when you are looking at the hand... ")

    winners = []
    link = build_link(args)
    try:
        for framing in sorted(codec.FRAMINGS):
            if probe_framing(link, framing, args.finger, tier):
                winners.append(framing)
    finally:
        link.close()

    print("\n=== result ===")
    if len(winners) == 1:
        print(f"VERIFIED FRAMING: {winners[0]}")
        print("Record it: set AIHAND_FRAMING and flip "
              "aihand_codec.VERIFIED_FRAMING, with a date.")
    elif not winners:
        print("Neither framing moved a finger. The hand is unpowered, the link "
              "is not reaching it, or the firmware speaks a third protocol.")
        print("Next: confirm the hand has its own 7.4V supply, and that the "
              "bridge printed the bytes it forwarded.")
    else:
        print(f"BOTH framings appeared to move it: {winners}. Re-run — one is "
              "probably the previous command still finishing.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
