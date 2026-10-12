#!/usr/bin/env python
"""Send SONIC the start + planner-mode command, and hold it there briefly.

The policy publishes nothing that keeps the robot upright until it receives
command{start:1, planner:1}. This is the equivalent of pressing ']' then Enter
on its keyboard interface, and it is a prerequisite for the robot surviving the
moment the elastic band is released.

Also parks the planner in IDLE for a couple of seconds so the robot has a
defined intent to hold rather than whatever the last stale message said.
"""
from __future__ import annotations

import sys
import time

import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from physicalagent.robots.sonic_zmq import (  # noqa: E402
    MODE_IDLE,
    ZmqSonicLink,
    encode_planner,
)


def main() -> int:
    link = ZmqSonicLink(port=5556, settle_s=1.0, auto_start=False)
    print("[start] publisher bound on tcp://*:5556")

    link.start_control()
    print("[start] sent command{start:1, stop:0, planner:1}")

    # Hold IDLE for a moment: a planner message older than ~1 s is treated as
    # stale, so a single send is not enough to establish intent.
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        link._pub.send(encode_planner(MODE_IDLE, (1.0, 0.0, 0.0),
                                      (1.0, 0.0, 0.0), 0.0, -1.0))
        time.sleep(0.05)
    print("[start] planner parked in IDLE — safe to release the band (key 9)")

    link.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
