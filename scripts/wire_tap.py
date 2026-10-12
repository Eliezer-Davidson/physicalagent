"""Log what mode is ACTUALLY on the wire, independently of what anyone intended.

    .venv_sim/bin/python scripts/wire_tap.py

WHY
    While testing modes by prompting a language model, there are three different
    things that can disagree: what the human asked for, what the model chose, and
    what reached the policy. Body measurements alone cannot tell those apart --
    a mode that behaves oddly looks identical to the model having picked a
    different mode. This taps the wire so the commanded side is a fact.

    It cost nothing to be wrong about this until a `stealth_walk` measurement
    came back with a flight phase and high knees, which is not what a sneak looks
    like, and there was no way to check whether mode 18 had been sent at all.

HOW
    The console BINDs PUB on 5556, so this CONNECTs a SUB -- extra subscribers
    are free and the publisher never knows. Only transitions are printed: the
    planner topic republishes at 50 Hz and every frame is otherwise identical.
"""
from __future__ import annotations

import json
import pathlib
import struct
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import zmq  # noqa: E402

from physicalagent.robots import sonic_modes  # noqa: E402
from physicalagent.robots.sonic_zmq import HEADER_SIZE  # noqa: E402
from physicalagent.paths import PA_ARTIFACTS  # noqa: E402

LOG = PA_ARTIFACTS / "live" / "wire_tap.jsonl"
TOPIC = b"planner"


def decode(msg: bytes) -> dict | None:
    if not msg.startswith(TOPIC):
        return None
    body = msg[len(TOPIC):]
    header = json.loads(body[:HEADER_SIZE].rstrip(b"\x00") or b"{}")
    p = body[HEADER_SIZE:]
    if len(p) < 32:
        return None
    mode, = struct.unpack_from("<i", p, 0)
    speed, height = struct.unpack_from("<ff", p, 28)
    names = [f["name"] for f in header.get("fields", [])]
    m = sonic_modes.BY_VALUE.get(mode)
    return {"mode": mode, "name": m.name if m else f"?{mode}",
            "kind": m.kind if m else "?",
            "speed": round(speed, 4), "height": round(height, 4),
            "movement": [round(v, 3) for v in struct.unpack_from("<fff", p, 4)],
            "upper_body": "upper_body_position" in names}


def lock_or_die():
    """One tap per log. A second one double-writes every transition, which reads
    as the console having sent the same mode twice."""
    import fcntl
    LOG.parent.mkdir(parents=True, exist_ok=True)
    fh = open(str(LOG) + ".lock", "w")
    try:
        fcntl.flock(fh, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        print(f"another tap already owns {LOG} -- refusing to start a second",
              file=sys.stderr)
        raise SystemExit(3)
    return fh


def main() -> int:
    _lock = lock_or_die()  # noqa: F841 -- must stay alive
    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    sub.connect("tcp://127.0.0.1:5556")
    sub.setsockopt(zmq.SUBSCRIBE, TOPIC)
    LOG.parent.mkdir(parents=True, exist_ok=True)

    print(f"tapping tcp://127.0.0.1:5556 [{TOPIC.decode()}] -> {LOG}", flush=True)
    prev = None
    with LOG.open("a") as fh:
        while True:
            row = decode(sub.recv())
            if row is None:
                continue
            key = (row["mode"], row["speed"], row["height"], row["upper_body"])
            if key == prev:
                continue
            prev = key
            row["wall"] = time.time()
            fh.write(json.dumps(row) + "\n")
            fh.flush()
            arms = "  ARMS HELD BY US" if row["upper_body"] else ""
            print(f"{time.strftime('%H:%M:%S')}  mode {row['mode']:>2} "
                  f"{row['name']:<16} {row['kind']:<8} "
                  f"speed {row['speed']:>6}  height {row['height']:>6}{arms}",
                  flush=True)


if __name__ == "__main__":
    sys.exit(main())
