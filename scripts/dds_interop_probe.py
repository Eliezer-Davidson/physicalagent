#!/usr/bin/env python
"""Does a DDS participant from interpreter A see one from interpreter B?

WHY THIS EXISTS
    On 2026-08-06 the Isaac sim published rt/lowstate at 78-88 Hz and rt/odostate
    at 12 Hz, our own subscriber saw every message, and SONIC's deploy binary sat
    printing "LowState is not available" ~2000 times and never emitted a single
    rt/lowcmd. Domain, interface, message type, quaternion order, CRC and publish
    rate were all checked and all correct.

    The variable nobody had checked was the DDS library itself. This repo's two
    live interpreters shipped DIFFERENT MAJOR VERSIONS of CycloneDDS:

        GR00T .venv_sim   python cyclonedds 0.10.2  ->  libddsc 0.11.0
        unitree_sim_env   python cyclonedds 11.0.1  ->  libddsc 11.0.1

    Measured then (4-cell matrix, domain 0 on "lo"): same-version cells ~384
    msgs, cross-version cells 0. The two versions cannot discover each other at
    all. They are sealed worlds. That was the entire M0 blocker: Isaac published
    into a world SONIC is not in.

    Note the shape of the trap -- BOTH same-version cells pass, so any test that
    stays inside one interpreter reports a healthy bus. Our probe subscriber ran
    under the Isaac interpreter, so it saw everything the sim published and
    "confirmed" telemetry was flowing. It was, into an empty room.

FIXED 2026-08-07
    The Isaac-side environment now runs the cyclonedds 0.10.2 bindings built
    from sdist against a source-built C core (0.10.5) at $ISAAC_CYCLONEDDS_HOME.
    Wire interop 0.10.5 <-> 0.11.0 was MEASURED first (293/291 msgs both
    directions, against a verified-quiet bus), so all four cells below now
    expect traffic: one world.

    The env vars in ISAAC_DDS_ENV are LOAD-BEARING. cyclonedds' loader falls
    back to ctypes.util.find_library("ddsc"), which consults only the ldconfig
    cache (LD_LIBRARY_PATH is ignored) and returns /usr/local's 11.0.1 -- while
    the compiled _clayer resolves 0.10.5. Two cores in one process is exactly
    the 2026-07-31 "source-built cyclonedds cannot create topics" curse:
    participant creation survives it, topic creation dies PRECONDITION_NOT_MET.
    CYCLONEDDS_HOME forces the ctypes layer onto the same core as _clayer.
    Every DDS-touching process under unitree_sim_env must run with these vars;
    endpoint() applies them ITSELF when it detects the Isaac interpreter, so
    the manual pub/sub forms below are safe to run verbatim.

WHY THE TOPIC IS probe/dds_interop AND NOT rt/lowstate
    The first version of this probe published default-constructed (all-zero)
    LowState_ on the REAL rt/lowstate -- fabricated telemetry on a live topic,
    the exact defect class this project once spent a day removing, and anything
    listening (SONIC included, post-fix) would ingest it as genuine. Discovery
    and wire compatibility are topic-name-agnostic, so the probe now uses the
    same IDL type on its own topic. This also closes a false-PASS hole: on a
    probe-scoped topic, ambient sim traffic cannot masquerade as the paired
    publisher's, so a cell's count answers for exactly one pub->sub pair.

CONFIGURATION
    Both interpreters are read from the environment; there are no hard-coded
    machine paths. Set these before `matrix`:

        ISAAC_PY               the Isaac-side interpreter
        ISAAC_CYCLONEDDS_HOME  the source-built CycloneDDS install prefix
        WBC_ROOT               GR00T-WholeBodyControl (for its .venv_sim python)

    Without ISAAC_PY / ISAAC_CYCLONEDDS_HOME this machine has no Isaac stack
    and `matrix` refuses rather than guessing a path.

USAGE
    Matrix (drives both interpreters itself):
        python scripts/dds_interop_probe.py matrix

    One side at a time, to test any other pair:
        <interpreter-a> scripts/dds_interop_probe.py pub 14
        <interpreter-b> scripts/dds_interop_probe.py sub 8

    `pub`/`sub` need unitree_sdk2py, so they do NOT run under the project venv.
    `matrix` is pure stdlib and runs anywhere.

EXIT CODES
    matrix:    0 all cells as documented | 1 a cell disagreed or a probe failed
    pub/sub:   0 ok / saw traffic | 1 saw nothing | 2 PROBE FAILED (an exception
               -- e.g. topic creation died; NOT the same thing as a silent bus)
"""
from __future__ import annotations

import os
import re
import subprocess
import sys
import time

# The two interpreters this stack actually runs on. Both come from the
# environment: a machine without the Isaac stack gets None and must skip, and
# guessing is how a literal home directory gets reintroduced.
ISAAC_PY = os.environ.get("ISAAC_PY") or None
MUJOCO_PY = os.path.join(os.environ.get("WBC_ROOT", os.path.expanduser("~/GR00T-WholeBodyControl")),
                         ".venv_sim", "bin", "python")

# Required for ANY DDS use under the Isaac interpreter -- see FIXED above.
_CDDS = os.environ.get("ISAAC_CYCLONEDDS_HOME") or None
ISAAC_DDS_ENV = ({"CYCLONEDDS_HOME": _CDDS, "LD_LIBRARY_PATH": f"{_CDDS}/lib"}
                 if _CDDS else {})

# Same IDL type the stack really uses, on a topic nothing else publishes.
TOPIC = "probe/dds_interop"

PUB_S, SUB_S, SETTLE_S = 14.0, 8.0, 2.0


def _env_for(py: str) -> dict:
    env = dict(os.environ)
    if ISAAC_PY and py == ISAAC_PY:
        env.update(ISAAC_DDS_ENV)
    return env


def _run_pub_sub(pub_py: str, sub_py: str) -> tuple[int, int]:
    """Run a publisher and a subscriber, return (published, received) counts.

    -1 for a side that never printed its summary line -- a broken probe, which
    must never be read as "silent bus".
    """
    here = __file__
    pub = subprocess.Popen([pub_py, here, "pub", str(PUB_S)],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           text=True, env=_env_for(pub_py))
    try:
        time.sleep(SETTLE_S)
        sub_out = subprocess.run([sub_py, here, "sub", str(SUB_S)],
                                 capture_output=True, text=True,
                                 timeout=SUB_S + 60, env=_env_for(sub_py)).stdout
    finally:
        pub_out = pub.communicate(timeout=PUB_S + 30)[0]
    mp = re.search(r"wrote (\d+) messages", pub_out or "")
    ms = re.search(r"received (\d+) messages", sub_out or "")
    return (int(mp.group(1)) if mp else -1, int(ms.group(1)) if ms else -1)


def matrix() -> int:
    if not ISAAC_PY or not _CDDS:
        print("ISAAC_PY and ISAAC_CYCLONEDDS_HOME are unset -- this machine has "
              "no Isaac-side stack, so the cross-interpreter matrix cannot run.",
              file=sys.stderr)
        return 2
    cells = [("MuJoCo->MuJoCo", MUJOCO_PY, MUJOCO_PY, True),
             ("Isaac ->Isaac ", ISAAC_PY, ISAAC_PY, True),
             ("Isaac ->MuJoCo", ISAAC_PY, MUJOCO_PY, True),
             ("MuJoCo->Isaac ", MUJOCO_PY, ISAAC_PY, True)]
    ok = True
    print(f"{'pub -> sub':<16} {'sent':>6} {'rcvd':>6}  expected  result")
    for name, pub_py, sub_py, expect_traffic in cells:
        sent, rcvd = _run_pub_sub(pub_py, sub_py)
        if sent < 0 or rcvd < 0:
            # A missing summary line is a broken probe, not a finding.
            print(f"{name:<16} {sent:>6} {rcvd:>6}  {'':8}  PROBE FAILED")
            ok = False
            continue
        agrees = (rcvd > 0) == expect_traffic and (not expect_traffic or sent > 0)
        ok = ok and agrees
        print(f"{name:<16} {sent:>6} {rcvd:>6}  "
              f"{'traffic' if expect_traffic else 'silent':8}  "
              f"{'as documented' if agrees else '*** DISAGREES ***'}")
    print("\nOne DDS world: Isaac and MuJoCo interpreters see each other."
          if ok else "\nMatrix disagrees with the documented result -- if a cell "
          "went silent, check ISAAC_DDS_ENV was applied (mixed-core loading is "
          "silent except for topic creation).")
    return 0 if ok else 1


def endpoint(role: str, secs: float) -> int:
    # Manual mode must be safe to run verbatim: apply the load-bearing env vars
    # BEFORE the first cyclonedds import when running under the Isaac
    # interpreter (imports are deferred to here for exactly this reason).
    if ISAAC_PY and sys.executable == ISAAC_PY:
        os.environ.update(ISAAC_DDS_ENV)
    try:
        from unitree_sdk2py.core.channel import (ChannelFactoryInitialize,
                                                 ChannelPublisher,
                                                 ChannelSubscriber)
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

        ChannelFactoryInitialize(0, "lo")
        import cyclonedds
        print(f"[{role}] cyclonedds: {cyclonedds.__file__}", flush=True)

        if role == "pub":
            pub = ChannelPublisher(TOPIC, LowState_)
            pub.Init()
            msg = unitree_hg_msg_dds__LowState_()
            t0, n = time.monotonic(), 0
            while time.monotonic() - t0 < secs:
                msg.tick = n
                pub.Write(msg)
                n += 1
                time.sleep(0.02)
            print(f"[pub] wrote {n} messages in {secs:.0f}s", flush=True)
            return 0

        got = {"n": 0}
        sub = ChannelSubscriber(TOPIC, LowState_)
        sub.Init(lambda _m: got.__setitem__("n", got["n"] + 1), 10)
        time.sleep(secs)
        print(f"[sub] received {got['n']} messages", flush=True)
        return 0 if got["n"] else 1
    except Exception as exc:
        # A crash is NOT a silent bus. Exit 2 keeps the two distinguishable.
        print(f"[{role}] PROBE FAILED: {type(exc).__name__}: {exc}", flush=True)
        return 2


if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1] not in ("matrix", "pub", "sub"):
        print(__doc__)
        raise SystemExit(2)
    if sys.argv[1] == "matrix":
        raise SystemExit(matrix())
    raise SystemExit(endpoint(sys.argv[1], float(sys.argv[2])))
