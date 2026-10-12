#!/usr/bin/env python
"""Live check of everything the Isaac sidecar wiring claims to provide.

    SONIC_FRONTEND=scripts/isaac_m1_verify.py ./scripts/sonic_isaac_live.sh

Runs as the launcher's frontend (under .venv_sim, SONIC's own world) so the
whole bring-up -- sim, policy, handshake -- is the real one, not a mock. Each
check prints PASS/FAIL with the measured value; exit code is the AND of all.

WHAT IT CHECKS
    ping          backend 'isaac', the five implemented ops, live cameras
    body_pose     pelvis at standing height, feet in ground_contact
    contacts      stance feet carrying real force (hundreds of N, not zeros)
    camera        a decodable, non-trivial JPEG from the head camera
    reset         rt/reset_pose/cmd category '2' teleports the robot back and
                  the policy re-stabilises at standing height
"""
from __future__ import annotations

import base64
import os
import pathlib
import sys
import time

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from physicalagent.robots.sonic_sim_client import SimClient          # noqa: E402
from physicalagent.robots.sonic_telemetry import UnitreeTelemetry    # noqa: E402

ART = REPO / "artifacts" / "isaac_m1"
STAND_M = (0.70, 0.85)


def main() -> int:
    ART.mkdir(parents=True, exist_ok=True)
    results: list[tuple[str, bool, str]] = []

    def check(name: str, ok: bool, detail: str) -> None:
        results.append((name, ok, detail))
        print(f"[m1] {'PASS' if ok else 'FAIL'}  {name:10} {detail}", flush=True)

    tel = UnitreeTelemetry(init_channel=True, interface="lo")
    tel.wait_for_data(timeout_s=10.0)
    sim = SimClient(port=5570)

    # -- ping ---------------------------------------------------------------
    r = sim.ping() or {}
    check("ping", r.get("backend") == "isaac"
          and set(r.get("ops", [])) == {"ping", "body_pose", "episode",
                                        "camera", "contacts"},
          f"backend={r.get('backend')!r} ops={r.get('ops')} "
          f"cameras={r.get('cameras')}")

    # -- body_pose ----------------------------------------------------------
    r = sim.body_pose(["pelvis", "left_ankle_roll_link",
                       "right_ankle_roll_link"]) or {}
    pw = r.get("pelvis_world_m") or [0, 0, -1]
    ground = r.get("ground_contact")
    feet_down = (ground is not None
                 and any("ankle" in g for g in ground))
    check("body_pose", bool(r.get("ok")) and STAND_M[0] <= pw[2] <= STAND_M[1]
          and feet_down,
          f"pelvis_z={pw[2]:.3f} ground_contact={ground}")

    # -- contacts -----------------------------------------------------------
    r = sim.request("contacts") or {}
    rows = r.get("contacts") or []
    foot_n = sum(c["force_n"] for c in rows if "ankle" in c["robot_part"])
    check("contacts", bool(rows) and foot_n > 100.0,
          f"{len(rows)} loaded bodies, feet carry {foot_n:.0f} N")

    # -- camera -------------------------------------------------------------
    r = sim.request("camera", camera="head") or {}
    jpeg = base64.b64decode(r.get("jpeg_b64", "") or "")
    ok_jpg = r.get("kind") == "jpeg" and len(jpeg) > 5000 \
        and jpeg[:2] == b"\xff\xd8"
    if ok_jpg:
        (ART / "cam_head_via_sidecar.jpg").write_bytes(jpeg)
    check("camera", ok_jpg,
          f"{r.get('width')}x{r.get('height')} {len(jpeg)}B "
          f"-> {ART / 'cam_head_via_sidecar.jpg' if ok_jpg else r.get('error')}")

    # -- reset --------------------------------------------------------------
    # A standing robot stays standing whether or not the reset fires, so
    # height alone cannot falsify this check (the first version passed
    # vacuously). The sim prints "reset all" when it ACTS on category '2';
    # success = that line count increments AND the robot is standing after.
    from unitree_sdk2py.core.channel import ChannelPublisher
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
    sim_log = pathlib.Path(os.environ.get("LOGS",
                                          REPO / "artifacts" / "isaac_live")) / "sim.log"

    def _reset_acks() -> int:
        try:
            return sim_log.read_text(errors="replace").count("reset all")
        except OSError:
            return -1

    acks_before = _reset_acks()
    before = (tel.pose() or {}).get("height_m", -1.0)
    pub = ChannelPublisher("rt/reset_pose/cmd", String_)
    pub.Init()
    pub.Write(String_(data="2"))
    time.sleep(4.0)
    after = tel.pose() or {}
    h = after.get("height_m", -1.0)
    acks_after = _reset_acks()
    acted = acks_before >= 0 and acks_after > acks_before
    check("reset", acted and bool(after.get("fresh"))
          and STAND_M[0] <= h <= STAND_M[1],
          f"sim 'reset all' acks {acks_before}->{acks_after}, pelvis_z "
          f"before={before:.3f} after={h:.3f} (published rt/reset_pose/cmd '2')")

    failed = [n for n, ok, _ in results if not ok]
    print(f"[m1] {'ALL PASS' if not failed else 'FAILED: ' + ', '.join(failed)}",
          flush=True)
    return 0 if not failed else 1


if __name__ == "__main__":
    raise SystemExit(main())
