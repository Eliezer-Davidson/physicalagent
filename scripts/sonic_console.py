#!/usr/bin/env python
"""Interactive console for the simulated G1: you type, Gemini drives, MuJoCo shows it.

Normally launched by scripts/sonic_live.sh, which brings up the simulator and the
SONIC policy first. Run directly only if those are already up.

WHAT THIS DOES THAT sonic_sim_demo.py DOES NOT
    The demo binds the ZMQ publisher, runs one prompt and exits. This holds the
    publisher open across many prompts, so the policy sees an uninterrupted
    stream and the robot keeps its footing between tasks. It also performs the
    start handshake and releases the elastic band itself, in the one order that
    works:

        bind publisher -> command{start:1, planner:1} -> park IDLE
        -> release band (key 9) -> hand the operator a prompt

    Release the band before the policy is holding the robot and it simply falls;
    that ordering is the whole reason this script exists rather than a README.

STOPPING
    Type `stop` at the prompt. It calls adapter.stop_all() directly and never
    consults the model -- an emergency stop must not wait behind inference
    latency, and the model must not get the chance to reinterpret it. Ctrl-C
    aborts the running task and halts the robot without ending the session, so
    you never lose the ability to type `stop`. Quitting halts the robot too.
"""
from __future__ import annotations

import argparse
import os
import pathlib
import subprocess
import sys
import time

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from physicalagent.apikey import load_gemini_key  # noqa: E402
from physicalagent.brain.er import Brain, GeminiClient  # noqa: E402
from physicalagent.robots.sonic_g1 import SonicG1Adapter  # noqa: E402
from physicalagent.robots.sonic_zmq import (  # noqa: E402
    MODE_IDLE,
    ZmqSonicLink,
    encode_planner,
)
from physicalagent.shell import enable_line_editing, run_shell  # noqa: E402

SYSTEM = """You control a Unitree G1 humanoid robot with Inspire hands, in a
physics simulation.

The robot balances itself — you never command body joints. You command where the
body should go and how it should behave, and a whole-body controller (NVIDIA's
SONIC policy) works out the 29 joint angles that keep it upright while doing what
you asked. The fingers are the exception: they are on a separate channel that
cannot upset balance, so you command those directly.

SENSING — three different sources, and it matters which is which
  get_status()      where you are, which way you face, how tilted, how fast.
                    Measured by the robot's own IMU and odometry.
  get_joints(limb)  angle, speed and torque per joint. limb is one of
                    left_leg, right_leg, waist, left_arm, right_arm.
  look()            an image from the robot's head camera.
  scan(beams)       distance and name of whatever is around you, in every
                    direction. Bearing 0 is straight ahead, positive to the left.
  describe_scene()  the objects near you, by name, with range and bearing.
  get_contacts()    what is physically touching the robot, and how hard.
  get_hands()       where the fingers actually are, versus where they were told
                    to go. A finger stopped short is blocked by something.

MOVING
  goto(x, y, yaw_deg)           GO TO A POINT and stop there. Prefer this
                                whenever you know where you want to end up: it
                                steers on measured position and tells you how
                                close it got. x/y are the world-frame metres
                                get_status() reports, so read your position
                                first. It does NOT avoid obstacles — scan() the
                                path, and break a route into legs around things.
  walk(vx, vy, duration_s)      vx forward m/s (+forward), vy sideways (+left).
                                Open loop: use it to nudge, not to arrive.
  turn(yaw_rate, duration_s)    rad/s, + turns left
  set_mode(name, speed, ...)    one of 27 whole-body behaviours — see the tool
                                description for the full list. This is your most
                                powerful control.
  set_height(height_m)          crouch or stand tall while doing anything else
  set_pose(name)                stand, sit, squat, damp (set_mode does more)
  stop()                        halt and stand still

ARMS AND WAIST — you have direct joint control of these
  reach_to(x, y, z, hand)       PUT A HAND AT A POINT and let the arm angles be
                                worked out for you. Coordinates are relative to
                                the PELVIS — x forward, y left, z up, metres —
                                which is NOT the world frame goto() uses. Says
                                so if the point is out of reach, and does not
                                move the arm when it is.
  where_are_my_hands()          where both hands are now, same pelvis frame.
                                Use it to check a reach actually arrived.
  set_arms(pose)                a named upper-body pose
  set_joints({name: radians})   set individual joints: waist_yaw/roll/pitch and
                                the seven joints of each arm. Only the ones you
                                name change.
  release_arms()                give the arms back to the balance controller

  This works while the robot is standing OR walking, because the arms ride
  alongside the balance controller rather than replacing it. The LEGS are not
  available at joint level and never will be — they are what keeps you upright.
  What you set is a REFERENCE: the controller may deviate from it to stay
  balanced. Check get_joints('left_arm') to see where the arm actually went.

HANDS
  set_grasp(grasp, hand, size, strength)
                                shape a hand into a named grasp. See the tool
                                description for which grasp suits what.

How to work well here:
  * Every walk and turn EXPIRES on its own after duration_s and the robot stops.
    To keep moving, call it again. This is deliberate. Modes that are postures
    rather than travel HOLD until you change them.
  * Speeds are clamped. The result tells you what was ACTUALLY commanded, which
    may be slower than you asked for. Read it and adapt rather than assuming.
  * LOOK BEFORE YOU MOVE somewhere you have not seen. scan() is cheap and tells
    you what you are about to walk into; a camera image tells you what it is.
  * get_status() gives you real position, so you can tell whether a move
    actually achieved what you wanted. Check it after a move rather than
    assuming the commanded distance happened. goto() does this checking for
    you and reports final_error_m — walk() does not, and cannot.
  * set_grasp() is a COMMAND. What it returns is the targets you just sent, not
    a measurement — it is not evidence the fingers moved. get_hands() tells you
    where they actually got to; get_contacts() tells you if you are holding
    anything. Never report a grasp as done on the strength of set_grasp's echo.
  * Some modes are marked [untested]: they are accepted by the policy and have
    never been observed running. Trying one is fine; verify with get_status()
    afterwards instead of assuming it worked.
  * get_status() reports controller.active. If it is FALSE the balance policy
    has stopped and nothing is holding the robot up — no command you send can
    reach it and it cannot be recovered from here. Say so plainly and stop;
    do not keep trying, and do not describe the robot as standing.
  * IF THE ROBOT FALLS the simulator instantly stands it back up at the origin,
    and everything you can measure then looks like a healthy robot. get_status()
    reports this as episode.fell_and_was_reset_since_last_check. When that is
    true, whatever you just did FAILED — say so, and do not read the recovered
    pose as evidence it worked.
  * 'damp' relaxes every joint and the robot sinks to the ground. It is a last
    resort, not a way to stop. Use stop() to stop.
  * If a tool refuses, the refusal explains why. Adjust and continue; don't repeat
    the identical call.

THE WORLD AROUND YOU
  The scene contains furniture and objects. They are REAL physics bodies: they
  can be pushed, toppled and picked up, and they do not move themselves.
  describe_scene() names each object with range, bearing AND world_xy — the
  world-frame point goto() accepts. goto_object(name) walks to a STANDOFF point
  short of the object and faces it; walking INTO furniture shoves it.
  The camera is bolted to the torso. turn() aims it in yaw;
  set_joints({'waist_pitch': ...}) tilts it (about +/-0.5 rad). There is no neck.
  To pick something up: goto_object() until it is within ~0.55 m of a shoulder,
  reach_to() just above it, set_grasp(), then get_contacts() to learn whether
  you are actually holding it, then reach_to() higher to lift. Report what the
  contacts and heights MEASURED, not what the plan intended.
"""

BANNER = """\
G1 console — Gemini Robotics-ER 2 has control of the SIMULATED Unitree G1.
Watch the simulator window; the robot is standing under the policy's balance.

  <task>   plain English, e.g. 'walk forward three metres then turn left'
  stop     halt the robot IMMEDIATELY (does not consult the model)
  forget   clear the robot's memory of this session's earlier tasks
  quit     end the session (always halts first)

This drives a simulation. Nothing here is cleared for hardware.
"""


class _Guard:
    """No e-stop in simulation. Kept as a real object rather than None so the
    adapter's guard checks are exercised on the same code path hardware will."""
    is_estopped = False


# The robot hangs on the elastic band at ~0.96 m and stands free at ~0.79 m.
# Anything above BAND_HEIGHT_M is still suspended.
BAND_HEIGHT_M = 0.88
# ...and anything below FLOOR_HEIGHT_M is not standing either, it is lying down.
# Checking only the upper bound reported "released — pelvis 0.29 m, robot is
# standing on its own" over a robot that had collapsed and was resetting once a
# third of a second. A one-sided check on a quantity with two failure directions
# is a check that passes on half its failures.
FLOOR_HEIGHT_M = 0.55


def _pelvis_height() -> float | None:
    """Read the last logged pelvis height, or None if there is no pose log.

    The simulator's own stdout cannot be used to confirm anything here: it is
    block-buffered to a file and lost when the process is killed, so the
    "ElasticBand enable: False" message it prints never lands. The pose log is
    line-buffered and is the only reliable read-back.
    """
    import glob
    import os
    base = os.environ.get("SONIC_POSE_LOG")
    if not base:
        return None
    paths = glob.glob(f"{base}.*")
    if not paths:
        return None
    # Newest by MTIME, and only if it is actually being written. The log dir
    # accumulates one file per session, and lexicographic sort once picked a
    # PREVIOUS night's log whose final line said 0.79 m — so a robot hanging on
    # the band at 0.96 m was declared "already free" and left flailing in mid
    # air. A stale file is not evidence; returning None routes release_band to
    # the deterministic release-file path instead of a confident lie.
    newest = max(paths, key=os.path.getmtime)
    if time.time() - os.path.getmtime(newest) > 30.0:
        return None
    try:
        with open(newest, "rb") as fh:             # tail without reading it all
            fh.seek(0, 2)
            fh.seek(max(0, fh.tell() - 4096))
            lines = [ln for ln in fh.read().decode().splitlines() if "," in ln]
        return float(lines[-1].split(",")[4])
    except Exception:
        return None


def _sidecar_pose(sim):
    """Pelvis height and ground contacts from the LIVE simulator state.

    The pose-log path below can only ever be evidence about the past — it once
    tailed a previous session's file and declared a robot hanging at 0.96 m
    "already free". This channel cannot be stale.
    """
    try:
        r = sim.body_pose(["pelvis"])
        if not r.get("ok"):
            return None
        return {"z": r["bodies"]["pelvis"]["world_m"][2],
                "ground": r.get("ground_contact") or []}
    except Exception:
        return None


def _zero_drop_release(sim, timeout_s: float = 15.0) -> bool:
    """Hand the robot to the policy with NO drop.

    The band holds the pelvis at ~0.96 m and the robot stands free at ~0.79 m,
    so simply switching the band off drops it ~17 cm onto an already-live
    policy — a coin-flip landing, and the real anatomy of the "flail on launch"
    flake. You cannot instead delay the policy: its TensorRT engine takes ~35 s
    to load, which is the whole reason the band exists.

    So: shorten the band until the feet are actually on the floor, let the
    running policy take the weight, and only then release. Verified on the
    sidecar at every step, because the failure this replaces was a bring-up
    that reported success while the robot hung in mid air.
    """
    before = _sidecar_pose(sim)
    if before is None:
        return False
    if before["z"] < BAND_HEIGHT_M and before["ground"]:
        print(f"[band] already free ({before['z']:.2f} m, feet down)")
        return True

    print(f"[band] pelvis {before['z']:.2f} m — lowering the band to rest the "
          "feet on the floor before releasing (zero-drop hand-off)")
    try:
        sim.band(length_m=-0.2)
    except Exception as exc:
        print(f"[band] could not drive the band: {exc}")
        return False

    deadline = time.monotonic() + timeout_s
    touched = False
    while time.monotonic() < deadline:
        s = _sidecar_pose(sim)
        if s and s["z"] < 0.86 and s["ground"]:
            touched = True
            break
        time.sleep(0.2)
    if not touched:
        s = _sidecar_pose(sim)
        print(f"[band] feet never reached the floor "
              f"(pelvis {s['z']:.2f} m)" if s else "[band] lost the sidecar")
        return False

    try:
        sim.band(release=True)
    except Exception as exc:
        print(f"[band] could not release: {exc}")
        return False
    time.sleep(2.0)
    after = _sidecar_pose(sim)
    if after is None:
        print("[band] released, but the sidecar went quiet — unverified")
        return True
    ok = FLOOR_HEIGHT_M <= after["z"] < BAND_HEIGHT_M
    print(f"[band] {'released — pelvis' if ok else 'RELEASE FAILED — pelvis'} "
          f"{after['z']:.2f} m (was {before['z']:.2f} m, zero-drop)")
    return ok


def release_band(attempts: int = 3, sim=None) -> bool:
    """Get the robot off the elastic band, and confirm it actually happened.

    Two mechanisms, in order of reliability:

      1. Touch SONIC_BAND_RELEASE_FILE. The simulator polls for it and drops the
         band itself (base_sim.py local addition). No window, no focus, no X.
      2. Synthesise the '9' keypress with XTEST, as a human would. This is the
         fallback because XTEST focus on an XWayland surface fails
         intermittently -- three runs in a row worked, then four attempts in a
         row failed, on 2026-07-31.

    Either way the height is read back. A silent failure here is the worst
    outcome available: the robot hangs at ~0.96 m and every subsequent command
    does nothing, which is indistinguishable from a dead command path.
    """
    def verdict(z: float) -> tuple[bool, str]:
        if z >= BAND_HEIGHT_M:
            return False, f"pelvis still {z:.2f} m — still hanging on the band"
        if z < FLOOR_HEIGHT_M:
            return False, (f"pelvis {z:.2f} m — the robot is DOWN, not standing. "
                           "The band is off but the policy is not holding it.")
        return True, f"released — pelvis {z:.2f} m, robot is standing on its own"

    # Preferred path: lower the band first so the hand-off costs no height at
    # all. Only reachable with the sidecar, because it is the one channel that
    # can drive the band AND read the result without going stale.
    if sim is not None and _zero_drop_release(sim):
        return True

    z = _pelvis_height()
    if z is not None:
        ok, why = verdict(z)
        if ok:
            print(f"[band] already free ({z:.2f} m)")
            return True
        if z < FLOOR_HEIGHT_M:
            print(f"[band] {why}")
            return False

    trigger = os.environ.get("SONIC_BAND_RELEASE_FILE")
    for i in range(1, attempts + 1):
        if trigger:
            try:
                pathlib.Path(trigger).touch()
            except Exception as exc:
                print(f"[band] could not touch {trigger}: {exc}")
        else:
            try:
                subprocess.run([sys.executable, str(REPO / "scripts/sonic_key.py"), "9"],
                               timeout=20, check=False)
            except Exception as exc:
                print(f"[band] could not send key 9: {exc}")
        time.sleep(1.5)
        z = _pelvis_height()
        if z is None:
            print("[band] no pose log — cannot verify the release")
            return True                     # unverifiable; nothing more to do
        ok, why = verdict(z)
        print(f"[band] {why}")
        if ok:
            return True
        if z < FLOOR_HEIGHT_M:
            return False                    # retrying a release cannot stand it up

    print("\n*** BAND NOT RELEASED — the robot is still hanging on the spring and "
          "will not go anywhere. Click the MuJoCo window and press 9, then carry on. ***\n")
    return False


def _open_telemetry():
    """Subscribe to what the robot itself publishes. Optional on purpose: the
    console must still come up without it, saying so, rather than refusing to
    start over a sensing channel."""
    try:
        from physicalagent.robots.sonic_telemetry import UnitreeTelemetry

        # The channel factory is already initialised by whoever brought DDS up in
        # this process; calling it again prints a benign create-domain error.
        # Here nothing else has, so we do it.
        tel = UnitreeTelemetry(init_channel=True, interface="lo")
        if not tel.wait_for_data(timeout_s=5.0):
            print("[telemetry] no rt/lowstate within 5 s — "
                  "measured state will be unavailable")
        return tel
    except Exception as exc:
        print(f"[telemetry] unavailable: {exc}")
        return None


def _open_sim(port: int):
    """Connect to the in-simulator sidecar (camera, lidar, contacts). Also
    optional: without it the model loses its eyes but keeps its legs."""
    try:
        from physicalagent.robots.sonic_sim_client import SimClient

        sim = SimClient(port=port)
        info = sim.ping()
        cams = ", ".join(info.get("cameras") or []) or "none"
        print(f"[sim] sidecar ok — cameras: {cams}, "
              f"hand actuators: {info.get('hand_actuators', 0)}")
        return sim
    except Exception as exc:
        print(f"[sim] sidecar unavailable: {exc}")
        return None


def _start_controller_watchdog(telemetry):
    """Shout, in the operator's terminal, the moment the balance controller dies.

    SONIC stops its control loop for good if rt/lowstate goes 500 ms stale, and
    it says so only on its own stdout -- which is redirected to a log file
    nobody is watching. The visible symptom is a robot that flails, falls, gets
    stood back up and falls again, while the console carries on taking commands
    as though everything were fine. That happened for five minutes in a live
    session before anyone could name it.
    """
    import threading

    if telemetry is None:
        return None

    def loop():
        warned = False
        while True:
            time.sleep(1.0)
            try:
                state = telemetry.controller()
            except Exception:
                continue
            if state.get("active"):
                warned = False
                continue
            if not warned and state.get("age_s") is not None:
                warned = True
                print("\n" + "!" * 72)
                print("!! THE BALANCE CONTROLLER HAS STOPPED (no rt/lowcmd for "
                      f"{state['age_s']}s).")
                print("!! SONIC drops control permanently when the simulator "
                      "stalls >500ms, and")
                print("!! does not restart. Nothing is holding the robot up and "
                      "no command can")
                print("!! reach it. Quit and relaunch -- see policy.log for "
                      "'Lost LowState'.")
                print("!" * 72 + "\n", flush=True)

    t = threading.Thread(target=loop, daemon=True)
    t.start()
    return t


def _sources_banner(telemetry, sim) -> str:
    """Say plainly which senses the model actually has this run. A missing
    channel that is never announced becomes a mysterious refusal ten turns in."""
    rows = [
        ("commands  → SONIC over ZMQ", True),
        ("telemetry ← Unitree SDK2 (rt/lowstate, rt/odostate)", telemetry is not None),
        ("camera + lidar + contacts ← simulator sidecar", sim is not None),
    ]
    lines = [("  ✓ " if ok else "  ✗ ") + label for label, ok in rows]
    return "Channels this session:\n" + "\n".join(lines)


def _run_battery(adapter, telemetry, script_path, log_path="",
                 sleep_fn=None) -> int:
    """Run a deterministic verb battery through the console's bring-up.

    The battery machinery lives in g1_sonic_session (load_script/run_script)
    and is IMPORTED, not copied: the same grammar, the same fail-stop rules,
    the same JSONL log that the hardware run wrote -- so the sim capture and
    the hardware capture are comparable by construction. No Brain, no API
    key: a battery is deterministic by contract and home may be offline.
    """
    sys.path.insert(0, str(REPO / "scripts"))
    import g1_sonic_session as sess
    try:
        script = sess.load_script(script_path)
    except sess.SafetyRefusal as e:
        print(f"\n{e}\n", file=sys.stderr)
        return 3
    log_path = log_path or str(
        REPO / "artifacts" / "sessions" /
        (time.strftime("%Y%m%d-%H%M%S") + "-script.jsonl"))
    print(f"\n[script] {script['path']} ({len(script['steps'])} steps) "
          f"-> {log_path}")
    kw = {"sleep_fn": sleep_fn} if sleep_fn is not None else {}
    summary = sess.run_script(adapter, telemetry, script, log_path,
                              real=False, argv=sys.argv, **kw)
    if summary["aborted"]:
        print(f"[script] ABORTED -- {summary['reason']} "
              f"({summary['steps_run']} steps run)")
    else:
        print(f"[script] complete ({summary['steps_run']} steps run)")
    print(f"[script] log: {log_path} -- keep it beside the recorder's "
          f"capture directory; sim_match_compare.py needs both.")
    return 4 if summary["aborted"] else 0


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--model", default="gemini-robotics-er-2-preview")
    p.add_argument("--port", type=int, default=5556)
    p.add_argument("--sidecar-port", type=int, default=5570)
    p.add_argument("--max-hops", type=int, default=40)
    p.add_argument("--no-band-release", action="store_true",
                   help="skip the automatic key-9; use if you already dropped the robot")
    p.add_argument("--script", default="",
                   help="run a deterministic verb battery (batteries/*.txt) "
                        "through the full sim bring-up, then exit; no model, "
                        "no API key")
    p.add_argument("--script-log", default="",
                   help="where the battery's JSONL log goes (default: "
                        "artifacts/sessions/<ts>-script.jsonl)")
    args = p.parse_args()

    link = ZmqSonicLink(port=args.port, settle_s=1.0, auto_start=False)
    print(f"[link] publisher bound on tcp://*:{args.port}")

    link.start_control()
    print("[link] sent command{start:1, stop:0, planner:1}")
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        # a planner message older than ~1 s is stale, so intent is a stream and
        # not a single send -- park IDLE until the operator asks for something
        link._pub.send(encode_planner(MODE_IDLE, (1.0, 0.0, 0.0),
                                      (1.0, 0.0, 0.0), 0.0, -1.0))
        time.sleep(0.05)
    print("[link] planner parked in IDLE")

    # The sidecar is opened BEFORE the release, not after: it is what makes the
    # zero-drop hand-off possible (it can set the band's length and read the
    # live result), and it is the only release check that cannot be stale.
    sim = _open_sim(args.sidecar_port)

    if not args.no_band_release:
        release_band(sim=sim)

    telemetry = _open_telemetry()

    if sim is not None:
        # Drain the fall latch before handing over. Bring-up sometimes drops the
        # robot as the band lets go; the simulator stands it straight back up, but
        # the latch would still be set when the model runs its FIRST command and
        # the fall would be reported as that command's failure. Charging an
        # action with a fall that predates it is its own kind of lie.
        try:
            startup = sim.episode()
            if startup.get("fell_and_was_reset_since_last_check"):
                print("[sim] the robot fell during bring-up and was reset; "
                      "not attributing that to your first command")
        except Exception:
            pass

    adapter = SonicG1Adapter(link, _Guard(), telemetry=telemetry, sim=sim)
    if adapter.start_fall_watchdog():
        print("fall watchdog ARMED: mode-aware detector at 10 Hz, auto-damp on "
              "confirmed fall. Envelopes cover all 27 modes (sim-derived: "
              "measured / vendor / plausible per sonic_fall.ENVELOPES); "
              "'reset-fall' clears the latch after an operator reset.")
    else:
        print("fall watchdog NOT armed — no telemetry link")
    _start_controller_watchdog(telemetry)

    print(_sources_banner(telemetry, sim))
    try:
        if args.script:
            return _run_battery(adapter, telemetry, args.script,
                                args.script_log)
        # The Brain is built only for the interactive shell: a battery run
        # must work with no API key and no network at all.
        brain = Brain(adapter, GeminiClient(load_gemini_key(),
                                            model=args.model), SYSTEM,
                      max_hops=args.max_hops)
        enable_line_editing()
        return run_shell(brain, adapter, banner=BANNER, prompt="\ng1> ")
    finally:
        # Watchdog first: it reads telemetry and damps through the link, and
        # tearing those down underneath it turns a shutdown into a
        # use-after-close on the safety actuator.
        adapter.stop_fall_watchdog()
        link.close()
        if telemetry is not None:
            telemetry.close()
        if sim is not None:
            sim.close()


if __name__ == "__main__":
    raise SystemExit(main())
