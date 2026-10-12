#!/usr/bin/env python3
"""Drive the PHYSICAL G1 through SONIC. This commands a 35 kg biped.

WHY THIS EXISTS
    SonicG1Adapter and ZmqSonicLink were both written and tested and nothing
    composed them against hardware. physicalagent/profiles/g1.py is an
    unregistered draft, and its unlocked_velocity permits 0.5 m/s -- above the
    0.3 in the tier table -- so this driver deliberately does NOT go
    through the profile registry. Limits come from sonic_link, which is pinned
    to the table by tests/test_g1_locomotion_plane.py.

TIER
    SONIC publishes raw rt/lowcmd (g1_deploy_onnx_ref.cpp:271,2258,2678).
    The G1 tier table puts a raw rt/lowcmd writer at TWO-KEY:
    gantry mandatory, operator PLUS a distinct e-stop holder. This script
    cannot verify either, so it refuses to command without --i-am-at-the-estop,
    which is a human asserting a fact about the room.

THE FALL GUARD (measured 2026-08-19, not assumed)
    It takes pelvis height from the odometry stream. sonic_telemetry read
    rt/odostate, which the physical G1 does not publish, so height was always
    None and the guard could never confirm a fall -- blind, while reporting
    "no fall" forever. It now reads rt/odommodestate, measured live at 497 Hz,
    so it can see.

    Seeing is not trusting. Its envelopes assume STANDING pelvis heights and
    this robot hangs at ~0.731 m, so it is attached in REPORT-ONLY mode:
    auto_damp is off unless --auto-damp is passed. Watch its verdicts against a
    real hoisted robot before letting it damp on its own. Keep the robot
    hoisted either way.

USAGE
    WBC=${WBC_ROOT:-$HOME/GR00T-WholeBodyControl}
    P=$WBC/.venv_sim/bin/python

    # 1. see the tool surface, build nothing that can command
    PYTHONPATH=. $P scripts/g1_sonic_session.py --dry

    # 2. bounded manual motion (SONIC must already be running)
    PYTHONPATH=. $P scripts/g1_sonic_session.py --real --i-am-at-the-estop \
        --manual arms --pose wave

    # 3. Gemini drives it
    PYTHONPATH=. $P scripts/g1_sonic_session.py --real --i-am-at-the-estop \
        --task "wave your right hand, then walk forward slowly one metre"
"""
from __future__ import annotations

import argparse
import contextlib
import os
import sys
import time

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)


class SafetyRefusal(Exception):
    """Raised instead of commanding. Never caught inside this file."""


# --- gates --------------------------------------------------------------------

def default_preflight():
    """(ok, detail) from the LOCKED-tier preflight checks. Imported lazily so
    --dry works on a machine with no SDK."""
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "hw_preflight", os.path.join(_ROOT, "scripts", "hw_preflight.py"))
    pf = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pf)
    bad = []
    for check in (pf.check_sim_only_env(expect_hw=True), pf.check_cyclonedds_uri(),
                  pf.check_target(expect_hw=True)):
        if not check.ok:
            bad.append(check.detail)
    return (not bad), "; ".join(bad) if bad else "ok"


def check_ready(estop_ack: bool, preflight=None) -> None:
    """Refuse unless a human has asserted the room is safe AND the environment
    is not carrying a simulator-only setting."""
    if not estop_ack:
        raise SafetyRefusal(
            "REFUSED: SONIC writes raw rt/lowcmd, which the tier table "
            "puts at TWO-KEY -- gantry mandatory, plus a distinct e-stop "
            "holder whose only job is the stop. Pass --i-am-at-the-estop only "
            "when that person is present with their hand on it.")
    ok, detail = (preflight or default_preflight)()
    if not ok:
        raise SafetyRefusal(f"REFUSED: preflight failed -- {detail}")


# --- construction -------------------------------------------------------------

def _zmq_link_class():
    from physicalagent.robots.sonic_zmq import ZmqSonicLink
    return ZmqSonicLink


def make_link(real: bool, port: int = 5556):
    if not real:
        from physicalagent.robots.sonic_link import FakeSonicLink
        return FakeSonicLink()
    return _zmq_link_class()(port=port)


def robot_interface():
    """The single interface on the robot subnet, or "" for the DDS default.
    Never guesses between two: guessing means commanding a real robot while
    believing it is a simulation."""
    import subprocess
    try:
        out = subprocess.run(["ip", "-o", "-4", "addr", "show"],
                             capture_output=True, text=True, timeout=5).stdout
    except Exception:
        return ""
    found = [ln.split()[1] for ln in out.splitlines()
             if len(ln.split()) >= 4 and ln.split()[3].startswith("192.168.123.")]
    return found[0] if len(found) == 1 else ""


def try_telemetry(factory=None):
    """(telemetry, note). Never raises: a robot mid-battery-swap publishes
    nothing, and losing a status readout is acceptable where losing the ability
    to stop the robot is not."""
    if factory is None:
        def factory():
            from physicalagent.robots.sonic_telemetry import UnitreeTelemetry
            return UnitreeTelemetry(init_channel=True,
                                    interface=robot_interface())
    try:
        return factory(), "attached"
    except Exception as e:
        return None, f"telemetry unavailable: {e}"


def make_adapter(link, telemetry=None, auto_damp=False):
    """No guard, no profile. Velocity bounds come from sonic_link.clamp_velocity,
    which the locomotion-plane test pins to the signed table.

    auto_damp defaults OFF even with telemetry attached. The fall detector's
    envelopes assume standing pelvis heights and this robot hangs at a 0.731 m
    suspension height, so a false positive would damp mid-run and read as a real
    fall. The guard is allowed to report before it is allowed to act."""
    from physicalagent.robots.sonic_g1 import SonicG1Adapter
    return SonicG1Adapter(link, telemetry=telemetry, auto_damp=bool(auto_damp))


@contextlib.contextmanager
def stopping(adapter, link):
    """The robot is stopped on every exit path, including the ones nobody
    planned. Ctrl-C escalates to emergency_stop, because that is what an
    operator is reaching for when they press it."""
    try:
        yield adapter
    except KeyboardInterrupt:
        with contextlib.suppress(Exception):
            link.emergency_stop()
        raise
    finally:
        with contextlib.suppress(Exception):
            adapter.stop_all()
        with contextlib.suppress(Exception):
            link.stop()
        with contextlib.suppress(Exception):
            link.close()


# --- banner -------------------------------------------------------------------

def reality_block(real: bool, telemetry_note: str = "not attached",
                  auto_damp: bool = False) -> str:
    from physicalagent.robots import sonic_link as L
    from physicalagent.robots import sonic_telemetry as T
    tel_on = telemetry_note == "attached"
    return "\n".join([
        "=" * 74,
        f"  TARGET: {'PHYSICAL G1 via SONIC (raw rt/lowcmd)' if real else 'FakeSonicLink -- nothing can move'}",
        "=" * 74,
        "  ACTIVE:",
        f"    velocity clamp    |vx|<={L.MAX_VX_M_S}  |vy|<={L.MAX_VY_M_S}  "
        f"|wz|<={L.MAX_WZ_RAD_S}  (signed table)",
        f"    command expiry    every motion expires in <= {L.MAX_DURATION_S}s; "
        f"no latching",
        "    stop on exit      stop_all + link.stop on every exit path",
        "    Ctrl-C            escalates to emergency_stop",
        f"    telemetry         {telemetry_note}"
        + (f" ({T.ODOM_TOPIC}, 497 Hz measured)" if tel_on else ""),
        "",
        "  FALL GUARD:",
        ("    REPORTS ONLY. It can now see pelvis height, but its envelopes\n"
         "                      assume STANDING heights and this robot hangs at\n"
         "                      ~0.731 m. Watch its verdicts before trusting it\n"
         "                      to damp on its own.  KEEP HOISTED."
         if tel_on and not auto_damp else
         "    ARMED -- it will damp the robot on a confirmed fall."
         if auto_damp else
         "    BLIND. No telemetry attached; it cannot confirm a fall\n"
         "                      at all.  KEEP HOISTED."),
        "=" * 74,
    ])


def needs_estop(check, dry, manual, task, real, script=False):
    """Only the paths that can move the ROBOT demand the safety assertion.

    Requiring it for a read would train an operator to pass it reflexively,
    which is exactly how a two-key gate becomes a habit instead of a check --
    and until 2026-08-19 this ignored `real`, so rehearsing a task against
    FakeSonicLink demanded it too. Nothing is published without --real and no
    motor exists, so that was asking someone to certify a human was standing
    at an e-stop in order to run a simulation. The flag has to mean one thing
    every single time it is typed, or it means nothing.

    A --script battery is motion by definition, so it sits with task/manual.
    """
    return real and not (check or dry) and bool(manual or task or script)


def read_state(telemetry):
    """Read-only snapshot. Never constructs a link."""
    if telemetry is None:
        return {"controller": {"active": None,
                               "note": "no telemetry -- cannot tell"},
                "pose": {}}
    return {"controller": telemetry.controller(), "pose": telemetry.pose(),
            "thermal": telemetry.thermal()}


def render_check(state):
    c = state.get("controller") or {}
    p = state.get("pose") or {}
    active = c.get("active")
    out = ["=" * 74]
    if active is True:
        out.append("  SONIC IS IN CONTROL")
        out.append(f"    gains seen: kp {c.get('max_kp')}  kd {c.get('max_kd')}"
                   f"  tau {c.get('max_tau')}")
    elif active is False:
        out.append("  SONIC IS NOT IN CONTROL")
        out.append(f"    {c.get('note', '')}")
    else:
        out.append("  UNKNOWN -- could not read the command stream")
        out.append(f"    {c.get('note', '')}")
    if p.get("available"):
        out.append(f"    pelvis height {p.get('height_m')} m   "
                   f"tilt {p.get('tilt_deg')} deg   "
                   f"heading {p.get('heading_deg')} deg")
    elif p:
        out.append(f"    pose unavailable: {p.get('note', '')}")
    th = state.get("thermal") or {}
    if th.get("available"):
        st = th.get("status")
        if st == "OVER":
            out.append(f"    OVER TEMPERATURE: {', '.join(th['over_warn'])} "
                       f"-- SONIC warns at {th.get('warn_c', 90)} C. STOP.")
        elif st == "WATCH":
            out.append(f"    WATCH: {', '.join(th['approaching'])} in the "
                       f"{th.get('clear_c', 85)}-{th.get('warn_c', 90)} C band, "
                       f"not yet warning")
        hot = ", ".join(f"{r['joint']} {r['max_c']}C" for r in th["hottest"][:4])
        out.append(f"    hottest: {hot}")
    out.append("=" * 74)
    return out


# --- modes --------------------------------------------------------------------

def interactive_help():
    return [
        "  READ-ONLY (these never move the robot):",
        "    status            pose, controller authority, motor heat",
        "    joints [limb]     angles/speeds/torques; limb = left_arm, "
        "right_leg, waist...",
        "    contacts          what is touching the robot, and how hard",
        "    hands             where both hands are right now",
        "",
        "  GEMINI ROBOTICS-ER (it calls the tools; SONIC keeps balancing):",
        "    task <what you want>   e.g. task wave your right hand, then "
        "return to neutral",
        "    ask  <question>        same thing; it answers by reading sensors",
        "    reset                  clear the model's memory, robot untouched",
        "",
        "  MOTION:",
        "    arms <pose>       neutral arms_down arms_up arms_forward arms_out",
        "                      carry reach_left reach_right wave_right",
        "    walk [speed] [s]  default 0.1 m/s for 2 s; clamped to the table",
        "    turn [rate] [s]   default 0.2 rad/s for 2 s; + is left",
        "    pose <name>       whole-body: stand, sit, squat...",
        "    height <m>        hold the body higher or lower",
        "    release           hand the arms back to the balance controller",
        "",
        "  STOPPING:",
        "    stop              stop motion, KEEP control",
        "    estop             emergency stop (software; the real one is the "
        "button)",
        "    quit              stop, release control, exit -- robot goes LIMP",
    ]


def parse_command(line):
    """Parse one typed or spoken command. Refuses rather than guessing.

    Three rules, each bought with a real failure:

    * A mistyped number is an ERROR, never a default. A command nobody typed is
      indistinguishable from a command that was typed and ignored.
    * A bare motion verb is an ERROR. `walk` used to move the robot at 0.1 m/s
      for 2 s, so a truncated input -- a mis-transcription, an early Enter --
      was a motion command with numbers nobody chose.
    * Trailing tokens are an ERROR, never dropped. `arms wave right` used to
      yield pose='wave', safe only because 'wave' happens not to be a pose.

    Trailing punctuation is stripped, because a transcriber emits "Stop." and
    an operator means stop. That is normalisation, not guessing: it cannot turn
    one valid command into a different valid command.
    """
    raw = (line or "").strip()
    # `task`/`ask` carry FREE TEXT to the model. They are handled before the
    # tokeniser because punctuation stripping and the trailing-token refusal
    # exist to protect the VERB grammar; applying them to a sentence would
    # mangle the prompt. Case and punctuation are preserved exactly.
    head, _, tail = raw.partition(" ")
    if head.lower() in ("task", "ask"):
        prompt = tail.strip()
        if not prompt:
            return "error", {"message": f"{head.lower()} needs a prompt, e.g. "
                                        f"`task wave your right hand`"}
        return "task", {"prompt": prompt}

    parts = raw.split()
    if not parts:
        return "noop", {}
    # Strip punctuation a speaker never intended, from the verb and the tail.
    parts = [w.strip(".,!?;:").strip() for w in parts]
    parts = [w for w in parts if w]
    if not parts:
        return "noop", {}
    verb = parts[0].lower()
    rest = parts[1:]

    def too_many(n):
        return "error", {"message": f"{verb} takes at most {n} argument(s); "
                                    f"got {' '.join(rest)!r}"}

    if verb in ("q", "quit", "exit"):
        return ("quit", {}) if not rest else too_many(0)
    if verb == "status":
        return ("status", {}) if not rest else too_many(0)
    if verb == "stop":
        return ("stop", {}) if not rest else too_many(0)
    if verb == "estop":
        return ("estop", {}) if not rest else too_many(0)
    if verb == "reset":
        return ("reset", {}) if not rest else too_many(0)
    if verb in ("contacts", "hands", "release"):
        return (verb, {}) if not rest else too_many(0)
    if verb == "joints":
        if len(rest) > 1:
            return too_many(1)
        return "joints", ({"limb": rest[0]} if rest else {})
    if verb == "arms":
        if not rest:
            return "error", {"message": "arms needs a pose name: "
                                        + ", ".join(_pose_names())}
        if len(rest) > 1:
            joined = "_".join(rest)
            names = _pose_names()
            hint = (f" Did you mean {joined!r}?" if joined in names else
                    f" Poses are one word: {', '.join(names)}")
            return "error", {"message": f"arms got {' '.join(rest)!r}.{hint}"}
        return "arms", {"pose": rest[0]}
    if verb == "pose":
        if len(rest) != 1:
            return "error", {"message": "pose needs exactly one name"}
        return "pose", {"pose": rest[0]}
    if verb == "height":
        if len(rest) != 1:
            return "error", {"message": "height needs one number in metres"}
        try:
            return "height", {"height_m": float(rest[0])}
        except ValueError:
            return "error", {"message": f"height needs a number: {rest[0]!r}"}
    if verb in ("walk", "turn"):
        if not rest:
            unit = "m/s" if verb == "walk" else "rad/s"
            return "error", {
                "message": f"{verb} needs a speed and a duration, e.g. "
                           f"`{verb} {0.1 if verb == 'walk' else 0.2} 2`. "
                           f"It will not move on a bare `{verb}` -- speed is "
                           f"in {unit} and nobody should inherit a default "
                           f"they did not choose."}
        if len(rest) > 2:
            return too_many(2)
        try:
            rate = float(rest[0])
            seconds = float(rest[1]) if len(rest) > 1 else 2.0
        except ValueError:
            return "error", {"message": f"{verb} needs numbers: "
                                        f"{' '.join(rest)!r}"}
        return verb, ({"speed": rate, "seconds": seconds} if verb == "walk"
                      else {"yaw_rate": rate, "seconds": seconds})
    return "error", {"message": f"unknown command {verb!r}"}


# A battery hold longer than this is a typo, not an experiment. 60 s covers
# every legitimate settle/thermal-observation pause in sim_match_v1 with room.
MAX_SCRIPT_SLEEP_S = 60.0

# Labels become comparator output filenames (seg_<label>_<joint>.svg) and
# segment keys, so the grammar is the intersection of "safe filename" and
# "greppable token".
_MARK_LABEL = __import__("re").compile(r"[A-Za-z0-9][A-Za-z0-9_-]*\Z")


def parse_script_line(line):
    """Parse one battery line. Two directives, then the keyboard grammar.

    A battery exists to be replayed bit-for-bit against the simulator, so
    anything nondeterministic or interactive is refused at PARSE time:
    `task`/`ask` go to a model, `quit`/`reset` are conversation acts, and
    `estop` is a HUMAN act -- a scripted one would either fire at a moment
    nobody chose or teach the reader that estop lines are routine.
    """
    raw = (line or "").strip()
    if not raw or raw.startswith("#"):
        return "noop", {}
    parts = raw.split()
    verb = parts[0].lower()
    if verb == "sleep":
        if len(parts) != 2:
            return "error", {"message": "sleep takes exactly one number of "
                                        "seconds"}
        try:
            s = float(parts[1])
        except ValueError:
            return "error", {"message": f"sleep needs a number: {parts[1]!r}"}
        if not (0 <= s <= MAX_SCRIPT_SLEEP_S):
            return "error", {"message": f"sleep {s:g} is outside "
                                        f"[0, {MAX_SCRIPT_SLEEP_S:g}] s -- "
                                        f"a longer hold is a typo, not an "
                                        f"experiment"}
        return "sleep", {"seconds": s}
    if verb == "mark":
        if len(parts) != 2 or not _MARK_LABEL.match(parts[1]):
            return "error", {"message": "mark takes one label of "
                                        "[A-Za-z0-9_-] (it becomes a "
                                        "filename and a segment key)"}
        return "mark", {"label": parts[1]}
    if verb in ("task", "ask"):
        return "error", {"message": f"{verb} consults a model; a battery "
                                    f"must be deterministic so the same "
                                    f"file can replay against the sim"}
    act, kw = parse_command(raw)
    if act in ("task", "quit", "reset", "estop"):
        return "error", {"message": f"{act} is interactive-only; it has no "
                                    f"place in a battery"}
    return act, kw


def load_script(path):
    """Load and validate a whole battery file BEFORE anything moves.

    One bad line refuses the file: a battery that dies on line 9 leaves the
    robot mid-experiment and a capture no comparator can use. Two marks
    minimum, because segments are the space BETWEEN consecutive marks -- a
    battery without them records data nothing can align.
    """
    import hashlib
    try:
        with open(path, "rb") as f:
            data = f.read()
    except OSError as exc:
        raise SafetyRefusal(f"REFUSED: cannot read battery {path!r}: {exc}")
    steps, problems, n_marks = [], [], 0
    for lineno, raw_line in enumerate(data.decode("utf-8").splitlines(), 1):
        act, kw = parse_script_line(raw_line)
        if act == "noop":
            continue
        if act == "error":
            problems.append(f"line {lineno}: {raw_line.strip()!r} -- "
                            f"{kw['message']}")
            continue
        if act == "mark":
            n_marks += 1
        steps.append((lineno, raw_line.strip(), act, kw))
    if problems:
        raise SafetyRefusal("REFUSED: battery has invalid lines; nothing "
                            "ran:\n  " + "\n  ".join(problems))
    if not steps:
        raise SafetyRefusal(f"REFUSED: battery {path!r} has no steps")
    if n_marks < 2:
        raise SafetyRefusal(
            f"REFUSED: battery {path!r} has {n_marks} mark(s); comparison "
            f"segments live BETWEEN marks, so a usable battery needs at "
            f"least two (e.g. `mark battery-start` ... `mark battery-end`)")
    return {"path": str(path), "sha256": hashlib.sha256(data).hexdigest(),
            "steps": steps}


def _script_stop_reason(telemetry, abort_flag, act_c=None):
    """Why the battery must stop BEFORE the next step, or None.

    The thermal watch thread acts on the robot; this keeps the runner from
    QUEUEING more motion behind an intervention. The direct verdict check is
    the belt for sessions where the watch thread is off.
    """
    if abort_flag and abort_flag.get("tripped"):
        return "thermal watch tripped"
    if telemetry is not None:
        from physicalagent.robots import sonic_telemetry as T
        try:
            v = (T.thermal_verdict(telemetry.thermal()) if act_c is None else
                 T.thermal_verdict(telemetry.thermal(), act_c=act_c))
        except Exception as exc:
            # A guard that cannot see the motors mid-battery stops the
            # battery. Batteries are cheap to re-run; a cooked motor is not.
            return f"thermal read failed: {exc!r}"
        if v["act"]:
            return f"thermal: {v['message']}"
    return None


def run_script(adapter, telemetry, script, log_path, *, real=False,
               abort_flag=None, sleep_fn=None, act_c=None, argv=None):
    """Run a validated battery, writing the JSONL log that aligns captures.

    The log is the join key between this run and its DDS capture, and between
    the hardware capture and the sim capture -- every event carries BOTH
    clocks (t_wall joins across processes; t_mono orders within one). Lines
    are flushed as written so a crash keeps everything up to the crash.

    Fail-stop: after a failed step every later step measures a robot in an
    unknown pose, so the first failure aborts the battery, logged with why.
    """
    import json
    sleep_fn = sleep_fn or time.sleep
    d = os.path.dirname(log_path)
    if d:
        os.makedirs(d, exist_ok=True)
    steps_run, aborted, reason = 0, False, ""
    with open(log_path, "w", encoding="utf-8") as f:
        def emit(rec):
            f.write(json.dumps(rec) + "\n")
            f.flush()

        def stamped(kind, **fields):
            rec = {"kind": kind, "t_wall": time.time(),
                   "t_mono": time.monotonic()}
            rec.update(fields)
            emit(rec)

        stamped("meta", script=script["path"], sha256=script["sha256"],
                real=bool(real), argv=list(argv or []))
        for lineno, raw, act, kw in script["steps"]:
            why = _script_stop_reason(telemetry, abort_flag, act_c)
            if why:
                aborted, reason = True, why
                stamped("abort", reason=reason, lineno=lineno)
                break
            if act == "mark":
                stamped("mark", label=kw["label"], lineno=lineno)
                steps_run += 1
                continue
            if act == "sleep":
                stamped("sleep", seconds=kw["seconds"], lineno=lineno)
                sleep_fn(kw["seconds"])
                steps_run += 1
                continue
            t_wall, t_mono = time.time(), time.monotonic()
            try:
                results = run_manual(adapter, act, **kw)
                ok, err = True, None
                note = "; ".join(f"{n}: {str(r)[:120]}" for n, r in results)
            except Exception as exc:
                ok, err, note = False, repr(exc), None
            emit({"kind": "verb", "t_wall": t_wall, "t_mono": t_mono,
                  "lineno": lineno, "line": raw, "verb": act, "args": kw,
                  "ok": ok, "error": err, "result": note,
                  "duration_s": round(time.monotonic() - t_mono, 4)})
            if not ok:
                aborted = True
                reason = f"step at line {lineno} failed: {err}"
                stamped("abort", reason=reason, lineno=lineno)
                break
            steps_run += 1
        stamped("end", steps_run=steps_run, ok=not aborted)
    return {"steps_run": steps_run, "aborted": aborted, "reason": reason}


def _pose_names():
    try:
        from physicalagent.robots import sonic_upper_body as ub
        return sorted(ub.POSES)
    except Exception:
        return ["neutral", "arms_down", "arms_up", "arms_forward", "arms_out",
                "carry", "reach_left", "reach_right", "wave_right"]


def _load_voice():
    """The voice module, by path -- scripts/ is not an importable package."""
    import importlib.util
    here = os.path.dirname(os.path.abspath(__file__))
    spec = importlib.util.spec_from_file_location(
        "g1_voice", os.path.join(here, "g1_voice.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# The act limit may only be LOWERED. 90 C is SONIC's own warn threshold and
# the highest number anyone here has authority to trust; below ~35 C a limit
# trips on a motor that was never powered (ambient was 31-42 C on 2026-08-19).
THERMAL_ACT_FLOOR_C = 35.0


def validate_thermal_act_c(value):
    """A lowered trip point for deliberately firing the thermal watch.

    The watch has never fired on hardware. Proving the MECHANISM at 55 C
    costs nothing; proving it at 90 C costs a hot motor. Raising the limit is
    refused outright -- that would be trusting an unfired guard with more,
    not less. NaN fails both comparisons and is refused with the rest.
    """
    from physicalagent.robots import sonic_telemetry as T
    v = float(value)
    if not (THERMAL_ACT_FLOOR_C <= v <= T.TEMP_ACT_C):
        raise SafetyRefusal(
            f"REFUSED: --thermal-act-c {value!r} -- the act limit may only "
            f"be lowered, within [{THERMAL_ACT_FLOOR_C:.0f}, "
            f"{T.TEMP_ACT_C:.0f}] C")
    return v


def _thermal_limit():
    """The temperature the watch acts at, read from the one place it is set.

    This was a module-level `import ... as T_MOD` until 2026-08-19, when a
    rewrite of an adjacent function removed the import and left the reference.
    Nothing caught it: no test exercised the --start banner, so it first
    appeared as a NameError AFTER control had been taken on the physical robot.
    """
    from physicalagent.robots import sonic_telemetry as T
    return T.TEMP_ACT_C


def thermal_watch_step(adapter, thermal, act_c=None):
    """One tick of the thermal watch. Returns True if it intervened.

    Split out from the loop so the POLICY is testable without a robot and
    without a thread. See TestTheThermalWatchStopsTheRobotWithoutBeingAsked
    for the measurement that sets the thresholds.

    `act_c` lowers the trip point (see validate_thermal_act_c); None keeps
    the default. The guard has never fired on hardware -- a deliberate trip
    at a lowered, harmless temperature is how it earns its first firing.

    Never raises: this runs on a background thread, and a guard that kills the
    session takes the operator's prompt away at the worst possible moment.
    """
    from physicalagent.robots import sonic_telemetry as T
    v = (T.thermal_verdict(thermal) if act_c is None
         else T.thermal_verdict(thermal, act_c=act_c))
    if not v["act"]:
        return False
    print(f"\n\n  !! THERMAL STOP -- {v['message']}\n")
    # Locomotion first, then unload the arms. A held raised-arm pose is what
    # heats the shoulder driver; the capture shows it cooling as soon as the
    # arm comes down.
    for what, fn in (("stop_all", lambda: adapter.stop_all()),
                     ("set_arms", lambda: adapter.execute(
                         "set_arms", {"pose": "neutral"}))):
        try:
            fn()
        except Exception as exc:
            print(f"  !! thermal stop: {what} FAILED ({exc!r}) -- "
                  f"USE THE PHYSICAL E-STOP")
    return True


class ThermalEscalation:
    """Decide when stop+neutral has provably NOT removed the load.

    2026-08-20, live: the watch fired at 90 C, ran stop_all + arms neutral --
    and right_ankle_roll kept climbing to 93 C, because the heat source was
    the LEGS: the balance policy pressing its commanded stance into the floor
    at 24-29 N.m while the gantry held the body. The first intervention
    removes an ARM load; if act persists past the grace period, the only
    load-removing act left is damp. The robot hangs on the gantry, which the
    TWO-KEY tier mandates for exactly this case.

    Pure state machine so the policy is testable without a thread or clock:
    feed it (is the verdict still act?, now). Fires at most once per
    continuous act window; a clear reading resets the window.
    """

    def __init__(self, after_s=10.0):
        self.after_s = after_s
        self._act_since = None
        self.fired = False

    def step(self, act, now_s):
        if not act:
            self._act_since = None
            self.fired = False
            return False
        if self._act_since is None:
            self._act_since = now_s
        if not self.fired and now_s - self._act_since >= self.after_s:
            self.fired = True
            return True
        return False


def thermal_watch(adapter, telemetry, stop_flag, period_s=0.2, act_c=None,
                  escalate_after_s=10.0):
    """Poll temperatures and act. Runs on its own thread for as long as the
    session holds control, because the operator provably cannot do it: motor 22
    went 65 -> 91 C in 6.5 s on 2026-08-19 while being watched."""
    import time as _t
    esc = ThermalEscalation(after_s=escalate_after_s)
    while not stop_flag["stop"]:
        try:
            acted = telemetry is not None and thermal_watch_step(
                adapter, telemetry.thermal(), act_c=act_c)
            if acted:
                stop_flag["tripped"] = True
            if esc.step(acted, _t.monotonic()):
                print("\n\n  !! STILL OVERHEATING after stop + arms neutral "
                      "-- the load is NOT in the arms. DAMPING. The robot "
                      "will hang on the gantry.\n")
                try:
                    adapter.execute("emergency_stop", {})
                except Exception as exc:
                    print(f"  !! damp FAILED ({exc!r}) -- "
                          f"USE THE PHYSICAL E-STOP")
        except Exception:
            pass                      # never let the watch kill the session
        _t.sleep(period_s)


def run_manual(adapter, kind, *, pose=None, seconds=2.0, speed=0.1,
               yaw_rate=0.2, height_m=None, limb=None):
    out = []
    if kind == "stop":
        out.append(("stop_all", adapter.stop_all()))
    elif kind == "estop":
        out.append(("emergency_stop", adapter.execute("emergency_stop", {})))
    elif kind == "contacts":
        out.append(("get_contacts", adapter.execute("get_contacts", {})))
    elif kind == "hands":
        out.append(("where_are_my_hands",
                    adapter.execute("where_are_my_hands", {})))
    elif kind == "release":
        out.append(("release_arms", adapter.execute("release_arms", {})))
    elif kind == "joints":
        out.append(("get_joints", adapter.execute(
            "get_joints", {"limb": limb} if limb else {})))
    elif kind == "pose":
        out.append(("set_pose", adapter.execute("set_pose", {"name": pose})))
    elif kind == "height":
        out.append(("set_height", adapter.execute(
            "set_height", {"height_m": height_m})))
    elif kind == "turn":
        out.append(("turn", adapter.execute(
            "turn", {"yaw_rate": yaw_rate, "duration_s": seconds})))
    elif kind == "arms":
        # "pose" and "vx" are the names the ToolSpecs REQUIRE. These read
        # "name" and "speed" until 2026-08-19, so every typed command on the
        # physical robot was refused -- the mock adapter in the tests accepted
        # any key, so the suite never saw it.
        out.append(("set_arms", adapter.execute(
            "set_arms", {"pose": pose or "neutral"})))
    elif kind == "walk":
        out.append(("walk", adapter.execute(
            "walk", {"vx": speed, "duration_s": seconds})))
    elif kind == "status":
        out.append(("get_status", adapter.execute("get_status", {})))
    else:
        raise SafetyRefusal(f"unknown manual action {kind!r}")
    return out


def brain_for(holder, factory):
    """One Brain per SESSION, not per prompt.

    Brain keeps its conversation in self._convo across run() calls, so reusing
    it is what makes "now do that again, slower" mean anything. Building a new
    one per prompt would silently throw that away and look like the model had
    simply ignored the follow-up.
    """
    if holder.get("brain") is None:
        holder["brain"] = factory()
    return holder["brain"]


def reset_brain(holder):
    """Clear the model's memory, keeping the robot exactly as it is."""
    b = holder.get("brain")
    if b is not None and hasattr(b, "reset"):
        b.reset()
    return b


def render_trace(trace, width=100):
    """One readable line per tool call.

    The first ER-2 run printed ~8 KB of repr() for five calls. At a gantry that
    is not a log, it is a wall -- and a wrong action scrolling past unnoticed is
    the failure mode. Keep the verb, the argument that identifies it, whether it
    worked, and how long it took; drop the joint tables.
    """
    out = []
    for t in trace or []:
        name = getattr(t, "name", "?")
        args = getattr(t, "args", {}) or {}
        ok = getattr(t, "ok", True)
        err = getattr(t, "error", None)
        secs = getattr(t, "elapsed_s", 0.0) or 0.0
        shown = ", ".join(f"{k}={v}" for k, v in list(args.items())[:3])
        mark = "  " if ok else "!!"
        line = f"  {mark} {name}({shown})"
        if err:
            line += f"  FAIL: {err}"
        line += f"  [{secs:.1f}s]"
        if len(line) > width:
            keep = width - 3 - len(f"  [{secs:.1f}s]")
            line = line[:keep] + "..." + f"  [{secs:.1f}s]"
        out.append(line)
    return out


SYSTEM_PROMPT = (
    "You are operating a PHYSICAL Unitree G1 humanoid that is SUSPENDED IN "
    "A GANTRY and not bearing its own weight. Its feet are off the ground, "
    "so walking will move its legs in the air and will not translate it. "
    "There is no fall detection. Make small, slow, deliberate movements and "
    "call get_status between them. Prefer arm motion over locomotion. Stop "
    "and say so if anything reads unexpectedly.\n\n"
    "A thermal watchdog is running underneath you and will stop the robot and "
    "return the arms to neutral by itself if any motor driver reaches 90 C. "
    "right_shoulder_pitch reached 106 C on 2026-08-19 holding a raised arm, "
    "climbing 215 C/min. So do NOT hold an extended arm pose for long: return "
    "to neutral between poses, and check get_status if you have held one for "
    "more than a few seconds."
)


def build_brain(adapter, max_hops=30, announce=True):
    """One Brain, shared by the one-shot --task path and the interactive one.

    Its conversation persists across run() calls, which is what makes a
    follow-up prompt ("now do that again, slower") mean anything.
    """
    from physicalagent.brain.er import Brain, GeminiClient
    # physicalagent.apikey exists precisely for this: it parses .env without
    # importing dotenv, which .venv_sim does not have. Reading os.environ alone
    # meant the key sat in .env and the session insisted it was missing.
    from physicalagent.apikey import load_gemini_key
    from physicalagent.brain import er as _er
    key = load_gemini_key()
    model = _er.default_model()
    if announce:
        print(f"[session] Gemini Robotics-ER model: {model}")
        print("[session] SONIC holds balance; ER only calls the tools above it.")
    return Brain(adapter, GeminiClient(key, model=model), SYSTEM_PROMPT,
                 max_hops=max_hops)


def run_task(adapter, task, max_hops=30):
    return build_brain(adapter, max_hops).run(task)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--real", action="store_true",
                    help="command the physical robot through SONIC")
    ap.add_argument("--i-am-at-the-estop", dest="estop", action="store_true",
                    help="a distinct e-stop holder is present, hand on the stop")
    ap.add_argument("--dry", action="store_true",
                    help="print the tool surface and exit; commands nothing")
    ap.add_argument("--check", action="store_true",
                    help="read-only: is SONIC actually in control? commands nothing")
    ap.add_argument("--start", action="store_true",
                    help="start control and HOLD it, taking typed commands. "
                         "Required with zmq_manager, which has no keyboard.")
    ap.add_argument("--manual", choices=("walk", "arms", "stop", "status"))
    ap.add_argument("--script", default="",
                    help="run a deterministic verb battery from a file, "
                         "writing a timestamped JSONL log; the same file "
                         "replays against the simulator off-robot so the two "
                         "captures can be compared (see batteries/)")
    ap.add_argument("--script-log", default="",
                    help="where the battery's JSONL log goes (default: "
                         "artifacts/sessions/<ts>-script.jsonl)")
    ap.add_argument("--thermal-act-c", type=float, default=None,
                    help="LOWER the thermal watch trip point (never raise; "
                         "35..90). Firing the never-yet-fired guard at 55 C "
                         "proves the mechanism without cooking a motor.")
    ap.add_argument("--pose", default="neutral")
    ap.add_argument("--seconds", type=float, default=2.0)
    ap.add_argument("--speed", type=float, default=0.1)
    ap.add_argument("--task", default="")
    ap.add_argument("--port", type=int, default=5556)
    ap.add_argument("--max-hops", type=int, default=30)
    ap.add_argument("--auto-damp", action="store_true",
                    help="let the fall guard damp the robot by itself. Off by "
                         "default: its envelopes assume standing heights and "
                         "this robot hangs.")
    ap.add_argument("--voice", action="store_true",
                    help="speak commands instead of typing them (needs a mic "
                         "and API credits; refuses loudly without either)")
    ap.add_argument("--no-thermal-guard", action="store_true",
                    help="do NOT stop the robot on an overheating motor")
    ap.add_argument("--no-telemetry", action="store_true",
                    help="do not attach telemetry (status readouts go dark)")
    a = ap.parse_args()

    if a.dry:
        link = make_link(real=False)
        adapter = make_adapter(link)
        print(reality_block(False))
        print("\nTOOL SURFACE Gemini would receive:\n")
        for d in adapter.gemini_declarations():
            print(f"  {d['name']:18s} {d['description'][:66]}...")
        return 0

    if a.check:
        telemetry, note = try_telemetry()
        if telemetry is None:
            print(f"[check] {note}", file=sys.stderr)
        else:
            telemetry.wait_for_data(timeout_s=8.0)
            time.sleep(0.5)
        state = read_state(telemetry)
        print("\n".join(render_check(state)))
        with contextlib.suppress(Exception):
            telemetry.close()
        return 0 if (state["controller"] or {}).get("active") else 1

    if not (a.manual or a.task or a.start or a.script):
        print("nothing to do: pass --check, --dry, --start, --manual, "
              "--task or --script", file=sys.stderr)
        return 2

    # needs_estop() decides WHETHER the two-key assertion applies; check_ready()
    # enforces it. Until 2026-08-19 needs_estop existed but was never called,
    # so the assertion was demanded for simulated runs too.
    #
    # The battery file and the act-limit override are validated HERE, before
    # any link exists: a refusal must cost nothing and touch nothing.
    script = None
    act_c = None
    try:
        if a.thermal_act_c is not None:
            act_c = validate_thermal_act_c(a.thermal_act_c)
        if a.script:
            script = load_script(a.script)
        if needs_estop(a.check, a.dry, a.manual, a.task or a.start, a.real,
                       script=bool(a.script)):
            check_ready(a.estop)
        else:
            ok, detail = default_preflight()
            if not ok and a.real:
                raise SafetyRefusal(f"REFUSED: preflight failed -- {detail}")
    except SafetyRefusal as e:
        print(f"\n{e}\n", file=sys.stderr)
        return 3

    telemetry, tel_note = (None, "not attached (--no-telemetry)") \
        if a.no_telemetry else try_telemetry()
    print(reality_block(a.real, tel_note, a.auto_damp))
    link = make_link(real=a.real, port=a.port)
    adapter = make_adapter(link, telemetry=telemetry, auto_damp=a.auto_damp)
    t0 = time.time()
    with stopping(adapter, link):
        if a.real:
            print("\n[session] start_control() ...")
            print("[session]", link.start_control())
            time.sleep(2.0)
        if a.manual:
            for name, r in run_manual(adapter, a.manual, pose=a.pose,
                                      seconds=a.seconds, speed=a.speed):
                print(f"[session] {name}: {r}")
        watch_flag = {"stop": False, "tripped": False}
        if a.start or a.script:
            if telemetry is not None and not a.no_thermal_guard:
                import threading
                threading.Thread(
                    target=thermal_watch,
                    args=(adapter, telemetry, watch_flag),
                    kwargs={"act_c": act_c},
                    daemon=True).start()
                limit = act_c if act_c is not None else _thermal_limit()
                lowered = (" (LOWERED from "
                           f"{_thermal_limit()} C to fire the guard on "
                           f"purpose)" if act_c is not None else "")
                print(f"\n[session] thermal watch ARMED "
                      f"(stop + arms neutral at {limit:g} C{lowered}, "
                      f"polled 5x/s). It acts on its own: motor 22 went "
                      f"65 -> 91 C in 6.5 s on 2026-08-19.")
            else:
                print("\n[session] thermal watch OFF -- nothing is watching "
                      "motor temperature. Watch it yourself with `status`, "
                      "knowing a driver can gain 20 C in 6 s.")
        script_aborted = False
        if a.script:
            log_path = a.script_log or os.path.join(
                "artifacts", "sessions",
                time.strftime("%Y%m%d-%H%M%S") + "-script.jsonl")
            print(f"\n[script] {script['path']} "
                  f"({len(script['steps'])} steps) -> {log_path}")
            summary = run_script(adapter, telemetry, script, log_path,
                                 real=a.real, abort_flag=watch_flag,
                                 act_c=act_c, argv=sys.argv)
            script_aborted = summary["aborted"]
            print(f"[script] "
                  f"{'ABORTED -- ' + summary['reason'] if script_aborted else 'complete'}"
                  f" ({summary['steps_run']} steps run)")
            print(f"[script] log: {log_path} -- keep it BESIDE the capture "
                  f"directory; the comparator needs both.")
        if a.start:
            print("\n[session] control held. Commands:")
            for ln in interactive_help():
                print(ln)
            print()
            brain_holder = {}
            voice = None
            if a.voice:
                voice = _load_voice()
                print(voice.banner())
                voice_restore = voice.ensure_microphone_profile()
            while True:
                try:
                    if voice is not None:
                        input("\n[voice] Enter to record, or type a command "
                              "at g1> ... ")
                        heard = voice.one_utterance()
                        print(f"  heard: {heard['transcript']!r}"
                              if heard["transcript"] is not None else
                              "  heard: (nothing)")
                        if not heard["ok"]:
                            print(f"  {heard['message']}")
                            continue
                        act, kw = heard["command"]
                        if voice.needs_confirmation(act):
                            print(f"  -> {heard['message']}")
                            if not voice.confirmed(input("  [y] to run: ")):
                                print("  discarded")
                                continue
                    else:
                        line = input("g1> ")
                        act, kw = parse_command(line)
                except EOFError:
                    break
                if act == "quit":
                    break
                if act == "noop":
                    continue
                if act == "error":
                    print(f"  refused: {kw['message']}")
                    continue
                if act == "status":
                    print("\n".join(render_check(read_state(telemetry))))
                    continue
                if act == "reset":
                    reset_brain(brain_holder)
                    print("  model memory cleared; the robot was not touched")
                    continue
                if act == "task":
                    try:
                        b = brain_for(brain_holder,
                                      lambda: build_brain(adapter, a.max_hops))
                        t0 = time.time()
                        res = b.run(kw["prompt"])
                        print(f"\n  {getattr(res, 'final_text', res)}")
                        tr = getattr(res, "trace", []) or []
                        if tr:
                            print(f"\n  {len(tr)} tool call(s):")
                            print("\n".join(render_trace(tr)))
                        print(f"  [{time.time() - t0:.1f}s]\n")
                    except Exception as e:
                        print(f"  task failed: {e!r}")
                    continue
                try:
                    for name, r in run_manual(adapter, act, **kw):
                        print(f"  {name}: {r}")
                except Exception as e:
                    print(f"  command failed: {e!r}")
        if a.task:
            res = run_task(adapter, a.task, max_hops=a.max_hops)
            print(f"\n[session] {getattr(res, 'final_text', None) or getattr(res, 'text', res)}")
            trace = getattr(res, "trace", []) or []
            if trace:
                print(f"\n[session] {len(trace)} tool call(s):")
                print("\n".join(render_trace(trace)))
            think = getattr(res, "think_s", None)
            if think:
                print(f"\n[session] model thinking time {think:.1f}s")
    with contextlib.suppress(Exception):
        if telemetry is not None:
            telemetry.close()
    print(f"\n[session] stopped after {time.time()-t0:.1f}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
