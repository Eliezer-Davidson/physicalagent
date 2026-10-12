#!/usr/bin/env python3
"""LOCKED tier -- provenance dump. Subscribe only; nothing here can move the robot.


hw_recorder answers "what did the robot DO". This answers "what WAS the robot",
which is the question that stops being answerable once a hardware session ends.
Every claim made away from the robot cites a machine, a firmware, a resting pose and a
mounting position that nobody will be able to look up.

Deliberately reflective: describe_message walks whatever the message actually
carries rather than a field list written today. A firmware revision that adds a
field lands in the dump without anyone editing this repo first -- which matters,
because there may be no second chance to re-run it against the real robot before
the next session.

The recorder's fixed 500 Hz struct drops LowState_.version (firmware),
MotorState_.motorstate (the per-joint fault word), .temperature and .vol. Those
are constants across a session -- worthless to stream, unrecoverable once gone.

USAGE
    WBC=${WBC_ROOT:-$HOME/GR00T-WholeBodyControl}
    $WBC/.venv_sim/bin/python scripts/hw_provenance.py --label day1
    # ... take the photographs PHOTOS.md asks for, into <run>/photos/ ...
    python3 scripts/hw_provenance.py --verify artifacts/hw/<run-dir>
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import os
import pathlib
import platform
import subprocess
import sys
import time

ART = pathlib.Path(__file__).resolve().parents[1] / "artifacts" / "hw"
REPO = pathlib.Path(__file__).resolve().parents[1]

# Streams snapshotted. Every one is a SUBSCRIBE; there is no publisher in this
# file, and tests/test_hw_capture.py asserts that stays true.
SNAPSHOT_TOPICS = ["rt/lowstate", "rt/odostate", "rt/secondary_imu"]

def _opt_env(name, default=None):
    """An external checkout's root, from the environment. None when this
    machine has none -- never guessed, and never a hard-coded home."""
    value = os.environ.get(name)
    if value:
        return pathlib.Path(value).expanduser()
    return default


# Other checkouts whose SHA a later question may reduce to. Every one is an
# EXTERNAL project with an env-configurable location. Missing ones are recorded
# as absent rather than skipped -- "this machine did not have it" is itself
# provenance.
SIBLING_REPOS = {
    "physicalagent": REPO,
    "GR00T-WholeBodyControl": _opt_env(
        "WBC_ROOT", pathlib.Path.home() / "GR00T-WholeBodyControl"),
    "Isaac-GR00T": _opt_env("ISAAC_GROOT_ROOT"),
    "unitree_sim_isaaclab": _opt_env("UNITREE_SIM_ROOT"),
}

PACKAGES = ["cyclonedds", "unitree_sdk2py", "numpy", "mujoco"]

# Environment that changes what the DDS stack does. CYCLONEDDS_HOME picks which
# native core loads; getting it wrong is silent (2026-08-07).
ENV_OF_INTEREST = ["CYCLONEDDS_HOME", "CYCLONEDDS_URI", "LD_LIBRARY_PATH",
                   "LD_PRELOAD", "ROS_DOMAIN_ID", "CONDA_DEFAULT_ENV",
                   "VIRTUAL_ENV", "PYTHONPATH", "PYTHONNOUSERSITE"]


@dataclasses.dataclass(frozen=True)
class Shot:
    id: str
    what: str
    why: str


# Photographs are the half of provenance that no script can take. The list is
# here, in code, because the checklist has to be generated INTO the run
# directory -- a checklist that lives in a document is a checklist nobody has
# on them at the robot.
SHOT_LIST = [
    Shot("nameplate",
         "Serial/model nameplate and every regulatory sticker, close enough to read",
         "The exact hardware revision is what every later question reduces to, and "
         "on this robot it exists only as a sticker."),
    Shot("full-body-front",
         "Whole robot, front, metre rule in frame",
         "Proportions and any non-stock part are checkable against the URDF only "
         "if a scaled photograph exists."),
    Shot("full-body-left",
         "Whole robot, left side, metre rule in frame",
         "Side profile is the view that shows mounting offsets the front view hides."),
    Shot("full-body-rear",
         "Whole robot, rear, metre rule in frame",
         "Backpack, battery and cable runs are all rear-face and all absent from CAD."),
    Shot("head-sensor-cluster",
         "Head sensor cluster three-quarter view, scale card touching the mount",
         "CAD says where the camera COULD be bolted; only this shows where it IS, "
         "and extrinsics are underivable away from the robot."),
    Shot("head-sensor-cluster-side",
         "Head sensor cluster from the side, same scale card",
         "One view cannot resolve depth of the mount; two intersecting views can."),
    Shot("pelvis-origin-region",
         "Pelvis area with a rule spanning to the sensor mount",
         "Every extrinsic is quoted relative to pelvis, so the measurement needs a "
         "photograph that shows both ends at once."),
    Shot("hand-left",
         "Left end-effector as actually mounted, palm and back",
         "Which hand revision is fitted, and at what wrist clocking, changes every "
         "grasp primitive."),
    Shot("hand-right",
         "Right end-effector as actually mounted, palm and back",
         "The two sides are not reliably identical once anything has been serviced."),
    Shot("wrist-mount-detail",
         "Wrist-to-hand joint close-up, both sides",
         "The adapter plate thickness is a real offset in the kinematic chain and "
         "appears in no document."),
    Shot("foot-soles",
         "Both foot soles, wear pattern visible, scale card",
         "Sole compliance and wear set the friction the simulator has to match, and "
         "wear is specific to this robot on this day."),
    Shot("floor-surface",
         "The actual floor it will stand and walk on, close-up with scale",
         "Contact parameters are a property of the surface, not the robot, and the "
         "surface is not available away from the robot either."),
    Shot("cable-routing",
         "External cable and harness routing at pelvis and torso",
         "Added mass and any tether that constrained motion has to be visible when "
         "a recording is later replayed and looks wrong."),
    Shot("battery-and-payload",
         "Battery and anything bolted on, with the whole robot for context",
         "Mass and inertia offsets from stock explain sim-to-real gaps that would "
         "otherwise be blamed on the controller."),
    Shot("gantry-attachment",
         "Hoist/gantry attachment point and how the robot hangs from it",
         "Whether a run was supported, and where the support pulled, decides "
         "whether that recording means anything."),
    Shot("estop-and-controls",
         "E-stop, its holder's position, and the control station",
         "The safety arrangement is the part of the record that has to survive for "
         "anyone reviewing whether the session was run as signed."),
    Shot("workspace-wide",
         "Whole workspace from the robot's start pose, lights on as they were",
         "The perception corpus is uninterpretable without knowing what scene it "
         "was looking at and under what lighting."),
]


# --- reflective description ---------------------------------------------------

def _public_field_names(obj):
    if dataclasses.is_dataclass(obj) and not isinstance(obj, type):
        return [f.name for f in dataclasses.fields(obj)]
    inst = getattr(obj, "__dict__", None)
    if inst:
        return [k for k in inst if not k.startswith("_")]
    return [a for a in dir(obj)
            if not a.startswith("_") and not callable(getattr(obj, a, None))]


def _describe_value(v, depth=0):
    """JSON-safe recursive description. Never calls a method: a callable
    attribute is skipped, not invoked, because this runs against a live robot's
    message objects and invoking an unknown method there is not a read."""
    if depth > 8:
        return "<max depth>"
    if v is None or isinstance(v, (bool, int, float, str)):
        return v
    if isinstance(v, (bytes, bytearray)):
        return v.hex()
    item = getattr(v, "item", None)          # numpy / array scalars
    if callable(item) and not hasattr(v, "__len__"):
        try:
            return item()
        except Exception:
            pass
    if isinstance(v, dict):
        return {str(k): _describe_value(x, depth + 1) for k, x in v.items()}
    if hasattr(v, "__iter__"):
        try:
            return [_describe_value(x, depth + 1) for x in v]
        except TypeError:
            pass
    names = _public_field_names(v)
    if names:
        out = {}
        for n in names:
            try:
                a = getattr(v, n)
            except Exception as e:
                out[n] = f"<unreadable: {e}>"
                continue
            if callable(a):
                continue
            out[n] = _describe_value(a, depth + 1)
        return out
    return repr(v)


def describe_message(msg):
    """Full contents of one DDS message as a plain dict."""
    d = _describe_value(msg, 0)
    return d if isinstance(d, dict) else {"value": d}


# --- host and network ---------------------------------------------------------

def _run(cmd):
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        return p.stdout.strip() if p.returncode == 0 else None
    except Exception:
        return None


def _git(path):
    if path is None:
        return {"path": None, "present": False, "reason": "root not configured"}
    if not pathlib.Path(path).exists():
        return {"path": str(path), "present": False}
    g = ["git", "-C", str(path)]
    status = _run(g + ["status", "--porcelain"])
    return {
        "path": str(path),
        "present": True,
        "sha": _run(g + ["rev-parse", "HEAD"]),
        "branch": _run(g + ["rev-parse", "--abbrev-ref", "HEAD"]),
        "describe": _run(g + ["describe", "--always", "--dirty", "--tags"]),
        "dirty": bool(status) if status is not None else None,
        "dirty_files": status.splitlines() if status else [],
    }


def _package_version(name):
    try:
        import importlib.metadata as md
        return md.version(name)
    except Exception:
        try:
            mod = __import__(name)
            return getattr(mod, "__version__", "(installed, version unknown)")
        except Exception:
            return None


def collect_host():
    """Everything about THIS machine that a later reader would have to guess."""
    return {
        "hostname": platform.node(),
        "wall_time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "uname": " ".join(platform.uname()),
        "python": {
            "executable": sys.executable,
            "version": sys.version.split()[0],
            "version_full": sys.version.replace("\n", " "),
            "prefix": sys.prefix,
        },
        "packages": {p: _package_version(p) for p in PACKAGES},
        "repos": {n: _git(p) for n, p in SIBLING_REPOS.items()},
        "env": {k: os.environ.get(k) for k in ENV_OF_INTEREST},
        "cwd": os.getcwd(),
        "argv": list(sys.argv),
    }


def collect_network():
    """Which interface the robot is on, and who answered on 192.168.123.x.
    Read-only: interface listing and the kernel's existing neighbour cache."""
    addrs = _run(["ip", "-o", "-4", "addr", "show"]) or ""
    ifaces = []
    for line in addrs.splitlines():
        parts = line.split()
        if len(parts) >= 4:
            ifaces.append({"iface": parts[1], "cidr": parts[3]})
    neigh = (_run(["ip", "neigh", "show"]) or "").splitlines()
    return {
        "interfaces": ifaces,
        "robot_subnet_interfaces": [i for i in ifaces
                                    if i["cidr"].startswith("192.168.123.")],
        "neighbours": neigh,
        "robot_subnet_neighbours": [n for n in neigh
                                    if n.startswith("192.168.123.")],
    }


# --- writing and verifying ----------------------------------------------------

def _unique_run_dir(out_root, label):
    """Run-stamped, and it refuses to reuse a directory even inside the same
    second. On 2026-08-18 this project lost five of eight evidence pairs to a
    fixed output name; provenance is written once and is exactly the artifact
    whose disappearance nobody would notice."""
    stamp = time.strftime("%Y%m%d-%H%M%S")
    n, d = 1, out_root / f"{stamp}-{label}-provenance"
    while d.exists():
        n += 1
        d = out_root / f"{stamp}-{label}-provenance-{n}"
    d.mkdir(parents=True)
    return d


def _photos_md(run_dir):
    lines = [
        "# Photographs -- take these before the session ends",
        "",
        f"Run: `{run_dir.name}`",
        "",
        "Save each one into `photos/` named `<id>-NN.jpg`. `hw_provenance.py",
        "--verify` refuses to pass until every id below has at least one",
        "non-empty file, because a provenance dump without the photographs is",
        "the half that a script could do, not the half that mattered.",
        "",
        "Include a scale reference in every frame that has one requested.",
        "",
    ]
    for i, s in enumerate(SHOT_LIST, 1):
        lines += [f"## {i}. `{s.id}`", "", f"**Shot:** {s.what}", "",
                  f"**Why it cannot wait:** {s.why}", ""]
    return "\n".join(lines)


def write_provenance(out_root, label, host, network, robot):
    run_dir = _unique_run_dir(pathlib.Path(out_root), label)
    (run_dir / "photos").mkdir()
    payload = {
        "stamp": run_dir.name,
        "label": label,
        "tier": "LOCKED (subscribe only)",
        "wall_time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "host": host,
        "network": network,
        "robot": robot,
        "shot_list": [dataclasses.asdict(s) for s in SHOT_LIST],
    }
    (run_dir / "provenance.json").write_text(json.dumps(payload, indent=2,
                                                        default=str))
    (run_dir / "PHOTOS.md").write_text(_photos_md(run_dir))
    return run_dir


def verify(run_dir):
    """'It ran' is not done. Returns ok=False while anything irreplaceable is
    still missing, so the gap is discovered while the robot is still here."""
    run_dir = pathlib.Path(run_dir)
    problems, warnings = [], []
    pj = run_dir / "provenance.json"
    if not pj.exists():
        return {"ok": False, "problems": ["no provenance.json in the run dir"],
                "warnings": [], "missing_photos": [s.id for s in SHOT_LIST]}
    data = json.loads(pj.read_text())

    if not data.get("robot"):
        problems.append(
            "the robot snapshot is EMPTY -- the subscribe never landed. A silent "
            "stream is not a quiet robot: check topic name, interface, then "
            "whether the robot was running, in that order.")
    if not data.get("host"):
        warnings.append("host block is empty")
    if not data.get("network"):
        warnings.append("network block is empty")

    photos = run_dir / "photos"
    present = [p for p in photos.iterdir()
               if p.is_file() and p.stat().st_size > 0] if photos.is_dir() else []
    missing = [s.id for s in SHOT_LIST
               if not any(p.name.startswith(s.id) for p in present)]

    return {
        "ok": not problems and not missing,
        "problems": problems,
        "warnings": warnings,
        "missing_photos": missing,
        "photos_present": len(present),
        "run_dir": str(run_dir),
    }


def _print_verify(r):
    print(f"[prov] {r['run_dir']}")
    print(f"[prov] photos present: {r['photos_present']}/{len(SHOT_LIST)}")
    for p in r["problems"]:
        print(f"  PROBLEM  {p}")
    for w in r["warnings"]:
        print(f"  warning  {w}")
    for m in r["missing_photos"]:
        print(f"  MISSING PHOTO  {m}")
    print("[prov] " + ("COMPLETE" if r["ok"] else "INCOMPLETE -- not done"))
    return 0 if r["ok"] else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="run")
    ap.add_argument("--out-dir", default=str(ART))
    ap.add_argument("--iface", default="", help="DDS interface; empty = default")
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--timeout", type=float, default=10.0,
                    help="seconds to wait for one message per topic")
    ap.add_argument("--verify", default="",
                    help="check an existing run dir instead of making one")
    a = ap.parse_args()

    if a.verify:
        return _print_verify(verify(a.verify))

    host, network = collect_host(), collect_network()

    robot, errors = {}, {}
    try:
        from unitree_sdk2py.core.channel import (ChannelSubscriber,
                                                 ChannelFactoryInitialize)
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (LowState_, IMUState_,
                                                            OdoState_)
    except Exception as e:                       # readable refusal, not a traceback
        errors["import"] = (f"{e} -- run this with the SDK interpreter, e.g. "
                            f"$WBC/.venv_sim/bin/python")
        print(f"[prov] SDK unavailable: {errors['import']}", file=sys.stderr)
    else:
        TYPES = {"rt/lowstate": LowState_, "rt/odostate": OdoState_,
                 "rt/secondary_imu": IMUState_}
        if a.iface:
            ChannelFactoryInitialize(a.domain, a.iface)
        else:
            ChannelFactoryInitialize(a.domain)
        got, subs = {}, []
        for topic in SNAPSHOT_TOPICS:
            sub = ChannelSubscriber(topic, TYPES[topic])
            sub.Init(lambda m, t=topic: got.setdefault(t, m), 5)
            subs.append(sub)
        deadline = time.monotonic() + a.timeout
        while time.monotonic() < deadline and len(got) < len(SNAPSHOT_TOPICS):
            time.sleep(0.05)
        for topic in SNAPSHOT_TOPICS:
            if topic in got:
                robot[topic] = describe_message(got[topic])
            else:
                errors[topic] = f"no message within {a.timeout:.0f}s"
                print(f"[prov] SILENT: {topic}", file=sys.stderr)
    if errors:
        robot.setdefault("_errors", errors) if robot else None
        host["provenance_errors"] = errors

    run_dir = write_provenance(a.out_dir, a.label, host, network, robot)
    print(f"[prov] -> {run_dir}")
    print(f"[prov] read {run_dir / 'PHOTOS.md'} and shoot {len(SHOT_LIST)} photos "
          f"into {run_dir / 'photos'}")
    return _print_verify(verify(run_dir))


if __name__ == "__main__":
    raise SystemExit(main())
