#!/usr/bin/env python3
"""Record every DDS message the G1 emits, at full rate, to a replayable file.

WHY THIS IS THE MOST IMPORTANT SCRIPT OF THE HARDWARE SESSIONS

    Hardware time on this robot is finite and ends after these sessions. After that,
    every simulator claim is unfalsifiable UNLESS it can be checked against something
    recorded now. A paired (command, state) trace is what makes that possible:
    feed the recorded real rt/lowcmd into the simulator, compare the simulated
    state against the recorded real state, and sim-to-real stops being an
    opinion and becomes a number -- recomputable on every future change, for
    as long as the files exist.

    So this runs whenever the robot is powered, including during failures.
    A robot that falls with 500 Hz telemetry is a BETTER artifact than one that
    stands with no logging, because the fall is precisely what a simulator must
    reproduce before anyone should trust it.

TIER: LOCKED. Subscribes only. Constructs no motor-command object. It cannot
move the robot, so it is safe to leave running unattended.

FORMAT, and why not JSON

    rt/lowstate carries ~29 motors x 4 floats at 500 Hz. As JSON that is
    several GB per hour and unparseable in reasonable time. Each stream is
    therefore a flat binary file of fixed-width little-endian records, plus a
    JSON sidecar giving the exact struct format and field names. Nothing about
    the layout is implicit: read_stream() below reconstructs dicts using only
    the sidecar, so a file recorded today is readable off-robot by code that has
    never seen this script.

ONE CLOCK

    Every record carries time.monotonic(). The header stores the single
    wall<->monotonic offset captured at start. Streams recorded in one session
    are therefore directly comparable, which is the whole point -- a corpus
    whose streams cannot be aligned is a pile of unrelated files.

USAGE

    $WBC/.venv_sim/bin/python scripts/hw_recorder.py --label day1-timebase
    $WBC/.venv_sim/bin/python scripts/hw_recorder.py --selftest   # no DDS needed
"""
from __future__ import annotations

import argparse
import json
import os
import pathlib
import signal
import struct
import subprocess
import sys
import threading
import time

ART = pathlib.Path(__file__).resolve().parents[1] / "artifacts" / "hw"

# The robot's own subnet. Nothing else on this laptop uses it, which is exactly
# why it is a usable test for "is the robot actually on the other end".
ROBOT_NET_PREFIX = "192.168.123."


def host_interfaces():
    """[(iface, ip)] from `ip -o -4 addr`. Empty on any failure -- this feeds a
    refusal, and a crash here would teach the operator nothing."""
    try:
        out = subprocess.run(["ip", "-o", "-4", "addr", "show"],
                             capture_output=True, text=True, timeout=5).stdout
    except Exception:
        return []
    found = []
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 4 and parts[2] == "inet":
            found.append((parts[1], parts[3].split("/")[0]))
    return found


def robot_net_iface(ifaces, want=""):
    """The interface carrying a 192.168.123.x address, or None.

    `want` narrows it to one name -- and does NOT excuse it from existing.
    Passing --iface for a link that is down would otherwise be a way to talk
    past exactly the failure this guard exists to catch."""
    for name, ip in ifaces:
        if ip.startswith(ROBOT_NET_PREFIX) and (not want or name == want):
            return name
    return None


def no_robot_net_message(ifaces):
    seen = ", ".join(f"{n}={ip}" for n, ip in ifaces) or "(no IPv4 at all)"
    return (f"REFUSING TO RECORD: no interface holds a {ROBOT_NET_PREFIX}x "
            f"address, so the robot is not on this machine's network.\n"
            f"  found: {seen}\n"
            f"  DDS would bind to whichever interface happens to be up -- on "
            f"2026-08-19 that was Wi-Fi -- and this run would produce six "
            f"zero-byte files that look like a real capture. The interface is "
            f"chosen once, at startup, so it would stay wrong even after the "
            f"robot came back.\n"
            f"  Bring the link up first (ip -br addr show enp132s0 must show "
            f"UP with 192.168.123.222), then start this again.\n"
            f"  --allow-any-net overrides this, for replay and bench work only.")

# Streams captured. `kind` selects the packer below. Every one of these is a
# SUBSCRIBE; there is no publisher anywhere in this file.
# Measured 2026-08-19 by a subscribe-only scan of the physical G1:
#   LIVE   rt/odommodestate  498.6 Hz     <- the odometry actually published
#   LIVE   rt/lf/lowstate     20.2 Hz
#   silent rt/odostate           0        <- what this list used to name
# rt/odostate is not published by this robot. Naming it produced a zero-byte
# file and a silent-stream warning on every run, which is exactly how a wrong
# topic name hides behind a symptom that looks like a quiet robot.
STREAMS = [
    ("rt/lowstate",       "lowstate"),
    ("rt/lowcmd",         "lowcmd"),
    ("rt/odommodestate",  "odommodestate"),
    ("rt/secondary_imu",  "imu"),
    ("rt/lf/lowstate",    "lowstate"),
    # Same messages as rt/lowstate, decimated, keeping the fields the full-rate
    # packer drops: both temperature sensors, bus voltage and the motor fault
    # word. On 2026-08-19 SONIC warned that an ankle was overheating and the
    # session was stopped; nothing on this machine had logged a temperature, so
    # the warning could not be checked. Temperature moves on a thermal time
    # constant -- 2 Hz answers it for ~2.5 MB/hour.
    ("rt/lowstate",       "thermal"),
]

# Streams carrying slowly-varying data are written at most this often.
THERMAL_PERIOD_S = 0.5


def _f(seq, n):
    """First n floats of a sequence, zero-padded. Vendor arrays are sometimes
    longer than documented; truncating silently would corrupt every later
    record's alignment, so the width is fixed here and recorded in the sidecar."""
    out = [0.0] * n
    if seq is not None:
        for i, v in enumerate(seq):
            if i >= n:
                break
            try:
                out[i] = float(v)
            except (TypeError, ValueError):
                out[i] = 0.0
    return out


class Packer:
    """Builds a fixed-width record layout on the FIRST message, so the motor
    count is whatever the robot actually reports rather than whatever a
    constant in this file guesses."""

    def __init__(self, kind):
        self.kind = kind
        self.fmt = None
        self.fields = None
        self.n_motor = 0

    def _init_from(self, msg):
        k = self.kind
        if k in ("lowstate", "lowcmd", "thermal"):
            self.n_motor = len(getattr(msg, "motor_state", None)
                               or getattr(msg, "motor_cmd", None) or [])
        if k == "lowstate":
            per = ("q", "dq", "ddq", "tau_est")
            self.fields = (["t_mono", "t_wall", "tick", "mode_pr", "mode_machine"]
                           + [f"imu_quat_{i}" for i in range(4)]
                           + [f"imu_gyro_{i}" for i in range(3)]
                           + [f"imu_accel_{i}" for i in range(3)]
                           + [f"imu_rpy_{i}" for i in range(3)]
                           + [f"m{j}_{p}" for j in range(self.n_motor) for p in per])
            self.fmt = "<dd" + "I" + "BB" + "f" * 13 + "f" * (self.n_motor * 4)
        elif k == "lowcmd":
            per = ("q", "dq", "tau", "kp", "kd")
            self.fields = (["t_mono", "t_wall", "crc"]
                           + [f"m{j}_{p}" for j in range(self.n_motor) for p in per])
            self.fmt = "<dd" + "I" + "f" * (self.n_motor * 5)
        elif k == "odostate":
            self.fields = (["t_mono", "t_wall", "tick"]
                           + [f"pos_{i}" for i in range(3)]
                           + [f"quat_{i}" for i in range(4)]
                           + [f"linvel_{i}" for i in range(3)]
                           + [f"angvel_{i}" for i in range(3)])
            self.fmt = "<dd" + "I" + "f" * 13
        elif k == "thermal":
            per = ("temp_winding", "temp_driver", "vol", "motorstate")
            self.fields = (["t_mono", "t_wall", "tick"]
                           + [f"m{j}_{p}" for j in range(self.n_motor)
                              for p in per])
            # h h f I per motor. The temperatures are int16 on the wire and
            # storing them as floats would invent precision the sensor has not.
            self.fmt = "<dd" + "I" + "hhfI" * self.n_motor
        elif k == "odommodestate":
            # SportModeState_ on rt/odommodestate. This is the trajectory
            # reference: pelvis position and velocity in the world, plus
            # foot_force, which is the only contact ground truth the robot
            # gives away. The publisher's own stamp is kept next to our clock
            # so transport delay stays recoverable.
            self.fields = (["t_mono", "t_wall", "stamp_sec", "stamp_nanosec",
                            "error_code", "mode", "gait_type",
                            "progress", "body_height", "yaw_speed",
                            "foot_raise_height"]
                           + [f"pos_{i}" for i in range(3)]
                           + [f"vel_{i}" for i in range(3)]
                           + [f"imu_quat_{i}" for i in range(4)]
                           + [f"imu_gyro_{i}" for i in range(3)]
                           + [f"imu_accel_{i}" for i in range(3)]
                           + [f"imu_rpy_{i}" for i in range(3)]
                           + [f"foot_force_{i}" for i in range(4)]
                           + [f"foot_position_body_{i}" for i in range(12)]
                           + [f"foot_speed_body_{i}" for i in range(12)]
                           + [f"range_obstacle_{i}" for i in range(4)])
            self.fmt = "<dd" + "II" + "I" + "BB" + "f" * 55
        elif k == "imu":
            self.fields = (["t_mono", "t_wall"]
                           + [f"quat_{i}" for i in range(4)]
                           + [f"gyro_{i}" for i in range(3)]
                           + [f"accel_{i}" for i in range(3)]
                           + [f"rpy_{i}" for i in range(3)]
                           + ["temperature"])
            self.fmt = "<dd" + "f" * 14
        else:
            raise ValueError(f"unknown stream kind {k!r}")

    def pack(self, msg, t_mono, t_wall):
        if self.fmt is None:
            self._init_from(msg)
        k = self.kind
        if k == "lowstate":
            imu = getattr(msg, "imu_state", None)
            vals = [t_mono, t_wall, int(getattr(msg, "tick", 0) or 0) & 0xFFFFFFFF,
                    int(getattr(msg, "mode_pr", 0) or 0) & 0xFF,
                    int(getattr(msg, "mode_machine", 0) or 0) & 0xFF]
            vals += _f(getattr(imu, "quaternion", None), 4)
            vals += _f(getattr(imu, "gyroscope", None), 3)
            vals += _f(getattr(imu, "accelerometer", None), 3)
            vals += _f(getattr(imu, "rpy", None), 3)
            for m in (getattr(msg, "motor_state", None) or [])[:self.n_motor]:
                vals += [float(getattr(m, "q", 0.0) or 0.0),
                         float(getattr(m, "dq", 0.0) or 0.0),
                         float(getattr(m, "ddq", 0.0) or 0.0),
                         float(getattr(m, "tau_est", 0.0) or 0.0)]
        elif k == "lowcmd":
            vals = [t_mono, t_wall, int(getattr(msg, "crc", 0) or 0) & 0xFFFFFFFF]
            for m in (getattr(msg, "motor_cmd", None) or [])[:self.n_motor]:
                vals += [float(getattr(m, "q", 0.0) or 0.0),
                         float(getattr(m, "dq", 0.0) or 0.0),
                         float(getattr(m, "tau", 0.0) or 0.0),
                         float(getattr(m, "kp", 0.0) or 0.0),
                         float(getattr(m, "kd", 0.0) or 0.0)]
        elif k == "odostate":
            vals = [t_mono, t_wall, int(getattr(msg, "tick", 0) or 0) & 0xFFFFFFFF]
            vals += _f(getattr(msg, "position", None), 3)
            vals += _f(getattr(msg, "orientation", None), 4)
            vals += _f(getattr(msg, "linear_velocity", None), 3)
            vals += _f(getattr(msg, "angular_velocity", None), 3)
        elif k == "thermal":
            vals = [t_mono, t_wall, int(getattr(msg, "tick", 0) or 0) & 0xFFFFFFFF]
            for m in (getattr(msg, "motor_state", None) or [])[:self.n_motor]:
                temp = getattr(m, "temperature", None) or [0, 0]
                vals += [int(temp[0]) if len(temp) > 0 else 0,
                         int(temp[1]) if len(temp) > 1 else 0,
                         float(getattr(m, "vol", 0.0) or 0.0),
                         int(getattr(m, "motorstate", 0) or 0) & 0xFFFFFFFF]
        elif k == "odommodestate":
            st = getattr(msg, "stamp", None)
            imu = getattr(msg, "imu_state", None)
            vals = [t_mono, t_wall,
                    int(getattr(st, "sec", 0) or 0) & 0xFFFFFFFF,
                    int(getattr(st, "nanosec", 0) or 0) & 0xFFFFFFFF,
                    int(getattr(msg, "error_code", 0) or 0) & 0xFFFFFFFF,
                    int(getattr(msg, "mode", 0) or 0) & 0xFF,
                    int(getattr(msg, "gait_type", 0) or 0) & 0xFF,
                    float(getattr(msg, "progress", 0.0) or 0.0),
                    float(getattr(msg, "body_height", 0.0) or 0.0),
                    float(getattr(msg, "yaw_speed", 0.0) or 0.0),
                    float(getattr(msg, "foot_raise_height", 0.0) or 0.0)]
            vals += _f(getattr(msg, "position", None), 3)
            vals += _f(getattr(msg, "velocity", None), 3)
            vals += _f(getattr(imu, "quaternion", None), 4)
            vals += _f(getattr(imu, "gyroscope", None), 3)
            vals += _f(getattr(imu, "accelerometer", None), 3)
            vals += _f(getattr(imu, "rpy", None), 3)
            vals += _f(getattr(msg, "foot_force", None), 4)
            vals += _f(getattr(msg, "foot_position_body", None), 12)
            vals += _f(getattr(msg, "foot_speed_body", None), 12)
            vals += _f(getattr(msg, "range_obstacle", None), 4)
        else:
            vals = [t_mono, t_wall]
            vals += _f(getattr(msg, "quaternion", None), 4)
            vals += _f(getattr(msg, "gyroscope", None), 3)
            vals += _f(getattr(msg, "accelerometer", None), 3)
            vals += _f(getattr(msg, "rpy", None), 3)
            vals += [float(getattr(msg, "temperature", 0.0) or 0.0)]
        # Pad/truncate defensively: a vendor field that changes width must not
        # silently shift every subsequent record by a few bytes.
        want = len(self.fields)
        if len(vals) != want:
            vals = (vals + [0.0] * want)[:want]
        return struct.pack(self.fmt, *vals)


def build_summary(streams):
    """One summary row PER FILE, keyed by the file's stem.

    The Stream file names were already de-collided ("or the second silently
    truncates the first") -- and summary.json then keyed rows by topic,
    recreating the identical collision one level up: in the 2026-08-19 runs
    the 'rt/lowstate' row silently described the 2 Hz thermal file while the
    900 MB full-rate stream had no row at all. Same trap, second floor.
    """
    return {s.path.stem: {"topic": s.topic, "kind": s.kind,
                          "records": s.n, "hz": round(s.rate(), 1),
                          "bytes": s.path.stat().st_size,
                          "dropped_after_close": s.dropped_after_close,
                          "write_errors": s.write_errors} for s in streams}


class Stream:
    def __init__(self, out_dir, topic, kind):
        self.topic, self.kind = topic, kind
        safe = topic.replace("/", "_")
        # Two streams can share a topic (full-rate and decimated). Distinct
        # file names, or the second silently truncates the first.
        if kind == "thermal":
            safe += "__thermal"
        self.min_period_s = THERMAL_PERIOD_S if kind == "thermal" else 0.0
        self._last_write_mono = None
        self.path = out_dir / f"{safe}.bin"
        self.meta_path = out_dir / f"{safe}.schema.json"
        self.fh = self.path.open("wb")
        self.packer = Packer(kind)
        self.n = 0
        self.first_mono = None
        self.last_mono = None
        self._meta_written = False
        # DDS reader threads keep delivering after close(). Without this the
        # late write raises out of the handler and takes the reader thread
        # down with a traceback (observed on the live robot, 2026-08-19).
        self._closed = False
        self._lock = threading.Lock()
        self.dropped_after_close = 0
        self.write_errors = 0

    def on_msg(self, msg):
        t_mono, t_wall = time.monotonic(), time.time()
        if self.min_period_s:
            last = self._last_write_mono
            if last is not None and (t_mono - last) < self.min_period_s:
                return
            self._last_write_mono = t_mono
        try:
            blob = self.packer.pack(msg, t_mono, t_wall)
        except Exception as exc:                      # never kill the recorder
            sys.stderr.write(f"[rec] {self.topic} pack error: {exc!r}\n")
            return
        with self._lock:
            if self._closed:
                self.dropped_after_close += 1
                return
            try:
                self.fh.write(blob)
            except Exception as exc:              # one message, not the session
                self.write_errors += 1
                sys.stderr.write(f"[rec] {self.topic} write error: {exc!r}\n")
                return
            self.n += 1
            if self.first_mono is None:
                self.first_mono = t_mono
            self.last_mono = t_mono
            if not self._meta_written:
                self.write_meta()
                self._meta_written = True

    def write_meta(self):
        self.meta_path.write_text(json.dumps({
            "topic": self.topic, "kind": self.kind,
            "struct_format": self.packer.fmt,
            "record_bytes": struct.calcsize(self.packer.fmt),
            "fields": self.packer.fields,
            "n_motor": self.packer.n_motor,
            "note": "little-endian fixed-width records; t_mono is "
                    "time.monotonic(), t_wall is time.time(); see session.json "
                    "for the single wall<->monotonic offset for this session",
        }, indent=2))

    def rate(self):
        if self.n < 2 or self.first_mono is None:
            return 0.0
        span = self.last_mono - self.first_mono
        return (self.n - 1) / span if span > 0 else 0.0

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            try:
                self.fh.flush()
                os.fsync(self.fh.fileno())
                self.fh.close()
            except Exception:
                pass
        if self.packer.fmt is not None and not self._meta_written:
            self.write_meta()


def read_stream(bin_path, schema_path=None):
    """Yield one dict per record. Uses ONLY the sidecar, so this function is
    the contract: anything that can read the sidecar can read the corpus at
    home without this repo."""
    bin_path = pathlib.Path(bin_path)
    schema_path = pathlib.Path(schema_path) if schema_path else \
        bin_path.with_suffix("").with_suffix(".schema.json") \
        if bin_path.suffix == ".bin" else None
    if schema_path is None or not schema_path.exists():
        schema_path = bin_path.parent / (bin_path.stem + ".schema.json")
    meta = json.loads(pathlib.Path(schema_path).read_text())
    fmt, fields = meta["struct_format"], meta["fields"]
    size = struct.calcsize(fmt)
    with open(bin_path, "rb") as fh:
        while True:
            blob = fh.read(size)
            if len(blob) < size:
                return
            yield dict(zip(fields, struct.unpack(fmt, blob)))


def selftest(out_dir):
    """Prove the format round-trips WITHOUT a robot or DDS. Run this before
    the robot is powered; a recorder discovered to be broken at the end of
    day 1 has cost a day that cannot be bought back."""
    class FakeMotor:
        q = 0.5; dq = -0.25; ddq = 1.5; tau_est = 3.0
    class FakeIMU:
        quaternion = [1.0, 0.0, 0.0, 0.0]; gyroscope = [0.1, 0.2, 0.3]
        accelerometer = [0.0, 0.0, 9.81]; rpy = [0.01, 0.02, 0.03]
    class FakeLowState:
        tick = 12345; mode_pr = 1; mode_machine = 2
        imu_state = FakeIMU(); motor_state = [FakeMotor() for _ in range(29)]

    d = out_dir / "selftest"
    d.mkdir(parents=True, exist_ok=True)
    s = Stream(d, "rt/lowstate", "lowstate")
    for _ in range(100):
        s.on_msg(FakeLowState())
    s.close()
    rows = list(read_stream(s.path, s.meta_path))
    assert len(rows) == 100, f"wrote 100, read {len(rows)}"
    r = rows[0]
    assert r["tick"] == 12345, r["tick"]
    assert abs(r["m0_q"] - 0.5) < 1e-6, r["m0_q"]
    assert abs(r["m28_tau_est"] - 3.0) < 1e-6, r["m28_tau_est"]
    assert abs(r["imu_accel_2"] - 9.81) < 1e-4, r["imu_accel_2"]
    assert r["t_mono"] > 0 and r["t_wall"] > 1.7e9
    print(f"SELFTEST OK -- 100 records round-tripped, "
          f"{struct.calcsize(s.packer.fmt)} bytes each, "
          f"{s.path.stat().st_size} bytes on disk")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="run", help="goes in the directory name")
    ap.add_argument("--seconds", type=float, default=0.0, help="0 = until Ctrl-C")
    ap.add_argument("--iface", default="", help="DDS interface; empty = default")
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--out-dir", default=str(ART))
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--allow-any-net", action="store_true",
                    help="record even with no robot subnet (replay/bench only)")
    a = ap.parse_args()

    out_root = pathlib.Path(a.out_dir)
    if a.selftest:
        return selftest(out_root)

    ifaces = host_interfaces()
    on_robot_net = robot_net_iface(ifaces, a.iface)
    if not on_robot_net and not a.allow_any_net:
        print(no_robot_net_message(ifaces), file=sys.stderr)
        return 2
    if on_robot_net:
        print(f"robot network: {on_robot_net} -- recording from the link the "
              f"robot is actually on")

    stamp = time.strftime("%Y%m%d-%H%M%S")
    out_dir = out_root / f"{stamp}-{a.label}"
    out_dir.mkdir(parents=True, exist_ok=True)

    from unitree_sdk2py.core.channel import (ChannelSubscriber,
                                             ChannelFactoryInitialize)
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (LowState_, LowCmd_,
                                                        IMUState_)
    # SportModeState_ lives in unitree_go, NOT unitree_hg -- the capture spec
    # says all four message types are in unitree_hg, which is true only of the
    # four it happened to list.
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
    TYPES = {"rt/lowstate": LowState_, "rt/lowcmd": LowCmd_,
             "rt/odommodestate": SportModeState_,
             "rt/secondary_imu": IMUState_, "rt/lf/lowstate": LowState_}

    if a.iface:
        ChannelFactoryInitialize(a.domain, a.iface)
    else:
        ChannelFactoryInitialize(a.domain)

    t_mono0, t_wall0 = time.monotonic(), time.time()
    (out_dir / "session.json").write_text(json.dumps({
        "stamp": stamp, "label": a.label,
        "wall_at_monotonic_zero": t_wall0 - t_mono0,
        "t_mono_start": t_mono0, "t_wall_start": t_wall0,
        "iface": a.iface or "(default)", "domain": a.domain,
        "argv": sys.argv, "python": sys.executable,
        "note": "add wall_at_monotonic_zero to any t_mono to get wall time",
    }, indent=2))

    streams, subs = [], []
    for topic, kind in STREAMS:
        st = Stream(out_dir, topic, kind)
        sub = ChannelSubscriber(topic, TYPES[topic])
        sub.Init(st.on_msg, 50)
        streams.append(st); subs.append(sub)

    stop = {"now": False}
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("now", True))
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("now", True))

    print(f"[rec] -> {out_dir}", flush=True)
    print(f"[rec] recording {len(streams)} streams; Ctrl-C to stop", flush=True)
    t_end = time.monotonic() + a.seconds if a.seconds > 0 else None
    last = 0.0
    while not stop["now"]:
        time.sleep(0.25)
        now = time.monotonic()
        if t_end and now >= t_end:
            break
        if now - last >= 5.0:
            last = now
            parts = [f"{s.topic}={s.n}@{s.rate():.0f}Hz" for s in streams]
            print("[rec] " + "  ".join(parts), flush=True)

    for s in streams:
        s.close()
    summary = build_summary(streams)
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2))
    print("\n[rec] stopped.")
    for t, v in summary.items():
        print(f"  {t:>28}  {v['records']:>9} rec  {v['hz']:>7.1f} Hz  "
              f"{v['bytes']/1e6:>8.1f} MB")
    silent = [t for t, v in summary.items() if v["records"] == 0]
    if silent:
        print(f"\n  WARNING: NO DATA on {silent}. A stream that never "
              f"published is not proof the robot is quiet -- check the topic "
              f"name, the interface, and that the robot is actually running.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
