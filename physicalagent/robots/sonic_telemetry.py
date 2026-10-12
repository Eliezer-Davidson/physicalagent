"""Real robot telemetry, read off the Unitree SDK2 DDS wire.

WHY THIS EXISTS
    Before this, `get_status()` could only report the command we had just sent.
    It said so honestly -- "height and tilt are not measured on this link" -- but
    a model that cannot see the robot cannot react to it, and the whole point of
    a reasoning layer is reacting. This subscribes to what the robot actually
    publishes.

WHAT IT READS, AND FROM WHERE
    rt/lowstate      LowState_   29 x (q, dq, ddq, tau_est), IMU quaternion,
                                 gyroscope, accelerometer, tick
    rt/odostate      OdoState_   world position[3], orientation[4] (w,x,y,z),
                                 linear_velocity[3], angular_velocity[3]
    rt/secondary_imu IMUState_   torso IMU, independent of the pelvis one

    All three are published by unitree_sdk2py_bridge.PublishLowState() at the
    simulator's rate. On the physical G1 the same three topics carry the same
    payloads from the real onboard sensors -- which is the reason to read the
    robot this way rather than by asking the simulator: this file does not know
    it is in a simulation, and does not have to change when it isn't.

    CORRECTION worth recording: an earlier note in this project asserted that
    "nothing on DDS carries world position, rt/lowstate has IMU only". The IMU
    half is right, the conclusion is wrong -- rt/odostate carries the full
    floating-base pose and is created unconditionally for g1 and h1-2
    (unitree_sdk2py_bridge.py:71-77). In simulation those values are ground
    truth from mj_data; on hardware they are the robot's own state estimator, so
    the number is honest either way but its provenance is not the same. The
    field is labelled `odometry` and never `ground truth` for that reason.

FRESHNESS IS A FIRST-CLASS FIELD
    Every reading carries the age of the sample it came from. A stale reading
    presented as current is how a fall detector ends up reporting a robot that
    is standing while it lies on the floor. `age_s` is always reported and
    `fresh` is False past STALE_AFTER_S, whatever the numbers look like.
"""
from __future__ import annotations

import math
import threading
import time

# A sample older than this is not describing the robot now. The simulator
# publishes every control step (~200 Hz), so anything approaching a tenth of a
# second means something upstream has stopped.
STALE_AFTER_S = 0.5

# rt/lowcmd arrives at 500 Hz while the policy is controlling and stops dead
# when it is not. Half a second is ~250 missed frames -- far beyond any
# scheduling gap this box produces, and far below the time it takes a falling
# robot to matter.
CONTROLLER_DEAD_AFTER_S = 0.5

# 29 body joints, in the order they appear in LowState_.motor_state. Same order
# as JOINT_KP / DEFAULT_DOF_ANGLES in the wbc yaml.
JOINT_NAMES = (
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee",
    "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee",
    "right_ankle_pitch", "right_ankle_roll",
    "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
    "left_elbow", "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
    "right_elbow", "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
)
NUM_BODY_MOTORS = len(JOINT_NAMES)

# Joints a model asks about by limb rather than by name.
LIMBS = {
    "left_leg": range(0, 6),
    "right_leg": range(6, 12),
    "waist": range(12, 15),
    "left_arm": range(15, 22),
    "right_arm": range(22, 29),
}


# The odometry topic this robot actually publishes. rt/odostate, which this
# module subscribed to until 2026-08-19, produced zero messages in every
# measured window on the physical G1 -- so pelvis height was always absent and
# sonic_fall could never confirm a fall. The topic name was the whole bug.
ODOM_TOPIC = "rt/odommodestate"


def _first(msg, *names):
    """First attribute present and not None. SportModeState_ and OdoState_ carry
    the same information under different names, and the simulator still sends
    the second one, so one reader has to handle both shapes rather than the
    hardware fix silently breaking every sim comparison."""
    for n in names:
        v = getattr(msg, n, None)
        if v is not None:
            return v
    return None


def pose_from_message(msg, age_s):
    """Pose dict from either message shape. Pure, so it is testable without DDS
    -- the previous version could only be exercised against a live publisher,
    which is why nobody noticed it was reading a dead topic."""
    if msg is None:
        return {"available": False,
                "note": f"no {ODOM_TOPIC} received \u2014 is the robot powered and "
                        f"on this interface?"}
    imu = getattr(msg, "imu_state", None)
    p = _first(msg, "position") or [0.0, 0.0, 0.0]
    q = _first(msg, "orientation") or getattr(imu, "quaternion", None) \
        or [1.0, 0.0, 0.0, 0.0]
    v = _first(msg, "linear_velocity", "velocity") or [0.0, 0.0, 0.0]

    w = _first(msg, "angular_velocity")
    if w is None:
        gyro = getattr(imu, "gyroscope", None) or [0.0, 0.0, 0.0]
        yaw_speed = _first(msg, "yaw_speed")
        w = [float(gyro[0]), float(gyro[1]),
             float(yaw_speed) if yaw_speed is not None else float(gyro[2])]

    shape = "OdoState_" if getattr(msg, "orientation", None) is not None \
        else "SportModeState_"
    roll, pitch, yaw = quat_to_rpy(q[0], q[1], q[2], q[3])
    out = {
        "available": True,
        "source": f"{ODOM_TOPIC} ({shape}; the robot\u2019s own estimator, "
                  f"odometry and not ground truth)",
        "position_m": {"x": round(float(p[0]), 3), "y": round(float(p[1]), 3),
                       "z": round(float(p[2]), 3)},
        "height_m": round(float(p[2]), 3),
        "heading_deg": round(math.degrees(yaw), 1),
        "roll_deg": round(math.degrees(roll), 1),
        "pitch_deg": round(math.degrees(pitch), 1),
        "tilt_deg": round(math.degrees(
            tilt_from_quat(q[0], q[1], q[2], q[3])), 1),
        "linear_velocity_m_s": [round(float(c), 3) for c in v[:3]],
        "speed_m_s": round(math.hypot(float(v[0]), float(v[1])), 3),
        "angular_velocity_rad_s": [round(float(c), 3) for c in w[:3]],
    }
    if age_s is not None:
        out["age_s"] = round(age_s, 3)
        out["fresh"] = age_s < STALE_AFTER_S
    return out


# SONIC's own hysteresis (g1_deploy_onnx_ref.cpp:256): it warns when
# max(winding, driver) reaches TEMP_WARN_C and clears below TEMP_CLEAR_C.
# Mirrored rather than invented so a reading is judged against the same bar
# that raises the alarm -- and so the band underneath it is visible, which is
# where a decision can still be made calmly.
TEMP_WARN_C = 90

# Guard thresholds, set from what the hardware actually did on 2026-08-19.
# right_shoulder_pitch went 65 -> 106 C in 11.5 s holding a raised-arm pose:
# 215 C/min on the DRIVER sensor while the winding sensor moved 3 C. A driver
# stage has almost no thermal mass, so it is both the first thing to get hot
# and the only one that can be read in time.
#
# WARN is deliberately low. From 70 C there are 5.6 s to 90 C, so a warning
# that fires at 85 buys nothing a human can use -- it is there for the log.
# ACT is the number that matters, and it is acted on by a machine.
TEMP_GUARD_WARN_C = 80
TEMP_ACT_C = 90


def thermal_verdict(thermal, warn_c=TEMP_GUARD_WARN_C, act_c=TEMP_ACT_C):
    """Decide what to do about the temperatures in a thermal() reading.

    Pure, so the policy can be tested without a robot, and so the same numbers
    drive the readout and the guard rather than drifting apart.

    Returns level: ok | warn | act | unknown. `act` is the one that stops the
    robot, and it is never true when the temperature is unknown -- but unknown
    is never reported as ok either, because a guard that reads silence as
    safety sleeps through exactly the event it exists for.
    """
    rows = (thermal or {}).get("hottest") or []
    if not (thermal or {}).get("available") or not rows:
        return {"level": "unknown", "act": False, "hottest": None,
                "message": "no temperature is being reported -- the guard "
                           "cannot see the motors and is NOT protecting them"}
    top = max(rows, key=lambda r: r.get("max_c", 0))
    c = top.get("max_c", 0)
    joint = top.get("joint", f"motor_{top.get('index', '?')}")
    if c >= act_c:
        return {"level": "act", "act": True, "hottest": top,
                "message": f"{joint} is at {c} C, at or past the {act_c} C "
                           f"limit -- stopping and unloading the arms. It "
                           f"cools fast once the load comes off."}
    if c >= warn_c:
        return {"level": "warn", "act": False, "hottest": top,
                "message": f"{joint} is at {c} C, climbing toward the {act_c} C "
                           f"limit. It can cover that in seconds under load."}
    return {"level": "ok", "act": False, "hottest": top,
            "message": f"hottest {joint} {c} C"}
TEMP_CLEAR_C = 85


def thermal_from_message(low_msg, top_n: int = 5):
    """Per-motor temperatures, hottest first.

    Exists because on 2026-08-19 an overheating warning stopped a session and
    nothing on the machine could say whether it was real. A number that is only
    ever printed by the thing raising the alarm cannot be used to check it."""
    if low_msg is None:
        return {"available": False, "status": "UNKNOWN",
                "note": "no rt/lowstate received \u2014 no temperature to read",
                "hottest": [], "over_warn": [], "approaching": []}
    rows = []
    for i, m in enumerate(getattr(low_msg, "motor_state", None) or []):
        temp = getattr(m, "temperature", None) or []
        if not temp:
            continue
        w = int(temp[0])
        d = int(temp[1]) if len(temp) > 1 else w
        rows.append({
            "index": i,
            "joint": JOINT_NAMES[i] if i < len(JOINT_NAMES) else f"motor_{i}",
            "winding_c": w, "driver_c": d, "max_c": max(w, d),
        })
    if not rows:
        return {"available": False, "status": "UNKNOWN",
                "note": "message carried no temperature field",
                "hottest": [], "over_warn": [], "approaching": []}
    rows.sort(key=lambda r: r["max_c"], reverse=True)
    over = [r["joint"] for r in rows if r["max_c"] >= TEMP_WARN_C]
    near = [r["joint"] for r in rows
            if TEMP_CLEAR_C <= r["max_c"] < TEMP_WARN_C]
    return {
        "available": True,
        "status": "OVER" if over else ("WATCH" if near else "OK"),
        "hottest": rows[:top_n],
        "over_warn": over,
        "approaching": near,
        "warn_c": TEMP_WARN_C, "clear_c": TEMP_CLEAR_C,
    }


def command_authority(cmd_msg):
    """Largest |kp|, |kd| and |tau| anywhere in a LowCmd_. Zero across all three
    means the message cannot move a joint however fast it is published."""
    mkp = mkd = mtau = 0.0
    for m in (getattr(cmd_msg, "motor_cmd", None) or []):
        mkp = max(mkp, abs(float(getattr(m, "kp", 0.0) or 0.0)))
        mkd = max(mkd, abs(float(getattr(m, "kd", 0.0) or 0.0)))
        mtau = max(mtau, abs(float(getattr(m, "tau", 0.0) or 0.0)))
    return mkp, mkd, mtau


def controller_from_message(cmd_msg, age_s):
    """Is anything actually holding the robot up?

    Measured 2026-08-19 on the physical G1 with SONIC NOT running: rt/lowcmd
    arrives at 995 Hz, byte-identical, every gain and torque zero. Recency
    alone therefore proves only that a publisher exists. This is the readout an
    operator uses to decide whether bring-up worked, so it asks the question
    that matters -- can these commands move a joint -- rather than the question
    that is easy to answer.
    """
    if cmd_msg is None:
        return {"active": False,
                "note": "no rt/lowcmd has EVER arrived \u2014 nothing is "
                        "publishing commands at all"}
    if age_s is not None and age_s > CONTROLLER_DEAD_AFTER_S:
        return {
            "active": False, "age_s": round(age_s, 2),
            "note": "THE BALANCE CONTROLLER HAS STOPPED. Nothing is holding "
                    "the robot up and no command you send can reach it. This "
                    "is not recoverable from here \u2014 the session must be "
                    "restarted. Report it; do not keep issuing commands.",
        }
    mkp, mkd, mtau = command_authority(cmd_msg)
    out = {"age_s": round(age_s, 3) if age_s is not None else None,
           "max_kp": round(mkp, 4), "max_kd": round(mkd, 4),
           "max_tau": round(mtau, 4)}
    if not (mkp or mkd or mtau):
        out["active"] = False
        out["note"] = ("rt/lowcmd IS arriving, and every kp, kd and "
                       "feedforward torque in it is zero \u2014 an idle "
                       "heartbeat, not a controller. NOTHING is holding the "
                       "robot up. Do not read this as bring-up having worked.")
        return out
    out["active"] = True
    out["source"] = "rt/lowcmd, carrying non-zero gains"
    return out


def quat_to_rpy(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    """(w,x,y,z) -> roll, pitch, yaw in radians. Unitree orders the quaternion
    w-first; scipy and most of the rest of the world order it w-last, and
    swapping them silently produces a plausible-looking wrong attitude."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    # asin domain-guards rather than raising: a slightly non-unit quaternion off
    # the wire must not take down the status call.
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def tilt_from_quat(w: float, x: float, y: float, z: float) -> float:
    """Angle between the robot's own up-axis and world up, in radians.

    This is the number that matters for "is it falling", and it is NOT
    hypot(roll, pitch): those are Euler terms that blow up near gimbal lock and
    that count a pure yaw rotation as tilt under some conventions. The gravity
    projection has neither problem. 0 is upright, pi/2 is horizontal."""
    # world z-axis expressed in body frame == third row of R(q)
    gz = 1.0 - 2.0 * (x * x + y * y)
    return math.acos(max(-1.0, min(1.0, gz)))


class _Sample:
    __slots__ = ("msg", "t")

    def __init__(self, msg, t):
        self.msg = msg
        self.t = t


class UnitreeTelemetry:
    """Subscribes to the robot's own state topics and answers questions about
    them. Read-only: this class can never move the robot.

    Constructing it does NOT initialise the DDS channel factory -- the process
    that owns the connection does that once. Calling ChannelFactoryInitialize a
    second time prints a create-domain error and is otherwise harmless, but
    owning it here would make the ordering matter, and ordering that matters
    without saying so is how the start handshake got lost the first time.
    """

    def __init__(self, *, init_channel: bool = False, domain_id: int = 0,
                 interface: str = "lo"):
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
            IMUState_,
            LowState_,
        )
        # SportModeState_ is in unitree_go, not unitree_hg. Measured 2026-08-19:
        # rt/odommodestate publishes at 497 Hz on the physical G1 and
        # rt/odostate publishes nothing at all.
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

        if init_channel:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            ChannelFactoryInitialize(domain_id, interface)

        self._lock = threading.Lock()
        self._low: _Sample | None = None
        self._odo: _Sample | None = None
        self._torso: _Sample | None = None
        self._cmd: _Sample | None = None

        self._low_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self._low_sub.Init(self._on_low, 1)
        self._odo_sub = ChannelSubscriber(ODOM_TOPIC, SportModeState_)
        self._odo_sub.Init(self._on_odo, 1)
        self._torso_sub = ChannelSubscriber("rt/secondary_imu", IMUState_)
        self._torso_sub.Init(self._on_torso, 1)

        # rt/lowcmd is what the POLICY writes. Subscribing to it is how we find
        # out the balance controller is alive, and it is the only way: SONIC
        # stops its control loop permanently when rt/lowstate goes 500 ms stale
        # (g1_deploy_onnx_ref.cpp:296) and announces it only on its own stdout,
        # which nothing downstream reads. Without this, a dead controller looks
        # exactly like a live one -- the simulator keeps publishing state, the
        # robot keeps falling and being reset, and the model keeps issuing
        # commands into a process that stopped listening minutes ago.
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_

        self._cmd_sub = ChannelSubscriber("rt/lowcmd", LowCmd_)
        self._cmd_sub.Init(self._on_cmd, 1)

    # -- callbacks ---------------------------------------------------------
    def _on_low(self, msg):
        with self._lock:
            self._low = _Sample(msg, time.monotonic())

    def _on_odo(self, msg):
        with self._lock:
            self._odo = _Sample(msg, time.monotonic())

    def _on_torso(self, msg):
        with self._lock:
            self._torso = _Sample(msg, time.monotonic())

    def _on_cmd(self, msg):
        with self._lock:
            self._cmd = _Sample(msg, time.monotonic())

    def controller(self) -> dict:
        """Is anything actually balancing the robot? See controller_from_message."""
        with self._lock:
            cmd, now = self._cmd, time.monotonic()
        # cmd.msg, NOT cmd. Until 2026-08-19 this passed the _Sample
        # wrapper, whose missing .motor_cmd hit command_authority's
        # getattr default and read as zero gains -- so this said "NOT IN
        # CONTROL" while the robot was walking. thermal() and pose()
        # always unwrapped correctly; only this one did not.
        return controller_from_message(
            None if cmd is None else cmd.msg,
            None if cmd is None else now - cmd.t)

    def thermal(self) -> dict:
        """Motor temperatures, hottest first. See thermal_from_message."""
        with self._lock:
            low = self._low
        return thermal_from_message(None if low is None else low.msg)

    def lowstate(self) -> dict:
        """Is the robot actually publishing its state?

        If rt/lowstate goes stale (> 500 ms), the SONIC policy stops controlling
        permanently. This is a live hazard on a real network.
        """
        with self._lock:
            low, now = self._low, time.monotonic()
        if low is None:
            return {"active": False, "note": "no rt/lowstate has EVER arrived — "
                                             "the robot is not publishing state"}
        age = now - low.t
        if age > STALE_AFTER_S:
            return {
                "active": False, "age_s": round(age, 2),
                "note": "THE ROBOT'S TELEMETRY IS STALE. rt/lowstate has stopped "
                        "arriving. The balance controller will stop permanently "
                        "and the robot will fall. Check the network or the robot.",
            }
        return {"active": True, "age_s": round(age, 3),
                "source": "rt/lowstate from the robot"}

    def wait_for_data(self, timeout_s: float = 5.0) -> bool:
        """Block until the first lowstate arrives. Returns False on timeout
        rather than raising, so a caller can carry on with no telemetry and say
        so, which is better than refusing to start."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                if self._low is not None:
                    return True
            time.sleep(0.05)
        return False

    def close(self) -> None:
        for sub in (self._low_sub, self._odo_sub, self._torso_sub, self._cmd_sub):
            try:
                sub.Close()
            except Exception:
                pass

    # -- reads -------------------------------------------------------------
    def _snapshot(self):
        with self._lock:
            return self._low, self._odo, self._torso, time.monotonic()

    def pose(self) -> dict:
        """Where the robot is and how it is oriented, from ODOM_TOPIC."""
        _, odo, _, now = self._snapshot()
        if odo is None:
            return pose_from_message(None, None)
        return pose_from_message(odo.msg, now - odo.t)

    def imu(self) -> dict:
        """Pelvis IMU off rt/lowstate, plus the torso IMU if it is arriving."""
        low, _, torso, now = self._snapshot()
        if low is None:
            return {"available": False, "note": "no rt/lowstate received"}
        age = now - low.t
        q = low.msg.imu_state.quaternion
        roll, pitch, yaw = quat_to_rpy(q[0], q[1], q[2], q[3])
        out = {
            "available": True,
            "fresh": age < STALE_AFTER_S,
            "age_s": round(age, 3),
            "source": "rt/lowstate imu_state",
            "roll_deg": round(math.degrees(roll), 1),
            "pitch_deg": round(math.degrees(pitch), 1),
            "yaw_deg": round(math.degrees(yaw), 1),
            "tilt_deg": round(math.degrees(tilt_from_quat(*q[:4])), 1),
            "gyroscope_rad_s": [round(float(c), 3) for c in low.msg.imu_state.gyroscope],
            "accelerometer_m_s2": [round(float(c), 2)
                                   for c in low.msg.imu_state.accelerometer],
        }
        if torso is not None:
            tq = torso.msg.quaternion
            _, tp, _ = quat_to_rpy(tq[0], tq[1], tq[2], tq[3])
            out["torso_pitch_deg"] = round(math.degrees(tp), 1)
            out["torso_tilt_deg"] = round(math.degrees(tilt_from_quat(*tq[:4])), 1)
        return out

    def joints(self, limb: str | None = None) -> dict:
        """Per-joint angle, speed and estimated torque.

        `limb` narrows it to one of LIMBS. Returning all 29 every time buries the
        answer the model wanted in 28 numbers it did not, and a tool result that
        needs re-reading is one the model will misread."""
        low, _, _, now = self._snapshot()
        if low is None:
            return {"available": False, "note": "no rt/lowstate received"}
        age = now - low.t
        if limb is not None:
            key = str(limb).strip().lower().replace(" ", "_")
            if key not in LIMBS:
                raise ValueError(f"{limb!r} is not a limb; choose from "
                                 + ", ".join(LIMBS))
            indices = list(LIMBS[key])
        else:
            indices = list(range(NUM_BODY_MOTORS))

        joints = {}
        for i in indices:
            ms = low.msg.motor_state[i]
            joints[JOINT_NAMES[i]] = {
                "angle_deg": round(math.degrees(ms.q), 1),
                "speed_rad_s": round(ms.dq, 2),
                "torque_nm": round(ms.tau_est, 1),
            }
        return {"available": True, "fresh": age < STALE_AFTER_S,
                "age_s": round(age, 3), "source": "rt/lowstate motor_state",
                "limb": limb, "joints": joints}

    def effort_summary(self) -> dict:
        """Which joints are working hardest. A compact read that answers "am I
        pushing against something" without shipping 29 rows to the model."""
        low, _, _, now = self._snapshot()
        if low is None:
            return {"available": False, "note": "no rt/lowstate received"}
        loads = sorted(
            ((abs(low.msg.motor_state[i].tau_est), JOINT_NAMES[i])
             for i in range(NUM_BODY_MOTORS)), reverse=True)
        return {
            "available": True,
            "fresh": (now - low.t) < STALE_AFTER_S,
            "highest_torque_joints": [
                {"joint": name, "torque_nm": round(tau, 1)} for tau, name in loads[:5]],
        }
