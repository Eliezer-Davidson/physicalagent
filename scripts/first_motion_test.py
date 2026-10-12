#!/usr/bin/env python3
"""STAGE 1 first-motion test — wired, no translation.

Exercises the full motor path (all 12 servos via IK) WITHOUT commanding any
velocity, so the robot cannot walk away. Every command goes through the real
SafetyGuard — this tests the safety layer too, it does not bypass it.

Sequence: battery gate -> enable_reception -> go_home -> small pose moves
(with before/after joint evidence) -> e-stop verification -> camera frame.
"""
import sys, time, threading
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import roslibpy
from physicalagent import config
from physicalagent.safety import (
    ROSPUG_SAFETY, SafetyGuard, SafetyError, Pose, check_battery_ok_for_motion, battery_state,
)

HOST = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("ROSPUG_HOST", "rospug.local")
ros = roslibpy.Ros(host=HOST, port=9090)
ros.run()
if not ros.is_connected:
    sys.exit("could not connect to rosbridge")


def hdr(t):
    print(f"\n{'='*68}\n{t}\n{'='*68}")


# --- telemetry -------------------------------------------------------------
state = {}
lock = threading.Lock()


def sub(topic, typ, key):
    t = roslibpy.Topic(ros, topic, typ, throttle_rate=200, queue_length=1)
    t.subscribe(lambda m: state.__setitem__(key, m))
    return t


subs = [
    sub(config.ROSPUG.topic_battery, "std_msgs/UInt16", "batt"),
    sub(config.ROSPUG.topic_joint_states, "sensor_msgs/JointState", "joints"),
    sub(config.ROSPUG.topic_imu, "sensor_msgs/Imu", "imu"),
]
time.sleep(2.5)


def joints():
    j = state.get("joints")
    return [round(p, 4) for p in j["position"]] if j else None


# --- 1. battery gate -------------------------------------------------------
hdr("1. BATTERY GATE (safety layer decides if motion is permitted)")
b = state.get("batt")
if not b:
    sys.exit("no battery telemetry — aborting")
mv = b["data"]
print(f"   {mv} mV = {mv/1000:.2f} V ({mv/3000:.2f} V/cell) state={battery_state(mv, ROSPUG_SAFETY)}")
try:
    check_battery_ok_for_motion(mv)
    print(f"   PASS — above the {config.BATTERY_MOTION_MIN_MV} mV floor")
except SafetyError as e:
    sys.exit(f"   REFUSED: {e}")

guard = SafetyGuard(ROSPUG_SAFETY, motion_unlocked=config.MOTION_UNLOCKED,
                    action_whitelist=config.ROSPUG.action_groups)
print(f"   SafetyGuard armed. MOTION_UNLOCKED={config.MOTION_UNLOCKED} "
      f"(velocity stays locked; this test sends NO velocity)")

# --- 2. enable_reception (open question: does it make joint_states real?) ---
hdr("2. enable_reception -> are /joint_states values real?")
print(f"   before: {joints()}")
rx = roslibpy.Topic(ros, config.ROSPUG.topic_enable_reception, "std_msgs/Bool")
rx.publish(roslibpy.Message({"data": True}))
time.sleep(2.0)
print(f"   after : {joints()}")

# --- 3. go_home ------------------------------------------------------------
hdr("3. go_home — the vendor's own safe init (FIRST MOTION)")
before = joints()
print(f"   joints before: {before}")
print("   calling /pug_control/go_home ...")
roslibpy.Service(ros, config.ROSPUG.srv_go_home, "std_srvs/Empty").call(
    roslibpy.ServiceRequest({}), timeout=15)
time.sleep(3.0)
after = joints()
print(f"   joints after : {after}")
if before and after:
    d = [round(a - b, 4) for a, b in zip(after, before)]
    print(f"   delta        : {d}")
    print(f"   MOVED: {any(abs(x) > 1e-3 for x in d)}")

# --- 4. pose moves through the safety layer --------------------------------
hdr("4. POSE MOVES (all 12 servos via IK; robot stays planted)")
pose_pub = roslibpy.Topic(ros, config.ROSPUG.topic_pose, config.ROSPUG.type_pose)
NEUTRAL = Pose(height=config.DEFAULT_HEIGHT_M, run_time=1.0)


def send(pose, label):
    """Clamp through SafetyGuard, publish, report joint delta."""
    safe = guard.guard_pose(pose)
    print(f"\n   >>> {label}")
    print(f"       requested: roll={pose.roll:+.3f} pitch={pose.pitch:+.3f} height={pose.height:+.3f}")
    print(f"       clamped  : roll={safe.roll:+.3f} pitch={safe.pitch:+.3f} height={safe.height:+.3f} "
          f"run_time={safe.run_time}s")
    b4 = joints()
    pose_pub.publish(roslibpy.Message({
        "roll": float(safe.roll), "pitch": float(safe.pitch), "yaw": 0.0,
        "height": float(safe.height), "x_shift": float(safe.x_shift),
        "stance_x": float(safe.stance_x), "stance_y": float(safe.stance_y),
        "run_time": float(safe.run_time),
    }))
    time.sleep(2.5)
    af = joints()
    if b4 and af:
        d = [round(a - x, 4) for a, x in zip(af, b4)]
        n = sum(1 for x in d if abs(x) > 1e-3)
        print(f"       joints changed: {n}/12   delta={d}")


send(NEUTRAL, "neutral / baseline")
# ROLL ALONE — this also reveals the suspected roll/pitch wire transposition.
send(Pose(roll=0.10, height=config.DEFAULT_HEIGHT_M, run_time=1.0),
     "ROLL +0.10 rad  <-- WATCH: does it roll (lean sideways) or PITCH (nose dip)?")
send(NEUTRAL, "back to neutral")
send(Pose(pitch=0.10, height=config.DEFAULT_HEIGHT_M, run_time=1.0),
     "PITCH +0.10 rad <-- WATCH: nose up/down or sideways lean?")
send(NEUTRAL, "back to neutral")
send(Pose(height=-0.10, run_time=1.0), "HEIGHT -0.10 m (stand taller)")
send(NEUTRAL, "back to neutral height -0.13")

# --- 5. clamp proof --------------------------------------------------------
hdr("5. CLAMP PROOF — request something absurd, watch the layer cut it down")
absurd = Pose(roll=5.0, pitch=-5.0, height=-90.0, x_shift=50.0, run_time=1.0)
safe = guard.guard_pose(absurd)
print(f"   requested: roll=5.0 pitch=-5.0 height=-90.0 x_shift=50.0")
print(f"   clamped  : roll={safe.roll} pitch={safe.pitch} height={safe.height} x_shift={safe.x_shift}")
print("   (NOT sent — demonstrating the clamp only)")

# --- 6. e-stop -------------------------------------------------------------
hdr("6. E-STOP VERIFICATION")
guard.estop()
print(f"   estop engaged. is_estopped={guard.is_estopped}")
for what, fn in [("pose", lambda: guard.guard_pose(NEUTRAL)),
                 ("action 'stand'", lambda: guard.guard_action("stand"))]:
    try:
        fn()
        print(f"   *** FAIL: {what} was NOT blocked ***")
    except SafetyError as e:
        print(f"   BLOCKED {what}: {e}")
# physical stop too
roslibpy.Topic(ros, config.ROSPUG.topic_robot_stop, config.ROSPUG.type_robot_stop).publish(
    roslibpy.Message({"duration": 0.0}))
print("   physical stop published to /ros_robot_controller/robot/stop")
guard.reset()
print(f"   reset. is_estopped={guard.is_estopped}")

# --- 7. return to neutral + camera ----------------------------------------
hdr("7. RETURN TO NEUTRAL")
send(NEUTRAL, "final neutral")
imu = state.get("imu")
if imu:
    o = imu["orientation"]
    print(f"\n   IMU orientation: w={o['w']:.3f} x={o['x']:.3f} y={o['y']:.3f} z={o['z']:.3f}")

for s in subs:
    try: s.unsubscribe()
    except Exception: pass
ros.terminate()
print("\nDONE — robot left at neutral pose.")
