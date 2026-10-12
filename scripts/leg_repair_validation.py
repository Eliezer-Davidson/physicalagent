"""STAGE 1 — base position + static load of the repaired FR leg. NO velocity sent."""
import sys, time, threading, math
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import roslibpy
from physicalagent import config
from physicalagent.safety import (ROSPUG_SAFETY, SafetyGuard, SafetyError, Pose,
                                  check_battery_ok_for_motion, battery_state)

HOST = os.environ.get("ROSPUG_HOST", "rospug.local")   # never a literal site address
ros = roslibpy.Ros(host=HOST, port=9090); ros.run(); time.sleep(2)
st = {}
roslibpy.Topic(ros, config.ROSPUG.topic_battery, "std_msgs/UInt16").subscribe(
    lambda m: st.__setitem__("batt", m["data"]))
roslibpy.Topic(ros, "/physicalagent/deadman_status", "std_msgs/String").subscribe(
    lambda m: st.__setitem__("dead", m["data"]))
roslibpy.Topic(ros, config.ROSPUG.topic_imu, "sensor_msgs/Imu",
               throttle_rate=100, queue_length=1).subscribe(
    lambda m: st.__setitem__("imu", m))

hb = roslibpy.Topic(ros, "/physicalagent/heartbeat", "std_msgs/Bool")
run = threading.Event(); run.set()
def beat():
    while run.is_set():
        hb.publish(roslibpy.Message({"data": True})); time.sleep(0.2)
threading.Thread(target=beat, daemon=True).start()
time.sleep(3)

def rpy():
    m = st.get("imu")
    if not m: return None
    o = m["orientation"]; w,x,y,z = o["w"],o["x"],o["y"],o["z"]
    r = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    p = math.asin(max(-1,min(1,2*(w*y-z*x))))
    return (math.degrees(r), math.degrees(p))

def hdr(t): print(f"\n{'='*66}\n{t}\n{'='*66}")

hdr("PREFLIGHT")
mv = st.get("batt"); print(f"   battery: {mv} mV ({mv/1000:.2f} V) {battery_state(mv, ROSPUG_SAFETY)}")
check_battery_ok_for_motion(mv); print("   battery gate: PASS")
print(f"   deadman: {st.get('dead')}  (heartbeat now beating -> expect ALIVE)")
r0 = rpy(); print(f"   IMU now: roll={r0[0]:+.2f}deg pitch={r0[1]:+.2f}deg")

guard = SafetyGuard(ROSPUG_SAFETY, motion_unlocked=config.MOTION_UNLOCKED,
                    action_whitelist=config.ROSPUG.action_groups)

hdr("BASE POSITION — go_home")
roslibpy.Service(ros, config.ROSPUG.srv_go_home, "std_srvs/Empty").call(
    roslibpy.ServiceRequest({}), timeout=20)
time.sleep(3)
rb = rpy(); print(f"   IMU after go_home: roll={rb[0]:+.2f}deg pitch={rb[1]:+.2f}deg")

# leg positions (forward kinematics — blind to rod length, but a consistency check)
try:
    lp = roslibpy.Service(ros, config.ROSPUG.srv_leg_position,
                          "ros_robot_controller/GetLegPosition").call(
        roslibpy.ServiceRequest({}), timeout=10)
    print(f"   leg positions: {lp}")
except Exception as e:
    print(f"   leg position svc: {e}")

pose_pub = roslibpy.Topic(ros, config.ROSPUG.topic_pose, config.ROSPUG.type_pose)
NEUTRAL = Pose(height=config.DEFAULT_HEIGHT_M, run_time=1.0)

def send(pose, label, expect):
    safe = guard.guard_pose(pose)
    before = rpy()
    pose_pub.publish(roslibpy.Message({
        "roll": float(safe.roll), "pitch": float(safe.pitch), "yaw": 0.0,
        "height": float(safe.height), "x_shift": float(safe.x_shift),
        "stance_x": float(safe.stance_x), "stance_y": float(safe.stance_y),
        "run_time": float(safe.run_time)}))
    time.sleep(2.5)
    after = rpy()
    d = (after[0]-before[0], after[1]-before[1])
    print(f"\n   {label}")
    print(f"       cmd roll={safe.roll:+.3f} pitch={safe.pitch:+.3f} height={safe.height:+.3f}")
    print(f"       IMU  roll {before[0]:+.2f} -> {after[0]:+.2f}  (d={d[0]:+.2f}deg)")
    print(f"       IMU  pitch{before[1]:+.2f} -> {after[1]:+.2f}  (d={d[1]:+.2f}deg)")
    print(f"       expect: {expect}")

hdr("STATIC LOAD — loads all 12 servos, robot stays planted")
send(NEUTRAL, "neutral baseline", "little change")
send(Pose(roll=0.12, height=config.DEFAULT_HEIGHT_M, run_time=1.2),
     "ROLL +0.12 rad (~6.9deg) — loads L/R asymmetrically", "roll delta approx +7deg")
send(NEUTRAL, "back to neutral", "returns to baseline")
send(Pose(roll=-0.12, height=config.DEFAULT_HEIGHT_M, run_time=1.2),
     "ROLL -0.12 rad — loads the REPAIRED (front-right) side", "roll delta approx -7deg")
send(NEUTRAL, "back to neutral", "returns to baseline")
send(Pose(pitch=0.12, height=config.DEFAULT_HEIGHT_M, run_time=1.2),
     "PITCH +0.12 rad — shifts weight fore/aft onto front legs", "pitch delta approx +7deg")
send(NEUTRAL, "back to neutral", "returns to baseline")
send(Pose(height=-0.09, run_time=1.5), "HEIGHT -0.09 (stand tall) — max extension on all legs", "body rises")
send(NEUTRAL, "back to neutral height -0.13", "body settles")

hdr("RESULT")
rf = rpy()
print(f"   IMU start   : roll={r0[0]:+.2f} pitch={r0[1]:+.2f}")
print(f"   IMU end     : roll={rf[0]:+.2f} pitch={rf[1]:+.2f}")
print(f"   drift       : roll={rf[0]-r0[0]:+.2f}deg pitch={rf[1]-r0[1]:+.2f}deg")
print(f"   deadman     : {st.get('dead')}")
print(f"   battery     : {st.get('batt')} mV")
print("\n   NOTE: on carpet, absolute tilt is not trustworthy. What matters is that")
print("   each commanded pose produced a MATCHING IMU response and it RETURNED to")
print("   baseline each time — that is what a correctly-linked leg looks like.")
run.clear(); time.sleep(0.5); ros.terminate()
