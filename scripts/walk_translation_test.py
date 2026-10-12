"""STAGE 3 — 0.10 m/s (vendor max) with a YAW-INVARIANT odometer.

Why the odometer changed:
    Stage 2 measured the median RANGE of the forward sector against a surface
    ~5.7 m away. That conflates two different things: real forward travel, and
    heading drift (a small yaw swings the beam onto a different bit of surface,
    which at 5.7 m changes the range a lot). The operator reported less travel
    than the lidar claimed -- consistent with yaw inflating the number.

    Fix: fit a LINE to the wall ahead (total least squares on the Cartesian
    points) and measure the PERPENDICULAR distance from the robot to that line.
    Rotating in place does not change perpendicular distance to a wall; only
    translating toward it does. We also log IMU yaw so heading drift is
    measured rather than assumed.
"""
import sys, time, threading, statistics, math
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import roslibpy
from physicalagent import config
from physicalagent.safety import (ROSPUG_SAFETY, SafetyGuard, Velocity, Pose,
                                  check_battery_ok_for_motion, battery_state)

HOST = os.environ.get("ROSPUG_HOST", "rospug.local")   # never a literal site address
ros = roslibpy.Ros(host=HOST, port=9090); ros.run(); time.sleep(2)
st = {}
roslibpy.Topic(ros, config.ROSPUG.topic_battery, "std_msgs/UInt16").subscribe(
    lambda m: st.__setitem__("batt", m["data"]))
roslibpy.Topic(ros, "/physicalagent/deadman_status", "std_msgs/String").subscribe(
    lambda m: st.__setitem__("dead", m["data"]))
roslibpy.Topic(ros, config.ROSPUG.topic_scan, "sensor_msgs/LaserScan",
               throttle_rate=100, queue_length=1).subscribe(
    lambda m: st.__setitem__("scan", m))
roslibpy.Topic(ros, config.ROSPUG.topic_imu, "sensor_msgs/Imu",
               throttle_rate=100, queue_length=1).subscribe(
    lambda m: st.__setitem__("imu", m))

hb = roslibpy.Topic(ros, "/physicalagent/heartbeat", "std_msgs/Bool")
running = threading.Event(); running.set()
def beat():
    while running.is_set():
        hb.publish(roslibpy.Message({"data": True})); time.sleep(0.2)
threading.Thread(target=beat, daemon=True).start()

vel_pub  = roslibpy.Topic(ros, config.ROSPUG.topic_velocity, config.ROSPUG.type_velocity)
pose_pub = roslibpy.Topic(ros, config.ROSPUG.topic_pose, config.ROSPUG.type_pose)
guard = SafetyGuard(ROSPUG_SAFETY, motion_unlocked=config.MOTION_UNLOCKED,
                    action_whitelist=config.ROSPUG.action_groups)

def yaw_deg():
    m = st.get("imu")
    if not m: return None
    o = m["orientation"]; w,x,y,z = o["w"],o["x"],o["y"],o["z"]
    return math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))

def wall_perp_distance(msg, half=0.52):
    """Perpendicular distance to the wall ahead, via total-least-squares line fit.
    Yaw-invariant: turning in place leaves this unchanged."""
    r = msg.get("ranges", []); amin = msg.get("angle_min",0.0); inc = msg.get("angle_increment",0.0)
    if not r or not inc: return None
    pts = []
    for i,d in enumerate(r):
        if not isinstance(d,(int,float)) or d<=0.05 or d>12: continue
        a = amin + i*inc
        while a >  3.14159: a -= 6.28318
        while a < -3.14159: a += 6.28318
        if abs(a) <= half:
            pts.append((d*math.cos(a), d*math.sin(a)))
    if len(pts) < 12: return None
    n = len(pts)
    mx = sum(p[0] for p in pts)/n; my = sum(p[1] for p in pts)/n
    sxx = sum((p[0]-mx)**2 for p in pts); syy = sum((p[1]-my)**2 for p in pts)
    sxy = sum((p[0]-mx)*(p[1]-my) for p in pts)
    # principal direction of the point cloud = wall direction
    theta = 0.5*math.atan2(2*sxy, sxx-syy)
    # normal to the wall
    nx, ny = -math.sin(theta), math.cos(theta)
    # perpendicular distance from origin to the fitted line through the centroid
    return abs(nx*mx + ny*my)

def measure(n=30, label=""):
    perp, rng, yaws = [], [], []
    for _ in range(n):
        m = st.get("scan")
        if m:
            p = wall_perp_distance(m)
            if p: perp.append(p)
            rr = [d for d in m.get("ranges",[]) if isinstance(d,(int,float)) and 0.05<d<12]
            if rr: rng.append(min(rr))
        y = yaw_deg()
        if y is not None: yaws.append(y)
        time.sleep(0.07)
    res = {}
    if perp: res["perp"] = statistics.median(perp); res["perp_sd"] = statistics.pstdev(perp) if len(perp)>1 else 0
    if yaws: res["yaw"] = statistics.median(yaws)
    if label:
        pv = f"{res['perp']:.4f}m +-{res['perp_sd']*100:.1f}cm" if "perp" in res else "n/a"
        print(f"   [{label}] wall_perp={pv}  yaw={res.get('yaw',float('nan')):+.1f}deg  (samples {len(perp)})")
    return res

def send(x=0.0, yaw=0.0, stop=False):
    v = guard.guard_velocity(Velocity(x=x, y=0.0, yaw_rate=yaw, stop=stop))
    vel_pub.publish(roslibpy.Message({"x": float(v.x), "y": float(v.y),
                                      "yaw_rate": float(v.yaw_rate), "stop": bool(v.stop)}))
    return v
def halt(): send(0.0,0.0,stop=True)
def base_position():
    p = guard.guard_pose(Pose(height=config.DEFAULT_HEIGHT_M, run_time=1.0))
    pose_pub.publish(roslibpy.Message({"roll":0.0,"pitch":0.0,"yaw":0.0,
        "height":float(p.height),"x_shift":float(p.x_shift),"stance_x":float(p.stance_x),
        "stance_y":float(p.stance_y),"run_time":float(p.run_time)}))
    time.sleep(2.0)
def hdr(t): print(f"\n{'='*66}\n{t}\n{'='*66}")

try:
    time.sleep(3)
    hdr("PREFLIGHT")
    mv = st.get("batt"); print(f"   battery {mv} mV ({mv/1000:.2f} V) {battery_state(mv, ROSPUG_SAFETY)}")
    check_battery_ok_for_motion(mv)
    print(f"   deadman {st.get('dead')}")
    print(f"   MOTION_UNLOCKED={config.MOTION_UNLOCKED}  speed cap={guard._vel_limits.max_abs_x} m/s")
    base_position()
    print("\n   noise floor (stationary, two back-to-back reads):")
    a = measure(30,"noise A"); time.sleep(1); b = measure(30,"noise B")
    if "perp" in a and "perp" in b:
        print(f"   >> noise floor {abs(a['perp']-b['perp'])*100:.1f} cm")

    for speed, dur in [(0.10, 3.0)]:
        hdr(f"WALK — {speed} m/s for {dur}s  (predict {speed*dur*100:.0f} cm)")
        base_position()
        before = measure(30,"before")
        print("   >>> WALKING NOW — WATCH AND MEASURE WITH YOUR EYES <<<")
        v = send(x=speed); print(f"   commanded x={v.x} m/s  (requested {speed})")
        time.sleep(dur)
        halt(); time.sleep(3.0)
        after = measure(30,"after")
        if "perp" in before and "perp" in after:
            d = (before["perp"]-after["perp"])*100
            print(f"\n   >>> TRAVEL (yaw-invariant): {d:+.1f} cm   [predicted {speed*dur*100:.0f} cm]")
        if "yaw" in before and "yaw" in after:
            dy = after["yaw"]-before["yaw"]
            while dy>180: dy-=360
            while dy<-180: dy+=360
            print(f"   >>> HEADING DRIFT: {dy:+.1f} deg"
                  f"  {'(significant - it is curving)' if abs(dy)>8 else '(tracking straight)'}")
        print(f"   battery {st.get('batt')} mV  deadman {st.get('dead')}")
finally:
    hdr("SHUTDOWN")
    try: halt(); time.sleep(1); halt(); base_position()
    except Exception as e: print("halt err:", e)
    running.clear(); time.sleep(0.5); ros.terminate()
    print("   robot halted, base position.")
