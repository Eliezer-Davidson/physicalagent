"""Measure a turn by 360-degree SCAN CORRELATION — works in cluttered rooms.

Wall-fitting failed here: an open-plan room has no flat wall in most sectors. This
instead takes the whole 360-degree range array before and after, and finds the
index shift that best aligns them. For a turn-in-place that shift IS the rotation.
No flat walls, no IMU yaw (which drifts 1.4 deg/s), no accumulation.
"""
import sys, time, threading, statistics, math
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import roslibpy
from physicalagent import config
from physicalagent.safety import (ROSPUG_SAFETY, SafetyGuard, Velocity, Pose,
                                  check_battery_ok_for_motion, battery_state)

HOST = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("ROSPUG_HOST", "rospug.local")
ros = roslibpy.Ros(host=HOST, port=9090); ros.run(); time.sleep(2)
st = {}
roslibpy.Topic(ros, config.ROSPUG.topic_battery, "std_msgs/UInt16").subscribe(
    lambda m: st.__setitem__("batt", m["data"]))
roslibpy.Topic(ros, "/physicalagent/deadman_status", "std_msgs/String").subscribe(
    lambda m: st.__setitem__("dead", m["data"]))
roslibpy.Topic(ros, config.ROSPUG.topic_scan, "sensor_msgs/LaserScan",
               queue_length=1).subscribe(lambda m: st.__setitem__("scan", m))
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

def grab(n=9):
    """Median-stack n scans into one clean range array."""
    acc=[]
    for _ in range(n):
        m=st.get("scan")
        if m: acc.append(list(m["ranges"]))
        time.sleep(0.12)
    if not acc: return None, None
    L=min(len(a) for a in acc)
    out=[]
    for i in range(L):
        v=[a[i] for a in acc if isinstance(a[i],(int,float)) and 0.05<a[i]<12 and a[i]==a[i]]
        out.append(statistics.median(v) if len(v)>=max(2,len(acc)//2) else None)
    return out, st.get("scan",{}).get("angle_increment",0.0)

def rotation_between(a, b, inc, max_deg=140):
    """Index shift minimising mean abs difference -> rotation in degrees."""
    n=min(len(a),len(b))
    step_deg=math.degrees(inc)
    maxs=int(max_deg/step_deg)
    best=(None,1e18)
    scores={}
    for s in range(-maxs, maxs+1):
        tot=0.0; cnt=0
        for i in range(n):
            x=a[i]; y=b[(i+s)%n]
            if x is None or y is None: continue
            tot+=abs(x-y); cnt+=1
        if cnt < n*0.25: continue
        sc=tot/cnt
        scores[s]=sc
        if sc<best[1]: best=(s,sc)
    if best[0] is None: return None,None,None
    # confidence: how much better is the best than the median candidate
    vals=sorted(scores.values())
    med=vals[len(vals)//2]
    return best[0]*step_deg, best[1], (med-best[1])/med if med else 0

def send(x=0.0,y=0.0,yaw=0.0,stop=False):
    v=guard.guard_velocity(Velocity(x=x,y=y,yaw_rate=yaw,stop=stop))
    vel_pub.publish(roslibpy.Message({"x":float(v.x),"y":float(v.y),
        "yaw_rate":float(v.yaw_rate),"stop":bool(v.stop)}))
    return v
def halt(): send(0,0,0,stop=True)
def base():
    p=guard.guard_pose(Pose(height=config.DEFAULT_HEIGHT_M, run_time=1.0))
    pose_pub.publish(roslibpy.Message({"roll":0.0,"pitch":0.0,"yaw":0.0,
        "height":float(p.height),"x_shift":float(p.x_shift),"stance_x":float(p.stance_x),
        "stance_y":float(p.stance_y),"run_time":float(p.run_time)}))
    time.sleep(2.0)
def hdr(t): print(f"\n{'='*70}\n{t}\n{'='*70}")

try:
    time.sleep(3)
    hdr(f"PREFLIGHT  (over WiFi — {HOST})")
    mv=st.get("batt"); print(f"   battery {mv} mV ({mv/1000:.2f} V) {battery_state(mv, ROSPUG_SAFETY)}")
    check_battery_ok_for_motion(mv)
    print(f"   deadman {st.get('dead')}   yaw cap {guard._vel_limits.max_abs_yaw} rad/s")
    base()
    print("\n   noise floor: two stationary scans, correlate (expect ~0 deg)")
    n1,inc=grab(); time.sleep(1.5); n2,_=grab()
    r,err,conf=rotation_between(n1,n2,inc)
    print(f"   stationary correlation: {r:+.2f} deg  (err {err:.3f} m, conf {conf*100:.0f}%)")

    CMD_YAW, DUR = 0.35, 3.0
    pred = math.degrees(CMD_YAW*DUR)
    hdr(f"TURN LEFT  yaw=+{CMD_YAW} rad/s for {DUR}s   PREDICT +{pred:.0f} deg")
    before,inc=grab()
    v=send(yaw=CMD_YAW); print(f"   commanded yaw_rate={v.yaw_rate} rad/s")
    time.sleep(DUR); halt(); time.sleep(3.0)
    after,_=grab()
    r,err,conf=rotation_between(before,after,inc)
    print(f"\n   >>> MEASURED TURN: {r:+.1f} deg   (predicted +{pred:.0f} deg)")
    print(f"       match {abs(r)/pred*100:.0f}%   fit err {err:.3f} m   confidence {conf*100:.0f}%")
    print(f"       battery {st.get('batt')} mV   deadman {st.get('dead')}")

    hdr(f"TURN RIGHT yaw=-{CMD_YAW} rad/s for {DUR}s   PREDICT -{pred:.0f} deg")
    before2,_=grab()
    send(yaw=-CMD_YAW); time.sleep(DUR); halt(); time.sleep(3.0)
    after2,_=grab()
    r2,err2,conf2=rotation_between(before2,after2,inc)
    print(f"\n   >>> MEASURED TURN: {r2:+.1f} deg   (predicted -{pred:.0f} deg)")
    print(f"       match {abs(r2)/pred*100:.0f}%   fit err {err2:.3f} m   confidence {conf2*100:.0f}%")
    print(f"       battery {st.get('batt')} mV   deadman {st.get('dead')}")
    if r and r2:
        print(f"\n   symmetry: left {r:+.1f} vs right {r2:+.1f} -> "
              f"net {r+r2:+.1f} deg (0 = perfectly reversible)")
finally:
    hdr("SHUTDOWN")
    try: halt(); time.sleep(1); halt(); base()
    except Exception as e: print("halt err:",e)
    print(f"   final battery {st.get('batt')} mV")
    running.clear(); time.sleep(0.5); ros.terminate()
    print("   robot halted, base position.")
