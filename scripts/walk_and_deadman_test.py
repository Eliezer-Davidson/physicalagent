#!/usr/bin/env python3
"""STAGE 2 — first WALKING test + live deadman proof. Wired, slow, short bursts.

Speed stays at the LOCKED cap (config.UNVERIFIED_MAX_ABS_X = 0.05 m/s), so a
1.5 s burst travels ~7.5 cm. Uses the lidar's forward sector as an objective
odometer so travel is measured, not eyeballed.

The point of the run: prove the robot-side deadman PHYSICALLY halts a walking
robot when the heartbeat stops — not just that it publishes stop messages.
"""
import sys, time, threading, statistics
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import roslibpy
from physicalagent import config
from physicalagent.safety import (
    ROSPUG_SAFETY, SafetyGuard, SafetyError, Velocity, check_battery_ok_for_motion, battery_state,
)

HOST = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("ROSPUG_HOST", "rospug.local")
ros = roslibpy.Ros(host=HOST, port=9090)
ros.run(); time.sleep(1)

state, lock = {}, threading.Lock()
roslibpy.Topic(ros, config.ROSPUG.topic_battery, "std_msgs/UInt16").subscribe(
    lambda m: state.__setitem__("batt", m["data"]))
roslibpy.Topic(ros, "/physicalagent/deadman_status", "std_msgs/String").subscribe(
    lambda m: state.__setitem__("dead", m["data"]))
roslibpy.Topic(ros, config.ROSPUG.topic_scan, "sensor_msgs/LaserScan",
               throttle_rate=100, queue_length=1).subscribe(
    lambda m: state.__setitem__("scan", m))

vel_pub = roslibpy.Topic(ros, config.ROSPUG.topic_velocity, config.ROSPUG.type_velocity)
hb_pub = roslibpy.Topic(ros, "/physicalagent/heartbeat", "std_msgs/Bool")
guard = SafetyGuard(ROSPUG_SAFETY, motion_unlocked=config.MOTION_UNLOCKED)

# --- heartbeat thread (this is what the deadman listens for) ---------------
beating = threading.Event(); beating.set()
running = threading.Event(); running.set()


def heartbeat_loop():
    while running.is_set():
        if beating.is_set():
            hb_pub.publish(roslibpy.Message({"data": True}))
        time.sleep(0.2)


threading.Thread(target=heartbeat_loop, daemon=True).start()


def fwd_distance():
    """Median range of the forward +/-10 deg sector, in metres. Our odometer."""
    s = state.get("scan")
    if not s:
        return None
    r = s.get("ranges", []); amin = s.get("angle_min", 0.0); inc = s.get("angle_increment", 0.0)
    if not r or not inc:
        return None
    vals = []
    for i, d in enumerate(r):
        if not isinstance(d, (int, float)) or d <= 0.02 or d > 12:
            continue
        ang = amin + i * inc
        while ang > 3.14159: ang -= 6.28318
        while ang < -3.14159: ang += 6.28318
        if abs(ang) <= 0.1745:          # +/- 10 degrees
            vals.append(d)
    return round(statistics.median(vals), 4) if vals else None


def send_vel(x=0.0, yaw=0.0, stop=False):
    v = guard.guard_velocity(Velocity(x=x, y=0.0, yaw_rate=yaw, stop=stop))
    vel_pub.publish(roslibpy.Message({
        "x": float(v.x), "y": float(v.y), "yaw_rate": float(v.yaw_rate), "stop": bool(v.stop)}))
    return v


def halt():
    """Clean stop: all-zero + stop=True routes to stop_pub in pug_node."""
    send_vel(0.0, 0.0, stop=True)


def hdr(t): print(f"\n{'='*70}\n{t}\n{'='*70}")


try:
    # --- preflight ---------------------------------------------------------
    hdr("PREFLIGHT")
    time.sleep(2)
    mv = state.get("batt")
    print(f"   battery : {mv} mV ({mv/1000:.2f} V) state={battery_state(mv, ROSPUG_SAFETY)}")
    check_battery_ok_for_motion(mv)
    print(f"   deadman : {state.get('dead')}")
    print(f"   speed cap: {guard._vel_limits.max_abs_x} m/s (MOTION_UNLOCKED={config.MOTION_UNLOCKED})")
    d0 = fwd_distance()
    print(f"   forward distance (lidar): {d0} m")
    if d0 is None:
        print("   !! no lidar reading — cannot measure travel; aborting")
        raise SystemExit(1)

    print("\n   >>> starting in 4s — WATCH THE ROBOT <<<")
    time.sleep(4)

    # --- burst 1: does it walk at all? ------------------------------------
    hdr("BURST 1 — forward 1.5s at 0.05 m/s (expect ~7.5 cm)")
    before = fwd_distance()
    v = send_vel(x=0.05)
    print(f"   commanded x={v.x} m/s ...")
    time.sleep(1.5)
    halt()
    time.sleep(2.5)
    after = fwd_distance()
    print(f"   lidar forward: {before} m -> {after} m")
    if before and after:
        print(f"   TRAVELLED: {round((before-after)*100, 1)} cm")

    time.sleep(2)

    # --- burst 2: THE DEADMAN TEST ----------------------------------------
    hdr("BURST 2 — walk, then KILL THE HEARTBEAT mid-stride")
    before = fwd_distance()
    print(f"   forward distance before: {before} m")
    send_vel(x=0.05)
    print("   walking ... (heartbeat alive)")
    time.sleep(1.0)

    d_at_cut = fwd_distance()
    print(f"\n   *** CUTTING HEARTBEAT NOW *** (simulating Wi-Fi loss)")
    print(f"   distance at cut: {d_at_cut} m")
    beating.clear()                      # stop sending heartbeats
    t_cut = time.monotonic()

    t_paused = None
    while time.monotonic() - t_cut < 6.0:
        if state.get("dead") == "PAUSED" and t_paused is None:
            t_paused = time.monotonic() - t_cut
            print(f"   deadman reported PAUSED after {t_paused:.2f}s")
        time.sleep(0.1)

    settled = fwd_distance()
    print(f"\n   distance after deadman stop: {settled} m")
    if d_at_cut and settled:
        print(f"   >>> TRAVEL AFTER HEARTBEAT LOSS: {round((d_at_cut-settled)*100,1)} cm")
    print(f"   deadman state now: {state.get('dead')}")

    # --- resume ------------------------------------------------------------
    hdr("RESUME — restore heartbeat, expect auto-resume with NO restart")
    beating.set()
    time.sleep(3)
    print(f"   deadman state: {state.get('dead')}  <-- expect ALIVE")
    print("   (note: robot does NOT resume walking by itself — by design)")
    final = fwd_distance()
    print(f"   final forward distance: {final} m (unchanged => it stayed put)")

finally:
    hdr("SHUTDOWN — halting")
    try:
        halt(); time.sleep(1); halt()
    except Exception as e:
        print("halt error:", e)
    running.clear()
    time.sleep(0.5)
    ros.terminate()
    print("done — robot stopped.")
