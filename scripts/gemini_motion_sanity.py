#!/usr/bin/env python3
"""FIRST AI-COMMANDED MOTION — bounded, every safety layer live.

The milestone: Gemini Robotics-ER decides to move and the robot physically
moves, with the model NEVER touching raw velocity. It calls walk_forward(cm);
the adapter owns all the safety:

  1. battery gate      — check_battery_ok_for_motion(); low battery => refusal
  2. SafetyGuard clamp — speed pinned to the LOCKED cap (0.05 m/s) for first
                         AI motion, regardless of what the model wants
  3. distance bound    — clamped to MAX_STEP_CM; model asking for 100 gets 20
  4. duration bound    — computed + capped, so a step can never run away
  5. motion budget     — at most MAX_MOTION_CALLS moves per session
  6. deadman           — heartbeat thread; if this process dies the robot pauses
  7. operator          — you, watching, 0.05 m/s is a slow shuffle

The response the model reads back reports BOTH what it asked for and what the
safety layer allowed — so the clamp is visible to the model, not hidden.

Same script works whether the battery is low (demonstrates the refusal path)
or charged (demonstrates real bounded motion).
"""
import json
import math
import os
import statistics
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dotenv import load_dotenv

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
load_dotenv(os.path.join(_ROOT, ".env"))

API_KEY = os.environ.get("GEMINI_API_KEY", "").strip()
if not API_KEY:
    sys.exit("GEMINI_API_KEY empty — paste into .env")

MODEL = os.environ.get("GEMINI_ER_MODEL", "gemini-robotics-er-1.6-preview")
ROBOT = os.environ.get("ROSPUG_HOST", "rospug.local")

# ---- first-AI-motion bounds (conservative on purpose) ----
MAX_STEP_CM = 20.0          # a single step can never exceed this
MAX_DURATION_S = 5.0        # nor run longer than this
MAX_MOTION_CALLS = 3        # nor more than this many moves per session

import roslibpy
from google import genai
from google.genai import types

from physicalagent import config
from physicalagent.robots.base import RobotAdapter, ToolResult, ToolSpec
from physicalagent.safety import (
    ROSPUG_SAFETY, SafetyGuard, Velocity, Pose, check_battery_ok_for_motion, battery_state,
)


class LiveMotionAdapter(RobotAdapter):
    def __init__(self, ros, guard):
        super().__init__(guard=guard)
        self._ros = ros
        self._st = {}
        self._motion_calls = 0
        roslibpy.Topic(ros, config.ROSPUG.topic_battery, "std_msgs/UInt16").subscribe(
            lambda m: self._st.__setitem__("batt", m["data"]))
        roslibpy.Topic(ros, "/physicalagent/deadman_status", "std_msgs/String").subscribe(
            lambda m: self._st.__setitem__("dead", m["data"]))
        roslibpy.Topic(ros, "/scan_raw", "sensor_msgs/LaserScan",
                       throttle_rate=150, queue_length=1).subscribe(
            lambda m: self._st.__setitem__("scan", m))
        self._vel = roslibpy.Topic(ros, config.ROSPUG.topic_velocity, config.ROSPUG.type_velocity)
        self._pose = roslibpy.Topic(ros, config.ROSPUG.topic_pose, config.ROSPUG.type_pose)
        # heartbeat -> deadman stays ALIVE while we command
        self._beating = threading.Event(); self._beating.set()
        hb = roslibpy.Topic(ros, "/physicalagent/heartbeat", "std_msgs/Bool")
        def _beat():
            while self._beating.is_set():
                hb.publish(roslibpy.Message({"data": True})); time.sleep(0.2)
        threading.Thread(target=_beat, daemon=True).start()

    # -- tools ----------------------------------------------------------
    def capabilities(self):
        return (
            ToolSpec(name="get_status",
                     description=("Robot vitals: battery millivolts (motion refused "
                                  "below 11100) and deadman state (ALIVE/PAUSED/DORMANT)."),
                     parameters={"type": "object", "properties": {}}),
            ToolSpec(name="scan_surroundings",
                     description=("360-degree lidar: nearest+median obstacle distance (m) "
                                  "per sector front/left/right/back. Under 0.3 m is a risk."),
                     parameters={"type": "object", "properties": {}}),
            ToolSpec(name="walk_forward",
                     description=(
                         "Take a small step straight forward. Provide the desired "
                         "distance in centimetres. The robot's safety layer will clamp "
                         "the distance, speed, and duration to safe limits and report "
                         "back exactly what it actually did, which may be less than "
                         "requested. Only walk forward if the front sector is clear."),
                     parameters={"type": "object",
                                 "properties": {"distance_cm": {
                                     "type": "number",
                                     "description": "desired forward distance in cm"}},
                                 "required": ["distance_cm"]}),
            ToolSpec(name="stop",
                     description="Immediately halt all motion and hold position.",
                     parameters={"type": "object", "properties": {}}),
        )

    def _handlers(self):
        return {"get_status": self._status, "scan_surroundings": self._scan,
                "walk_forward": self._walk, "stop": self._stop}

    def _status(self):
        mv = self._st.get("batt")
        return {"battery_mv": mv, "battery_volts": round(mv/1000, 2) if mv else None,
                "deadman": self._st.get("dead", "unknown"),
                "moves_used": self._motion_calls, "moves_remaining": MAX_MOTION_CALLS - self._motion_calls}

    def _scan(self):
        m = self._st.get("scan")
        if not m:
            raise RuntimeError("no lidar yet")
        r = m["ranges"]; amin = m["angle_min"]; inc = m["angle_increment"]
        sect = {"front": 0.0, "left": 1.571, "right": -1.571, "back": 3.1416}
        out = {}
        for name, c in sect.items():
            vals = []
            for i, d in enumerate(r):
                if not isinstance(d, (int, float)) or d <= 0.12 or d > 12 or d != d:
                    continue
                a = amin + i*inc
                while a > 3.1416: a -= 6.2832
                while a < -3.1416: a += 6.2832
                diff = abs(a-c)
                if diff > 3.1416: diff = 6.2832-diff
                if diff <= 0.785:
                    vals.append(d)
            out[name] = ({"nearest_m": round(min(vals), 2),
                          "median_m": round(statistics.median(vals), 2)} if vals else "clear")
        return {"sectors": out}

    def _wall_perp(self, n=18):
        v = []
        for _ in range(n):
            m = self._st.get("scan")
            if m:
                pts = []
                r = m["ranges"]; amin = m["angle_min"]; inc = m["angle_increment"]
                for i, d in enumerate(r):
                    if not isinstance(d, (int, float)) or d <= 0.12 or d > 12 or d != d:
                        continue
                    a = amin + i*inc
                    while a > 3.1416: a -= 6.2832
                    while a < -3.1416: a += 6.2832
                    if abs(a) <= 0.52:
                        pts.append((d*math.cos(a), d*math.sin(a)))
                if len(pts) >= 12:
                    k = len(pts); mx = sum(p[0] for p in pts)/k; my = sum(p[1] for p in pts)/k
                    sxx = sum((p[0]-mx)**2 for p in pts); syy = sum((p[1]-my)**2 for p in pts)
                    sxy = sum((p[0]-mx)*(p[1]-my) for p in pts)
                    th = 0.5*math.atan2(2*sxy, sxx-syy)
                    v.append(abs(-math.sin(th)*mx + math.cos(th)*my))
            time.sleep(0.07)
        return statistics.median(v) if v else None

    def _walk(self, distance_cm):
        # 1. battery gate — raises SafetyError -> execute() returns a refusal
        mv = self._st.get("batt")
        check_battery_ok_for_motion(mv)
        # 5. motion budget
        if self._motion_calls >= MAX_MOTION_CALLS:
            raise RuntimeError(f"motion budget exhausted ({MAX_MOTION_CALLS} moves used)")
        # 3. distance bound
        requested = float(distance_cm)
        bounded = max(0.0, min(requested, MAX_STEP_CM))
        if bounded <= 0:
            return {"requested_cm": requested, "note": "non-positive distance; no motion"}
        # 2. speed clamp via SafetyGuard (locked cap)
        v = self._guard.guard_velocity(Velocity(x=0.10, y=0.0, yaw_rate=0.0))
        speed = v.x
        # 4. duration bound
        duration = min(bounded/100.0/speed, MAX_DURATION_S)
        before = self._wall_perp()
        self._motion_calls += 1
        self._vel.publish(roslibpy.Message({"x": float(speed), "y": 0.0,
                                            "yaw_rate": 0.0, "stop": False}))
        time.sleep(duration)
        self._vel.publish(roslibpy.Message({"x": 0.0, "y": 0.0, "yaw_rate": 0.0, "stop": True}))
        time.sleep(2.5)
        after = self._wall_perp()
        travelled = round((before-after)*100, 1) if (before and after) else None
        return {"requested_cm": requested, "allowed_cm": round(bounded, 1),
                "commanded_speed_ms": round(speed, 3), "duration_s": round(duration, 2),
                "measured_travel_cm": travelled if travelled is not None else "unmeasured",
                "deadman": self._st.get("dead"), "moves_remaining": MAX_MOTION_CALLS - self._motion_calls}

    def _stop(self):
        self.stop_all()
        return {"stopped": True}

    def get_frame(self):
        import urllib.request
        return urllib.request.urlopen(
            f"http://{ROBOT}:8080/snapshot?topic=/csi_camera/image_raw&quality=90", timeout=10).read()

    def telemetry(self):
        return self._status()

    def stop_all(self):
        try:
            self._vel.publish(roslibpy.Message({"x": 0.0, "y": 0.0, "yaw_rate": 0.0, "stop": True}))
        except Exception:
            pass

    def base_position(self):
        p = self._guard.guard_pose(Pose(height=config.DEFAULT_HEIGHT_M, run_time=1.0))
        self._pose.publish(roslibpy.Message({"roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            "height": float(p.height), "x_shift": float(p.x_shift),
            "stance_x": float(p.stance_x), "stance_y": float(p.stance_y),
            "run_time": float(p.run_time)}))
        time.sleep(2)

    def shutdown(self):
        self._beating.clear()


def main():
    print(f"model: {MODEL}\nrobot: {ROBOT}")
    print(f"bounds: <={MAX_STEP_CM}cm/step, <={MAX_DURATION_S}s, <={MAX_MOTION_CALLS} moves, "
          f"speed LOCKED at {'0.05' if not config.MOTION_UNLOCKED else '0.10'} m/s\n")
    ros = roslibpy.Ros(host=ROBOT, port=9090); ros.run()
    guard = SafetyGuard(ROSPUG_SAFETY, motion_unlocked=config.MOTION_UNLOCKED,
                        action_whitelist=config.ROSPUG.action_groups)
    adapter = LiveMotionAdapter(ros, guard)
    time.sleep(4)

    client = genai.Client(api_key=API_KEY)
    decls = [types.FunctionDeclaration(name=s.name, description=s.description,
                                       parameters_json_schema=s.parameters)
             for s in adapter.capabilities()]
    tools = [types.Tool(function_declarations=decls)]

    contents = [types.Content(role="user", parts=[types.Part.from_text(text=(
        "You are the brain of a small quadruped robot (HiWonder ROSPug) on a carpeted "
        "floor. Your task: FIRST verify it is safe to move (check vitals and what is in "
        "front), and ONLY IF it is safe and the battery permits, take one small step "
        "forward of about 10 cm to demonstrate you can move. If anything prevents motion, "
        "do not force it — just explain why. Finish with a one-paragraph report of what "
        "you did and what actually happened (including any difference between what you "
        "requested and what the robot allowed)."
    ))])]

    try:
        for hop in range(8):
            resp = client.models.generate_content(
                model=MODEL, contents=contents,
                config=types.GenerateContentConfig(tools=tools, temperature=0.0))
            cand = resp.candidates[0]
            calls = [p.function_call for p in cand.content.parts if p.function_call]
            if not calls:
                print(f"\n{'='*68}\nGEMINI'S REPORT:\n{'='*68}\n{resp.text}")
                break
            contents.append(cand.content)
            parts = []
            for fc in calls:
                args = dict(fc.args or {})
                print(f"  hop {hop}: {fc.name}({args})")
                result = adapter.execute(fc.name, args)
                print(f"           -> {json.dumps(result.to_gemini())[:220]}")
                parts.append(types.Part.from_function_response(name=fc.name, response=result.to_gemini()))
            contents.append(types.Content(role="user", parts=parts))
        else:
            print("!! no final answer in 8 hops")
    finally:
        adapter.stop_all(); time.sleep(0.5)
        try: adapter.base_position()
        except Exception: pass
        adapter.shutdown(); time.sleep(0.3)
        ros.terminate()
        print("\nrobot halted, base position. motion sanity complete.")


if __name__ == "__main__":
    main()
