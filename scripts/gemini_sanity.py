#!/usr/bin/env python3
"""FIRST GEMINI CONTACT — sensing-only sanity check.

Proves the full function-calling loop end-to-end with ZERO motion risk:

    ToolSpec (base.py) -> Gemini FunctionDeclarations
    -> model emits a tool call
    -> RobotAdapter.execute() validates + runs it on the LIVE robot
    -> FunctionResponse back to the model
    -> model's final text

Tools exposed: get_status, scan_surroundings. Nothing that moves. The
adapter here is a minimal live sensing adapter; the full ROSPug adapter
(with motion + closed-loop turn) is built separately with TDD.

Test B sends a live camera frame and asks ER for scene understanding with
its signature [y, x] 0-1000 pointing.
"""
import json
import os
import statistics
import sys
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dotenv import load_dotenv

load_dotenv(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), ".env"))

API_KEY = os.environ.get("GEMINI_API_KEY", "").strip()
if not API_KEY:
    sys.exit("GEMINI_API_KEY is empty — paste it into .env first")

MODEL = os.environ.get("GEMINI_ER_MODEL", "gemini-robotics-er-1.6-preview")
ROBOT = os.environ.get("ROSPUG_HOST", "rospug.local")

import roslibpy
from google import genai
from google.genai import types

from physicalagent.robots.base import RobotAdapter, ToolResult, ToolSpec


# ---------------------------------------------------------------------------
# Minimal LIVE sensing adapter (no motion tools at all)
# ---------------------------------------------------------------------------
class LiveSensingAdapter(RobotAdapter):
    def __init__(self, ros):
        super().__init__(guard=None)
        self._st = {}
        roslibpy.Topic(ros, "/ros_robot_controller/battery", "std_msgs/UInt16").subscribe(
            lambda m: self._st.__setitem__("batt", m["data"]))
        roslibpy.Topic(ros, "/physicalagent/deadman_status", "std_msgs/String").subscribe(
            lambda m: self._st.__setitem__("dead", m["data"]))
        roslibpy.Topic(ros, "/scan_raw", "sensor_msgs/LaserScan",
                       throttle_rate=200, queue_length=1).subscribe(
            lambda m: self._st.__setitem__("scan", m))

    def capabilities(self):
        return (
            ToolSpec(
                name="get_status",
                description=(
                    "Read the robot's vital signs: battery voltage (millivolts; "
                    "motion is refused below 11100), and the safety deadman state "
                    "(ALIVE = controller connected, PAUSED = robot holding position, "
                    "DORMANT = deadman unarmed)."
                ),
                parameters={"type": "object", "properties": {}},
            ),
            ToolSpec(
                name="scan_surroundings",
                description=(
                    "360-degree lidar summary around the robot. Returns nearest and "
                    "median obstacle distance in metres for each sector: front, "
                    "left, right, back. Distances under 0.3 m are collision risks."
                ),
                parameters={"type": "object", "properties": {}},
            ),
        )

    def _handlers(self):
        return {"get_status": self._status, "scan_surroundings": self._scan}

    def _status(self):
        mv = self._st.get("batt")
        return {
            "battery_mv": mv,
            "battery_volts": round(mv / 1000, 2) if mv else None,
            "motion_floor_mv": 11100,
            "deadman": self._st.get("dead", "unknown"),
        }

    def _scan(self):
        import math
        m = self._st.get("scan")
        if not m:
            raise RuntimeError("no lidar data yet")
        r = m["ranges"]; amin = m["angle_min"]; inc = m["angle_increment"]
        sectors = {"front": (0.0, 0.785), "left": (1.571, 0.785),
                   "right": (-1.571, 0.785), "back": (3.1416, 0.785)}
        out = {}
        for name, (centre, half) in sectors.items():
            vals = []
            for i, d in enumerate(r):
                if not isinstance(d, (int, float)) or d <= 0.12 or d > 12 or d != d:
                    continue  # >0.12 skips the robot's own body
                a = amin + i * inc
                while a > 3.1416: a -= 6.2832
                while a < -3.1416: a += 6.2832
                diff = abs(a - centre)
                if diff > 3.1416: diff = 6.2832 - diff
                if diff <= half:
                    vals.append(d)
            out[name] = ({"nearest_m": round(min(vals), 2),
                          "median_m": round(statistics.median(vals), 2)}
                         if vals else "no returns")
        return {"sectors": out}

    def get_frame(self):
        url = f"http://{ROBOT}:8080/snapshot?topic=/csi_camera/image_raw&quality=90"
        return urllib.request.urlopen(url, timeout=10).read()

    def telemetry(self):
        return self._status()

    def stop_all(self):
        pass  # sensing-only adapter commands no motion


# ---------------------------------------------------------------------------
def main():
    print(f"model: {MODEL}\nrobot: {ROBOT}\n")
    ros = roslibpy.Ros(host=ROBOT, port=9090)
    ros.run()
    adapter = LiveSensingAdapter(ros)
    time.sleep(4)  # battery topic needs >3s window

    client = genai.Client(api_key=API_KEY)
    decls = [types.FunctionDeclaration(name=s.name, description=s.description,
                                       parameters_json_schema=s.parameters)
             for s in adapter.capabilities()]
    tools = [types.Tool(function_declarations=decls)]

    # ------------------- TEST A: function-calling loop -------------------
    print("=" * 68)
    print("TEST A — function calling against the live robot")
    print("=" * 68)
    contents = [types.Content(role="user", parts=[types.Part.from_text(text=(
        "You are the brain of a small quadruped robot (a HiWonder ROSPug). "
        "Use your tools to check the robot's vital signs and what is "
        "physically around it, then give me a two-sentence situation report. "
        "Mention whether it would be safe to walk forward."
    ))])]

    for hop in range(6):
        resp = client.models.generate_content(
            model=MODEL, contents=contents,
            config=types.GenerateContentConfig(tools=tools, temperature=0.0),
        )
        cand = resp.candidates[0]
        calls = [p.function_call for p in cand.content.parts if p.function_call]
        if not calls:
            print(f"\nFINAL ANSWER:\n{resp.text}")
            break
        contents.append(cand.content)
        parts = []
        for fc in calls:
            args = dict(fc.args or {})
            print(f"  hop {hop}: model called {fc.name}({args})")
            result = adapter.execute(fc.name, args)
            print(f"           -> {json.dumps(result.to_gemini())[:180]}")
            parts.append(types.Part.from_function_response(
                name=fc.name, response=result.to_gemini()))
        contents.append(types.Content(role="user", parts=parts))
    else:
        print("!! model never produced a final answer in 6 hops")

    # ------------------- TEST B: vision + ER pointing --------------------
    print()
    print("=" * 68)
    print("TEST B — live camera frame + ER spatial pointing")
    print("=" * 68)
    jpeg = adapter.get_frame()
    print(f"  camera frame: {len(jpeg)} bytes")
    resp = client.models.generate_content(
        model=MODEL,
        contents=[types.Content(role="user", parts=[
            types.Part.from_bytes(data=jpeg, mime_type="image/jpeg"),
            types.Part.from_text(text=(
                "This is the live view from a small quadruped robot at floor "
                "level. 1) Describe the scene in one sentence. 2) List up to 5 "
                "notable objects as JSON: [{\"label\": str, \"point\": [y, x]}] "
                "with coordinates normalized 0-1000. 3) Name the biggest "
                "hazard for a 25 cm tall walking robot here."
            )),
        ])],
        config=types.GenerateContentConfig(temperature=0.0),
    )
    print(resp.text)

    ros.terminate()
    print("\nsanity check complete.")


if __name__ == "__main__":
    main()
