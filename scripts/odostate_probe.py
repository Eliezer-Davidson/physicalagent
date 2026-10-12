"""Bench probe for validating rt/odostate on hardware.

    .venv_sim/bin/python scripts/odostate_probe.py

WHY
    Whether the physical G1 publishes 
    rt/odostate in the same frame/byte order as the simulator is an 
    ASSUMPTION. The fall detector relies on tilt and height from this 
    topic. If the byte order is different on hardware, or if Z is not Up,
    the safety shield will falsely trigger or fail to trigger.

HOW TO USE
    1. Connect this script to the real robot (modify interface if needed).
    2. Hold the robot safely in the air.
    3. Tilt it forward, backward, left, right, and watch the Pitch and Roll.
    4. Move it up and down, and watch Height (Z).
    5. Verify the numbers match the physical reality.
"""
import math
import sys
import time

import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from physicalagent.robots import sonic_telemetry

def main():
    # Hardware validation usually needs the actual network interface, e.g., 'eth0'.
    # Defaulting to 'lo' so it works in sim out of the box.
    interface = "lo"
    if len(sys.argv) > 1:
        interface = sys.argv[1]

    print(f"Starting telemetry on interface: {interface}")
    telemetry = sonic_telemetry.UnitreeTelemetry(init_channel=True, interface=interface)

    print("Waiting for rt/odostate...")
    if not telemetry.wait_for_data(timeout_s=5.0):
        print("Timeout: No data received. Is the robot/simulator publishing?")
        telemetry.close()
        return 1

    print("\n--- ODOSTATE BENCH VALIDATION ---")
    print("Hold the robot and tilt it. Verify the signs match physical reality.")
    print("Press Ctrl+C to exit.\n")
    print(f"{'Height (Z)':<12} | {'Tilt (deg)':<12} | {'Pitch (deg)':<12} | {'Roll (deg)':<12}")
    print("-" * 55)

    try:
        while True:
            pose = telemetry.pose()
            if not pose.get("available") or not pose.get("fresh"):
                print("Data stale or unavailable...")
            else:
                h = pose.get('height_m', 0.0)
                t = pose.get('tilt_deg', 0.0)
                p = pose.get('pitch_deg', 0.0)
                r = pose.get('roll_deg', 0.0)
                
                # Overwrite line to make it easy to read
                sys.stdout.write(f"\r{h:<12.3f} | {t:<12.1f} | {p:<12.1f} | {r:<12.1f}")
                sys.stdout.flush()
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        telemetry.close()

    return 0

if __name__ == "__main__":
    sys.exit(main())
