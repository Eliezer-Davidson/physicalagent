"""Benchmark the EXACT path Gemini will use: roslibpy -> rosbridge WebSocket.
Runs identical tests over ethernet and WiFi against the same robot, same rosbridge.
"""
import sys, time, statistics, subprocess, threading
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import roslibpy
from physicalagent import config

import os
PATHS = [(os.environ.get("BL","X"), os.environ.get("BH","127.0.0.1"))]

def icmp(host, n=30):
    try:
        out = subprocess.run(["ping","-c",str(n),"-i","0.2","-W","2",host],
                             capture_output=True, text=True, timeout=40).stdout
        loss = [l for l in out.splitlines() if "packet loss" in l]
        rtt  = [l for l in out.splitlines() if "rtt min" in l or "round-trip" in l]
        return (loss[0].strip() if loss else "?"), (rtt[0].strip() if rtt else "?")
    except Exception as e:
        return f"ping failed: {e}", "?"

def pct(v, p):
    if not v: return float("nan")
    s = sorted(v); k = (len(s)-1)*p/100.0
    f = int(k); c = min(f+1, len(s)-1)
    return s[f] + (s[c]-s[f])*(k-f)

def bench(label, host):
    print(f"\n{'='*70}\n{label}  ({host})\n{'='*70}")
    loss, rtt = icmp(host)
    print(f"  ICMP  {loss}")
    print(f"  ICMP  {rtt}")

    t0 = time.monotonic()
    ros = roslibpy.Ros(host=host, port=9090)
    ros.run(); time.sleep(1.5)
    if not ros.is_connected:
        print("  !! rosbridge did not connect"); return None
    print(f"  rosbridge connect: {(time.monotonic()-t0)*1000:.0f} ms")

    # --- 1. service-call round trip (rosapi) = command-ack latency proxy ---
    rtts = []
    for _ in range(25):
        s = time.monotonic()
        try:
            ros.get_topics(); rtts.append((time.monotonic()-s)*1000)
        except Exception: pass
        time.sleep(0.05)
    if rtts:
        print(f"  rosapi RTT: median {statistics.median(rtts):6.1f} ms   "
              f"p95 {pct(rtts,95):6.1f} ms   max {max(rtts):6.1f} ms   n={len(rtts)}")

    # --- 2. telemetry jitter on a small, fast topic (IMU) ---
    stamps = []
    lock = threading.Lock()
    def on_imu(_m):
        with lock: stamps.append(time.monotonic())
    imu = roslibpy.Topic(ros, config.ROSPUG.topic_imu, "sensor_msgs/Imu", queue_length=1)
    imu.subscribe(on_imu)
    time.sleep(20)
    imu.unsubscribe()
    with lock: s = list(stamps)
    if len(s) > 5:
        gaps = [(b-a)*1000 for a,b in zip(s, s[1:])]
        print(f"  IMU stream: {len(s)} msgs in 20s = {len(s)/20:.1f} Hz")
        print(f"              gap median {statistics.median(gaps):5.1f} ms  "
              f"p95 {pct(gaps,95):5.1f} ms  max {max(gaps):6.1f} ms")
        stalls = [g for g in gaps if g > 500]
        print(f"              stalls >500ms: {len(stalls)}"
              + (f"  worst {max(stalls):.0f} ms" if stalls else ""))
    else:
        print(f"  IMU stream: only {len(s)} msgs — topic may be idle")

    # --- 3. big-payload throughput (lidar scan ~ 1000 floats) ---
    sc = []
    def on_scan(m):
        sc.append((time.monotonic(), len(m.get("ranges", []))))
    scan = roslibpy.Topic(ros, config.ROSPUG.topic_scan, "sensor_msgs/LaserScan", queue_length=1)
    scan.subscribe(on_scan)
    time.sleep(15)
    scan.unsubscribe()
    if len(sc) > 3:
        gaps = [(b[0]-a[0])*1000 for a,b in zip(sc, sc[1:])]
        pts = sc[0][1]
        print(f"  LIDAR stream: {len(sc)} scans in 15s = {len(sc)/15:.1f} Hz, {pts} pts/scan")
        print(f"                gap median {statistics.median(gaps):5.1f} ms  max {max(gaps):6.1f} ms")
        print(f"                approx payload {pts*8*len(sc)/15/1024:.0f} KB/s decoded")
    else:
        print(f"  LIDAR stream: only {len(sc)} scans")

    ros.terminate(); time.sleep(0.5)
    return {"rtt_med": statistics.median(rtts) if rtts else None,
            "imu_hz": len(s)/20 if len(s)>5 else 0,
            "scan_hz": len(sc)/15 if len(sc)>3 else 0}

res = {}
for label, host in PATHS:
    res[label] = bench(label, host)

import json; print("JSON:"+json.dumps({k:v for k,v in res.items()}))
print(f"  {'metric':<22}{'ETHERNET':>14}{'WIFI(AP)':>14}")
e, w = res.get("ETHERNET"), res.get("WIFI(AP)")
if e and w:
    print(f"  {'rosapi RTT median':<22}{e['rtt_med']:>11.1f} ms{w['rtt_med']:>11.1f} ms")
    print(f"  {'IMU rate':<22}{e['imu_hz']:>11.1f} Hz{w['imu_hz']:>11.1f} Hz")
    print(f"  {'LIDAR rate':<22}{e['scan_hz']:>11.1f} Hz{w['scan_hz']:>11.1f} Hz")
