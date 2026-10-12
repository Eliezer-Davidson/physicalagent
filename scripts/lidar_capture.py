"""Capture a lidar scan and render it as a top-down SVG map (no numpy/matplotlib)."""
import os, sys, time, math, json
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import roslibpy
from physicalagent import config

TOPIC = __import__("os").environ.get("SCAN_TOPIC","/scan")
HOST = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("ROSPUG_HOST", "rospug.local")
OUT  = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "artifacts", "session3", "lidar.svg")

ros = roslibpy.Ros(host=HOST, port=9090); ros.run(); time.sleep(2)
got = []
roslibpy.Topic(ros, TOPIC, "sensor_msgs/LaserScan",
               queue_length=1).subscribe(lambda m: got.append(m))
for _ in range(60):
    time.sleep(0.1)
    if len(got) >= 6: break
ros.terminate()
if not got:
    sys.exit("no scan received")

s = got[-1]
r = s["ranges"]; amin = s["angle_min"]; inc = s["angle_increment"]
rmin, rmax = s.get("range_min",0), s.get("range_max",0)
print(f"frame_id      : {s.get('header',{}).get('frame_id')}")
print(f"points        : {len(r)}")
print(f"angle_min/max : {amin:.4f} .. {s.get('angle_max',0):.4f} rad "
      f"({math.degrees(amin):.1f} .. {math.degrees(s.get('angle_max',0)):.1f} deg)")
print(f"increment     : {inc:.6f} rad ({math.degrees(inc):.3f} deg/step)")
print(f"range_min/max : {rmin} .. {rmax} m")
print(f"scan_time     : {s.get('scan_time')}  time_increment {s.get('time_increment')}")

pts = []
for i, d in enumerate(r):
    if not isinstance(d,(int,float)) or d <= 0.02 or d > 12 or d != d:
        continue
    a = amin + i*inc
    pts.append((d*math.cos(a), d*math.sin(a), d, a))
valid = len(pts)
print(f"valid returns : {valid}/{len(r)} ({100*valid/len(r):.0f}%)")
if pts:
    ds = sorted(p[2] for p in pts)
    print(f"nearest       : {ds[0]:.3f} m     farthest: {ds[-1]:.3f} m")
    print(f"median range  : {ds[len(ds)//2]:.3f} m")

# ---- render SVG (top-down, robot at centre, +x = forward = up) ----
W = H = 720; CX = CY = H/2
span = max((max(abs(p[0]) for p in pts), max(abs(p[1]) for p in pts))) if pts else 1
span = max(span, 0.5)
scale = (H/2 - 40)/span

def sx(x, y):  # ROS x fwd, y left -> screen: forward=up, left=left
    return CX - y*scale, CY - x*scale

rings = []
step = 1.0 if span > 2 else 0.5
k = step
while k <= span:
    rings.append(f'<circle cx="{CX}" cy="{CY}" r="{k*scale:.1f}" fill="none" '
                 f'stroke="#2a3a4a" stroke-width="1" stroke-dasharray="3,4"/>')
    rings.append(f'<text x="{CX+4}" y="{CY-k*scale+12:.1f}" fill="#5b7a99" '
                 f'font-size="11" font-family="monospace">{k:g}m</text>')
    k += step

dots = []
for x, y, d, a in pts:
    px, py = sx(x, y)
    hue = 200 - min(d/span, 1.0)*160        # near = warm, far = cool
    dots.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="1.9" '
                f'fill="hsl({hue:.0f},85%,58%)" opacity="0.92"/>')

svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">
<rect width="{W}" height="{H}" fill="#0d1520"/>
{''.join(rings)}
<line x1="{CX}" y1="{CY}" x2="{CX}" y2="{CY-span*scale:.0f}" stroke="#3d5a7a" stroke-width="1.5"/>
<text x="{CX+6}" y="{CY-span*scale+26:.0f}" fill="#7da9d4" font-size="13" font-family="monospace">FORWARD</text>
{''.join(dots)}
<circle cx="{CX}" cy="{CY}" r="7" fill="#ff5a5a"/>
<polygon points="{CX},{CY-14} {CX-6},{CY+5} {CX+6},{CY+5}" fill="#ff5a5a"/>
<text x="14" y="26" fill="#c8dbef" font-size="15" font-family="monospace">ROSPug lidar — {valid} returns, {span:.1f} m span</text>
<text x="14" y="46" fill="#6f8ba8" font-size="12" font-family="monospace">nearest {ds[0]:.2f} m · farthest {ds[-1]:.2f} m · {len(r)} rays</text>
</svg>'''
with open(OUT, "w") as f:
    f.write(svg)
print(f"\nSVG written: {OUT}")
json.dump({"points":len(r),"valid":valid,"nearest":ds[0],"farthest":ds[-1],
           "angle_min":amin,"increment":inc,"frame":s.get('header',{}).get('frame_id')},
          open(OUT.replace(".svg",".json"),"w"), indent=2)
