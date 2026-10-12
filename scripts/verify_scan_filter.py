"""Verify the survey claim: /scan has its REAR sector filtered out, /scan_raw does not."""
import sys, time, math
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import roslibpy
HOST = os.environ.get("ROSPUG_HOST", "rospug.local")   # never a literal site address
ros = roslibpy.Ros(host=HOST, port=9090); ros.run(); time.sleep(2)
got={}
for t in ["/scan", "/scan_raw"]:
    roslibpy.Topic(ros, t, "sensor_msgs/LaserScan", queue_length=1).subscribe(
        lambda m, t=t: got.setdefault(t, m))
for _ in range(50):
    time.sleep(0.2)
    if len(got)==2: break
for t,m in got.items():
    r=m["ranges"]; amin=m["angle_min"]; inc=m["angle_increment"]
    valid=[(amin+i*inc) for i,d in enumerate(r)
           if isinstance(d,(int,float)) and d==d and 0.05<d<20]
    invalid=[(amin+i*inc) for i,d in enumerate(r)
             if not(isinstance(d,(int,float)) and d==d and 0.05<d<20)]
    print(f"\n{t}: {len(r)} rays, {len(valid)} valid ({100*len(valid)/len(r):.0f}%)")
    if invalid:
        lo,hi=min(invalid),max(invalid)
        # find the contiguous dead band
        inv=sorted(invalid)
        print(f"   invalid beams span {lo:.3f}..{hi:.3f} rad ({math.degrees(lo):.0f}..{math.degrees(hi):.0f} deg)")
        # count invalid inside the claimed filter window [2, 4.28]
        inwin=[a for a in invalid if 2.0<=a<=4.28]
        print(f"   invalid INSIDE claimed filter window [2.0,4.28] rad: {len(inwin)}/{len(invalid)}"
              f" ({100*len(inwin)/len(invalid):.0f}%)")
    else:
        print("   no invalid beams")
if len(got)==2:
    a=sum(1 for d in got['/scan']['ranges'] if isinstance(d,(int,float)) and d==d and 0.05<d<20)
    b=sum(1 for d in got['/scan_raw']['ranges'] if isinstance(d,(int,float)) and d==d and 0.05<d<20)
    print(f"\n>>> VERDICT: /scan_raw has {b-a} MORE valid returns than /scan "
          f"({b} vs {a}) — filter confirmed" if b>a else f"\n>>> no difference ({a} vs {b})")
ros.terminate()
