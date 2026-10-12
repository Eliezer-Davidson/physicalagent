#!/usr/bin/env python3
"""Live TonyPi camera view — for focusing the lens, and for watching what the
model sees while it drives the robot.

    python3 tools/tonypi_view.py                 # then open http://127.0.0.1:8090/
    python3 tools/tonypi_view.py --open          # and launch a browser for you
    python3 tools/tonypi_view.py --host 192.168.149.1 --port 8090

WHY THIS EXISTS RATHER THAN JUST OPENING THE ROBOT'S OWN STREAM.
The robot serves two video endpoints and they do not behave the same way.
Measured on hardware 2026-07-28, heartbeat fed, function 9 loaded, ten samples
one second apart on each:

    ?action=snapshot    10/10 DISTINCT    65443..66076 B
    ?action=stream       1/10 distinct    15986 B, byte-identical

So the endpoint that *looks* like the live one is the stale one. This tool polls
the snapshot endpoint and re-serves it as a real MJPEG stream on localhost.

IT RUNS ALONGSIDE THE CONTROLLER. Watching the feed while Gemini drives the robot
is the intended use. Nothing here conflicts with motion: the adapter's motion
calls (RunAction, SetBusServoPulse, StopActionGroup) are separate RPC methods,
and function 9 (lab_adjust) is camera-only with no motion of its own. This tool
also owns the camera keepalive, which means the camera stays alive during the
long gaps while the model is thinking — the controller only feeds the heartbeat
while a session is running.

THE SHARPNESS NUMBER. The TonyPi lens is FIXED-FOCUS and adjusted by physically
rotating the barrel (vendor FAQ Q1). Focusing by eye on a blurry preview is
guesswork; focusing by maximising a number is not. Turn the barrel until the
value peaks. For reference, this camera went from ~275 (as shipped) to ~3030
(focused) — an 11x change, so a correct adjustment is obvious, not marginal.
"""
import argparse
import io
import json
import subprocess
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    from PIL import Image, ImageFilter, ImageStat
except ImportError:                       # degrade, never fail: raw frames still stream
    Image = None

CAMERA_FUNC = 9                           # lab_adjust: opens the camera, moves nothing
HEARTBEAT_S = 3.0                         # vendor deadman is 7 s
POLL_S = 0.10

state = {"jpeg": None, "sharp": 0.0, "peak": 0.0, "n": 0, "err": None, "fps": 0.0}
_id = [0]


class Robot:
    def __init__(self, host):
        self.rpc_url = f"http://{host}:9030"
        self.snap_url = f"http://{host}:8080/?action=snapshot"

    def rpc(self, method, params=()):
        _id[0] += 1
        body = json.dumps({"jsonrpc": "2.0", "method": method,
                           "params": list(params), "id": _id[0]}).encode()
        req = urllib.request.Request(
            self.rpc_url, data=body, headers={"Content-Type": "application/json"})
        return json.loads(urllib.request.urlopen(req, timeout=5).read()).get("result")

    def arm(self):
        """Load the camera function and start it. True on success."""
        try:
            self.rpc("LoadFunc", [CAMERA_FUNC])
            self.rpc("StartFunc", [])
            return True
        except Exception:
            return False

    def grab(self):
        return urllib.request.urlopen(self.snap_url, timeout=5).read()


def keepalive(robot):
    """Feed the deadman, and RE-ARM after any gap.

    A dropped heartbeat is not merely a missed ping: by the time the link is
    back, the vendor's heartbeatTask has already called unloadFunc(), which
    closes the camera. Recovery therefore has to re-load and re-start the
    function, not just resume pinging. The AP dongle drops often enough that
    this matters.
    """
    missed = 0
    while True:
        try:
            robot.rpc("Heartbeat", [])
            if missed:
                robot.arm()
                missed = 0
                state["err"] = None
        except Exception:
            missed += 1
            state["err"] = f"link lost x{missed}"
        time.sleep(HEARTBEAT_S)


def grabber(robot, crop=True):
    last, frames = time.time(), 0
    while True:
        try:
            raw = robot.grab()
            if Image is None:
                state["jpeg"] = raw
            else:
                im = Image.open(io.BytesIO(raw))
                w, h = im.size
                # This camera publishes 640x960 with the bottom half a BYTE-EXACT
                # copy of the top (mean abs diff 0.00, zero offset both axes).
                # Duplication, not stereo — there is no disparity to recover.
                if crop and h % 2 == 0:
                    top, bot = im.crop((0, 0, w, h // 2)), im.crop((0, h // 2, w, h))
                    from PIL import ImageChops
                    if ImageStat.Stat(ImageChops.difference(
                            top.convert("L"), bot.convert("L"))).mean[0] <= 0.5:
                        im = top
                state["sharp"] = ImageStat.Stat(
                    im.convert("L").filter(ImageFilter.FIND_EDGES)).var[0]
                state["peak"] = max(state["peak"], state["sharp"])
                out = io.BytesIO()
                im.save(out, format="JPEG", quality=85)
                state["jpeg"] = out.getvalue()
            state["n"] += 1
            state["err"] = None
            frames += 1
            if time.time() - last >= 1.0:
                state["fps"] = frames / (time.time() - last)
                last, frames = time.time(), 0
        except Exception as exc:
            state["err"] = f"{type(exc).__name__}"
        time.sleep(POLL_S)


PAGE = b"""<!doctype html><meta charset=utf-8><title>TonyPi live</title>
<style>
 :root{color-scheme:dark}
 body{background:#14110e;color:#e8e2d9;font:14px/1.5 ui-monospace,SFMono-Regular,monospace;
      margin:0;display:flex;flex-direction:column;align-items:center;gap:12px;padding:18px}
 h1{font-size:13px;letter-spacing:.16em;text-transform:uppercase;color:#E0913F;margin:0;font-weight:600}
 img{width:min(94vw,820px);border:1px solid #3a322a;border-radius:3px;display:block;background:#000}
 .bar{width:min(94vw,820px);height:22px;background:#241f19;border:1px solid #3a322a;
      border-radius:3px;overflow:hidden}
 .fill{height:100%;background:linear-gradient(90deg,#7a4f1c,#E0913F);width:0%;transition:width .12s}
 .row{width:min(94vw,820px);display:flex;justify-content:space-between;align-items:baseline;
      font-variant-numeric:tabular-nums}
 b{color:#E0913F;font-size:24px}
 small{color:#9a9086}
 .err{color:#d9694a}
</style>
<h1>TonyPi &mdash; live &middot; turn the lens until sharpness peaks</h1>
<img id=v src="/stream">
<div class=bar><div class=fill id=f></div></div>
<div class=row>
 <span><small>sharpness</small> <b id=s>-</b></span>
 <span><small>best</small> <b id=k>-</b></span>
 <span><small id=m></small></span>
</div>
<script>
setInterval(async()=>{try{
 const d=await(await fetch('/sharp')).json();
 s.textContent=d.sharp.toFixed(0); k.textContent=d.peak.toFixed(0);
 f.style.width=Math.min(100,100*d.sharp/Math.max(d.peak,1))+'%';
 m.textContent=d.err?('LINK '+d.err):(d.fps.toFixed(1)+' fps \\u00b7 '+d.n+' frames');
 m.className=d.err?'err':'';
}catch(e){m.textContent='viewer offline';m.className='err'}},150);
</script>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path.startswith("/sharp"):
            b = json.dumps({k: state[k] for k in
                            ("sharp", "peak", "n", "err", "fps")}).encode()
            self._send(b, "application/json")
        elif self.path.startswith("/stream"):
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    j = state["jpeg"]
                    if j:
                        self.wfile.write(
                            b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                            + str(len(j)).encode() + b"\r\n\r\n" + j + b"\r\n")
                    time.sleep(POLL_S)
            except Exception:
                pass                       # browser closed the tab; not an error
        else:
            self._send(PAGE, "text/html; charset=utf-8")

    def _send(self, body, ctype):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="192.168.149.1")
    p.add_argument("--port", type=int, default=8090)
    p.add_argument("--no-crop", action="store_true",
                   help="serve the raw 640x960 frame including the duplicated half")
    p.add_argument("--open", action="store_true", help="launch a browser")
    args = p.parse_args()

    robot = Robot(args.host)
    for attempt in range(1, 21):           # the AP dongle blips; do not die on it
        if robot.arm():
            print(f"armed on attempt {attempt}")
            break
        print(f"  attempt {attempt}: {args.host} unreachable, retrying...", flush=True)
        time.sleep(2.0)
    else:
        print("could not reach the robot — serving anyway, the grabber keeps trying")

    threading.Thread(target=keepalive, args=(robot,), daemon=True).start()
    threading.Thread(target=grabber, args=(robot, not args.no_crop),
                     daemon=True).start()
    time.sleep(1.0)

    url = f"http://127.0.0.1:{args.port}/"
    print(f"\n  ==>  {url}  <==\n", flush=True)
    if args.open:
        subprocess.Popen(["xdg-open", url],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        ThreadingHTTPServer(("127.0.0.1", args.port), Handler).serve_forever()
    except KeyboardInterrupt:
        print("\nstopped")


if __name__ == "__main__":
    raise SystemExit(main())
