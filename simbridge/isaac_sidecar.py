"""Isaac sidecar: pose, episode, camera and contacts over the same ZMQ
protocol as simbridge/sidecar.py, on the same port, so SimClient and every
probe run unchanged against either backend.

What this file refuses to do is as load-bearing as what it does. An earlier
revision declared every op and answered ok:false stubs -- except _op_ping,
which advertised cameras:['head_camera'] with no camera behind it, and
_op_contacts, which answered "no contacts" (indistinguishable from a floating
robot) instead of "not implemented". Every op here answers from a real
measurement or refuses loudly; scan/objects/hands/hand_state/band remain
unimplemented and say so.

Camera frames come from the vendor sim's own /dev/shm segments (already JPEG,
written by the sim's image server) -- read as ORDINARY FILES, never via
MultiImageReader from another process, which unlinks the writer's segment on
exit (bpo-38119) and darkens the cameras until the sim restarts. Frames carry
the writer's timestamp and are refused when stale: a dead sim's last frame
must never pass for a live view.

Contacts come from the scene's ContactSensor over every robot body
(env.scene["contact_forces"], vendor task cfg). Unlike MuJoCo's contact list
it reports NET force per robot body and cannot name the partner body, and it
counts contact with ANYTHING (table included), not just the world -- both
divergences are stated in the replies rather than papered over.

THREADING CONTRACT, written in blood: the ZMQ thread NEVER touches the env.
Isaac Lab's data buffers compute lazily through omni.physx.tensors, which is
not thread-safe; the first wiring read them from this thread and a probe
polling body_pose through a descent raced the physics step into a device-side
CUDA assert that poisoned the whole GPU context (2026-08-07). The vendor main
loop calls update_snapshot(env) at ~10 Hz; every world-reading op serves that
plain-python snapshot and refuses when it goes stale.

Runs under the Isaac interpreter (unitree_sim_env) -- needs zmq, which the
stdlib-only project venv does not have. tests/test_sidecar_contract.py parses
this file with ast instead of importing it, so protocol drift still fails the
main suite.
"""
from __future__ import annotations

import base64
import ctypes
import json
import pathlib
import time

import zmq

try:
    # A pure R^T . v rotation (it normalises the quaternion first); used here to
    # express world positions in the pelvis frame, matching the MuJoCo sidecar.
    from simbridge.odostate_fields import world_ang_vel_to_body as _to_body_frame
except ImportError:  # running as a bare script from inside simbridge/
    from odostate_fields import world_ang_vel_to_body as _to_body_frame

PROTOCOL = 1
DEFAULT_PORT = 5570

_IMPLEMENTED_OPS = ("ping", "body_pose", "episode", "camera", "contacts")

# A body reporting less net force than this is treated as not in contact --
# the sensor reads a fraction of a newton of noise on airborne bodies, while a
# stance foot carries hundreds. Same order of cutoff the MuJoCo side's contact
# list applies implicitly by only listing actual contact points.
CONTACT_FORCE_N = 1.0

# The vendor image server rewrites each segment every camera cycle; a frame
# older than this is a dead or stalled sim, not a view.
CAMERA_STALE_S = 2.0

# The main sim loop feeds update_snapshot at ~10 Hz; a snapshot older than
# this means the loop stopped and every world-reading op refuses.
SNAPSHOT_STALE_S = 1.0

_SHM_DIR = pathlib.Path("/dev/shm")


class _ShmImageHeader(ctypes.LittleEndianStructure):
    """Mirror of the vendor's SimpleImageHeader (tools/shared_memory_utils.py).
    Payload starts at ctypes.sizeof(...) == 48 -- NOT 128, which is buffer
    padding; reading there yields convincing noise."""
    _fields_ = [
        ("timestamp", ctypes.c_uint64),      # epoch milliseconds
        ("height", ctypes.c_uint32),
        ("width", ctypes.c_uint32),
        ("channels", ctypes.c_uint32),
        ("image_name", ctypes.c_char * 16),
        ("data_size", ctypes.c_uint32),
        ("encoding", ctypes.c_uint32),       # 0=raw BGR, 1=JPEG
        ("quality", ctypes.c_uint32),
    ]


class IsaacSidecar:
    def __init__(self, port: int = DEFAULT_PORT):
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.REP)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.bind(f"tcp://127.0.0.1:{port}")
        self._poller = zmq.Poller()
        self._poller.register(self._sock, zmq.POLLIN)
        self._snap = None
        # NOTE: no fall/reset counters yet -- _op_episode refuses until a real
        # writer exists (counters nothing increments are fabricated zeros).
        print(f"[isaac_sidecar] listening on tcp://127.0.0.1:{port}", flush=True)

    def update_snapshot(self, env) -> None:
        """MAIN SIM THREAD ONLY. The one place env tensors are read.

        The first wiring had the ZMQ thread read env tensors directly; Isaac Lab
        computes those buffers lazily through omni.physx.tensors, which is
        NOT thread-safe, and a get-up probe polling body_pose through a descent
        raced the physics step into a device-side CUDA assert that killed the
        whole GPU context (2026-08-07, first A/B attempt). Now the vendor loop
        calls this at ~10 Hz and the ZMQ thread serves only the resulting
        plain-python snapshot. Swapped in one assignment, so readers see either
        the old snapshot or the new one, never a half-built dict."""
        try:
            robot = env.scene["robot"]
            names = [str(n) for n in robot.body_names]
            pos = robot.data.body_pos_w[0].detach().cpu().tolist()
            pi = names.index("pelvis")
            quat = robot.data.body_quat_w[0, pi].detach().cpu().tolist()
            snap = {
                "wall_t": time.time(),
                "sim_time": self._env_sim_time(env),
                "bodies": {n: [float(v) for v in p]
                           for n, p in zip(names, pos)},
                "pelvis_quat_wxyz": [float(v) for v in quat],
                "contacts": self._env_contact_rows(env),
            }
        except Exception as exc:
            # A bad frame is skipped and the last good one stands -- but a
            # PERSISTENT failure must not be silent: it becomes "no fresh
            # snapshot" refusals with no cause on record.
            self._snap_fails = getattr(self, "_snap_fails", 0) + 1
            if self._snap_fails == 1:
                print(f"[sidecar] update_snapshot failed (first of possibly "
                      f"many): {type(exc).__name__}: {exc}", flush=True)
            return
        # Success resets the failure one-shot: a transient early failure must
        # not consume the breadcrumb a LATER persistent streak needs.
        self._snap_fails = 0
        self._snap = snap

    @staticmethod
    def _env_sim_time(env):
        try:
            return float(env.sim.current_time)
        except AttributeError:
            return None

    @staticmethod
    def _env_contact_rows(env):
        """[[body_name, net_force_N], ...] above the noise cutoff, hardest
        first -- or None when the scene has no contact sensor."""
        try:
            sensor = env.scene["contact_forces"]
            forces = sensor.data.net_forces_w[0].detach().cpu().tolist()
            names = sensor.body_names
        except (KeyError, AttributeError, IndexError):
            return None
        rows = []
        for name, f in zip(names, forces):
            mag = (f[0] * f[0] + f[1] * f[1] + f[2] * f[2]) ** 0.5
            if mag >= CONTACT_FORCE_N:
                rows.append([str(name), mag])
        rows.sort(key=lambda r: -r[1])
        return rows

    # -- plumbing (ZMQ thread; touches self._snap and /dev/shm, NEVER env) --

    def poll(self) -> None:
        try:
            if not self._poller.poll(0):
                return
            raw = self._sock.recv()
        except Exception:
            return

        try:
            req = json.loads(raw.decode())
            reply = self._dispatch(req)
        except Exception as exc:
            reply = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}

        # Serialise OUTSIDE the send guard: a dumps failure that skips send()
        # would leave the REP socket owing a reply forever -- poll() never
        # fires POLLIN again and the sidecar is a permanent silent wedge.
        try:
            wire = json.dumps(reply).encode()
        except Exception as exc:
            wire = json.dumps({"ok": False,
                               "error": f"unserialisable reply: {exc}"}).encode()
        try:
            self._sock.send(wire)
        except Exception:
            pass

    def _dispatch(self, req: dict) -> dict:
        op = req.get("op")
        handler = {
            "ping": self._op_ping,
            "body_pose": self._op_body_pose,
            "episode": self._op_episode,
            "camera": self._op_camera,
            "contacts": self._op_contacts,
        }.get(op)
        if handler is None:
            return {"ok": False,
                    "error": f"op {op!r} is not implemented on the Isaac backend"}
        out = handler(req)
        out.setdefault("ok", True)
        # The MuJoCo sidecar stamps sim time on every reply; drift here would
        # be silent contract divergence. None when no snapshot has arrived --
        # an honest unknown, never a wall-clock stand-in.
        snap = self._fresh_snap()
        t = snap["sim_time"] if snap else None
        out["sim_time_s"] = round(t, 3) if t is not None else None
        return out

    # -- snapshot access, None-not-zero --

    def _fresh_snap(self):
        """The current snapshot, or None when there is none or the sim loop
        has stopped feeding us -- a stale snapshot served as current would be
        exactly the dead-sim-as-live-view lie the camera path refuses."""
        snap = self._snap
        if snap is None or time.time() - snap["wall_t"] > SNAPSHOT_STALE_S:
            return None
        return snap

    def _read_camera_frame(self, name: str, _retry: bool = True):
        """(header, jpeg bytes) from the sim's shm segment, or a string saying
        exactly why not. Plain file read -- see the module docstring.

        The writer stamps the header (fresh timestamp, new data_size) BEFORE
        copying the payload, so a read landing between the two can pair a
        fresh header with old or partial payload bytes. Two guards: the
        header is re-read after slicing and must be unchanged (any writer
        activity during our read changes the timestamp), and the payload must
        end in the JPEG EOI marker."""
        p = _SHM_DIR / f"isaac_{name}_image_shm"
        if not p.exists():
            return f"no camera segment {p.name} -- is the sim publishing images?"
        try:
            raw = p.read_bytes()
        except OSError as exc:
            return f"cannot read {p.name}: {exc}"
        hdr_size = ctypes.sizeof(_ShmImageHeader)
        if len(raw) < hdr_size:
            return f"{p.name} is truncated"
        h = _ShmImageHeader.from_buffer_copy(raw[:hdr_size])
        age_s = time.time() - h.timestamp / 1000.0
        if age_s > CAMERA_STALE_S:
            return (f"frame is {age_s:.1f}s old (limit {CAMERA_STALE_S}s) -- "
                    "the sim is dead or its image server has stalled; refusing "
                    "to serve a stale view as current")
        if h.encoding != 1:
            return (f"segment encoding={h.encoding} is not JPEG; the raw-BGR "
                    "path is not implemented here")
        if h.data_size <= 0 or hdr_size + h.data_size > len(raw):
            return f"{p.name} header claims impossible data_size={h.data_size}"
        jpeg = raw[hdr_size:hdr_size + h.data_size]
        torn = jpeg[-2:] != b"\xff\xd9"
        if not torn:
            try:
                h2 = _ShmImageHeader.from_buffer_copy(p.read_bytes()[:hdr_size])
                torn = (h2.timestamp != h.timestamp
                        or h2.data_size != h.data_size)
            except OSError:
                torn = True
        if torn:
            if _retry:
                time.sleep(0.03)              # let the writer finish its frame
                return self._read_camera_frame(name, _retry=False)
            return (f"{p.name} was mid-write on both read attempts -- "
                    "refusing a possibly-torn frame")
        return h, jpeg

    # -- ops --

    def _op_ping(self, req: dict) -> dict:
        """Advertise ONLY what is real RIGHT NOW. The previous stub reported
        cameras:['head_camera'] with no camera behind it; this one lists the
        segments the sim is actually writing, checked fresh per ping."""
        cameras = [n for n in ("head", "left", "right")
                   if not isinstance(self._read_camera_frame(n), str)]
        return {"protocol": PROTOCOL, "backend": "isaac",
                "cameras": cameras, "hand_actuators": 0,
                "render_available": bool(cameras),
                "ops": list(_IMPLEMENTED_OPS),
                "note": "scan (lidar), objects, hands, hand_state and band are "
                        "NOT implemented on this backend -- they answer "
                        "ok:false rather than a plausible zero."}

    def _op_body_pose(self, req: dict) -> dict:
        """Same reply shape as the MuJoCo sidecar's _op_body_pose -- probes
        read pelvis_m / pelvis_world_m / frame and must run unchanged. Served
        from the main-loop snapshot (<=SNAPSHOT_STALE_S old), never from live
        env tensors -- see update_snapshot for why.

        ground_contact lists robot bodies whose NET contact force exceeds the
        noise cutoff. Divergence from MuJoCo, stated: that backend counts only
        contact with the world (floor); this sensor counts contact with
        anything, table included. For "did the feet leave the floor" checks
        the difference errs strict, never permissive. None (not []) when the
        sensor is unavailable -- a floating robot and a missing sensor must
        never be indistinguishable.
        """
        names = req.get("bodies") or []
        if not names:
            raise ValueError("body_pose needs a 'bodies' list")
        snap = self._fresh_snap()
        if snap is None:
            return {"ok": False,
                    "error": "no fresh snapshot from the sim loop -- the sim "
                             "is not running or the sidecar is not wired"}
        p0 = snap["bodies"].get("pelvis")
        if p0 is None:
            return {"ok": False, "error": "no 'pelvis' body in the scene"}
        quat = snap["pelvis_quat_wxyz"]
        out = {}
        for name in names[:32]:
            w = snap["bodies"].get(name)
            if w is None:
                out[name] = None
                continue
            dv = [w[i] - p0[i] for i in range(3)]
            local = _to_body_frame(quat, dv)
            if local is None:      # untrustworthy pelvis quat: say so, whole reply
                return {"ok": False, "error": "pelvis orientation untrustworthy"}
            out[name] = {"world_m": [round(v, 4) for v in w],
                         "pelvis_m": [round(v, 4) for v in local]}
        rows = snap["contacts"]
        return {"ok": True, "bodies": out,
                "pelvis_world_m": [round(v, 4) for v in p0],
                "ground_contact": None if rows is None else [r[0] for r in rows],
                "frame": "pelvis_m is x forward, y left, z up, origin at pelvis"}

    def _op_camera(self, req: dict) -> dict:
        """A JPEG from one of the robot's own cameras, straight from the sim's
        shared-memory image server. Same reply shape as the MuJoCo sidecar."""
        name = req.get("camera") or "head"
        if bool(req.get("depth", False)):
            return {"ok": False,
                    "error": "depth is not available from the Isaac image "
                             "server (RGB JPEG only); do not guess distances"}
        if name not in ("head", "left", "right"):
            return {"ok": False,
                    "error": f"unknown camera {name!r}; this scene publishes "
                             "head, left, right"}
        frame = self._read_camera_frame(name)
        if isinstance(frame, str):
            return {"ok": False, "error": frame,
                    "note": "no camera frame is available; do not guess what "
                            "is in front of the robot"}
        h, jpeg = frame
        return {"camera": name, "kind": "jpeg",
                "width": int(h.width), "height": int(h.height),
                "jpeg_b64": base64.b64encode(jpeg).decode(),
                "source": "Isaac Sim onboard camera via the vendor image server"}

    def _op_contacts(self, req: dict) -> dict:
        """Which robot bodies carry contact force, and how much. Divergences
        from the MuJoCo reply, stated rather than papered over: this sensor
        reports NET force per body and cannot name the partner body
        (touching=None), and total_contacts counts loaded BODIES, not contact
        points."""
        snap = self._fresh_snap()
        if snap is None:
            return {"ok": False,
                    "error": "no fresh snapshot from the sim loop -- the sim "
                             "is not running or the sidecar is not wired"}
        rows = snap["contacts"]
        if rows is None:
            return {"ok": False,
                    "error": "no contact_forces sensor in this scene"}
        limit = int(req.get("limit", 12))
        return {"contacts": [{"robot_part": n, "touching": None,
                              "force_n": round(f, 1)} for n, f in rows[:limit]],
                "total_contacts": len(rows),
                "note": "net force per robot body in newtons; the Isaac sensor "
                        "cannot name the partner body (touching=None) and "
                        "counts contact with anything, not just the ground"}

    def _op_episode(self, req: dict) -> dict:
        """Honest until wired: nothing on this backend ever records a fall or
        a reset yet, so 'no falls' would be an answer no measurement backs.
        The op exists (SimClient callers get a well-formed refusal instead of
        'unknown op') but refuses rather than fabricating a zero."""
        return {"ok": False,
                "error": "fall/reset tracking is not wired on the Isaac "
                         "backend yet"}


if __name__ == "__main__":
    sidecar = IsaacSidecar()
    try:
        while True:
            sidecar.poll()
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
