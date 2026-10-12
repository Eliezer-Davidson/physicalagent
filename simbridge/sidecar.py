"""Sidecar: answers questions about the running MuJoCo simulation, in-process.

WHAT IT IS FOR
    Three things a language model needs are properties of the WORLD, not of the
    robot, so no amount of Unitree DDS telemetry will ever carry them:

        camera   what the robot can see        -> needs the renderer
        scan     what is around the robot      -> needs the scene geometry
        objects  what those things are called  -> needs the model's names

    Everything the robot itself knows -- joints, IMU, odometry -- is read off
    rt/lowstate and rt/odostate instead (see physicalagent/robots/
    sonic_telemetry.py). Keeping that split honest matters: telemetry ports to
    hardware unchanged, and NOTHING in this file does. On a real G1 the camera
    is a RealSense and the scan is a Livox MID-360, and both would be replaced
    wholesale. This file is labelled simulation-only for that reason, and every
    answer it returns says where it came from.

WHY IT RUNS INSIDE THE SIMULATOR PROCESS
    mj_data is a live buffer being written at 200 Hz. Reading it from another
    thread yields torn state -- a position from one step and a velocity from the
    next -- which is exactly the kind of small lie that is impossible to debug
    later. So the socket is polled from sim_step, between mj_step calls, and
    every answer is a coherent snapshot of one simulation step.

    The cost is that a request stalls the simulation for as long as it takes to
    serve. A scan is microseconds. A render is 10-20 ms, i.e. two to four
    control steps, which shows up as a brief hitch and is the honest price of a
    coherent frame. `budget_ms` caps how long one poll may spend so a burst of
    requests cannot starve the controller.

WHY REQUEST/REPLY AND NOT A STREAM
    A stream would publish frames the model never asked for and drop the ones it
    did. REP also gives natural backpressure: the simulator can only ever be one
    request behind.

THE 500 ms RULE
    Serving from the loop means a slow op freezes the physics, and there is a
    hard deadline on that. SONIC drops control PERMANENTLY if rt/lowstate goes
    LOW_STATE_ABSENT_THRESHOLD = 500 ms stale (g1_deploy_onnx_ref.cpp:296,2773):
    it prints "Lost LowState data connection from robot!", stops the control
    loop, and does not come back. One long stall therefore costs the balance
    controller for the whole rest of the session, after which the robot falls,
    gets reset, and falls again indefinitely.

    Measured on this box: scan / objects / contacts are 5-10 ms and a
    steady-state render is 8.7 ms -- all comfortably clear. But CREATING a
    renderer costs 200-350 ms, 70% of the entire budget in one hit, and it used
    to be paid lazily on the model's first look(). That killed a live session at
    t=90 s and produced 89 resets over the following five minutes.

    So: renderers are built UP FRONT in the constructor, which runs on the first
    sim step, ~12 s before the policy finishes loading and starts watching. And
    the resolution is FIXED -- a free width/height parameter re-pays the
    construction cost on every new size, which is the same bug wearing a hat.
"""
from __future__ import annotations

import base64
import json
import time

import numpy as np

PROTOCOL = 1
# 5556/5557/5558 all belong to SONIC: 5556 is the planner input, 5557 is its
# realtime debug publisher and 5558 is a test sender. Binding 5557 first makes
# the policy abort on startup with "Address already in use" -- from a socket it
# never mentions in its own help -- so the sidecar sits well clear of the block.
DEFAULT_PORT = 5570

# How many times one scan ray may re-cast past the robot's own geometry before
# giving up. A ray leaving the pelvis sideways crosses shoulder, upper arm,
# forearm, hand and up to twelve finger links -- and each geom costs one cast.
MAX_SELF_HITS = 48

# The ONE camera size. Fixed rather than per-request: every distinct size needs
# its own renderer, and building one freezes the simulation for 200-350 ms --
# long enough to put SONIC's 500 ms lowstate watchdog in real danger. 448x336 is
# plenty for a vision model and costs ~4 ms to render once the context exists.
CAM_W, CAM_H = 448, 336
# Serving an op is dead time for the physics. Anything past this is logged, so a
# regression shows up in the sim log instead of as an unexplained fall.
SLOW_OP_WARN_MS = 60.0

# Bodies whose names start with these are the robot, not the world. Used to skip
# the robot's own geometry in a scan and to leave it out of the object list.
_ROBOT_HINTS = ("pelvis", "waist", "torso", "hip", "knee", "ankle", "shoulder",
                "elbow", "wrist", "hand", "thumb", "index", "middle", "ring",
                "pinky", "link", "head", "logo", "imu")

# mjGEOM_PLANE and mjGEOM_HFIELD. A plane's bounding box is not a fact about an
# object -- MuJoCo gives it half-extents of 1e10 -- and a height field is the
# terrain, not a thing standing in it. One unfiltered plane geom makes every
# object in the scene the size of the world.
#
# Integer literals rather than the mujoco enum because this module must import
# without mujoco present (mujoco is imported inside the methods that need it,
# and tests/test_sidecar_contract.py parses this file with ast); the geometry
# test pins both against mujoco.mjtGeom so a renumbering cannot pass silently.
_UNBOUNDED_GEOM_TYPES = (0, 1)


def body_extent(m, d, b: int):
    """The world-frame axis-aligned box body `b`'s geometry actually occupies.

    WHY THIS EXISTS. A MuJoCo body's origin is wherever the modeller put its
    frame, and that is near the object only by luck. Every piece of lounge
    furniture comes out of photo reconstruction carrying the SOURCE
    SCAN's origin -- measured on the live scene, all five bodies sit within
    2 cm of each other at z~0 while the things they represent are spread over
    1.8 m and stand up to 0.71 m off the floor. Reporting d.xpos put the
    thermos, which is on a table, at the robot's ankles, and made
    goto_object('thermos') and goto_object('bench') the same instruction.

    The box is the union over the body's geoms of each geom's own local AABB
    (m.geom_aabb, which MuJoCo computes for meshes too -- m.geom_size does not
    describe a mesh at all, and m.geom_rbound is a sphere that cannot tell a
    0.96 m-tall bench from a flat 1.4 m table). A rotated box's axis-aligned
    envelope has half-extents |R| @ h, which is conservative -- never smaller
    than the object -- and that is the correct direction to be wrong in for a
    number a robot walks towards.

    Covers this body's OWN geoms, not its children: _op_objects lists every
    body separately, so a child body gets its own row rather than being folded
    into its parent's box.

    Returns None when there is nothing to measure, so the caller can fall back
    to the origin AND SAY SO rather than invent an extent.
    """
    adr = int(m.body_geomadr[b])          # -1 on a body with no geoms
    num = int(m.body_geomnum[b])
    if adr < 0 or num <= 0:
        return None
    gs = slice(adr, adr + num)            # MuJoCo stores a body's geoms contiguously
    keep = ~np.isin(np.asarray(m.geom_type[gs]), _UNBOUNDED_GEOM_TYPES)
    if not keep.any():
        return None
    rot = np.asarray(d.geom_xmat[gs], dtype=np.float64).reshape(-1, 3, 3)[keep]
    aabb = np.asarray(m.geom_aabb[gs], dtype=np.float64)[keep]
    pos = np.asarray(d.geom_xpos[gs], dtype=np.float64)[keep]
    centres = pos + np.einsum("nij,nj->ni", rot, aabb[:, :3])
    halves = np.einsum("nij,nj->ni", np.abs(rot), aabb[:, 3:])
    lo = (centres - halves).min(axis=0)
    hi = (centres + halves).max(axis=0)
    return {"lo": lo, "hi": hi, "centre": (lo + hi) / 2.0, "size": hi - lo}


def footprint_range_m(lo, hi, rx: float, ry: float) -> float:
    """Horizontal distance from (rx, ry) to the nearest point of the box's
    footprint; 0.0 when the robot is standing over it.

    This answers 'how far before I walk into it', which the centre-to-centre
    range does not: measured on the live lounge, the bench's centre is 2.46 m
    away and its near edge is 1.81 m. Reported alongside range_m rather than
    instead of it -- a nearest-surface point MOVES as the robot walks, and a
    moving reference point would break the constant world_xy that the frame
    bridge in sonic_world.augment_objects depends on and that is the one part
    of this chain validated in flight.
    """
    dx = max(float(lo[0]) - rx, 0.0, rx - float(hi[0]))
    dy = max(float(lo[1]) - ry, 0.0, ry - float(hi[1]))
    return float(np.hypot(dx, dy))


class Sidecar:
    """Serves scene questions from inside the simulation loop.

    Constructed with the env (which owns mj_model/mj_data) rather than with the
    raw arrays, because a reset swaps mj_data out from under a saved reference
    and the stale one keeps answering plausibly forever.
    """

    def __init__(self, env, port: int = DEFAULT_PORT, budget_ms: float = 40.0):
        import zmq

        self._env = env
        self._budget_s = budget_ms / 1000.0
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.REP)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.bind(f"tcp://127.0.0.1:{port}")
        self._poller = zmq.Poller()
        self._poller.register(self._sock, zmq.POLLIN)
        self._renderer = None
        self._render_failed: str | None = None
        self._robot_body_ids: set[int] | None = None
        # Fall/reset bookkeeping. check_fall() calls reset() when the pelvis
        # drops below 0.2 m, which sets mj_data.time back to zero and stands the
        # robot back up. Nothing on any other channel records that this
        # happened, so a status read taken afterwards describes a FRESH robot
        # and is indistinguishable from a manoeuvre that worked. Observed live:
        # an untested mode dropped the robot at t=44 s, the sim reset it, and the
        # model reported "remains fully balanced and upright" from the respawn.
        self._last_sim_time = float(self._env.mj_data.time)
        self._resets = 0
        self._fell_since_check = False
        print(f"[sidecar] listening on tcp://127.0.0.1:{port}", flush=True)
        self._prewarm()

    def _prewarm(self) -> None:
        """Build the renderer NOW, while stalling costs nothing.

        This constructor runs on the simulator's first step, roughly 12 s before
        the policy finishes loading its TensorRT engine and starts enforcing the
        500 ms lowstate watchdog. Doing it lazily instead -- on the model's first
        look() -- puts a 350 ms freeze in the middle of a balancing robot, and
        that is what took control down in a live session.
        """
        import time as _t

        t0 = _t.perf_counter()
        try:
            import mujoco

            self._renderer = mujoco.Renderer(self._env.mj_model,
                                             height=CAM_H, width=CAM_W)
            # Render once too: the first render after construction also carries
            # one-off cost, and it is no cheaper for being deferred.
            self._renderer.update_scene(self._env.mj_data, camera=self._camera_name())
            pixels = self._renderer.render()
            # ...and encode once. cv2's first imencode initialises its codec and
            # cost 168 ms on the first live look() even after the renderer was
            # already warm -- the same bug one layer down.
            import cv2

            cv2.imencode(".jpg", np.asarray(pixels)[:, :, ::-1],
                         [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            print(f"[sidecar] camera ready ({CAM_W}x{CAM_H}, "
                  f"{(_t.perf_counter() - t0) * 1000:.0f} ms warm-up)", flush=True)
        except Exception as exc:
            self._renderer = None
            self._render_failed = f"renderer unavailable: {exc}"
            print(f"[sidecar] camera disabled: {exc}", flush=True)

    def _camera_name(self) -> str:
        m = self._env.mj_model
        names = [m.camera(i).name for i in range(m.ncam)]
        return "head_camera" if "head_camera" in names else (names[0] if names else "")

    # ---- called from the simulation loop ---------------------------------
    def poll(self) -> None:
        """Serve whatever is waiting, then return. Never raises into the sim
        loop: a broken sidecar must not be able to stop the robot."""
        self._watch_for_reset()
        deadline = time.monotonic() + self._budget_s
        while True:
            try:
                if not self._poller.poll(0):
                    return
                raw = self._sock.recv()
            except Exception:
                return
            t0 = time.perf_counter()
            op = "?"
            try:
                req = json.loads(raw.decode())
                op = req.get("op", "?")
                reply = self._dispatch(req)
            except Exception as exc:
                reply = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
            # Serving is dead time for the physics, and 500 ms of it kills the
            # balance controller for good. Log anything slow so a regression is
            # visible in the sim log rather than as an unexplained fall.
            took_ms = (time.perf_counter() - t0) * 1000.0
            if took_ms > SLOW_OP_WARN_MS:
                print(f"[sidecar] SLOW op {op}: {took_ms:.0f} ms of frozen "
                      f"simulation (watchdog trips at 500 ms)", flush=True)
            try:
                self._sock.send(json.dumps(reply).encode())
            except Exception:
                return
            if time.monotonic() > deadline:
                return

    def close(self) -> None:
        try:
            self._sock.close(linger=0)
        except Exception:
            pass

    # ---- dispatch --------------------------------------------------------
    def _dispatch(self, req: dict) -> dict:
        op = req.get("op")
        handler = {
            "ping": self._op_ping,
            "camera": self._op_camera,
            "scan": self._op_scan,
            "objects": self._op_objects,
            "contacts": self._op_contacts,
            "hands": self._op_hands,
            "hand_state": self._op_hand_state,
            "episode": self._op_episode,
            "band": self._op_band,
            "body_pose": self._op_body_pose,
        }.get(op)
        if handler is None:
            raise ValueError(f"unknown op {op!r}")
        out = handler(req)
        out.setdefault("ok", True)
        out["sim_time_s"] = round(float(self._env.mj_data.time), 3)
        return out

    # ---- ops -------------------------------------------------------------
    def _op_ping(self, req: dict) -> dict:
        m = self._env.mj_model
        return {"protocol": PROTOCOL, "nq": int(m.nq), "nu": int(m.nu),
                "nbody": int(m.nbody),
                "cameras": [m.camera(i).name for i in range(m.ncam)],
                "hand_actuators": len(self._hand_actuator_ids()),
                "render_available": self._render_failed is None and self._renderer is not None}

    def _op_body_pose(self, req: dict) -> dict:
        """Where named bodies actually are, both in world and relative to the
        pelvis.

        This is MuJoCo's own forward kinematics, and it exists to CHECK ours:
        physicalagent.robots.sonic_arm_kin recomputes the arm chain in pure
        python from the MJCF, and the only way to know that reimplementation is
        right is to compare it against the simulator that owns the model. It is
        a verification channel, not something the model needs -- on hardware
        there is no such oracle, which is exactly why our own FK has to be
        correct rather than convenient.
        """
        names = req.get("bodies") or []
        if not names:
            raise ValueError("body_pose needs a 'bodies' list")
        m, d = self._env.mj_model, self._env.mj_data
        pelvis = d.body("pelvis")
        p0 = [float(v) for v in pelvis.xpos]
        # world -> pelvis frame: R^T (p - p0), with R row-major 3x3 in xmat
        r = [float(v) for v in pelvis.xmat]
        out = {}
        for name in names[:32]:
            try:
                b = d.body(name)
            except (KeyError, ValueError):
                out[name] = None
                continue
            w = [float(v) for v in b.xpos]
            dv = [w[i] - p0[i] for i in range(3)]
            local = [sum(r[k * 3 + i] * dv[k] for k in range(3)) for i in range(3)]
            out[name] = {"world_m": [round(v, 4) for v in w],
                         "pelvis_m": [round(v, 4) for v in local]}
        return {"bodies": out,
                "pelvis_world_m": [round(v, 4) for v in p0],
                "ground_contact": self._ground_contact(),
                "frame": "pelvis_m is x forward, y left, z up, origin at pelvis"}

    def _ground_contact(self) -> list:
        """Which robot bodies are touching the world RIGHT NOW.

        Returned alongside body poses because the question "did both feet leave
        the floor" cannot be answered from body heights. The lowest body on each
        leg is `*_ankle_roll_link`, whose origin is the ankle JOINT, not the
        sole -- so lifting a heel while the toe stays planted raises it by
        several centimetres with the foot still firmly on the ground. Inferring
        flight from that height reads a deep crouch as a jump.

        MuJoCo already knows the answer. `ncon` against the world body is the
        ground truth, and it costs one pass over the contact list.
        """
        m, d = self._env.mj_model, self._env.mj_data
        robot_bodies = self._robot_bodies()
        touching = set()
        for i in range(d.ncon):
            con = d.contact[i]
            b1 = int(m.geom_bodyid[con.geom1])
            b2 = int(m.geom_bodyid[con.geom2])
            in1, in2 = b1 in robot_bodies, b2 in robot_bodies
            if in1 == in2:
                continue
            rb = b1 if in1 else b2
            touching.add(m.body(rb).name or f"body_{rb}")
        return sorted(touching)

    def _op_camera(self, req: dict) -> dict:
        """A JPEG from one of the model's cameras, rendered from the CURRENT
        simulation state. This is the robot's own view, not a screenshot of the
        viewer window -- the viewer tracks the pelvis from outside."""
        import mujoco

        name = req.get("camera") or self._camera_name()
        quality = int(req.get("quality", 80))
        depth = bool(req.get("depth", False))

        if self._render_failed or self._renderer is None:
            return {"ok": False,
                    "error": self._render_failed or "no renderer",
                    "note": "no camera frame is available; do not guess what is "
                            "in front of the robot"}
        # Deliberately NOT per-request: width/height are fixed at CAM_W/CAM_H so
        # a request can never trigger a 350 ms renderer build inside the loop.
        renderer = self._renderer
        width, height = CAM_W, CAM_H

        if depth:
            renderer.enable_depth_rendering()
        try:
            renderer.update_scene(self._env.mj_data, camera=name)
            pixels = renderer.render()
        finally:
            if depth:
                renderer.disable_depth_rendering()

        if depth:
            # Metric depth, downsampled to a grid a model can actually reason
            # over. A 640x480 float array is not something to put in a prompt.
            d = np.asarray(pixels, dtype=np.float32)
            d = np.where(np.isfinite(d), d, np.inf)
            rows, cols = 3, 5
            gh, gw = d.shape[0] // rows, d.shape[1] // cols
            grid = [[round(float(np.median(d[r * gh:(r + 1) * gh,
                                            c * gw:(c + 1) * gw])), 2)
                     for c in range(cols)] for r in range(rows)]
            return {"camera": name, "kind": "depth_m", "grid_rows_top_to_bottom": grid,
                    "nearest_m": round(float(d.min()), 2),
                    "source": "simulated depth from the MuJoCo camera"}

        import cv2
        bgr = np.asarray(pixels)[:, :, ::-1]
        ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if not ok:
            raise RuntimeError("JPEG encode failed")
        return {"camera": name, "kind": "jpeg",
                "width": width, "height": height,
                "jpeg_b64": base64.b64encode(buf.tobytes()).decode(),
                "source": "simulated RGB from the MuJoCo camera"}

    def _op_scan(self, req: dict) -> dict:
        """A horizontal ring of ray casts around the robot -- a simulated planar
        lidar.

        THIS IS NOT THE G1'S LIDAR. The physical G1 carries a Livox MID-360 and
        this MuJoCo model has no lidar sensor of any kind -- no rangefinder, no
        replicated sensor, nothing. What this does is cast mj_ray through the
        actual scene geometry, which is a real measurement of the simulated
        world and a fair stand-in for a planar scan. The name of the sensor it
        is standing in for is not claimed anywhere in the reply.

        Rays start at the pelvis, so the first thing most of them hit is the
        robot's own arm or leg. Those hits are skipped and the ray is re-cast
        from just past them, up to a few times -- otherwise every scan reports
        an obstacle at 15 cm in every direction, which is true and useless.
        """
        import mujoco

        beams = max(4, min(180, int(req.get("beams", 36))))
        max_range = float(req.get("max_range", 10.0))
        z_offset = float(req.get("z_offset", 0.2))   # above the pelvis
        d = self._env.mj_data
        origin0 = np.array([d.qpos[0], d.qpos[1], d.qpos[2] + z_offset], dtype=np.float64)
        heading = self._heading_rad()

        robot_bodies = self._robot_bodies()
        geomid = np.zeros(1, dtype=np.int32)
        ranges, names = [], []
        for k in range(beams):
            rel = 2.0 * np.pi * k / beams          # 0 = straight ahead
            ang = heading + rel
            vec = np.array([np.cos(ang), np.sin(ang), 0.0], dtype=np.float64)
            origin = origin0.copy()
            travelled = 0.0
            hit_name = None
            exhausted = True
            # A ray leaving the pelvis crosses a LOT of our own geometry: upper
            # arm, forearm, hand and then up to twelve finger links, each a
            # separate geom needing its own re-cast. Six was not close; running
            # out used to report the accumulated distance as a nameless
            # obstacle at 0.12 m, which is the worst possible wrong answer for a
            # collision-avoidance tool -- it reads as a wall in your face.
            for _ in range(MAX_SELF_HITS):
                dist = mujoco.mj_ray(self._env.mj_model, d, origin, vec,
                                     None, 1, -1, geomid)
                if dist < 0 or geomid[0] < 0:
                    travelled = float("inf")        # clear line of sight
                    exhausted = False
                    break
                body = int(self._env.mj_model.geom_bodyid[geomid[0]])
                travelled += float(dist)
                if body not in robot_bodies:
                    hit_name = self._env.mj_model.body(body).name
                    exhausted = False
                    break
                origin = origin + vec * (dist + 1e-3)
                travelled += 1e-3
            if exhausted:
                # Still inside our own body after the budget. Unknown, and said
                # so -- never a number.
                ranges.append(None)
                names.append("(unknown: ray never left the robot)")
            elif travelled > max_range:
                ranges.append(None)                 # nothing within max_range
                names.append(None)
            else:
                ranges.append(round(travelled, 2))
                names.append(hit_name)

        finite = [(r, names[i], 360.0 * i / beams)
                  for i, r in enumerate(ranges) if r is not None]
        nearest = min(finite, default=None)
        return {
            "kind": "planar_scan",
            "source": "simulated: mj_ray through the MuJoCo scene. This is NOT "
                      "the G1's Livox lidar; this model has no lidar sensor.",
            "beams": beams,
            "max_range_m": max_range,
            "bearing_deg_is_relative_to": "the robot's forward direction, "
                                          "increasing to the left",
            "ranges_m": ranges,
            "hit": names,
            "nearest": None if nearest is None else {
                "range_m": nearest[0], "what": nearest[1],
                "bearing_deg": round(nearest[2], 1)},
        }

    def _op_objects(self, req: dict) -> dict:
        """Everything in the scene that is not part of the robot, with where it
        is relative to the robot. Names come from the MJCF, so they are the
        modeller's names and can be unhelpful -- reported verbatim rather than
        prettied up, because a renamed object cannot be found again.

        WHERE an object is means where its GEOMETRY is, not where its body
        frame sits; see body_extent(). This used to report d.xpos and told the
        model that a thermos standing on a table was at its ankles, and that
        the bench, the table, one laptop and the thermos were all the same
        place -- so goto_object() aimed every one of them at the same world
        point. Nothing downstream wanted the origin: the raw body frame is
        still available, exactly and unrounded, from the body_pose op, which is
        where a verification channel belongs.

        NOT YET VALIDATED LIVE. The change is proven by
        tests/test_sidecar_geometry.py against a real MuJoCo model, and the
        lounge numbers quoted above were measured off the live scene XML with
        mj_forward on CPU -- but no run of the simulator has served this reply
        to the model. What the model does with a correct scene is unknown.
        """
        m, d = self._env.mj_model, self._env.mj_data
        robot_bodies = self._robot_bodies()
        rx, ry, rz = float(d.qpos[0]), float(d.qpos[1]), float(d.qpos[2])
        heading = self._heading_rad()
        out = []
        for b in range(1, m.nbody):                 # 0 is the world body
            if b in robot_bodies:
                continue
            name = m.body(b).name or f"body_{b}"
            box = body_extent(m, d, b)
            if box is None:
                # Nothing to measure. Fall back to the origin and say so ON THE
                # OBJECT -- a silent fallback here is the exact bug this
                # function exists to remove.
                centre = np.asarray(d.xpos[b], dtype=np.float64)
                row = {"name": name,
                       "measured_from":
                           "body_origin — this body has no bounding geometry"}
            else:
                centre = box["centre"]
                row = {"name": name, "measured_from": "bounding_box_centre"}
            dx, dy = float(centre[0]) - rx, float(centre[1]) - ry
            bearing = float(np.degrees(np.arctan2(dy, dx) - heading))
            row.update(range_m=round(float(np.hypot(dx, dy)), 2),
                       bearing_deg=round((bearing + 180) % 360 - 180, 1),
                       height_m=round(float(centre[2]) - rz, 2))
            if box is not None:
                # Omitted, not null, on the fallback path: a top_m that merely
                # equalled height_m would read as a measurement of a flat thing.
                row["top_m"] = round(float(box["hi"][2]) - rz, 2)
                row["size_m"] = [round(float(v), 2) for v in box["size"]]
                row["nearest_edge_m"] = round(
                    footprint_range_m(box["lo"], box["hi"], rx, ry), 2)
            out.append(row)
        # Sorted on centre range, the same number that is displayed. Sorting on
        # nearest_edge_m while showing range_m makes the list look unsorted.
        out.sort(key=lambda o: o["range_m"])
        return {"objects": out[:int(req.get("limit", 20))],
                "total": len(out),
                "note": "range_m, bearing_deg and height_m are to the CENTRE of "
                        "the box the object occupies, not to its model origin. "
                        "bearing 0 is straight ahead, positive to the left. "
                        "height_m and top_m are relative to the robot's pelvis, "
                        "so top_m near 0 is about waist height and a large "
                        "negative height_m is near the floor. nearest_edge_m is "
                        "how far the robot is from walking into it. size_m is "
                        "the world-axis-aligned box the object occupies — for a "
                        "thing standing at an angle that is wider than the "
                        "object itself."}

    def _op_contacts(self, req: dict) -> dict:
        """What is currently touching the robot, and how hard. This is the only
        honest way to answer 'am I holding it' -- a gripper closed around
        nothing looks identical to one closed around a can, from joint angles
        alone."""
        import mujoco

        m, d = self._env.mj_model, self._env.mj_data
        robot_bodies = self._robot_bodies()
        seen: dict[tuple, float] = {}
        force = np.zeros(6, dtype=np.float64)
        for i in range(d.ncon):
            con = d.contact[i]
            b1 = int(m.geom_bodyid[con.geom1])
            b2 = int(m.geom_bodyid[con.geom2])
            in1, in2 = b1 in robot_bodies, b2 in robot_bodies
            if in1 == in2:                          # self-contact or world-world
                continue
            mujoco.mj_contactForce(m, d, i, force)
            mag = float(np.linalg.norm(force[:3]))
            rb, ob = (b1, b2) if in1 else (b2, b1)
            key = (m.body(rb).name or f"body_{rb}", m.body(ob).name or f"body_{ob}")
            seen[key] = seen.get(key, 0.0) + mag
        rows = [{"robot_part": k[0], "touching": k[1], "force_n": round(v, 1)}
                for k, v in sorted(seen.items(), key=lambda kv: -kv[1])]
        return {"contacts": rows[:int(req.get("limit", 12))],
                "total_contacts": d.ncon,
                "note": "force is the contact normal+friction magnitude in newtons"}

    def _op_hands(self, req: dict) -> dict:
        """Set finger joint targets, if this scene has finger actuators.

        Refuses loudly when it does not. A hand tool that silently succeeds
        against rigid plastic hands is worse than no hand tool: the model would
        report a grasp that never happened."""
        ids = self._hand_actuator_ids()
        if not ids:
            return {"ok": False,
                    "error": "this scene's hands have no actuators — they are "
                             "rigid geometry. Nothing can be commanded.",
                    "scene_hint": "launch with SONIC_HANDS=actuated for a scene "
                                  "whose Inspire fingers are actuated"}
        targets = self._env.sidecar_finger_targets
        applied = {}
        for side in ("left", "right"):
            spec = req.get(side)
            if spec is None:
                continue
            applied[side] = self._apply_hand(side, spec, ids, targets)
        # Named "targets_commanded", never "joints". The first live run had the
        # model read this echo back and report that "these joint angles confirm
        # the fingers have moved" -- they confirm nothing of the sort, they are
        # the numbers it just sent. Measuring the hand is hand_state's job.
        return {"targets_commanded": applied,
                "note": "these are TARGETS, not measurements — the fingers have "
                        "not moved yet. Targets are held until changed and the "
                        "simulator runs a PD law onto them every step; call "
                        "hand_state after a moment to see where they got to."}

    def _op_hand_state(self, req: dict) -> dict:
        """Where the fingers actually ARE, and how far that is from where they
        were told to be. The gap is the interesting number: a finger stalled far
        from its target is one pressing against something."""
        ids = self._hand_actuator_ids()
        if not ids:
            return {"ok": False,
                    "error": "this scene's hands have no actuators — there are "
                             "no finger angles to read"}
        m, d = self._env.mj_model, self._env.mj_data
        targets = getattr(self._env, "sidecar_finger_targets", {})
        out: dict = {}
        stalled = []
        for side, side_ids in ids.items():
            rows = {}
            for name, a in side_ids.items():
                j = int(m.actuator_trnid[a][0])
                actual = float(d.qpos[int(m.jnt_qposadr[j])])
                row = {"angle_rad": round(actual, 3)}
                if a in targets:
                    err = targets[a] - actual
                    row["target_rad"] = round(float(targets[a]), 3)
                    row["error_rad"] = round(err, 3)
                    if abs(err) > 0.15:
                        stalled.append(name)
                rows[name] = row
            out[side] = rows
        return {"hands": out, "stalled_short_of_target": stalled,
                "note": "a finger stopped well short of its target is being "
                        "blocked by something — check contacts to see by what"}

    def _op_band(self, req: dict) -> dict:
        """Read or drive the elastic band. SIM ONLY — the band does not exist on
        hardware.

        `engage`/`release` toggle the spring; `length_m` sets its length
        offset (hang pelvis height ~= 0.97 + length, so -0.2 rests the feet on
        the floor). These exist because the only other path is a synthesized
        keypress into the viewer window, which silently fails whenever the
        compositor keeps focus elsewhere — the actual anatomy of the "band
        release fails on half of launches" flake. A deterministic bring-up is:
        set length_m=-0.2, wait for the feet to touch, then release — a
        zero-height drop instead of a 0.2 m fall the policy must catch.

        The band anchors at world (0,0,1) with kp=10000: engaging it with the
        robot displaced from the origin applies a catapult-scale force (it has
        crashed the simulator with a zero-norm quaternion). Refused beyond
        0.5 m for that reason.
        """
        band = getattr(self._env, "elastic_band", None)
        if band is None:
            return {"ok": False, "error": "no elastic band in this scene"}
        if req.get("engage"):
            pos = self._env.mj_data.body("pelvis").xpos
            r = float(pos[0] ** 2 + pos[1] ** 2) ** 0.5
            if r > 0.5:
                return {"ok": False, "band_enabled": bool(band.enable),
                        "error": f"robot is {r:.2f} m from the band anchor at the "
                                 "origin; engaging would catapult it (kp=10000). "
                                 "Restart the simulator instead."}
            band.enable = True
        if req.get("release"):
            band.enable = False
        if req.get("length_m") is not None:
            band.length = max(-0.5, min(0.5, float(req["length_m"])))
        return {"band_enabled": bool(band.enable),
                "length_m": float(getattr(band, "length", 0.0))}

    def _op_episode(self, req: dict) -> dict:
        """Has the robot fallen since you last asked?

        Reading this CLEARS the latch, so each fall is reported exactly once, to
        whoever asks first. That is deliberate: the alternative -- a sticky flag
        -- means one early fall poisons every later status read and the model
        stops believing it.
        """
        fell = self._fell_since_check
        self._fell_since_check = False
        return {"fell_and_was_reset_since_last_check": fell,
                "falls_this_session": self._resets,
                "note": ("THE ROBOT FELL and the simulator stood it back up at "
                         "the origin. Anything you measure now describes the "
                         "reset robot, not the result of what you just did — the "
                         "manoeuvre FAILED." if fell else
                         "no fall since the last check")}

    # ---- helpers ---------------------------------------------------------
    def _watch_for_reset(self) -> None:
        """mj_data.time running backwards is the only signal a reset happened.
        Checked every step from poll() rather than on demand, because by the
        time anyone asks, the clock has already run forward again from zero and
        the evidence is gone."""
        now = float(self._env.mj_data.time)
        if now < self._last_sim_time - 1e-9:
            self._resets += 1
            self._fell_since_check = True
        self._last_sim_time = now

    def _heading_rad(self) -> float:
        q = self._env.mj_data.qpos[3:7]             # w, x, y, z
        return float(np.arctan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
                                1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2)))

    def _robot_bodies(self) -> set[int]:
        """Body ids belonging to the robot. Computed once: the model does not
        change, and doing this per ray cast would dominate a scan."""
        if self._robot_body_ids is None:
            m = self._env.mj_model
            ids = set()
            for b in range(m.nbody):
                name = (m.body(b).name or "").lower()
                if any(h in name for h in _ROBOT_HINTS):
                    ids.add(b)
            # Anything descending from a known robot body is also the robot,
            # whatever it is called.
            for b in range(1, m.nbody):
                parent = int(m.body_parentid[b])
                if parent in ids:
                    ids.add(b)
            self._robot_body_ids = ids
        return self._robot_body_ids

    def _hand_actuator_ids(self) -> dict:
        """Finger actuators by side, discovered by name. Empty on a rigid-hand
        scene, which is the signal _op_hands refuses on."""
        cached = getattr(self, "_hand_ids_cache", None)
        if cached is not None:
            return cached
        m = self._env.mj_model
        ids: dict = {"left": {}, "right": {}}
        for a in range(m.nu):
            name = m.actuator(a).name or ""
            if name.startswith("L_"):
                ids["left"][name] = a
            elif name.startswith("R_"):
                ids["right"][name] = a
        ids = {k: v for k, v in ids.items() if v}
        self._hand_ids_cache = ids
        return ids

    def _apply_hand(self, side: str, spec, ids: dict, targets: dict) -> dict:
        """spec is either a scalar closure in [0,1] or a per-joint mapping."""
        side_ids = ids.get(side, {})
        if isinstance(spec, dict):
            wrote = {}
            for jname, value in spec.items():
                a = side_ids.get(jname)
                if a is None:
                    continue
                targets[a] = float(value)
                wrote[jname] = float(value)
            return {"joints": wrote}
        closure = max(0.0, min(1.0, float(spec)))
        m = self._env.mj_model
        for name, a in side_ids.items():
            j = int(m.actuator_trnid[a][0])
            lo, hi = m.jnt_range[j]
            # Closure 0 is the joint's open end, 1 is its closed end. The Inspire
            # ranges all run open->closed in the positive direction.
            targets[a] = float(lo + (hi - lo) * closure)
        return {"closure": closure, "joints_set": len(side_ids)}
