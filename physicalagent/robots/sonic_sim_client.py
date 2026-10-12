"""Client for the in-simulator sidecar (simbridge/sidecar.py).

Everything here is SIMULATION ONLY and says so in its results. The split is
deliberate and worth keeping: sonic_telemetry.py reads the robot over the
Unitree DDS topics and would work unchanged on hardware; this file reads the
simulated world and would not exist there. Mixing them would produce a tool
surface that silently stops working the day the robot is real.

A request that cannot be served returns a refusal the model can read. It never
returns a placeholder frame, an empty scan or a plausible zero -- a model shown
a black image will reason about the darkness.
"""
from __future__ import annotations

import base64
import json

DEFAULT_PORT = 5570


class SidecarUnavailable(RuntimeError):
    """The simulator is not answering. Distinct from a refusal: a refusal means
    the sim said no, this means nothing said anything."""


class SimClient:
    def __init__(self, *, port: int = DEFAULT_PORT, host: str = "127.0.0.1",
                 timeout_ms: int = 3000):
        import zmq

        self._zmq = zmq
        self._addr = f"tcp://{host}:{port}"
        self._timeout_ms = timeout_ms
        self._ctx = zmq.Context.instance()
        self._sock = None
        self._connect()

    def _connect(self):
        if self._sock is not None:
            self._sock.close(linger=0)
        self._sock = self._ctx.socket(self._zmq.REQ)
        self._sock.setsockopt(self._zmq.LINGER, 0)
        self._sock.setsockopt(self._zmq.RCVTIMEO, self._timeout_ms)
        self._sock.setsockopt(self._zmq.SNDTIMEO, self._timeout_ms)
        self._sock.connect(self._addr)

    def request(self, op: str, **kwargs) -> dict:
        """One round trip. On timeout the socket is rebuilt before raising:
        a REQ socket that timed out is stuck in the wrong state and every
        later call on it fails too, which reads as 'the simulator died' when
        it was one slow render."""
        try:
            self._sock.send(json.dumps({"op": op, **kwargs}).encode())
            return json.loads(self._sock.recv().decode())
        except Exception as exc:
            self._connect()
            raise SidecarUnavailable(
                f"the simulator did not answer ({type(exc).__name__}). "
                f"Is it running with SONIC_SIDECAR_PORT set?") from exc

    def close(self):
        if self._sock is not None:
            self._sock.close(linger=0)
            self._sock = None

    # -- typed helpers -----------------------------------------------------
    def ping(self) -> dict:
        return self.request("ping")

    def frame_jpeg(self, camera: str = "head_camera") -> bytes:
        """No size parameter, deliberately. The sidecar renders at ONE fixed
        resolution because every distinct size needs its own renderer, and
        building one freezes the simulation for 200-350 ms -- close enough to
        SONIC's 500 ms lowstate watchdog to take the balance controller down."""
        r = self.request("camera", camera=camera)
        if not r.get("ok"):
            raise RuntimeError(r.get("error", "camera unavailable"))
        return base64.b64decode(r["jpeg_b64"])

    def depth(self, camera: str = "head_camera") -> dict:
        return self.request("camera", camera=camera, depth=True)

    def scan(self, beams: int = 36, max_range: float = 10.0) -> dict:
        return self.request("scan", beams=beams, max_range=max_range)

    def objects(self, limit: int = 20) -> dict:
        return self.request("objects", limit=limit)

    def contacts(self, limit: int = 12) -> dict:
        return self.request("contacts", limit=limit)

    def set_hands(self, left=None, right=None) -> dict:
        payload = {}
        if left is not None:
            payload["left"] = left
        if right is not None:
            payload["right"] = right
        if not payload:
            raise ValueError("set_hands needs at least one of left/right")
        return self.request("hands", **payload)

    def hand_state(self) -> dict:
        return self.request("hand_state")

    def body_pose(self, bodies) -> dict:
        """MuJoCo's own body positions, world and pelvis-relative.

        A verification channel: it is how sonic_arm_kin's pure-python FK gets
        checked against the simulator that owns the model. Not part of the tool
        surface -- hardware has no equivalent oracle.
        """
        return self.request("body_pose", bodies=list(bodies))

    def episode(self) -> dict:
        """Whether the robot has fallen since this was last called. Reading it
        clears the latch, so call it from ONE place -- get_status -- or falls
        will be consumed by whichever caller happens to get there first."""
        return self.request("episode")

    def release_band(self) -> dict:
        return self.request("band", release=True)

    def band(self, **kw) -> dict:
        """Read or drive the elastic band directly (engage / release /
        length_m). SIM ONLY. Exposed because release alone drops the robot
        ~17 cm onto a live policy: the band holds the pelvis at ~0.96 m and it
        stands free at ~0.79 m. Setting length_m=-0.2 first rests the feet on
        the floor, turning the hand-off into a zero-height release."""
        return self.request("band", **kw)
