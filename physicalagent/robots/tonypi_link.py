"""TonyLink — the seam between the TonyPi adapter and its JSON-RPC / MJPEG links.

The TonyPiAdapter never imports urllib. It talks to a TonyLink:
  * call(method, params) -> the parsed JSON-RPC `result`
  * snapshot()           -> the latest camera JPEG bytes

TonyPi speaks JSON-RPC 2.0 over HTTP on :9030 (reference/TonyPi/RPCServer.py) and
serves an MJPEG snapshot on :8080/?action=snapshot. The vendor methods return a
3-tuple `(ok, data, method_name)`, which crosses JSON as `[ok, data, name]` — use
rpc_result() to read (ok, data) out of it uniformly.

Production uses HttpTonyLink (urllib). Tests use FakeTonyLink, which records every
call and serves scripted results — so the adapter's clamping, whitelist and gate
logic run fully offline. This mirrors transport.FakeTransport's discipline.
"""
from __future__ import annotations

import json
import urllib.request
from abc import ABC, abstractmethod
from typing import Any


# JPEG frame markers, and how much of a stream to scan before giving up. The
# TonyPi's frames run ~31 kB, so 2 MB is many frames' worth of slack without
# letting a stream that never sends one grow without bound.
JPEG_SOI = b"\xff\xd8\xff"
JPEG_EOI = b"\xff\xd9"
FRAME_CHUNK_BYTES = 8192
MAX_FRAME_SCAN_BYTES = 2 * 1024 * 1024


def first_jpeg(buf: bytes) -> bytes | None:
    """The first COMPLETE JPEG in a multipart MJPEG body, or None.

    Pure, so the stream parsing is testable without a socket. Returns None
    rather than a partial frame: half a JPEG decodes to a grey band, and a
    corrupt image handed to a vision model is worse than no image, because the
    model will describe it rather than report it.
    """
    start = buf.find(JPEG_SOI)
    if start < 0:
        return None
    end = buf.find(JPEG_EOI, start + len(JPEG_SOI))
    if end < 0:
        return None
    return buf[start:end + len(JPEG_EOI)]


class TonyRpcError(Exception):
    """A JSON-RPC transport/protocol error (bad envelope, method not found).

    NOT the same as a vendor `[False, "E0x ...", name]` business failure — that
    is a normal result the adapter reads via rpc_result() and turns into a tool
    refusal. This exception is for the RPC layer itself failing.
    """


def rpc_result(parsed: Any) -> tuple[bool, Any]:
    """Read (ok, data) out of a vendor result.

    The TonyPi RPC methods return `(ok, data, method_name)` which arrives as a
    JSON array `[ok, data, name]`. A missing/None result (e.g. a dropped call)
    is treated as a failure — never assume success we cannot see.
    """
    if isinstance(parsed, (list, tuple)) and len(parsed) >= 2:
        return bool(parsed[0]), parsed[1]
    if isinstance(parsed, (list, tuple)) and len(parsed) == 1:
        return bool(parsed[0]), None
    return False, parsed


class TonyLink(ABC):
    @abstractmethod
    def call(self, method: str, params: list) -> Any:
        """Invoke a JSON-RPC method; return the parsed `result` member."""

    @abstractmethod
    def snapshot(self) -> bytes:
        """Latest camera frame as JPEG bytes."""


class FakeTonyLink(TonyLink):
    """In-memory link for tests. Records (method, params) calls and serves
    scripted results; snapshot() returns scripted bytes.

    `results` maps a method name to the result to return for it (default is a
    vendor-shaped `[True, [], method]` success). `snapshot` is the bytes every
    snapshot() returns.
    """

    def __init__(self, results: dict | None = None, snapshot: bytes = b""):
        self.calls: list[tuple[str, list]] = []
        self._results: dict[str, Any] = dict(results or {})
        self._snapshot = snapshot
        self.snapshots_taken = 0

    def call(self, method: str, params: list) -> Any:
        self.calls.append((method, list(params)))   # copy: caller may mutate
        if method in self._results:
            return self._results[method]
        return [True, [], method]                    # default vendor success

    def snapshot(self) -> bytes:
        self.snapshots_taken += 1
        return self._snapshot

    # -- test helpers -------------------------------------------------------
    def last_call(self, method: str) -> tuple[str, list] | None:
        for entry in reversed(self.calls):
            if entry[0] == method:
                return entry
        return None

    def set_result(self, method: str, result: Any) -> None:
        self._results[method] = result


class HttpTonyLink(TonyLink):
    """Production link over urllib. The opener is injected so the request shaping
    is unit-testable; the network itself is verified on hardware, never in CI.

    JSON-RPC POST goes to http://<host>:<rpc_port>; the camera snapshot is a GET
    from http://<host>:<video_port><snapshot_path>.
    """

    def __init__(self, host: str, rpc_port: int = 9030, video_port: int = 8080,
                 snapshot_path: str = "/?action=snapshot",
                 stream_path: str = "/?action=stream", timeout: float = 8.0,
                 opener=None):
        self._rpc_url = f"http://{host}:{rpc_port}"
        self._snap_url = f"http://{host}:{video_port}{snapshot_path}"
        self._stream_url = f"http://{host}:{video_port}{stream_path}"
        self._timeout = timeout
        self._opener = opener or urllib.request.urlopen
        self._id = 0

    def call(self, method: str, params: list) -> Any:
        self._id += 1
        body = json.dumps({
            "jsonrpc": "2.0", "method": method,
            "params": list(params), "id": self._id,
        }).encode("utf-8")
        req = urllib.request.Request(
            self._rpc_url, data=body,
            headers={"Content-Type": "application/json"})
        resp = json.loads(self._opener(req, timeout=self._timeout).read())
        if resp.get("error"):
            raise TonyRpcError(f"{method} JSON-RPC error: {resp['error']}")
        return resp.get("result")

    def snapshot(self) -> bytes:
        """One frame from `?action=snapshot`. MEASURED LIVE 2026-07-28.

        THE STREAM ENDPOINT IS THE FROZEN ONE. This was backwards for a day and
        the reversal is worth recording, because the original measurement was
        not wrong — it was taken on a wedged robot and then generalised.

        Measured on a healthy TonyPi, heartbeat fed, function 9 loaded, ten
        samples one second apart on each path:

            ?action=snapshot   10/10 DISTINCT   65443..66076 B
            ?action=stream      1/10 distinct   15986 B, byte-identical

        On 2026-07-27 the robot's RPC had wedged after a hung LoadFunc; in that
        state `?action=snapshot` served a frozen 31432 B frame, the stream served
        a different 15202 B one, and the difference was read as "the stream is a
        second, better source". It is not. It is a stale one. Power-cycling
        restored the snapshot path, and the stream stayed frozen.

        The lesson is the one this project keeps paying for: a measurement taken
        while the system is in a failed state describes the failure, not the
        system. Re-measure on a known-good robot before designing around it.

        WHAT THIS DOES NOT FIX, and must not be read as fixing: on that same
        session the camera appeared frozen on BOTH endpoints, and the reason is
        upstream of this function. Nobody has yet observed two frames from this
        robot that differ from each other. Until they do, treat TonyPi vision as
        unproven.

        CORRECTED 2026-07-28, from the vendor source in reference/TonyPi. An
        earlier version of this docstring read the diagnosis off GetRunningFunc
        returning [True,[0]] and concluded "nothing was running". That inference
        is VOID: RPCServer.py:424-426 is

            def GetRunningFunc():
                #return runbymainth("GetRunningFunc", ())
                return (True, (0,))

        a hardcoded stub with the real call commented out. It returns 0 always.
        Functions/Running.py:80 has a working getLoadedFunc(), but it is never
        registered with @dispatcher.add_method — so this robot exposes NO way to
        ask what it is running. Do not read running state off this RPC.

        Two real mechanisms replace that inference, neither yet confirmed on
        hardware because both leave their evidence on the Pi's stdout:

        (a) Running.loadFunc (lines 59-69) wraps FUNCTIONS[f].init() in
            `except Exception as e: print(e)` and then returns (True, ...)
            regardless. So LoadFunc genuinely can ack a load that threw — but
            the traceback goes to the Pi's console, not to us.
        (b) MjpgServer.py declares `global img_show` in do_GET and then, on the
            STREAM branch (line 33), assigns `img_show = None` — nulling the
            shared frame buffer that ?action=snapshot reads. Every stream
            connection blanks the snapshot source until the vision loop
            repopulates it. This function opens a stream connection, so it
            participates in that bug.

        Liveness still depends on a vision function being loaded AND its 7-second
        deadman fed (Functions/Running.py: heartbeatTask unloads the function and
        closes the camera when Heartbeat lapses). A frozen frame here means the
        keepalive stopped, not that this endpoint is wrong.
        """
        resp = self._opener(self._snap_url, timeout=self._timeout)
        data = b""
        try:
            # Read to EOF rather than trusting one read() to return everything:
            # urllib does, but a socket-backed file object need not, and a JPEG
            # truncated at a chunk boundary decodes to a grey band.
            while len(data) < MAX_FRAME_SCAN_BYTES:
                chunk = resp.read(FRAME_CHUNK_BYTES)
                if not chunk:
                    break
                data += chunk
        finally:
            close = getattr(resp, "close", None)
            if close:
                close()
        if not data.startswith(JPEG_SOI):
            raise OSError(
                f"{self._snap_url} returned {len(data)} B that are not a JPEG — "
                f"is a vision function loaded and the heartbeat being fed?")
        return data

    def stream_frame(self) -> bytes:
        """One frame off `?action=stream`, by reading until the first whole JPEG.

        KEPT, NOT USED. On this hardware the stream serves a stale frame (see
        snapshot()), so this is not the live path. It stays because the parsing
        is tested and because a different HiWonder model may well behave the
        other way round — MjpgServer.py is vendored per-robot.

        Reads only until the first complete JPEG, so an endless stream costs one
        frame rather than everything the socket will give.
        """
        resp = self._opener(self._stream_url, timeout=self._timeout)
        buf = b""
        try:
            while len(buf) < MAX_FRAME_SCAN_BYTES:
                chunk = resp.read(FRAME_CHUNK_BYTES)
                if not chunk:
                    break
                buf += chunk
                frame = first_jpeg(buf)
                if frame is not None:
                    return frame
        finally:
            close = getattr(resp, "close", None)
            if close:
                close()
        raise OSError(
            f"no frame in the first {len(buf)} bytes of {self._stream_url} — "
            f"is the camera running?")
