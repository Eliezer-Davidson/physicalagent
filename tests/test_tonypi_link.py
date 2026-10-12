"""Tests for the TonyPi JSON-RPC link seam — offline, no hardware, no network.

TonyLink is to the TonyPiAdapter what Transport is to the ROSPug adapter: the
one seam the adapter talks through, with a recording fake for tests and a real
urllib impl for hardware. FakeTonyLink records every (method, params) call and
serves scripted results, mirroring FakeTransport's discipline. HttpTonyLink is
exercised here only through an INJECTED opener (never a real socket) — hardware
behaviour is verified on the robot, not in this suite.
"""
import json

import pytest

from physicalagent.robots.tonypi_link import (
    FakeTonyLink, HttpTonyLink, TonyLink, TonyRpcError, rpc_result,
)


class TestFakeTonyLinkRecords:
    def test_call_is_recorded_as_method_and_params(self):
        link = FakeTonyLink()
        link.call("RunAction", ["wave", 1])
        assert link.calls == [("RunAction", ["wave", 1])]

    def test_calls_accumulate_in_order(self):
        link = FakeTonyLink()
        link.call("StopActionGroup", ["stopActionGroup"])
        link.call("StopBusServo", ["stopAction"])
        assert [m for m, _ in link.calls] == ["StopActionGroup", "StopBusServo"]

    def test_default_result_is_a_vendor_shaped_success(self):
        # the vendor methods return [ok, data, name]; the fake mimics that shape
        link = FakeTonyLink()
        res = link.call("RunAction", ["wave", 1])
        assert rpc_result(res) == (True, [])

    def test_scripted_result_is_returned(self):
        link = FakeTonyLink(results={
            "GetBusServosPulse": [True, [500] * 15, "GetBusServosPulse"]})
        res = link.call("GetBusServosPulse", ["angularReadback"])
        ok, data = rpc_result(res)
        assert ok is True
        assert data == [500] * 15

    def test_last_call_helper_finds_the_most_recent(self):
        link = FakeTonyLink()
        link.call("RunAction", ["wave", 1])
        link.call("RunAction", ["bow", 1])
        assert link.last_call("RunAction") == ("RunAction", ["bow", 1])

    def test_last_call_returns_none_when_never_called(self):
        assert FakeTonyLink().last_call("RunAction") is None

    def test_params_are_copied_not_aliased(self):
        # a caller mutating its list afterwards must not rewrite history
        link = FakeTonyLink()
        params = ["wave", 1]
        link.call("RunAction", params)
        params.append("mutated")
        assert link.calls[0] == ("RunAction", ["wave", 1])


class TestFakeTonyLinkSnapshot:
    def test_snapshot_returns_scripted_bytes(self):
        link = FakeTonyLink(snapshot=b"\xff\xd8jpeg")
        assert link.snapshot() == b"\xff\xd8jpeg"

    def test_snapshot_count_is_tracked(self):
        link = FakeTonyLink(snapshot=b"x")
        link.snapshot()
        link.snapshot()
        assert link.snapshots_taken == 2


class TestRpcResultParsing:
    def test_success_tuple_is_parsed(self):
        assert rpc_result([True, [1, 2, 3], "Whatever"]) == (True, [1, 2, 3])

    def test_failure_tuple_is_parsed(self):
        ok, data = rpc_result([False, "E02 - Invalid parameter!", "SetPWMServo"])
        assert ok is False
        assert "E02" in data

    def test_none_result_is_treated_as_failure(self):
        ok, _ = rpc_result(None)
        assert ok is False


class TestFakeTonyLinkIsATonyLink:
    def test_fake_satisfies_the_abc(self):
        assert isinstance(FakeTonyLink(), TonyLink)


class TestHttpTonyLinkRequestShapeOffline:
    """HttpTonyLink is verified on hardware, not here. But its request SHAPE is
    pure string/JSON assembly, so an injected opener pins it without a socket."""

    class _FakeOpener:
        def __init__(self, body):
            self._body = body
            self.requests = []

        def __call__(self, req, timeout=None):
            # req is a urllib.request.Request for POST, or a URL string for GET
            self.requests.append(req)
            return _FakeHTTPResponse(self._body)

    def test_call_posts_jsonrpc_2_0_envelope_to_port_9030(self):
        opener = self._FakeOpener(
            json.dumps({"jsonrpc": "2.0", "id": 1,
                        "result": [True, [], "RunAction"]}).encode())
        link = HttpTonyLink("tonypi.local", opener=opener)
        res = link.call("RunAction", ["wave", 1])

        req = opener.requests[0]
        assert req.full_url == "http://tonypi.local:9030"
        sent = json.loads(req.data)
        assert sent["jsonrpc"] == "2.0"
        assert sent["method"] == "RunAction"
        assert sent["params"] == ["wave", 1]
        assert rpc_result(res) == (True, [])

    def test_snapshot_gets_a_live_frame_from_the_video_port_8080(self):
        # ?action=snapshot, NOT the stream. Measured on hardware 2026-07-28:
        # snapshot 10/10 distinct frames, stream 1/10 (byte-identical, stale).
        # The opposite was believed for a day; see tonypi_link.snapshot().
        opener = self._FakeOpener(b"\xff\xd8\xff" + b"jpegbytes" + b"\xff\xd9")
        link = HttpTonyLink("tonypi.local", opener=opener)
        data = link.snapshot()
        assert data == b"\xff\xd8\xff" + b"jpegbytes" + b"\xff\xd9"
        assert opener.requests[0] == "http://tonypi.local:8080/?action=snapshot"

    def test_a_jsonrpc_error_envelope_raises(self):
        opener = self._FakeOpener(
            json.dumps({"jsonrpc": "2.0", "id": 1,
                        "error": {"code": -32601, "message": "Method not found"}}).encode())
        link = HttpTonyLink("tonypi.local", opener=opener)
        with pytest.raises(TonyRpcError):
            link.call("NoSuchMethod", [])


class _FakeHTTPResponse:
    def __init__(self, body):
        self._body = body

    def read(self, size=None):
        # Real HTTP responses take a size; snapshot() reads the stream in
        # chunks. Serve the whole body once, then EOF.
        body, self._body = self._body, b""
        return body

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False
