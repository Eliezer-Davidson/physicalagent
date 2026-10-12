"""Tests for pulling a LIVE frame off the TonyPi's MJPEG server.

Found on hardware 2026-07-27: the robot's `?action=snapshot` endpoint returned a
byte-identical JPEG (md5 e7a1d7607218, 31432 B) sampled every two seconds across
six, with the vision heartbeat fed throughout. The `?action=stream` endpoint on
the same server serves a different, smaller image (15202 B) — a second source,
not the same cache — so the link reads from the stream.

That matters because `snapshot()` is what get_frame() calls, which is what the
model SEES. A stale frame means the vision model reasons about a moment that has
passed, and nothing in the stack reports an error: the fetch succeeds, the bytes
are a valid JPEG, and the robot appears to be looking at a world that stopped.

WHAT IS NOT ESTABLISHED. Nobody has yet seen two frames from this robot that
differ from each other, on either endpoint. These tests therefore pin the
PARSING (which bytes constitute a frame, when to stop reading, what happens when
there is no frame) and deliberately claim nothing about liveness. A test that
asserted "the camera works" would be asserting something no observation
supports.

CORRECTED 2026-07-28. This module previously stated the cause as "the vendor RPC
acknowledges a start it does not perform", inferred from GetRunningFunc
answering [True,[0]]. That is void — RPCServer.py:424-426 hardcodes
`return (True, (0,))` with the real call commented out, so it answers 0 whether
or not anything is running, and no other RPC method exposes the running function.
We read a constant and called it telemetry. The live candidates are now
Running.loadFunc swallowing init() exceptions while still returning True, and
MjpgServer's stream branch nulling the shared `img_show` global that the
snapshot branch reads. Both leave their evidence only on the Pi's stdout.
See tonypi_link.snapshot() for the citations.
"""
import pytest

from physicalagent.robots.tonypi_link import HttpTonyLink, first_jpeg

SOI, EOI = b"\xff\xd8\xff", b"\xff\xd9"


def mjpeg(*frames: bytes) -> bytes:
    """A multipart/x-mixed-replace body carrying the given JPEG payloads."""
    out = b""
    for f in frames:
        out += (b"--boundarydonotcross\r\n"
                b"Content-Type: image/jpeg\r\n"
                b"Content-Length: " + str(len(f)).encode() + b"\r\n\r\n")
        out += f + b"\r\n"
    return out


def jpg(marker: bytes = b"A") -> bytes:
    return SOI + b"\x00" + marker * 40 + EOI


class TestFirstJpeg:
    def test_extracts_a_frame_from_a_multipart_body(self):
        assert first_jpeg(mjpeg(jpg())) == jpg()

    def test_takes_the_first_complete_frame_not_the_last(self):
        assert first_jpeg(mjpeg(jpg(b"A"), jpg(b"B"))) == jpg(b"A")

    def test_skips_the_multipart_headers(self):
        frame = first_jpeg(mjpeg(jpg()))
        assert frame.startswith(SOI)

    def test_a_frame_is_returned_whole(self):
        assert first_jpeg(mjpeg(jpg())).endswith(EOI)

    def test_a_truncated_frame_is_not_returned_half_built(self):
        # A half-received JPEG decodes to garbage or a grey band; better to
        # have nothing and say so than to hand the model a corrupt image.
        assert first_jpeg(SOI + b"\x00" + b"A" * 20) is None

    def test_a_body_with_no_frame_at_all_returns_none(self):
        assert first_jpeg(b"--boundary\r\nContent-Type: text/plain\r\n\r\n") is None

    def test_empty_input_returns_none(self):
        assert first_jpeg(b"") is None


class _FakeResponse:
    """A stream that hands out bytes in chunks, like a real socket read."""

    def __init__(self, body: bytes, chunk: int = 16):
        self._body, self._chunk, self._pos = body, chunk, 0

    def read(self, n=None):
        n = self._chunk if n is None else min(n, self._chunk)
        out = self._body[self._pos:self._pos + n]
        self._pos += len(out)
        return out

    def close(self):
        pass


def _link(body):
    opened = []

    def opener(url, timeout=None):
        opened.append(url if isinstance(url, str) else url.full_url)
        return _FakeResponse(body)

    return HttpTonyLink(host="h", opener=opener), opened


class TestSnapshotUsesTheLiveEndpoint:
    """snapshot() reads ?action=snapshot — the path measured live on hardware."""

    def test_it_reads_the_snapshot_endpoint_not_the_stale_stream(self):
        link, opened = _link(jpg())
        link.snapshot()
        assert opened and "action=snapshot" in opened[0]
        assert "action=stream" not in opened[0]

    def test_it_returns_the_jpeg_whole(self):
        link, _ = _link(jpg())
        assert link.snapshot() == jpg()

    def test_a_non_jpeg_body_raises_rather_than_being_handed_to_the_model(self):
        # An HTML error page is bytes too. Handing it to a vision model gets it
        # described as if it were a photograph.
        link, _ = _link(b"<html>500 no camera</html>")
        with pytest.raises(OSError, match="not a JPEG"):
            link.snapshot()

    def test_the_error_names_the_keepalive_as_the_likely_cause(self):
        # The vendor's 7s deadman closes the camera when Heartbeat lapses, so
        # that is the first thing to check — the message should say so.
        link, _ = _link(b"")
        with pytest.raises(OSError, match="heartbeat"):
            link.snapshot()


class TestStreamFrameParsing:
    """stream_frame() is kept but unused here: on THIS robot the stream is the
    stale path. The parsing stays tested because MjpgServer is vendored
    per-robot and a sibling model may invert this."""

    def test_it_returns_a_frame_from_the_stream(self):
        link, _ = _link(mjpeg(jpg()))
        assert link.stream_frame() == jpg()

    def test_it_reads_the_stream_endpoint(self):
        link, opened = _link(mjpeg(jpg()))
        link.stream_frame()
        assert opened and "action=stream" in opened[0]

    def test_it_stops_reading_once_it_has_a_whole_frame(self):
        # A never-ending stream must not be drained into memory: the read stops
        # at the first complete JPEG, or a 5-second grab becomes 8 MB.
        link, _ = _link(mjpeg(jpg(b"A")) + b"X" * 5_000_000)
        assert link.stream_frame() == jpg(b"A")

    def test_a_stream_that_never_yields_a_frame_raises(self):
        link, _ = _link(b"not a jpeg at all")
        with pytest.raises(OSError, match="no frame"):
            link.stream_frame()
