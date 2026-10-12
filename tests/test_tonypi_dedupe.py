"""Tests for dedupe_frame and — the point of them — the path it sits on.

MEASURED 2026-07-28: this TonyPi's camera publishes 640x960 frames whose bottom
half is a BYTE-EXACT copy of the top. `dedupe_frame()` was written to crop that
away before the image reaches a vision model, for two reasons recorded in its
docstring: half of every image is paid for in tokens and carries nothing, and a
duplicated scene invites the model to report two of every object.

The function was correct and had no tests, and the wiring was the part that was
wrong. `get_frame()` deduped; the `look` TOOL did not, and the `look` tool is
what actually feeds the model (brain/er.py reads `adapter.last_frame` after a
`look` and attaches it as an image). So the fix ran on a path production never
took. These tests pin BOTH the function and the wiring, because on this stack a
correct function on a dead path is indistinguishable from no fix at all.

Pillow is a hard requirement of that path — dedupe_frame passes the frame
through untouched when it cannot import PIL, which is the right failure for one
bad frame and the wrong one for a missing library. TestPillowIsPresent turns
that silent degradation into a red test.
"""
import io

import pytest

from physicalagent.robots.tonypi import TonyPiAdapter, dedupe_frame
from physicalagent.robots.tonypi_link import FakeTonyLink
from physicalagent.robots.transport import ScriptedClock
from physicalagent.safety import TONYPI_SAFETY, SafetyGuard

Image = pytest.importorskip("PIL.Image")

WIDTH, HALF = 64, 48


def _half(shade: int):
    """One WIDTH x HALF panel of a flat colour, with a corner mark so JPEG has
    something to encode and the two halves are not trivially identical."""
    im = Image.new("RGB", (WIDTH, HALF), (shade, shade, shade))
    im.paste((255, 0, 0), (0, 0, 8, 8))
    return im


def _jpeg(im) -> bytes:
    buf = io.BytesIO()
    im.save(buf, format="JPEG", quality=90)
    return buf.getvalue()


def duplicated_frame() -> bytes:
    """A frame shaped like this robot's: the bottom half copies the top."""
    top = _half(120)
    out = Image.new("RGB", (WIDTH, HALF * 2))
    out.paste(top, (0, 0))
    out.paste(top, (0, HALF))
    return _jpeg(out)


def stereo_frame() -> bytes:
    """A frame whose halves genuinely differ — must NOT be cropped."""
    out = Image.new("RGB", (WIDTH, HALF * 2))
    out.paste(_half(30), (0, 0))
    out.paste(_half(220), (0, HALF))
    return _jpeg(out)


def size_of(jpeg: bytes):
    return Image.open(io.BytesIO(jpeg)).size


def make_adapter(snapshot: bytes):
    link = FakeTonyLink(snapshot=snapshot)
    a = TonyPiAdapter(link, SafetyGuard(TONYPI_SAFETY), clock=ScriptedClock())
    return a, link


class TestPillowIsPresent:
    def test_pillow_imports_so_dedupe_is_not_a_silent_noop(self):
        # dedupe_frame returns the frame untouched on ImportError. That is the
        # right call for robustness and the wrong one for observability: with
        # no Pillow the vision fix disappears and nothing anywhere says so.
        # Pillow is therefore a declared dependency, and this test is what
        # notices when it stops being installed.
        from PIL import Image as _Image  # noqa: F401


class TestDedupeFrame:
    def test_a_duplicated_frame_is_cropped_to_its_top_half(self):
        assert size_of(dedupe_frame(duplicated_frame())) == (WIDTH, HALF)

    def test_a_frame_whose_halves_differ_is_left_alone(self):
        frame = stereo_frame()
        assert dedupe_frame(frame) is frame
        assert size_of(dedupe_frame(frame)) == (WIDTH, HALF * 2)

    def test_an_odd_height_frame_is_left_alone(self):
        # h % 2 means the halves cannot be compared; never guess a crop line.
        frame = _jpeg(Image.new("RGB", (WIDTH, 47), (10, 10, 10)))
        assert dedupe_frame(frame) is frame

    def test_bytes_that_are_not_an_image_pass_through_untouched(self):
        # A frame we cannot parse is still a frame: hand it on rather than
        # raising inside a sensing call the model is waiting on.
        assert dedupe_frame(b"\xff\xd8not-a-jpeg") == b"\xff\xd8not-a-jpeg"

    def test_empty_bytes_pass_through_untouched(self):
        assert dedupe_frame(b"") == b""

    def test_the_cropped_frame_is_smaller_on_the_wire(self):
        # The token cost is the reason this exists, so assert the payload
        # actually shrank rather than only that the geometry changed.
        frame = duplicated_frame()
        assert len(dedupe_frame(frame)) < len(frame)


class TestTheLookToolDedupes:
    """`look` is the tool the model calls, and brain/er.py attaches
    `adapter.last_frame` as the image on the next turn. Whatever `look` leaves
    in last_frame IS what the model sees."""

    def test_look_leaves_a_deduped_frame_in_last_frame(self):
        a, _ = make_adapter(duplicated_frame())
        assert a.execute("look", {}).ok
        assert size_of(a.last_frame) == (WIDTH, HALF)

    def test_look_reports_the_size_of_what_it_actually_kept(self):
        # The payload's `bytes` is the model's and the operator's only readout
        # of frame size; it must describe the cropped image, not the original.
        a, _ = make_adapter(duplicated_frame())
        r = a.execute("look", {})
        assert r.payload["bytes"] == len(a.last_frame)
        assert r.payload["bytes"] < len(duplicated_frame())

    def test_look_does_not_crop_a_frame_whose_halves_differ(self):
        a, _ = make_adapter(stereo_frame())
        assert a.execute("look", {}).ok
        assert size_of(a.last_frame) == (WIDTH, HALF * 2)

    def test_get_frame_and_look_agree(self):
        # Two entry points to one camera must not disagree about what a frame
        # is. They did: get_frame deduped and look did not.
        a, _ = make_adapter(duplicated_frame())
        a.execute("look", {})
        via_look = a.last_frame
        assert size_of(a.get_frame()) == size_of(via_look)

    def test_an_unparseable_frame_still_reaches_the_model(self):
        a, _ = make_adapter(b"\xff\xd8xyz")
        r = a.execute("look", {})
        assert r.ok
        assert a.last_frame == b"\xff\xd8xyz"
