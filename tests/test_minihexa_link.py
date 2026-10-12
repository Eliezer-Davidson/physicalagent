"""Tests for the MiniHexa link seam — offline, no hardware, no serial, no BLE.

MiniHexaLink is to the MiniHexaAdapter what TonyLink is to the TonyPi adapter:
the one seam the adapter talks through, with a recording fake for tests and real
serial / BLE impls for hardware. FakeMiniHexaLink records every frame sent and
serves a scripted battery — so the adapter's clamping, gating and stop-invariant
logic run fully offline. It carries ALL the adapter tests.

The real links (Serial over CH340, BLE via bleak) are verified on hardware and
NEVER in CI — mirroring HttpTonyLink. Their PURE shaping (which frame bytes go to
which characteristic, which telemetry parser reads the battery) is pinned here
through INJECTED fakes, never a real socket or a real BLE stack. Neither real
backend opens any I/O at construction: opening is lazy, so link_for can build one
without touching hardware.
"""
import pytest

from physicalagent.robots import minihexa_codec as codec
from physicalagent.robots.minihexa_link import (
    BleMiniHexaLink, FakeMiniHexaLink, MiniHexaLink, SerialMiniHexaLink,
)


# ---------------------------------------------------------------------------
# FakeMiniHexaLink — the workhorse
# ---------------------------------------------------------------------------
class TestFakeRecordsFrames:
    def test_send_is_recorded(self):
        link = FakeMiniHexaLink()
        link.send(b"C|10|0|0&")
        assert link.sent == [b"C|10|0|0&"]

    def test_frames_accumulate_in_order(self):
        link = FakeMiniHexaLink()
        link.send(b"C|10|0|0&")
        link.send(b"C|0|0|0&")
        assert link.sent == [b"C|10|0|0&", b"C|0|0|0&"]

    def test_last_frame_helper(self):
        link = FakeMiniHexaLink()
        link.send(b"C|10|0|0&")
        link.send(b"C|0|0|0&")
        assert link.last_frame() == b"C|0|0|0&"

    def test_last_frame_is_none_when_nothing_sent(self):
        assert FakeMiniHexaLink().last_frame() is None

    def test_sent_text_decodes_every_frame(self):
        link = FakeMiniHexaLink()
        link.send(b"C|10|0|0&")
        link.send(codec.encode_stop())
        assert link.sent_text() == ["C|10|0|0&", "C|0|0|0&"]


class TestFakeBattery:
    def test_default_battery_is_healthy_2s(self):
        assert FakeMiniHexaLink().read_battery_mv() == 8100

    def test_battery_is_settable(self):
        link = FakeMiniHexaLink()
        link.battery_mv = 6800
        assert link.read_battery_mv() == 6800

    def test_battery_can_be_set_unreadable(self):
        link = FakeMiniHexaLink(battery_mv=None)
        assert link.read_battery_mv() is None


class TestFakeCapabilities:
    def test_defaults_to_per_joint_capable(self):
        assert FakeMiniHexaLink().supports_per_joint is True

    def test_per_joint_can_be_disabled_to_simulate_ble(self):
        assert FakeMiniHexaLink(supports_per_joint=False).supports_per_joint is False

    def test_default_kind_is_fake(self):
        assert FakeMiniHexaLink().kind == "fake"

    def test_kind_is_settable(self):
        assert FakeMiniHexaLink(kind="serial").kind == "serial"


class TestFakeIsALink:
    def test_fake_satisfies_the_abc(self):
        assert isinstance(FakeMiniHexaLink(), MiniHexaLink)


# ---------------------------------------------------------------------------
# SerialMiniHexaLink — shaping only, via an injected fake serial (no socket)
# ---------------------------------------------------------------------------
class _FakeSerial:
    def __init__(self, to_read=b""):
        self.written: list[bytes] = []
        self._to_read = bytes(to_read)

    def write(self, data):
        self.written.append(bytes(data))

    def read(self, n):
        chunk, self._to_read = self._to_read[:n], self._to_read[n:]
        return chunk


class TestSerialLinkShapeOffline:
    def _link(self, to_read=b""):
        fake = _FakeSerial(to_read=to_read)
        link = SerialMiniHexaLink("/dev/ttyUSB0", 115200, opener=lambda d, b: fake)
        return link, fake

    def test_no_io_happens_at_construction(self):
        # opening is lazy: constructing must not call the opener at all, so
        # link_for can build a serial link without touching hardware.
        opened = []
        SerialMiniHexaLink("/dev/ttyUSB0", 115200,
                           opener=lambda d, b: opened.append((d, b)) or _FakeSerial())
        assert opened == []

    def test_send_writes_the_exact_frame_bytes(self):
        link, fake = self._link()
        link.send(b"C|30|0|1&")
        assert fake.written == [b"C|30|0|1&"]

    def test_read_battery_parses_the_usb_a_frame(self):
        link, _ = self._link(to_read=b"noise A|7900& more")
        assert link.read_battery_mv() == 7900

    def test_read_battery_is_none_when_no_frame_seen(self):
        link, _ = self._link(to_read=b"garbage")
        assert link.read_battery_mv() is None

    def test_serial_supports_per_joint(self):
        link, _ = self._link()
        assert link.supports_per_joint is True

    def test_kind_is_serial(self):
        link, _ = self._link()
        assert link.kind == "serial"

    def test_is_a_minihexa_link(self):
        link, _ = self._link()
        assert isinstance(link, MiniHexaLink)


# ---------------------------------------------------------------------------
# BleMiniHexaLink — shaping only, via an injected fake bleak client (no radio)
# ---------------------------------------------------------------------------
class _FakeBleClient:
    def __init__(self):
        self.written: list[tuple[str, bytes]] = []
        self._cb = None
        self._notify_char = None

    def start_notify(self, char, cb):
        self._notify_char = char
        self._cb = cb

    def write(self, char, data):
        self.written.append((char, bytes(data)))

    def push_notify(self, data):  # test helper: simulate a firmware notification
        self._cb(self._notify_char, bytes(data))


WRITE_CHAR = "0000ffe1-0000-1000-8000-00805f9b34fb"
NOTIFY_CHAR = "0000ffe2-0000-1000-8000-00805f9b34fb"


class TestBleLinkShapeOffline:
    def _link(self):
        fake = _FakeBleClient()
        link = BleMiniHexaLink("<BLE_ADDR>", write_char=WRITE_CHAR,
                               notify_char=NOTIFY_CHAR, client=fake)
        return link, fake

    def test_no_connect_happens_at_construction(self):
        # a fake client is bound lazily; construction must not call start_notify,
        # so link_for can build a BLE link without opening the radio.
        fake = _FakeBleClient()
        BleMiniHexaLink("addr", write_char=WRITE_CHAR, notify_char=NOTIFY_CHAR,
                        client=fake)
        assert fake._cb is None

    def test_send_writes_to_the_write_characteristic_ffe1(self):
        link, fake = self._link()
        link.send(b"C|5|0|0&")
        assert fake.written == [(WRITE_CHAR, b"C|5|0|0&")]

    def test_read_battery_parses_the_ble_dollar_frame(self):
        link, fake = self._link()
        link.send(b"C|0|0|0&")            # triggers lazy connect + start_notify
        fake.push_notify(b"$7900$65535$")
        assert link.read_battery_mv() == 7900

    def test_read_battery_is_none_before_any_notification(self):
        link, _ = self._link()
        link.send(b"C|0|0|0&")
        assert link.read_battery_mv() is None

    def test_ble_does_not_support_per_joint(self):
        # the BLE firmware does not parse L or K|2 — the central capability gap.
        link, _ = self._link()
        assert link.supports_per_joint is False

    def test_kind_is_ble(self):
        link, _ = self._link()
        assert link.kind == "ble"

    def test_is_a_minihexa_link(self):
        link, _ = self._link()
        assert isinstance(link, MiniHexaLink)
