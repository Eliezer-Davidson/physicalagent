"""Tests for the AiHand link seam — offline, no serial, no sockets, no hardware.

AiHandLink is to the hand what MiniHexaLink is to the hexapod: the one seam the
adapter sends bytes through, with a recording fake carrying every logic test and
real backends (serial to the micro:bit, TCP to a bridge on the robot's Pi)
verified on hardware and NEVER in CI. The real backends' PURE shaping is pinned
here through INJECTED fakes.

Neither real backend opens anything at construction. That is load-bearing: the
profile factory builds a link before anyone knows whether the hand is plugged
in, and a constructor that dialled a socket would either hang the CLI or force
the caller to swallow the failure — which is precisely how an earlier draft of
this integration ended up silently handless.
"""
import pytest

from physicalagent.robots import aihand_codec as codec
from physicalagent.robots.aihand_link import (
    AiHandLink, FakeAiHandLink, SerialAiHandLink, TcpAiHandLink,
)


# ---------------------------------------------------------------------------
# FakeAiHandLink — the workhorse
# ---------------------------------------------------------------------------
class TestFakeRecordsFrames:
    def test_send_is_recorded(self):
        link = FakeAiHandLink()
        link.send(b"\x55\x55\x05\x03\x01")
        assert link.sent == [b"\x55\x55\x05\x03\x01"]

    def test_frames_accumulate_in_order(self):
        link = FakeAiHandLink()
        link.send(b"a")
        link.send(b"b")
        assert link.sent == [b"a", b"b"]

    def test_kind_names_the_transport(self):
        assert FakeAiHandLink().kind == "fake"

    def test_close_is_safe_to_call_twice(self):
        link = FakeAiHandLink()
        link.close()
        link.close()

    def test_a_fake_can_be_scripted_to_fail_like_an_unplugged_hand(self):
        link = FakeAiHandLink(fail_on_send=True)
        with pytest.raises(OSError):
            link.send(b"a")


class TestFakeIsALink:
    def test_fake_satisfies_the_abc(self):
        assert isinstance(FakeAiHandLink(), AiHandLink)


# ---------------------------------------------------------------------------
# SerialAiHandLink — micro:bit over USB CDC
# ---------------------------------------------------------------------------
class _FakePort:
    def __init__(self):
        self.written = []
        self.closed = False

    def write(self, data):
        self.written.append(bytes(data))

    def close(self):
        self.closed = True


class TestSerialLink:
    def test_construction_opens_nothing(self):
        opened = []
        SerialAiHandLink(device="/dev/ttyACM0",
                         opener=lambda d, b: opened.append((d, b)))
        assert opened == []

    def test_first_send_opens_the_port_once(self):
        opens = []

        def opener(device, baud):
            opens.append((device, baud))
            return _FakePort()

        link = SerialAiHandLink(device="/dev/ttyACM0", baud=115200,
                                opener=opener)
        link.send(b"a")
        link.send(b"b")
        assert opens == [("/dev/ttyACM0", 115200)]

    def test_bytes_reach_the_port_unaltered(self):
        port = _FakePort()
        link = SerialAiHandLink(opener=lambda d, b: port)
        link.send(codec.encode_lobot({1: 1500}, 1000))
        assert port.written == [codec.encode_lobot({1: 1500}, 1000)]

    def test_close_closes_an_opened_port(self):
        port = _FakePort()
        link = SerialAiHandLink(opener=lambda d, b: port)
        link.send(b"a")
        link.close()
        assert port.closed is True

    def test_close_without_a_send_touches_nothing(self):
        link = SerialAiHandLink(opener=lambda d, b: pytest.fail("opened"))
        link.close()

    def test_kind_names_the_transport(self):
        assert SerialAiHandLink(opener=lambda d, b: _FakePort()).kind == "serial"


# ---------------------------------------------------------------------------
# TcpAiHandLink — through a bridge process on the robot's own computer
# ---------------------------------------------------------------------------
class _FakeSock:
    def __init__(self):
        self.sent = []
        self.closed = False
        self.timeout = None

    def sendall(self, data):
        self.sent.append(bytes(data))

    def settimeout(self, t):
        self.timeout = t

    def close(self):
        self.closed = True


class TestTcpLink:
    def test_construction_dials_nothing(self):
        dialled = []
        TcpAiHandLink(host="10.0.0.1", port=9031,
                      dialer=lambda h, p, t: dialled.append((h, p)))
        assert dialled == []

    def test_first_send_dials_once(self):
        dials = []

        def dialer(host, port, timeout):
            dials.append((host, port))
            return _FakeSock()

        link = TcpAiHandLink(host="10.0.0.1", port=9031, dialer=dialer)
        link.send(b"a")
        link.send(b"b")
        assert dials == [("10.0.0.1", 9031)]

    def test_bytes_reach_the_socket_unaltered(self):
        sock = _FakeSock()
        link = TcpAiHandLink(host="h", dialer=lambda h, p, t: sock)
        frame = codec.encode_ascii({1: 1500}, 1000)
        link.send(frame)
        assert sock.sent == [frame]

    def test_a_dead_bridge_surfaces_as_an_error_not_a_silent_noop(self):
        def dialer(host, port, timeout):
            raise ConnectionRefusedError("nothing listening")

        link = TcpAiHandLink(host="h", dialer=dialer)
        with pytest.raises(OSError):
            link.send(b"a")

    def test_a_failed_dial_is_retried_on_the_next_send(self):
        # The bridge may be started after the CLI. A first failure must not
        # latch the link into a permanently-dead state.
        attempts = []

        def dialer(host, port, timeout):
            attempts.append(1)
            if len(attempts) == 1:
                raise ConnectionRefusedError("not yet")
            return _FakeSock()

        link = TcpAiHandLink(host="h", dialer=dialer)
        with pytest.raises(OSError):
            link.send(b"a")
        link.send(b"a")
        assert len(attempts) == 2

    def test_close_closes_a_dialled_socket(self):
        sock = _FakeSock()
        link = TcpAiHandLink(host="h", dialer=lambda h, p, t: sock)
        link.send(b"a")
        link.close()
        assert sock.closed is True

    def test_kind_names_the_transport(self):
        assert TcpAiHandLink(host="h").kind == "tcp"
