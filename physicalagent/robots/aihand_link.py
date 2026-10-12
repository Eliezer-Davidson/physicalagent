"""AiHandLink — the seam between the AiHand controller and its transports.

Two real transports, because the hand can be reached two ways and which one is
in play is a deployment fact, not a code fact:

    serial   the hand's micro:bit V2 enumerates as USB CDC (/dev/ttyACM*) on
             whichever computer it is plugged into. Direct, no middleman.
    tcp      when the micro:bit is plugged into the ROBOT's computer (the
             TonyPi's Raspberry Pi) rather than the operator's laptop, a small
             bridge process on that computer forwards a TCP port to the serial
             device, and this link dials it. See tools/aihand_bridge.py.

Both open LAZILY. A link is built by the profile factory before anyone knows
whether the hand is powered, plugged in, or bridged; a constructor that opened
a port would either hang the CLI or push the caller into swallowing the error.
An earlier draft did exactly that — it dialled in the factory under a bare
`except`, so an unreachable hand silently became no hand at all and the tool
just never appeared. Failing loudly at send time is the fix: an operator who
asked for a hand and has no hand must be told.

Neither backend is exercised in CI. Their pure shaping — which bytes go to the
port, when the port is opened, what happens when the far end is dead — is pinned
in tests through injected fakes.
"""
from __future__ import annotations

from abc import ABC, abstractmethod

DEFAULT_DEVICE = "/dev/ttyACM0"     # micro:bit V2 CDC, VID:PID 0d28:0204
DEFAULT_BAUD = 115200
DEFAULT_TCP_PORT = 9031
DEFAULT_TIMEOUT_S = 2.0


class AiHandLink(ABC):
    """One direction only: frames out. The AiHand has NO feedback of any kind —
    no position, no load, no temperature — so there is deliberately no read()
    on this interface. Anything that looked like a read would be a fiction."""

    @abstractmethod
    def send(self, frame: bytes) -> None:
        """Put one encoded frame on the wire. Raises OSError if it cannot."""

    def close(self) -> None:
        """Release the transport. Safe to call when nothing was ever opened."""

    @property
    @abstractmethod
    def kind(self) -> str:
        """Transport name, for the operator's status line."""


class FakeAiHandLink(AiHandLink):
    """Recording fake. Carries every offline test.

    `fail_on_send` scripts the unplugged-hand case so callers can be tested
    against a transport that refuses, not just one that accepts.
    """

    def __init__(self, *, fail_on_send: bool = False):
        self.sent: list[bytes] = []
        self._fail = fail_on_send
        self.closed = False

    def send(self, frame: bytes) -> None:
        if self._fail:
            raise OSError("fake AiHand link is scripted to fail")
        self.sent.append(bytes(frame))

    def close(self) -> None:
        self.closed = True

    @property
    def kind(self) -> str:
        return "fake"


class SerialAiHandLink(AiHandLink):
    """The micro:bit over USB CDC, on this computer.

    Watch for brltty: the screen-reader daemon has twice hijacked a CDC/CH340
    port on this machine and made a present device look absent. If the device
    node vanishes moments after plugging in, that is the cause, not the hand.
    """

    def __init__(self, device: str = DEFAULT_DEVICE, baud: int = DEFAULT_BAUD,
                 *, opener=None):
        self._device = device
        self._baud = baud
        self._opener = opener or self._default_open
        self._port = None

    @staticmethod
    def _default_open(device: str, baud: int):
        import serial  # imported here so the registry stays importable
        return serial.Serial(device, baud, timeout=1)

    def _ensure(self):
        if self._port is None:
            self._port = self._opener(self._device, self._baud)
        return self._port

    def send(self, frame: bytes) -> None:
        self._ensure().write(frame)

    def close(self) -> None:
        if self._port is not None:
            self._port.close()
            self._port = None

    @property
    def kind(self) -> str:
        return "serial"


class TcpAiHandLink(AiHandLink):
    """A bridge process on the robot's own computer, forwarding TCP to serial.

    A failed dial does NOT latch: the bridge is frequently started after the
    CLI, so every send retries the connection. What it must never do is
    swallow the failure — a hand that cannot be reached has to raise.
    """

    def __init__(self, host: str, port: int = DEFAULT_TCP_PORT, *,
                 timeout: float = DEFAULT_TIMEOUT_S, dialer=None):
        self._host = host
        self._port = port
        self._timeout = timeout
        self._dialer = dialer or self._default_dial
        self._sock = None

    @staticmethod
    def _default_dial(host: str, port: int, timeout: float):
        import socket
        return socket.create_connection((host, port), timeout=timeout)

    def _ensure(self):
        if self._sock is None:
            self._sock = self._dialer(self._host, self._port, self._timeout)
        return self._sock

    def send(self, frame: bytes) -> None:
        try:
            self._ensure().sendall(frame)
        except OSError:
            # Drop the dead socket so the next send redials rather than
            # writing into a closed pipe forever.
            self._sock = None
            raise

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    @property
    def kind(self) -> str:
        return "tcp"
