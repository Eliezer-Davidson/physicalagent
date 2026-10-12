"""MiniHexaLink — the seam between the MiniHexa adapter and its serial / BLE links.

The MiniHexaAdapter never imports pyserial or bleak. It talks to a MiniHexaLink:
  * send(frame)            -> push raw codec bytes at the robot
  * read_battery_mv()      -> the link parses its OWN telemetry format
  * supports_per_joint     -> True for serial (L / K|2 available), False for BLE
  * kind                   -> "serial" | "ble" | "fake"

The two transports are NOT interchangeable, and that asymmetry is a safety fact,
not an implementation detail:

  * SERIAL (CH340, /dev/ttyUSB0, 115200 8N1) parses the FULL command set,
    including L (per-joint) and K|2 (action-stop), and pushes 'A|<mV>&' telemetry.
  * BLE   (device "miniHexa", write char ffe1, notify char ffe2) parses a REDUCED
    set — C (move/stop), F (pose), K|1|N (run action) — but NOT L and NOT K|2,
    and pushes a different '$<mV>$<field2>$' telemetry framing. A BLE disconnect
    does NOT stop the robot.

So `supports_per_joint` is how the adapter refuses per-joint control over BLE, and
each link owns its own battery parser (codec.parse_battery_frame vs
codec.parse_ble_battery).

Production uses SerialMiniHexaLink / BleMiniHexaLink. Tests use FakeMiniHexaLink,
which records every frame sent and serves a scripted battery — so the adapter runs
fully offline. The real links mirror HttpTonyLink: their shaping is unit-testable
through an injected opener / client, but the socket and the radio are verified on
hardware, NEVER in CI. Neither opens any I/O at construction — opening is lazy, so
the CLI can build a link object without touching hardware.
"""
from __future__ import annotations

from abc import ABC, abstractmethod

from physicalagent.robots import minihexa_codec as codec

# BLE GATT: service ffe0, write char ffe1, notify char ffe2 (device "miniHexa").
BLE_WRITE_CHAR = "0000ffe1-0000-1000-8000-00805f9b34fb"
BLE_NOTIFY_CHAR = "0000ffe2-0000-1000-8000-00805f9b34fb"
# Keep only the tail of the telemetry stream — a battery frame is tiny and arrives
# ~1 Hz, so a small rolling buffer is all we ever need to scan.
_ROLLING_BUFFER_BYTES = 256


class MiniHexaLink(ABC):
    """The one seam the MiniHexa adapter talks through."""

    @abstractmethod
    def send(self, frame: bytes) -> None:
        """Push raw codec frame bytes at the robot."""

    @abstractmethod
    def read_battery_mv(self) -> int | None:
        """Latest pack millivolts, parsed from THIS link's telemetry framing, or
        None if nothing readable has arrived (which the battery gate treats as
        unsafe and refuses motion — fail closed)."""

    @property
    @abstractmethod
    def supports_per_joint(self) -> bool:
        """True where the firmware parses L (per-joint) and K|2 (action-stop) —
        i.e. serial. False over BLE, where those opcodes are silently ignored."""

    @property
    @abstractmethod
    def kind(self) -> str:
        """"serial" | "ble" | "fake"."""


class FakeMiniHexaLink(MiniHexaLink):
    """In-memory link for tests. Records every frame sent and serves a scripted
    battery; mirrors FakeTonyLink's discipline. This carries all the adapter
    tests, so its behaviour is deliberately simple and inspectable.

    `battery_mv` is what read_battery_mv() returns (settable directly, or via the
    constructor); None models an unreadable pack. `supports_per_joint` models the
    serial/BLE capability split (pass False to exercise the BLE refusal paths).
    `kind` is the reported transport label.
    """

    def __init__(self, battery_mv: int | None = 8100,
                 supports_per_joint: bool = True, kind: str = "fake"):
        self.sent: list[bytes] = []
        self.battery_mv = battery_mv
        self._supports_per_joint = supports_per_joint
        self._kind = kind

    def send(self, frame: bytes) -> None:
        self.sent.append(bytes(frame))   # copy: caller may reuse the buffer

    def read_battery_mv(self) -> int | None:
        return self.battery_mv

    @property
    def supports_per_joint(self) -> bool:
        return self._supports_per_joint

    @property
    def kind(self) -> str:
        return self._kind

    # -- test helpers -------------------------------------------------------
    def last_frame(self) -> bytes | None:
        return self.sent[-1] if self.sent else None

    def sent_text(self) -> list[str]:
        return [f.decode("utf-8", "replace") for f in self.sent]


class SerialMiniHexaLink(MiniHexaLink):
    """Production serial link over pyserial (CH340, 115200 8N1). The opener is
    injected so the frame shaping is unit-testable; the socket itself is verified
    on hardware, never in CI.

    Opening is LAZY: construction stores config and opens NOTHING, so the CLI can
    build the object without touching hardware. On the first I/O the opener runs;
    the default opener sets DTR=False and RTS=False BEFORE opening the port, so
    opening does not pulse the ESP32's reset line (which a bare open would).

    read_battery_mv scans a small rolling buffer of the most recent bytes for the
    USB 'A|<mV>&' telemetry frame (codec.parse_battery_frame).
    """

    def __init__(self, device: str = "/dev/ttyUSB0", baud: int = 115200, *,
                 opener=None, read_bytes: int = _ROLLING_BUFFER_BYTES):
        self._device = device
        self._baud = baud
        self._opener = opener or self._default_open
        self._read_bytes = read_bytes
        self._buf = bytearray()
        self._serial = None   # opened lazily on first send / read

    @staticmethod
    def _default_open(device: str, baud: int):
        """Open the real port with the ESP32-reset-safe DTR/RTS dance. Hardware
        only — never reached in CI, which always injects an opener."""
        import serial  # pyserial; imported lazily so tests never need it
        port = serial.Serial()
        port.port = device
        port.baudrate = baud
        # DTR/RTS low BEFORE open so opening the port does not assert the auto-
        # reset line and reboot the ESP32 mid-session.
        port.dtr = False
        port.rts = False
        port.open()
        return port

    def _port(self):
        if self._serial is None:
            self._serial = self._opener(self._device, self._baud)
        return self._serial

    def send(self, frame: bytes) -> None:
        self._port().write(bytes(frame))

    def read_battery_mv(self) -> int | None:
        data = self._port().read(self._read_bytes)
        if data:
            self._buf.extend(data)
            if len(self._buf) > self._read_bytes:
                del self._buf[:-self._read_bytes]   # keep only the tail
        return codec.parse_battery_frame(bytes(self._buf))

    @property
    def supports_per_joint(self) -> bool:
        return True

    @property
    def kind(self) -> str:
        return "serial"


class BleMiniHexaLink(MiniHexaLink):
    """Production BLE link (device "miniHexa"). The client is injected so the
    write-characteristic routing and the telemetry parse are unit-testable; the
    real radio I/O is verified on hardware, never in CI.

    Connecting is LAZY: construction stores address/characteristics and connects
    NOTHING, so the CLI can build the object without opening the radio. On the
    first I/O the client is bound and the notify characteristic (ffe2) is
    subscribed into a rolling buffer; send writes to the write characteristic
    (ffe1); read_battery_mv parses the BLE '$<mV>$<field2>$' framing
    (codec.parse_ble_battery, which fails closed outside a sane LiPo window).

    supports_per_joint is False: the BLE firmware does not parse L or K|2, so the
    adapter must refuse per-joint control and must not rely on K|2 to stop.
    """

    def __init__(self, address: str, *, write_char: str = BLE_WRITE_CHAR,
                 notify_char: str = BLE_NOTIFY_CHAR, client=None,
                 buffer_bytes: int = _ROLLING_BUFFER_BYTES):
        self._address = address
        self._write_char = write_char
        self._notify_char = notify_char
        self._pending_client = client   # injected fake, or None -> real bleak
        self._client = None             # bound lazily on first I/O
        self._buffer_bytes = buffer_bytes
        self._buf = bytearray()

    def _default_client(self, address: str):
        """Connect the real bleak-backed synchronous client. Hardware only —
        never reached in CI, which always injects a client (this is the same
        seam as SerialMiniHexaLink._default_open / HttpTonyLink's urllib opener:
        real I/O glue, verified on hardware, not in unit tests)."""
        if not address:
            raise RuntimeError(
                "no BLE address: set MINIHEXA_BLE_ADDRESS to the hexapod's MAC "
                "(bluetoothctl scan on; the device advertises 'miniHexa')")
        return _BleakSyncClient(address)

    def _on_notify(self, _char, data) -> None:
        self._buf.extend(bytes(data))
        if len(self._buf) > self._buffer_bytes:
            del self._buf[:-self._buffer_bytes]

    def _conn(self):
        if self._client is None:
            client = (self._pending_client if self._pending_client is not None
                      else self._default_client(self._address))
            client.start_notify(self._notify_char, self._on_notify)
            self._client = client
        return self._client

    def send(self, frame: bytes) -> None:
        self._conn().write(self._write_char, bytes(frame))

    def read_battery_mv(self) -> int | None:
        self._conn()   # ensure we are subscribed before reading the buffer
        return codec.parse_ble_battery(bytes(self._buf))

    def close(self) -> None:
        """Disconnect the radio if it was ever opened. Best-effort — a BLE
        disconnect does NOT stop the robot, so callers must ensure the robot is
        already stopped BEFORE closing (the adapter's walk always does)."""
        client = self._client
        self._client = None
        if client is not None and hasattr(client, "disconnect"):
            try:
                client.disconnect()
            except Exception:
                pass

    @property
    def supports_per_joint(self) -> bool:
        return False

    @property
    def kind(self) -> str:
        return "ble"


class _BleakSyncClient:
    """Synchronous facade over an async bleak BleakClient, exposing exactly the
    two methods BleMiniHexaLink drives — start_notify(char, cb) and
    write(char, bytes) — plus disconnect(). Hardware-only glue: an asyncio loop
    runs on a daemon thread and every call is marshalled onto it and waited on.

    Writes use response=True to match the ffe1 characteristic's declared 'write'
    (with-response) property — so a stop or move write is ACKED by the peripheral
    rather than fire-and-forget, which is the safer choice for a robot whose stop
    frame must land. bleak/asyncio/threading are imported here, never at module
    load, so the offline test path never needs a Bluetooth stack.
    """

    def __init__(self, address: str, *, timeout: float = 20.0):
        import asyncio
        import threading
        from bleak import BleakClient

        self._asyncio = asyncio
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        self._client = BleakClient(address, timeout=timeout)
        # Connect synchronously; a failure here surfaces to the caller (the
        # adapter's first I/O), which becomes a tool refusal — never a silent
        # half-open link.
        self._call(self._client.connect(), timeout + 5.0)

    def _run_loop(self):
        self._asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _call(self, coro, timeout: float):
        fut = self._asyncio.run_coroutine_threadsafe(coro, self._loop)
        return fut.result(timeout)

    def start_notify(self, char: str, callback):
        self._call(self._client.start_notify(char, callback), 10.0)

    def write(self, char: str, data: bytes):
        self._call(self._client.write_gatt_char(char, data, response=True), 10.0)

    def disconnect(self):
        try:
            self._call(self._client.disconnect(), 10.0)
        finally:
            self._loop.call_soon_threadsafe(self._loop.stop)
