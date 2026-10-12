"""USB/serial identification for the bring-up probe.

VID:PID pairs are VERIFIED -- these are the bridge chips the six robots in the
fleet actually use. Anything not in the table is reported as UNKNOWN rather
than guessed at; a wrong guess here sends the operator down a blind alley.
"""
from __future__ import annotations

import glob
import re
from dataclasses import dataclass

# VERIFIED (spec section 12, bring-up kit)
KNOWN_DEVICES: dict[str, str] = {
    "0d28:0204": "micro:bit V2 (DAPLink)",   # AiHand's controller
    "1a86:7523": "CH340 serial bridge",       # common on ESP32 boards
    "10c4:ea60": "CP2102 serial bridge",
    "0403:6001": "FTDI FT232 serial bridge",
}

_LSUSB_RE = re.compile(
    r"^Bus\s+\d+\s+Device\s+\d+:\s+ID\s+([0-9a-fA-F]{4}):([0-9a-fA-F]{4})\s*(.*)$"
)


@dataclass(frozen=True)
class UsbDevice:
    vid: str
    pid: str
    label: str
    description: str


def identify(vid: str, pid: str) -> str:
    """Human label for a VID:PID, or the literal 'UNKNOWN'."""
    return KNOWN_DEVICES.get(f"{vid.lower()}:{pid.lower()}", "UNKNOWN")


def parse_lsusb(text: str) -> tuple[UsbDevice, ...]:
    """Parse `lsusb` output. Unparseable lines are skipped, never fatal."""
    found = []
    for line in text.splitlines():
        m = _LSUSB_RE.match(line.strip())
        if m is None:
            continue
        vid, pid, description = m.group(1).lower(), m.group(2).lower(), m.group(3).strip()
        found.append(UsbDevice(vid=vid, pid=pid, label=identify(vid, pid),
                               description=description))
    return tuple(found)


def serial_ports() -> tuple[str, ...]:
    """Every serial device currently present, sorted."""
    return tuple(sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")))
