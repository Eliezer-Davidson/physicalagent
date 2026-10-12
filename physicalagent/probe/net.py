"""Network surface probing for the bring-up kit.

The socket call is injected so the parsers and the classifier are testable
with no network at all -- the same discipline the Transport ABC uses.
"""
from __future__ import annotations

import socket
from dataclasses import dataclass

# VERIFIED port meanings for the fleet.
INTERESTING_PORTS: tuple[tuple[int, str], ...] = (
    (22, "SSH"),
    (80, "HTTP"),
    (4000, "NoMachine (ROSPug GUI)"),
    (8080, "MJPEG video / web_video_server"),
    (9030, "TonyPi JSON-RPC"),
    (9090, "rosbridge WebSocket"),
    (9027, "MechDog UDP control (TCP check only)"),
)


@dataclass(frozen=True)
class PortResult:
    port: int
    open: bool
    meaning: str


def _tcp_probe(host: str, port: int, timeout: float) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(timeout)
        return s.connect_ex((host, port)) == 0


def scan_ports(host: str, ports=INTERESTING_PORTS, probe=None,
               timeout_s: float = 1.0) -> tuple[PortResult, ...]:
    """Check each port. A probe that raises is recorded as closed, never fatal --
    a half-finished capture is worth more than a traceback."""
    probe = probe or _tcp_probe
    results = []
    for port, meaning in ports:
        try:
            is_open = bool(probe(host, port, timeout_s))
        except Exception:
            is_open = False
        results.append(PortResult(port=port, open=is_open, meaning=meaning))
    return tuple(results)


def classify(results) -> tuple[str, ...]:
    """Turn open ports into plain-language conclusions for the operator."""
    open_ports = {r.port for r in results if r.open}
    notes = []
    if 9090 in open_ports:
        notes.append("rosbridge is up — this is a ROS robot; dump the graph next")
    if 9030 in open_ports:
        notes.append("port 9030 answers — this is a TonyPi (JSON-RPC over HTTP)")
    if 8080 in open_ports:
        notes.append("MJPEG video server is up — camera frames are available over HTTP")
    if 22 in open_ports:
        notes.append("SSH is open — vendor source can be copied off the robot")
    if not notes:
        notes.append("no known service ports answered — check the link, or it is a "
                     "serial-only robot (MiniHexa / MechDog / AiHand)")
    return tuple(notes)
