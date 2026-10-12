"""Bring-up probe — capture first, analyse later.

The project's own rule, learned the hard way: artifacts cannot be re-derived
from insight. When a robot is on the bench, copy everything; work out what it
means afterwards.
"""
from __future__ import annotations

from dataclasses import asdict
from pathlib import Path

from physicalagent.probe.capture import CaptureWriter, capture_dir
from physicalagent.probe.net import classify, scan_ports
from physicalagent.probe.ros import dump_graph, summarise_samples
from physicalagent.probe.usb import parse_lsusb, serial_ports


def run_probe(host: str, robot: str, base_dir, stamp: str,
              lsusb_text: str | None = None, serial=None,
              port_probe=None, ros_call=None, ros_samples=None) -> Path:
    """Capture everything about whatever is plugged in, and return the folder.

    Every I/O source is injectable so the whole orchestration is unit-tested
    offline. Defaults hit the real machine.
    """
    if lsusb_text is None:
        import subprocess
        try:
            lsusb_text = subprocess.run(["lsusb"], capture_output=True,
                                        text=True, timeout=10).stdout
        except Exception as exc:
            lsusb_text = f"lsusb failed: {exc}"
    if serial is None:
        serial = serial_ports()

    usb = parse_lsusb(lsusb_text)
    ports = scan_ports(host, probe=port_probe)
    conclusions = classify(ports)
    graph = dump_graph(ros_call) if ros_call is not None else {"skipped": "no ROS link"}
    telemetry = summarise_samples(ros_samples) if ros_samples else {}

    writer = CaptureWriter(capture_dir(Path(base_dir), robot, stamp))
    writer.write_text("lsusb.txt", lsusb_text)
    writer.write_manifest({
        "robot": robot,
        "host": host,
        "stamp": stamp,
        "usb": [asdict(d) for d in usb],
        "serial_ports": list(serial),
        "ports": [asdict(p) for p in ports],
        "conclusions": list(conclusions),
        "ros_graph": graph,
        "telemetry": telemetry,
    })
    return writer.path
