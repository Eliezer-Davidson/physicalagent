"""Preflight diagnostics — automate the by-hand checks done every session.

Every session so far has started with the same manual dance: is the battery
above the floor? is the deadman actually running (not just pgrep matching
itself)? is rf2o publishing so turns can close the loop? is the camera alive?
diagnose() runs all of it and returns a report, so a bad state is caught before
a task commands motion, not mid-motion.

Pure logic over an injected Transport + camera fetcher, so it is fully tested
offline; the CLI wires the real RosbridgeTransport.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field

from physicalagent import config
from physicalagent.safety import ROSPUG_SAFETY, battery_state


@dataclass
class Check:
    name: str
    ok: bool
    detail: str


@dataclass
class Report:
    checks: list[Check] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return all(c.ok for c in self.checks)

    def summary(self) -> str:
        lines = ["PhysicalAgent preflight:"]
        for c in self.checks:
            mark = "✓ OK  " if c.ok else "✗ FAIL"
            lines.append(f"  {mark} {c.name:12s} {c.detail}")
        lines.append("READY" if self.ok else "NOT READY — resolve the failures above")
        return "\n".join(lines)


def diagnose(transport, frame_fetcher, settle_s: float = 3.0,
             min_frame_bytes: int = 3000, profile=ROSPUG_SAFETY) -> Report:
    """Subscribe, let telemetry arrive, and check each subsystem.

    `profile` supplies the battery thresholds, so a robot on a different pack
    is not judged against the ROSPug's 3S floor. It still defaults to the
    ROSPug because the topic names below are the ROSPug's; making the rest of
    this function profile-driven belongs with the per-robot doctor dispatch.
    """
    transport.subscribe(config.ROSPUG.topic_battery, "std_msgs/UInt16", "batt")
    transport.subscribe("/physicalagent/deadman_status", "std_msgs/String", "dead")
    transport.subscribe("/scan_raw", "sensor_msgs/LaserScan", "scan")
    transport.subscribe("/odom_raw", "nav_msgs/Odometry", "odom")
    if settle_s:
        time.sleep(settle_s)

    rep = Report()

    # battery
    floor = profile.battery_motion_min_mv
    mv = transport.latest("batt")
    if floor is None:
        rep.checks.append(Check("battery", True, "mains-powered (no pack to check)"))
    elif mv is None:
        rep.checks.append(Check("battery", False, "no telemetry (is the robot publishing?)"))
    elif mv != mv:
        rep.checks.append(Check("battery", False, "reading is NaN"))
    elif mv < floor:
        rep.checks.append(Check("battery", False,
                                f"{mv/1000:.2f} V ({mv} mV) below floor {floor} — CHARGE"))
    else:
        rep.checks.append(Check("battery", True,
                                f"{mv/1000:.2f} V ({mv} mV) {battery_state(mv, profile)}"))

    # deadman
    dead = transport.latest("dead")
    if dead is None:
        rep.checks.append(Check("deadman", False,
                                "no status topic — deadman node not running (relaunch it)"))
    elif dead == "PAUSED":
        rep.checks.append(Check("deadman", True,
                                "PAUSED (link was lost / no heartbeat yet — resumes on heartbeat)"))
    else:
        rep.checks.append(Check("deadman", True, dead))

    # lidar
    # scan.get(), not scan['ranges']: truthiness only proves a message arrived,
    # not that it is a well-formed LaserScan. A message missing 'ranges' used to
    # raise KeyError straight out of preflight -- and since main() narrowed its
    # handler to UnknownRobotError, that now escapes as a raw traceback.
    scan = transport.latest("scan")
    rep.checks.append(Check("lidar", bool(scan),
                            f"{len(scan.get('ranges', []))} rays" if scan
                            else "no /scan_raw"))

    # odometry (needed for closed-loop turn)
    odom = transport.latest("odom")
    rep.checks.append(Check("odometry", bool(odom),
                            "/odom_raw live" if odom
                            else "no /odom_raw — turns will refuse (launch rf2o_laser_odometry)"))

    # camera
    try:
        frame = frame_fetcher()
        n = len(frame or b"")
        rep.checks.append(Check("camera", n >= min_frame_bytes,
                                f"{n} byte frame" if n >= min_frame_bytes
                                else f"frame too small ({n} bytes)"))
    except Exception as exc:
        rep.checks.append(Check("camera", False, f"fetch failed: {exc}"))

    return rep
