"""Tests for physicalagent/doctor.py — preflight diagnostics.

Automates the checks done by hand every session (battery, deadman, telemetry,
camera, odometry) so a bad state is caught before a task runs, not mid-motion.
Runs offline against FakeTransport + a fake camera fetcher.
"""
from physicalagent.doctor import diagnose
from physicalagent.robots.transport import FakeTransport


def _transport(batt=11800, dead="ALIVE", scan=True, odom=True):
    tele = {"batt": batt, "dead": dead}
    if scan:
        tele["scan"] = {"ranges": [1.0] * 360, "angle_min": 0.0, "angle_increment": 0.017}
    if odom:
        tele["odom"] = {"pose": {"pose": {"position": {"x": 0, "y": 0}, "orientation": {"w": 1, "x": 0, "y": 0, "z": 0}}}}
    return FakeTransport(telemetry=tele)


def _find(report, name):
    return next(c for c in report.checks if c.name == name)


class TestDoctorHealthy:
    def test_all_green_when_healthy(self):
        rep = diagnose(_transport(), frame_fetcher=lambda: b"\xff\xd8" + b"x" * 5000, settle_s=0)
        assert rep.ok is True
        assert all(c.ok for c in rep.checks)

    def test_reports_battery_voltage(self):
        rep = diagnose(_transport(batt=11800), frame_fetcher=lambda: b"\xff\xd8xx", settle_s=0)
        batt = _find(rep, "battery")
        assert batt.ok
        assert "11.8" in batt.detail or "11800" in batt.detail


class TestDoctorCatchesProblems:
    def test_low_battery_fails_check(self):
        rep = diagnose(_transport(batt=11000), frame_fetcher=lambda: b"\xff\xd8xx", settle_s=0)
        assert rep.ok is False
        assert _find(rep, "battery").ok is False

    def test_missing_battery_telemetry_fails(self):
        t = FakeTransport(telemetry={"dead": "ALIVE"})  # no batt
        rep = diagnose(t, frame_fetcher=lambda: b"\xff\xd8xx", settle_s=0)
        assert _find(rep, "battery").ok is False

    def test_deadman_not_running_fails(self):
        t = FakeTransport(telemetry={"batt": 11800})  # no dead status
        rep = diagnose(t, frame_fetcher=lambda: b"\xff\xd8xx", settle_s=0)
        c = _find(rep, "deadman")
        assert c.ok is False
        assert "not running" in c.detail.lower() or "no status" in c.detail.lower()

    def test_deadman_paused_is_warned(self):
        rep = diagnose(_transport(dead="PAUSED"), frame_fetcher=lambda: b"\xff\xd8xx", settle_s=0)
        c = _find(rep, "deadman")
        # PAUSED is not a failure per se, but must be surfaced
        assert "PAUSED" in c.detail

    def test_no_odometry_fails_turn_capability(self):
        rep = diagnose(_transport(odom=False), frame_fetcher=lambda: b"\xff\xd8xx", settle_s=0)
        c = _find(rep, "odometry")
        assert c.ok is False
        assert "turn" in c.detail.lower() or "rf2o" in c.detail.lower()

    def test_camera_failure_fails_check(self):
        def broken():
            raise OSError("connection refused")
        rep = diagnose(_transport(), frame_fetcher=broken, settle_s=0)
        assert _find(rep, "camera").ok is False

    def test_tiny_frame_fails_camera(self):
        rep = diagnose(_transport(), frame_fetcher=lambda: b"\xff\xd8", settle_s=0)  # 2 bytes
        assert _find(rep, "camera").ok is False


class TestMalformedLaserScanDoesNotCrashPreflight:
    """REGRESSION: the lidar check tested `bool(scan)` and then indexed
    `scan['ranges']`. Truthiness only proves a message arrived, not that it is
    a well-formed LaserScan -- so a message missing that key raised KeyError
    out of the middle of preflight.

    It got worse on this branch: main()'s handler was narrowed from
    `except KeyError` to `except UnknownRobotError`, so what used to be
    swallowed (and mislabelled "unknown robot") now escapes as a raw traceback.
    tests/test_cli.py cites this exact line as the motivating example.
    """

    def test_scan_without_a_ranges_key_is_a_failed_check_not_a_keyerror(self):
        t = FakeTransport(telemetry={"batt": 11800, "dead": "ALIVE",
                                     "scan": {"angle_min": 0.0}})     # no 'ranges'
        rep = diagnose(t, frame_fetcher=lambda: b"\xff\xd8xx", settle_s=0)
        assert _find(rep, "lidar").detail == "0 rays"

    def test_a_wellformed_scan_still_counts_its_rays(self):
        rep = diagnose(_transport(), frame_fetcher=lambda: b"\xff\xd8xx", settle_s=0)
        assert _find(rep, "lidar").detail == "360 rays"


class TestBatteryThresholdsComeFromTheProfile:
    """A 2S robot must not be reported flat at a perfectly healthy 7.4 V."""

    def _two_s(self):
        from dataclasses import replace
        from physicalagent.safety import ROSPUG_SAFETY
        return replace(ROSPUG_SAFETY, name="2s",
                       battery_motion_min_mv=7400, battery_warn_mv=7000)

    def test_a_2s_pack_at_7400_passes_against_its_own_profile(self):
        rep = diagnose(_transport(batt=7400), frame_fetcher=lambda: b"\xff\xd8xx",
                       settle_s=0, profile=self._two_s())
        assert _find(rep, "battery").ok is True

    def test_the_same_pack_fails_against_the_rospugs_3s_floor(self):
        """Confirms the previous test is discriminating, not vacuous."""
        rep = diagnose(_transport(batt=7400), frame_fetcher=lambda: b"\xff\xd8xx",
                       settle_s=0)
        assert _find(rep, "battery").ok is False

    def test_the_rospug_default_is_unchanged(self):
        rep = diagnose(_transport(batt=11800), frame_fetcher=lambda: b"\xff\xd8xx",
                       settle_s=0)
        assert _find(rep, "battery").ok is True
        assert "ok" in _find(rep, "battery").detail

    def test_a_mains_powered_robot_reports_no_pack_instead_of_no_telemetry(self):
        from dataclasses import replace
        from physicalagent.safety import NO_BATTERY, ROSPUG_SAFETY
        mains = replace(ROSPUG_SAFETY, name="arm",
                        battery_motion_min_mv=NO_BATTERY, battery_warn_mv=NO_BATTERY)
        t = FakeTransport(telemetry={"dead": "ALIVE"})     # no battery topic at all
        rep = diagnose(t, frame_fetcher=lambda: b"\xff\xd8xx", settle_s=0, profile=mains)
        c = _find(rep, "battery")
        assert c.ok is True
        assert "mains" in c.detail


class TestDoctorReport:
    def test_report_summary_lists_failures(self):
        rep = diagnose(_transport(batt=10000), frame_fetcher=lambda: b"\xff\xd8xx", settle_s=0)
        text = rep.summary()
        assert "battery" in text.lower()
        assert "FAIL" in text or "✗" in text
