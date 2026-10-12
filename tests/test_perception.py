"""Tests for physicalagent/robots/perception.py — pure lidar interpretation.

summarize_scan turns a raw LaserScan dict into a compact per-sector summary
the model can reason over. Pure function, no ROS. Uses /scan_raw conventions
(360 deg, CCW, metres). Returns of <=0.12 m are the robot's own body and are
dropped; NaN/inf/out-of-range are dropped.
"""
import math

from physicalagent.robots.perception import summarize_scan


def _scan(ranges, angle_min=0.0, inc=None):
    n = len(ranges)
    inc = inc if inc is not None else (2 * math.pi / n)
    return {"ranges": ranges, "angle_min": angle_min, "angle_increment": inc}


class TestSummarizeScanSectors:
    def test_front_object_appears_in_front_sector(self):
        # 360 beams, put a 1.0 m return dead ahead (index 0 = angle 0)
        ranges = [8.0] * 360
        ranges[0] = 1.0
        out = summarize_scan(_scan(ranges, inc=math.radians(1)))
        assert out["front"]["nearest_m"] == 1.0

    def test_left_and_right_sectors_separated(self):
        ranges = [8.0] * 360
        ranges[90] = 0.5   # +90 deg = left
        ranges[270] = 0.7  # +270 = -90 = right
        out = summarize_scan(_scan(ranges, inc=math.radians(1)))
        assert out["left"]["nearest_m"] == 0.5
        assert out["right"]["nearest_m"] == 0.7

    def test_reports_nearest_and_median(self):
        ranges = [8.0] * 360
        ranges[0] = 1.0
        ranges[1] = 2.0
        ranges[359] = 3.0
        out = summarize_scan(_scan(ranges, inc=math.radians(1)))
        assert out["front"]["nearest_m"] == 1.0
        assert out["front"]["median_m"] >= 1.0


class TestSummarizeScanFiltering:
    def test_drops_self_body_returns(self):
        ranges = [8.0] * 360
        ranges[0] = 0.05  # inside the robot's body -> dropped
        ranges[1] = 2.5
        out = summarize_scan(_scan(ranges, inc=math.radians(1)))
        assert out["front"]["nearest_m"] == 2.5

    def test_drops_nan_and_inf(self):
        ranges = [8.0] * 360
        ranges[0] = float("nan")
        ranges[1] = float("inf")
        ranges[2] = 4.0
        out = summarize_scan(_scan(ranges, inc=math.radians(1)))
        assert out["front"]["nearest_m"] == 4.0

    def test_empty_sector_reports_clear(self):
        # everything far away -> still returns numbers, never crashes
        out = summarize_scan(_scan([8.0] * 360, inc=math.radians(1)))
        assert out["front"]["nearest_m"] == 8.0

    def test_all_invalid_sector_reports_no_returns(self):
        ranges = [float("nan")] * 360
        out = summarize_scan(_scan(ranges, inc=math.radians(1)))
        assert out["front"] == "no returns"


class TestSummarizeScanRobustness:
    def test_none_scan_raises_valueerror(self):
        try:
            summarize_scan(None)
            assert False, "expected ValueError"
        except ValueError:
            pass

    def test_min_front_clearance_returns_nearest_ahead(self):
        from physicalagent.robots.perception import min_front_clearance
        ranges = [8.0] * 360
        ranges[0] = 0.25  # close ahead
        assert min_front_clearance(_scan(ranges, inc=math.radians(1))) == 0.25

    def test_min_front_clearance_none_when_front_empty(self):
        from physicalagent.robots.perception import min_front_clearance
        ranges = [8.0] * 360
        for i in list(range(0, 46)) + list(range(315, 360)):  # blank the front wedge
            ranges[i] = float("nan")
        assert min_front_clearance(_scan(ranges, inc=math.radians(1))) is None
