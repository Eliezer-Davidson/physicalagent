"""The frame bridge between describe_scene (robot-relative bearing/range) and
goto (world metres). Bearing convention is the sidecar's: 0 = straight ahead,
positive to the LEFT (sidecar.py _op_objects: degrees(atan2(dy,dx) - heading))."""
import math

import pytest

from physicalagent.robots import sonic_world as sw


class TestObjectWorldXy:
    def test_dead_ahead_heading_zero(self):
        assert sw.object_world_xy((0.0, 0.0), 0.0, 0.0, 2.0) == pytest.approx((2.0, 0.0))

    def test_dead_ahead_heading_90(self):
        x, y = sw.object_world_xy((0.0, 0.0), 90.0, 0.0, 2.0)
        assert (x, y) == pytest.approx((0.0, 2.0), abs=1e-9)

    def test_bearing_left_is_plus_90(self):
        x, y = sw.object_world_xy((1.0, 1.0), 0.0, 90.0, 3.0)
        assert (x, y) == pytest.approx((1.0, 4.0), abs=1e-9)

    def test_round_trip_against_sidecar_convention(self):
        # forward model of sidecar.py:459-461, then invert through the bridge
        rx, ry, heading = 0.7, -0.3, math.radians(37.0)
        ox, oy = 2.2, 1.9
        rng = math.hypot(ox - rx, oy - ry)
        bearing = math.degrees(math.atan2(oy - ry, ox - rx) - heading)
        got = sw.object_world_xy((rx, ry), math.degrees(heading), bearing, rng)
        assert got == pytest.approx((ox, oy), abs=1e-9)


class TestStandoffPoint:
    def test_stops_short_facing_object(self):
        x, y, yaw = sw.standoff_point((0.0, 0.0), (2.0, 0.0), 0.6)
        assert (x, y) == pytest.approx((1.4, 0.0))
        assert yaw == pytest.approx(0.0)

    def test_already_inside_standoff_holds_position(self):
        x, y, yaw = sw.standoff_point((1.8, 0.0), (2.0, 0.0), 0.6)
        assert (x, y) == pytest.approx((1.8, 0.0))
        assert yaw == pytest.approx(0.0)

    def test_standoff_clamped_to_040(self):
        x, y, _ = sw.standoff_point((0.0, 0.0), (2.0, 0.0), 0.05)
        assert x == pytest.approx(1.6)   # 2.0 - 0.4, not 2.0 - 0.05


class TestAugmentObjects:
    POSE = {"available": True, "fresh": True, "heading_deg": 0.0,
            "position_m": {"x": 0.0, "y": 0.0, "z": 0.79}}
    REPLY = {"objects": [{"name": "white_round_table_0", "range_m": 2.0,
                          "bearing_deg": 0.0, "height_m": -0.3}], "note": "n"}

    def test_adds_world_xy_when_fresh(self):
        out = sw.augment_objects(self.REPLY, self.POSE)
        assert out["objects"][0]["world_xy"] == pytest.approx([2.0, 0.0])
        assert "world_xy" in out["note"]

    def test_says_why_when_pose_stale(self):
        out = sw.augment_objects(self.REPLY, {"available": True, "fresh": False})
        assert "world_xy" not in out["objects"][0]
        assert "world_xy unavailable" in out["note"]

    def test_no_pose_at_all(self):
        out = sw.augment_objects(self.REPLY, None)
        assert "world_xy" not in out["objects"][0]

    def test_malformed_object_is_skipped_not_fatal(self):
        # A malformed sidecar object (missing bearing_deg/range_m) must not
        # KeyError the whole describe_scene/goto_object call -- the rest of
        # the scene still has to reach the model.
        reply = {"objects": [
            {"name": "white_round_table_0", "range_m": 2.0, "bearing_deg": 0.0,
             "height_m": -0.3},
            {"name": "broken_thing"},          # no bearing_deg/range_m
        ], "note": "n"}
        out = sw.augment_objects(reply, self.POSE)
        good, bad = out["objects"]
        assert good["world_xy"] == pytest.approx([2.0, 0.0])
        assert "world_xy" not in bad
        assert bad["name"] == "broken_thing"   # object itself left intact
        assert "broken_thing" in out["note"]   # omission is visible

    def test_malformed_object_missing_name_does_not_crash(self):
        reply = {"objects": [{"range_m": 1.0}], "note": ""}
        out = sw.augment_objects(reply, self.POSE)
        assert "world_xy" not in out["objects"][0]

    def test_world_xy_keeps_full_precision_for_the_control_path(self):
        # resolve_goto_object() consumes world_xy directly for navigation --
        # 2dp rounding on that value is a control-path precision loss, not a
        # display nicety. Values chosen so 2dp rounding would visibly differ.
        pose = {"available": True, "fresh": True, "heading_deg": 37.0,
                "position_m": {"x": 0.0, "y": 0.0, "z": 0.79}}
        reply = {"objects": [{"name": "black_thermos_0", "range_m": 1.234567,
                              "bearing_deg": 12.3456, "height_m": -0.3}],
                 "note": ""}
        out = sw.augment_objects(reply, pose)
        wx, wy = out["objects"][0]["world_xy"]
        assert (wx, wy) == pytest.approx((0.8043140065442347, 0.9366080590971663))
        assert wx != round(wx, 2) and wy != round(wy, 2)


class TestResolveGotoObject:
    def test_unknown_name_lists_what_exists(self):
        reply = {"objects": [{"name": "black_thermos_0", "world_xy": [1.0, -2.0]}]}
        with pytest.raises(ValueError) as e:
            sw.resolve_goto_object(reply, "tumbler", 0.6, robot_xy=(0.0, 0.0))
        assert "black_thermos_0" in str(e.value)

    def test_missing_world_xy_is_a_runtime_error(self):
        reply = {"objects": [{"name": "black_thermos_0"}]}
        with pytest.raises(RuntimeError):
            sw.resolve_goto_object(reply, "black_thermos_0", 0.6, robot_xy=(0.0, 0.0))

    def test_resolves_to_standoff(self):
        reply = {"objects": [{"name": "black_thermos_0", "world_xy": [2.0, 0.0]}]}
        x, y, yaw = sw.resolve_goto_object(reply, "black_thermos_0", 0.6,
                                           robot_xy=(0.0, 0.0))
        assert (x, y, yaw) == (pytest.approx(1.4), pytest.approx(0.0), pytest.approx(0.0))
