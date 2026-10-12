"""goto_object and the world_xy augmentation, on a bare adapter instance:
__new__ skips the constructor so only the attributes the handlers actually
touch (_sim, _telemetry, _goto) need faking."""
import pytest

from physicalagent.robots.sonic_g1 import SonicG1Adapter

POSE = {"available": True, "fresh": True, "heading_deg": 0.0,
        "position_m": {"x": 0.0, "y": 0.0, "z": 0.79}}
OBJECTS = {"objects": [
    {"name": "white_round_table_0", "range_m": 2.0, "bearing_deg": 0.0,
     "height_m": -0.33},
    {"name": "black_thermos_0", "range_m": 2.1, "bearing_deg": 5.0,
     "height_m": -0.30}], "note": "n"}


class FakeSim:
    def objects(self):
        return OBJECTS


class FakeTelemetry:
    def pose(self):
        return POSE


def _adapter():
    a = SonicG1Adapter.__new__(SonicG1Adapter)
    a._sim = FakeSim()
    a._telemetry = FakeTelemetry()
    return a


def test_describe_scene_gains_world_xy():
    out = _adapter()._describe_scene()
    table = next(o for o in out["objects"] if o["name"] == "white_round_table_0")
    assert table["world_xy"] == pytest.approx([2.0, 0.0])


def test_goto_object_delegates_to_goto_at_the_standoff():
    a = _adapter()
    seen = {}
    a._goto = lambda x, y, yaw_deg=None, timeout_s=None: (
        seen.update(x=x, y=y, yaw=yaw_deg) or {"ok": True, "final_error_m": 0.1})
    res = a._goto_object("white_round_table_0")
    assert seen["x"] == pytest.approx(1.4)
    assert seen["y"] == pytest.approx(0.0)
    assert res["object"] == "white_round_table_0"
    assert res["object_world_xy"] == pytest.approx([2.0, 0.0])


def test_goto_object_unknown_name_lists_scene():
    with pytest.raises(ValueError) as e:
        _adapter()._goto_object("tumbler")
    assert "black_thermos_0" in str(e.value)


def test_goto_object_declared_and_wired():
    a = _adapter()
    names = {t.name for t in SonicG1Adapter.capabilities(a)}
    assert "goto_object" in names
    assert "goto_object" in a._handlers()
    assert "goto_object" in a._motion_tool_names()
