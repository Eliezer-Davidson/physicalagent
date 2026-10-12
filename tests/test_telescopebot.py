"""Tests for the TelescopeBot spine scaffold.

The twin-screw spine is driven OPEN LOOP: continuous-rotation servos have no
position feedback, so height tracking is nothing but lead-screw arithmetic
times wall-clock. That makes the arithmetic itself safety-relevant — the
original draft assumed a T8 lead screw's 8 mm/rev on the M8 threaded rod the
robot actually uses (1.25 mm/rev), a 6.4x error that would overrun every
height target and drive the chest carriage into its hard stop. What this
file pins:

  * the travel rate derives from the M8 THREAD PITCH, not the rod diameter;
  * height requests clamp to the physical envelope [0, SPINE_MAX_HEIGHT_CM];
  * the returned plan (travel_cm + duration_s) is consistent with the rate;
  * spine motion is a motion tool, so sensing-only profiles withhold it.
"""
import pytest

from physicalagent.robots.telescopebot import (
    SPINE_LEAD_MM_PER_REV,
    SPINE_MAX_HEIGHT_CM,
    SPINE_TRAVEL_CM_PER_S,
    TelescopeBotAdapter,
)


class TestSpineLeadMath:
    def test_lead_is_the_m8_thread_pitch_not_the_rod_diameter(self):
        # M8 coarse pitch is 1.25 mm/rev. 8.0 here means someone confused
        # the rod's DIAMETER (or a T8 lead screw) with its lead again.
        assert SPINE_LEAD_MM_PER_REV == 1.25
        assert SPINE_LEAD_MM_PER_REV != 8.0

    def test_full_extension_plan_is_200_seconds(self):
        a = TelescopeBotAdapter()
        r = a.execute("set_spine_height", {"height_cm": SPINE_MAX_HEIGHT_CM})
        assert r.ok
        assert r.payload["duration_s"] == pytest.approx(200.0)

    def test_duration_scales_linearly_with_travel(self):
        a = TelescopeBotAdapter()
        r = a.execute("set_spine_height", {"height_cm": 10.0})
        assert r.ok
        assert r.payload["travel_cm"] == pytest.approx(10.0)
        assert r.payload["duration_s"] == pytest.approx(10.0 / SPINE_TRAVEL_CM_PER_S)

    def test_second_move_plans_from_current_height_not_zero(self):
        a = TelescopeBotAdapter()
        assert a.execute("set_spine_height", {"height_cm": 25.0}).ok
        r = a.execute("set_spine_height", {"height_cm": 20.0})
        assert r.ok
        assert r.payload["travel_cm"] == pytest.approx(-5.0)
        assert r.payload["duration_s"] == pytest.approx(40.0)


class TestSpineEnvelope:
    def test_over_travel_clamps_to_max(self):
        a = TelescopeBotAdapter()
        r = a.execute("set_spine_height", {"height_cm": 40.0})
        assert r.ok
        assert a.telemetry()["spine_height_cm"] == SPINE_MAX_HEIGHT_CM

    def test_negative_height_clamps_to_zero(self):
        a = TelescopeBotAdapter()
        r = a.execute("set_spine_height", {"height_cm": -3.0})
        assert r.ok
        assert a.telemetry()["spine_height_cm"] == 0.0
        assert r.payload["duration_s"] == 0.0


class TestToolSurface:
    def test_spine_and_actions_are_motion_tools(self):
        a = TelescopeBotAdapter()
        assert {"set_spine_height", "do_action"} <= a._motion_tool_names()

    def test_look_is_not_a_motion_tool(self):
        a = TelescopeBotAdapter()
        assert "look" not in a._motion_tool_names()
