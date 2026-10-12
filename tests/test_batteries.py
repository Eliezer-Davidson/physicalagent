"""Every battery file must be provably safe BEFORE it exists on a robot day.

A battery is the one artifact that runs unmodified against the hardware today
and against the simulator off-robot -- so its constraints are enforced here, at
test time, not discovered at the gantry. Thermal numbers are MEASURED
(2026-08-19): a raised-arm hold heats the shoulder driver at ~215 C/min
(~3.6 C/s), so raised holds are budgeted in seconds, not vibes.
"""
from __future__ import annotations

import importlib.util
import pathlib
import sys

import pytest

REPO = pathlib.Path(__file__).resolve().parents[1]
BATTERIES = sorted((REPO / "batteries").glob("*.txt"))


def _load(name):
    spec = importlib.util.spec_from_file_location(
        name, REPO / "scripts" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def sess():
    return _load("g1_sonic_session")


def test_the_standard_battery_exists():
    names = [p.name for p in BATTERIES]
    assert "sim_match_v1.txt" in names


@pytest.mark.parametrize("path", BATTERIES, ids=lambda p: p.name)
class TestEveryBatteryFile:
    def test_it_loads_whole(self, sess, path):
        sess.load_script(str(path))

    def test_it_is_bracketed_by_start_and_end_marks(self, sess, path):
        steps = sess.load_script(str(path))["steps"]
        marks = [kw["label"] for _, _, act, kw in steps if act == "mark"]
        assert marks[0] == "battery-start"
        assert marks[-1] == "battery-end"

    def test_mark_labels_are_unique(self, sess, path):
        # Duplicate labels make comparator segments ambiguous.
        steps = sess.load_script(str(path))["steps"]
        marks = [kw["label"] for _, _, act, kw in steps if act == "mark"]
        assert len(marks) == len(set(marks))

    def test_motion_stays_inside_the_signed_tier_table(self, sess, path):
        from physicalagent.robots import sonic_link as L
        from physicalagent.robots import sonic_modes as M
        for _, raw, act, kw in sess.load_script(str(path))["steps"]:
            if act == "walk":
                assert 0 <= kw["speed"] <= L.MAX_VX_M_S, raw
            elif act == "turn":
                assert abs(kw["yaw_rate"]) <= L.MAX_WZ_RAD_S, raw
            elif act == "height":
                assert M.HEIGHT_MIN_M <= kw["height_m"] <= M.HEIGHT_MAX_M, raw

    def test_arm_poses_are_ones_the_robot_has(self, sess, path):
        from physicalagent.robots import sonic_upper_body as ub
        for _, raw, act, kw in sess.load_script(str(path))["steps"]:
            if act == "arms":
                assert kw["pose"] in ub.POSES, raw

    def test_raised_arm_holds_fit_the_thermal_budget(self, sess, path):
        # 215 C/min measured on motor 22: 8 s raised is ~+29 C worst case,
        # survivable from cool. Anything longer needs a human, not a file.
        raised_since = None
        budget = 0.0
        for _, raw, act, kw in sess.load_script(str(path))["steps"]:
            if act == "arms":
                raised = kw["pose"] not in ("neutral", "arms_down")
                if raised:
                    raised_since, budget = raw, 0.0
                else:
                    raised_since = None
            elif act == "sleep" and raised_since is not None:
                budget += kw["seconds"]
                assert budget <= 8.0, (
                    f"{raised_since!r} held raised for {budget:g}s of sleep")

    def test_the_whole_battery_fits_one_sitting(self, sess, path):
        # Nominal duration: sleeps plus blocking verb durations. A battery
        # over 5 minutes stops being repeatable 2-3x in a hardware day.
        total = 0.0
        for _, raw, act, kw in sess.load_script(str(path))["steps"]:
            if act == "sleep":
                total += kw["seconds"]
            elif act in ("walk", "turn"):
                total += kw["seconds"]
        assert total <= 300.0, f"nominal {total:g}s"

    def test_it_ends_stopped(self, sess, path):
        # The last motion-capable step before battery-end must be `stop`.
        acts = [act for _, _, act, kw in sess.load_script(str(path))["steps"]
                if act not in ("mark", "sleep", "status")]
        assert acts[-1] == "stop"
