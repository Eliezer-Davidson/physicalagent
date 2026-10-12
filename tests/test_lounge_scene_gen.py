"""Pins the generator's contracts that do not need mujoco: the robot-hint list
must be the sidecar's (a drifted copy silently hides furniture from
describe_scene), the banner must survive base_sim's XML round-trip, and the
clearance arithmetic must be exact."""
import pathlib
import re

import pytest

from simbridge import make_lounge_scene as gen

REPO = pathlib.Path(__file__).resolve().parents[1]


def test_robot_hints_match_sidecar_source():
    src = (REPO / "simbridge" / "sidecar.py").read_text()
    m = re.search(r"_ROBOT_HINTS = \((.*?)\)", src, re.S)
    assert m, "sidecar.py no longer defines _ROBOT_HINTS where expected"
    sidecar_hints = tuple(re.findall(r'"([^"]+)"', m.group(1)))
    assert gen._ROBOT_HINTS == sidecar_hints


def test_banner_survives_xml_comment_rules():
    assert "--" not in gen.BANNER


def test_furniture_names_pass_the_hint_filter():
    # KNOWN_BODIES, not FURNITURE_BODIES: the hint filter is a name-shape
    # check that must hold for every body the source scene ships, including
    # the excluded backpack (it's just not extracted, its name is fine).
    for name in gen.KNOWN_BODIES:
        assert not any(name.startswith(h) for h in gen._ROBOT_HINTS), name
        # substring form too: the classifier the sidecar applies is prefix-based
        # today, but a name CONTAINING a hint is one refactor from invisible.
        assert not any(h in name for h in gen._ROBOT_HINTS), name


def test_backpack_excluded_no_settled_pose_in_any_variant():
    # black_backpack_0 tumbles ~2.9-3.2 m in bare mj_step in every measured
    # reconstructed variant (2026-08-12) - it has no equilibrium pose to ship.
    assert "black_backpack_0" not in gen.FURNITURE_BODIES
    assert len(gen.FURNITURE_BODIES) == 5


def test_leg_joint_regex_matches_what_base_sim_asserts_on():
    assert gen.LEG_JOINT_RE.search("left_hip_pitch_joint")
    assert not gen.LEG_JOINT_RE.search("white_round_table_0_free")


class TestNeededShift:
    def test_no_shift_when_already_clear(self):
        assert gen.needed_shift_m(1.5) == 0.0

    def test_shift_makes_up_the_difference_rounded_up_to_cm(self):
        assert gen.needed_shift_m(0.43) == pytest.approx(0.77)

    def test_negative_clearance_counts_from_zero(self):
        assert gen.needed_shift_m(-0.1) == pytest.approx(1.30)
