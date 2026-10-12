"""Arm kinematics: the table against the vendor MJCF, FK against geometry, and
IK against FK.

The table test is the one that matters. This project shipped a hand-copied
kinematic ordering that was wrong and passed a full suite because every test
compared it to another copy we had made ourselves. So LINKS is re-derived here
from the vendor XML with ElementTree, and nothing about it is taken on trust.

IK is checked by round trip -- pick a reachable joint configuration, run FK to
get a target, solve, and require the solution to land on the same POINT. Not
the same angles: the arm is redundant, so a different elbow that reaches the
same place is a correct answer, and asserting on angles would fail a working
solver.
"""
from __future__ import annotations

import math
import pathlib
import random
import xml.etree.ElementTree as ET

import pytest

from physicalagent.robots import sonic_arm_kin as kin
from physicalagent.robots import sonic_upper_body as ub

from physicalagent.paths import G1_MODEL_DIR

MJCF = G1_MODEL_DIR / "g1_29dof_with_inspire_actuated.xml"


def mjcf_joints():
    """{joint_name: (body, pos, quat, axis, range)} straight from the vendor."""
    root = ET.parse(MJCF).getroot().find("worldbody")
    out = {}

    def walk(e):
        for b in e.findall("body"):
            for j in b.findall("joint"):
                if j.get("name"):
                    def nums(s, d):
                        return tuple(float(v) for v in s.split()) if s else d
                    out[j.get("name")] = (
                        b.get("name"), nums(b.get("pos"), (0.0, 0.0, 0.0)),
                        nums(b.get("quat"), (1.0, 0.0, 0.0, 0.0)),
                        nums(j.get("axis"), (0.0, 0.0, 1.0)),
                        nums(j.get("range"), None))
            walk(b)

    walk(root)
    return out


@pytest.mark.skipif(not MJCF.exists(), reason="vendor MJCF absent")
class TestTableIsTheVendors:
    def test_link_table_matches_the_vendor_mjcf(self):
        vendor = mjcf_joints()
        for side, chain in kin.LINKS.items():
            for link in chain:
                body, pos, quat, axis, rng = vendor[link.joint + "_joint"]
                assert link.body == body, link.joint
                assert link.pos == pytest.approx(pos), link.joint
                assert link.quat == pytest.approx(quat), link.joint
                assert link.axis == pytest.approx(axis), link.joint
                assert link.range == pytest.approx(rng), link.joint

    def test_every_arm_joint_is_one_we_can_actually_command(self):
        """Solving for a joint the wire cannot carry would be a solver that
        produces unusable answers."""
        for side, chain in kin.LINKS.items():
            for link in chain:
                assert link.joint in ub.INDEX, link.joint

    def test_palm_offset_matches_the_vendor_finger_roots(self):
        """PALM_OFFSET must be the finger roots composed THROUGH the
        intermediate hand_base_link, in the wrist frame.

        Reading the finger `pos` attributes without composing gives
        (0, -0.137, 0) -- a different axis and the wrong sign -- which is
        exactly the bug this test now pins.
        """
        root = ET.parse(MJCF).getroot().find("worldbody")

        def nums(s, d):
            return tuple(float(v) for v in s.split()) if s else d

        for side, wrist_body in (("left", "left_wrist_yaw_link"),
                                 ("right", "right_wrist_yaw_link")):
            roots = {}

            def walk(e, rot, pos, inside):
                for b in e.findall("body"):
                    nm = b.get("name", "")
                    off = kin.mat_vec(rot, nums(b.get("pos"), (0.0, 0.0, 0.0)))
                    p2 = tuple(pos[i] + off[i] for i in range(3))
                    r2 = kin.mat_mul(rot, kin.quat_to_mat(
                        nums(b.get("quat"), (1.0, 0.0, 0.0, 0.0))))
                    if nm == wrist_body:
                        walk(b, (1, 0, 0, 0, 1, 0, 0, 0, 1), (0.0, 0.0, 0.0), True)
                        continue
                    if inside and nm.endswith("_proximal") and any(
                            f in nm for f in ("index", "middle", "ring", "pinky")):
                        roots[nm] = p2
                    walk(b, r2, p2, inside)

            walk(root, (1, 0, 0, 0, 1, 0, 0, 0, 1), (0.0, 0.0, 0.0), False)
            assert len(roots) == 4, f"{side}: found {sorted(roots)}"
            mean = tuple(sum(v[i] for v in roots.values()) / 4 for i in range(3))
            assert kin.PALM_OFFSET[side] == pytest.approx(mean, abs=1e-4)

    def test_limits_agree_with_the_ones_the_command_path_clamps_to(self):
        for side, chain in kin.LINKS.items():
            for link in chain:
                i = ub.INDEX[link.joint]
                assert link.range[0] == pytest.approx(ub.LOWER[i], abs=1e-3)
                assert link.range[1] == pytest.approx(ub.UPPER[i], abs=1e-3)


class TestLinearAlgebra:
    def test_quat_identity_is_identity(self):
        assert kin.quat_to_mat((1, 0, 0, 0)) == pytest.approx(
            (1, 0, 0, 0, 1, 0, 0, 0, 1))

    def test_rotation_about_z_sends_x_to_y(self):
        m = kin.axis_angle_to_mat((0, 0, 1), math.pi / 2)
        assert kin.mat_vec(m, (1, 0, 0)) == pytest.approx((0, 1, 0), abs=1e-9)

    def test_solve3_solves(self):
        m = (2.0, 1.0, 0.0, 1.0, 3.0, 1.0, 0.0, 1.0, 4.0)
        x = kin.solve3(m, (5.0, 10.0, 14.0))
        for r in range(3):
            assert sum(m[r * 3 + c] * x[c] for c in range(3)) == pytest.approx(
                (5.0, 10.0, 14.0)[r])

    def test_singular_returns_none_rather_than_a_huge_step(self):
        assert kin.solve3((1, 2, 3, 2, 4, 6, 3, 6, 9), (1, 2, 3)) is None


class TestForwardKinematics:
    def test_zero_pose_puts_the_hands_where_a_G1_has_hands(self):
        """Sanity against the physical robot rather than against our own maths:
        arms hanging by the sides, hands below the shoulder, left hand on the
        left."""
        left = kin.fk([0.0] * 10, "left")["palm_pos"]
        right = kin.fk([0.0] * 10, "right")["palm_pos"]
        assert left[1] > 0.05, "left hand should be on the +y (left) side"
        assert right[1] < -0.05, "right hand should be on the -y side"
        assert 0.0 < left[2] < 0.4, f"hand at z={left[2]:.2f} is not by the hip"
        assert left[0] == pytest.approx(right[0], abs=0.02)
        assert left[2] == pytest.approx(right[2], abs=0.02)

    @pytest.mark.parametrize("point", ["wrist_pos", "palm_pos"])
    def test_the_two_arms_are_mirror_images(self, point):
        """Same angles mirrored must give a mirrored hand.

        This is the test that caught PALM_OFFSET being read in the wrong frame:
        the wrist mirrored to 1e-5 while the palm did not mirror at all, because
        the same offset was moving the left hand inboard and the right outboard.
        The 1e-4 tolerance is the robot's own asymmetry -- the shoulder y offsets
        are 0.10022 and -0.10021.
        """
        q = [0.0, 0.0, 0.0, -0.6, 0.4, 0.2, 0.9, 0.1, 0.2, 0.0]
        qm = [0.0, 0.0, 0.0, -0.6, -0.4, -0.2, 0.9, -0.1, 0.2, 0.0]
        left = kin.fk(q, "left")[point]
        right = kin.fk(qm, "right")[point]
        assert left[0] == pytest.approx(right[0], abs=1e-4)
        assert left[1] == pytest.approx(-right[1], abs=1e-4)
        assert left[2] == pytest.approx(right[2], abs=1e-4)

    def test_more_negative_shoulder_pitch_lifts_the_hand(self):
        """The sign convention the pose library also depends on.

        Stated as monotonicity rather than as a distance. An earlier version
        asserted the hand moves 0.2 m FORWARD as well, which is simply not what
        this robot does -- at zero the arm already hangs forward-and-down, so
        pitching it up mostly gains height (+0.35 m) and barely any reach
        (+0.03 m). The test was wrong about the robot, not the other way round.
        """
        heights = [kin.fk([0, 0, 0, p, 0, 0, 0, 0, 0, 0], "left")["palm_pos"][2]
                   for p in (0.0, -0.4, -0.8, -1.2, -1.6)]
        assert heights == sorted(heights), heights
        assert heights[-1] > heights[0] + 0.3

    def test_waist_yaw_swings_the_whole_arm_round(self):
        a = kin.fk([0.0] * 10, "left")["palm_pos"]
        b = kin.fk([1.0] + [0.0] * 9, "left")["palm_pos"]
        assert math.dist(a, b) > 0.1

    def test_arm_cannot_be_further_than_its_own_span(self):
        _, span = kin.reach_limits("left")
        for _ in range(200):
            q = [0.0, 0.0, 0.0] + [random.uniform(l.range[0], l.range[1])
                                   for l in kin.LINKS["left"][3:]]
            p = kin.fk(q, "left")["palm_pos"]
            shoulder = kin.fk(q, "left")["anchors"][3]
            assert math.dist(p, shoulder) <= span + 1e-6

    def test_wrong_length_input_is_refused(self):
        with pytest.raises(ValueError):
            kin.fk([0.0] * 7, "left")


class TestInverseKinematics:
    @pytest.mark.parametrize("side", ["left", "right"])
    def test_round_trip_from_reachable_poses(self, side):
        """FK a random reachable configuration, then require IK to reach that
        POINT. Not those angles -- a 7-DOF arm has many ways to get there."""
        rng = random.Random(7)
        arm = kin.LINKS[side][3:]
        misses = []
        for _ in range(40):
            q = [rng.uniform(l.range[0] * 0.6, l.range[1] * 0.6) for l in arm]
            target = kin.fk([0.0, 0.0, 0.0] + q, side)["palm_pos"]
            out = kin.ik(target, side)
            if not out["ok"]:
                misses.append((target, out["error_m"]))
        assert not misses, f"{len(misses)}/40 missed: {misses[:3]}"

    def test_the_solution_is_within_joint_limits(self):
        rng = random.Random(11)
        arm = kin.LINKS["left"][3:]
        for _ in range(20):
            q = [rng.uniform(l.range[0] * 0.6, l.range[1] * 0.6) for l in arm]
            target = kin.fk([0.0, 0.0, 0.0] + q, "left")["palm_pos"]
            out = kin.ik(target, "left")
            for i, l in enumerate(arm):
                assert l.range[0] - 1e-9 <= out["values"][i] <= l.range[1] + 1e-9

    def test_solving_actually_moves_toward_a_target_it_cannot_reach(self):
        """An unreachable point must report ok=False with the distance it got
        to, not silently return the seed pose as though it had succeeded."""
        out = kin.ik((3.0, 0.0, 0.0), "left")
        assert out["ok"] is False
        assert out["error_m"] > 1.0
        # ...and it should still have stretched toward it
        assert out["palm_pos"][0] > 0.2

    def test_a_target_inside_the_torso_is_reported_not_forced(self):
        out = kin.ik((0.0, 0.0, 0.2), "left")
        assert "error_m" in out and out["error_m"] >= 0.0

    def test_seed_is_respected_and_clamped(self):
        out = kin.ik((0.25, 0.25, 0.1), "left", seed=[99.0] * 7)
        arm = kin.LINKS["left"][3:]
        for i, l in enumerate(arm):
            assert out["values"][i] <= l.range[1] + 1e-9

    def test_joints_come_back_named_so_they_can_be_commanded_directly(self):
        out = kin.ik((0.3, 0.2, 0.1), "left")
        assert set(out["joints"]) == {l.joint for l in kin.LINKS["left"][3:]}
        for name in out["joints"]:
            assert name in ub.INDEX

    def test_waist_angle_changes_the_answer(self):
        """If the waist were ignored rather than held, these would agree and the
        arm would be solved in the wrong frame whenever the torso was turned."""
        a = kin.ik((0.3, 0.2, 0.1), "left", waist=(0.0, 0.0, 0.0))
        b = kin.ik((0.3, 0.2, 0.1), "left", waist=(0.8, 0.0, 0.0))
        assert a["values"] != pytest.approx(b["values"])
        assert a["ok"] and b["ok"]
