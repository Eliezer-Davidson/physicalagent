"""rt/odostate is the ONLY source of pelvis height in this stack. If it is ever
wrong or invented, the fall detector, _posture() and the whole get-up route are
wrong with it -- so these rules get pinned offline."""
from __future__ import annotations

import math

import pytest

from simbridge.odostate_fields import (odostate_fields, unit_quat_wxyz,
                                       world_ang_vel_to_body)


class TestItRefusesToInventState:
    @pytest.mark.parametrize("pos", [None, [0.0, 0.0], [0.0, 0.0, None]])
    def test_missing_position_returns_none(self, pos):
        assert odostate_fields(pos, [1.0, 0.0, 0.0, 0.0]) is None

    @pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
    def test_non_finite_position_returns_none(self, bad):
        assert odostate_fields([0.0, 0.0, bad], [1.0, 0.0, 0.0, 0.0]) is None

    @pytest.mark.parametrize("bad", [float("nan"), float("inf")])
    def test_non_finite_quaternion_returns_none(self, bad):
        assert odostate_fields([0.0, 0.0, 0.8], [bad, 0.0, 0.0, 0.0]) is None

    def test_zero_norm_quaternion_returns_none(self):
        """A zero-norm quaternion has already crashed the MuJoCo sim once."""
        assert odostate_fields([0.0, 0.0, 0.8], [0.0, 0.0, 0.0, 0.0]) is None

    def test_overflowing_quaternion_returns_none(self):
        """Every component is finite but sum(c*c) overflows to inf; the old
        zero-norm guard was bypassed and c/inf returned the exact degenerate
        [0,0,0,0] it existed to reject. Found in the 2026-08-07 review."""
        assert odostate_fields([0.0, 0.0, 0.8], [1e200, 0.0, 0.0, 0.0]) is None

    @pytest.mark.parametrize("quat", [
        [2.5, 0.0, 0.0, 0.0],     # grossly too large: corruption, not rounding
        [0.3, 0.0, 0.0, 0.0],     # grossly too small
        [1e-9, 0.0, 0.0, 0.0],    # numerical noise must not become "upright"
    ])
    def test_grossly_non_unit_quaternion_is_refused_not_repaired(self, quat):
        """Isaac Lab emits near-unit quaternions; a norm far outside (0.5, 2)
        is evidence of corrupted state (prior art: the numpy-2 wrong-stride
        readback). Silently normalising it would launder garbage into a
        confident orientation."""
        assert odostate_fields([0.0, 0.0, 0.8], quat) is None

    def test_mapping_input_returns_none_not_keyerror(self):
        """A dict with a missing index used to escape as KeyError, violating
        the None-means-untrusted contract."""
        assert odostate_fields({0: 0.0, 1: 0.0, 3: 0.8}, [1.0, 0.0, 0.0, 0.0]) is None

    def test_huge_int_returns_none_not_overflowerror(self):
        """float(10**400) raises OverflowError, which is not a ValueError and
        used to escape the except tuple."""
        assert odostate_fields([0.0, 0.0, 10 ** 400], [1.0, 0.0, 0.0, 0.0]) is None

    @pytest.mark.parametrize("bad", [float("nan"), float("inf")])
    def test_non_finite_linear_velocity_refuses_the_whole_message(self, bad):
        """A validated pose next to a NaN velocity is still an invented
        message. Velocities can go non-finite a step before positions do."""
        assert odostate_fields([0.0, 0.0, 0.8], [1.0, 0.0, 0.0, 0.0],
                               lin_vel_xyz=[bad, 0.0, 0.0]) is None

    @pytest.mark.parametrize("bad", [float("nan"), float("inf")])
    def test_non_finite_angular_velocity_refuses_the_whole_message(self, bad):
        assert odostate_fields([0.0, 0.0, 0.8], [1.0, 0.0, 0.0, 0.0],
                               ang_vel_xyz=[0.0, bad, 0.0]) is None

    def test_short_velocity_refuses_the_whole_message(self):
        assert odostate_fields([0.0, 0.0, 0.8], [1.0, 0.0, 0.0, 0.0],
                               lin_vel_xyz=[0.1, 0.2]) is None


class TestItPassesGoodStateThrough:
    def test_height_is_the_third_position_component(self):
        out = odostate_fields([1.5, -2.0, 0.786], [1.0, 0.0, 0.0, 0.0])
        assert out["position"] == pytest.approx([1.5, -2.0, 0.786])

    def test_quaternion_is_w_first_and_normalised(self):
        """Isaac Lab root_state_w is (w, x, y, z). sonic_telemetry reads w-first.
        If these two ever disagree, tilt is silently wrong. 1.2 is inside the
        sanity window, so it is treated as rounding and normalised."""
        out = odostate_fields([0.0, 0.0, 0.8], [1.2, 0.0, 0.0, 0.0])
        assert out["quaternion_wxyz"] == pytest.approx([1.0, 0.0, 0.0, 0.0])

    def test_a_ninety_degree_roll_survives_normalisation(self):
        s = math.sqrt(0.5)
        out = odostate_fields([0.0, 0.0, 0.8], [s, s, 0.0, 0.0])
        assert out["quaternion_wxyz"] == pytest.approx([s, s, 0.0, 0.0])

    def test_it_accepts_any_indexable_sequence(self):
        out = odostate_fields((0.0, 0.0, 0.8), (1.0, 0.0, 0.0, 0.0))
        assert out is not None

    def test_velocities_pass_through_when_finite(self):
        out = odostate_fields([0.0, 0.0, 0.8], [1.0, 0.0, 0.0, 0.0],
                              lin_vel_xyz=[0.1, -0.2, 0.05],
                              ang_vel_xyz=[0.01, 0.02, -0.03])
        assert out["linear_velocity"] == pytest.approx([0.1, -0.2, 0.05])
        assert out["angular_velocity"] == pytest.approx([0.01, 0.02, -0.03])

    def test_velocities_are_optional_and_omitted_when_not_given(self):
        """Absent is honest; zeros would be a fabricated standstill."""
        out = odostate_fields([0.0, 0.0, 0.8], [1.0, 0.0, 0.0, 0.0])
        assert "linear_velocity" not in out
        assert "angular_velocity" not in out


class TestUnitQuat:
    def test_normalises_within_the_window(self):
        out = unit_quat_wxyz([1.2, 0.0, 0.0, 0.0])
        assert out == pytest.approx([1.0, 0.0, 0.0, 0.0])

    @pytest.mark.parametrize("quat", [None, [float("nan"), 0, 0, 0],
                                      [0.0, 0.0, 0.0, 0.0], [3.0, 0, 0, 0],
                                      [1e200, 0, 0, 0]])
    def test_untrustworthy_returns_none(self, quat):
        assert unit_quat_wxyz(quat) is None


class TestWorldAngVelToBody:
    """The reference publisher (GR00T MuJoCo bridge) writes BODY-frame angular
    velocity (mj qvel[3:6]); Isaac Lab's root_state_w is WORLD-frame. The
    empirical anchor, measured in GR00T's own .venv_sim during the 2026-08-07
    review: a body yawed 90 deg with body ang vel (1,0,0) has world ang vel
    (0,1,0)."""

    def test_identity_orientation_is_a_passthrough(self):
        out = world_ang_vel_to_body([1.0, 0.0, 0.0, 0.0], [0.3, -0.1, 0.7])
        assert out == pytest.approx([0.3, -0.1, 0.7])

    def test_ninety_degree_yaw_matches_the_mujoco_anchor(self):
        s = math.sqrt(0.5)
        # world (0,1,0) on a body yawed +90 deg about z is body (1,0,0)
        out = world_ang_vel_to_body([s, 0.0, 0.0, s], [0.0, 1.0, 0.0])
        assert out == pytest.approx([1.0, 0.0, 0.0])

    def test_it_normalises_before_rotating(self):
        s = math.sqrt(0.5)
        out = world_ang_vel_to_body([1.9 * s, 0.0, 0.0, 1.9 * s], [0.0, 1.0, 0.0])
        assert out == pytest.approx([1.0, 0.0, 0.0])

    @pytest.mark.parametrize("quat", [None, [float("nan"), 0, 0, 0],
                                      [0.0, 0.0, 0.0, 0.0], [3.0, 0, 0, 0]])
    def test_untrustworthy_quaternion_returns_none(self, quat):
        assert world_ang_vel_to_body(quat, [0.0, 0.0, 0.1]) is None

    def test_untrustworthy_omega_returns_none(self):
        assert world_ang_vel_to_body([1.0, 0, 0, 0], [float("inf"), 0, 0]) is None
