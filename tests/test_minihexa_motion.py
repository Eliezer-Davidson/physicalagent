"""Tests for the MiniHexa tiered motion clamp — pure, no I/O, no hardware.

The tier mechanism is the safety-critical heart of the MiniHexa adapter, exactly
as tonypi_servo is for TonyPi. The MiniHexa firmware has NO WATCHDOG and its walk
command is VELOCITY-LATCHED, so how far/fast a burst may go — and how long it
lasts before the mandatory stop — is the difference between a bounded step and a
robot that walks off the bench. There is NO torque channel in the wire protocol:
"aggressiveness" is only ever SPEED (move time / velocity byte) and RANGE (duty
band / velocity cap). These tests pin every band edge, burst duration, and floor
of every tier so a silent widening cannot slip through.

LOCKED is the timid default; NORMAL widens; FULL is the deliberate override up to
the firmware/electrical maximum (velocity byte 50 = codec MOVE_BYTE_LIMIT, duty
500-2500 = the servo electrical range).
"""
import pytest

from physicalagent.robots import minihexa_motion as mot
from physicalagent.robots.minihexa_motion import (
    MotionTier, burst_duration_s, clamp_pose_bytes, clamp_servo_duty, clamp_walk,
)

L = MotionTier.LOCKED
N = MotionTier.NORMAL
F = MotionTier.FULL


class TestTierOrdering:
    def test_tiers_are_strictly_ordered_locked_lt_normal_lt_full(self):
        assert MotionTier.LOCKED < MotionTier.NORMAL < MotionTier.FULL

    def test_locked_is_the_lowest_tier(self):
        assert min(MotionTier) is MotionTier.LOCKED


class TestTierResolution:
    def test_resolve_by_name_is_case_insensitive(self):
        assert MotionTier.resolve("full") is MotionTier.FULL
        assert MotionTier.resolve("NORMAL") is MotionTier.NORMAL
        assert MotionTier.resolve("Locked") is MotionTier.LOCKED

    def test_resolve_none_defaults_to_locked(self):
        assert MotionTier.resolve(None) is MotionTier.LOCKED

    def test_resolve_blank_defaults_to_locked(self):
        assert MotionTier.resolve("   ") is MotionTier.LOCKED

    def test_resolve_unknown_name_raises(self):
        with pytest.raises(ValueError):
            MotionTier.resolve("ludicrous")


# ---------------------------------------------------------------------------
# clamp_walk — the velocity-byte cap per tier (|vx|,|vy|)
# ---------------------------------------------------------------------------
class TestClampWalk:
    def test_locked_caps_velocity_byte_at_15(self):
        assert clamp_walk(50, -50, 0, L) == (15, -15, 0)

    def test_normal_caps_velocity_byte_at_30(self):
        assert clamp_walk(50, -50, 1, N) == (30, -30, 1)

    def test_full_reaches_the_codec_move_byte_limit_50(self):
        assert clamp_walk(999, -999, 2, F) == (50, -50, 2)

    def test_in_band_velocity_passes_through(self):
        assert clamp_walk(10, -5, 0, N) == (10, -5, 0)

    def test_turn_passes_through_untouched(self):
        # turn is validated by the codec (0/1/2); the tier does not touch it.
        assert clamp_walk(0, 0, 2, L)[2] == 2

    def test_locked_narrows_where_full_would_reach(self):
        assert clamp_walk(40, 0, 0, L)[0] == 15
        assert clamp_walk(40, 0, 0, F)[0] == 40


class TestVelocityCapIsMonotonic:
    @pytest.mark.parametrize("v", [-999, -40, -20, -10, 0, 10, 20, 40, 999])
    def test_clamped_velocity_widens_with_tier(self, v):
        lo = clamp_walk(v, 0, 0, L)[0]
        mid = clamp_walk(v, 0, 0, N)[0]
        hi = clamp_walk(v, 0, 0, F)[0]
        if v >= 0:
            assert lo <= mid <= hi
        else:
            assert lo >= mid >= hi


# ---------------------------------------------------------------------------
# burst_duration_s — how long a latched walk runs before the mandatory stop
# ---------------------------------------------------------------------------
class TestBurstDuration:
    def test_locked_burst_is_short(self):
        assert burst_duration_s(L) == 0.4

    def test_normal_burst(self):
        assert burst_duration_s(N) == 0.8

    def test_full_burst_is_longest(self):
        assert burst_duration_s(F) == 1.5

    def test_burst_grows_with_tier(self):
        assert burst_duration_s(L) < burst_duration_s(N) < burst_duration_s(F)


# ---------------------------------------------------------------------------
# clamp_servo_duty — per-joint duty band + move-time floor (USB-only path)
# ---------------------------------------------------------------------------
class TestClampServoDutyLocked:
    """LOCKED: 1200-1800 us, floor 800 ms."""

    def test_duty_in_band_passes_through(self):
        d, _ = clamp_servo_duty(1500, 1000, L)
        assert d == 1500

    def test_duty_above_band_clamps_to_1800(self):
        d, _ = clamp_servo_duty(2500, 1000, L)
        assert d == 1800

    def test_duty_below_band_clamps_to_1200(self):
        d, _ = clamp_servo_duty(500, 1000, L)
        assert d == 1200

    def test_too_fast_move_is_raised_to_the_800ms_floor(self):
        _, t = clamp_servo_duty(1500, 10, L)
        assert t == 800


class TestClampServoDutyNormal:
    """NORMAL: 900-2100 us, floor 400 ms."""

    def test_duty_above_band_clamps_to_2100(self):
        d, _ = clamp_servo_duty(2500, 1000, N)
        assert d == 2100

    def test_duty_below_band_clamps_to_900(self):
        d, _ = clamp_servo_duty(500, 1000, N)
        assert d == 900

    def test_a_duty_locked_would_reject_is_allowed_here(self):
        # 2000 is outside LOCKED's 1200-1800 but inside NORMAL's 900-2100
        d, _ = clamp_servo_duty(2000, 1000, N)
        assert d == 2000

    def test_too_fast_move_is_raised_to_the_400ms_floor(self):
        _, t = clamp_servo_duty(1500, 10, N)
        assert t == 400


class TestClampServoDutyFull:
    """FULL: the deliberate override — full electrical 500-2500, floor 120 ms."""

    def test_reaches_the_electrical_maximum_2500(self):
        d, _ = clamp_servo_duty(9999, 1000, F)
        assert d == 2500

    def test_reaches_the_electrical_minimum_500(self):
        d, _ = clamp_servo_duty(-9999, 1000, F)
        assert d == 500

    def test_fast_moves_allowed_down_to_the_120ms_floor(self):
        _, t = clamp_servo_duty(2500, 10, F)
        assert t == 120


class TestClampServoDutyMoveTime:
    def test_none_move_time_becomes_the_tier_floor(self):
        _, t = clamp_servo_duty(1500, None, N)
        assert t == 400

    def test_a_slower_than_floor_move_is_kept(self):
        _, t = clamp_servo_duty(1500, 5000, F)
        assert t == 5000

    def test_absurd_move_time_is_capped_at_two_byte_max(self):
        _, t = clamp_servo_duty(1500, 10_000_000, F)
        assert t == 0xFFFF


class TestServoDutyBandsAreStrictlyNested:
    @pytest.mark.parametrize("duty", [400, 500, 900, 1200, 1500, 1800, 2100, 2500, 3000])
    def test_clamped_duty_is_monotonic_in_tier(self, duty):
        lo = clamp_servo_duty(duty, 1000, L)[0]
        mid = clamp_servo_duty(duty, 1000, N)[0]
        hi = clamp_servo_duty(duty, 1000, F)[0]
        if duty >= 1500:
            assert lo <= mid <= hi
        else:
            assert lo >= mid >= hi

    def test_floor_strictly_decreases_with_tier(self):
        f = lambda tier: clamp_servo_duty(1500, 0, tier)[1]
        assert f(L) > f(N) > f(F)


# ---------------------------------------------------------------------------
# clamp_pose_bytes — the F-frame byte caps per tier
# ---------------------------------------------------------------------------
class TestClampPoseBytes:
    def test_locked_caps_the_first_five_at_15(self):
        assert clamp_pose_bytes(50, -50, 50, -50, 50, 20, L) == (15, -15, 15, -15, 15, 20)

    def test_normal_caps_the_first_five_at_30(self):
        assert clamp_pose_bytes(50, -50, 50, -50, 50, 20, N) == (30, -30, 30, -30, 30, 20)

    def test_full_reaches_the_pose_byte_limit_50(self):
        assert clamp_pose_bytes(99, -99, 99, -99, 99, 20, F) == (50, -50, 50, -50, 50, 20)

    def test_height_is_always_clamped_to_the_codec_0_30_regardless_of_tier(self):
        assert clamp_pose_bytes(0, 0, 0, 0, 0, 99, L)[5] == 30
        assert clamp_pose_bytes(0, 0, 0, 0, 0, -5, F)[5] == 0

    def test_in_band_pose_passes_through(self):
        assert clamp_pose_bytes(5, -3, 2, 1, -4, 15, N) == (5, -3, 2, 1, -4, 15)

    def test_returns_six_ints(self):
        out = clamp_pose_bytes(1, 2, 3, 4, 5, 6, L)
        assert len(out) == 6
        assert all(isinstance(v, int) for v in out)


class TestPoseCapIsMonotonic:
    @pytest.mark.parametrize("v", [-99, -30, -10, 0, 10, 30, 99])
    def test_yaw_cap_widens_with_tier(self, v):
        lo = clamp_pose_bytes(v, 0, 0, 0, 0, 0, L)[0]
        mid = clamp_pose_bytes(v, 0, 0, 0, 0, 0, N)[0]
        hi = clamp_pose_bytes(v, 0, 0, 0, 0, 0, F)[0]
        if v >= 0:
            assert lo <= mid <= hi
        else:
            assert lo >= mid >= hi


class TestNoTorqueChannel:
    """The wire protocol has NO torque field: the module must express aggression
    only as SPEED and RANGE. Guard against a stray 'torque'/'protection' knob."""

    def test_module_exposes_no_torque_or_protection_symbol(self):
        for name in dir(mot):
            low = name.lower()
            assert "torque" not in low
            assert "protection" not in low
