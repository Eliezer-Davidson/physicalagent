"""Tests for the TonyPi tiered servo-clamp — pure, no I/O, no hardware.

The tier mechanism is the safety-critical heart of the TonyPi adapter: the one
place that decides how far and how fast a servo may move, given an explicitly-
selected TIER *and* which physical servo class (DOMAIN) it is.

The two domains have DIFFERENT electrical ranges and must never be confused:
  * BUS — body serial bus servos (SetBusServoPulse), pulse 0-1000, centre 500.
    VERIFIED in config.py (body bus servos ids 1-16, pulse 0-1000) and by the
    bus_servo/get_state readback. Commanding one a PWM-scale 1500 drives it hard
    past its range.
  * PWM — head pan/tilt servos (SetPWMServo id 1/2), pulse 500-2500, centre 1500.

LOCKED is the timid default; NORMAL widens the band and lets moves go quicker;
FULL is the deliberate override up to each domain's true electrical range and
fast moves. These tests pin every band edge and move-time floor of every tier in
BOTH domains so a silent widening — or a domain mix-up — cannot slip through.
"""
import pytest

from physicalagent.robots import tonypi_servo as srv
from physicalagent.robots.tonypi_servo import ServoDomain, ServoTier, clamp_servo

BUS = ServoDomain.BUS
PWM = ServoDomain.PWM


class TestTierOrdering:
    def test_tiers_are_strictly_ordered_locked_lt_normal_lt_full(self):
        # the override design is a LADDER: each rung is strictly more permissive.
        assert ServoTier.LOCKED < ServoTier.NORMAL < ServoTier.FULL

    def test_locked_is_the_lowest_tier(self):
        assert min(ServoTier) is ServoTier.LOCKED


# ---------------------------------------------------------------------------
# BUS domain — body servos, 0-1000, centre 500
# ---------------------------------------------------------------------------
class TestBusLockedTier:
    """BUS LOCKED: 500 +/- 150 => 350-650, floor 800 ms."""

    def test_pulse_in_band_passes_through(self):
        p, _ = clamp_servo(500, 1000, ServoTier.LOCKED, BUS)
        assert p == 500

    def test_pulse_above_band_clamps_to_650(self):
        p, _ = clamp_servo(1000, 1000, ServoTier.LOCKED, BUS)
        assert p == 650

    def test_a_pwm_scale_pulse_is_clamped_hard_into_the_bus_band(self):
        # the exact bug: 1500 is a PWM centre, NOT a bus value; on a bus servo it
        # must be clamped to the band ceiling, never sent as-is.
        p, _ = clamp_servo(1500, 1000, ServoTier.LOCKED, BUS)
        assert p == 650

    def test_pulse_below_band_clamps_to_350(self):
        p, _ = clamp_servo(0, 1000, ServoTier.LOCKED, BUS)
        assert p == 350

    def test_too_fast_move_is_raised_to_the_800ms_floor(self):
        _, t = clamp_servo(500, 100, ServoTier.LOCKED, BUS)
        assert t == 800


class TestBusNormalTier:
    """BUS NORMAL: 200-800, floor 400 ms."""

    def test_pulse_above_band_clamps_to_800(self):
        p, _ = clamp_servo(1000, 1000, ServoTier.NORMAL, BUS)
        assert p == 800

    def test_pulse_below_band_clamps_to_200(self):
        p, _ = clamp_servo(0, 1000, ServoTier.NORMAL, BUS)
        assert p == 200

    def test_a_pulse_locked_would_reject_is_allowed_here(self):
        # 750 is outside BUS LOCKED's 350-650 but inside NORMAL's 200-800
        p, _ = clamp_servo(750, 1000, ServoTier.NORMAL, BUS)
        assert p == 750

    def test_too_fast_move_is_raised_to_the_400ms_floor(self):
        _, t = clamp_servo(500, 100, ServoTier.NORMAL, BUS)
        assert t == 400


class TestBusFullTier:
    """BUS FULL: the deliberate override — full electrical 0-1000, floor 120 ms."""

    def test_reaches_the_true_electrical_maximum_1000(self):
        p, _ = clamp_servo(9999, 1000, ServoTier.FULL, BUS)
        assert p == 1000

    def test_reaches_the_true_electrical_minimum_0(self):
        p, _ = clamp_servo(-9999, 1000, ServoTier.FULL, BUS)
        assert p == 0

    def test_a_pulse_in_the_full_band_passes_through(self):
        p, _ = clamp_servo(900, 1000, ServoTier.FULL, BUS)
        assert p == 900

    def test_fast_moves_are_allowed_down_to_the_120ms_floor(self):
        _, t = clamp_servo(1000, 50, ServoTier.FULL, BUS)
        assert t == 120

    def test_never_reaches_a_pwm_scale_value(self):
        # even FULL bus tops out at 1000 — a PWM 2500 is not reachable on a bus servo
        p, _ = clamp_servo(2500, 1000, ServoTier.FULL, BUS)
        assert p == 1000


# ---------------------------------------------------------------------------
# PWM domain — head servos, 500-2500, centre 1500
# ---------------------------------------------------------------------------
class TestPwmLockedTier:
    """PWM LOCKED: 1500 +/- 300 => 1200-1800, floor 800 ms."""

    def test_pulse_in_band_passes_through(self):
        p, _ = clamp_servo(1500, 1000, ServoTier.LOCKED, PWM)
        assert p == 1500

    def test_pulse_above_band_clamps_to_1800(self):
        p, _ = clamp_servo(2500, 1000, ServoTier.LOCKED, PWM)
        assert p == 1800

    def test_pulse_below_band_clamps_to_1200(self):
        p, _ = clamp_servo(500, 1000, ServoTier.LOCKED, PWM)
        assert p == 1200

    def test_too_fast_move_is_raised_to_the_800ms_floor(self):
        _, t = clamp_servo(1500, 100, ServoTier.LOCKED, PWM)
        assert t == 800


class TestPwmNormalTier:
    """PWM NORMAL: 700-2300, floor 400 ms."""

    def test_pulse_above_band_clamps_to_2300(self):
        p, _ = clamp_servo(2500, 1000, ServoTier.NORMAL, PWM)
        assert p == 2300

    def test_pulse_below_band_clamps_to_700(self):
        p, _ = clamp_servo(500, 1000, ServoTier.NORMAL, PWM)
        assert p == 700

    def test_too_fast_move_is_raised_to_the_400ms_floor(self):
        _, t = clamp_servo(1500, 100, ServoTier.NORMAL, PWM)
        assert t == 400


class TestPwmFullTier:
    """PWM FULL: full electrical 500-2500, floor 120 ms."""

    def test_reaches_the_true_electrical_maximum_2500(self):
        p, _ = clamp_servo(9999, 1000, ServoTier.FULL, PWM)
        assert p == 2500

    def test_reaches_the_true_electrical_minimum_500(self):
        p, _ = clamp_servo(-9999, 1000, ServoTier.FULL, PWM)
        assert p == 500

    def test_fast_moves_are_allowed_down_to_the_120ms_floor(self):
        _, t = clamp_servo(2500, 50, ServoTier.FULL, PWM)
        assert t == 120


class TestDomainsAreDistinct:
    """The whole point of the fix: the SAME pulse clamps differently per domain."""

    def test_full_maxima_differ_between_domains(self):
        assert clamp_servo(9999, 1000, ServoTier.FULL, BUS)[0] == 1000
        assert clamp_servo(9999, 1000, ServoTier.FULL, PWM)[0] == 2500

    def test_the_bus_centre_500_is_valid_bus_but_below_the_pwm_band(self):
        # 500 is the BUS centre (untouched) but the PWM floor edge
        assert clamp_servo(500, 1000, ServoTier.NORMAL, BUS)[0] == 500   # in bus band
        assert clamp_servo(500, 1000, ServoTier.LOCKED, PWM)[0] == 1200  # clamped up in pwm


class TestBandsAreStrictlyNested:
    """Within a domain, each tier's band sits inside the next: LOCKED c NORMAL c
    FULL. If a lower tier ever allowed a wider pulse than a higher one, the ladder
    would be a lie."""

    @pytest.mark.parametrize("domain,centre,pulse", [
        (BUS, 500, p) for p in [0, 200, 350, 500, 650, 800, 1000, 1500, -50]
    ] + [
        (PWM, 1500, p) for p in [500, 700, 1200, 1500, 1800, 2300, 2500, 3000, -50]
    ])
    def test_clamped_pulse_is_monotonic_in_tier(self, domain, centre, pulse):
        lo = clamp_servo(pulse, 1000, ServoTier.LOCKED, domain)[0]
        mid = clamp_servo(pulse, 1000, ServoTier.NORMAL, domain)[0]
        hi = clamp_servo(pulse, 1000, ServoTier.FULL, domain)[0]
        if pulse >= centre:
            assert lo <= mid <= hi
        else:
            assert lo >= mid >= hi

    @pytest.mark.parametrize("domain", [BUS, PWM])
    def test_floor_strictly_decreases_with_tier(self, domain):
        f = lambda tier: clamp_servo(500, 0, tier, domain)[1]
        assert f(ServoTier.LOCKED) > f(ServoTier.NORMAL) > f(ServoTier.FULL)


class TestMoveTimeHandling:
    def test_none_move_time_becomes_the_tier_floor(self):
        # the adapter passes move_time_ms=None to mean 'use the safe default'
        _, t = clamp_servo(500, None, ServoTier.NORMAL, BUS)
        assert t == 400

    def test_absurd_move_time_is_capped(self):
        _, t = clamp_servo(500, 10_000_000, ServoTier.FULL, BUS)
        assert t == srv.TIME_MAX_MS


class TestTierResolution:
    """Tier is selected explicitly by name (env var / --tier / adapter arg)."""

    def test_resolve_by_name_is_case_insensitive(self):
        assert ServoTier.resolve("full") is ServoTier.FULL
        assert ServoTier.resolve("NORMAL") is ServoTier.NORMAL
        assert ServoTier.resolve("Locked") is ServoTier.LOCKED

    def test_resolve_none_defaults_to_locked(self):
        assert ServoTier.resolve(None) is ServoTier.LOCKED

    def test_resolve_empty_defaults_to_locked(self):
        assert ServoTier.resolve("  ") is ServoTier.LOCKED

    def test_resolve_unknown_name_raises(self):
        with pytest.raises(ValueError):
            ServoTier.resolve("ludicrous")


class TestProtectionOverrideSeam:
    """CRITICAL: the servos' thermal/overload/voltage protection is NEVER
    disabled by the clamp or the normal path. The protection-disable capability
    is deliberately unimplemented; only a clearly-named, OFF-by-default seam
    exists. clamp_servo must not so much as mention it."""

    def test_protection_override_is_not_supported_in_this_pass(self):
        assert srv.PROTECTION_OVERRIDE_SUPPORTED is False

    def test_clamp_never_toggles_protection(self):
        # clamp_servo returns exactly (pulse, time) — no third 'protection' knob
        out = clamp_servo(500, 1000, ServoTier.FULL, BUS)
        assert len(out) == 2
