"""Unit tests for the safety layer. Pure logic — no hardware, no network."""
import pytest

from physicalagent.safety import Velocity, VelocityLimits, clamp_velocity


class TestClampVelocity:
    def test_forward_speed_above_max_is_clamped(self):
        limits = VelocityLimits(max_abs_x=0.2, max_abs_yaw=0.2, allow_lateral_y=False)
        out = clamp_velocity(Velocity(x=5.0), limits)
        assert out.x == 0.2

    def test_reverse_speed_beyond_max_is_clamped_to_negative_max(self):
        limits = VelocityLimits(max_abs_x=0.2, max_abs_yaw=0.2, allow_lateral_y=False)
        out = clamp_velocity(Velocity(x=-5.0), limits)
        assert out.x == -0.2

    def test_yaw_rate_above_max_is_clamped(self):
        limits = VelocityLimits(max_abs_x=0.2, max_abs_yaw=0.89, allow_lateral_y=False)
        out = clamp_velocity(Velocity(yaw_rate=10.0), limits)
        assert out.yaw_rate == 0.89

    def test_lateral_y_is_forced_to_zero_when_not_allowed(self):
        limits = VelocityLimits(max_abs_x=0.2, max_abs_yaw=0.2, allow_lateral_y=False)
        out = clamp_velocity(Velocity(y=3.0), limits)
        assert out.y == 0.0

    def test_in_range_command_passes_through_unchanged(self):
        limits = VelocityLimits(max_abs_x=0.2, max_abs_yaw=0.5, allow_lateral_y=False)
        out = clamp_velocity(Velocity(x=0.1, yaw_rate=-0.3), limits)
        assert out.x == 0.1
        assert out.yaw_rate == -0.3

    def test_stop_flag_is_preserved(self):
        limits = VelocityLimits(max_abs_x=0.2, max_abs_yaw=0.2, allow_lateral_y=False)
        out = clamp_velocity(Velocity(x=5.0, stop=True), limits)
        assert out.stop is True


from physicalagent import config
from physicalagent.safety import velocity_limits_for


class TestVelocityLimitsRespectMotionLock:
    def test_locked_uses_tiny_unverified_limits_safe_under_any_unit(self):
        # While units of Velocity.x are unverified, limits must be safe read as
        # cm/s OR m/s. That is config.UNVERIFIED_MAX_ABS_X.
        limits = velocity_limits_for(motion_unlocked=False)
        assert limits.max_abs_x == config.UNVERIFIED_MAX_ABS_X
        assert limits.max_abs_yaw == config.UNVERIFIED_MAX_ABS_YAW

    def test_unlocked_uses_verified_limits(self):
        limits = velocity_limits_for(motion_unlocked=True)
        assert limits.max_abs_x == config.VERIFIED_MAX_ABS_X_M_S
        assert limits.max_abs_yaw == config.VERIFIED_MAX_ABS_YAW_RAD_S

    def test_lateral_y_follows_config_flag_in_both_modes(self):
        assert velocity_limits_for(False).allow_lateral_y == config.ALLOW_LATERAL_Y
        assert velocity_limits_for(True).allow_lateral_y == config.ALLOW_LATERAL_Y

    def test_locked_limit_is_strictly_smaller_than_unlocked(self):
        # The whole point: locked must be more conservative.
        assert velocity_limits_for(False).max_abs_x < velocity_limits_for(True).max_abs_x


from physicalagent.safety import SafetyError, check_action_allowed


class TestActionWhitelist:
    def test_empty_whitelist_rejects_every_action(self):
        # ROSPug ships with an empty action whitelist until we inventory the real
        # names from hardware. Until then, EVERY action name must be refused.
        with pytest.raises(SafetyError):
            check_action_allowed("stand", whitelist=())

    def test_name_not_in_whitelist_is_rejected(self):
        with pytest.raises(SafetyError):
            check_action_allowed("backflip", whitelist=("stand", "sit"))

    def test_name_in_whitelist_is_allowed(self):
        # Returns the name (no raise) when allowed.
        assert check_action_allowed("sit", whitelist=("stand", "sit")) == "sit"

    def test_rejection_message_names_the_offending_action(self):
        with pytest.raises(SafetyError, match="backflip"):
            check_action_allowed("backflip", whitelist=("stand",))


from physicalagent.safety import bound_steps, bound_duration, max_duration_for


class TestBoundedActions:
    def test_steps_capped_at_max(self):
        assert bound_steps(99, max_steps=4) == 4

    def test_steps_within_cap_pass_through(self):
        assert bound_steps(2, max_steps=4) == 2

    def test_negative_steps_floored_to_zero(self):
        assert bound_steps(-3, max_steps=4) == 0

    def test_steps_default_cap_comes_from_config(self):
        assert bound_steps(1000) == config.MAX_STEPS_PER_CALL

    def test_duration_capped(self):
        assert bound_duration(10.0, max_s=3.0) == 3.0

    def test_negative_duration_floored_to_zero(self):
        assert bound_duration(-1.0, max_s=3.0) == 0.0

    def test_locked_duration_cap_is_the_short_unverified_one(self):
        assert max_duration_for(motion_unlocked=False) == config.UNVERIFIED_MAX_DURATION_S

    def test_unlocked_duration_cap_is_the_verified_one(self):
        assert max_duration_for(motion_unlocked=True) == config.VERIFIED_MAX_DURATION_S


from physicalagent.safety import Pose, PoseLimits, clamp_pose, default_pose_limits


def _pose_limits():
    return PoseLimits(
        max_abs_roll=0.54, max_abs_pitch=0.54,
        height_min=-15.0, height_max=-5.0,
        max_abs_x_shift=10.0, max_abs_stance=5.0,
    )


class TestClampPose:
    def test_roll_above_max_clamped(self):
        out = clamp_pose(Pose(roll=2.0), _pose_limits())
        assert out.roll == 0.54

    def test_pitch_below_neg_max_clamped(self):
        out = clamp_pose(Pose(pitch=-2.0), _pose_limits())
        assert out.pitch == -0.54

    def test_yaw_forced_to_zero(self):
        # Pose yaw must be 0 on this robot family.
        out = clamp_pose(Pose(yaw=0.4), _pose_limits())
        assert out.yaw == 0.0

    def test_height_too_high_clamped_into_negative_range(self):
        # height must live in [-15, -5]; a request of 0 clamps up to -5.
        out = clamp_pose(Pose(height=0.0), _pose_limits())
        assert out.height == -5.0

    def test_height_too_low_clamped_to_min(self):
        out = clamp_pose(Pose(height=-40.0), _pose_limits())
        assert out.height == -15.0

    def test_x_shift_and_stance_clamped(self):
        out = clamp_pose(Pose(x_shift=99.0, stance_x=99.0, stance_y=-99.0), _pose_limits())
        assert out.x_shift == 10.0
        assert out.stance_x == 5.0
        assert out.stance_y == -5.0

    def test_in_range_pose_passes_through(self):
        p = Pose(roll=0.1, pitch=-0.2, height=-10.0, x_shift=2.0)
        out = clamp_pose(p, _pose_limits())
        assert (out.roll, out.pitch, out.height, out.x_shift) == (0.1, -0.2, -10.0, 2.0)

    def test_default_pose_limits_come_from_config(self):
        lim = default_pose_limits()
        assert lim.max_abs_roll == config.MAX_ABS_ROLL_RAD
        assert lim.height_min == config.HEIGHT_MIN_M
        assert lim.height_max == config.HEIGHT_MAX_M


from physicalagent.safety import ROSPUG_SAFETY, check_battery_ok_for_motion, battery_state


class TestBatteryGate:
    def test_motion_refused_below_threshold(self):
        # The pack we measured (~10050 mV) must NOT be allowed to drive motion.
        with pytest.raises(SafetyError):
            check_battery_ok_for_motion(10050, min_mv=11100)

    def test_motion_allowed_at_or_above_threshold(self):
        # Returns None (no raise) when healthy.
        assert check_battery_ok_for_motion(11400, min_mv=11100) is None

    def test_default_threshold_comes_from_config(self):
        with pytest.raises(SafetyError):
            check_battery_ok_for_motion(config.BATTERY_MOTION_MIN_MV - 1)

    def test_error_reports_the_voltage(self):
        with pytest.raises(SafetyError, match="10050"):
            check_battery_ok_for_motion(10050, min_mv=11100)

    def test_state_critical_below_warn(self):
        assert battery_state(10050, ROSPUG_SAFETY) == "critical"

    def test_state_warn_between_warn_and_motion_min(self):
        # Above warn floor but below the motion gate = warn.
        assert battery_state(10800, ROSPUG_SAFETY) == "warn"

    def test_state_ok_above_motion_min(self):
        assert battery_state(12000, ROSPUG_SAFETY) == "ok"

    def test_thresholds_come_from_the_PROFILE_not_the_rospug_globals(self):
        """A 2S pack resting at a healthy 7.4 V was reported 'critical',
        because the thresholds were read from the ROSPug's 3S constants. The
        model reads battery_state and stops working when it says critical, so
        this was a fleet-wide false alarm waiting to happen."""
        from dataclasses import replace
        two_s = replace(ROSPUG_SAFETY, name="2s", battery_motion_min_mv=7400,
                        battery_warn_mv=7000)
        assert battery_state(7400, two_s) == "ok"
        assert battery_state(7400, ROSPUG_SAFETY) == "critical"   # the old answer

    def test_a_mains_powered_robot_has_no_battery_state(self):
        from dataclasses import replace
        from physicalagent.safety import NO_BATTERY
        mains = replace(ROSPUG_SAFETY, name="arm",
                        battery_motion_min_mv=NO_BATTERY, battery_warn_mv=NO_BATTERY)
        assert battery_state(0, mains) == "no_battery"

    def test_exact_threshold_is_permitted(self):
        # off-by-one boundary: floor exactly is OK (gate is `< min`, not `<=`)
        assert check_battery_ok_for_motion(11100, min_mv=11100) is None

    def test_none_reading_refused_not_crash(self):
        # unreadable battery (telemetry not arrived) must REFUSE, never assume OK
        with pytest.raises(SafetyError, match="unreadable|unknown|no reading"):
            check_battery_ok_for_motion(None, min_mv=11100)

    def test_nan_reading_refused(self):
        # nan < min is False in IEEE-754 — must not silently permit motion
        with pytest.raises(SafetyError):
            check_battery_ok_for_motion(float("nan"), min_mv=11100)


from physicalagent.safety import RateLimiter


class FakeClock:
    """Injected monotonic clock for deterministic time tests."""
    def __init__(self, t=0.0):
        self.t = t
    def __call__(self):
        return self.t
    def advance(self, dt):
        self.t += dt


class TestRateLimiter:
    def test_first_call_is_allowed(self):
        rl = RateLimiter(max_hz=10.0, clock=FakeClock())
        assert rl.allow() is True

    def test_immediate_second_call_is_denied(self):
        clk = FakeClock()
        rl = RateLimiter(max_hz=10.0, clock=clk)
        rl.allow()
        assert rl.allow() is False   # 0s later, min interval is 0.1s

    def test_call_allowed_after_interval_elapses(self):
        clk = FakeClock()
        rl = RateLimiter(max_hz=10.0, clock=clk)
        rl.allow()
        clk.advance(0.1)             # exactly one interval
        assert rl.allow() is True

    def test_denied_call_does_not_consume_the_slot(self):
        clk = FakeClock()
        rl = RateLimiter(max_hz=10.0, clock=clk)
        rl.allow()                   # t=0 allowed
        clk.advance(0.05); assert rl.allow() is False   # too soon, denied
        clk.advance(0.05); assert rl.allow() is True    # now 0.1 since last ALLOW


from physicalagent.safety import Watchdog


class TestWatchdog:
    def test_freshly_armed_watchdog_is_not_expired(self):
        wd = Watchdog(timeout_s=1.0, clock=FakeClock())
        assert wd.expired() is False

    def test_expires_after_timeout_without_a_kick(self):
        clk = FakeClock()
        wd = Watchdog(timeout_s=1.0, clock=clk)
        clk.advance(1.5)
        assert wd.expired() is True

    def test_kick_resets_the_deadman(self):
        clk = FakeClock()
        wd = Watchdog(timeout_s=1.0, clock=clk)
        clk.advance(0.9)
        wd.kick()
        clk.advance(0.9)            # 0.9s since kick < 1.0 timeout
        assert wd.expired() is False

    def test_still_alive_just_under_timeout(self):
        clk = FakeClock()
        wd = Watchdog(timeout_s=1.0, clock=clk)
        clk.advance(0.99)
        assert wd.expired() is False

    def test_default_timeout_from_config(self):
        wd = Watchdog(clock=FakeClock())
        assert wd.timeout_s == config.WATCHDOG_TIMEOUT_S


from physicalagent.safety import ROSPUG_SAFETY, SafetyGuard


class TestSafetyGuard:
    def test_new_guard_is_not_estopped(self):
        assert SafetyGuard(ROSPUG_SAFETY).is_estopped is False

    def test_guard_velocity_clamps_using_locked_limits_by_default(self):
        g = SafetyGuard(ROSPUG_SAFETY, motion_unlocked=False)
        out = g.guard_velocity(Velocity(x=5.0))
        assert out.x == config.UNVERIFIED_MAX_ABS_X

    def test_guard_velocity_uses_verified_limits_when_unlocked(self):
        g = SafetyGuard(ROSPUG_SAFETY, motion_unlocked=True)
        out = g.guard_velocity(Velocity(x=999.0))
        assert out.x == config.VERIFIED_MAX_ABS_X_M_S

    def test_estop_blocks_velocity_commands(self):
        g = SafetyGuard(ROSPUG_SAFETY)
        g.estop()
        with pytest.raises(SafetyError, match="stop"):
            g.guard_velocity(Velocity(x=0.1))

    def test_estop_blocks_actions(self):
        g = SafetyGuard(ROSPUG_SAFETY, action_whitelist=("stand",))
        g.estop()
        with pytest.raises(SafetyError):
            g.guard_action("stand")

    def test_estop_latches_until_reset(self):
        g = SafetyGuard(ROSPUG_SAFETY)
        g.estop()
        assert g.is_estopped is True
        g.reset()
        assert g.is_estopped is False
        # works again after reset
        assert g.guard_velocity(Velocity(x=0.05)).x == 0.05

    def test_guard_action_enforces_whitelist_when_not_estopped(self):
        g = SafetyGuard(ROSPUG_SAFETY, action_whitelist=())
        with pytest.raises(SafetyError):
            g.guard_action("stand")

    def test_guard_action_allows_whitelisted_name(self):
        g = SafetyGuard(ROSPUG_SAFETY, action_whitelist=("stand", "sit"))
        assert g.guard_action("sit") == "sit"

    def test_guard_pose_clamps_and_blocks_on_estop(self):
        g = SafetyGuard(ROSPUG_SAFETY)
        assert g.guard_pose(Pose(roll=9.0)).roll == config.MAX_ABS_ROLL_RAD
        g.estop()
        with pytest.raises(SafetyError):
            g.guard_pose(Pose())


class TestLateralYClamping:
    """The ROSPug DOES implement lateral y (unlike the PuppyPi), so when it is
    allowed it must still be clamped — not passed through unbounded."""

    def test_lateral_y_clamped_to_max_when_allowed(self):
        limits = VelocityLimits(max_abs_x=0.1, max_abs_y=0.05,
                                max_abs_yaw=0.35, allow_lateral_y=True)
        out = clamp_velocity(Velocity(y=9.0), limits)
        assert out.y == 0.05

    def test_negative_lateral_y_clamped(self):
        limits = VelocityLimits(max_abs_x=0.1, max_abs_y=0.05,
                                max_abs_yaw=0.35, allow_lateral_y=True)
        out = clamp_velocity(Velocity(y=-9.0), limits)
        assert out.y == -0.05

    def test_in_range_lateral_y_passes_through(self):
        limits = VelocityLimits(max_abs_x=0.1, max_abs_y=0.05,
                                max_abs_yaw=0.35, allow_lateral_y=True)
        assert clamp_velocity(Velocity(y=0.02), limits).y == 0.02

    def test_lateral_y_still_zeroed_when_disallowed(self):
        limits = VelocityLimits(max_abs_x=0.1, max_abs_y=0.05,
                                max_abs_yaw=0.35, allow_lateral_y=False)
        assert clamp_velocity(Velocity(y=0.02), limits).y == 0.0


class TestVendorCeilingsAreNeverExceeded:
    """The hard ceiling that survives every 'remove the limits' request.

    Speed/yaw limits above the vendor's own values do not make the robot faster
    in a useful way — they destabilise the gait and risk stripping servos. The
    owner can unlock everything else; this invariant is what stops 'unlimited'
    from meaning 'damaged'.

    Asserted against ROSPUG_SAFETY, which is what the live guard actually
    clamps with. It used to assert against velocity_limits_for(), a legacy
    helper the adapter no longer consults -- so the invariant was guarding a
    code path that could not reach the robot, and would have kept passing
    while ROSPUG_SAFETY was edited past the ceiling.
    """

    def test_unlocked_velocity_never_exceeds_vendor_maxima(self):
        lim = ROSPUG_SAFETY.velocity_limits(motion_unlocked=True)
        assert lim.max_abs_x <= config.VENDOR_MAX_ABS_X_M_S
        assert lim.max_abs_y <= config.VENDOR_MAX_ABS_Y_M_S
        assert lim.max_abs_yaw <= config.VENDOR_MAX_ABS_YAW_RAD_S

    def test_locked_velocity_never_exceeds_vendor_maxima(self):
        lim = ROSPUG_SAFETY.velocity_limits(motion_unlocked=False)
        assert lim.max_abs_x <= config.VENDOR_MAX_ABS_X_M_S
        assert lim.max_abs_yaw <= config.VENDOR_MAX_ABS_YAW_RAD_S

    def test_the_profiles_declared_ceiling_is_the_vendors(self):
        """The __post_init__ invariant is only as good as the ceiling it
        compares against, so pin the ceiling itself to the verified numbers."""
        ceiling = ROSPUG_SAFETY.vendor_ceiling
        assert ceiling.max_abs_x == config.VENDOR_MAX_ABS_X_M_S
        assert ceiling.max_abs_y == config.VENDOR_MAX_ABS_Y_M_S
        assert ceiling.max_abs_yaw == config.VENDOR_MAX_ABS_YAW_RAD_S

    def test_clamp_cannot_emit_a_velocity_above_vendor_max(self):
        """Even an absurd request comes out within the hardware ceiling."""
        out = clamp_velocity(Velocity(x=99.0, y=99.0, yaw_rate=99.0),
                             ROSPUG_SAFETY.velocity_limits(motion_unlocked=True))
        assert abs(out.x) <= config.VENDOR_MAX_ABS_X_M_S
        assert abs(out.y) <= config.VENDOR_MAX_ABS_Y_M_S
        assert abs(out.yaw_rate) <= config.VENDOR_MAX_ABS_YAW_RAD_S

    def test_the_guard_the_cli_builds_cannot_exceed_the_ceiling(self):
        """End of the live path: the guard cli.build_guard() hands the adapter."""
        from physicalagent.cli import build_guard
        from physicalagent.profiles import get_profile
        out = build_guard(get_profile("rospug")).guard_velocity(
            Velocity(x=99.0, y=99.0, yaw_rate=99.0))
        assert abs(out.x) <= config.VENDOR_MAX_ABS_X_M_S
        assert abs(out.y) <= config.VENDOR_MAX_ABS_Y_M_S
        assert abs(out.yaw_rate) <= config.VENDOR_MAX_ABS_YAW_RAD_S
