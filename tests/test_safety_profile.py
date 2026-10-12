"""SafetyProfile — per-robot limits, with the vendor ceiling as a construction invariant.

The project's load-bearing comment says it plainly: on the primary interface the
vendor does NOT clamp, so this layer is the only guard between a bad number and
12 servos. A profile that declares a limit above its vendor ceiling is therefore
not a test failure to be discovered later -- it must be impossible to build.
"""
import pytest

from physicalagent import config
from physicalagent.safety import (
    NO_BATTERY, PoseLimits, ROSPUG_SAFETY, SafetyError, SafetyGuard,
    SafetyProfile, Velocity, VelocityLimits,
)


def _ceiling(x=0.1, y=0.1, yaw=0.4):
    return VelocityLimits(max_abs_x=x, max_abs_y=y, max_abs_yaw=yaw, allow_lateral_y=True)


def _profile(**over):
    base = dict(
        name="test",
        locked_velocity=_ceiling(0.05, 0.025, 0.15),
        unlocked_velocity=_ceiling(),
        vendor_ceiling=_ceiling(),
        pose=PoseLimits(max_abs_roll=0.8, max_abs_pitch=0.8, height_min=-0.22,
                        height_max=-0.04, max_abs_x_shift=0.08, max_abs_stance=0.08),
        locked_max_duration_s=1.0,
        unlocked_max_duration_s=15.0,
        max_steps_per_call=4,
        # Stated, not defaulted -- SafetyProfile has no default for these.
        battery_motion_min_mv=11100,
        battery_warn_mv=10500,
    )
    base.update(over)
    return SafetyProfile(**base)


class TestVendorCeilingIsAConstructionInvariant:
    def test_a_profile_within_its_ceiling_builds(self):
        assert _profile().name == "test"

    def test_unlocked_x_above_ceiling_is_refused(self):
        with pytest.raises(SafetyError, match="max_abs_x"):
            _profile(unlocked_velocity=_ceiling(x=0.2))

    def test_unlocked_yaw_above_ceiling_is_refused(self):
        with pytest.raises(SafetyError, match="max_abs_yaw"):
            _profile(unlocked_velocity=_ceiling(yaw=0.9))

    def test_locked_limits_are_checked_too(self):
        with pytest.raises(SafetyError, match="max_abs_y"):
            _profile(locked_velocity=_ceiling(y=0.5))


class TestProfileSelectsLimits:
    def test_locked_returns_locked_velocity(self):
        p = _profile()
        assert p.velocity_limits(motion_unlocked=False) is p.locked_velocity

    def test_unlocked_returns_unlocked_velocity(self):
        p = _profile()
        assert p.velocity_limits(motion_unlocked=True) is p.unlocked_velocity

    def test_duration_follows_the_same_switch(self):
        p = _profile()
        assert p.max_duration_s(False) == 1.0
        assert p.max_duration_s(True) == 15.0


class TestRospugProfileMatchesConfig:
    """The extraction must not silently change any live limit."""

    def test_unlocked_velocity_matches_config(self):
        v = ROSPUG_SAFETY.velocity_limits(motion_unlocked=True)
        assert v.max_abs_x == config.VERIFIED_MAX_ABS_X_M_S
        assert v.max_abs_y == config.VERIFIED_MAX_ABS_Y_M_S
        assert v.max_abs_yaw == config.VERIFIED_MAX_ABS_YAW_RAD_S

    def test_locked_velocity_matches_config(self):
        v = ROSPUG_SAFETY.velocity_limits(motion_unlocked=False)
        assert v.max_abs_x == config.UNVERIFIED_MAX_ABS_X
        assert v.max_abs_yaw == config.UNVERIFIED_MAX_ABS_YAW

    def test_pose_matches_config(self):
        assert ROSPUG_SAFETY.pose.height_min == config.HEIGHT_MIN_M
        assert ROSPUG_SAFETY.pose.max_abs_roll == config.MAX_ABS_ROLL_RAD

    def test_battery_floor_matches_config(self):
        assert ROSPUG_SAFETY.battery_motion_min_mv == config.BATTERY_MOTION_MIN_MV


class TestGuardUsesItsProfile:
    def test_profile_is_required(self):
        with pytest.raises(TypeError):
            SafetyGuard()

    def test_guard_clamps_using_the_supplied_profile(self):
        tiny = _profile(unlocked_velocity=_ceiling(x=0.01, y=0.01, yaw=0.01))
        g = SafetyGuard(profile=tiny, motion_unlocked=True)
        assert g.guard_velocity(Velocity(x=5.0)).x == pytest.approx(0.01)

    def test_whitelist_defaults_to_the_profile_whitelist(self):
        g = SafetyGuard(profile=_profile(action_whitelist=("stand",)))
        assert g.guard_action("stand") == "stand"

    def test_explicit_whitelist_overrides_the_profile(self):
        g = SafetyGuard(profile=_profile(action_whitelist=("stand",)),
                        action_whitelist=("sit",))
        assert g.guard_action("sit") == "sit"
        with pytest.raises(SafetyError):
            g.guard_action("stand")


class TestGuardBattery:
    def test_below_floor_is_refused(self):
        g = SafetyGuard(profile=_profile(battery_motion_min_mv=11100))
        with pytest.raises(SafetyError, match="below the motion floor"):
            g.guard_battery(11000)

    def test_above_floor_passes(self):
        g = SafetyGuard(profile=_profile(battery_motion_min_mv=11100))
        g.guard_battery(11200)

    def test_mains_powered_profile_skips_the_gate(self):
        """SO-ARM101 runs off a PSU -- there is no pack to gate on. Both
        battery fields must be NO_BATTERY together (see
        TestBatteryFieldsMustBeBothOrNeither below) -- a floor of NO_BATTERY
        with a real warn level is now a construction error."""
        g = SafetyGuard(profile=_profile(battery_motion_min_mv=NO_BATTERY,
                                         battery_warn_mv=NO_BATTERY))
        g.guard_battery(None)


class TestTheBatteryGateCannotBeOmittedByAccident:
    """REGRESSION GUARD: battery_motion_min_mv used to default to None, i.e.
    'no battery gate'. A profile author adding the fleet's next robot and
    simply not thinking about the pack got a robot that would drive its LiPo
    flat, with nothing anywhere reporting a problem -- fail-open in the one
    layer that matters.

    Forgetting is now a TypeError at construction. Declining a battery gate is
    still allowed, but only by saying NO_BATTERY out loud.
    """

    def _without(self, *drop):
        base = dict(
            name="forgetful",
            locked_velocity=_ceiling(0.05, 0.025, 0.15),
            unlocked_velocity=_ceiling(),
            vendor_ceiling=_ceiling(),
            pose=PoseLimits(max_abs_roll=0.8, max_abs_pitch=0.8, height_min=-0.22,
                            height_max=-0.04, max_abs_x_shift=0.08, max_abs_stance=0.08),
            locked_max_duration_s=1.0,
            unlocked_max_duration_s=15.0,
            max_steps_per_call=4,
            battery_motion_min_mv=11100,
            battery_warn_mv=10500,
        )
        for key in drop:
            base.pop(key)
        return base

    def test_omitting_the_motion_floor_is_a_construction_error(self):
        with pytest.raises(TypeError, match="battery_motion_min_mv"):
            SafetyProfile(**self._without("battery_motion_min_mv", "battery_warn_mv"))

    def test_omitting_only_the_warn_level_is_also_a_construction_error(self):
        with pytest.raises(TypeError, match="battery_warn_mv"):
            SafetyProfile(**self._without("battery_warn_mv"))

    def test_declining_the_gate_explicitly_is_allowed(self):
        p = _profile(battery_motion_min_mv=NO_BATTERY, battery_warn_mv=NO_BATTERY)
        assert p.battery_motion_min_mv is None
        SafetyGuard(profile=p).guard_battery(None)   # no pack, no refusal

    def test_no_battery_sentinel_is_a_readable_spelling_of_none(self):
        assert NO_BATTERY is None

    def test_the_rospug_states_its_floor_explicitly(self):
        assert ROSPUG_SAFETY.battery_motion_min_mv == config.BATTERY_MOTION_MIN_MV
        assert ROSPUG_SAFETY.battery_warn_mv == config.BATTERY_WARN_MV


class TestBatteryFieldsMustBeBothOrNeither:
    """REGRESSION GUARD: the fix above (TestTheBatteryGateCannotBeOmittedByAccident)
    made it impossible to OMIT the battery fields, but not impossible to make
    them INCONSISTENT.

    A profile with battery_motion_min_mv=NO_BATTERY, battery_warn_mv=10500
    builds fine, looks battery-aware, but has no motion gate at all --
    guard_battery() no-ops on a None floor, so guard_battery(3000) returns
    without raising however flat the pack. The mirror direction (floor=11100,
    warn=NO_BATTERY) builds fine too, but battery_state() only ever checks
    warn when it is not None, so it can never report 'critical'. Both fields
    must move together: NO_BATTERY/NO_BATTERY, or a real value/real value.
    """

    def test_floor_set_but_warn_is_no_battery_is_refused(self):
        with pytest.raises(SafetyError, match="battery_motion_min_mv"):
            _profile(battery_motion_min_mv=11100, battery_warn_mv=NO_BATTERY)

    def test_floor_is_no_battery_but_warn_is_set_is_refused(self):
        with pytest.raises(SafetyError, match="battery_warn_mv"):
            _profile(battery_motion_min_mv=NO_BATTERY, battery_warn_mv=10500)

    def test_both_no_battery_is_allowed(self):
        p = _profile(battery_motion_min_mv=NO_BATTERY, battery_warn_mv=NO_BATTERY)
        assert p.battery_motion_min_mv is None
        assert p.battery_warn_mv is None

    def test_both_set_is_allowed(self):
        p = _profile(battery_motion_min_mv=11100, battery_warn_mv=10500)
        assert p.battery_motion_min_mv == 11100
        assert p.battery_warn_mv == 10500

    def test_error_names_the_profile_and_both_field_values(self):
        with pytest.raises(SafetyError) as excinfo:
            _profile(name="patchy", battery_motion_min_mv=11100,
                    battery_warn_mv=NO_BATTERY)
        msg = str(excinfo.value)
        assert "patchy" in msg
        assert "11100" in msg
        assert "None" in msg

    def test_replace_also_enforces_the_invariant(self):
        """dataclasses.replace() re-runs __post_init__, and profiles in the
        wild are commonly built via replace() from ROSPUG_SAFETY -- see
        TestGuardBattery / the 2S example above -- so the invariant must hold
        there too, not just on a from-scratch SafetyProfile(...) call."""
        from dataclasses import replace
        with pytest.raises(SafetyError, match="battery_warn_mv"):
            replace(ROSPUG_SAFETY, name="half-aware", battery_warn_mv=NO_BATTERY)
