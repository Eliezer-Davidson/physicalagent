"""Tests for the MiniHexa RobotProfile and its SafetyProfile.

MiniHexa is the first hexapod in the registry, and the first robot whose honest
supervision story is OPERATOR_BOUNDED: the firmware has NO watchdog and its walk
LATCHES a velocity, so the robot never self-stops — safe motion is the host's job
(bounded burst + stop) with a human present. Two facts drive its profile and must
be pinned: a hexapod is STATICALLY STABLE, so its safe state FREEZES (no settle-
to-stance); and it gates motion on a MEASURED 2S-LiPo floor (7000 mV), unlike the
TonyPi which has no battery telemetry. Its action whitelist starts EMPTY on
purpose.
"""
import pytest

from physicalagent.profiles import get_profile, list_profiles
from physicalagent.profiles.base import SafeStatePolicy, SupervisionClass


class TestMiniHexaRegistered:
    def test_minihexa_is_in_the_registry(self):
        assert "minihexa" in list_profiles()

    def test_get_profile_resolves_it(self):
        assert get_profile("minihexa").name == "minihexa"

    def test_lookup_is_case_insensitive(self):
        assert get_profile("MiniHexa").name == "minihexa"

    def test_the_other_robots_are_still_registered(self):
        names = list_profiles()
        assert "rospug" in names
        assert "tonypi" in names


class TestMiniHexaProfileShape:
    def test_it_is_a_hexapod(self):
        assert get_profile("minihexa").embodiment == "hexapod"

    def test_a_hexapod_freezes_rather_than_settling_to_a_stance(self):
        """Statically stable: freezing the legs where they are is safe, so no
        settle-to-stance (that is the biped's policy)."""
        assert get_profile("minihexa").safe_state is SafeStatePolicy.FREEZE

    def test_supervision_is_operator_bounded(self):
        """No watchdog + velocity-latched walk + mobile: none of DEADMAN,
        COMMAND_EXPIRY or TETHERED fit. The host bounds every burst."""
        assert get_profile("minihexa").supervision is SupervisionClass.OPERATOR_BOUNDED

    def test_motion_tools_are_enabled(self):
        # it HAS a supervision story (operator-bounded bursts), so motion tools
        # are allowed — the safety comes from the tier clamps + battery gate.
        assert get_profile("minihexa").motion_tools_enabled is True

    def test_motion_is_locked_by_default(self):
        assert get_profile("minihexa").motion_unlocked is False

    def test_it_links_over_serial_by_default(self):
        from physicalagent import config
        link = get_profile("minihexa").link
        assert link.kind == "serial"
        assert link.device == config.MINIHEXA.device
        assert link.baud == config.MINIHEXA.baud

    def test_it_has_a_hexapod_embodiment_prompt(self):
        assert "hexapod" in get_profile("minihexa").embodiment_prompt.lower()

    def test_it_carries_an_adapter_factory(self):
        assert callable(get_profile("minihexa").adapter_factory)

    def test_the_factory_builds_a_minihexa_adapter_defaulting_to_locked(self):
        from physicalagent.robots.minihexa import MiniHexaAdapter
        from physicalagent.robots.minihexa_link import FakeMiniHexaLink
        from physicalagent.robots.minihexa_motion import MotionTier
        from physicalagent.safety import MINIHEXA_SAFETY, SafetyGuard
        p = get_profile("minihexa")
        a = p.adapter_factory(FakeMiniHexaLink(), SafetyGuard(MINIHEXA_SAFETY))
        assert isinstance(a, MiniHexaAdapter)
        assert a._tier is MotionTier.LOCKED


class TestMiniHexaSafetyProfile:
    def test_it_carries_the_minihexa_safety_profile(self):
        from physicalagent.safety import MINIHEXA_SAFETY
        assert get_profile("minihexa").safety is MINIHEXA_SAFETY

    def test_it_gates_motion_on_a_2s_lipo_floor(self):
        s = get_profile("minihexa").safety
        assert s.battery_motion_min_mv == 7000    # 3.5 V/cell
        assert s.battery_warn_mv == 6600          # 3.3 V/cell

    def test_the_action_whitelist_is_empty_on_purpose(self):
        """No built-in action group is characterized yet; an empty whitelist is
        the framework's safe default (refuses all)."""
        assert get_profile("minihexa").safety.action_whitelist == ()

    def test_burst_caps_match_the_locked_and_full_durations(self):
        s = get_profile("minihexa").safety
        assert s.locked_max_duration_s == 0.4     # == LOCKED burst
        assert s.unlocked_max_duration_s == 1.5   # == FULL burst

    def test_it_builds_without_a_safety_error(self):
        assert get_profile("minihexa").safety is not None
