"""Tests for the TonyPi RobotProfile and its SafetyProfile.

TonyPi is the first humanoid in the registry. Two facts about a biped drive its
profile and must be pinned: a biped frozen mid-stride is a FALL, so its safe
state settles to a stable stance rather than freezing; and the tool surface we
expose is DISCRETE self-terminating commands, so its supervision class is
COMMAND_EXPIRY, not DEADMAN. Its safety is enforced by the servo/tier clamps, not
by a cmd_vel guard it does not have, and it has no battery telemetry path — so it
declares NO_BATTERY honestly rather than inventing a floor.
"""
import pytest

from physicalagent.profiles import get_profile, list_profiles
from physicalagent.profiles.base import SafeStatePolicy, SupervisionClass


class TestTonyPiRegistered:
    def test_tonypi_is_in_the_registry(self):
        assert "tonypi" in list_profiles()

    def test_get_profile_resolves_it(self):
        assert get_profile("tonypi").name == "tonypi"

    def test_lookup_is_case_insensitive(self):
        assert get_profile("TonyPi").name == "tonypi"

    def test_the_rospug_is_still_registered_too(self):
        assert "rospug" in list_profiles()


class TestTonyPiProfileShape:
    def test_it_is_a_humanoid(self):
        assert get_profile("tonypi").embodiment == "humanoid"

    def test_a_biped_settles_to_a_stance_rather_than_freezing(self):
        """The load-bearing biped fact: freezing mid-stride is a fall, so stop
        must settle to a stable stance."""
        assert get_profile("tonypi").safe_state is SafeStatePolicy.SETTLE_TO_STANCE

    def test_supervision_is_command_expiry_not_deadman(self):
        """The exposed tools are discrete bounded commands that self-terminate;
        there is no continuous-walk surface, so each command stops by default."""
        assert get_profile("tonypi").supervision is SupervisionClass.COMMAND_EXPIRY

    def test_motion_tools_are_enabled(self):
        assert get_profile("tonypi").motion_tools_enabled is True

    def test_motion_is_locked_by_default(self):
        """A new body has to earn its way out of locked; nothing has watched
        this robot move under our control yet."""
        assert get_profile("tonypi").motion_unlocked is False

    def test_it_links_over_json_rpc_http(self):
        from physicalagent import config
        link = get_profile("tonypi").link
        assert link.kind == "http_rpc"
        assert link.port == config.TONYPI.rpc_port

    def test_it_has_a_humanoid_embodiment_prompt(self):
        assert "humanoid" in get_profile("tonypi").embodiment_prompt.lower()

    def test_it_carries_an_adapter_factory(self):
        # the wiring the research found missing: a profile that can build itself
        assert callable(get_profile("tonypi").adapter_factory)

    def test_it_builds_without_a_safety_error(self):
        # get_profile succeeding at import time already proves __post_init__ and
        # the SafetyProfile's vendor-ceiling / battery-consistency checks passed.
        assert get_profile("tonypi").safety is not None


class TestTonyPiSafetyProfile:
    def test_it_carries_the_tonypi_safety_profile(self):
        from physicalagent.safety import TONYPI_SAFETY
        assert get_profile("tonypi").safety is TONYPI_SAFETY

    def test_it_declares_no_battery_telemetry(self):
        """The JSON-RPC API exposes no voltage method (GetBusServosPulse returns
        positions only), so a battery floor would be invented, not measured.
        NO_BATTERY is the honest declaration."""
        s = get_profile("tonypi").safety
        assert s.battery_motion_min_mv is None
        assert s.battery_warn_mv is None

    def test_the_action_whitelist_is_the_curated_safe_set(self):
        s = get_profile("tonypi").safety
        assert set(s.action_whitelist)  # non-empty
        # walks / kicks / get-ups are NOT in the initial safe set
        for banned in ("go_forward", "left_kick", "right_kick", "stand_up_front"):
            assert banned not in s.action_whitelist
        # standing, in-place gestures ARE
        assert "wave" in s.action_whitelist
        assert "stand" in s.action_whitelist

    def test_it_is_constructable_despite_having_no_cmd_vel(self):
        """SafetyProfile demands ROS-shaped velocity/pose fields; TonyPi has no
        cmd_vel, so those are honest zero/placeholder values and its real
        enforcement lives in the servo tier clamps. It must still build."""
        from physicalagent.safety import TONYPI_SAFETY, SafetyError
        # construction happened at import; re-assert the invariant holds
        assert isinstance(TONYPI_SAFETY.name, str)
