"""The RobotProfile registry -- the whole of the 'single command change'."""
import pytest

from physicalagent.profiles import UnknownRobotError, get_profile, list_profiles
from physicalagent.profiles.base import (
    LinkSpec, RobotProfile, SafeStatePolicy, SupervisionClass,
)
from physicalagent.safety import ROSPUG_SAFETY, Velocity


class TestRegistry:
    def test_rospug_is_registered(self):
        assert "rospug" in list_profiles()

    def test_get_profile_returns_the_rospug(self):
        assert get_profile("rospug").name == "rospug"

    def test_unknown_name_lists_what_is_available(self):
        with pytest.raises(KeyError, match="rospug"):
            get_profile("nosuchrobot")

    def test_lookup_is_case_insensitive(self):
        assert get_profile("ROSPug").name == "rospug"


class TestUnknownRobotErrorIsAKeyError:
    """REGRESSION GUARD: cli.main() narrows its `except KeyError` around
    args.func(args) down to `except UnknownRobotError` specifically, so a
    bare KeyError raised during real command work (e.g. an unchecked dict
    key in a telemetry message) isn't mislabelled as a bad --robot name.
    That narrowing is only safe because UnknownRobotError IS-A KeyError --
    every existing `pytest.raises(KeyError, ...)` call against get_profile
    (including test_unknown_name_lists_what_is_available above) must keep
    passing unchanged.
    """

    def test_get_profile_raises_the_specific_subclass(self):
        with pytest.raises(UnknownRobotError):
            get_profile("nosuchrobot")

    def test_unknown_robot_error_is_a_key_error_subclass(self):
        assert issubclass(UnknownRobotError, KeyError)

    def test_unknown_robot_error_message_lists_every_registered_robot(self):
        # Registering each new robot necessarily extends the "available"
        # enumeration; it is sorted, so minihexa now leads. This pins that the
        # message names EVERY registered robot, not a frozen single-robot string.
        with pytest.raises(UnknownRobotError) as excinfo:
            get_profile("nosuchrobot")
        assert excinfo.value.args[0] == (
            "unknown robot 'nosuchrobot'; available: minihexa, rospug, tonypi"
        )


class TestRospugProfile:
    def test_it_is_a_quadruped_with_a_deadman(self):
        p = get_profile("rospug")
        assert p.embodiment == "quadruped"
        assert p.supervision is SupervisionClass.DEADMAN

    def test_a_quadruped_freezes_as_its_safe_state(self):
        """Four legs hold it statically stable -- stopping IS safe here.
        This does NOT generalise; a biped frozen mid-stride falls."""
        assert get_profile("rospug").safe_state is SafeStatePolicy.FREEZE

    def test_it_carries_the_rospug_safety_profile(self):
        assert get_profile("rospug").safety is ROSPUG_SAFETY

    def test_it_links_over_rosbridge(self):
        """The port is asserted against config, not hard-coded: it is
        overridable via $ROSPUG_ROSBRIDGE_PORT, so a literal 9090 here would
        fail for anyone who has set it -- and would pass while the profile
        silently ignored the override, which is the bug worth catching."""
        from physicalagent import config
        link = get_profile("rospug").link
        assert link.kind == "rosbridge"
        assert link.port == config.ROSPUG.rosbridge_port

    def test_it_has_an_embodiment_prompt(self):
        assert "four-legged" in get_profile("rospug").embodiment_prompt

    def test_motion_tools_are_enabled(self):
        assert get_profile("rospug").motion_tools_enabled is True


class TestMotionUnlockIsPerRobot:
    """REGRESSION GUARD: the unlock lived in config.MOTION_UNLOCKED, a single
    global read from one env var -- and `./robot` exports that var set to 1
    unconditionally. The constant's own docstring said it "stays False until a
    human has watched THIS robot move under our control", but a global cannot
    express "this robot". The moment a second profile was registered, its very
    first run would have started at the vendor's full limits on the strength of
    the ROSPug's field testing.

    The decision is per-body, so it lives on the body, and locked is the
    default a new profile has to earn its way out of.
    """

    def _fresh(self, **over):
        base = dict(
            name="newbot", embodiment="hexapod",
            supervision=SupervisionClass.COMMAND_EXPIRY,
            safe_state=SafeStatePolicy.FREEZE,
            safety=ROSPUG_SAFETY,
            link=LinkSpec(kind="serial", device="/dev/ttyUSB0", baud=115200),
            embodiment_prompt="x",
        )
        base.update(over)
        return RobotProfile(**base)

    def test_a_fresh_profile_defaults_to_locked(self):
        assert self._fresh().motion_unlocked is False

    def test_the_rospug_is_unlocked_because_it_has_been_field_tested(self):
        assert get_profile("rospug").motion_unlocked is True

    def test_a_fresh_profile_gets_the_timid_limits_from_build_guard(self, monkeypatch):
        """The guard the CLI would actually build for a brand-new robot."""
        from physicalagent import config
        from physicalagent.cli import build_guard
        monkeypatch.delenv("PHYSICALAGENT_MOTION_UNLOCKED", raising=False)
        g = build_guard(self._fresh())
        assert g.motion_unlocked is False
        assert g.guard_velocity(Velocity(x=99.0)).x == config.UNVERIFIED_MAX_ABS_X

    def test_the_launcher_env_var_cannot_unlock_a_fresh_profile(self, monkeypatch):
        """`./robot` exports PHYSICALAGENT_MOTION_UNLOCKED=1 unconditionally.
        A launcher that knows nothing about the robot must not be able to hand
        it the vendor's full limits."""
        from physicalagent.cli import build_guard
        monkeypatch.setenv("PHYSICALAGENT_MOTION_UNLOCKED", "1")
        assert build_guard(self._fresh()).motion_unlocked is False

    def test_the_env_var_can_still_lock_a_field_tested_robot(self, monkeypatch):
        """The override survives in the conservative direction: setting it to 0
        is how an operator asks for the timid caps on a known-good robot."""
        from physicalagent.cli import build_guard
        monkeypatch.setenv("PHYSICALAGENT_MOTION_UNLOCKED", "0")
        assert build_guard(get_profile("rospug")).motion_unlocked is False

    def test_the_rospug_under_the_launcher_is_unlocked_exactly_as_before(self, monkeypatch):
        """Today's workflow: ./robot exports =1, ROSPug runs at 0.10 m/s."""
        from physicalagent import config
        from physicalagent.cli import build_guard
        monkeypatch.setenv("PHYSICALAGENT_MOTION_UNLOCKED", "1")
        g = build_guard(get_profile("rospug"))
        assert g.motion_unlocked is True
        assert g.guard_velocity(Velocity(x=99.0)).x == config.VERIFIED_MAX_ABS_X_M_S


class TestSupervisionContract:
    def test_a_robot_with_no_supervision_story_gets_no_motion_tools(self):
        """Spec 4.1: 'A robot with no supervision story gets no motion tools.
        Not negotiable.' Enforced at construction."""
        from physicalagent.safety import SafetyError
        with pytest.raises(SafetyError, match="motion_tools_enabled"):
            RobotProfile(
                name="unsafe", embodiment="hexapod",
                supervision=None,
                safe_state=SafeStatePolicy.FREEZE,
                safety=ROSPUG_SAFETY,
                link=LinkSpec(kind="serial", device="/dev/ttyUSB0", baud=115200),
                embodiment_prompt="x",
                motion_tools_enabled=True,
            )

    def test_sensing_only_robot_without_supervision_is_allowed(self):
        """MiniHexa pre-bench-test: no supervision, but no motion tools either."""
        p = RobotProfile(
            name="minihexa", embodiment="hexapod",
            supervision=None,
            safe_state=SafeStatePolicy.FREEZE,
            safety=ROSPUG_SAFETY,
            link=LinkSpec(kind="serial", device="/dev/ttyUSB0", baud=115200),
            embodiment_prompt="x",
            motion_tools_enabled=False,
        )
        assert p.motion_tools_enabled is False
