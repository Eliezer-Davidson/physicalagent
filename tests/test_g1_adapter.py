"""Tests for the G1 DESIGN SKETCH — pinning that it stays a sketch.

physicalagent/robots/g1.py and physicalagent/profiles/g1.py are proposals, not
working code, and the real design is still being written. These tests exist to
stop the sketch drifting into looking usable while nobody is watching: they
record the shape of the proposed tool surface, and they assert the two things
that MUST stay true until a real G1 link exists — the profile is not offered by
the registry, and the adapter refuses to invent telemetry.
"""
import pytest

from physicalagent.profiles import UnknownRobotError, get_profile
from physicalagent.profiles.g1 import G1
from physicalagent.robots.g1 import UnitreeG1Adapter
from physicalagent.safety import G1SafetyGuard, SafetyError


class TestTheSketchIsNotReachable:
    def test_g1_is_not_in_the_registry(self):
        # It has no adapter_factory and link_for() has no "dds" builder, so a
        # registered g1 would advertise a robot the CLI cannot build.
        with pytest.raises(UnknownRobotError):
            get_profile("g1")

    def test_the_draft_profile_still_imports_directly(self):
        assert G1.name == "g1"
        assert G1.embodiment == "humanoid"


class TestNoFabricatedSensors:
    def test_telemetry_refuses_rather_than_inventing_a_reading(self):
        # The original sketch returned a fixed "pitch 0.0, roll 0.0, standing".
        # A stability check fed that would never see a fall.
        with pytest.raises(NotImplementedError, match="not implemented"):
            UnitreeG1Adapter().telemetry()


class TestProposedToolShape:
    """What the sketch proposes to offer a model on a G1. Recorded, not endorsed
    — the groot_* tools in particular name a policy stack that does not exist in
    this workspace, which is one of the questions the design has to answer."""

    def test_it_proposes_velocity_walking_and_a_seated_safe_pose(self):
        names = {c.name for c in UnitreeG1Adapter().capabilities()}
        assert {"walk_velocity", "sit_down"} <= names

    def test_it_proposes_cartesian_arm_control(self):
        names = {c.name for c in UnitreeG1Adapter().capabilities()}
        assert "move_arm_ik" in names

    def test_every_proposed_mover_is_declared_a_motion_tool(self):
        a = UnitreeG1Adapter()
        names = {c.name for c in a.capabilities()}
        assert names <= a._motion_tool_names() | a._always_allowed_tools()

    def test_stop_is_always_allowed(self):
        assert "stop" in UnitreeG1Adapter()._always_allowed_tools()


class TestDynamicStabilityCheckIsUnvalidated:
    """G1SafetyGuard.check_dynamic_stability exists but NOTHING CALLS IT, and
    its 0.5 rad thresholds are, in its own words, "simple thresholds for
    demonstration" — not measured against any reference motion. Both facts are
    pinned here so neither is mistaken for a working fall detector.

    There is also a design question these tests do not answer: e-stopping a
    walking biped at 0.5 rad of tilt may CAUSE the fall it is detecting. A guard
    has to be validated against the motion it is guarding before it is armed.
    """

    def test_it_trips_on_large_tilt(self):
        guard = G1SafetyGuard(G1.safety)
        guard.check_dynamic_stability(pitch=0.1, roll=0.2)
        assert not guard.is_estopped
        with pytest.raises(SafetyError, match="falling"):
            guard.check_dynamic_stability(pitch=0.6, roll=0.1)
        assert guard.is_estopped

    def test_no_adapter_calls_it(self):
        # The check is decorative until something feeds it real IMU data.
        import inspect

        from physicalagent.robots import g1 as g1_module
        assert "check_dynamic_stability" not in inspect.getsource(g1_module)
