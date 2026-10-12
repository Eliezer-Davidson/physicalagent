"""Named grasps expand to joint targets that are inside the model's limits.

A finger target outside its joint range is not an error anywhere in the chain:
MuJoCo clamps it, the PD law pushes against the stop forever, and the hand looks
subtly wrong while burning torque. So the expansion is checked against the
model's own declared ranges.
"""
from __future__ import annotations

import pathlib

import pytest

from physicalagent.robots import inspire_grasps as g

from physicalagent.paths import G1_MODEL_DIR

MODEL = G1_MODEL_DIR / "g1_29dof_with_inspire_actuated.xml"


class TestLibrary:
    def test_the_eight_grasps_are_all_present(self):
        assert set(g.grasp_names()) == {
            "open_hand", "flat_open", "power_grasp", "precision_pinch",
            "tripod", "lateral_key", "hook", "index_point"}

    def test_sizes_and_strengths_exclude_the_doc_keys(self):
        assert set(g.size_names()) == {"small", "medium", "large"}
        assert set(g.strength_names()) == {"light", "firm", "tight"}

    def test_describe_names_every_grasp_with_a_reason_to_pick_it(self):
        text = g.describe()
        for name in g.grasp_names():
            assert name in text
        assert "bottle" in text          # power_grasp's tags reach the model


class TestExpansion:
    def test_twelve_joints_per_hand(self):
        assert len(g.expand("power_grasp", side="right")) == 12

    def test_sides_are_prefixed_and_never_mixed(self):
        right = g.expand("power_grasp", side="right")
        left = g.expand("power_grasp", side="left")
        assert all(k.startswith("R_") for k in right)
        assert all(k.startswith("L_") for k in left)
        assert set(right) & set(left) == set()

    def test_open_hand_is_actually_open(self):
        """Every curl joint at zero. If 'release' quietly curled a finger, a
        model told to let go would keep holding on."""
        for joint, value in g.expand("open_hand", side="right").items():
            if "yaw" in joint:
                continue                  # thumb yaw's open end is -0.1, not 0
            assert value == pytest.approx(0.0, abs=1e-9), joint

    def test_strength_only_tightens_fingers_that_are_already_closing(self):
        """Over-curl applied to an open finger would turn a release into a
        light grip -- silently, and only when 'tight' was asked for."""
        light = g.expand("precision_pinch", strength="light", side="right")
        tight = g.expand("precision_pinch", strength="tight", side="right")
        # the pinch leaves pinky and ring extended
        assert light["R_pinky_proximal_joint"] == tight["R_pinky_proximal_joint"] == 0.0
        # ...and closes the index, which tight must close further
        assert tight["R_index_proximal_joint"] > light["R_index_proximal_joint"]

    def test_size_scales_only_sizeable_grasps(self):
        small = g.expand("power_grasp", size="small", side="right")
        large = g.expand("power_grasp", size="large", side="right")
        assert small["R_index_proximal_joint"] > large["R_index_proximal_joint"]
        # index_point is not sizeable: size must be a no-op, not a quiet change
        assert (g.expand("index_point", size="small", side="right")
                == g.expand("index_point", size="large", side="right"))


class TestRefusals:
    def test_unknown_grasp_lists_the_menu(self):
        with pytest.raises(ValueError) as exc:
            g.expand("crush", side="right")
        assert "power_grasp" in str(exc.value)

    @pytest.mark.parametrize("kwargs", [
        {"side": "middle"}, {"size": "enormous"}, {"strength": "crushing"}])
    def test_bad_arguments_are_refused_with_the_alternatives(self, kwargs):
        with pytest.raises(ValueError):
            g.expand("power_grasp", **{"side": "right", **kwargs})


class TestAgainstTheModel:
    @pytest.mark.skipif(not MODEL.exists(), reason="actuated model not generated")
    def test_every_target_is_inside_its_joint_range(self):
        """The real check: expand every grasp at every size and strength, and
        compare each target against the range declared in the MJCF."""
        mujoco = pytest.importorskip("mujoco")
        m = mujoco.MjModel.from_xml_path(str(MODEL))
        ranges = {}
        for j in range(m.njnt):
            name = m.joint(j).name
            if name and name.startswith(("L_", "R_")):
                ranges[name] = tuple(m.jnt_range[j])
        assert len(ranges) == 24, f"expected 24 finger joints, found {len(ranges)}"

        checked = 0
        for name in g.grasp_names():
            for size in g.size_names():
                for strength in g.strength_names():
                    for side in ("left", "right"):
                        for joint, value in g.expand(
                                name, size=size, strength=strength, side=side).items():
                            lo, hi = ranges[joint]
                            assert lo - 1e-6 <= value <= hi + 1e-6, (
                                f"{name}/{size}/{strength} {joint}={value:.3f} "
                                f"outside [{lo}, {hi}]")
                            checked += 1
        assert checked == 8 * 3 * 3 * 2 * 12

    @pytest.mark.skipif(not MODEL.exists(), reason="actuated model not generated")
    def test_every_finger_joint_has_an_actuator(self):
        """A target on an unactuated joint is a command that goes nowhere and
        reports success."""
        mujoco = pytest.importorskip("mujoco")
        m = mujoco.MjModel.from_xml_path(str(MODEL))
        actuated = {int(m.actuator_trnid[a][0]) for a in range(m.nu)}
        for j in range(m.njnt):
            name = m.joint(j).name
            if name and name.startswith(("L_", "R_")):
                assert j in actuated, f"{name} has no actuator"
