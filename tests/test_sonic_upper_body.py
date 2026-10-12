"""Upper-body joint control: the table, the clamps, and the wire format.

The wire format is checked against the vendor's OWN sender
(gear_sonic_deploy/.../tests/test_zmq_manager.py), because this protocol fails
silently: a wrong dtype, shape or field name produces no error and no motion,
and the project has already lost time to exactly that with the 1280-byte header.
"""
from __future__ import annotations

import json
import pathlib
import struct

import pytest

from physicalagent.robots import sonic_upper_body as ub
from physicalagent.robots import sonic_zmq as z

from physicalagent.paths import WBC_DEPLOY, WBC_ROOT

_DEPLOY_G1 = WBC_DEPLOY / "src" / "g1" / "g1_deploy_onnx_ref"
VENDOR_SENDER = _DEPLOY_G1 / "tests" / "test_zmq_manager.py"
VENDOR_PARAMS = _DEPLOY_G1 / "include" / "policy_parameters.hpp"


def vendor_wire_order() -> list[str]:
    """The wire order, derived from the VENDOR's own index map rather than from
    anything we wrote.

    upper_body_joint_isaaclab_order_in_mujoco_index maps wire slot -> motor
    index, and sonic_telemetry.JOINT_NAMES maps motor index -> name. Composing
    them gives the order the policy actually expects, with no input from us.
    """
    import re

    from physicalagent.robots import sonic_telemetry as tel

    text = VENDOR_PARAMS.read_text()
    m = re.search(r"upper_body_joint_isaaclab_order_in_mujoco_index\s*=\s*\{([^}]*)\}",
                  text)
    assert m, "the vendor index map is gone or was renamed -- do not guess, go read it"
    motor_indices = [int(tok) for tok in m.group(1).replace(",", " ").split()]
    assert len(motor_indices) == 17, motor_indices
    return [tel.JOINT_NAMES[i] for i in motor_indices]


class TestJointTable:
    def test_seventeen_joints_waist_then_left_then_right(self):
        assert len(ub.JOINT_NAMES) == ub.NUM_UPPER_BODY_JOINTS == 17
        assert ub.JOINT_NAMES[:3] == ("waist_yaw", "waist_roll", "waist_pitch")
        assert ub.JOINT_NAMES[3].startswith("left_")
        assert ub.JOINT_NAMES[10].startswith("right_")

    def test_our_order_matches_the_telemetry_order_we_read_back(self):
        """Our internal order equals the readback order, so a commanded list can
        be compared elementwise with a measured one.

        NOTE what this does NOT establish. It once carried a comment claiming it
        was 'the check that matters' for the wire format. It is not: both tables
        are ours, both are serial, and the wire is neither. That mistake shipped.
        The wire is covered by test_wire_order_matches_the_vendors_own_map.
        """
        from physicalagent.robots import sonic_telemetry as tel

        assert tuple(tel.JOINT_NAMES[12:29]) == ub.JOINT_NAMES


class TestWireOrder:
    """The permutation between our order and the policy's, checked against the
    vendor -- the check that was missing when the scrambled order shipped."""

    @pytest.mark.skipif(not VENDOR_PARAMS.exists(), reason="vendor header absent")
    def test_wire_order_matches_the_vendors_own_map(self):
        assert list(ub.WIRE_JOINT_NAMES) == vendor_wire_order()

    def test_wire_order_is_interleaved_not_serial(self):
        """The property in one line: after the waist it alternates sides. If a
        future edit 'tidies' this back into left-then-right, this fails."""
        assert ub.WIRE_JOINT_NAMES[3:7] == (
            "left_shoulder_pitch", "right_shoulder_pitch",
            "left_shoulder_roll", "right_shoulder_roll")
        assert ub.WIRE_JOINT_NAMES != ub.JOINT_NAMES

    def test_to_wire_is_a_permutation_and_round_trips(self):
        values = [0.1 * i for i in range(17)]
        assert sorted(ub.to_wire(values)) == pytest.approx(sorted(values))
        assert ub.from_wire(ub.to_wire(values)) == pytest.approx(values)

    def test_to_wire_puts_each_joint_where_the_policy_reads_it(self):
        """Name-indexed, so it states the intent rather than restating TO_WIRE."""
        values = [0.0] * 17
        values[ub.INDEX["right_elbow"]] = 1.23
        wire = ub.to_wire(values)
        assert wire[ub.WIRE_JOINT_NAMES.index("right_elbow")] == pytest.approx(1.23)
        assert sum(abs(v) for v in wire) == pytest.approx(1.23)

    def test_the_exact_bug_that_shipped(self):
        """Regression, stated as the observed symptom.

        Sending our serial order unpermuted put right_shoulder_pitch's value on
        right_elbow and left the right shoulder tracking a number nobody asked
        for -- which read as 'the right arm tracks 59 degrees worse than the
        left'. Both shoulders commanded alike must now arrive alike.
        """
        vals, _ = ub.from_named_joints({"left_shoulder_pitch": -1.4,
                                        "right_shoulder_pitch": -1.4})
        wire = ub.to_wire(vals)
        assert wire[ub.WIRE_JOINT_NAMES.index("left_shoulder_pitch")] == pytest.approx(-1.4)
        assert wire[ub.WIRE_JOINT_NAMES.index("right_shoulder_pitch")] == pytest.approx(-1.4)
        # ...and the joint that used to receive it does not.
        assert wire[ub.WIRE_JOINT_NAMES.index("right_elbow")] == pytest.approx(
            ub.NEUTRAL[ub.INDEX["right_elbow"]])

    def test_no_leg_joint_is_in_the_table(self):
        for name in ub.JOINT_NAMES:
            assert not any(p in name for p in ("hip", "knee", "ankle"))

    def test_limits_are_ordered_and_the_right_length(self):
        assert len(ub.LOWER) == len(ub.UPPER) == 17
        for lo, hi in zip(ub.LOWER, ub.UPPER):
            assert lo < hi

    def test_neutral_matches_the_policys_own_default(self):
        """NEUTRAL is loco_upper_body_dof_pos from the wbc yaml. Drifting from it
        means 'hands back to the controller' would hand back a different pose
        from the one the controller expects."""
        yaml = (WBC_ROOT / "gear_sonic" / "utils" / "mujoco_sim" / "wbc_configs"
                / "g1_29dof_sonic_model12.yaml")
        if not yaml.exists():
            pytest.skip("wbc yaml not on this box")
        text = yaml.read_text()
        block = text.split("loco_upper_body_dof_pos:")[1].split("]")[0]
        nums = [float(tok) for tok in block.replace("[", " ").replace(",", " ").split()
                if tok.replace("-", "").replace(".", "").isdigit()]
        assert len(nums) == 17
        assert list(ub.NEUTRAL) == pytest.approx(nums)


class TestClamping:
    def test_out_of_range_is_clamped_not_refused(self):
        out = ub.clamp([99.0] * 17)
        assert out == list(ub.UPPER)
        assert ub.clamp([-99.0] * 17) == list(ub.LOWER)

    def test_in_range_passes_through(self):
        assert ub.clamp(ub.NEUTRAL) == list(ub.NEUTRAL)


class TestPartialCommands:
    def test_only_named_joints_change(self):
        vals, unknown = ub.from_named_joints({"right_elbow": 1.2})
        assert unknown == []
        assert vals[ub.INDEX["right_elbow"]] == pytest.approx(1.2)
        for i, v in enumerate(vals):
            if i != ub.INDEX["right_elbow"]:
                assert v == pytest.approx(ub.NEUTRAL[i])

    def test_unknown_joints_are_returned_not_silently_dropped(self):
        """A model whose typo vanished concludes the arm is broken."""
        vals, unknown = ub.from_named_joints({"right_elbow": 1.0, "left_knee": 0.5,
                                              "elbow": 0.2})
        assert set(unknown) == {"left_knee", "elbow"}
        assert vals[ub.INDEX["right_elbow"]] == pytest.approx(1.0)

    def test_merging_onto_a_base_preserves_the_rest_of_the_pose(self):
        base = ub.pose("arms_forward")
        vals, _ = ub.from_named_joints({"right_elbow": 1.5}, base=base)
        assert vals[ub.INDEX["right_elbow"]] == pytest.approx(1.5)
        assert vals[ub.INDEX["left_elbow"]] == pytest.approx(base[ub.INDEX["left_elbow"]])

    def test_partial_command_result_is_still_clamped(self):
        vals, _ = ub.from_named_joints({"waist_roll": 99.0})
        assert vals[ub.INDEX["waist_roll"]] == pytest.approx(ub.UPPER[1])


class TestPoses:
    def test_every_named_pose_is_within_limits(self):
        for name in ub.POSES:
            vals = ub.pose(name)
            assert len(vals) == 17
            assert vals == ub.clamp(vals), f"{name} is outside the joint limits"

    def test_unknown_pose_lists_the_menu(self):
        with pytest.raises(ValueError) as exc:
            ub.pose("interpretive_dance")
        assert "arms_forward" in str(exc.value)

    def test_arms_forward_actually_raises_both_arms(self):
        """Shoulder pitch is NEGATIVE forward on this robot -- the sign most
        likely to be wrong, and wrong silently."""
        vals = ub.pose("arms_forward")
        assert vals[ub.INDEX["left_shoulder_pitch"]] < -0.5
        assert vals[ub.INDEX["right_shoulder_pitch"]] < -0.5

    def test_arms_out_moves_them_to_opposite_sides(self):
        vals = ub.pose("arms_out")
        assert vals[ub.INDEX["left_shoulder_roll"]] > 0.5
        assert vals[ub.INDEX["right_shoulder_roll"]] < -0.5


class TestWireFormat:
    def test_upper_body_is_omitted_when_not_commanded(self):
        """Absent means the policy keeps the arms. The field must not appear at
        all -- sending neutral instead would take control and hold it."""
        msg = z.encode_planner(0, (1, 0, 0), (1, 0, 0), 0.0, -1.0)
        header = json.loads(msg[len(b"planner"):len(b"planner") + z.HEADER_SIZE]
                            .rstrip(b"\x00").decode())
        assert [f["name"] for f in header["fields"]] == [
            "mode", "movement", "facing", "speed", "height"]

    def test_upper_body_field_matches_the_vendors_own_sender(self):
        """Name, dtype and shape, verbatim from test_zmq_manager.py:126."""
        msg = z.encode_planner(0, (1, 0, 0), (1, 0, 0), 0.0, -1.0,
                               upper_body=list(ub.NEUTRAL))
        header = json.loads(msg[len(b"planner"):len(b"planner") + z.HEADER_SIZE]
                            .rstrip(b"\x00").decode())
        field = header["fields"][-1]
        assert field == {"name": "upper_body_position", "dtype": "f32", "shape": [17]}

    @pytest.mark.skipif(not VENDOR_SENDER.exists(), reason="vendor sender absent")
    def test_the_vendor_declares_the_same_field_spec(self):
        text = VENDOR_SENDER.read_text()
        assert '{"name": "upper_body_position", "dtype": "f32", "shape": [17]}' in text

    def test_payload_is_17_little_endian_floats_IN_WIRE_ORDER(self):
        """The encoder permutes. This test asserted straight-through before, and
        passing was exactly what let the scrambled order reach the robot."""
        values = [0.1 * i for i in range(17)]
        msg = z.encode_planner(1, (1, 0, 0), (1, 0, 0), 0.5, -1.0,
                               upper_body=values)
        tail = list(struct.unpack("<17f", msg[-17 * 4:]))
        assert tail == pytest.approx(ub.to_wire(values), abs=1e-6)
        assert tail != pytest.approx(values, abs=1e-6), "not permuted at all"

    def test_a_single_named_joint_lands_on_that_joint_on_the_wire(self):
        """End to end through the real encoder: name in, correct wire slot out."""
        vals, _ = ub.from_named_joints({"left_elbow": 1.5})
        msg = z.encode_planner(0, (0, 0, 0), (1, 0, 0), 0.0, -1.0, upper_body=vals)
        tail = list(struct.unpack("<17f", msg[-17 * 4:]))
        assert tail[ub.WIRE_JOINT_NAMES.index("left_elbow")] == pytest.approx(1.5)

    def test_wrong_length_is_refused_loudly(self):
        with pytest.raises(ValueError):
            z.encode_planner(0, (1, 0, 0), (1, 0, 0), 0.0, -1.0, upper_body=[0.0] * 16)

    def test_header_still_fits_its_fixed_slot(self):
        """The header is a fixed 1280-byte field; overflow is silently never
        parsed. Adding a field brings it closer to the edge."""
        msg = z.encode_planner(0, (1, 0, 0), (1, 0, 0), 0.0, -1.0,
                               upper_body=list(ub.NEUTRAL))
        raw = msg[len(b"planner"):len(b"planner") + z.HEADER_SIZE]
        assert len(raw.rstrip(b"\x00")) < z.HEADER_SIZE
