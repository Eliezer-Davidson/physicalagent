"""The 27-mode table, pinned against the vendor enum.

These integers are an enum ordinal on a wire with no error reporting: sending
the wrong one produces a robot doing a different thing, never an error. So the
table is checked against the header it came from rather than trusted.
"""
from __future__ import annotations

import pathlib
import re

import pytest

from physicalagent.robots import sonic_modes as sm

from physicalagent.paths import WBC_DEPLOY

_INCLUDE = WBC_DEPLOY / "src" / "g1" / "g1_deploy_onnx_ref" / "include"
HEADER = _INCLUDE / "localmotion_kplanner.hpp"


class TestAgainstTheVendorEnum:
    @pytest.mark.skipif(not HEADER.exists(), reason="vendor header not on this box")
    def test_every_value_matches_the_cpp_enum(self):
        """Parse LocomotionMode out of the header and compare value for value.

        This is the test that would have caught an off-by-one in the table, and
        it is the reason the table can be trusted without a robot in the loop.
        """
        text = HEADER.read_text()
        block = re.search(r"enum class LocomotionMode \{(.*?)\};", text, re.S)
        assert block, "LocomotionMode enum not found — did the vendor rename it?"
        vendor = {}
        for name, value in re.findall(r"(\w+)\s*=\s*(\d+)", block.group(1)):
            vendor[int(value)] = name
        assert len(vendor) == 27, f"vendor now has {len(vendor)} modes, table has 27"
        assert set(vendor) == set(sm.BY_VALUE), "value sets differ"

    def test_the_four_previously_exposed_modes_did_not_move(self):
        assert sm.BY_NAME["idle"].value == 0
        assert sm.BY_NAME["slow_walk"].value == 1
        assert sm.BY_NAME["walk"].value == 2
        assert sm.BY_NAME["squat"].value == 4

    def test_values_are_unique_and_contiguous(self):
        values = sorted(m.value for m in sm.MODES)
        assert values == list(range(27))


GAMEPAD = _INCLUDE / "input_interface" / "gamepad_manager.hpp"


class TestAgainstTheVendorOperatingPoint:
    """`speed` and `height` are the vendor's numbers, so they are read back OUT
    of the vendor's file rather than trusted to a copy-paste.

    This is the test that would have caught the bug it was written for: mode 17
    commanded at speed 0.0, which is not a slow jump, it is a robot told to stay
    put. Nothing errored, and the failure looked like a missing capability.
    """

    @pytest.mark.skipif(not GAMEPAD.exists(), reason="vendor gamepad not on this box")
    def test_speed_and_height_match_applySpeedAndHeight(self):
        text = GAMEPAD.read_text()
        block = re.search(r"void applySpeedAndHeight\(LocomotionMode mode\) \{(.*?)\n    \}",
                          text, re.S)
        assert block, "applySpeedAndHeight not found — did the vendor rename it?"

        # Walk the switch: labels accumulate until a body assigns both, then the
        # break flushes them. `default:` covers every mode the switch never named.
        vendor: dict[str, tuple[float, float]] = {}
        pending: list[str] = []
        speed = height = None
        has_default = False
        for line in block.group(1).splitlines():
            line = line.strip()
            if m := re.match(r"case LocomotionMode::(\w+):", line):
                pending.append(m.group(1))
            elif line.startswith("default:"):
                has_default = True
            elif m := re.match(r"planner_use_movement_speed_ = (-?[\d.]+);", line):
                speed = float(m.group(1))
            elif m := re.match(r"planner_use_height_ = (-?[\d.]+);", line):
                height = float(m.group(1))
            elif line.startswith("break;"):
                for name in pending:
                    vendor[name] = (speed, height)
                pending, speed, height = [], None, None
        assert has_default, "the switch lost its default arm"
        assert vendor, "parsed no cases out of applySpeedAndHeight"

        # Our names differ from the vendor's enum identifiers; map by value.
        enum_names = {}
        header = HEADER.read_text()
        eblock = re.search(r"enum class LocomotionMode \{(.*?)\};", header, re.S)
        for name, value in re.findall(r"(\w+)\s*=\s*(\d+)", eblock.group(1)):
            enum_names[int(value)] = name

        for mode in sm.MODES:
            want = vendor.get(enum_names[mode.value], (-1.0, -1.0))  # default arm
            assert (mode.speed, mode.height) == want, (
                f"{mode.name} (={mode.value}, vendor {enum_names[mode.value]}) "
                f"has {(mode.speed, mode.height)}, vendor says {want}")

    def test_no_standing_mode_defaults_to_a_dead_stop(self):
        """The vendor's gamepad cannot put a standing mode on the wire at speed
        0 — a centred stick switches to IDLE instead (gamepad.hpp:697-700). Our
        table must not be able to either, or jump_forward regresses."""
        for mode in sm.MODES:
            if mode.kind == sm.STANDING:
                assert abs(mode.speed) > 1e-9, f"{mode.name} would be told to stand still"

    def test_the_grounded_modes_ask_for_a_height_below_the_standing_floor(self):
        """If this ever passes trivially the per-kind floor has stopped mattering."""
        low = [m for m in sm.MODES if 0 < m.height < sm.HEIGHT_MIN_M]
        assert {m.name for m in low} == {"squat", "kneel_two_legs", "kneel",
                                         "crawl", "elbow_crawl"}
        for m in low:
            assert sm.clamp_height(m.height, m) == m.height, (
                f"{m.name} wants {m.height} m and the clamp moved it")
            assert sm.clamp_height(m.height, None) == sm.HEIGHT_MIN_M


class TestResolution:
    def test_aliases_resolve_to_real_modes(self):
        for alias, target in sm.ALIASES.items():
            assert sm.resolve(alias).name == target, alias

    def test_case_and_separators_are_forgiving(self):
        for spelling in ("Slow Walk", "slow-walk", "  SLOW_WALK "):
            assert sm.resolve(spelling).name == "slow_walk"

    def test_unknown_mode_raises_with_the_menu(self):
        """A refusal listing the alternatives is a turn the model can recover
        from; a bare KeyError is not."""
        with pytest.raises(ValueError) as exc:
            sm.resolve("moonwalk")
        assert "moonwalk" in str(exc.value)
        assert "zombie_walk" in str(exc.value)


class TestHonesty:
    def test_only_observed_modes_are_marked_tested(self):
        """Four modes have been seen driving this robot. Marking more as tested
        would be the exact overclaim this project keeps having to correct."""
        assert {m.name for m in sm.MODES if m.tested} == {
            "idle", "slow_walk", "walk", "squat"}

    def test_the_menu_flags_untested_modes_to_the_model(self):
        menu = sm.menu()
        assert "[untested]" in menu
        assert "zombie_walk" in menu
        # ...and does not flag the ones that are proven
        proven_line = next(ln for ln in menu.splitlines() if ln.startswith("walk "))
        assert "[untested]" not in proven_line

    def test_only_standing_modes_accept_a_speed(self):
        """Commanding speed at a kneeling robot is meaningless; the flag is what
        stops the adapter offering it."""
        assert sm.BY_NAME["slow_walk"].takes_speed
        assert not sm.BY_NAME["kneel"].takes_speed
        assert not sm.BY_NAME["squat"].takes_speed
        assert not sm.BY_NAME["lie_face_down"].takes_speed


class TestHeight:
    def test_none_keeps_the_mode_default_rather_than_picking_one(self):
        assert sm.clamp_height(None) == sm.HEIGHT_DEFAULT == -1.0
        assert sm.clamp_height(-1) == -1.0

    def test_out_of_range_is_clamped_not_rejected(self):
        assert sm.clamp_height(0.1) == sm.HEIGHT_MIN_M
        assert sm.clamp_height(9.0) == sm.HEIGHT_MAX_M

    def test_a_reasonable_height_passes_through(self):
        assert sm.clamp_height(0.65) == 0.65
