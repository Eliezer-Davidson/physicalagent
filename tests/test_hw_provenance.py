"""Guards for the LOCKED-tier provenance dump.

The recorder answers "what did the robot do". This answers "what WAS the
robot" -- the question that becomes permanently unanswerable on Friday.

Offline: no DDS, no robot, no simulator.
"""
from __future__ import annotations

import dataclasses
import importlib.util
import json
import pathlib
import sys

import pytest

REPO = pathlib.Path(__file__).resolve().parents[1]
SCRIPTS = REPO / "scripts"


def _load(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def prov():
    return _load("hw_provenance")


# --- fakes shaped like the real unitree_hg IDL --------------------------------
# These are dataclasses because the real IDL types are, and describe_message
# walks dataclass fields. A fake with a different shape would test a path the
# robot never takes.

@dataclasses.dataclass
class FakeIMU:
    quaternion: list = dataclasses.field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
    gyroscope: list = dataclasses.field(default_factory=lambda: [0.01, -0.02, 0.003])
    accelerometer: list = dataclasses.field(default_factory=lambda: [0.1, 0.0, 9.79])
    rpy: list = dataclasses.field(default_factory=lambda: [0.001, 0.002, 1.57])
    temperature: int = 41


@dataclasses.dataclass
class FakeMotor:
    mode: int = 1
    q: float = 0.0
    dq: float = 0.0
    ddq: float = 0.0
    tau_est: float = 0.0
    temperature: list = dataclasses.field(default_factory=lambda: [38, 39])
    vol: float = 48.2
    sensor: list = dataclasses.field(default_factory=lambda: [0, 0])
    motorstate: int = 0
    reserve: list = dataclasses.field(default_factory=lambda: [0, 0, 0, 0])


@dataclasses.dataclass
class FakeLowState:
    version: list = dataclasses.field(default_factory=lambda: [1, 33554688])
    mode_pr: int = 0
    mode_machine: int = 5
    tick: int = 918273
    imu_state: FakeIMU = dataclasses.field(default_factory=FakeIMU)
    motor_state: list = dataclasses.field(default_factory=list)
    wireless_remote: list = dataclasses.field(default_factory=lambda: [0] * 40)
    reserve: list = dataclasses.field(default_factory=lambda: [0, 0, 0, 0])
    crc: int = 3735928559


def lowstate(n_motor=29):
    ls = FakeLowState()
    ls.motor_state = [FakeMotor(q=0.01 * i, motorstate=i, temperature=[30 + i, 31 + i])
                      for i in range(n_motor)]
    return ls


class TestTheSnapshotKeepsWhatTheRecorderDrops:
    """hw_recorder packs a fixed struct chosen for streaming at 500 Hz: tick,
    modes, IMU, and four floats per motor. Everything else in LowState_ is
    dropped -- including the firmware version and every per-motor fault and
    thermal field. Those are constants during a session, worthless to stream,
    and unrecoverable once the robot is gone."""

    def test_firmware_version_is_captured(self, prov):
        d = prov.describe_message(lowstate())
        assert d["version"] == [1, 33554688]

    def test_per_motor_status_word_and_temperature_are_captured(self, prov):
        d = prov.describe_message(lowstate())
        m7 = d["motor_state"][7]
        assert m7["motorstate"] == 7
        assert m7["temperature"] == [37, 38]
        assert m7["vol"] == pytest.approx(48.2)

    def test_every_motor_is_captured_not_just_the_first(self, prov):
        d = prov.describe_message(lowstate(n_motor=29))
        assert len(d["motor_state"]) == 29
        assert d["motor_state"][28]["q"] == pytest.approx(0.28)

    def test_motor_count_comes_from_the_message_not_a_constant(self, prov):
        """The IDL declares the array 35 wide; the G1 populates 29. A constant
        either truncates a different robot or invents motors this one lacks."""
        assert len(prov.describe_message(lowstate(n_motor=35))["motor_state"]) == 35

    def test_resting_pose_is_captured_to_full_precision(self, prov):
        """The at-rest joint vector is the calibration zero reference. There is
        no way to recover it away from the robot, and every retarget is quoted against it."""
        d = prov.describe_message(lowstate())
        assert [m["q"] for m in d["motor_state"]][:3] == pytest.approx([0.0, 0.01, 0.02])

    def test_imu_at_rest_is_captured(self, prov):
        d = prov.describe_message(lowstate())
        assert d["imu_state"]["rpy"] == pytest.approx([0.001, 0.002, 1.57])
        assert d["imu_state"]["temperature"] == 41

    def test_the_snapshot_is_json_serialisable(self, prov):
        """It has to survive as a file. A dump that only exists in memory is
        not provenance."""
        json.dumps(prov.describe_message(lowstate()))

    def test_an_unexpected_field_is_kept_rather_than_filtered(self, prov):
        """Reflective on purpose: a firmware revision that adds a field must
        land in the dump without anyone editing this repo first."""
        @dataclasses.dataclass
        class Future:
            tick: int = 1
            battery_soc_pct: float = 88.5
        assert prov.describe_message(Future())["battery_soc_pct"] == pytest.approx(88.5)

    def test_a_non_dataclass_message_still_describes(self, prov):
        class Plain:
            def __init__(self):
                self.tick = 4
                self.crc = 9
            def Write(self):  # noqa: N802 - callables must not be walked
                raise AssertionError("describe_message must not call methods")
        d = prov.describe_message(Plain())
        assert d["tick"] == 4 and d["crc"] == 9


class TestTheShotList:
    def test_every_shot_says_why_it_cannot_be_reshot_later(self, prov):
        assert prov.SHOT_LIST, "an empty shot list is not a checklist"
        for shot in prov.SHOT_LIST:
            assert shot.id and shot.what and shot.why
            assert len(shot.why) > 20, f"{shot.id} has no real justification"

    def test_shot_ids_are_unique(self, prov):
        ids = [s.id for s in prov.SHOT_LIST]
        assert len(ids) == len(set(ids))

    def test_ids_are_filename_safe_because_photos_are_named_after_them(self, prov):
        for s in prov.SHOT_LIST:
            assert s.id.replace("-", "").isalnum(), s.id


class TestWritingAndVerifying:
    def test_run_directory_is_stamped_so_a_second_run_cannot_overwrite(self, prov,
                                                                       tmp_path):
        """2026-08-18: five of eight evidence pairs were destroyed by a fixed
        output filename. Provenance is written once per session and is exactly
        the artifact nobody would notice going missing."""
        a = prov.write_provenance(tmp_path, "day1", host={}, network={}, robot={})
        b = prov.write_provenance(tmp_path, "day1", host={}, network={}, robot={})
        assert a != b
        assert (a / "provenance.json").exists() and (b / "provenance.json").exists()

    def test_the_photo_checklist_is_written_for_a_human_to_carry(self, prov, tmp_path):
        run = prov.write_provenance(tmp_path, "day1", host={}, network={}, robot={})
        text = (run / "PHOTOS.md").read_text()
        for s in prov.SHOT_LIST:
            assert s.id in text and s.what in text

    def test_verify_refuses_while_any_shot_is_unshot(self, prov, tmp_path):
        """'It ran' is not done. The dump is worthless without the photographs,
        and the only moment anyone can act on that is while the robot is here."""
        run = prov.write_provenance(tmp_path, "day1", host={}, network={}, robot={})
        r = prov.verify(run)
        assert r["ok"] is False
        assert set(r["missing_photos"]) == {s.id for s in prov.SHOT_LIST}

    def test_verify_passes_once_every_shot_has_a_file(self, prov, tmp_path):
        run = prov.write_provenance(tmp_path, "day1", host={},
                                    network={}, robot={"version": [1, 2]})
        for s in prov.SHOT_LIST:
            (run / "photos" / f"{s.id}-01.jpg").write_bytes(b"\xff\xd8jpeg")
        r = prov.verify(run)
        assert r["ok"] is True, r
        assert r["missing_photos"] == []

    def test_verify_reports_an_empty_robot_snapshot_as_a_failure(self, prov,
                                                                 tmp_path):
        """A silent stream is not a quiet robot. An empty snapshot means the
        subscribe never landed, and it must not read as a completed dump."""
        run = prov.write_provenance(tmp_path, "day1", host={}, network={}, robot={})
        for s in prov.SHOT_LIST:
            (run / "photos" / f"{s.id}-01.jpg").write_bytes(b"x")
        r = prov.verify(run)
        assert r["ok"] is False
        assert "robot" in " ".join(r["problems"]).lower()

    def test_a_zero_byte_photo_does_not_count_as_a_photo(self, prov, tmp_path):
        run = prov.write_provenance(tmp_path, "day1", host={}, network={},
                                    robot={"version": [1, 2]})
        for s in prov.SHOT_LIST:
            (run / "photos" / f"{s.id}-01.jpg").write_bytes(b"")
        assert prov.verify(run)["ok"] is False


class TestHostProvenance:
    def test_this_repos_git_sha_is_recorded(self, prov):
        """Every claim made away from the robot cites code. Which code is a fact with a
        one-week shelf life unless it is written down next to the data."""
        host = prov.collect_host()
        assert host["repos"]["physicalagent"]["sha"]
        assert len(host["repos"]["physicalagent"]["sha"]) >= 7

    def test_dirty_worktrees_are_flagged_not_hidden(self, prov):
        host = prov.collect_host()
        assert "dirty" in host["repos"]["physicalagent"]

    def test_the_interpreter_and_dds_stack_are_recorded(self, prov):
        host = prov.collect_host()
        assert host["python"]["executable"] and host["python"]["version"]
        assert "cyclonedds" in host["packages"]

    def test_host_block_is_json_serialisable(self, prov):
        json.dumps(prov.collect_host())
