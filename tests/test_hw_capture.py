"""Guards for the three LOCKED-tier hardware capture tools.

These tools run next to a physical G1 during a small number of scheduled
hardware sessions, and then not again. Every guard here exists because the corresponding
mistake would either be unrecoverable (lost capture) or dangerous (a
simulator-only setting reaching hardware).

Everything is offline: no DDS, no robot, no simulator.
"""
from __future__ import annotations

import importlib.util
import json
import pathlib
import struct
import sys

import pytest

REPO = pathlib.Path(__file__).resolve().parents[1]
SCRIPTS = REPO / "scripts"
LAUNCHER = SCRIPTS / "sonic_isaac_live.sh"


def _load(name):
    """Import a script by path. These deliberately keep their DDS imports
    INSIDE main(), so importing them must work in the plain test venv -- if it
    ever stops working, the tools have grown a hard dependency that will also
    break on any machine without the SDK."""
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def preflight():
    return _load("hw_preflight")


@pytest.fixture(scope="module")
def recorder():
    return _load("hw_recorder")


@pytest.fixture(scope="module")
def timebase():
    return _load("hw_timebase_probe")


class TestNothingHereCanMoveTheRobot:
    """The capture tools are LOCKED tier: subscribe only. A publisher appearing
    in any of them is a tier violation, not a feature."""

    @pytest.mark.parametrize("name", ["hw_preflight", "hw_recorder",
                                      "hw_timebase_probe", "hw_provenance"])
    def test_no_publisher_is_ever_constructed(self, name):
        text = (SCRIPTS / f"{name}.py").read_text()
        for forbidden in ("ChannelPublisher", "ChannelFactoryPublisher",
                          ".Write(", "LowCmd_()"):
            assert forbidden not in text, (
                f"{name} contains {forbidden!r} -- these tools must not be "
                f"able to command the robot")

    @pytest.mark.parametrize("name", ["hw_preflight", "hw_recorder",
                                      "hw_timebase_probe", "hw_provenance"])
    def test_importable_without_the_unitree_sdk(self, name):
        """DDS imports belong inside main(), so the tools can be tested, and
        so a missing SDK produces a readable refusal rather than an ImportError
        traceback in front of the robot."""
        _load(name)


class TestPreflightRefusesSimOnlySettings:
    """Every one of these is safe in simulation and dangerous on hardware.
    A human cannot be asked to remember five environment variables at 9 a.m.
    next to a robot."""

    def test_timescale_shim_in_ld_preload_is_fatal(self, preflight, monkeypatch):
        monkeypatch.setenv("LD_PRELOAD", "/x/tools/sonic_timescale.so")
        c = preflight.check_sim_only_env(expect_hw=True)
        assert not c.ok and c.fatal
        assert "sonic_timescale" in c.detail

    @pytest.mark.parametrize("var", sorted(
        ["SONIC_TIME_SCALE", "SONIC_FALL_RESET_M", "SONIC_WB_FULL_BODY",
         "SONIC_WB_TARGET_RTF", "SONIC_WB_RENDER_EVERY", "SONIC_SIDECAR_PORT"]))
    def test_each_sim_knob_is_fatal_on_hardware(self, preflight, monkeypatch, var):
        monkeypatch.setenv(var, "1")
        c = preflight.check_sim_only_env(expect_hw=True)
        assert not c.ok and c.fatal, f"{var} did not block a hardware run"

    def test_the_same_knobs_are_allowed_in_sim(self, preflight, monkeypatch):
        monkeypatch.setenv("SONIC_TIME_SCALE", "3.4")
        c = preflight.check_sim_only_env(expect_hw=False)
        assert c.ok, "a sim run must not be blocked by sim settings"

    def test_clean_env_passes(self, preflight, monkeypatch):
        for v in list(preflight.SIM_ONLY_ENV) + ["LD_PRELOAD"]:
            monkeypatch.delenv(v, raising=False)
        assert preflight.check_sim_only_env(expect_hw=True).ok

    def test_cyclonedds_uri_is_refused(self, preflight, monkeypatch):
        monkeypatch.setenv("CYCLONEDDS_URI", "file:///x.xml")
        assert not preflight.check_cyclonedds_uri().ok
        monkeypatch.delenv("CYCLONEDDS_URI")
        assert preflight.check_cyclonedds_uri().ok

    def test_sim_knob_list_covers_what_the_isaac_launcher_exports(self, preflight):
        """Drift guard. If someone adds a new SONIC_* simulator knob to the
        Isaac launcher, preflight must learn about it -- otherwise the new knob
        silently becomes the one that reaches hardware."""
        import re
        text = LAUNCHER.read_text()
        exported = set(re.findall(r"\b(SONIC_[A-Z0-9_]+)=", text))
        exported -= {"SONIC_ISAAC_PYTHON", "SONIC_FRONTEND_PYTHON"}  # paths
        known = set(preflight.SIM_ONLY_ENV)
        missing = exported - known
        assert not missing, (
            f"the Isaac launcher sets {sorted(missing)}, which hw_preflight "
            f"does not know are simulator-only. Add them to SIM_ONLY_ENV.")


class TestPreflightRefusesAnAmbiguousTarget:
    """An agreeing DDS pub/sub pair proves the two ends share a world, not that
    it is the right one (2026-08-07). Guessing means commanding a real robot
    while believing it is a simulation."""

    def test_no_robot_interface_is_fatal(self, preflight, monkeypatch):
        monkeypatch.setattr(preflight, "interfaces", lambda: [("lo", "127.0.0.1")])
        c = preflight.check_target(expect_hw=True)
        assert not c.ok and c.fatal

    def test_two_robot_interfaces_is_fatal(self, preflight, monkeypatch):
        monkeypatch.setattr(preflight, "interfaces", lambda: [
            ("eth0", "192.168.123.10"), ("eth1", "192.168.123.11")])
        c = preflight.check_target(expect_hw=True)
        assert not c.ok and "AMBIGUOUS" in c.detail

    def test_exactly_one_robot_interface_passes(self, preflight, monkeypatch):
        monkeypatch.setattr(preflight, "interfaces", lambda: [
            ("lo", "127.0.0.1"), ("eth0", "192.168.123.10")])
        assert preflight.check_target(expect_hw=True).ok


class TestRecorderRoundTrip:
    """A recording that cannot be read back is not a recording. This is checked
    before the robot is powered, because a recorder discovered to be broken at
    the end of day 1 has cost a day that cannot be bought back."""

    def _fake_lowstate(self, n_motor=29, tick=7):
        class M:
            q, dq, ddq, tau_est = 0.5, -0.25, 1.5, 3.0
        class I:
            quaternion = [1.0, 0.0, 0.0, 0.0]
            gyroscope = [0.1, 0.2, 0.3]
            accelerometer = [0.0, 0.0, 9.81]
            rpy = [0.01, 0.02, 0.03]
        class LS:
            pass
        ls = LS()
        ls.tick, ls.mode_pr, ls.mode_machine = tick, 1, 2
        ls.imu_state, ls.motor_state = I(), [M() for _ in range(n_motor)]
        return ls

    def test_selftest_passes(self, recorder, tmp_path):
        assert recorder.selftest(tmp_path) == 0

    def test_records_round_trip_with_values_intact(self, recorder, tmp_path):
        s = recorder.Stream(tmp_path, "rt/lowstate", "lowstate")
        for _ in range(25):
            s.on_msg(self._fake_lowstate())
        s.close()
        rows = list(recorder.read_stream(s.path, s.meta_path))
        assert len(rows) == 25
        assert rows[0]["tick"] == 7
        assert rows[0]["m0_q"] == pytest.approx(0.5)
        assert rows[0]["m28_tau_est"] == pytest.approx(3.0)
        assert rows[0]["imu_accel_2"] == pytest.approx(9.81, abs=1e-4)

    def test_schema_sidecar_is_self_describing(self, recorder, tmp_path):
        """The corpus must be readable off-robot by code that has never seen this
        repo, so the sidecar carries the full layout, not a version number."""
        s = recorder.Stream(tmp_path, "rt/lowstate", "lowstate")
        s.on_msg(self._fake_lowstate())
        s.close()
        meta = json.loads(s.meta_path.read_text())
        assert meta["struct_format"] and meta["fields"]
        assert meta["record_bytes"] == struct.calcsize(meta["struct_format"])
        assert len(meta["fields"]) == len(
            struct.unpack(meta["struct_format"],
                          b"\0" * meta["record_bytes"]))

    def test_motor_count_comes_from_the_robot_not_a_constant(self, recorder,
                                                             tmp_path):
        """A hard-coded 29 would silently truncate or misalign if the robot
        reports a different number of motor slots."""
        s = recorder.Stream(tmp_path, "rt/lowstate", "lowstate")
        s.on_msg(self._fake_lowstate(n_motor=35))
        s.close()
        assert json.loads(s.meta_path.read_text())["n_motor"] == 35

    def test_a_packing_error_does_not_kill_the_recording(self, recorder,
                                                         tmp_path):
        """The recorder runs unattended for hours. One malformed message must
        cost one message, not the session."""
        s = recorder.Stream(tmp_path, "rt/lowstate", "lowstate")
        s.on_msg(self._fake_lowstate())
        s.on_msg(object())                      # nothing this can pack
        s.on_msg(self._fake_lowstate())
        s.close()
        assert len(list(recorder.read_stream(s.path, s.meta_path))) >= 2

    def test_a_message_arriving_after_close_does_not_raise(self, recorder,
                                                           tmp_path):
        """Observed against the live robot 2026-08-19: DDS reader threads keep
        delivering after close(), and the write escaped the pack-error guard --
        `ValueError: write to closed file` killed the ch_reader thread with a
        traceback at the end of every recording. Harmless to the data that was
        already flushed, but it is an exception in front of the robot at the
        one moment an operator is deciding whether the capture worked."""
        s = recorder.Stream(tmp_path, "rt/lowstate", "lowstate")
        s.on_msg(TestRecorderRoundTrip()._fake_lowstate())
        s.close()
        s.on_msg(TestRecorderRoundTrip()._fake_lowstate())   # late delivery
        assert len(list(recorder.read_stream(s.path, s.meta_path))) == 1

    def test_late_messages_are_counted_as_dropped_not_recorded(self, recorder,
                                                              tmp_path):
        """The count has to stay honest: a message that was not written must
        not be added to n, or summary.json overstates the corpus."""
        s = recorder.Stream(tmp_path, "rt/lowstate", "lowstate")
        s.on_msg(TestRecorderRoundTrip()._fake_lowstate())
        s.close()
        s.on_msg(TestRecorderRoundTrip()._fake_lowstate())
        assert s.n == 1
        assert s.dropped_after_close == 1

    def test_every_record_carries_one_monotonic_clock(self, recorder):
        text = (SCRIPTS / "hw_recorder.py").read_text()
        assert "time.monotonic()" in text
        assert "wall_at_monotonic_zero" in text, (
            "the session header must let any t_mono be converted to wall time")


class TestTimebaseProbeFindsTheThingItExistsToFind:
    """The Isaac backend republished lowstate at a steady 100 Hz from data
    refreshed at ~10 Hz, with an incrementing tick and a valid CRC. That is
    invisible to every consumer. The probe exists to make it visible."""

    def _rows(self, n, dt, tick_step, t0=100.0, tick0=1000, refresh_every=1):
        """refresh_every>1 emits runs of IDENTICAL payloads -- a stream
        published faster than its source is sampled."""
        return [{"t_mono": t0 + i * dt, "tick": tick0 + i * tick_step,
                 "m0_q": float(i // refresh_every)}
                for i in range(n)]

    def test_a_frozen_tick_is_reported_and_does_not_crash(self, timebase):
        out = timebase.analyse_stream(self._rows(50, 0.002, 0), "rt/lowstate")
        assert out["tick"]["frozen"] is True
        assert "TICK NEVER ADVANCED" in out["tick"]["note"]

    def test_a_healthy_stream_is_not_flagged(self, timebase):
        out = timebase.analyse_stream(self._rows(500, 0.002, 1), "rt/lowstate")
        assert out["tick"]["frozen"] is False
        assert out["mean_hz"] == pytest.approx(500, rel=0.02)
        assert out["freshness"]["republished"] is False
        assert out["freshness"]["effective_sample_hz"] == pytest.approx(
            500, rel=0.05)

    def test_a_republisher_is_caught_by_the_payload_not_the_tick(self, timebase):
        """The exact Isaac defect: emitted at 500 Hz with a faithfully
        incrementing tick, but the payload only changes every 10th message, so
        it truly informs at 50 Hz. The tick is useless here -- it increments
        perfectly -- which is why the test is on the payload."""
        rows = self._rows(500, 0.002, 1, refresh_every=10)
        out = timebase.analyse_stream(rows, "rt/lowstate")
        assert out["tick"]["frozen"] is False, "tick looks perfectly healthy"
        assert out["freshness"]["republished"] is True
        assert out["freshness"]["arrival_hz"] == pytest.approx(500, rel=0.02)
        assert out["freshness"]["effective_sample_hz"] == pytest.approx(
            50, rel=0.15)
        assert out["freshness"]["max_identical_run"] == 10

    def test_stalls_are_counted_and_the_worst_is_reported(self, timebase):
        rows = self._rows(100, 0.002, 1)
        for i in range(50, 100):                 # inject one 400 ms hole
            rows[i]["t_mono"] += 0.4
        out = timebase.analyse_stream(rows, "rt/lowstate")
        assert out["stalls_over_3x_median"] >= 1
        assert out["worst_stall_ms"] > 300

    def test_the_tail_is_reported_not_just_the_mean(self, timebase):
        """A 500 ms watchdog does not care about the median gap."""
        out = timebase.analyse_stream(self._rows(200, 0.002, 1), "rt/lowstate")
        for k in ("p50", "p90", "p99", "p99_9", "max"):
            assert k in out["gap_ms"]

    def test_an_empty_stream_says_so_instead_of_reporting_zero(self, timebase):
        out = timebase.analyse_stream([], "rt/odostate")
        assert "NO DATA" in out["verdict"]

    def test_missing_streams_are_named(self, recorder, timebase, tmp_path):
        s = recorder.Stream(tmp_path, "rt/lowstate", "lowstate")
        s.on_msg(TestRecorderRoundTrip()._fake_lowstate())
        s.close()
        empty = recorder.Stream(tmp_path, "rt/odostate", "odostate")
        empty.packer._init_from(None if False else type("O", (), {})())
        empty.write_meta()
        empty.close()
        r = timebase.analyse(tmp_path)
        assert "rt/odostate" in r["missing_streams"]
        assert r["controller_active"] is False


class TestTheProbeCanSeeAnIdleHeartbeat:
    """Measured against the live robot 2026-08-19. rt/lowcmd published at
    995 Hz carrying byte-identical all-zero commands -- every q/dq/tau/kp/kd
    zero across 35 slots. duplicate_fraction was 1.0 and effective_sample_hz
    0.07, both computed correctly and both invisible: the renderer gated the
    freshness line behind `if "tick" in s`, and only lowstate carries a tick.
    The probe then printed `controller writing rt/lowcmd: True`.

    So the one defect the probe exists to catch was present, detected, and
    discarded before it reached the operator -- on 2 of the 3 live streams."""

    def _lowcmd_rows(self, n=400, dt=0.001, kp=0.0, tau=0.0, vary=False):
        rows = []
        for i in range(n):
            r = {"t_mono": 100.0 + i * dt, "crc": 42}
            for j in range(3):
                r[f"m{j}_q"] = 0.01 * i if vary else 0.0
                r[f"m{j}_dq"] = 0.0
                r[f"m{j}_tau"] = tau
                r[f"m{j}_kp"] = kp
                r[f"m{j}_kd"] = 0.0
            rows.append(r)
        return rows

    def _result(self, timebase, streams, **kw):
        r = {"run_dir": "x", "session": {}, "streams": streams,
             "missing_streams": []}
        r.update(kw)
        return r

    def test_freshness_is_reported_for_a_stream_that_has_no_tick(self, timebase):
        s = timebase.analyse_stream(self._lowcmd_rows(), "rt/lowcmd")
        assert "tick" not in s, "fixture must exercise the tickless path"
        assert s["freshness"]["republished"] is True
        text = "\n".join(timebase.render_lines(self._result(timebase, [s])))
        assert "REPUBLISHED" in text, (
            "a stream carrying 100% duplicate payloads must say so on screen, "
            "whether or not it happens to carry a tick field")

    def test_an_all_zero_command_stream_is_not_a_live_controller(self, timebase):
        s = timebase.analyse_stream(self._lowcmd_rows(), "rt/lowcmd")
        assert s["command"]["has_authority"] is False
        r = timebase.analyse_result_flags(self._result(timebase, [s]))
        assert r["controller_active"] is False, (
            "zero kp, zero kd and zero tau is not a controller holding the "
            "robot up, however fast it is published")

    def test_a_command_stream_with_real_gains_is_a_live_controller(self, timebase):
        s = timebase.analyse_stream(self._lowcmd_rows(kp=60.0, vary=True),
                                    "rt/lowcmd")
        assert s["command"]["has_authority"] is True
        r = timebase.analyse_result_flags(self._result(timebase, [s]))
        assert r["controller_active"] is True

    def test_feedforward_torque_alone_counts_as_authority(self, timebase):
        """kp=kd=0 with tau_ff set is a real torque command, not an idle."""
        s = timebase.analyse_stream(self._lowcmd_rows(tau=1.5), "rt/lowcmd")
        assert s["command"]["has_authority"] is True

    def test_the_gains_actually_seen_are_reported_not_just_a_boolean(self,
                                                                    timebase):
        s = timebase.analyse_stream(self._lowcmd_rows(kp=60.0), "rt/lowcmd")
        c = s["command"]
        assert c["max_kp"] == pytest.approx(60.0)
        assert c["max_kd"] == 0.0 and c["max_tau"] == 0.0

    def test_a_sensor_stream_gets_no_command_block(self, timebase):
        rows = [{"t_mono": 100.0 + i * 0.002, "tick": i, "m0_q": float(i)}
                for i in range(50)]
        assert "command" not in timebase.analyse_stream(rows, "rt/lowstate")


class TestTheRecorderSubscribesToTheTopicThatActuallyPublishes:
    """Measured 2026-08-19 by a subscribe-only scan of the live robot:

        LIVE   rt/odommodestate    498.6 Hz
        LIVE   rt/lf/lowstate       20.2 Hz
        silent rt/odostate            0

    Every recording made before this fix carried a zero-byte odometry file and
    a warning telling the operator to check the topic name. The topic name was
    the thing that was wrong. rt/odommodestate carries SportModeState_, which
    is in unitree_go -- NOT unitree_hg, where the capture spec says all four
    message types live.

    This is the stream a sim-to-real trajectory diff is built on: pelvis
    position and velocity, plus foot_force, which is contact ground truth."""

    def _odom(self):
        class Stamp:
            sec, nanosec = 1787119378, 250000000
        class I:
            quaternion = [1.0, 0.0, 0.0, 0.0]
            gyroscope = [0.01, 0.02, 0.03]
            accelerometer = [0.0, 0.1, 9.81]
            rpy = [0.001, 0.002, 1.5]
        class S:
            pass
        s = S()
        s.stamp, s.imu_state = Stamp(), I()
        s.error_code, s.mode, s.gait_type = 0, 3, 1
        s.progress, s.foot_raise_height = 0.5, 0.09
        s.position = [1.25, -0.5, 0.78]
        s.body_height, s.yaw_speed = 0.78, -0.15
        s.velocity = [0.31, -0.02, 0.004]
        s.range_obstacle = [1.0, 2.0, 3.0, 4.0]
        s.foot_force = [180, 175, 0, 0]
        s.foot_position_body = [float(i) for i in range(12)]
        s.foot_speed_body = [float(-i) for i in range(12)]
        return s

    def test_the_stream_list_names_the_live_topic_not_the_silent_one(self,
                                                                    recorder):
        topics = [t for t, _ in recorder.STREAMS]
        assert "rt/odommodestate" in topics, (
            "the recorder must subscribe to the odometry topic this robot "
            "actually publishes")
        assert "rt/odostate" not in topics, (
            "rt/odostate produced zero records in every measured window; "
            "keeping it only manufactures a silent-stream warning every run")

    def test_pelvis_position_and_velocity_round_trip(self, recorder, tmp_path):
        s = recorder.Stream(tmp_path, "rt/odommodestate", "odommodestate")
        s.on_msg(self._odom())
        s.close()
        r = list(recorder.read_stream(s.path, s.meta_path))[0]
        assert [r["pos_0"], r["pos_1"], r["pos_2"]] == pytest.approx(
            [1.25, -0.5, 0.78])
        assert [r["vel_0"], r["vel_1"], r["vel_2"]] == pytest.approx(
            [0.31, -0.02, 0.004], abs=1e-6)
        assert r["yaw_speed"] == pytest.approx(-0.15)
        assert r["body_height"] == pytest.approx(0.78)

    def test_foot_force_is_captured_because_it_is_contact_ground_truth(self,
                                                                      recorder,
                                                                      tmp_path):
        s = recorder.Stream(tmp_path, "rt/odommodestate", "odommodestate")
        s.on_msg(self._odom())
        s.close()
        r = list(recorder.read_stream(s.path, s.meta_path))[0]
        assert [r[f"foot_force_{i}"] for i in range(4)] == pytest.approx(
            [180, 175, 0, 0])

    def test_the_imu_nested_in_the_odometry_message_is_captured(self, recorder,
                                                               tmp_path):
        s = recorder.Stream(tmp_path, "rt/odommodestate", "odommodestate")
        s.on_msg(self._odom())
        s.close()
        r = list(recorder.read_stream(s.path, s.meta_path))[0]
        assert r["imu_rpy_2"] == pytest.approx(1.5)
        assert r["imu_accel_2"] == pytest.approx(9.81, abs=1e-4)

    def test_the_publishers_own_stamp_is_kept_alongside_our_clock(self,
                                                                 recorder,
                                                                 tmp_path):
        """Our t_mono says when WE saw it. The robot's stamp says when it
        happened. The difference between those two is the transport delay, and
        it cannot be recovered later if only one of them was written down."""
        s = recorder.Stream(tmp_path, "rt/odommodestate", "odommodestate")
        s.on_msg(self._odom())
        s.close()
        r = list(recorder.read_stream(s.path, s.meta_path))[0]
        assert r["stamp_sec"] == 1787119378
        assert r["stamp_nanosec"] == 250000000
        assert r["t_mono"] > 0

    def test_mode_and_gait_are_captured_so_the_condition_is_recoverable(self,
                                                                       recorder,
                                                                       tmp_path):
        s = recorder.Stream(tmp_path, "rt/odommodestate", "odommodestate")
        s.on_msg(self._odom())
        s.close()
        r = list(recorder.read_stream(s.path, s.meta_path))[0]
        assert r["mode"] == 3 and r["gait_type"] == 1
        assert r["error_code"] == 0


class TestTheProbeSurvivesARealRecording:
    """The spec makes this probe the mandatory end-of-day replay check. On the
    first real hour-long recording -- 3.6M lowstate records across 5.1 GB --
    it did not finish in three minutes.

    Reading is not the problem: the full stream decodes to dicts in ~16 s.
    analyse() did `rows = list(read_stream(...))`, holding 3.6M dicts of 158
    fields at once. The check that exists to be run on the real corpus could
    only run on the toy one, and every test here used a short list, so nothing
    failed."""

    def _rows(self, n, dt=0.002, tick_step=1):
        return [{"t_mono": 100.0 + i * dt, "tick": 1000 + i * tick_step,
                 "m0_q": float(i), "crc": 7} for i in range(n)]

    def test_it_consumes_the_stream_in_a_single_pass(self, timebase):
        """A generator can only be iterated once. If the implementation walks
        the rows twice -- or calls len() on them -- this fails, which is the
        same property that makes it safe on a multi-gigabyte file."""
        gen = (r for r in self._rows(500))
        out = timebase.analyse_stream(gen, "rt/lowstate")
        assert out["records"] == 500
        assert out["mean_hz"] == pytest.approx(500, rel=0.02)
        assert out["freshness"]["republished"] is False
        assert out["tick"]["frozen"] is False

    def test_an_empty_generator_reports_no_data_rather_than_raising(self, timebase):
        out = timebase.analyse_stream((r for r in []), "rt/odommodestate")
        assert "NO DATA" in out["verdict"]
        assert out["records"] == 0

    def test_a_single_record_generator_is_not_a_crash(self, timebase):
        out = timebase.analyse_stream((r for r in self._rows(1)), "rt/lowstate")
        assert out["records"] == 1
        assert "NO DATA" in out["verdict"]

    def test_a_republisher_is_still_caught_from_a_generator(self, timebase):
        rows = [{"t_mono": 100.0 + i * 0.002, "tick": 1000 + i,
                 "m0_q": float(i // 10), "crc": 7} for i in range(500)]
        out = timebase.analyse_stream((r for r in rows), "rt/lowstate")
        assert out["freshness"]["republished"] is True
        assert out["freshness"]["max_identical_run"] == 10

    def test_analyse_does_not_materialise_the_stream(self, timebase):
        src = (SCRIPTS / "hw_timebase_probe.py").read_text()
        assert "list(read_stream(" not in src, (
            "materialising the whole stream is what made the probe unusable "
            "on the only recording it was built to check")


class TestTheProbeSurvivesARealRecording:
    """Measured 2026-08-19: an hour of capture is 5.1 GB and 3.6M lowstate
    records of ~130 fields each. analyse() did `rows = list(read_stream(...))`,
    materialising every record as a dict, and did not finish in three minutes.

    The probe is specified as the MANDATORY end-of-day replay check, so the
    only recording size it has to handle is precisely the one it could not.
    It passed every test because every test fed it a few hundred rows."""

    def _gen(self, n, dt=0.002, tick_step=1):
        for i in range(n):
            yield {"t_mono": 100.0 + i * dt, "tick": 1000 + i * tick_step,
                   "m0_q": float(i), "crc": 7}

    def test_it_consumes_a_one_shot_iterator(self, timebase):
        """A generator can only be walked once. Anything that indexes rows[0],
        calls len(), or loops a second time fails here -- which is the same
        property that makes it safe on 3.6M records."""
        out = timebase.analyse_stream(self._gen(500), "rt/lowstate")
        assert out["records"] == 500
        assert out["mean_hz"] == pytest.approx(500, rel=0.02)

    def test_tick_analysis_survives_the_single_pass(self, timebase):
        out = timebase.analyse_stream(self._gen(500), "rt/lowstate")
        assert out["tick"]["frozen"] is False
        assert out["tick"]["median_increment"] == 1

    def test_freshness_survives_the_single_pass(self, timebase):
        out = timebase.analyse_stream(self._gen(500), "rt/lowstate")
        assert out["freshness"]["republished"] is False

    def test_a_republisher_is_still_caught_from_an_iterator(self, timebase):
        def rows():
            for i in range(500):
                yield {"t_mono": 100.0 + i * 0.002, "tick": 1000 + i,
                       "m0_q": float(i // 10), "crc": 7}
        out = timebase.analyse_stream(rows(), "rt/lowstate")
        assert out["freshness"]["republished"] is True
        assert out["freshness"]["max_identical_run"] == 10

    def test_an_empty_iterator_says_no_data(self, timebase):
        out = timebase.analyse_stream(iter([]), "rt/odommodestate")
        assert "NO DATA" in out["verdict"]

    def test_a_single_record_is_not_a_crash(self, timebase):
        out = timebase.analyse_stream(self._gen(1), "rt/lowstate")
        assert "NO DATA" in out["verdict"] or out["records"] == 1

    def test_analyse_does_not_materialise_the_stream(self, timebase):
        src = (SCRIPTS / "hw_timebase_probe.py").read_text()
        assert "list(read_stream(" not in src, (
            "materialising a 3.6M-record stream as dicts is what made the "
            "mandatory end-of-day check unusable on a real recording")


class TestATickThatGoesBackwardsMeansTheRobotRebooted:
    """The hour-long capture spans a battery swap. The tick counter reset, so
    the stream shows regressions=1 and a NEGATIVE overall advance, rendered as
    "+-867.3/s". Two problems: the sign glyph is nonsense in a readout someone
    reads under time pressure, and a counter reset is not a minor statistic --
    it means the file spans two boots, so anything treating the tick as
    monotonic across it is silently wrong."""

    def _rows(self, n, dt=0.002, reset_at=None):
        tick = 1000
        for i in range(n):
            if reset_at is not None and i == reset_at:
                tick = 0
            yield {"t_mono": 100.0 + i * dt, "tick": tick, "m0_q": float(i)}
            tick += 1

    def _result(self, s):
        return {"run_dir": "x", "session": {}, "streams": [s],
                "missing_streams": [], "controller_active": False}

    def test_a_reset_is_counted_as_a_regression(self, timebase):
        out = timebase.analyse_stream(self._rows(400, reset_at=200),
                                      "rt/lowstate")
        assert out["tick"]["regressions"] == 1

    def test_the_report_never_prints_a_plus_minus(self, timebase):
        out = timebase.analyse_stream(self._rows(400, reset_at=200),
                                      "rt/lowstate")
        text = "\n".join(timebase.render_lines(self._result(out)))
        assert "+-" not in text

    def test_a_reset_is_called_out_not_just_counted(self, timebase):
        out = timebase.analyse_stream(self._rows(400, reset_at=200),
                                      "rt/lowstate")
        text = "\n".join(timebase.render_lines(self._result(out)))
        assert "RESET" in text.upper(), (
            "a tick that goes backwards means the counter restarted -- the "
            "recording spans a reboot and is not one continuous timebase")

    def test_a_clean_stream_says_nothing_about_resets(self, timebase):
        out = timebase.analyse_stream(self._rows(400), "rt/lowstate")
        text = "\n".join(timebase.render_lines(self._result(out)))
        assert "RESET" not in text.upper()


class TestThermalIsRecordedBecauseAWarningMustBeAdjudicable:
    """2026-08-19: SONIC warned "right ankle roll overheating" and the operator
    stopped the session. Nobody could tell whether it was real, because the
    recorder logs q/dq/ddq/tau_est per motor and drops temperature, voltage and
    the motor fault word -- and SONIC's own motor_temperature.csv needs
    --enable-csv-logs, which deploy.sh never passes.

    Torque is not a substitute. SONIC runs the ankle in PR (series) mode, so
    tau_est at the roll index is a JOINT-space estimate, while the temperature
    sensor is on a PHYSICAL motor in a coupled linkage that shares the pitch
    load. The two are in different frames, so the question is unanswerable from
    torque no matter how carefully it is analysed.

    Temperature moves on a thermal time constant, so it is recorded as its own
    decimated stream rather than at 1052 Hz -- the answer costs ~2.5 MB/hour."""

    def _ls(self, n_motor=29, temp=(40, 39), vol=49.0, state=0):
        class M:
            q = dq = ddq = tau_est = 0.0
        class I:
            quaternion = [1.0, 0.0, 0.0, 0.0]
            gyroscope = accelerometer = rpy = [0.0, 0.0, 0.0]
        class LS:
            pass
        ls = LS()
        ls.tick, ls.mode_pr, ls.mode_machine = 5, 0, 5
        ls.imu_state = I()
        ls.motor_state = []
        for i in range(n_motor):
            m = M()
            m.temperature = [temp[0] + i, temp[1] + i]
            m.vol, m.motorstate = vol, state
            ls.motor_state.append(m)
        return ls

    def test_the_thermal_stream_is_recorded(self, recorder):
        assert any(k == "thermal" for _, k in recorder.STREAMS), (
            "a thermal warning that cannot be checked against a log is an "
            "unanswerable question about a robot nobody has any more")

    def test_both_temperature_sensors_are_kept(self, recorder, tmp_path):
        """SONIC warns on max(winding, driver). Keeping one loses the trigger."""
        s = recorder.Stream(tmp_path, "rt/lowstate", "thermal")
        s.on_msg(self._ls())
        s.close()
        r = list(recorder.read_stream(s.path, s.meta_path))[0]
        assert r["m11_temp_winding"] == 51   # 40 + 11
        assert r["m11_temp_driver"] == 50    # 39 + 11

    def test_the_motor_fault_word_and_voltage_are_kept(self, recorder, tmp_path):
        s = recorder.Stream(tmp_path, "rt/lowstate", "thermal")
        s.on_msg(self._ls(state=7, vol=48.5))
        s.close()
        r = list(recorder.read_stream(s.path, s.meta_path))[0]
        assert r["m11_motorstate"] == 7
        assert r["m11_vol"] == pytest.approx(48.5)

    def test_it_is_decimated_rather_than_logged_at_full_rate(self, recorder,
                                                            tmp_path):
        """A 1 kHz temperature log is 1000x oversampled for a thermal question
        and would add ~1.2 GB/hour to a corpus with no second copy."""
        s = recorder.Stream(tmp_path, "rt/lowstate", "thermal")
        for _ in range(500):
            s.on_msg(self._ls())
        s.close()
        n = len(list(recorder.read_stream(s.path, s.meta_path)))
        assert 1 <= n <= 5, f"expected a decimated stream, got {n} records"

    def test_it_does_not_overwrite_the_full_rate_lowstate_file(self, recorder,
                                                              tmp_path):
        a = recorder.Stream(tmp_path, "rt/lowstate", "lowstate")
        b = recorder.Stream(tmp_path, "rt/lowstate", "thermal")
        assert a.path != b.path and a.meta_path != b.meta_path
        a.close(); b.close()

    def test_motor_count_still_comes_from_the_robot(self, recorder, tmp_path):
        s = recorder.Stream(tmp_path, "rt/lowstate", "thermal")
        s.on_msg(self._ls(n_motor=35))
        s.close()
        import json
        assert json.loads(s.meta_path.read_text())["n_motor"] == 35


class TestTheRecorderRefusesToRecordOffTheRobotsNetwork:
    """2026-08-19: a recorder was started while enp132s0 was DOWN. CycloneDDS
    picked the only interface that was up -- Wi-Fi -- and bound its unicast
    socket to the Wi-Fi address instead. It ran happily for minutes writing six zero-byte
    files, and would have kept writing them after the robot came back, because
    the participant's interface is chosen once at startup and never revisited.

    A capture tool whose failure mode is a plausible-looking empty file is
    worse than no capture tool. It has to refuse."""

    NET = "192.168.123."

    def test_it_finds_the_interface_on_the_robots_subnet(self, recorder):
        got = recorder.robot_net_iface([("lo", "127.0.0.1"),
                                        ("enp132s0", "192.168.123.222"),
                                        ("wlp131s0f0", "10.0.0.57")])
        assert got == "enp132s0"

    def test_it_returns_none_when_the_link_is_down(self, recorder):
        """The exact state observed: enp132s0 gone, only Wi-Fi and loopback."""
        got = recorder.robot_net_iface([("lo", "127.0.0.1"),
                                        ("wlp131s0f0", "10.0.0.57"),
                                        ("docker0", "172.17.0.1")])
        assert got is None

    def test_wifi_alone_is_never_mistaken_for_the_robot(self, recorder):
        assert recorder.robot_net_iface([("wlp131s0f0", "192.168.1.50")]) is None

    def test_the_refusal_names_what_it_looked_for_and_what_it_found(self,
                                                                   recorder):
        msg = recorder.no_robot_net_message([("lo", "127.0.0.1"),
                                             ("wlp131s0f0", "10.0.0.57")])
        assert self.NET in msg
        assert "wlp131s0f0=10.0.0.57" in msg
        assert "zero-byte" in msg or "empty" in msg

    def test_an_explicit_iface_still_has_to_exist(self, recorder):
        """--iface enp132s0 must not be a way to talk past a dead link."""
        assert recorder.robot_net_iface([], want="enp132s0") is None
        assert recorder.robot_net_iface(
            [("enp132s0", "192.168.123.222")], want="enp132s0") == "enp132s0"


class TestTheSummaryKeepsCollidingStreamsApart:
    """The FILE names of two streams sharing a topic were already made
    distinct ("or the second silently truncates the first") -- and then
    summary.json keyed rows by topic and recreated the identical collision
    one level up. In the 2026-08-19 runs with a thermal stream, the
    'rt/lowstate' summary row silently described the 2 Hz thermal file while
    the 900 MB full-rate stream had no row at all."""

    def test_two_streams_on_one_topic_get_distinct_rows(
            self, recorder, tmp_path):
        full = recorder.Stream(tmp_path, "rt/lowstate", "lowstate")
        thermal = recorder.Stream(tmp_path, "rt/lowstate", "thermal")
        full.close()
        thermal.close()
        s = recorder.build_summary([full, thermal])
        assert len(s) == 2, f"one row swallowed the other: {list(s)}"

    def test_each_row_still_names_its_topic_and_kind(
            self, recorder, tmp_path):
        full = recorder.Stream(tmp_path, "rt/lowstate", "lowstate")
        thermal = recorder.Stream(tmp_path, "rt/lowstate", "thermal")
        full.close()
        thermal.close()
        s = recorder.build_summary([full, thermal])
        rows = list(s.values())
        assert {r["topic"] for r in rows} == {"rt/lowstate"}
        assert {r["kind"] for r in rows} == {"lowstate", "thermal"}

    def test_rows_are_keyed_by_the_file_they_describe(
            self, recorder, tmp_path):
        full = recorder.Stream(tmp_path, "rt/lowstate", "lowstate")
        thermal = recorder.Stream(tmp_path, "rt/lowstate", "thermal")
        full.close()
        thermal.close()
        s = recorder.build_summary([full, thermal])
        assert "rt_lowstate" in s
        assert "rt_lowstate__thermal" in s
