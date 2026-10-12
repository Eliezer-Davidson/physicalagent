"""Guards for sim_match_compare.py -- the sim-vs-hardware error report.

This tool is what turns "the simulator looks right" into a number. It joins a
verb log (mark events) against two hw_recorder captures -- one from the real
G1, one from the MuJoCo G1 -- and reports per-segment, per-joint error.

Everything here is offline: no DDS, no robot, no simulator. Fixtures are
written with hw_recorder's own Packer so the on-disk format is the recorder's,
never a parallel invention that could drift.
"""
from __future__ import annotations

import importlib.util
import json
import math
import pathlib
import struct
import sys

import pytest

REPO = pathlib.Path(__file__).resolve().parents[1]
SCRIPTS = REPO / "scripts"


def _load(name):
    """Import a script by path -- same pattern as test_hw_capture.py. The tool
    keeps any heavy/optional imports out of module scope, so importing it must
    work in the plain test venv."""
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def smc():
    return _load("sim_match_compare")


@pytest.fixture(scope="module")
def recorder():
    return _load("hw_recorder")


# ---------------------------------------------------------------- fixtures

def write_verb_log(path, marks, sha="a" * 64, extra_lines=(), t0=1000.0):
    """A minimal battery-runner verb log: meta line, then mark events.
    `marks` is [(label, t_wall)]."""
    lines = [{"kind": "meta", "t_wall": t0, "t_mono": 5.0,
              "script": "battery.txt", "sha256": sha, "real": True}]
    for label, t in marks:
        lines.append({"kind": "mark", "t_wall": t, "t_mono": 5.0 + (t - t0),
                      "label": label})
    lines = [json.dumps(x) for x in lines] + [json.dumps(x) for x in extra_lines]
    pathlib.Path(path).write_text("\n".join(lines) + "\n")
    return pathlib.Path(path)


# ---------------------------------------------------------------- verb log

class TestVerbLogParsing:

    def test_segments_come_from_consecutive_marks(self, smc, tmp_path):
        """Three marks = two segments; the final mark only closes the last
        segment, it opens nothing."""
        log = write_verb_log(tmp_path / "v.jsonl",
                             [("stand", 100.0), ("wave", 108.0),
                              ("end", 120.0)])
        parsed = smc.parse_verb_log(log)
        segs = parsed["segments"]
        assert [(s["label"], s["t_start"], s["t_end"]) for s in segs] == [
            ("stand", 100.0, 108.0), ("wave", 108.0, 120.0)]

    def test_unknown_kinds_are_ignored(self, smc, tmp_path):
        """Forward compatibility: a future battery runner may log new kinds,
        and this tool must not care."""
        log = write_verb_log(
            tmp_path / "v.jsonl",
            [("a", 10.0), ("b", 20.0)],
            extra_lines=[{"kind": "hologram", "t_wall": 15.0, "wat": 1},
                         {"kind": "verb", "t_wall": 12.0, "t_mono": 1.0,
                          "line": "wave", "verb": "wave", "args": {},
                          "ok": True, "error": None, "duration_s": 2.0}])
        parsed = smc.parse_verb_log(log)
        assert len(parsed["segments"]) == 1
        assert parsed["segments"][0]["label"] == "a"

    def test_meta_is_captured(self, smc, tmp_path):
        log = write_verb_log(tmp_path / "v.jsonl", [("a", 1.0), ("b", 2.0)],
                             sha="f" * 64)
        assert smc.parse_verb_log(log)["meta"]["sha256"] == "f" * 64

    def test_sha_mismatch_produces_a_warning(self, smc):
        """Different battery scripts on the two sides is worth a loud warning
        -- the comparison is still computable, so it must not be fatal."""
        w = smc.meta_warnings({"sha256": "a" * 64}, {"sha256": "b" * 64})
        assert any("sha" in x.lower() for x in w)

    def test_matching_sha_produces_no_warning(self, smc):
        assert smc.meta_warnings({"sha256": "c" * 64},
                                 {"sha256": "c" * 64}) == []


class TestSegmentMatching:

    def _segs(self, labels):
        return [{"label": l, "t_start": 10.0 * i, "t_end": 10.0 * i + 10.0}
                for i, l in enumerate(labels)]

    def test_identical_sequences_match_fully(self, smc):
        pairs, only_a, only_b = smc.match_segments(
            self._segs(["stand", "wave", "sit"]),
            self._segs(["stand", "wave", "sit"]))
        assert [p[0]["label"] for p in pairs] == ["stand", "wave", "sit"]
        assert only_a == [] and only_b == []

    def test_one_sided_segments_are_named_not_processed(self, smc):
        """The hw run aborted mid-battery: sim has segments hw never reached.
        Those must be reported by NAME, and the common ones still compared."""
        pairs, only_a, only_b = smc.match_segments(
            self._segs(["stand", "wave"]),
            self._segs(["stand", "wave", "sit", "bow"]))
        assert [p[0]["label"] for p in pairs] == ["stand", "wave"]
        assert only_a == []
        assert only_b == ["sit", "bow"]

    def test_lcs_survives_an_inserted_segment(self, smc):
        """Longest common subsequence, not zip: an extra segment in the middle
        of one side must not misalign everything after it."""
        pairs, only_a, only_b = smc.match_segments(
            self._segs(["stand", "recover", "wave"]),
            self._segs(["stand", "wave"]))
        assert [p[0]["label"] for p in pairs] == ["stand", "wave"]
        assert only_a == ["recover"]

    def test_zero_common_segments_is_empty(self, smc):
        pairs, only_a, only_b = smc.match_segments(
            self._segs(["a"]), self._segs(["b"]))
        assert pairs == []


# ------------------------------------------------------------ stream slicing

def _rec(t_wall, qs, dqs=None, taus=None):
    """One synthetic lowstate record dict, shaped like read_stream() yields."""
    r = {"t_mono": t_wall - 1000.0, "t_wall": t_wall, "tick": 0}
    for i, q in enumerate(qs):
        r[f"m{i}_q"] = q
        r[f"m{i}_dq"] = 0.0 if dqs is None else dqs[i]
        r[f"m{i}_tau_est"] = 0.0 if taus is None else taus[i]
    return r


class TestStreamSlicing:
    """Production scale is ~3.6M records (~1 GB) per stream: the slicer gets
    ONE forward pass over a generator and may retain only in-segment,
    downsampled data. hw_timebase_probe already died once from list()-ing a
    stream; that bug class is tested away here."""

    SEGS = [{"label": "a", "t_start": 10.0, "t_end": 20.0},
            {"label": "b", "t_start": 20.0, "t_end": 30.0}]

    def test_records_route_to_the_right_segments(self, smc):
        recs = [_rec(t, [float(t)]) for t in
                [5.0, 10.0, 12.0, 19.9, 20.0, 25.0, 29.9, 31.0]]
        segs, n_motor = smc.slice_stream(iter(recs), self.SEGS, grid_hz=100.0)
        assert n_motor == 1
        # [10, 20): the 20.0 record belongs to segment b, not a.
        assert list(segs[0]["t"]) == pytest.approx([0.0, 2.0, 9.9])
        assert list(segs[0]["q"][0]) == pytest.approx([10.0, 12.0, 19.9])
        # [20, 30): 31.0 is after the closing mark and is ignored.
        assert list(segs[1]["t"]) == pytest.approx([0.0, 5.0, 9.9])
        assert list(segs[1]["q"][0]) == pytest.approx([20.0, 25.0, 29.9])

    def test_single_pass_over_a_pure_generator(self, smc):
        """read_stream() is a generator: single-use, no len(), and a second
        iteration silently yields nothing. The slicer must therefore work in
        exactly one pass -- this generator counts every pull to prove it."""
        pulls = {"n": 0}

        def gen():
            for t in range(5, 35):
                pulls["n"] += 1
                yield _rec(float(t), [1.0])

        segs, n_motor = smc.slice_stream(gen(), self.SEGS, grid_hz=100.0)
        assert sum(len(s["t"]) for s in segs) == 20   # t=10..29
        # <= is the contract: early-stop after the last segment is allowed,
        # a second pass (2x pulls) is not.
        assert pulls["n"] <= 30

    def test_only_in_segment_data_is_retained(self, smc):
        """Bounded memory: 50k records stream through, the segments cover 20
        of them, and the slicer may hold at most those 20. Holding the full
        stream is exactly the failure that killed hw_timebase_probe at 3.6M
        records."""
        def gen():
            for k in range(50_000):
                yield _rec(1000.0 + k, [0.5])
        segs = [{"label": "tiny", "t_start": 2000.0, "t_end": 2020.0}]
        out, _ = smc.slice_stream(gen(), segs, grid_hz=100.0)
        assert 0 < len(out[0]["t"]) <= 20

    def test_in_segment_ingest_is_rate_capped(self, smc):
        """A segment is allowed to be minutes long at 1052 Hz; the retained
        arrays must be downsampled relative to the grid, not raw-rate. 10 kHz
        in, 100 Hz grid -> the keeper must hold far fewer than all of it."""
        def gen():
            t = 10.0
            while t < 20.0:
                yield _rec(t, [t])
                t += 0.0001                    # 10 kHz
        out, _ = smc.slice_stream(gen(), self.SEGS[:1], grid_hz=100.0)
        kept = len(out[0]["t"])
        assert kept < 20_000, "raw-rate retention = unbounded memory"
        assert kept >= 1000, "too sparse to interpolate a 100 Hz grid from"


# ---------------------------------------------------------- metrics/resample

def _seg_data(smc, f, dur, hz, t0=100.0, n_motor=1, grid_hz=100.0, dq=None):
    """One segment's sliced data for signal q(t)=f(t_rel), built through the
    already-tested slicer so the metrics see exactly what production sees."""
    segs = [{"label": "s", "t_start": t0, "t_end": t0 + dur}]

    def gen():
        k = 0
        while t0 + k / hz < t0 + dur:
            t = t0 + k / hz
            qs = [f(t - t0)] * n_motor
            dqs = None if dq is None else [dq(t - t0)] * n_motor
            yield _rec(t, qs, dqs)
            k += 1

    out, _ = smc.slice_stream(gen(), segs, grid_hz=grid_hz)
    return out[0]


class TestSegmentMetrics:

    def test_identical_traces_rmse_zero_corr_one(self, smc):
        f = lambda t: math.sin(2 * math.pi * t)
        a = _seg_data(smc, f, dur=2.0, hz=500.0)
        b = _seg_data(smc, f, dur=2.0, hz=500.0)
        m = smc.compare_segment(a, b, 2.0, 2.0, grid_hz=100.0,
                                normalize=False, n_motor=1, joint_names=("m0",))
        j = m["joints"]["m0"]
        assert j["rmse_q"] == pytest.approx(0.0, abs=1e-12)
        assert j["corr_q"] == pytest.approx(1.0, abs=1e-9)
        assert j["max_abs_delta_q"] == pytest.approx(0.0, abs=1e-12)

    def test_constant_offset_shows_up_as_exactly_that_rmse(self, smc):
        f = lambda t: math.sin(2 * math.pi * t)
        a = _seg_data(smc, f, dur=2.0, hz=500.0)
        b = _seg_data(smc, lambda t: f(t) + 0.35, dur=2.0, hz=500.0)
        m = smc.compare_segment(a, b, 2.0, 2.0, grid_hz=100.0,
                                normalize=False, n_motor=1, joint_names=("m0",))
        assert m["joints"]["m0"]["rmse_q"] == pytest.approx(0.35, abs=1e-9)

    def test_different_sample_rates_compare_clean(self, smc):
        """1052 Hz hardware vs ~500 Hz sim is the NORMAL case. The same
        underlying signal at 1000 vs 500 Hz must resample to near-zero RMSE."""
        f = lambda t: math.sin(2 * math.pi * 0.5 * t)
        a = _seg_data(smc, f, dur=4.0, hz=1000.0)
        b = _seg_data(smc, f, dur=4.0, hz=500.0)
        m = smc.compare_segment(a, b, 4.0, 4.0, grid_hz=100.0,
                                normalize=False, n_motor=1, joint_names=("m0",))
        assert m["joints"]["m0"]["rmse_q"] < 1e-3

    def test_normalize_time_undoes_a_stretched_run(self, smc):
        """The sim ran the same verb 1.5x slower. In wall time the sines beat
        against each other (large RMSE); mapped onto u in [0,1] they must
        coincide. This is what --normalize-time is FOR."""
        shape = lambda u: math.sin(2 * math.pi * 2 * u)
        a = _seg_data(smc, lambda t: shape(t / 4.0), dur=4.0, hz=500.0)
        b = _seg_data(smc, lambda t: shape(t / 6.0), dur=6.0, hz=500.0)
        wall = smc.compare_segment(a, b, 4.0, 6.0, grid_hz=100.0,
                                   normalize=False, n_motor=1, joint_names=("m0",))
        norm = smc.compare_segment(a, b, 4.0, 6.0, grid_hz=100.0,
                                   normalize=True, n_motor=1, joint_names=("m0",))
        assert wall["joints"]["m0"]["rmse_q"] > 0.1
        assert norm["joints"]["m0"]["rmse_q"] < 1e-2

    def test_duration_mismatch_over_ten_percent_warns(self, smc):
        f = lambda t: t
        a = _seg_data(smc, f, dur=4.0, hz=500.0)
        b = _seg_data(smc, f, dur=6.0, hz=500.0)
        m = smc.compare_segment(a, b, 4.0, 6.0, grid_hz=100.0,
                                normalize=False, n_motor=1, joint_names=("m0",))
        assert any("time-warp" in w for w in m["warnings"])
        assert m["duration_ratio"] == pytest.approx(6.0 / 4.0)

    def test_idle_joint_corr_is_none_not_a_crash(self, smc):
        """A joint that never moves has zero variance; Pearson would divide by
        zero. corr=None is the honest answer -- there is nothing to correlate."""
        a = _seg_data(smc, lambda t: 0.7, dur=2.0, hz=500.0)
        b = _seg_data(smc, lambda t: 0.7, dur=2.0, hz=500.0)
        m = smc.compare_segment(a, b, 2.0, 2.0, grid_hz=100.0,
                                normalize=False, n_motor=1, joint_names=("m0",))
        j = m["joints"]["m0"]
        assert j["corr_q"] is None
        assert j["rmse_q"] == pytest.approx(0.0, abs=1e-12)

    def test_worst_joint_and_mean_are_reported(self, smc):
        segs = [{"label": "s", "t_start": 0.0, "t_end": 2.0}]

        def gen(offsets):
            for k in range(1000):
                t = k / 500.0
                yield _rec(t, [math.sin(t) + o for o in offsets])

        a, _ = smc.slice_stream(gen([0.0, 0.0]), segs, grid_hz=100.0)
        b, _ = smc.slice_stream(gen([0.0, 0.5]), segs, grid_hz=100.0)
        m = smc.compare_segment(a[0], b[0], 2.0, 2.0, grid_hz=100.0,
                                normalize=False, n_motor=2,
                                joint_names=("j0", "j1"))
        assert m["worst_joint"]["name"] == "j1"
        assert m["worst_joint"]["rmse_q"] == pytest.approx(0.5, abs=1e-6)
        assert m["mean_rmse_q"] == pytest.approx(0.25, abs=1e-6)

    def test_g1_motor_names_are_the_29_body_joints(self, smc):
        """Motor index -> joint name follows the LowState_.motor_state order
        pinned in physicalagent/robots/sonic_telemetry.py."""
        assert smc.joint_name(0) == "left_hip_pitch"
        assert smc.joint_name(11) == "right_ankle_roll"
        assert smc.joint_name(28) == "right_wrist_yaw"
        assert smc.joint_name(29) == "m29"      # beyond the table: honest m{i}


# ------------------------------------------------------------- end to end

def _lowstate_msg(qs, dqs=None):
    """Vendor-shaped LowState_ stand-in, same style as hw_recorder.selftest."""
    class M:
        pass

    class I:
        quaternion = [1.0, 0.0, 0.0, 0.0]
        gyroscope = [0.0, 0.0, 0.0]
        accelerometer = [0.0, 0.0, 9.81]
        rpy = [0.0, 0.0, 0.0]

    class LS:
        pass

    motors = []
    for i, q in enumerate(qs):
        m = M()
        m.q, m.dq, m.ddq, m.tau_est = q, (dqs[i] if dqs else 0.0), 0.0, 0.0
        motors.append(m)
    ls = LS()
    ls.tick, ls.mode_pr, ls.mode_machine = 0, 1, 2
    ls.imu_state, ls.motor_state = I(), motors
    return ls


def _thermal_msg(temps):
    """temps: [(winding, driver)] per motor."""
    class M:
        pass

    class LS:
        pass

    motors = []
    for w, d in temps:
        m = M()
        m.temperature, m.vol, m.motorstate = [w, d], 48.0, 0
        motors.append(m)
    ls = LS()
    ls.tick, ls.motor_state = 0, motors
    return ls


def _write_capture(recorder, dirpath, kind, samples):
    """Write .bin + .schema.json with hw_recorder's OWN Packer and the same
    sidecar keys Stream.write_meta writes -- the fixture format is the
    recorder's format, never a reinvention that could drift from it.
    samples: [(t_wall, msg)]."""
    dirpath.mkdir(parents=True, exist_ok=True)
    p = recorder.Packer(kind)
    name = "rt_lowstate" + ("__thermal" if kind == "thermal" else "")
    binp = dirpath / f"{name}.bin"
    with binp.open("wb") as fh:
        for t_wall, msg in samples:
            fh.write(p.pack(msg, t_wall - 1000.0, t_wall))
    (dirpath / f"{name}.schema.json").write_text(json.dumps({
        "topic": "rt/lowstate", "kind": kind,
        "struct_format": p.fmt, "record_bytes": struct.calcsize(p.fmt),
        "fields": p.fields, "n_motor": p.n_motor}))
    return binp


def _build_pair(tmp_path, recorder, sha_a="s" * 64, sha_b="s" * 64,
                b_extra_q=0.0, thermal_a=None, thermal_b=None,
                hz_a=250.0, hz_b=125.0, n_motor=3):
    """Two capture dirs + verb logs for the same 2-segment battery.

    Side b's wall clock is offset by +5000 s on purpose: the runs happen at
    different times of day, and the tool must only ever use each side's OWN
    marks against its OWN capture -- never absolute wall time across sides.
    Motor 2 on side b carries a constant +b_extra_q offset, the known error
    the report must find.
    """
    def q_at(phase, side_b):
        qs = [math.sin(2 * math.pi * 0.5 * phase) + 0.1 * j
              for j in range(n_motor)]
        if side_b:
            qs[2] += b_extra_q
        return qs

    def build(root, t0, hz, side_b, sha, thermal):
        cap = root / ("cap_b" if side_b else "cap_a")
        samples = []
        k = 0
        while t0 - 1.0 + k / hz < t0 + 9.0:      # records spill past the marks
            t = t0 - 1.0 + k / hz
            samples.append((t, _lowstate_msg(q_at(t - t0, side_b))))
            k += 1
        _write_capture(recorder, cap, "lowstate", samples)
        if thermal is not None:
            _write_capture(recorder, cap, "thermal",
                           [(t0 + s, _thermal_msg(temps))
                            for s, temps in thermal])
        log = write_verb_log(root / ("b.jsonl" if side_b else "a.jsonl"),
                             [("wave", t0), ("sit", t0 + 4.0),
                              ("end", t0 + 8.0)], sha=sha, t0=t0 - 1.0)
        return cap, log

    cap_a, log_a = build(tmp_path, 100.0, hz_a, False, sha_a, thermal_a)
    cap_b, log_b = build(tmp_path, 5100.0, hz_b, True, sha_b, thermal_b)
    return cap_a, log_a, cap_b, log_b


def _argv(cap_a, log_a, cap_b, log_b, out, extra=()):
    return ["--a", str(cap_a), "--a-log", str(log_a), "--a-label", "hw",
            "--b", str(cap_b), "--b-log", str(log_b), "--b-label", "sim",
            "--out", str(out), "--grid-hz", "50", "--top-joints", "2",
            *extra]


class TestEndToEnd:

    def test_happy_path_report_json(self, smc, recorder, tmp_path):
        cap_a, log_a, cap_b, log_b = _build_pair(tmp_path, recorder,
                                                 b_extra_q=0.2)
        out = tmp_path / "report"
        assert smc.main(_argv(cap_a, log_a, cap_b, log_b, out)) == 0
        rep = json.loads((out / "report.json").read_text())
        assert set(rep["segments"]) == {"wave", "sit"}
        seg = rep["segments"]["wave"]
        # motor 2 = left_hip_yaw in the pinned G1 order; its +0.2 offset is
        # the number this whole tool exists to surface.
        j = seg["joints"]["left_hip_yaw"]
        assert j["rmse_q"] == pytest.approx(0.2, abs=5e-3)
        assert j["corr_q"] == pytest.approx(1.0, abs=1e-3)
        assert "rmse_dq" in j and "max_abs_delta_q" in j
        assert seg["worst_joint"]["name"] == "left_hip_yaw"
        assert seg["duration_hw_s"] == pytest.approx(4.0)
        assert seg["duration_sim_s"] == pytest.approx(4.0)
        # the clean joints really are clean despite 250 vs 125 Hz sampling
        assert seg["joints"]["left_hip_pitch"]["rmse_q"] < 1e-2

    def test_happy_path_report_md_and_svg(self, smc, recorder, tmp_path):
        cap_a, log_a, cap_b, log_b = _build_pair(tmp_path, recorder,
                                                 b_extra_q=0.2)
        out = tmp_path / "report"
        assert smc.main(_argv(cap_a, log_a, cap_b, log_b, out)) == 0
        md = (out / "report.md").read_text()
        assert "wave" in md and "sit" in md
        assert "left_hip_yaw" in md
        assert "## Warnings" in md
        svgs = sorted(x.name for x in out.glob("seg_*.svg"))
        # --top-joints 2 and 2 segments -> 4 overlay plots
        assert len(svgs) == 4
        assert "seg_wave_left_hip_yaw.svg" in svgs
        svg = (out / "seg_wave_left_hip_yaw.svg").read_text()
        assert svg.count("<polyline") == 2, "one trace per side, exactly"
        assert "RMSE" in svg and "left_hip_yaw" in svg and "wave" in svg
        assert "rad" in svg and "t_rel" in svg      # axis labels carry units
        assert "hw" in svg and "sim" in svg          # legend names the sides

    def test_sha_mismatch_is_loud_but_not_fatal(self, smc, recorder,
                                                tmp_path, capsys):
        cap_a, log_a, cap_b, log_b = _build_pair(tmp_path, recorder,
                                                 sha_b="d" * 64)
        out = tmp_path / "report"
        assert smc.main(_argv(cap_a, log_a, cap_b, log_b, out)) == 0
        err = capsys.readouterr().err
        assert "sha256" in err and "DIFFERENT" in err
        md = (out / "report.md").read_text()
        assert "sha256" in md.split("## Warnings")[1]


class TestUnusableInputs:

    def test_zero_common_segments_exits_2(self, smc, recorder, tmp_path,
                                          capsys):
        cap_a, log_a, cap_b, log_b = _build_pair(tmp_path, recorder)
        # rewrite side b's log with disjoint labels
        write_verb_log(log_b, [("jump", 5100.0), ("end", 5108.0)])
        rc = smc.main(_argv(cap_a, log_a, cap_b, log_b, tmp_path / "r"))
        assert rc == 2
        assert "common" in capsys.readouterr().err.lower()

    def test_missing_lowstate_exits_2_naming_the_file(self, smc, recorder,
                                                      tmp_path, capsys):
        cap_a, log_a, cap_b, log_b = _build_pair(tmp_path, recorder)
        (cap_b / "rt_lowstate.bin").unlink()
        rc = smc.main(_argv(cap_a, log_a, cap_b, log_b, tmp_path / "r"))
        assert rc == 2
        assert "rt_lowstate.bin" in capsys.readouterr().err

    def test_zero_byte_lowstate_is_missing_with_the_named_warning(
            self, smc, recorder, tmp_path, capsys):
        """This repo has REAL zero-byte captures: the 2026-08-19 dead-link bug
        made DDS bind to the wrong interface and write six 0-byte files that
        look like a capture. The tool must name the file and say what a
        zero-byte stream means, not crash on an empty read."""
        cap_a, log_a, cap_b, log_b = _build_pair(tmp_path, recorder)
        (cap_b / "rt_lowstate.bin").write_bytes(b"")
        rc = smc.main(_argv(cap_a, log_a, cap_b, log_b, tmp_path / "r"))
        assert rc == 2
        err = capsys.readouterr().err
        assert "rt_lowstate.bin" in err and "zero-byte" in err


class TestThermal:

    THERM_A = [(s, [(40, 39), (70, 55), (41, 40)]) for s in (0.5, 3.0, 6.0)]
    THERM_B = [(s, [(45, 44), (42, 41), (43, 42)]) for s in (0.5, 3.0, 6.0)]

    def test_thermal_reported_when_both_sides_have_it(self, smc, recorder,
                                                      tmp_path):
        cap_a, log_a, cap_b, log_b = _build_pair(
            tmp_path, recorder, thermal_a=self.THERM_A,
            thermal_b=self.THERM_B)
        out = tmp_path / "r"
        assert smc.main(_argv(cap_a, log_a, cap_b, log_b, out)) == 0
        seg = json.loads((out / "report.json").read_text())["segments"]["wave"]
        th = seg["thermal"]
        # winding=70 on motor 1 beats driver=55: max of the TWO sensors.
        assert th["hw"]["hottest_motor"] == smc.joint_name(1)
        assert th["hw"]["max_temp_c"] == 70
        assert th["sim"]["hottest_motor"] == smc.joint_name(0)
        assert th["sim"]["max_temp_c"] == 45

    def test_thermal_missing_on_one_side_skips_with_info(self, smc, recorder,
                                                         tmp_path, capsys):
        """A sim capture has no temperatures. That is normal, not an error:
        one info line, no thermal section, exit 0."""
        cap_a, log_a, cap_b, log_b = _build_pair(tmp_path, recorder,
                                                 thermal_a=self.THERM_A)
        out = tmp_path / "r"
        assert smc.main(_argv(cap_a, log_a, cap_b, log_b, out)) == 0
        seg = json.loads((out / "report.json").read_text())["segments"]["wave"]
        assert "thermal" not in seg
        assert "thermal" in (capsys.readouterr().out + "").lower()


class TestSinglePassContract:

    def test_whole_compare_path_runs_on_pure_generators(self, smc, recorder,
                                                        tmp_path, monkeypatch):
        """Production is ~3.6M records (~1 GB) per stream, delivered by the
        read_stream GENERATOR: single-use, no len(), and a re-open is the only
        way to see the data twice. Wrapping read_stream to (a) return a bare
        generator and (b) count opens proves the whole path -- slicing,
        metrics, plots -- completes in exactly one pass per file."""
        cap_a, log_a, cap_b, log_b = _build_pair(tmp_path, recorder)
        real = smc.read_stream
        opens = []

        def strict(bin_path, schema_path=None):
            opens.append(str(bin_path))
            return (r for r in real(bin_path, schema_path))

        monkeypatch.setattr(smc, "read_stream", strict)
        assert smc.main(_argv(cap_a, log_a, cap_b, log_b, tmp_path / "r")) == 0
        assert len(opens) == len(set(opens)), (
            "a stream file was opened twice -- that is a second pass, and at "
            "production scale it doubles a multi-minute read")

    def test_no_stream_is_ever_materialised(self, smc):
        """Same static guard test_hw_capture.py keeps on the probe: the exact
        line `list(read_stream(` is what made the timebase probe unusable on
        real captures."""
        src = (SCRIPTS / "sim_match_compare.py").read_text()
        assert "list(read_stream(" not in src
