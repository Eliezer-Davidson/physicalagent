"""The interop probe is the gate everything DDS stands on, so its stdlib half
is pinned here. Two properties matter enough to test: it must never publish on
a real telemetry topic (the first version fabricated zero LowState_ on
rt/lowstate), and the env plumbing that makes the Isaac interpreter's DDS work
must actually be applied where it claims to be."""
from __future__ import annotations

import importlib.util
import pathlib
import re

import pytest

REPO = pathlib.Path(__file__).resolve().parents[1]

spec = importlib.util.spec_from_file_location(
    "dds_interop_probe", REPO / "scripts" / "dds_interop_probe.py")
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


class TestItNeverTouchesRealTopics:
    def test_probe_topic_is_probe_scoped(self):
        """Fabricated telemetry on a live topic is the defect class this
        project once spent a day removing. rt/* is the live namespace."""
        assert not probe.TOPIC.startswith("rt/")
        assert probe.TOPIC == "probe/dds_interop"

    def test_source_never_names_a_live_topic_for_publishing(self):
        src = (REPO / "scripts" / "dds_interop_probe.py").read_text()
        assert 'ChannelPublisher("rt/' not in src


class TestEnvPlumbing:
    def test_the_load_bearing_install_exists(self):
        """If the source-built CycloneDDS install disappears (machine wipe,
        cleanup), every DDS process under the Isaac interpreter silently mixes
        C cores again. Fail loudly here instead -- but only on a machine that
        claims to have that stack. ISAAC_CYCLONEDDS_HOME unset means there is
        nothing to check, not that something is broken."""
        if not probe.ISAAC_DDS_ENV:
            pytest.skip("ISAAC_CYCLONEDDS_HOME unset: no Isaac stack here")
        cdds = pathlib.Path(probe.ISAAC_DDS_ENV["CYCLONEDDS_HOME"])
        assert (cdds / "lib" / "libddsc.so").exists()

    def test_env_for_isaac_applies_both_vars(self, monkeypatch):
        """Pointed at an Isaac interpreter, _env_for must add BOTH vars -- one
        without the other is the mixed-core failure this whole module exists
        for. Exercised against injected values so it runs anywhere."""
        monkeypatch.setattr(probe, "ISAAC_PY", "/opt/isaac/bin/python")
        monkeypatch.setattr(probe, "ISAAC_DDS_ENV",
                            {"CYCLONEDDS_HOME": "/opt/cdds",
                             "LD_LIBRARY_PATH": "/opt/cdds/lib"})
        env = probe._env_for(probe.ISAAC_PY)
        assert env["CYCLONEDDS_HOME"] == "/opt/cdds"
        assert env["LD_LIBRARY_PATH"] == "/opt/cdds/lib"

    def test_env_for_mujoco_stays_clean(self, monkeypatch):
        """SONIC's world must never be pointed at the Isaac core."""
        import os
        monkeypatch.setattr(probe, "ISAAC_PY", "/opt/isaac/bin/python")
        monkeypatch.setattr(probe, "ISAAC_DDS_ENV",
                            {"CYCLONEDDS_HOME": "/opt/cdds",
                             "LD_LIBRARY_PATH": "/opt/cdds/lib"})
        env = probe._env_for(probe.MUJOCO_PY)
        assert env.get("CYCLONEDDS_HOME") == os.environ.get("CYCLONEDDS_HOME")

    def test_no_interpreter_path_is_hard_coded(self):
        """Both interpreters come from the environment. A literal path here
        would be both a portability bug and a privacy leak."""
        src = (REPO / "scripts" / "dds_interop_probe.py").read_text()
        assert 'os.environ.get("ISAAC_PY")' in src
        assert 'os.environ.get("ISAAC_CYCLONEDDS_HOME")' in src

    def test_matrix_refuses_instead_of_guessing_when_unset(self, monkeypatch, capsys):
        """Without the Isaac stack the matrix must refuse with exit 2, not
        invent a path and report a false silent bus."""
        monkeypatch.setattr(probe, "ISAAC_PY", None)
        monkeypatch.setattr(probe, "_CDDS", None)
        assert probe.matrix() == 2
        assert "no Isaac-side stack" in capsys.readouterr().err


class TestSummaryLineCoupling:
    """_run_pub_sub parses the endpoint's printed summaries; if either format
    drifts, counts read as -1 and the matrix must say PROBE FAILED, never
    'silent'. Pin the format coupling."""

    def test_pub_summary_matches_the_parser(self):
        line = "[pub] wrote 384 messages in 14s"
        m = re.search(r"wrote (\d+) messages", line)
        assert m and m.group(1) == "384"
        assert 'wrote {n} messages' in (REPO / "scripts" / "dds_interop_probe.py").read_text()

    def test_sub_summary_matches_the_parser(self):
        line = "[sub] received 293 messages"
        m = re.search(r"received (\d+) messages", line)
        assert m and m.group(1) == "293"
        assert "received {got['n']} messages" in (REPO / "scripts" / "dds_interop_probe.py").read_text()
