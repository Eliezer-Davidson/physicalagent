"""physicalagent.paths — the one place machine layout lives."""
import importlib
import pathlib
import re

from physicalagent import paths

_EXTERNAL_ROOTS = ("UNITREE_SIM_ROOT", "ISAAC_PY", "ISAAC_CYCLONEDDS_HOME")


def _reload():
    return importlib.reload(paths)


def test_pa_root_is_this_repo(monkeypatch):
    monkeypatch.delenv("PA_ROOT", raising=False)
    p = _reload()
    assert (p.PA_ROOT / "pyproject.toml").exists()
    assert p.PA_ARTIFACTS == p.PA_ROOT / "artifacts"


def test_wbc_defaults_live_beside_the_repo_under_home(monkeypatch):
    for k in ("WBC_ROOT", "WBC_PY", "SCENE_SOURCE_ROOT"):
        monkeypatch.delenv(k, raising=False)
    p = _reload()
    assert p.WBC_ROOT == pathlib.Path.home() / "GR00T-WholeBodyControl"
    assert p.WBC_PY == p.WBC_ROOT / ".venv_sim" / "bin" / "python"
    assert p.WBC_DEPLOY == p.WBC_ROOT / "gear_sonic_deploy"
    assert p.G1_MODEL_DIR == p.WBC_ROOT / "gear_sonic/data/robot_model/model_data/g1"
    assert p.SCENE_SOURCE_ROOT == pathlib.Path.home() / "scene-source"


def test_env_overrides_win(monkeypatch, tmp_path):
    monkeypatch.setenv("WBC_ROOT", str(tmp_path / "wbc"))
    monkeypatch.setenv("PA_ARTIFACTS", str(tmp_path / "art"))
    monkeypatch.setenv("SCENE_SOURCE_ROOT", str(tmp_path / "scene"))
    p = _reload()
    assert p.WBC_ROOT == tmp_path / "wbc"
    assert p.WBC_PY == tmp_path / "wbc" / ".venv_sim" / "bin" / "python"
    assert p.G1_MODEL_DIR.is_relative_to(tmp_path / "wbc")
    assert p.PA_ARTIFACTS == tmp_path / "art"
    assert p.SCENE_SOURCE_ROOT == tmp_path / "scene"


def test_roots_with_no_sane_default_are_none_when_unset(monkeypatch):
    """A machine without the frozen Isaac stack must get None, not a guess.
    Guessing is how a literal home directory gets reintroduced."""
    for k in _EXTERNAL_ROOTS:
        monkeypatch.delenv(k, raising=False)
    p = _reload()
    assert p.UNITREE_SIM_ROOT is None
    assert p.ISAAC_PY is None
    assert p.ISAAC_CYCLONEDDS_HOME is None


def test_external_roots_take_the_environment(monkeypatch, tmp_path):
    monkeypatch.setenv("UNITREE_SIM_ROOT", str(tmp_path / "usi"))
    monkeypatch.setenv("ISAAC_PY", str(tmp_path / "py"))
    monkeypatch.setenv("ISAAC_CYCLONEDDS_HOME", str(tmp_path / "cdds"))
    p = _reload()
    assert p.UNITREE_SIM_ROOT == tmp_path / "usi"
    assert p.ISAAC_PY == tmp_path / "py"
    assert p.ISAAC_CYCLONEDDS_HOME == tmp_path / "cdds"


def test_no_developer_home_left_anywhere_in_the_tree():
    """Machine layout belongs in this module and nowhere else. Any
    ``/home/<user>`` path in a source file is a developer machine leaking into
    the tree -- except the robots' own vendor accounts, which are the robots'
    layout, not ours. The rule is generic on purpose: no hostname or user name
    has to be spelled here for the guard to work, and there is no exemption
    list: every source file in the repo is checked."""
    allowed = {"/home/hiwonder", "/home/pi"}
    home_path = re.compile(r"/home/[A-Za-z0-9_.-]+")
    root = paths.PA_ROOT
    offenders = []
    for pat in ("*.py", "*.sh"):
        for f in root.rglob(pat):
            rel = f.relative_to(root).as_posix()
            if rel.startswith(".venv"):
                continue
            text = f.read_text(errors="replace")
            for m in home_path.findall(text):
                if m not in allowed:
                    offenders.append(f"{rel}: {m}")
    assert offenders == []
