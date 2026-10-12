"""Machine-specific roots, resolved once.

Every path that would otherwise be spelled with one developer's home directory
goes through here. Precedence: an environment variable first, then a layout
that works on any machine (``~/GR00T-WholeBodyControl`` beside this repo).

**Never re-hardcode a home directory.** ``tests/test_paths.py`` walks every
``.py`` and ``.sh`` in the tree and fails if one reappears, and it builds its
search string from fragments so the test is not its own offender.

Override examples::

    WBC_ROOT=/data/GR00T-WholeBodyControl python scripts/sonic_sim_demo.py ...
    PA_ARTIFACTS=/tmp/pa-artifacts pytest

Roots
-----
``PA_ROOT`` / ``PA_ARTIFACTS``
    This repository and its evidence directory.
``WBC_ROOT`` / ``WBC_PY``
    NVIDIA GR00T-WholeBodyControl (the SONIC whole-body controller) and the
    interpreter of its simulation virtualenv. External project, not shipped.
``UNITREE_SIM_ROOT``
    A checkout of Unitree's ``unitree_sim_isaaclab``, which the patches under
    ``third_party/unitree-sim-isaaclab-local/`` apply to. External, not shipped.
``ISAAC_PY`` / ``ISAAC_CYCLONEDDS_HOME``
    The Isaac-side interpreter and the source-built CycloneDDS C core it must be
    pointed at. Unset on any machine without that (frozen) stack, and the
    scripts and tests that need them skip cleanly rather than guess a path.
``SCENE_SOURCE_ROOT``
    Output tree of a photo-to-MuJoCo scene reconstruction tool -- a separate,
    unpublished project of the same author, not included here.
    ``simbridge/make_lounge_scene.py`` reads a furnished-room MJCF out of it;
    that module's docstring gives the layout it expects.
"""
from __future__ import annotations

import os
import pathlib


def _env_path(name: str, default: pathlib.Path) -> pathlib.Path:
    value = os.environ.get(name)
    return pathlib.Path(value).expanduser() if value else default


def _env_path_or_none(name: str) -> pathlib.Path | None:
    """For roots with no sane default: absent means "this machine has no such
    stack", and callers must skip rather than guess."""
    value = os.environ.get(name)
    return pathlib.Path(value).expanduser() if value else None


# This repository.
PA_ROOT = _env_path("PA_ROOT", pathlib.Path(__file__).resolve().parents[1])
PA_ARTIFACTS = _env_path("PA_ARTIFACTS", PA_ROOT / "artifacts")

# The vendor GR00T-WholeBodyControl tree (NVIDIA upstream + the local patch in
# third_party/groot-wbc-local/).
WBC_ROOT = _env_path("WBC_ROOT", pathlib.Path.home() / "GR00T-WholeBodyControl")
WBC_PY = _env_path("WBC_PY", WBC_ROOT / ".venv_sim" / "bin" / "python")
WBC_DEPLOY = WBC_ROOT / "gear_sonic_deploy"
G1_MODEL_DIR = WBC_ROOT / "gear_sonic" / "data" / "robot_model" / "model_data" / "g1"

# Unitree's unitree_sim_isaaclab checkout (Apache-2.0): the base the eight
# patches in third_party/unitree-sim-isaaclab-local/ apply to.
UNITREE_SIM_ROOT = _env_path_or_none("UNITREE_SIM_ROOT")

# The Isaac side: a separate interpreter, and a source-built CycloneDDS whose
# C core every DDS process under it must agree on.
ISAAC_PY = _env_path_or_none("ISAAC_PY")
ISAAC_CYCLONEDDS_HOME = _env_path_or_none("ISAAC_CYCLONEDDS_HOME")

# Output of a photo-to-MuJoCo scene reconstruction tool: a separate, unpublished
# project of the same author, not shipped here.
SCENE_SOURCE_ROOT = _env_path("SCENE_SOURCE_ROOT", pathlib.Path.home() / "scene-source")

__all__ = [
    "PA_ROOT", "PA_ARTIFACTS",
    "WBC_ROOT", "WBC_PY", "WBC_DEPLOY", "G1_MODEL_DIR",
    "UNITREE_SIM_ROOT", "ISAAC_PY", "ISAAC_CYCLONEDDS_HOME",
    "SCENE_SOURCE_ROOT",
]
