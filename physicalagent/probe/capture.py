"""Capture-folder layout for the bring-up probe."""
from __future__ import annotations

import json
from pathlib import Path


def capture_dir(base: Path, robot: str, stamp: str) -> Path:
    """<base>/<robot>_<stamp> — sorts chronologically per robot."""
    return Path(base) / f"{robot}_{stamp}"


class CaptureWriter:
    """Writes one capture folder. Creates it on first use."""

    def __init__(self, root: Path):
        self._root = Path(root)

    @property
    def path(self) -> Path:
        return self._root

    def _ensure(self) -> Path:
        self._root.mkdir(parents=True, exist_ok=True)
        return self._root

    def write_text(self, name: str, text: str) -> Path:
        p = self._ensure() / name
        p.write_text(text)
        return p

    def write_manifest(self, manifest: dict) -> Path:
        """Indented JSON — a human reads this during bring-up."""
        return self.write_text("manifest.json", json.dumps(manifest, indent=2, sort_keys=True))
