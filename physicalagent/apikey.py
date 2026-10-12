"""Load the Gemini API key with no third-party dependencies.

physicalagent.config does this too, but it imports python-dotenv at module
scope. The SONIC scripts run inside GR00T-WholeBodyControl/.venv_sim, which is
NVIDIA's simulator environment and does not have dotenv -- importing config
there fails before it can read anything. This parses .env directly so the same
key works from either interpreter.

Deliberately minimal: it reads one variable and does not export anything into
the process environment, so importing it cannot change the behaviour of
anything else.
"""
from __future__ import annotations

import os
import pathlib

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]


def load_gemini_key(env_path: str | os.PathLike | None = None) -> str:
    """Return the Gemini API key, or raise SystemExit with an actionable message.

    Environment wins over .env so a session can override without editing a file.
    """
    key = os.environ.get("GEMINI_API_KEY", "").strip()
    if key:
        return key

    path = pathlib.Path(env_path) if env_path else REPO_ROOT / ".env"
    if path.exists():
        for line in path.read_text().splitlines():
            line = line.strip()
            if line.startswith("#") or "=" not in line:
                continue
            name, _, value = line.partition("=")
            if name.strip() == "GEMINI_API_KEY":
                value = value.strip().strip('"').strip("'")
                if value:
                    return value

    raise SystemExit(
        f"GEMINI_API_KEY not found. Export it, or put it in {path}"
    )
