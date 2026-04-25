"""Repository paths used by replay tools."""

from __future__ import annotations

from pathlib import Path


def find_repo_root(start: Path | None = None) -> Path:
    """Find the repository root by walking upward to platformio.ini."""

    current = (start or Path(__file__)).resolve()
    for parent in (current, *current.parents):
        if (parent / "platformio.ini").exists():
            return parent
    raise RuntimeError(f"Could not find repository root from {current}")


ROOT = find_repo_root()
TOOLS_DIR = ROOT / "tools"
REPLAY_DIR = TOOLS_DIR / "replay"
SCRIPTS_DIR = REPLAY_DIR / "scripts"
DATA_DIR = REPLAY_DIR / "data"
PLOTS_DIR = REPLAY_DIR / "plots"
EXPERIMENTS_DIR = REPLAY_DIR / "experiments"
ARCHIVE_DIR = EXPERIMENTS_DIR / "archive"


def replay_binary_candidates() -> list[Path]:
    return [
        TOOLS_DIR / "build" / "bin" / "acs_replay",
        REPLAY_DIR / "build" / "bin" / "acs_replay",
    ]


def find_replay_binary(explicit: Path | None = None) -> Path:
    if explicit is not None:
        path = explicit.expanduser().resolve()
        if path.exists():
            return path
        raise FileNotFoundError(f"Replay binary not found: {path}")

    for path in replay_binary_candidates():
        if path.exists():
            return path

    candidates = "\n".join(f"  - {path}" for path in replay_binary_candidates())
    raise FileNotFoundError(f"Replay binary not found. Checked:\n{candidates}")

