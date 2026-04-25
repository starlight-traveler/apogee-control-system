"""Frame-adapter helpers for legacy replay logs."""

from __future__ import annotations

from pathlib import Path

from .paths import EXPERIMENTS_DIR


def archived_frame_adapter() -> Path:
    return EXPERIMENTS_DIR / "fullscale_4" / "replay_legacy_frame_adapter.py"


def archived_state_zenith_recalculator() -> Path:
    return EXPERIMENTS_DIR / "fullscale_4" / "recalculate_state_zenith_csv.py"

