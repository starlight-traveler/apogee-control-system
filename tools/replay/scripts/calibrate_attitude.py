#!/usr/bin/env python3
"""Align ACS attitude data to a recovery-module reference log."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from replaylib.cli import run_archived


if __name__ == "__main__":
    raise SystemExit(
        run_archived("fullscale_3/calibrate_attitude_to_recovery.py", sys.argv[1:], "calibrate_attitude.py")
    )
