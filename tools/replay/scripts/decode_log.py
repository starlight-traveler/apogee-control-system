#!/usr/bin/env python3
"""Decode a binary ACS sensor log into CSV and event output."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from replaylib.cli import run_archived


if __name__ == "__main__":
    raise SystemExit(run_archived("archive/decode_sensor_log.py", sys.argv[1:], "decode_log.py"))
