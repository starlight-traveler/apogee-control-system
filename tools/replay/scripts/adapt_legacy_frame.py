#!/usr/bin/env python3
"""Adapt historical replay logs into the current replay frame convention."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from replaylib.cli import run_archived


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--tool",
        default="frame-adapter",
        choices=("frame-adapter", "state-zenith"),
        help="Legacy conversion tool to run.",
    )
    args, passthrough = parser.parse_known_args()
    script = {
        "frame-adapter": "fullscale_4/replay_legacy_frame_adapter.py",
        "state-zenith": "fullscale_4/recalculate_state_zenith_csv.py",
    }[args.tool]
    return run_archived(script, passthrough, "adapt_legacy_frame.py")


if __name__ == "__main__":
    raise SystemExit(main())
