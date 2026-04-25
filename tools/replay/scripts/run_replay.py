#!/usr/bin/env python3
"""Run the hosted acs_replay binary with a stable command name."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from replaylib.replay_runner import run_replay


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_csv", nargs="?", type=Path, help="Replay input CSV.")
    parser.add_argument("--binary", type=Path, help="Explicit acs_replay binary path.")
    args, passthrough = parser.parse_known_args()
    return run_replay(args.input_csv, passthrough, args.binary)


if __name__ == "__main__":
    raise SystemExit(main())

