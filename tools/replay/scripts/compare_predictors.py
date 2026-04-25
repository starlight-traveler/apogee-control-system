#!/usr/bin/env python3
"""Run predictor comparison and model-variant analyses."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from replaylib.cli import format_choices, run_archived


MODES: dict[str, str] = {
    "adaptive-drag": "aero_model/compute_apogee_adaptive_drag.py",
    "adaptive-drag-banded": "aero_model/compute_apogee_adaptive_drag_banded.py",
    "compare": "aero_model/compare_apogee_predictors.py",
    "current-model": "aero_model/compute_apogee_current_model.py",
    "merged-seed": "aero_model/compute_apogee_merged_seed.py",
    "prediction-variants": "aero_model/compute_apogee_prediction_variants.py",
    "predictor-sim": "aero_model/plot_predictor_sim_comparison.py",
    "scaled-cfd": "aero_model/plot_scaled_cfd_replay.py",
    "tune-adaptive-drag": "aero_model/tune_adaptive_drag.py",
    "burnout-aggressive": "burnout_detection/compute_apogee_aggressive_burnout.py",
    "burnout-bridge": "burnout_detection/compute_apogee_burnout_bridge.py",
    "burnout-tail": "burnout_detection/compute_apogee_burnout_tail.py",
}


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        epilog=format_choices(MODES.items()),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--mode", default="current-model", choices=sorted(MODES), help="Analysis mode.")
    parser.add_argument("--list", action="store_true", help="List available modes.")
    args, passthrough = parser.parse_known_args()

    if args.list:
        print(format_choices(MODES.items()))
        return 0
    return run_archived(MODES[args.mode], passthrough)


if __name__ == "__main__":
    raise SystemExit(main())

