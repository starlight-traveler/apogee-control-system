#!/usr/bin/env python3
"""Generate curated flight-analysis plots from archived implementations."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from replaylib.cli import format_choices, run_archived


PLOT_VIEWS: dict[tuple[str, str], str] = {
    ("fullscale_2", "apogee"): "archive/plot_fullscale_2_apogee.py",
    ("fullscale_2", "compare"): "archive/plot_fullscale_2_compare.py",
    ("fullscale_2", "zoom"): "archive/plot_fullscale_2_compare_zoom.py",
    ("fullscale_3", "baro-seed"): "fullscale_3/plot_fullscale_3_baro_seed_validation.py",
    ("fullscale_3", "body-axis"): "fullscale_3/diagnose_body_axis.py",
    ("fullscale_3", "causal-reconstruction"): "fullscale_3/plot_fullscale_3_causal_reconstruction.py",
    ("fullscale_3", "estimator-diagnostics"): "fullscale_3/plot_estimator_diagnostics.py",
    ("fullscale_3", "lsm-zenith"): "fullscale_3/plot_lsm_zenith_expected.py",
    ("fullscale_3", "zenith-methods"): "fullscale_3/compare_zenith_methods.py",
    ("fullscale_4", "aero-bias"): "fullscale_4/plot_apogee_bias_breakdown.py",
    ("fullscale_4", "baro-seed"): "fullscale_4/plot_fullscale_4_baro_altitude_seed.py",
    ("fullscale_4", "baro-vz"): "fullscale_4/plot_baro_vz_counterfactual_apogee.py",
    ("fullscale_4", "fullscale-params"): "fullscale_4/plot_fullscale_4_fullscale_params_replay.py",
    ("fullscale_4", "lsm-track"): "fullscale_4/plot_fullscale_4_lsm_track_replay.py",
    ("fullscale_4", "overshoot-actuation"): "fullscale_4/plot_fullscale_4_overshoot_actuation.py",
    ("fullscale_4", "quaternion-path"): "fullscale_4/plot_quaternion_path_diagnosis.py",
    ("fullscale_4", "raw-quaternion"): "fullscale_4/plot_raw_quaternion_zenith.py",
    ("fullscale_4", "tilt"): "fullscale_4/plot_tilt_magnitude_comparison.py",
    ("fullscale_4", "validation"): "fullscale_4/plot_fullscale_4_validation.py",
    ("fullscale_4", "wt901-zenith"): "fullscale_4/plot_wt901_rotated_zenith.py",
}


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        epilog=format_choices((f"{flight}:{view}", script) for (flight, view), script in PLOT_VIEWS.items()),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--flight", help="Flight key, for example fullscale_4.")
    parser.add_argument("--view", help="Plot view, for example validation.")
    parser.add_argument("--list", action="store_true", help="List available flight/view combinations.")
    args, passthrough = parser.parse_known_args()

    if args.list:
        print(format_choices((f"{flight}:{view}", script) for (flight, view), script in PLOT_VIEWS.items()))
        return 0
    if not args.flight or not args.view:
        parser.error("--flight and --view are required unless --list is used")

    key = (args.flight, args.view)
    if key not in PLOT_VIEWS:
        parser.error(f"unknown plot view {args.flight}:{args.view}; use --list")
    return run_archived(PLOT_VIEWS[key], passthrough)


if __name__ == "__main__":
    raise SystemExit(main())
