#!/usr/bin/env python3

from __future__ import annotations

import itertools
import math
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
REPLAY_DIR = SCRIPT_DIR.parent
ROOT = REPLAY_DIR.parents[1]
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import compute_apogee_adaptive_drag as adaptive


def run_case(
    active_rows,
    merged_rows,
    force_table,
    actual_apogee_m: float,
    *,
    min_drag_scale: float,
    max_drag_scale: float,
    adaptation_tau_s: float,
    residual_clamp_mps2: float,
    min_axial_accel_abs_mps2: float,
    residual_gain: float,
    coast_delay_s: float,
):
    drag_scale = 1.0
    previous_time_s = active_rows[0].time_s
    coast_start_time_s = None
    rows = []

    for active_row, merged_row in zip(active_rows, merged_rows):
        dt_seconds = active_row.time_s - previous_time_s
        if merged_row.status == "coast" and coast_start_time_s is None:
            coast_start_time_s = active_row.time_s

        seed_state = adaptive.ApogeeState(
            altitude_m=merged_row.altitude_m,
            horizontal_m=0.0,
            vertical_v=merged_row.velocity_mps,
            horizontal_v=merged_row.horizontal_speed_mps,
            zenith=merged_row.zenith_rad,
            angular_v=merged_row.angular_rate_rad_s,
            acs_deg=0.0,
        )
        predicted_accel_z_mps2, _, _, axial_accel_z_mps2 = adaptive._compute_accel_with_axial_scale(
            seed_state, force_table, drag_scale
        )
        accel_residual_mps2 = active_row.measured_accel_z_mps2 - predicted_accel_z_mps2

        valid_adaptation = (
            merged_row.status == "coast"
            and merged_row.velocity_mps > 0.0
            and adaptive._fresh_sample(dt_seconds)
            and math.isfinite(active_row.measured_accel_z_mps2)
            and math.isfinite(merged_row.zenith_rad)
            and coast_start_time_s is not None
            and (active_row.time_s - coast_start_time_s) >= coast_delay_s
        )
        if valid_adaptation and abs(axial_accel_z_mps2) >= min_axial_accel_abs_mps2:
            clamped_residual = adaptive._clamp(accel_residual_mps2, -residual_clamp_mps2, residual_clamp_mps2)
            target_scale = drag_scale + residual_gain * (clamped_residual / axial_accel_z_mps2)
            target_scale = adaptive._clamp(target_scale, min_drag_scale, max_drag_scale)
            alpha = 1.0 - math.exp(-dt_seconds / adaptation_tau_s) if dt_seconds > 0.0 else 0.0
            drag_scale = adaptive._clamp(
                drag_scale + alpha * (target_scale - drag_scale),
                min_drag_scale,
                max_drag_scale,
            )

        adaptive_apogee_m = adaptive._predict_apogee_with_axial_scale(seed_state, force_table, drag_scale)
        rows.append(
            {
                "time_s": active_row.time_s,
                "altitude_m": active_row.altitude_m,
                "adaptive_apogee_m": adaptive_apogee_m,
            }
        )
        previous_time_s = active_row.time_s

    lock25 = adaptive._first_within(rows, "adaptive_apogee_m", actual_apogee_m, 25.0)
    lock10 = adaptive._first_within(rows, "adaptive_apogee_m", actual_apogee_m, 10.0)
    lock5 = adaptive._first_within(rows, "adaptive_apogee_m", actual_apogee_m, 5.0)
    stats = adaptive._post_lock_metrics(rows, "adaptive_apogee_m", actual_apogee_m, 25.0)
    if stats is None:
        return None

    score = (
        abs(stats["mean_error_m"]) * 3.0
        + max(0.0, stats["max_overshoot_m"]) * 1.5
        + stats["overshoot_time_s"] * 8.0
        + max(0.0, (lock10["time_s"] - 604.5092163085938) if lock10 else 20.0) * 10.0
        + max(0.0, (lock25["time_s"] - 604.406005859375) if lock25 else 20.0) * 6.0
    )

    return {
        "score": score,
        "min_drag_scale": min_drag_scale,
        "max_drag_scale": max_drag_scale,
        "adaptation_tau_s": adaptation_tau_s,
        "residual_clamp_mps2": residual_clamp_mps2,
        "min_axial_accel_abs_mps2": min_axial_accel_abs_mps2,
        "residual_gain": residual_gain,
        "coast_delay_s": coast_delay_s,
        "lock25_time_s": None if lock25 is None else lock25["time_s"],
        "lock10_time_s": None if lock10 is None else lock10["time_s"],
        "lock5_time_s": None if lock5 is None else lock5["time_s"],
        "mean_error_m": stats["mean_error_m"],
        "max_overshoot_m": stats["max_overshoot_m"],
        "max_undershoot_m": stats["max_undershoot_m"],
        "overshoot_time_s": stats["overshoot_time_s"],
    }


def main() -> None:
    active_rows = adaptive._load_active_replay_rows(adaptive.INPUT_CSV)
    merged_rows = adaptive._load_merged_seed_rows(adaptive.MERGED_CSV)
    force_table = adaptive.load_force_table(adaptive.CFD_CSV)
    actual_apogee_m = adaptive._actual_apogee_from_output(adaptive.INPUT_CSV)

    results = []
    grid = itertools.product(
        [1.00],
        [1.25, 1.30, 1.35],
        [0.10, 0.20],
        [4.0, 6.0],
        [0.75],
        [0.75, 1.00],
        [0.00, 0.10],
    )
    total = 0
    for params in grid:
        total += 1
        result = run_case(
            active_rows,
            merged_rows,
            force_table,
            actual_apogee_m,
            min_drag_scale=params[0],
            max_drag_scale=params[1],
            adaptation_tau_s=params[2],
            residual_clamp_mps2=params[3],
            min_axial_accel_abs_mps2=params[4],
            residual_gain=params[5],
            coast_delay_s=params[6],
        )
        if result is not None:
            results.append(result)

    results.sort(key=lambda row: row["score"])
    print(f"cases={total} valid={len(results)} actual_apogee={actual_apogee_m:.2f}m")
    for row in results[:12]:
        print(
            "score={score:.2f} max_scale={max_drag_scale:.2f} tau={adaptation_tau_s:.2f} "
            "clamp={residual_clamp_mps2:.1f} min_axial={min_axial_accel_abs_mps2:.2f} "
            "gain={residual_gain:.2f} delay={coast_delay_s:.2f} "
            "lock25={lock25_time_s:.3f} lock10={lock10_time_s:.3f} lock5={lock5_time_s:.3f} "
            "mean={mean_error_m:.2f} over={max_overshoot_m:.2f} over_t={overshoot_time_s:.3f}".format(**row)
        )


if __name__ == "__main__":
    main()
