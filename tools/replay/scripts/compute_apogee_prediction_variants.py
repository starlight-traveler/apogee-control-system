#!/usr/bin/env python3

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
REPLAY_DIR = SCRIPT_DIR.parent
ROOT = REPLAY_DIR.parents[1]
DATA_DIR = REPLAY_DIR / "data"
PLOTS_DIR = REPLAY_DIR / "plots"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import compute_apogee_adaptive_drag as adaptive


INPUT_CSV = adaptive.INPUT_CSV
MERGED_CSV = adaptive.MERGED_CSV
CFD_CSV = adaptive.CFD_CSV

OUTPUT_CSV = DATA_DIR / "output_prediction_variants.csv"
OUTPUT_COMPARE_SVG = PLOTS_DIR / "prediction_variants.svg"
OUTPUT_CONTROL_SVG = PLOTS_DIR / "prediction_variant_controls.svg"


LOW_SPEED_SPLIT_MPS = 100.0
LOW_SPEED_MIN_SCALE = 1.00
LOW_SPEED_MAX_SCALE = 1.45
LOW_SPEED_TAU_S = 0.20

MACH_BLEND_START = 0.55
MACH_BLEND_FULL = 0.25
MACH_MIN_SCALE = 1.00
MACH_MAX_SCALE = 1.45
MACH_TAU_S = 0.20

RESIDUAL_CLAMP_MPS2 = 6.0
MIN_AXIAL_ACCEL_ABS_MPS2 = 0.75
RESIDUAL_GAIN = 1.0

PREDICTOR_BLEND_START_VZ_MPS = 75.0
PREDICTOR_BLEND_FULL_VZ_MPS = 30.0


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def _smoothstep(edge0: float, edge1: float, x: float) -> float:
    if edge0 == edge1:
        return 1.0 if x >= edge1 else 0.0
    t = _clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def _compute_state_mach(state: adaptive.ApogeeState) -> float:
    rel_x = state.vertical_v
    rel_y = state.horizontal_v - adaptive.gradient_wind()[1]
    temp_k = adaptive.temperature_k(state.altitude_m)
    speed_of_sound = math.sqrt(adaptive.GAMMA * adaptive.R_GAS * temp_k) if temp_k > 0.0 else 0.0
    return (math.hypot(rel_x, rel_y) / speed_of_sound) if speed_of_sound > 0.0 else 0.0


def _predict_with_scale(state: adaptive.ApogeeState, force_table: adaptive.ForceTable, drag_scale: float) -> float:
    return adaptive._predict_apogee_with_axial_scale(state, force_table, drag_scale)


def _predict_with_mach_blend(
    state: adaptive.ApogeeState,
    force_table: adaptive.ForceTable,
    mach_drag_scale: float,
) -> float:
    mach = _compute_state_mach(state)
    mach_weight = 1.0 - _smoothstep(MACH_BLEND_FULL, MACH_BLEND_START, mach)
    effective_scale = 1.0 + mach_weight * (mach_drag_scale - 1.0)
    return adaptive._predict_apogee_with_axial_scale(state, force_table, effective_scale)


def _first_within(rows, key: str, actual_apogee_m: float, threshold_m: float):
    for row in rows:
        if abs(float(row[key]) - actual_apogee_m) <= threshold_m:
            return row
    return None


def _post_lock_metrics(rows, key: str, actual_apogee_m: float, lock_threshold_m: float):
    return adaptive._post_lock_metrics(rows, key, actual_apogee_m, lock_threshold_m)


def _print_metrics(label: str, rows, key: str, actual_apogee_m: float) -> None:
    print(label)
    for threshold_m in (25.0, 10.0, 5.0):
        hit = _first_within(rows, key, actual_apogee_m, threshold_m)
        if hit is None:
            print(f"  within_{int(threshold_m)}m: none")
        else:
            print(
                f"  within_{int(threshold_m)}m: "
                f"t={float(hit['time_s']):.3f}s alt={float(hit['altitude_m']):.1f}m pred={float(hit[key]):.1f}m"
            )
    stats = _post_lock_metrics(rows, key, actual_apogee_m, 25.0)
    if stats is None:
        print("  post_lock_25m: none")
    else:
        print(
            "  post_lock_25m: "
            f"start={stats['start_time_s']:.3f}s/{stats['start_altitude_m']:.1f}m "
            f"mean_err={stats['mean_error_m']:.1f}m "
            f"max_over={stats['max_overshoot_m']:.1f}m "
            f"time_over={stats['overshoot_time_s']:.3f}s"
        )


def _write_compare_svg(rows: list[dict[str, float]], actual_apogee_m: float) -> None:
    sampled = adaptive._decimate(rows, 6000)
    times = [float(row["time_s"]) for row in sampled]
    altitude = [float(row["altitude_m"]) for row in sampled]
    logged = [float(row["logged_apogee_m"]) for row in sampled]
    old_model = [float(row["merged_apogee_m"]) for row in sampled]
    low_speed = [float(row["low_speed_apogee_m"]) for row in sampled]
    mach_based = [float(row["mach_adaptive_apogee_m"]) for row in sampled]
    blended = [float(row["blended_apogee_m"]) for row in sampled]
    actual = [actual_apogee_m for _ in sampled]

    width = 1800
    height = 1000
    left = 110
    right = 40
    top = 70
    bottom_margin = 80
    plot_w = width - left - right
    plot_h = height - top - bottom_margin

    t_min = min(times)
    t_max = max(times)
    y_min = min(min(altitude), min(logged), min(old_model), min(low_speed), min(mach_based), min(blended), actual_apogee_m)
    y_max = max(max(altitude), max(logged), max(old_model), max(low_speed), max(mach_based), max(blended), actual_apogee_m)
    y_pad = max(1.0, (y_max - y_min) * 0.05)
    y_min -= y_pad
    y_max += y_pad

    altitude_points = adaptive._polyline_points(times, altitude, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    logged_points = adaptive._polyline_points(times, logged, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    old_points = adaptive._polyline_points(times, old_model, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    low_points = adaptive._polyline_points(times, low_speed, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    mach_points = adaptive._polyline_points(times, mach_based, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    blended_points = adaptive._polyline_points(times, blended, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    actual_points = adaptive._polyline_points(times, actual, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)

    grid_lines = []
    for i in range(6):
        x = left + (plot_w * i / 5.0)
        grid_lines.append(
            f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{top + plot_h}" stroke="#d8d8d8" stroke-width="1"/>'
        )
    for i in range(6):
        y = top + (plot_h * i / 5.0)
        grid_lines.append(
            f'<line x1="{left}" y1="{y:.2f}" x2="{left + plot_w}" y2="{y:.2f}" stroke="#d8d8d8" stroke-width="1"/>'
        )

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="white"/>
  <text x="{width / 2:.0f}" y="36" text-anchor="middle" font-family="monospace" font-size="24">Prediction Variant Comparison</text>
  <text x="{width / 2:.0f}" y="{height - 20}" text-anchor="middle" font-family="monospace" font-size="18">Time (s)</text>
  <text x="30" y="{height / 2:.0f}" text-anchor="middle" font-family="monospace" font-size="18" transform="rotate(-90 30 {height / 2:.0f})">Meters</text>
  {''.join(grid_lines)}
  <rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="none" stroke="black" stroke-width="2"/>
  <polyline fill="none" stroke="#1f77b4" stroke-width="1.5" points="{altitude_points}"/>
  <polyline fill="none" stroke="#9467bd" stroke-width="1.3" points="{logged_points}"/>
  <polyline fill="none" stroke="#d62728" stroke-width="1.3" points="{old_points}"/>
  <polyline fill="none" stroke="#ff7f0e" stroke-width="1.3" points="{low_points}"/>
  <polyline fill="none" stroke="#2ca02c" stroke-width="1.4" points="{mach_points}"/>
  <polyline fill="none" stroke="#17becf" stroke-width="1.6" points="{blended_points}"/>
  <polyline fill="none" stroke="#111111" stroke-width="1.2" stroke-dasharray="6,4" points="{actual_points}"/>
  <line x1="{left + 20}" y1="{top + 20}" x2="{left + 90}" y2="{top + 20}" stroke="#1f77b4" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 26}" font-family="monospace" font-size="16">Altitude</text>
  <line x1="{left + 20}" y1="{top + 48}" x2="{left + 90}" y2="{top + 48}" stroke="#9467bd" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 54}" font-family="monospace" font-size="16">Logged Predictor</text>
  <line x1="{left + 20}" y1="{top + 76}" x2="{left + 90}" y2="{top + 76}" stroke="#d62728" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 82}" font-family="monospace" font-size="16">Old Model (Merged Seed)</text>
  <line x1="{left + 20}" y1="{top + 104}" x2="{left + 90}" y2="{top + 104}" stroke="#ff7f0e" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 110}" font-family="monospace" font-size="16">Low-Speed Only Adaptive</text>
  <line x1="{left + 20}" y1="{top + 132}" x2="{left + 90}" y2="{top + 132}" stroke="#2ca02c" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 138}" font-family="monospace" font-size="16">Mach-Based Adaptive</text>
  <line x1="{left + 20}" y1="{top + 160}" x2="{left + 90}" y2="{top + 160}" stroke="#17becf" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 166}" font-family="monospace" font-size="16">Near-Apogee Blend</text>
  <line x1="{left + 20}" y1="{top + 188}" x2="{left + 90}" y2="{top + 188}" stroke="#111111" stroke-width="2" stroke-dasharray="6,4"/>
  <text x="{left + 100}" y="{top + 194}" font-family="monospace" font-size="16">Actual Apogee</text>
</svg>
"""
    OUTPUT_COMPARE_SVG.write_text(svg)


def _write_control_svg(rows: list[dict[str, float]]) -> None:
    sampled = adaptive._decimate(rows, 6000)
    times = [float(row["time_s"]) for row in sampled]
    low_speed_scale = [float(row["low_speed_drag_scale"]) for row in sampled]
    mach_scale = [float(row["mach_drag_scale"]) for row in sampled]
    mach_weight = [float(row["mach_weight"]) for row in sampled]
    blend_weight = [float(row["blend_weight"]) for row in sampled]

    width = 1800
    height = 1000
    left = 110
    right = 40
    top = 70
    bottom_margin = 80
    plot_w = width - left - right
    plot_h = height - top - bottom_margin

    t_min = min(times)
    t_max = max(times)
    y_min = min(min(low_speed_scale), min(mach_scale), min(mach_weight), min(blend_weight))
    y_max = max(max(low_speed_scale), max(mach_scale), max(mach_weight), max(blend_weight))
    y_pad = max(0.1, (y_max - y_min) * 0.05)
    y_min -= y_pad
    y_max += y_pad

    low_scale_points = adaptive._polyline_points(times,
                                                 low_speed_scale,
                                                 t_min,
                                                 t_max,
                                                 y_min,
                                                 y_max,
                                                 left,
                                                 top,
                                                 plot_w,
                                                 plot_h)
    mach_scale_points = adaptive._polyline_points(times,
                                                  mach_scale,
                                                  t_min,
                                                  t_max,
                                                  y_min,
                                                  y_max,
                                                  left,
                                                  top,
                                                  plot_w,
                                                  plot_h)
    mach_weight_points = adaptive._polyline_points(times,
                                                   mach_weight,
                                                   t_min,
                                                   t_max,
                                                   y_min,
                                                   y_max,
                                                   left,
                                                   top,
                                                   plot_w,
                                                   plot_h)
    blend_weight_points = adaptive._polyline_points(times,
                                                    blend_weight,
                                                    t_min,
                                                    t_max,
                                                    y_min,
                                                    y_max,
                                                    left,
                                                    top,
                                                    plot_w,
                                                    plot_h)

    grid_lines = []
    for i in range(6):
        x = left + (plot_w * i / 5.0)
        grid_lines.append(
            f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{top + plot_h}" stroke="#d8d8d8" stroke-width="1"/>'
        )
    for i in range(6):
        y = top + (plot_h * i / 5.0)
        grid_lines.append(
            f'<line x1="{left}" y1="{y:.2f}" x2="{left + plot_w}" y2="{y:.2f}" stroke="#d8d8d8" stroke-width="1"/>'
        )

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="white"/>
  <text x="{width / 2:.0f}" y="36" text-anchor="middle" font-family="monospace" font-size="24">Prediction Variant Controls</text>
  <text x="{width / 2:.0f}" y="{height - 20}" text-anchor="middle" font-family="monospace" font-size="18">Time (s)</text>
  <text x="30" y="{height / 2:.0f}" text-anchor="middle" font-family="monospace" font-size="18" transform="rotate(-90 30 {height / 2:.0f})">Scale / Weight</text>
  {''.join(grid_lines)}
  <rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="none" stroke="black" stroke-width="2"/>
  <polyline fill="none" stroke="#ff7f0e" stroke-width="1.5" points="{low_scale_points}"/>
  <polyline fill="none" stroke="#2ca02c" stroke-width="1.5" points="{mach_scale_points}"/>
  <polyline fill="none" stroke="#1f77b4" stroke-width="1.4" points="{mach_weight_points}"/>
  <polyline fill="none" stroke="#17becf" stroke-width="1.4" points="{blend_weight_points}"/>
  <line x1="{left + 20}" y1="{top + 20}" x2="{left + 90}" y2="{top + 20}" stroke="#ff7f0e" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 26}" font-family="monospace" font-size="16">Low-Speed Drag Scale</text>
  <line x1="{left + 20}" y1="{top + 48}" x2="{left + 90}" y2="{top + 48}" stroke="#2ca02c" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 54}" font-family="monospace" font-size="16">Mach Drag Scale State</text>
  <line x1="{left + 20}" y1="{top + 76}" x2="{left + 90}" y2="{top + 76}" stroke="#1f77b4" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 82}" font-family="monospace" font-size="16">Mach Correction Weight</text>
  <line x1="{left + 20}" y1="{top + 104}" x2="{left + 90}" y2="{top + 104}" stroke="#17becf" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 110}" font-family="monospace" font-size="16">Near-Apogee Blend Weight</text>
</svg>
"""
    OUTPUT_CONTROL_SVG.write_text(svg)


def main() -> None:
    active_rows = adaptive._load_active_replay_rows(INPUT_CSV)
    merged_rows = adaptive._load_merged_seed_rows(MERGED_CSV)
    if len(active_rows) != len(merged_rows):
        raise SystemExit(f"Row mismatch: active={len(active_rows)} merged={len(merged_rows)}")

    force_table = adaptive.load_force_table(CFD_CSV)
    actual_apogee_m = adaptive._actual_apogee_from_output(INPUT_CSV)

    low_speed_drag_scale = 1.0
    mach_drag_scale = 1.0
    previous_time_s = active_rows[0].time_s
    output_rows: list[dict[str, float]] = []

    for active_row, merged_row in zip(active_rows, merged_rows):
        dt_seconds = active_row.time_s - previous_time_s
        seed_state = adaptive.ApogeeState(
            altitude_m=merged_row.altitude_m,
            horizontal_m=0.0,
            vertical_v=merged_row.velocity_mps,
            horizontal_v=merged_row.horizontal_speed_mps,
            zenith=merged_row.zenith_rad,
            angular_v=merged_row.angular_rate_rad_s,
            acs_deg=0.0,
        )

        valid_adaptation = (
            merged_row.status == "coast"
            and merged_row.velocity_mps > 0.0
            and adaptive._fresh_sample(dt_seconds)
            and math.isfinite(active_row.measured_accel_z_mps2)
            and math.isfinite(merged_row.zenith_rad)
        )

        low_speed_predicted_accel_z_mps2 = None
        low_speed_axial_accel_z_mps2 = None
        if valid_adaptation and merged_row.velocity_mps <= LOW_SPEED_SPLIT_MPS:
            low_speed_predicted_accel_z_mps2, _, _, low_speed_axial_accel_z_mps2 = adaptive._compute_accel_with_axial_scale(
                seed_state, force_table, low_speed_drag_scale
            )
            if abs(low_speed_axial_accel_z_mps2) >= MIN_AXIAL_ACCEL_ABS_MPS2:
                residual = active_row.measured_accel_z_mps2 - low_speed_predicted_accel_z_mps2
                target_scale = low_speed_drag_scale + RESIDUAL_GAIN * (
                    _clamp(residual, -RESIDUAL_CLAMP_MPS2, RESIDUAL_CLAMP_MPS2) / low_speed_axial_accel_z_mps2
                )
                target_scale = _clamp(target_scale, LOW_SPEED_MIN_SCALE, LOW_SPEED_MAX_SCALE)
                alpha = 1.0 - math.exp(-dt_seconds / LOW_SPEED_TAU_S) if dt_seconds > 0.0 else 0.0
                low_speed_drag_scale = _clamp(
                    low_speed_drag_scale + alpha * (target_scale - low_speed_drag_scale),
                    LOW_SPEED_MIN_SCALE,
                    LOW_SPEED_MAX_SCALE,
                )

        mach = _compute_state_mach(seed_state)
        mach_weight = 1.0 - _smoothstep(MACH_BLEND_FULL, MACH_BLEND_START, mach)
        mach_effective_scale = 1.0 + mach_weight * (mach_drag_scale - 1.0)
        mach_predicted_accel_z_mps2, _, _, mach_axial_accel_z_mps2 = adaptive._compute_accel_with_axial_scale(
            seed_state, force_table, mach_effective_scale
        )
        if valid_adaptation and mach_weight > 1.0e-3 and abs(mach_axial_accel_z_mps2) >= MIN_AXIAL_ACCEL_ABS_MPS2:
            residual = active_row.measured_accel_z_mps2 - mach_predicted_accel_z_mps2
            effective_target = mach_effective_scale + RESIDUAL_GAIN * (
                _clamp(residual, -RESIDUAL_CLAMP_MPS2, RESIDUAL_CLAMP_MPS2) / mach_axial_accel_z_mps2
            )
            if mach_weight > 1.0e-6:
                target_state = 1.0 + (effective_target - 1.0) / mach_weight
            else:
                target_state = mach_drag_scale
            target_state = _clamp(target_state, MACH_MIN_SCALE, MACH_MAX_SCALE)
            alpha = 1.0 - math.exp(-dt_seconds / MACH_TAU_S) if dt_seconds > 0.0 else 0.0
            mach_drag_scale = _clamp(
                mach_drag_scale + alpha * (target_state - mach_drag_scale),
                MACH_MIN_SCALE,
                MACH_MAX_SCALE,
            )
            mach_effective_scale = 1.0 + mach_weight * (mach_drag_scale - 1.0)

        low_speed_apogee_m = _predict_with_scale(
            seed_state,
            force_table,
            low_speed_drag_scale if merged_row.velocity_mps <= LOW_SPEED_SPLIT_MPS else 1.0,
        )
        mach_adaptive_apogee_m = _predict_with_mach_blend(seed_state, force_table, mach_drag_scale)

        blend_weight = 1.0 - _smoothstep(PREDICTOR_BLEND_FULL_VZ_MPS, PREDICTOR_BLEND_START_VZ_MPS, merged_row.velocity_mps)
        blended_apogee_m = (1.0 - blend_weight) * mach_adaptive_apogee_m + blend_weight * active_row.logged_apogee_m

        output_rows.append(
            {
                "time_s": active_row.time_s,
                "altitude_m": active_row.altitude_m,
                "velocity_mps": active_row.velocity_mps,
                "logged_apogee_m": active_row.logged_apogee_m,
                "merged_apogee_m": merged_row.merged_apogee_m,
                "low_speed_apogee_m": low_speed_apogee_m,
                "mach_adaptive_apogee_m": mach_adaptive_apogee_m,
                "blended_apogee_m": blended_apogee_m,
                "low_speed_drag_scale": low_speed_drag_scale,
                "mach_drag_scale": mach_drag_scale,
                "mach_weight": mach_weight,
                "blend_weight": blend_weight,
            }
        )
        previous_time_s = active_row.time_s

    with OUTPUT_CSV.open("w", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "time_s",
                "altitude_m",
                "velocity_mps",
                "logged_apogee_m",
                "merged_apogee_m",
                "low_speed_apogee_m",
                "mach_adaptive_apogee_m",
                "blended_apogee_m",
                "low_speed_drag_scale",
                "mach_drag_scale",
                "mach_weight",
                "blend_weight",
            ],
        )
        writer.writeheader()
        writer.writerows(output_rows)

    print(f"Actual apogee: {actual_apogee_m:.2f} m")
    print(
        "Variant settings: "
        f"low_speed_split={LOW_SPEED_SPLIT_MPS:.1f}m/s "
        f"mach_window=[{MACH_BLEND_START:.2f}->{MACH_BLEND_FULL:.2f}] "
        f"blend_vz=[{PREDICTOR_BLEND_START_VZ_MPS:.1f}->{PREDICTOR_BLEND_FULL_VZ_MPS:.1f}]m/s"
    )
    _print_metrics("logged", output_rows, "logged_apogee_m", actual_apogee_m)
    _print_metrics("old_model", output_rows, "merged_apogee_m", actual_apogee_m)
    _print_metrics("low_speed_only", output_rows, "low_speed_apogee_m", actual_apogee_m)
    _print_metrics("mach_adaptive", output_rows, "mach_adaptive_apogee_m", actual_apogee_m)
    _print_metrics("blended", output_rows, "blended_apogee_m", actual_apogee_m)

    _write_compare_svg(output_rows, actual_apogee_m)
    _write_control_svg(output_rows)
    print(f"Wrote {OUTPUT_CSV}")
    print(f"Wrote {OUTPUT_COMPARE_SVG}")
    print(f"Wrote {OUTPUT_CONTROL_SVG}")


if __name__ == "__main__":
    main()
