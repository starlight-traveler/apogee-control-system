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
SINGLE_SCALE_CSV = adaptive.OUTPUT_CSV
CFD_CSV = adaptive.CFD_CSV

OUTPUT_CSV = DATA_DIR / "output_adaptive_drag_banded_prediction.csv"
OUTPUT_COMPARE_SVG = PLOTS_DIR / "adaptive_drag_banded_comparison.svg"
OUTPUT_SCALE_SVG = PLOTS_DIR / "adaptive_drag_banded_scale.svg"


VELOCITY_SPLIT_MPS = 100.0
HIGH_BAND_MIN_SCALE = 0.78
HIGH_BAND_MAX_SCALE = 1.05
LOW_BAND_MIN_SCALE = 1.00
LOW_BAND_MAX_SCALE = 1.50
ADAPTATION_TAU_S = 0.20
RESIDUAL_CLAMP_MPS2 = 6.0
MIN_AXIAL_ACCEL_ABS_MPS2 = 0.75
RESIDUAL_GAIN = 1.0
COAST_DELAY_S = 0.0


def _load_single_scale_rows(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rows.append(
                {
                    "time_s": float(row["time_s"]),
                    "adaptive_apogee_m": float(row["adaptive_apogee_m"]),
                    "drag_scale": float(row["drag_scale"]),
                }
            )
    return rows


def _select_drag_scale(vertical_velocity_mps: float, high_band_scale: float, low_band_scale: float) -> float:
    return high_band_scale if vertical_velocity_mps > VELOCITY_SPLIT_MPS else low_band_scale


def _compute_accel_with_banded_scale(
    state: adaptive.ApogeeState,
    table: adaptive.ForceTable,
    high_band_scale: float,
    low_band_scale: float,
) -> tuple[float, float, float, float, float]:
    selected_scale = _select_drag_scale(state.vertical_v, high_band_scale, low_band_scale)
    linear_x, linear_y, angular, axial_linear_x = adaptive._compute_accel_with_axial_scale(state, table, selected_scale)
    return linear_x, linear_y, angular, axial_linear_x, selected_scale


def _evaluate_derivative_with_banded_scale(
    state: adaptive.ApogeeState,
    table: adaptive.ForceTable,
    high_band_scale: float,
    low_band_scale: float,
):
    ax, ay, angular, _, _ = _compute_accel_with_banded_scale(state, table, high_band_scale, low_band_scale)
    return (state.vertical_v, state.horizontal_v, ax, ay, state.angular_v, angular)


def _apply_step(state: adaptive.ApogeeState, deriv, dt: float) -> adaptive.ApogeeState:
    return adaptive.ApogeeState(
        altitude_m=state.altitude_m + deriv[0] * dt,
        horizontal_m=state.horizontal_m + deriv[1] * dt,
        vertical_v=state.vertical_v + deriv[2] * dt,
        horizontal_v=state.horizontal_v + deriv[3] * dt,
        zenith=state.zenith + deriv[4] * dt,
        angular_v=state.angular_v + deriv[5] * dt,
        acs_deg=state.acs_deg,
    )


def _integrate_rk4(state: adaptive.ApogeeState, k1, k2, k3, k4) -> None:
    sixth = adaptive.PRED_DT / 6.0
    state.altitude_m += sixth * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0])
    state.horizontal_m += sixth * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1])
    state.vertical_v += sixth * (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2])
    state.horizontal_v += sixth * (k1[3] + 2.0 * k2[3] + 2.0 * k3[3] + k4[3])
    state.zenith += sixth * (k1[4] + 2.0 * k2[4] + 2.0 * k3[4] + k4[4])
    state.angular_v += sixth * (k1[5] + 2.0 * k2[5] + 2.0 * k3[5] + k4[5])


def _predict_apogee_with_banded_scale(
    state: adaptive.ApogeeState,
    table: adaptive.ForceTable,
    high_band_scale: float,
    low_band_scale: float,
) -> float:
    if state.vertical_v <= 0.0:
        return state.altitude_m
    current = adaptive.ApogeeState(**state.__dict__)
    steps = 0
    while current.vertical_v > 0.0 and steps < adaptive.MAX_STEPS:
        k1 = _evaluate_derivative_with_banded_scale(current, table, high_band_scale, low_band_scale)
        k2 = _evaluate_derivative_with_banded_scale(_apply_step(current, k1, 0.5 * adaptive.PRED_DT),
                                                    table,
                                                    high_band_scale,
                                                    low_band_scale)
        k3 = _evaluate_derivative_with_banded_scale(_apply_step(current, k2, 0.5 * adaptive.PRED_DT),
                                                    table,
                                                    high_band_scale,
                                                    low_band_scale)
        k4 = _evaluate_derivative_with_banded_scale(_apply_step(current, k3, adaptive.PRED_DT),
                                                    table,
                                                    high_band_scale,
                                                    low_band_scale)
        _integrate_rk4(current, k1, k2, k3, k4)
        steps += 1
    return current.altitude_m


def _write_comparison_svg(rows: list[dict[str, float]], actual_apogee_m: float) -> None:
    sampled = adaptive._decimate(rows, 6000)
    times = [float(row["time_s"]) for row in sampled]
    altitude = [float(row["altitude_m"]) for row in sampled]
    logged = [float(row["logged_apogee_m"]) for row in sampled]
    old_model = [float(row["merged_apogee_m"]) for row in sampled]
    single_scale = [float(row["single_scale_apogee_m"]) for row in sampled]
    banded = [float(row["banded_apogee_m"]) for row in sampled]
    actual = [actual_apogee_m for _ in sampled]

    width = 1750
    height = 980
    left = 110
    right = 40
    top = 70
    bottom_margin = 80
    plot_w = width - left - right
    plot_h = height - top - bottom_margin

    t_min = min(times)
    t_max = max(times)
    y_min = min(min(altitude), min(logged), min(old_model), min(single_scale), min(banded), actual_apogee_m)
    y_max = max(max(altitude), max(logged), max(old_model), max(single_scale), max(banded), actual_apogee_m)
    y_pad = max(1.0, (y_max - y_min) * 0.05)
    y_min -= y_pad
    y_max += y_pad

    altitude_points = adaptive._polyline_points(times, altitude, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    logged_points = adaptive._polyline_points(times, logged, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    old_points = adaptive._polyline_points(times, old_model, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    single_points = adaptive._polyline_points(times,
                                              single_scale,
                                              t_min,
                                              t_max,
                                              y_min,
                                              y_max,
                                              left,
                                              top,
                                              plot_w,
                                              plot_h)
    banded_points = adaptive._polyline_points(times, banded, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
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
  <text x="{width / 2:.0f}" y="36" text-anchor="middle" font-family="monospace" font-size="24">Banded Adaptive Drag Predictor Comparison</text>
  <text x="{width / 2:.0f}" y="{height - 20}" text-anchor="middle" font-family="monospace" font-size="18">Time (s)</text>
  <text x="30" y="{height / 2:.0f}" text-anchor="middle" font-family="monospace" font-size="18" transform="rotate(-90 30 {height / 2:.0f})">Meters</text>
  {''.join(grid_lines)}
  <rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="none" stroke="black" stroke-width="2"/>
  <polyline fill="none" stroke="#1f77b4" stroke-width="1.5" points="{altitude_points}"/>
  <polyline fill="none" stroke="#9467bd" stroke-width="1.4" points="{logged_points}"/>
  <polyline fill="none" stroke="#d62728" stroke-width="1.4" points="{old_points}"/>
  <polyline fill="none" stroke="#ff7f0e" stroke-width="1.4" points="{single_points}"/>
  <polyline fill="none" stroke="#2ca02c" stroke-width="1.6" points="{banded_points}"/>
  <polyline fill="none" stroke="#111111" stroke-width="1.2" stroke-dasharray="6,4" points="{actual_points}"/>
  <line x1="{left + 20}" y1="{top + 20}" x2="{left + 90}" y2="{top + 20}" stroke="#1f77b4" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 26}" font-family="monospace" font-size="16">Altitude</text>
  <line x1="{left + 20}" y1="{top + 48}" x2="{left + 90}" y2="{top + 48}" stroke="#9467bd" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 54}" font-family="monospace" font-size="16">Logged Predictor</text>
  <line x1="{left + 20}" y1="{top + 76}" x2="{left + 90}" y2="{top + 76}" stroke="#d62728" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 82}" font-family="monospace" font-size="16">Old Model (Merged Seed)</text>
  <line x1="{left + 20}" y1="{top + 104}" x2="{left + 90}" y2="{top + 104}" stroke="#ff7f0e" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 110}" font-family="monospace" font-size="16">Single-Scale Adaptive</text>
  <line x1="{left + 20}" y1="{top + 132}" x2="{left + 90}" y2="{top + 132}" stroke="#2ca02c" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 138}" font-family="monospace" font-size="16">Banded Adaptive</text>
  <line x1="{left + 20}" y1="{top + 160}" x2="{left + 90}" y2="{top + 160}" stroke="#111111" stroke-width="2" stroke-dasharray="6,4"/>
  <text x="{left + 100}" y="{top + 166}" font-family="monospace" font-size="16">Actual Apogee</text>
</svg>
"""
    OUTPUT_COMPARE_SVG.write_text(svg)


def _write_scale_svg(rows: list[dict[str, float]]) -> None:
    sampled = adaptive._decimate(rows, 6000)
    times = [float(row["time_s"]) for row in sampled]
    high_scale = [float(row["high_band_scale"]) for row in sampled]
    low_scale = [float(row["low_band_scale"]) for row in sampled]
    active_scale = [float(row["selected_drag_scale"]) for row in sampled]
    measured_accel = [float(row["measured_accel_z_mps2"]) for row in sampled]
    predicted_accel = [float(row["predicted_accel_z_mps2"]) for row in sampled]

    width = 1750
    height = 980
    left = 110
    right = 40
    top = 70
    bottom_margin = 80
    plot_w = width - left - right
    plot_h = height - top - bottom_margin

    t_min = min(times)
    t_max = max(times)
    y_min = min(min(high_scale), min(low_scale), min(active_scale), min(measured_accel), min(predicted_accel))
    y_max = max(max(high_scale), max(low_scale), max(active_scale), max(measured_accel), max(predicted_accel))
    y_pad = max(1.0, (y_max - y_min) * 0.05)
    y_min -= y_pad
    y_max += y_pad

    high_points = adaptive._polyline_points(times, high_scale, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    low_points = adaptive._polyline_points(times, low_scale, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    active_points = adaptive._polyline_points(times, active_scale, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    measured_points = adaptive._polyline_points(times,
                                                measured_accel,
                                                t_min,
                                                t_max,
                                                y_min,
                                                y_max,
                                                left,
                                                top,
                                                plot_w,
                                                plot_h)
    predicted_points = adaptive._polyline_points(times,
                                                 predicted_accel,
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
  <text x="{width / 2:.0f}" y="36" text-anchor="middle" font-family="monospace" font-size="24">Banded Adaptive Drag Scales</text>
  <text x="{width / 2:.0f}" y="{height - 20}" text-anchor="middle" font-family="monospace" font-size="18">Time (s)</text>
  <text x="30" y="{height / 2:.0f}" text-anchor="middle" font-family="monospace" font-size="18" transform="rotate(-90 30 {height / 2:.0f})">Scale / m/s²</text>
  {''.join(grid_lines)}
  <rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="none" stroke="black" stroke-width="2"/>
  <polyline fill="none" stroke="#2ca02c" stroke-width="1.5" points="{high_points}"/>
  <polyline fill="none" stroke="#ff7f0e" stroke-width="1.5" points="{low_points}"/>
  <polyline fill="none" stroke="#111111" stroke-width="1.3" stroke-dasharray="6,4" points="{active_points}"/>
  <polyline fill="none" stroke="#1f77b4" stroke-width="1.4" points="{measured_points}"/>
  <polyline fill="none" stroke="#d62728" stroke-width="1.4" points="{predicted_points}"/>
  <line x1="{left + 20}" y1="{top + 20}" x2="{left + 90}" y2="{top + 20}" stroke="#2ca02c" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 26}" font-family="monospace" font-size="16">High-Speed Drag Scale</text>
  <line x1="{left + 20}" y1="{top + 48}" x2="{left + 90}" y2="{top + 48}" stroke="#ff7f0e" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 54}" font-family="monospace" font-size="16">Low-Speed Drag Scale</text>
  <line x1="{left + 20}" y1="{top + 76}" x2="{left + 90}" y2="{top + 76}" stroke="#111111" stroke-width="2" stroke-dasharray="6,4"/>
  <text x="{left + 100}" y="{top + 82}" font-family="monospace" font-size="16">Selected Drag Scale</text>
  <line x1="{left + 20}" y1="{top + 104}" x2="{left + 90}" y2="{top + 104}" stroke="#1f77b4" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 110}" font-family="monospace" font-size="16">Measured Vertical Accel</text>
  <line x1="{left + 20}" y1="{top + 132}" x2="{left + 90}" y2="{top + 132}" stroke="#d62728" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 138}" font-family="monospace" font-size="16">Model Vertical Accel</text>
</svg>
"""
    OUTPUT_SCALE_SVG.write_text(svg)


def main() -> None:
    active_rows = adaptive._load_active_replay_rows(INPUT_CSV)
    merged_rows = adaptive._load_merged_seed_rows(MERGED_CSV)
    single_scale_rows = _load_single_scale_rows(SINGLE_SCALE_CSV)
    if not (len(active_rows) == len(merged_rows) == len(single_scale_rows)):
        raise SystemExit(
            f"Row count mismatch: active={len(active_rows)} merged={len(merged_rows)} single={len(single_scale_rows)}"
        )

    force_table = adaptive.load_force_table(CFD_CSV)
    actual_apogee_m = adaptive._actual_apogee_from_output(INPUT_CSV)

    high_band_scale = 1.0
    low_band_scale = 1.0
    previous_time_s = active_rows[0].time_s
    coast_start_time_s = None
    output_rows: list[dict[str, float]] = []

    for active_row, merged_row, single_scale_row in zip(active_rows, merged_rows, single_scale_rows):
        if abs(active_row.time_s - merged_row.time_s) > 1.0e-3:
            raise SystemExit(f"Replay alignment mismatch at {active_row.time_s:.6f}s")

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

        predicted_accel_z_mps2, _, _, axial_accel_z_mps2, selected_drag_scale = _compute_accel_with_banded_scale(
            seed_state, force_table, high_band_scale, low_band_scale
        )
        accel_residual_mps2 = active_row.measured_accel_z_mps2 - predicted_accel_z_mps2
        valid_adaptation = (
            merged_row.status == "coast"
            and merged_row.velocity_mps > 0.0
            and adaptive._fresh_sample(dt_seconds)
            and math.isfinite(active_row.measured_accel_z_mps2)
            and math.isfinite(merged_row.zenith_rad)
            and coast_start_time_s is not None
            and (active_row.time_s - coast_start_time_s) >= COAST_DELAY_S
            and abs(axial_accel_z_mps2) >= MIN_AXIAL_ACCEL_ABS_MPS2
        )
        if valid_adaptation:
            clamped_residual = adaptive._clamp(accel_residual_mps2, -RESIDUAL_CLAMP_MPS2, RESIDUAL_CLAMP_MPS2)
            target_scale = selected_drag_scale + RESIDUAL_GAIN * (clamped_residual / axial_accel_z_mps2)
            alpha = 1.0 - math.exp(-dt_seconds / ADAPTATION_TAU_S) if dt_seconds > 0.0 else 0.0
            if merged_row.velocity_mps > VELOCITY_SPLIT_MPS:
                target_scale = adaptive._clamp(target_scale, HIGH_BAND_MIN_SCALE, HIGH_BAND_MAX_SCALE)
                high_band_scale = adaptive._clamp(high_band_scale + alpha * (target_scale - high_band_scale),
                                                  HIGH_BAND_MIN_SCALE,
                                                  HIGH_BAND_MAX_SCALE)
            else:
                target_scale = adaptive._clamp(target_scale, LOW_BAND_MIN_SCALE, LOW_BAND_MAX_SCALE)
                low_band_scale = adaptive._clamp(low_band_scale + alpha * (target_scale - low_band_scale),
                                                 LOW_BAND_MIN_SCALE,
                                                 LOW_BAND_MAX_SCALE)

        banded_apogee_m = _predict_apogee_with_banded_scale(seed_state, force_table, high_band_scale, low_band_scale)
        output_rows.append(
            {
                "time_s": active_row.time_s,
                "altitude_m": active_row.altitude_m,
                "velocity_mps": active_row.velocity_mps,
                "logged_apogee_m": active_row.logged_apogee_m,
                "merged_apogee_m": merged_row.merged_apogee_m,
                "single_scale_apogee_m": single_scale_row["adaptive_apogee_m"],
                "banded_apogee_m": banded_apogee_m,
                "high_band_scale": high_band_scale,
                "low_band_scale": low_band_scale,
                "selected_drag_scale": _select_drag_scale(merged_row.velocity_mps, high_band_scale, low_band_scale),
                "predicted_accel_z_mps2": predicted_accel_z_mps2,
                "measured_accel_z_mps2": active_row.measured_accel_z_mps2,
                "accel_residual_mps2": accel_residual_mps2,
                "status": active_row.status,
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
                "single_scale_apogee_m",
                "banded_apogee_m",
                "high_band_scale",
                "low_band_scale",
                "selected_drag_scale",
                "predicted_accel_z_mps2",
                "measured_accel_z_mps2",
                "accel_residual_mps2",
                "status",
            ],
        )
        writer.writeheader()
        writer.writerows(output_rows)

    print(f"Actual apogee: {actual_apogee_m:.2f} m")
    print(
        "Banded settings: "
        f"split={VELOCITY_SPLIT_MPS:.1f}m/s "
        f"high=[{HIGH_BAND_MIN_SCALE:.2f}, {HIGH_BAND_MAX_SCALE:.2f}] "
        f"low=[{LOW_BAND_MIN_SCALE:.2f}, {LOW_BAND_MAX_SCALE:.2f}] "
        f"tau={ADAPTATION_TAU_S:.2f}s clamp={RESIDUAL_CLAMP_MPS2:.1f} gain={RESIDUAL_GAIN:.2f}"
    )
    for label, key in (
        ("logged", "logged_apogee_m"),
        ("old_model", "merged_apogee_m"),
        ("single_scale", "single_scale_apogee_m"),
        ("banded", "banded_apogee_m"),
    ):
        print(label)
        for threshold_m in (25.0, 10.0, 5.0):
            hit = adaptive._first_within(output_rows, key, actual_apogee_m, threshold_m)
            if hit is None:
                print(f"  within_{int(threshold_m)}m: none")
            else:
                print(
                    f"  within_{int(threshold_m)}m: "
                    f"t={float(hit['time_s']):.3f}s alt={float(hit['altitude_m']):.1f}m pred={float(hit[key]):.1f}m"
                )
        stats = adaptive._post_lock_metrics(output_rows, key, actual_apogee_m, 25.0)
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

    _write_comparison_svg(output_rows, actual_apogee_m)
    _write_scale_svg(output_rows)
    print(f"Wrote {OUTPUT_CSV}")
    print(f"Wrote {OUTPUT_COMPARE_SVG}")
    print(f"Wrote {OUTPUT_SCALE_SVG}")


if __name__ == "__main__":
    main()
