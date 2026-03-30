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

OUTPUT_CSV = DATA_DIR / "output_burnout_bridge_prediction.csv"
OUTPUT_COMPARE_SVG = PLOTS_DIR / "burnout_bridge_comparison.svg"
OUTPUT_CONTROL_SVG = PLOTS_DIR / "burnout_bridge_controls.svg"


# Burnout-bridge experiment tuning.
# The idea is to start predicting a little earlier by carrying a short,
# decaying positive burn acceleration into the predictor during late burn.
BURN_BRIDGE_ENABLE_ACCEL_MPS2 = 8.0
BURNOUT_ACCEL_THRESHOLD_MPS2 = 2.0
BURN_BRIDGE_DECAY_RATE_MPS3 = 20.0
BURN_BRIDGE_MAX_TIME_S = 0.35
BURN_BRIDGE_STEP_S = 0.02
BURN_BRIDGE_EXCESS_CLAMP_MPS2 = 40.0
MAX_BRIDGE_ZENITH_DEG = 20.0

ADAPTIVE_DRAG_SCALE_MIN = 1.0
ADAPTIVE_DRAG_SCALE_MAX = 1.35
ADAPTIVE_RESIDUAL_CLAMP_MPS2 = 8.0
ADAPTIVE_TAU_S = 0.45
ADAPTIVE_MIN_AXIAL_ACCEL_ABS_MPS2 = 0.75


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def _estimate_bridge_duration_s(measured_accel_z_mps2: float) -> float:
    if measured_accel_z_mps2 <= BURNOUT_ACCEL_THRESHOLD_MPS2:
        return 0.0
    if measured_accel_z_mps2 > BURN_BRIDGE_ENABLE_ACCEL_MPS2:
        return 0.0
    return _clamp(
        (measured_accel_z_mps2 - BURNOUT_ACCEL_THRESHOLD_MPS2) / BURN_BRIDGE_DECAY_RATE_MPS3,
        0.0,
        BURN_BRIDGE_MAX_TIME_S,
    )


def _advance_burn_bridge(
    seed_state: adaptive.ApogeeState,
    force_table: adaptive.ForceTable,
    drag_scale: float,
    measured_accel_z_mps2: float,
) -> tuple[adaptive.ApogeeState, float, float]:
    duration_s = _estimate_bridge_duration_s(measured_accel_z_mps2)
    if duration_s <= 0.0:
        return adaptive.ApogeeState(**seed_state.__dict__), 0.0, 0.0

    initial_predicted_accel_z_mps2, _, _, _ = adaptive._compute_accel_with_axial_scale(
        seed_state, force_table, drag_scale
    )
    initial_excess_accel_z_mps2 = _clamp(
        measured_accel_z_mps2 - initial_predicted_accel_z_mps2,
        0.0,
        BURN_BRIDGE_EXCESS_CLAMP_MPS2,
    )
    if initial_excess_accel_z_mps2 <= 0.0:
        return adaptive.ApogeeState(**seed_state.__dict__), 0.0, 0.0

    current = adaptive.ApogeeState(**seed_state.__dict__)
    elapsed_s = 0.0
    clamped_zenith = _clamp(
        current.zenith,
        -math.radians(MAX_BRIDGE_ZENITH_DEG),
        math.radians(MAX_BRIDGE_ZENITH_DEG),
    )
    tan_zenith = math.tan(clamped_zenith)

    while elapsed_s < duration_s:
        dt_s = min(BURN_BRIDGE_STEP_S, duration_s - elapsed_s)
        if dt_s <= 0.0:
            break

        base_ax, base_ay, base_ang, _ = adaptive._compute_accel_with_axial_scale(current, force_table, drag_scale)
        remaining_fraction = max(0.0, 1.0 - (elapsed_s / duration_s))
        vertical_boost = initial_excess_accel_z_mps2 * remaining_fraction
        horizontal_boost = vertical_boost * tan_zenith
        derivative = (
            current.vertical_v,
            current.horizontal_v,
            base_ax + vertical_boost,
            base_ay + horizontal_boost,
            current.angular_v,
            base_ang,
        )
        current = adaptive._apply_step(current, derivative, dt_s)
        elapsed_s += dt_s

    return current, duration_s, initial_excess_accel_z_mps2


def _predict_with_burn_bridge(
    seed_state: adaptive.ApogeeState,
    force_table: adaptive.ForceTable,
    drag_scale: float,
    measured_accel_z_mps2: float,
) -> tuple[float, float, float]:
    bridged_state, bridge_duration_s, initial_excess_accel_z_mps2 = _advance_burn_bridge(
        seed_state,
        force_table,
        drag_scale,
        measured_accel_z_mps2,
    )
    prediction_m = adaptive._predict_apogee_with_axial_scale(bridged_state, force_table, drag_scale)
    return prediction_m, bridge_duration_s, initial_excess_accel_z_mps2


def _print_metrics(label: str, rows: list[dict[str, float | str]], key: str, actual_apogee_m: float) -> None:
    print(label)
    for threshold_m in (100.0, 50.0, 25.0, 10.0, 5.0):
        hit = adaptive._first_within(rows, key, actual_apogee_m, threshold_m)
        if hit is None:
            print(f"  within_{int(threshold_m)}m: none")
        else:
            print(
                f"  within_{int(threshold_m)}m: "
                f"t={float(hit['time_s']):.3f}s alt={float(hit['altitude_m']):.1f}m "
                f"vz={float(hit['velocity_mps']):.1f}m/s pred={float(hit[key]):.1f}m "
                f"status={hit['status']}"
            )
    stats = adaptive._post_lock_metrics(rows, key, actual_apogee_m, 25.0)
    if stats is None:
        print("  post_lock_25m: none")
    else:
        print(
            "  post_lock_25m: "
            f"start={stats['start_time_s']:.3f}s/{stats['start_altitude_m']:.1f}m "
            f"mean_err={stats['mean_error_m']:.1f}m "
            f"max_over={stats['max_overshoot_m']:.1f}m "
            f"max_under={stats['max_undershoot_m']:.1f}m "
            f"time_over={stats['overshoot_time_s']:.3f}s"
        )


def _write_compare_svg(rows: list[dict[str, float | str]], actual_apogee_m: float) -> None:
    sampled = adaptive._decimate(rows, 6000)
    times = [float(row["time_s"]) for row in sampled]
    altitude = [float(row["altitude_m"]) for row in sampled]
    logged = [float(row["logged_apogee_m"]) for row in sampled]
    current = [float(row["current_logic_apogee_m"]) for row in sampled]
    burnout_bridge = [float(row["burnout_bridge_apogee_m"]) for row in sampled]
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
    y_min = min(min(altitude), min(logged), min(current), min(burnout_bridge), actual_apogee_m)
    y_max = max(max(altitude), max(logged), max(current), max(burnout_bridge), actual_apogee_m)
    y_pad = max(1.0, (y_max - y_min) * 0.05)
    y_min -= y_pad
    y_max += y_pad

    coast_start_s = next((float(row["time_s"]) for row in rows if row["status"] == "coast"), None)
    altitude_points = adaptive._polyline_points(times, altitude, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    logged_points = adaptive._polyline_points(times, logged, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    current_points = adaptive._polyline_points(times, current, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    burnout_bridge_points = adaptive._polyline_points(
        times, burnout_bridge, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h
    )
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

    coast_marker = ""
    if coast_start_s is not None:
        coast_x = adaptive._scale(coast_start_s, t_min, t_max, left, left + plot_w)
        coast_marker = (
            f'<line x1="{coast_x:.2f}" y1="{top}" x2="{coast_x:.2f}" y2="{top + plot_h}" '
            f'stroke="#555555" stroke-width="1.4" stroke-dasharray="7,5"/>'
            f'<text x="{coast_x + 8:.2f}" y="{top + 24}" font-family="monospace" font-size="14" fill="#333333">coast start</text>'
        )

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="white"/>
  <text x="{width / 2:.0f}" y="36" text-anchor="middle" font-family="monospace" font-size="24">Burnout Bridge Predictor Comparison</text>
  <text x="{width / 2:.0f}" y="{height - 20}" text-anchor="middle" font-family="monospace" font-size="18">Time (s)</text>
  <text x="30" y="{height / 2:.0f}" text-anchor="middle" font-family="monospace" font-size="18" transform="rotate(-90 30 {height / 2:.0f})">Meters</text>
  {''.join(grid_lines)}
  <rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="none" stroke="black" stroke-width="2"/>
  {coast_marker}
  <polyline fill="none" stroke="#1f77b4" stroke-width="1.5" points="{altitude_points}"/>
  <polyline fill="none" stroke="#9467bd" stroke-width="1.4" points="{logged_points}"/>
  <polyline fill="none" stroke="#d62728" stroke-width="1.4" points="{current_points}"/>
  <polyline fill="none" stroke="#2ca02c" stroke-width="1.5" points="{burnout_bridge_points}"/>
  <polyline fill="none" stroke="#111111" stroke-width="1.2" stroke-dasharray="6,4" points="{actual_points}"/>
  <line x1="{left + 20}" y1="{top + 20}" x2="{left + 90}" y2="{top + 20}" stroke="#1f77b4" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 26}" font-family="monospace" font-size="16">Altitude</text>
  <line x1="{left + 20}" y1="{top + 48}" x2="{left + 90}" y2="{top + 48}" stroke="#9467bd" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 54}" font-family="monospace" font-size="16">Logged Predictor</text>
  <line x1="{left + 20}" y1="{top + 76}" x2="{left + 90}" y2="{top + 76}" stroke="#d62728" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 82}" font-family="monospace" font-size="16">Current Logic Recompute</text>
  <line x1="{left + 20}" y1="{top + 104}" x2="{left + 90}" y2="{top + 104}" stroke="#2ca02c" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 110}" font-family="monospace" font-size="16">Burnout Bridge Variant</text>
  <line x1="{left + 20}" y1="{top + 132}" x2="{left + 90}" y2="{top + 132}" stroke="#111111" stroke-width="2" stroke-dasharray="6,4"/>
  <text x="{left + 100}" y="{top + 138}" font-family="monospace" font-size="16">Actual Apogee</text>
</svg>
"""
    OUTPUT_COMPARE_SVG.write_text(svg)


def _write_control_svg(rows: list[dict[str, float | str]]) -> None:
    sampled = adaptive._decimate(rows, 6000)
    times = [float(row["time_s"]) for row in sampled]
    drag_scale = [float(row["drag_scale"]) for row in sampled]
    measured_accel = [float(row["measured_accel_z_mps2"]) for row in sampled]
    predicted_accel = [float(row["predicted_accel_z_mps2"]) for row in sampled]
    bridge_duration = [float(row["burnout_bridge_duration_s"]) for row in sampled]
    bridge_excess = [float(row["burnout_bridge_initial_excess_accel_z_mps2"]) for row in sampled]

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
    y_min = min(min(drag_scale), min(measured_accel), min(predicted_accel), min(bridge_duration), min(bridge_excess))
    y_max = max(max(drag_scale), max(measured_accel), max(predicted_accel), max(bridge_duration), max(bridge_excess))
    y_pad = max(1.0, (y_max - y_min) * 0.05)
    y_min -= y_pad
    y_max += y_pad

    drag_points = adaptive._polyline_points(times, drag_scale, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    measured_points = adaptive._polyline_points(
        times, measured_accel, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h
    )
    predicted_points = adaptive._polyline_points(
        times, predicted_accel, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h
    )
    bridge_duration_points = adaptive._polyline_points(
        times, bridge_duration, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h
    )
    bridge_excess_points = adaptive._polyline_points(
        times, bridge_excess, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h
    )

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
  <text x="{width / 2:.0f}" y="36" text-anchor="middle" font-family="monospace" font-size="24">Burnout Bridge Controls</text>
  <text x="{width / 2:.0f}" y="{height - 20}" text-anchor="middle" font-family="monospace" font-size="18">Time (s)</text>
  <text x="30" y="{height / 2:.0f}" text-anchor="middle" font-family="monospace" font-size="18" transform="rotate(-90 30 {height / 2:.0f})">Scale / Accel / Bridge</text>
  {''.join(grid_lines)}
  <rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="none" stroke="black" stroke-width="2"/>
  <polyline fill="none" stroke="#2ca02c" stroke-width="1.5" points="{drag_points}"/>
  <polyline fill="none" stroke="#1f77b4" stroke-width="1.4" points="{measured_points}"/>
  <polyline fill="none" stroke="#d62728" stroke-width="1.4" points="{predicted_points}"/>
  <polyline fill="none" stroke="#9467bd" stroke-width="1.4" points="{bridge_duration_points}"/>
  <polyline fill="none" stroke="#ff7f0e" stroke-width="1.4" points="{bridge_excess_points}"/>
  <line x1="{left + 20}" y1="{top + 20}" x2="{left + 90}" y2="{top + 20}" stroke="#2ca02c" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 26}" font-family="monospace" font-size="16">Adaptive Drag Scale</text>
  <line x1="{left + 20}" y1="{top + 48}" x2="{left + 90}" y2="{top + 48}" stroke="#1f77b4" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 54}" font-family="monospace" font-size="16">Measured Vertical Accel</text>
  <line x1="{left + 20}" y1="{top + 76}" x2="{left + 90}" y2="{top + 76}" stroke="#d62728" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 82}" font-family="monospace" font-size="16">Current Model Vertical Accel</text>
  <line x1="{left + 20}" y1="{top + 104}" x2="{left + 90}" y2="{top + 104}" stroke="#9467bd" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 110}" font-family="monospace" font-size="16">Burnout Bridge Duration [s]</text>
  <line x1="{left + 20}" y1="{top + 132}" x2="{left + 90}" y2="{top + 132}" stroke="#ff7f0e" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 138}" font-family="monospace" font-size="16">Initial Burnout Excess Accel</text>
</svg>
"""
    OUTPUT_CONTROL_SVG.write_text(svg)


def main() -> None:
    active_rows = adaptive._load_active_replay_rows(INPUT_CSV)
    merged_rows = adaptive._load_merged_seed_rows(MERGED_CSV)
    if len(active_rows) != len(merged_rows):
        raise SystemExit(
            f"Replay row count mismatch: active={len(active_rows)} merged={len(merged_rows)}. "
            "Regenerate the merged-seed CSV first."
        )

    force_table = adaptive.load_force_table(CFD_CSV)
    actual_apogee_m = adaptive._actual_apogee_from_output(INPUT_CSV)

    drag_scale = 1.0
    previous_time_s = active_rows[0].time_s
    output_rows: list[dict[str, float | str]] = []

    for active_row, merged_row in zip(active_rows, merged_rows):
        if abs(active_row.time_s - merged_row.time_s) > 1.0e-3:
            raise SystemExit(
                f"Replay alignment mismatch at t={active_row.time_s:.6f}s vs {merged_row.time_s:.6f}s"
            )

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
        )
        if valid_adaptation and abs(axial_accel_z_mps2) >= ADAPTIVE_MIN_AXIAL_ACCEL_ABS_MPS2:
            clamped_residual = adaptive._clamp(
                accel_residual_mps2,
                -ADAPTIVE_RESIDUAL_CLAMP_MPS2,
                ADAPTIVE_RESIDUAL_CLAMP_MPS2,
            )
            target_scale = drag_scale + (clamped_residual / axial_accel_z_mps2)
            target_scale = adaptive._clamp(target_scale, ADAPTIVE_DRAG_SCALE_MIN, ADAPTIVE_DRAG_SCALE_MAX)
            alpha = 1.0 - math.exp(-dt_seconds / ADAPTIVE_TAU_S) if dt_seconds > 0.0 else 0.0
            drag_scale = adaptive._clamp(
                drag_scale + alpha * (target_scale - drag_scale),
                ADAPTIVE_DRAG_SCALE_MIN,
                ADAPTIVE_DRAG_SCALE_MAX,
            )

        current_logic_apogee_m = adaptive._predict_apogee_with_axial_scale(seed_state, force_table, drag_scale)
        burnout_bridge_apogee_m = current_logic_apogee_m
        burnout_bridge_duration_s = 0.0
        burnout_bridge_initial_excess_accel_z_mps2 = 0.0
        if merged_row.status == "burn":
            burnout_bridge_apogee_m, burnout_bridge_duration_s, burnout_bridge_initial_excess_accel_z_mps2 = (
                _predict_with_burn_bridge(
                    seed_state,
                    force_table,
                    drag_scale,
                    active_row.measured_accel_z_mps2,
                )
            )

        output_rows.append(
            {
                "time_s": active_row.time_s,
                "altitude_m": active_row.altitude_m,
                "velocity_mps": active_row.velocity_mps,
                "logged_apogee_m": active_row.logged_apogee_m,
                "current_logic_apogee_m": current_logic_apogee_m,
                "burnout_bridge_apogee_m": burnout_bridge_apogee_m,
                "drag_scale": drag_scale,
                "predicted_accel_z_mps2": predicted_accel_z_mps2,
                "measured_accel_z_mps2": active_row.measured_accel_z_mps2,
                "accel_residual_mps2": accel_residual_mps2,
                "axial_accel_z_mps2": axial_accel_z_mps2,
                "burnout_bridge_duration_s": burnout_bridge_duration_s,
                "burnout_bridge_initial_excess_accel_z_mps2": burnout_bridge_initial_excess_accel_z_mps2,
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
                "current_logic_apogee_m",
                "burnout_bridge_apogee_m",
                "drag_scale",
                "predicted_accel_z_mps2",
                "measured_accel_z_mps2",
                "accel_residual_mps2",
                "axial_accel_z_mps2",
                "burnout_bridge_duration_s",
                "burnout_bridge_initial_excess_accel_z_mps2",
                "status",
            ],
        )
        writer.writeheader()
        writer.writerows(output_rows)

    print(f"Actual apogee: {actual_apogee_m:.2f} m")
    print(
        "Burnout bridge settings: "
        f"enable_accel<={BURN_BRIDGE_ENABLE_ACCEL_MPS2:.1f}m/s^2 "
        f"burnout_accel={BURNOUT_ACCEL_THRESHOLD_MPS2:.1f}m/s^2 "
        f"decay={BURN_BRIDGE_DECAY_RATE_MPS3:.1f}m/s^3 "
        f"max_time={BURN_BRIDGE_MAX_TIME_S:.2f}s"
    )
    burn_rows = [row for row in output_rows if row["status"] == "burn"]
    active_bridge_rows = [row for row in burn_rows if float(row["burnout_bridge_duration_s"]) > 0.0]
    if active_bridge_rows:
        max_delta_row = max(
            active_bridge_rows,
            key=lambda row: float(row["burnout_bridge_apogee_m"]) - float(row["current_logic_apogee_m"]),
        )
        last_burn_row = burn_rows[-1]
        print(
            "burnout_bridge_activation: "
            f"rows={len(active_bridge_rows)} "
            f"window={float(active_bridge_rows[0]['time_s']):.3f}s..{float(active_bridge_rows[-1]['time_s']):.3f}s "
            f"max_delta={float(max_delta_row['burnout_bridge_apogee_m']) - float(max_delta_row['current_logic_apogee_m']):.1f}m "
            f"at t={float(max_delta_row['time_s']):.3f}s"
        )
        print(
            "last_burn_sample: "
            f"t={float(last_burn_row['time_s']):.3f}s "
            f"current={float(last_burn_row['current_logic_apogee_m']):.1f}m "
            f"bridge={float(last_burn_row['burnout_bridge_apogee_m']):.1f}m"
        )
    _print_metrics("current_logic", output_rows, "current_logic_apogee_m", actual_apogee_m)
    _print_metrics("burnout_bridge", output_rows, "burnout_bridge_apogee_m", actual_apogee_m)

    _write_compare_svg(output_rows, actual_apogee_m)
    _write_control_svg(output_rows)
    print(f"Wrote {OUTPUT_CSV}")
    print(f"Wrote {OUTPUT_COMPARE_SVG}")
    print(f"Wrote {OUTPUT_CONTROL_SVG}")


if __name__ == "__main__":
    main()
