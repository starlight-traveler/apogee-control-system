#!/usr/bin/env python3

from __future__ import annotations

import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
REPLAY_DIR = SCRIPT_DIR.parent
ROOT = REPLAY_DIR.parents[1]
DATA_DIR = REPLAY_DIR / "data"
PLOTS_DIR = REPLAY_DIR / "plots"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from python.apogee_predictor_sim import ApogeeState
from python.apogee_predictor_sim import ForceTable
from python.apogee_predictor_sim import G
from python.apogee_predictor_sim import SaferSeedConfig
from python.apogee_predictor_sim import gradient_wind
from python.apogee_predictor_sim import interp_axis
from python.apogee_predictor_sim import load_force_table
from python.apogee_predictor_sim import temperature_k
from python.apogee_predictor_sim import trilinear
from python.apogee_predictor_sim import wrap_to_pi
from python.apogee_predictor_sim import CP_CG_M, DRY_MASS, GAMMA, MAX_STEPS, MOMENT_INERTIA, PRED_DT, R_GAS


INPUT_CSV = DATA_DIR / "output.csv"
MERGED_CSV = DATA_DIR / "output_merged_seed_prediction.csv"
CFD_CSV = ROOT / "lib" / "cfd.csv"
OUTPUT_CSV = DATA_DIR / "output_adaptive_drag_prediction.csv"
OUTPUT_COMPARE_SVG = PLOTS_DIR / "adaptive_drag_comparison.svg"
OUTPUT_DRAG_SVG = PLOTS_DIR / "adaptive_drag_scale.svg"


@dataclass
class ActiveReplayRow:
    time_s: float
    altitude_m: float
    velocity_mps: float
    measured_accel_z_mps2: float
    logged_apogee_m: float
    status: str


@dataclass
class MergedSeedRow:
    time_s: float
    altitude_m: float
    velocity_mps: float
    merged_apogee_m: float
    horizontal_speed_mps: float
    zenith_rad: float
    angular_rate_rad_s: float
    status: str


def _parse_float(value: str | None) -> float | None:
    if value is None:
        return None
    text = value.strip()
    if not text:
        return None
    lowered = text.lower()
    if lowered in {"nan", "inf", "-inf"}:
        return None
    return float(text)


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def _fresh_sample(dt_seconds: float) -> bool:
    return math.isfinite(dt_seconds) and dt_seconds > 1.0e-4 and dt_seconds <= 0.25


def _load_active_replay_rows(path: Path) -> list[ActiveReplayRow]:
    rows: list[ActiveReplayRow] = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            if row.get("has_filtered_state") != "True":
                continue
            status = row.get("flight_status", "")
            velocity_mps = _parse_float(row.get("state_velocity_z"))
            if status not in {"burn", "coast"} or velocity_mps is None or velocity_mps <= 0.0:
                continue
            time_s = _parse_float(row.get("state_time"))
            altitude_m = _parse_float(row.get("state_position_z"))
            measured_accel_z_mps2 = _parse_float(row.get("state_acceleration_z"))
            logged_apogee_m = _parse_float(row.get("state_apogee_estimate"))
            if (
                time_s is None
                or altitude_m is None
                or measured_accel_z_mps2 is None
                or logged_apogee_m is None
            ):
                continue
            rows.append(
                ActiveReplayRow(
                    time_s=time_s,
                    altitude_m=altitude_m,
                    velocity_mps=velocity_mps,
                    measured_accel_z_mps2=measured_accel_z_mps2,
                    logged_apogee_m=logged_apogee_m,
                    status=status,
                )
            )
    return rows


def _load_merged_seed_rows(path: Path) -> list[MergedSeedRow]:
    rows: list[MergedSeedRow] = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            status = row.get("status", "")
            velocity_mps = _parse_float(row.get("velocity_mps"))
            if status not in {"burn", "coast"} or velocity_mps is None or velocity_mps <= 0.0:
                continue
            time_s = _parse_float(row.get("time_s"))
            altitude_m = _parse_float(row.get("altitude_m"))
            merged_apogee_m = _parse_float(row.get("apogee_prediction_m"))
            horizontal_speed_mps = _parse_float(row.get("merged_horizontal_speed_mps"))
            zenith_deg = _parse_float(row.get("zenith_deg"))
            angular_rate_rad_s = _parse_float(row.get("angular_rate_rad_s"))
            if (
                time_s is None
                or altitude_m is None
                or merged_apogee_m is None
                or horizontal_speed_mps is None
                or zenith_deg is None
                or angular_rate_rad_s is None
            ):
                continue
            rows.append(
                MergedSeedRow(
                    time_s=time_s,
                    altitude_m=altitude_m,
                    velocity_mps=velocity_mps,
                    merged_apogee_m=merged_apogee_m,
                    horizontal_speed_mps=horizontal_speed_mps,
                    zenith_rad=math.radians(zenith_deg),
                    angular_rate_rad_s=angular_rate_rad_s,
                    status=status,
                )
            )
    return rows


def _compute_accel_with_axial_scale(
    state: ApogeeState,
    table: ForceTable,
    axial_drag_scale: float,
) -> tuple[float, float, float, float]:
    rel_x = state.vertical_v
    rel_y = state.horizontal_v - gradient_wind()[1]
    temp_k = temperature_k(state.altitude_m)
    speed_of_sound = math.sqrt(GAMMA * R_GAS * temp_k) if temp_k > 0.0 else 0.0
    mach = math.hypot(rel_x, rel_y) / speed_of_sound if speed_of_sound > 0.0 else 0.0

    gravity_x = -G
    gravity_y = 0.0
    linear_x = gravity_x
    linear_y = gravity_y
    angular = 0.0
    axial_linear_x = 0.0

    if mach >= 0.025:
        velocity_angle = math.atan2(rel_y, rel_x)
        signed_atk = wrap_to_pi(state.zenith - velocity_angle)
        lift_state = signed_atk >= 0.0
        atk_deg = math.degrees(abs(signed_atk))

        i0, i1, ti = interp_axis(table.acs_angles, state.acs_deg)
        j0, j1, tj = interp_axis(table.atk_angles, atk_deg)
        k0, k1, tk = interp_axis(table.mach_numbers, mach)

        axial_force = trilinear(table.axial_forces, i0, i1, j0, j1, k0, k1, ti, tj, tk) * axial_drag_scale
        normal_force = trilinear(table.normal_forces, i0, i1, j0, j1, k0, k1, ti, tj, tk)

        sin_z = math.sin(state.zenith)
        cos_z = math.cos(state.zenith)
        axial_x = -axial_force * cos_z
        axial_y = -axial_force * sin_z
        normal_x = -normal_force * sin_z
        normal_y = normal_force * cos_z
        angular = (-normal_force * CP_CG_M * 0.2) / MOMENT_INERTIA

        if not lift_state:
            normal_x = -normal_x
            normal_y = -normal_y
            angular = -angular

        inv_mass = 1.0 / DRY_MASS
        axial_linear_x = axial_x * inv_mass
        linear_x = gravity_x + (axial_x + normal_x) * inv_mass
        linear_y = gravity_y + (axial_y + normal_y) * inv_mass

    return linear_x, linear_y, angular, axial_linear_x


def _evaluate_derivative_with_axial_scale(
    state: ApogeeState,
    table: ForceTable,
    axial_drag_scale: float,
) -> tuple[float, float, float, float, float, float]:
    ax, ay, angular, _ = _compute_accel_with_axial_scale(state, table, axial_drag_scale)
    return (state.vertical_v, state.horizontal_v, ax, ay, state.angular_v, angular)


def _apply_step(state: ApogeeState, deriv, dt: float) -> ApogeeState:
    return ApogeeState(
        altitude_m=state.altitude_m + deriv[0] * dt,
        horizontal_m=state.horizontal_m + deriv[1] * dt,
        vertical_v=state.vertical_v + deriv[2] * dt,
        horizontal_v=state.horizontal_v + deriv[3] * dt,
        zenith=state.zenith + deriv[4] * dt,
        angular_v=state.angular_v + deriv[5] * dt,
        acs_deg=state.acs_deg,
    )


def _integrate_rk4(state: ApogeeState, k1, k2, k3, k4) -> None:
    sixth = PRED_DT / 6.0
    state.altitude_m += sixth * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0])
    state.horizontal_m += sixth * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1])
    state.vertical_v += sixth * (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2])
    state.horizontal_v += sixth * (k1[3] + 2.0 * k2[3] + 2.0 * k3[3] + k4[3])
    state.zenith += sixth * (k1[4] + 2.0 * k2[4] + 2.0 * k3[4] + k4[4])
    state.angular_v += sixth * (k1[5] + 2.0 * k2[5] + 2.0 * k3[5] + k4[5])


def _predict_apogee_with_axial_scale(state: ApogeeState, table: ForceTable, axial_drag_scale: float) -> float:
    if state.vertical_v <= 0.0:
        return state.altitude_m
    current = ApogeeState(**state.__dict__)
    steps = 0
    while current.vertical_v > 0.0 and steps < MAX_STEPS:
        k1 = _evaluate_derivative_with_axial_scale(current, table, axial_drag_scale)
        k2 = _evaluate_derivative_with_axial_scale(_apply_step(current, k1, 0.5 * PRED_DT), table, axial_drag_scale)
        k3 = _evaluate_derivative_with_axial_scale(_apply_step(current, k2, 0.5 * PRED_DT), table, axial_drag_scale)
        k4 = _evaluate_derivative_with_axial_scale(_apply_step(current, k3, PRED_DT), table, axial_drag_scale)
        _integrate_rk4(current, k1, k2, k3, k4)
        steps += 1
    return current.altitude_m


def _actual_apogee_from_output(path: Path) -> float:
    maximum = -math.inf
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            altitude = _parse_float(row.get("state_position_z"))
            if altitude is None:
                continue
            maximum = max(maximum, altitude)
    return maximum


def _first_within(rows, key: str, actual_apogee_m: float, threshold_m: float):
    for row in rows:
        if abs(float(row[key]) - actual_apogee_m) <= threshold_m:
            return row
    return None


def _post_lock_metrics(rows, key: str, actual_apogee_m: float, lock_threshold_m: float):
    start_index = None
    for idx, row in enumerate(rows):
        if abs(float(row[key]) - actual_apogee_m) <= lock_threshold_m:
            start_index = idx
            break
    if start_index is None:
        return None

    subset = rows[start_index:]
    errors = [float(row[key]) - actual_apogee_m for row in subset]
    overshoot_time_s = 0.0
    for previous, current in zip(subset, subset[1:]):
        if (float(previous[key]) - actual_apogee_m) > 0.0:
            overshoot_time_s += max(0.0, float(current["time_s"]) - float(previous["time_s"]))
    return {
        "start_time_s": float(subset[0]["time_s"]),
        "start_altitude_m": float(subset[0]["altitude_m"]),
        "mean_error_m": sum(errors) / len(errors),
        "max_overshoot_m": max(errors),
        "max_undershoot_m": min(errors),
        "overshoot_time_s": overshoot_time_s,
    }


def _scale(value: float, lower: float, upper: float, out_lower: float, out_upper: float) -> float:
    if upper <= lower:
        return (out_lower + out_upper) * 0.5
    fraction = (value - lower) / (upper - lower)
    return out_lower + fraction * (out_upper - out_lower)


def _polyline_points(times, values, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h):
    points = []
    bottom = top + plot_h
    for time_s, value in zip(times, values):
        x = _scale(time_s, t_min, t_max, left, left + plot_w)
        y = _scale(value, y_min, y_max, bottom, top)
        points.append(f"{x:.2f},{y:.2f}")
    return " ".join(points)


def _decimate(rows: list[dict[str, float | str]], max_points: int) -> list[dict[str, float | str]]:
    if len(rows) <= max_points:
        return rows
    step = max(1, len(rows) // max_points)
    reduced = rows[::step]
    if reduced[-1] != rows[-1]:
        reduced.append(rows[-1])
    return reduced


def _write_comparison_svg(rows: list[dict[str, float | str]], actual_apogee_m: float) -> None:
    sampled = _decimate(rows, 6000)
    times = [float(row["time_s"]) for row in sampled]
    altitude = [float(row["altitude_m"]) for row in sampled]
    logged = [float(row["logged_apogee_m"]) for row in sampled]
    merged = [float(row["merged_apogee_m"]) for row in sampled]
    adaptive = [float(row["adaptive_apogee_m"]) for row in sampled]
    actual = [actual_apogee_m for _ in sampled]

    width = 1700
    height = 950
    left = 110
    right = 40
    top = 70
    bottom_margin = 80
    plot_w = width - left - right
    plot_h = height - top - bottom_margin

    t_min = min(times)
    t_max = max(times)
    y_min = min(min(altitude), min(logged), min(merged), min(adaptive), actual_apogee_m)
    y_max = max(max(altitude), max(logged), max(merged), max(adaptive), actual_apogee_m)
    y_pad = max(1.0, (y_max - y_min) * 0.05)
    y_min -= y_pad
    y_max += y_pad

    altitude_points = _polyline_points(times, altitude, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    logged_points = _polyline_points(times, logged, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    merged_points = _polyline_points(times, merged, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    adaptive_points = _polyline_points(times, adaptive, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    actual_points = _polyline_points(times, actual, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)

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
  <text x="{width / 2:.0f}" y="36" text-anchor="middle" font-family="monospace" font-size="24">Adaptive Drag Predictor Comparison</text>
  <text x="{width / 2:.0f}" y="{height - 20}" text-anchor="middle" font-family="monospace" font-size="18">Time (s)</text>
  <text x="30" y="{height / 2:.0f}" text-anchor="middle" font-family="monospace" font-size="18" transform="rotate(-90 30 {height / 2:.0f})">Meters</text>
  {''.join(grid_lines)}
  <rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="none" stroke="black" stroke-width="2"/>
  <polyline fill="none" stroke="#1f77b4" stroke-width="1.5" points="{altitude_points}"/>
  <polyline fill="none" stroke="#9467bd" stroke-width="1.5" points="{logged_points}"/>
  <polyline fill="none" stroke="#d62728" stroke-width="1.5" points="{merged_points}"/>
  <polyline fill="none" stroke="#2ca02c" stroke-width="1.5" points="{adaptive_points}"/>
  <polyline fill="none" stroke="#111111" stroke-width="1.2" stroke-dasharray="6,4" points="{actual_points}"/>
  <line x1="{left + 20}" y1="{top + 20}" x2="{left + 90}" y2="{top + 20}" stroke="#1f77b4" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 26}" font-family="monospace" font-size="16">Altitude</text>
  <line x1="{left + 20}" y1="{top + 48}" x2="{left + 90}" y2="{top + 48}" stroke="#9467bd" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 54}" font-family="monospace" font-size="16">Logged Predictor</text>
  <line x1="{left + 20}" y1="{top + 76}" x2="{left + 90}" y2="{top + 76}" stroke="#d62728" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 82}" font-family="monospace" font-size="16">Old Model (Merged Seed)</text>
  <line x1="{left + 20}" y1="{top + 104}" x2="{left + 90}" y2="{top + 104}" stroke="#2ca02c" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 110}" font-family="monospace" font-size="16">Adaptive Drag Model</text>
  <line x1="{left + 20}" y1="{top + 132}" x2="{left + 90}" y2="{top + 132}" stroke="#111111" stroke-width="2" stroke-dasharray="6,4"/>
  <text x="{left + 100}" y="{top + 138}" font-family="monospace" font-size="16">Actual Apogee</text>
</svg>
"""
    OUTPUT_COMPARE_SVG.write_text(svg)


def _write_drag_svg(rows: list[dict[str, float | str]]) -> None:
    sampled = _decimate(rows, 6000)
    times = [float(row["time_s"]) for row in sampled]
    drag_scale = [float(row["drag_scale"]) for row in sampled]
    measured_accel = [float(row["measured_accel_z_mps2"]) for row in sampled]
    predicted_accel = [float(row["predicted_accel_z_mps2"]) for row in sampled]

    width = 1700
    height = 950
    left = 110
    right = 40
    top = 70
    bottom_margin = 80
    plot_w = width - left - right
    plot_h = height - top - bottom_margin

    t_min = min(times)
    t_max = max(times)
    y_min = min(min(drag_scale), min(measured_accel), min(predicted_accel))
    y_max = max(max(drag_scale), max(measured_accel), max(predicted_accel))
    y_pad = max(1.0, (y_max - y_min) * 0.05)
    y_min -= y_pad
    y_max += y_pad

    drag_points = _polyline_points(times, drag_scale, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    measured_points = _polyline_points(times, measured_accel, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    predicted_points = _polyline_points(times, predicted_accel, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)

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
  <text x="{width / 2:.0f}" y="36" text-anchor="middle" font-family="monospace" font-size="24">Adaptive Drag Scale and Acceleration Match</text>
  <text x="{width / 2:.0f}" y="{height - 20}" text-anchor="middle" font-family="monospace" font-size="18">Time (s)</text>
  <text x="30" y="{height / 2:.0f}" text-anchor="middle" font-family="monospace" font-size="18" transform="rotate(-90 30 {height / 2:.0f})">Scale / m/s²</text>
  {''.join(grid_lines)}
  <rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="none" stroke="black" stroke-width="2"/>
  <polyline fill="none" stroke="#2ca02c" stroke-width="1.5" points="{drag_points}"/>
  <polyline fill="none" stroke="#1f77b4" stroke-width="1.5" points="{measured_points}"/>
  <polyline fill="none" stroke="#d62728" stroke-width="1.5" points="{predicted_points}"/>
  <line x1="{left + 20}" y1="{top + 20}" x2="{left + 90}" y2="{top + 20}" stroke="#2ca02c" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 26}" font-family="monospace" font-size="16">Drag Scale</text>
  <line x1="{left + 20}" y1="{top + 48}" x2="{left + 90}" y2="{top + 48}" stroke="#1f77b4" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 54}" font-family="monospace" font-size="16">Measured Vertical Accel</text>
  <line x1="{left + 20}" y1="{top + 76}" x2="{left + 90}" y2="{top + 76}" stroke="#d62728" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 82}" font-family="monospace" font-size="16">Model Vertical Accel</text>
</svg>
"""
    OUTPUT_DRAG_SVG.write_text(svg)


def main() -> None:
    config = SaferSeedConfig()
    min_drag_scale = 1.00
    max_drag_scale = 1.35
    residual_clamp_mps2 = 8.0
    adaptation_tau_s = 0.45
    min_axial_accel_abs_mps2 = 0.75

    active_rows = _load_active_replay_rows(INPUT_CSV)
    merged_rows = _load_merged_seed_rows(MERGED_CSV)
    if len(active_rows) != len(merged_rows):
        raise SystemExit(
            f"Active replay row count mismatch: active={len(active_rows)} merged={len(merged_rows)}. "
            "Regenerate the merged-seed CSV first."
        )

    force_table = load_force_table(CFD_CSV)
    actual_apogee_m = _actual_apogee_from_output(INPUT_CSV)

    drag_scale = 1.0
    previous_time_s = active_rows[0].time_s
    output_rows: list[dict[str, float | str]] = []

    for active_row, merged_row in zip(active_rows, merged_rows):
        if abs(active_row.time_s - merged_row.time_s) > 1.0e-3:
            raise SystemExit(
                f"Replay alignment mismatch at t={active_row.time_s:.6f}s vs {merged_row.time_s:.6f}s"
            )

        dt_seconds = active_row.time_s - previous_time_s
        seed_state = ApogeeState(
            altitude_m=merged_row.altitude_m,
            horizontal_m=0.0,
            vertical_v=merged_row.velocity_mps,
            horizontal_v=merged_row.horizontal_speed_mps,
            zenith=merged_row.zenith_rad,
            angular_v=merged_row.angular_rate_rad_s,
            acs_deg=0.0,
        )

        predicted_accel_z_mps2, _, _, axial_accel_z_mps2 = _compute_accel_with_axial_scale(
            seed_state, force_table, drag_scale
        )
        accel_residual_mps2 = active_row.measured_accel_z_mps2 - predicted_accel_z_mps2

        valid_adaptation = (
            merged_row.status == "coast"
            and merged_row.velocity_mps > 0.0
            and _fresh_sample(dt_seconds)
            and math.isfinite(active_row.measured_accel_z_mps2)
            and math.isfinite(merged_row.zenith_rad)
        )
        if valid_adaptation and abs(axial_accel_z_mps2) >= min_axial_accel_abs_mps2:
            clamped_residual = _clamp(accel_residual_mps2, -residual_clamp_mps2, residual_clamp_mps2)
            target_scale = drag_scale + (clamped_residual / axial_accel_z_mps2)
            target_scale = _clamp(target_scale, min_drag_scale, max_drag_scale)
            alpha = 1.0 - math.exp(-dt_seconds / adaptation_tau_s) if dt_seconds > 0.0 else 0.0
            drag_scale = _clamp(drag_scale + alpha * (target_scale - drag_scale), min_drag_scale, max_drag_scale)

        adaptive_apogee_m = _predict_apogee_with_axial_scale(seed_state, force_table, drag_scale)
        output_rows.append(
            {
                "time_s": active_row.time_s,
                "altitude_m": active_row.altitude_m,
                "velocity_mps": active_row.velocity_mps,
                "logged_apogee_m": active_row.logged_apogee_m,
                "merged_apogee_m": merged_row.merged_apogee_m,
                "adaptive_apogee_m": adaptive_apogee_m,
                "drag_scale": drag_scale,
                "predicted_accel_z_mps2": predicted_accel_z_mps2,
                "measured_accel_z_mps2": active_row.measured_accel_z_mps2,
                "accel_residual_mps2": accel_residual_mps2,
                "axial_accel_z_mps2": axial_accel_z_mps2,
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
                "adaptive_apogee_m",
                "drag_scale",
                "predicted_accel_z_mps2",
                "measured_accel_z_mps2",
                "accel_residual_mps2",
                "axial_accel_z_mps2",
                "status",
            ],
        )
        writer.writeheader()
        writer.writerows(output_rows)

    print(f"Actual apogee: {actual_apogee_m:.2f} m")
    print(
        "Adaptive settings: "
        f"drag_scale=[{min_drag_scale:.2f}, {max_drag_scale:.2f}] "
        f"tau={adaptation_tau_s:.2f}s residual_clamp={residual_clamp_mps2:.1f}m/s^2 "
        f"min_axial={min_axial_accel_abs_mps2:.2f}m/s^2"
    )

    for label, key in (
        ("logged", "logged_apogee_m"),
        ("old_model", "merged_apogee_m"),
        ("adaptive", "adaptive_apogee_m"),
    ):
        print(label)
        for threshold_m in (25.0, 10.0, 5.0):
            hit = _first_within(output_rows, key, actual_apogee_m, threshold_m)
            if hit is None:
                print(f"  within_{int(threshold_m)}m: none")
            else:
                print(
                    f"  within_{int(threshold_m)}m: "
                    f"t={float(hit['time_s']):.3f}s alt={float(hit['altitude_m']):.1f}m "
                    f"pred={float(hit[key]):.1f}m"
                )
        stats = _post_lock_metrics(output_rows, key, actual_apogee_m, 25.0)
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
    _write_drag_svg(output_rows)
    print(f"Wrote {OUTPUT_CSV}")
    print(f"Wrote {OUTPUT_COMPARE_SVG}")
    print(f"Wrote {OUTPUT_DRAG_SVG}")


if __name__ == "__main__":
    main()
