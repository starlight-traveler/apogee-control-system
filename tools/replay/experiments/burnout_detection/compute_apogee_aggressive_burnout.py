#!/usr/bin/env python3

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path


from replaylib.paths import SCRIPTS_DIR
SCRIPT_DIR = SCRIPTS_DIR
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

OUTPUT_CSV = DATA_DIR / "output_aggressive_burnout_prediction.csv"
OUTPUT_COMPARE_SVG = PLOTS_DIR / "aggressive_burnout_comparison.svg"

ADAPTIVE_DRAG_SCALE_MIN = 1.0
ADAPTIVE_DRAG_SCALE_MAX = 1.35
ADAPTIVE_RESIDUAL_CLAMP_MPS2 = 8.0
ADAPTIVE_TAU_S = 0.45
ADAPTIVE_MIN_AXIAL_ACCEL_ABS_MPS2 = 0.75

AGGRESSIVE_THRESHOLDS_MPS2 = (8.0, 6.0, 4.0, 2.0)
MIN_BURN_TIME_S = 0.75


def _variant_key(threshold_mps2: float) -> str:
    return f"aggressive_{int(threshold_mps2)}"


def _compute_effective_status(
    merged_rows: list[adaptive.MergedSeedRow],
    active_rows: list[adaptive.ActiveReplayRow],
    accel_threshold_mps2: float,
) -> list[str]:
    effective_status: list[str] = []
    burn_start_s: float | None = None
    switched_to_coast = False

    for merged_row, active_row in zip(merged_rows, active_rows):
        status = merged_row.status
        if status == "burn" and burn_start_s is None:
            burn_start_s = merged_row.time_s

        if switched_to_coast:
            effective_status.append("coast")
            continue

        enough_burn_time = burn_start_s is not None and (merged_row.time_s - burn_start_s) >= MIN_BURN_TIME_S
        if (
            status == "burn"
            and enough_burn_time
            and active_row.measured_accel_z_mps2 <= accel_threshold_mps2
            and active_row.velocity_mps > 0.0
        ):
            switched_to_coast = True
            effective_status.append("coast")
            continue

        effective_status.append(status)

    return effective_status


def _print_metrics(
    label: str,
    rows: list[dict[str, float | str]],
    key: str,
    status_key: str,
    actual_apogee_m: float,
) -> None:
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
                f"effective_status={row_or_default(hit, status_key, hit['status'])}"
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


def row_or_default(row: dict[str, float | str], key: str, default: float | str) -> float | str:
    value = row.get(key)
    return default if value is None else value


def _write_compare_svg(
    rows: list[dict[str, float | str]],
    actual_apogee_m: float,
    variant_keys: list[str],
    coast_start_times_s: dict[str, float | None],
) -> None:
    sampled = adaptive._decimate(rows, 6000)
    times = [float(row["time_s"]) for row in sampled]
    altitude = [float(row["altitude_m"]) for row in sampled]
    logged = [float(row["logged_apogee_m"]) for row in sampled]
    actual = [actual_apogee_m for _ in sampled]

    series = {key: [float(row[f"{key}_apogee_m"]) for row in sampled] for key in variant_keys}
    colors = {
        "baseline": "#d62728",
        _variant_key(8.0): "#2ca02c",
        _variant_key(6.0): "#ff7f0e",
        _variant_key(4.0): "#9467bd",
        _variant_key(2.0): "#8c564b",
    }
    labels = {
        "baseline": "Logged Burnout / Coast Gate",
        _variant_key(8.0): "Aggressive Burnout <= 8 m/s^2",
        _variant_key(6.0): "Aggressive Burnout <= 6 m/s^2",
        _variant_key(4.0): "Aggressive Burnout <= 4 m/s^2",
        _variant_key(2.0): "Aggressive Burnout <= 2 m/s^2",
    }

    width = 1900
    height = 1100
    left = 110
    right = 50
    top = 70
    bottom_margin = 80
    plot_w = width - left - right
    plot_h = height - top - bottom_margin

    y_min = min(min(altitude), min(logged), actual_apogee_m, *(min(values) for values in series.values()))
    y_max = max(max(altitude), max(logged), actual_apogee_m, *(max(values) for values in series.values()))
    y_pad = max(1.0, (y_max - y_min) * 0.05)
    y_min -= y_pad
    y_max += y_pad
    t_min = min(times)
    t_max = max(times)

    altitude_points = adaptive._polyline_points(times, altitude, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    logged_points = adaptive._polyline_points(times, logged, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    actual_points = adaptive._polyline_points(times, actual, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    variant_points = {
        key: adaptive._polyline_points(times, values, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
        for key, values in series.items()
    }

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

    coast_markers = []
    for key in variant_keys:
        coast_start_s = coast_start_times_s.get(key)
        if coast_start_s is None:
            continue
        coast_x = adaptive._scale(coast_start_s, t_min, t_max, left, left + plot_w)
        coast_markers.append(
            f'<line x1="{coast_x:.2f}" y1="{top}" x2="{coast_x:.2f}" y2="{top + plot_h}" '
            f'stroke="{colors[key]}" stroke-width="1.2" stroke-dasharray="7,5"/>'
        )

    legend_y = top + 20
    legend_entries = [
        ('#1f77b4', 'Altitude'),
        ('#7f7f7f', 'Logged Predictor'),
        ('#111111', 'Actual Apogee'),
    ]
    for key in variant_keys:
        legend_entries.append((colors[key], labels[key]))

    legend_svg = []
    for idx, (color, label) in enumerate(legend_entries):
        y = legend_y + (idx * 28)
        dash = ' stroke-dasharray="6,4"' if label == "Actual Apogee" else ""
        legend_svg.append(
            f'<line x1="{left + 20}" y1="{y}" x2="{left + 90}" y2="{y}" stroke="{color}" stroke-width="3"{dash}/>'
        )
        legend_svg.append(
            f'<text x="{left + 100}" y="{y + 6}" font-family="monospace" font-size="16">{label}</text>'
        )

    variant_svg = [
        f'<polyline fill="none" stroke="{colors[key]}" stroke-width="1.5" points="{variant_points[key]}"/>'
        for key in variant_keys
    ]

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="white"/>
  <text x="{width / 2:.0f}" y="36" text-anchor="middle" font-family="monospace" font-size="24">Aggressive Burnout Detection Sweep</text>
  <text x="{width / 2:.0f}" y="{height - 20}" text-anchor="middle" font-family="monospace" font-size="18">Time (s)</text>
  <text x="30" y="{height / 2:.0f}" text-anchor="middle" font-family="monospace" font-size="18" transform="rotate(-90 30 {height / 2:.0f})">Meters</text>
  {''.join(grid_lines)}
  <rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="none" stroke="black" stroke-width="2"/>
  {''.join(coast_markers)}
  <polyline fill="none" stroke="#1f77b4" stroke-width="1.5" points="{altitude_points}"/>
  <polyline fill="none" stroke="#7f7f7f" stroke-width="1.2" points="{logged_points}"/>
  {''.join(variant_svg)}
  <polyline fill="none" stroke="#111111" stroke-width="1.2" stroke-dasharray="6,4" points="{actual_points}"/>
  {''.join(legend_svg)}
</svg>
"""
    OUTPUT_COMPARE_SVG.write_text(svg)


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

    variant_keys = ["baseline", *(_variant_key(threshold) for threshold in AGGRESSIVE_THRESHOLDS_MPS2)]
    effective_status_map: dict[str, list[str]] = {
        "baseline": [row.status for row in merged_rows],
    }
    for threshold_mps2 in AGGRESSIVE_THRESHOLDS_MPS2:
        key = _variant_key(threshold_mps2)
        effective_status_map[key] = _compute_effective_status(merged_rows, active_rows, threshold_mps2)

    coast_start_times_s: dict[str, float | None] = {}
    for key, statuses in effective_status_map.items():
        coast_index = next((idx for idx, status in enumerate(statuses) if status == "coast"), None)
        coast_start_times_s[key] = None if coast_index is None else active_rows[coast_index].time_s

    drag_scale_map = {key: 1.0 for key in variant_keys}
    previous_time_s = active_rows[0].time_s
    output_rows: list[dict[str, float | str]] = []

    for row_index, (active_row, merged_row) in enumerate(zip(active_rows, merged_rows)):
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

        output_row: dict[str, float | str] = {
            "time_s": active_row.time_s,
            "altitude_m": active_row.altitude_m,
            "velocity_mps": active_row.velocity_mps,
            "logged_apogee_m": active_row.logged_apogee_m,
            "status": active_row.status,
            "measured_accel_z_mps2": active_row.measured_accel_z_mps2,
        }

        for key in variant_keys:
            drag_scale = drag_scale_map[key]
            effective_status = effective_status_map[key][row_index]
            predicted_accel_z_mps2, _, _, axial_accel_z_mps2 = adaptive._compute_accel_with_axial_scale(
                seed_state, force_table, drag_scale
            )
            accel_residual_mps2 = active_row.measured_accel_z_mps2 - predicted_accel_z_mps2

            valid_adaptation = (
                effective_status == "coast"
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
                drag_scale_map[key] = drag_scale

            output_row[f"{key}_effective_status"] = effective_status
            output_row[f"{key}_drag_scale"] = drag_scale
            output_row[f"{key}_predicted_accel_z_mps2"] = predicted_accel_z_mps2
            output_row[f"{key}_apogee_m"] = adaptive._predict_apogee_with_axial_scale(seed_state, force_table, drag_scale)

        output_rows.append(output_row)
        previous_time_s = active_row.time_s

    with OUTPUT_CSV.open("w", newline="") as handle:
        fieldnames = [
            "time_s",
            "altitude_m",
            "velocity_mps",
            "logged_apogee_m",
            "status",
            "measured_accel_z_mps2",
        ]
        for key in variant_keys:
            fieldnames.extend(
                [
                    f"{key}_effective_status",
                    f"{key}_drag_scale",
                    f"{key}_predicted_accel_z_mps2",
                    f"{key}_apogee_m",
                ]
            )
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(output_rows)

    print(f"Actual apogee: {actual_apogee_m:.2f} m")
    for key in variant_keys:
        coast_start_s = coast_start_times_s[key]
        if coast_start_s is None:
            print(f"{key}: no coast transition")
            continue
        print(f"{key}: coast_start={coast_start_s:.3f}s")
        _print_metrics(key, output_rows, f"{key}_apogee_m", f"{key}_effective_status", actual_apogee_m)

    _write_compare_svg(output_rows, actual_apogee_m, variant_keys, coast_start_times_s)
    print(f"Wrote {OUTPUT_CSV}")
    print(f"Wrote {OUTPUT_COMPARE_SVG}")


if __name__ == "__main__":
    main()
