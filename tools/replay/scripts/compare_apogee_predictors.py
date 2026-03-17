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

from python.apogee_predictor_sim import compute_actual_apogee
from python.apogee_predictor_sim import load_force_table
from python.apogee_predictor_sim import parse_replay_rows
from python.apogee_predictor_sim import simulate_predictions


INPUT_CSV = DATA_DIR / "output.csv"
MERGED_CSV = DATA_DIR / "output_merged_seed_prediction.csv"
CFD_CSV = ROOT / "lib" / "cfd.csv"
OUTPUT_CSV = DATA_DIR / "apogee_predictor_comparison.csv"
OUTPUT_SVG = PLOTS_DIR / "predictor_comparison.svg"


@dataclass
class CompareRow:
    time_s: float
    altitude_m: float
    velocity_mps: float
    logged_apogee_m: float
    safer_apogee_m: float
    merged_apogee_m: float


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


def _decimate(rows: list[CompareRow], max_points: int) -> list[CompareRow]:
    if len(rows) <= max_points:
        return rows
    step = max(1, len(rows) // max_points)
    reduced = rows[::step]
    if reduced[-1] != rows[-1]:
        reduced.append(rows[-1])
    return reduced


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


def _first_within(rows: list[CompareRow], values: list[float], actual_apogee_m: float, threshold_m: float):
    for row, value in zip(rows, values):
        if abs(value - actual_apogee_m) <= threshold_m:
            return row, value
    return None, None


def _error_stats(rows: list[CompareRow], values: list[float], actual_apogee_m: float, lock_threshold_m: float):
    start_index = None
    for idx, value in enumerate(values):
        if abs(value - actual_apogee_m) <= lock_threshold_m:
            start_index = idx
            break
    if start_index is None:
        return None

    locked_errors = [value - actual_apogee_m for value in values[start_index:]]
    overs = [err for err in locked_errors if err > 0.0]
    unders = [err for err in locked_errors if err < 0.0]
    return {
        "start_time_s": rows[start_index].time_s,
        "start_altitude_m": rows[start_index].altitude_m,
        "mean_error_m": sum(locked_errors) / len(locked_errors),
        "max_overshoot_m": max(locked_errors),
        "max_undershoot_m": min(locked_errors),
        "overshoot_fraction": (len(overs) / len(locked_errors)) if locked_errors else 0.0,
        "undershoot_fraction": (len(unders) / len(locked_errors)) if locked_errors else 0.0,
    }


def _emit_summary(label: str, rows: list[CompareRow], values: list[float], actual_apogee_m: float) -> None:
    print(label)
    for threshold_m in (100.0, 50.0, 25.0, 10.0, 5.0):
        hit_row, hit_value = _first_within(rows, values, actual_apogee_m, threshold_m)
        if hit_row is None:
            print(f"  within_{int(threshold_m)}m: none")
        else:
            print(
                f"  within_{int(threshold_m)}m: "
                f"t={hit_row.time_s:.3f}s alt={hit_row.altitude_m:.1f}m "
                f"vz={hit_row.velocity_mps:.1f}m/s pred={hit_value:.1f}m"
            )
    stats = _error_stats(rows, values, actual_apogee_m, 25.0)
    if stats is None:
        print("  post_lock_25m: none")
    else:
        print(
            "  post_lock_25m: "
            f"start={stats['start_time_s']:.3f}s/{stats['start_altitude_m']:.1f}m "
            f"mean_err={stats['mean_error_m']:.1f}m "
            f"max_over={stats['max_overshoot_m']:.1f}m "
            f"max_under={stats['max_undershoot_m']:.1f}m "
            f"over_frac={stats['overshoot_fraction']:.3f}"
        )


def _write_svg(rows: list[CompareRow], actual_apogee_m: float) -> None:
    sampled = _decimate(rows, max_points=6000)
    times = [row.time_s for row in sampled]
    altitude = [row.altitude_m for row in sampled]
    logged = [row.logged_apogee_m for row in sampled]
    safer = [row.safer_apogee_m for row in sampled]
    merged = [row.merged_apogee_m for row in sampled]
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
    y_min = min(min(altitude), min(logged), min(safer), min(merged), actual_apogee_m)
    y_max = max(max(altitude), max(logged), max(safer), max(merged), actual_apogee_m)
    y_pad = max(1.0, (y_max - y_min) * 0.05)
    y_min -= y_pad
    y_max += y_pad

    altitude_points = _polyline_points(times, altitude, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    logged_points = _polyline_points(times, logged, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    safer_points = _polyline_points(times, safer, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    merged_points = _polyline_points(times, merged, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
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
  <text x="{width / 2:.0f}" y="36" text-anchor="middle" font-family="monospace" font-size="24">Apogee Predictor Comparison</text>
  <text x="{width / 2:.0f}" y="{height - 20}" text-anchor="middle" font-family="monospace" font-size="18">Time (s)</text>
  <text x="30" y="{height / 2:.0f}" text-anchor="middle" font-family="monospace" font-size="18" transform="rotate(-90 30 {height / 2:.0f})">Meters</text>
  {''.join(grid_lines)}
  <rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="none" stroke="black" stroke-width="2"/>
  <polyline fill="none" stroke="#1f77b4" stroke-width="1.5" points="{altitude_points}"/>
  <polyline fill="none" stroke="#9467bd" stroke-width="1.5" points="{logged_points}"/>
  <polyline fill="none" stroke="#2ca02c" stroke-width="1.5" points="{safer_points}"/>
  <polyline fill="none" stroke="#d62728" stroke-width="1.5" points="{merged_points}"/>
  <polyline fill="none" stroke="#111111" stroke-width="1.2" stroke-dasharray="6,4" points="{actual_points}"/>
  <line x1="{left + 20}" y1="{top + 20}" x2="{left + 90}" y2="{top + 20}" stroke="#1f77b4" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 26}" font-family="monospace" font-size="16">Altitude</text>
  <line x1="{left + 20}" y1="{top + 48}" x2="{left + 90}" y2="{top + 48}" stroke="#9467bd" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 54}" font-family="monospace" font-size="16">Logged Predictor</text>
  <line x1="{left + 20}" y1="{top + 76}" x2="{left + 90}" y2="{top + 76}" stroke="#2ca02c" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 82}" font-family="monospace" font-size="16">Safer Seed Recompute</text>
  <line x1="{left + 20}" y1="{top + 104}" x2="{left + 90}" y2="{top + 104}" stroke="#d62728" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 110}" font-family="monospace" font-size="16">Merged Seed Recompute</text>
  <line x1="{left + 20}" y1="{top + 132}" x2="{left + 90}" y2="{top + 132}" stroke="#111111" stroke-width="2" stroke-dasharray="6,4"/>
  <text x="{left + 100}" y="{top + 138}" font-family="monospace" font-size="16">Actual Apogee</text>
</svg>
"""

    OUTPUT_SVG.write_text(svg)


def main() -> None:
    replay_rows = parse_replay_rows(INPUT_CSV)
    actual_apogee_m = compute_actual_apogee(replay_rows)
    force_table = load_force_table(CFD_CSV)
    safer_trace = simulate_predictions(replay_rows, force_table, sample_stride=1)

    merged_rows = []
    with MERGED_CSV.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            status = row.get("status", "")
            velocity_mps = _parse_float(row.get("velocity_mps"))
            merged_apogee_m = _parse_float(row.get("apogee_prediction_m"))
            if status not in {"burn", "coast"} or velocity_mps is None or velocity_mps <= 0.0 or merged_apogee_m is None:
                continue
            time_s = _parse_float(row.get("time_s"))
            altitude_m = _parse_float(row.get("altitude_m"))
            if time_s is None or altitude_m is None:
                continue
            merged_rows.append((time_s, altitude_m, velocity_mps, merged_apogee_m))

    if len(safer_trace) != len(merged_rows):
        raise SystemExit(
            f"Row count mismatch: safer_trace={len(safer_trace)} merged_rows={len(merged_rows)}. "
            "Regenerate the merged-seed CSV before comparing."
        )

    compare_rows: list[CompareRow] = []
    for safer_row, merged_row in zip(safer_trace, merged_rows):
        compare_rows.append(
            CompareRow(
                time_s=safer_row.time_s,
                altitude_m=safer_row.altitude_m,
                velocity_mps=safer_row.vertical_velocity_mps,
                logged_apogee_m=safer_row.logged_apogee_m,
                safer_apogee_m=safer_row.safer_apogee_m,
                merged_apogee_m=merged_row[3],
            )
        )

    with OUTPUT_CSV.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "time_s",
                "altitude_m",
                "velocity_mps",
                "logged_apogee_m",
                "safer_apogee_m",
                "merged_apogee_m",
            ]
        )
        for row in compare_rows:
            writer.writerow(
                [
                    row.time_s,
                    row.altitude_m,
                    row.velocity_mps,
                    row.logged_apogee_m,
                    row.safer_apogee_m,
                    row.merged_apogee_m,
                ]
            )

    print(f"Actual apogee: {actual_apogee_m:.2f} m")
    _emit_summary("logged", compare_rows, [row.logged_apogee_m for row in compare_rows], actual_apogee_m)
    _emit_summary("safer", compare_rows, [row.safer_apogee_m for row in compare_rows], actual_apogee_m)
    _emit_summary("merged", compare_rows, [row.merged_apogee_m for row in compare_rows], actual_apogee_m)
    _write_svg(compare_rows, actual_apogee_m)
    print(f"Wrote {OUTPUT_CSV}")
    print(f"Wrote {OUTPUT_SVG}")


if __name__ == "__main__":
    main()
