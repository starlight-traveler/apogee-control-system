#!/usr/bin/env python3

import csv
import sys
from pathlib import Path


from replaylib.paths import SCRIPTS_DIR
SCRIPT_DIR = SCRIPTS_DIR
REPLAY_DIR = SCRIPT_DIR.parent
DATA_DIR = REPLAY_DIR / "data"
PLOTS_DIR = REPLAY_DIR / "plots"
DEFAULT_CSV_PATH = DATA_DIR / "fullscale_2.csv"


def _parse_float(value: str):
    if value is None:
        return None
    value = value.strip()
    if not value:
        return None
    lower = value.lower()
    if lower in {"nan", "inf", "-inf"}:
        return None
    return float(value)


def _decimate(values, max_points: int):
    if len(values) <= max_points:
        return values
    step = max(1, len(values) // max_points)
    reduced = values[::step]
    if reduced[-1] != values[-1]:
        reduced.append(values[-1])
    return reduced


def _scale(value: float, lower: float, upper: float, out_lower: float, out_upper: float) -> float:
    if upper <= lower:
        return (out_lower + out_upper) * 0.5
    fraction = (value - lower) / (upper - lower)
    return out_lower + fraction * (out_upper - out_lower)


def _polyline_points(times, values, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h):
    points = []
    bottom = top + plot_h
    for t, v in zip(times, values):
        x = _scale(t, t_min, t_max, left, left + plot_w)
        y = _scale(v, y_min, y_max, bottom, top)
        points.append(f"{x:.2f},{y:.2f}")
    return " ".join(points)


def main() -> None:
    csv_path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_CSV_PATH
    stem = csv_path.stem.replace("fullscale_2", "fullscale2")
    output_path = PLOTS_DIR / f"{stem}_apogee_vs_altitude.svg"
    output_path.parent.mkdir(parents=True, exist_ok=True)

    times = []
    altitude_m = []
    apogee_m = []

    with csv_path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        state_time_mode = "state_time" in (reader.fieldnames or [])
        for row in reader:
            if state_time_mode:
                if row.get("has_filtered_state") != "True":
                    continue
                t = _parse_float(row.get("state_time"))
                z = _parse_float(row.get("state_position_z"))
                a = _parse_float(row.get("state_apogee_estimate"))
            else:
                t = _parse_float(row.get("time_s"))
                z = _parse_float(row.get("altitude_m"))
                a = _parse_float(row.get("apogee_prediction_m"))
            if t is None or z is None or a is None:
                continue

            times.append(t)
            altitude_m.append(z)
            apogee_m.append(a)

    if not times:
        raise SystemExit("No filtered-state rows with valid apogee data found.")

    original_count = len(times)
    sampled = list(zip(times, altitude_m, apogee_m))
    sampled = _decimate(sampled, max_points=6000)
    times = [row[0] for row in sampled]
    altitude_m = [row[1] for row in sampled]
    apogee_m = [row[2] for row in sampled]

    width = 1600
    height = 900
    left = 110
    right = 40
    top = 70
    bottom_margin = 80
    plot_w = width - left - right
    plot_h = height - top - bottom_margin

    t_min = min(times)
    t_max = max(times)
    y_min = min(min(altitude_m), min(apogee_m))
    y_max = max(max(altitude_m), max(apogee_m))
    y_pad = max(1.0, (y_max - y_min) * 0.05)
    y_min -= y_pad
    y_max += y_pad

    altitude_points = _polyline_points(times, altitude_m, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    apogee_points = _polyline_points(times, apogee_m, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)

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
  <text x="{width / 2:.0f}" y="36" text-anchor="middle" font-family="monospace" font-size="24">fullscale2: Altitude vs Apogee Estimate</text>
  <text x="{width / 2:.0f}" y="{height - 20}" text-anchor="middle" font-family="monospace" font-size="18">Time (s)</text>
  <text x="30" y="{height / 2:.0f}" text-anchor="middle" font-family="monospace" font-size="18" transform="rotate(-90 30 {height / 2:.0f})">Meters</text>
  {''.join(grid_lines)}
  <rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="none" stroke="black" stroke-width="2"/>
  <polyline fill="none" stroke="#1f77b4" stroke-width="1.5" points="{altitude_points}"/>
  <polyline fill="none" stroke="#d62728" stroke-width="1.5" points="{apogee_points}"/>
  <line x1="{left + 20}" y1="{top + 20}" x2="{left + 90}" y2="{top + 20}" stroke="#1f77b4" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 26}" font-family="monospace" font-size="16">Altitude (state_position_z)</text>
  <line x1="{left + 20}" y1="{top + 48}" x2="{left + 90}" y2="{top + 48}" stroke="#d62728" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 54}" font-family="monospace" font-size="16">Apogee Estimate (state_apogee_estimate)</text>
  <text x="{left}" y="{height - 45}" font-family="monospace" font-size="14">t_min={t_min:.3f}</text>
  <text x="{left + plot_w - 170}" y="{height - 45}" font-family="monospace" font-size="14">t_max={t_max:.3f}</text>
  <text x="{left}" y="{top - 12}" font-family="monospace" font-size="14">y_max={y_max:.2f}</text>
  <text x="{left}" y="{top + plot_h + 20}" font-family="monospace" font-size="14">y_min={y_min:.2f}</text>
</svg>
"""

    output_path.write_text(svg)

    print(f"Saved plot to {output_path}")
    print(f"Plotted {len(times)} decimated samples from {original_count} rows.")


if __name__ == "__main__":
    main()
