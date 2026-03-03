#!/usr/bin/env python3

import csv
import math
import sys
from pathlib import Path


G = 9.8067
DEFAULT_CSV_PATH = Path(__file__).with_name("fullscale_2_replay.csv")


def _parse_float(value: str):
    if value is None:
        return None
    text = value.strip()
    if not text:
        return None
    lowered = text.lower()
    if lowered in {"nan", "inf", "-inf"}:
        return None
    return float(text)


def _decimate(rows, max_points: int):
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


def _ballistic_apogee(altitude_m: float, velocity_mps: float) -> float:
    if velocity_mps <= 0.0:
        return altitude_m
    return altitude_m + (velocity_mps * velocity_mps) / (2.0 * G)


def main() -> None:
    csv_path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_CSV_PATH
    output_path = csv_path.with_name(f"{csv_path.stem}_compare.svg")

    rows = []
    with csv_path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            time_s = _parse_float(row.get("time_s"))
            altitude_m = _parse_float(row.get("altitude_m"))
            velocity_mps = _parse_float(row.get("velocity_mps"))
            aero_apogee_m = _parse_float(row.get("apogee_prediction_m"))
            if time_s is None or altitude_m is None or velocity_mps is None or aero_apogee_m is None:
                continue
            ballistic_apogee_m = _ballistic_apogee(altitude_m, velocity_mps)
            rows.append((time_s, altitude_m, aero_apogee_m, ballistic_apogee_m))

    if not rows:
        raise SystemExit("No valid replay rows found.")

    original_count = len(rows)
    rows = _decimate(rows, max_points=6000)

    times = [row[0] for row in rows]
    altitude = [row[1] for row in rows]
    aero_apogee = [row[2] for row in rows]
    ballistic_apogee = [row[3] for row in rows]

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
    y_min = min(min(altitude), min(aero_apogee), min(ballistic_apogee))
    y_max = max(max(altitude), max(aero_apogee), max(ballistic_apogee))
    y_pad = max(1.0, (y_max - y_min) * 0.05)
    y_min -= y_pad
    y_max += y_pad

    altitude_points = _polyline_points(times, altitude, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    aero_points = _polyline_points(times, aero_apogee, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    ballistic_points = _polyline_points(times,
                                        ballistic_apogee,
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
  <text x="{width / 2:.0f}" y="36" text-anchor="middle" font-family="monospace" font-size="24">fullscale_2 Replay: Altitude vs Aero vs Ballistic</text>
  <text x="{width / 2:.0f}" y="{height - 20}" text-anchor="middle" font-family="monospace" font-size="18">Time (s)</text>
  <text x="30" y="{height / 2:.0f}" text-anchor="middle" font-family="monospace" font-size="18" transform="rotate(-90 30 {height / 2:.0f})">Meters</text>
  {''.join(grid_lines)}
  <rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="none" stroke="black" stroke-width="2"/>
  <polyline fill="none" stroke="#1f77b4" stroke-width="1.5" points="{altitude_points}"/>
  <polyline fill="none" stroke="#d62728" stroke-width="1.5" points="{aero_points}"/>
  <polyline fill="none" stroke="#2ca02c" stroke-width="1.5" points="{ballistic_points}"/>
  <line x1="{left + 20}" y1="{top + 20}" x2="{left + 90}" y2="{top + 20}" stroke="#1f77b4" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 26}" font-family="monospace" font-size="16">Altitude</text>
  <line x1="{left + 20}" y1="{top + 48}" x2="{left + 90}" y2="{top + 48}" stroke="#d62728" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 54}" font-family="monospace" font-size="16">Aero Predictor (Replay)</text>
  <line x1="{left + 20}" y1="{top + 76}" x2="{left + 90}" y2="{top + 76}" stroke="#2ca02c" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 82}" font-family="monospace" font-size="16">Ballistic (z + vz^2 / 2g)</text>
  <text x="{left}" y="{height - 45}" font-family="monospace" font-size="14">t_min={t_min:.3f}</text>
  <text x="{left + plot_w - 170}" y="{height - 45}" font-family="monospace" font-size="14">t_max={t_max:.3f}</text>
  <text x="{left}" y="{top - 12}" font-family="monospace" font-size="14">y_max={y_max:.2f}</text>
  <text x="{left}" y="{top + plot_h + 20}" font-family="monospace" font-size="14">y_min={y_min:.2f}</text>
</svg>
"""

    output_path.write_text(svg)
    print(f"Saved plot to {output_path}")
    print(f"Plotted {len(rows)} decimated samples from {original_count} rows.")


if __name__ == "__main__":
    main()
