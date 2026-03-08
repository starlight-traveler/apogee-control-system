#!/usr/bin/env python3

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from python.apogee_predictor_sim import compute_actual_apogee, load_force_table, parse_replay_rows, simulate_predictions


INPUT_CSV = ROOT / "tools" / "replay" / "output.csv"
CFD_CSV = ROOT / "lib" / "cfd.csv"
OUTPUT_CSV = ROOT / "tools" / "replay" / "predictor_sim_trace.csv"
OUTPUT_APOGEE_SVG = ROOT / "tools" / "replay" / "predictor_sim_apogee.svg"
OUTPUT_SPEED_SVG = ROOT / "tools" / "replay" / "predictor_sim_horizontal_speed.svg"


def _polyline_points(xs, ys, x_min, x_max, y_min, y_max, left, top, width, height) -> str:
    points = []
    x_span = max(x_max - x_min, 1.0e-9)
    y_span = max(y_max - y_min, 1.0e-9)
    for x, y in zip(xs, ys):
        px = left + (x - x_min) / x_span * width
        py = top + height - (y - y_min) / y_span * height
        points.append(f"{px:.2f},{py:.2f}")
    return " ".join(points)


def _write_svg(path: Path, title: str, x_label: str, y_label: str, series, y_min: float, y_max: float) -> None:
    times = series[0][1]
    x_min = min(times)
    x_max = max(times)
    width = 1400
    height = 820
    left = 90
    top = 70
    plot_w = width - left - 60
    plot_h = height - top - 110
    grid = []
    for i in range(6):
        frac = i / 5.0
        y = top + plot_h - frac * plot_h
        value = y_min + frac * (y_max - y_min)
        grid.append((y, value))
    x_ticks = []
    for i in range(6):
        frac = i / 5.0
        x = left + frac * plot_w
        value = x_min + frac * (x_max - x_min)
        x_ticks.append((x, value))

    lines = []
    for label, xs, ys, color in series:
        lines.append(
            f'<polyline fill="none" stroke="{color}" stroke-width="2" points="'
            f'{_polyline_points(xs, ys, x_min, x_max, y_min, y_max, left, top, plot_w, plot_h)}"/>'
        )

    y_grid = "\n".join(
        f'<line x1="{left}" y1="{y:.2f}" x2="{left + plot_w}" y2="{y:.2f}" stroke="#d9dde3" stroke-width="1"/>'
        f'<text x="{left - 10}" y="{y + 5:.2f}" text-anchor="end" font-family="monospace" font-size="13">{value:.1f}</text>'
        for y, value in grid
    )
    x_grid = "\n".join(
        f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{top + plot_h}" stroke="#eceff3" stroke-width="1"/>'
        f'<text x="{x:.2f}" y="{top + plot_h + 24}" text-anchor="middle" font-family="monospace" font-size="13">{value:.1f}</text>'
        for x, value in x_ticks
    )
    legend = "\n".join(
        f'<rect x="{left + 20}" y="{top + 10 + i * 22}" width="14" height="14" fill="{color}"/>'
        f'<text x="{left + 42}" y="{top + 22 + i * 22}" font-family="monospace" font-size="14">{label}</text>'
        for i, (label, _, _, color) in enumerate(series)
    )

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="#f8f6f1"/>
  <rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="#fffdfa" stroke="#1f2933" stroke-width="1.5"/>
  <text x="{left}" y="36" font-family="monospace" font-size="24" fill="#1f2933">{title}</text>
  <text x="{left + plot_w / 2}" y="{height - 22}" text-anchor="middle" font-family="monospace" font-size="15">{x_label}</text>
  <text x="24" y="{top + plot_h / 2}" transform="rotate(-90 24 {top + plot_h / 2})" text-anchor="middle" font-family="monospace" font-size="15">{y_label}</text>
  {y_grid}
  {x_grid}
  {' '.join(lines)}
  {legend}
</svg>
"""
    path.write_text(svg)


def main() -> None:
    rows = parse_replay_rows(INPUT_CSV)
    actual_apogee = compute_actual_apogee(rows)
    force_table = load_force_table(CFD_CSV)
    trace = simulate_predictions(rows, force_table, sample_stride=50)
    if not trace:
        raise SystemExit("No predictor trace samples were generated.")

    with OUTPUT_CSV.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "time_s",
                "status",
                "altitude_m",
                "vertical_velocity_mps",
                "logged_apogee_m",
                "legacy_apogee_m",
                "safer_apogee_m",
                "legacy_horizontal_speed_mps",
                "safer_horizontal_speed_mps",
                "zenith_deg",
            ]
        )
        for row in trace:
            writer.writerow(
                [
                    row.time_s,
                    row.status,
                    row.altitude_m,
                    row.vertical_velocity_mps,
                    row.logged_apogee_m,
                    row.legacy_apogee_m,
                    row.safer_apogee_m,
                    row.legacy_horizontal_speed_mps,
                    row.safer_horizontal_speed_mps,
                    row.zenith_deg,
                ]
            )

    times = [row.time_s for row in trace]
    altitude = [row.altitude_m for row in trace]
    logged_apogee = [row.logged_apogee_m for row in trace]
    legacy_apogee = [row.legacy_apogee_m for row in trace]
    safer_apogee = [row.safer_apogee_m for row in trace]
    actual_apogee_series = [actual_apogee for _ in trace]

    y_min = min(min(altitude), min(safer_apogee), min(legacy_apogee), min(logged_apogee))
    y_max = max(max(actual_apogee_series), max(safer_apogee), max(legacy_apogee), max(logged_apogee))
    _write_svg(
        OUTPUT_APOGEE_SVG,
        "Apogee Predictor Replay Comparison",
        "Time [s]",
        "Altitude / Predicted Apogee [m]",
        [
            ("Altitude", times, altitude, "#1f77b4"),
            ("Logged Predictor", times, logged_apogee, "#9467bd"),
            ("Legacy Recomputed", times, legacy_apogee, "#d62728"),
            ("Safer Simulator", times, safer_apogee, "#2ca02c"),
            ("Actual Apogee", times, actual_apogee_series, "#111111"),
        ],
        y_min,
        y_max,
    )

    legacy_h = [row.legacy_horizontal_speed_mps for row in trace]
    safer_h = [row.safer_horizontal_speed_mps for row in trace]
    zenith_deg = [row.zenith_deg for row in trace]
    y_min = min(0.0, min(safer_h), min(legacy_h))
    y_max = max(max(legacy_h), max(safer_h), max(zenith_deg))
    _write_svg(
        OUTPUT_SPEED_SVG,
        "Predictor Seed Horizontal Speed",
        "Time [s]",
        "Speed [m/s] / Zenith [deg]",
        [
            ("Legacy Horizontal Speed", times, legacy_h, "#d62728"),
            ("Safer Horizontal Speed", times, safer_h, "#2ca02c"),
            ("Zenith [deg]", times, zenith_deg, "#ff7f0e"),
        ],
        y_min,
        y_max,
    )

    print(f"Wrote {OUTPUT_CSV}")
    print(f"Wrote {OUTPUT_APOGEE_SVG}")
    print(f"Wrote {OUTPUT_SPEED_SVG}")


if __name__ == "__main__":
    main()
