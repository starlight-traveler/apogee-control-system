#!/usr/bin/env python3

import csv
import sys
from pathlib import Path


G = 9.8067
DEFAULT_CSV_PATH = Path(__file__).with_name("fullscale_2_replay.csv")
DEFAULT_T_MIN = 607.9
DEFAULT_T_MAX = 609.2
DEFAULT_FLAP_TIME = 608.2


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


def _aero_shift_levels(times, aero_values, max_levels: int):
    deltas = []
    for i in range(1, len(aero_values)):
        dp = aero_values[i] - aero_values[i - 1]
        if abs(dp) < 0.5:
            continue
        deltas.append((abs(dp), times[i], aero_values[i]))
    deltas.sort(reverse=True, key=lambda item: item[0])

    levels = []
    for _, time_s, level_m in deltas:
        if any(abs(level_m - existing[1]) < 6.0 for existing in levels):
            continue
        levels.append((time_s, level_m))
        if len(levels) >= max_levels:
            break
    return sorted(levels, key=lambda item: item[0])


def main() -> None:
    csv_path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_CSV_PATH
    t_min = float(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_T_MIN
    t_max = float(sys.argv[3]) if len(sys.argv) > 3 else DEFAULT_T_MAX
    flap_time = float(sys.argv[4]) if len(sys.argv) > 4 else DEFAULT_FLAP_TIME

    if t_max <= t_min:
        raise SystemExit("Invalid window: t_max must be > t_min.")

    t_min_tag = f"{t_min:.1f}".replace(".", "p")
    t_max_tag = f"{t_max:.1f}".replace(".", "p")
    output_path = csv_path.with_name(f"{csv_path.stem}_compare_zoom_{t_min_tag}_{t_max_tag}.svg")

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
            if time_s < t_min or time_s > t_max:
                continue
            ballistic_apogee_m = _ballistic_apogee(altitude_m, velocity_mps)
            rows.append((time_s, altitude_m, aero_apogee_m, ballistic_apogee_m))

    if not rows:
        raise SystemExit("No valid replay rows found in the requested time window.")

    times = [row[0] for row in rows]
    altitude = [row[1] for row in rows]
    aero_apogee = [row[2] for row in rows]
    ballistic_apogee = [row[3] for row in rows]

    width = 1800
    height = 1000
    left = 120
    right = 40
    top = 80
    bottom_margin = 120
    plot_w = width - left - right
    plot_h = height - top - bottom_margin

    y_min = min(min(altitude), min(aero_apogee), min(ballistic_apogee))
    y_max = max(max(altitude), max(aero_apogee), max(ballistic_apogee))
    y_pad = max(2.0, (y_max - y_min) * 0.12)
    y_min -= y_pad
    y_max += y_pad

    altitude_points = _polyline_points(times, altitude, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    aero_points = _polyline_points(times, aero_apogee, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    ballistic_points = _polyline_points(times, ballistic_apogee, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)

    x_tick_count = 26
    y_tick_count = 10
    grid_lines = []
    x_ticks = []
    for i in range(x_tick_count + 1):
        x = left + (plot_w * i / x_tick_count)
        grid_lines.append(
            f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{top + plot_h}" stroke="#dddddd" stroke-width="1"/>'
        )
        tick_time = _scale(i, 0.0, x_tick_count, t_min, t_max)
        x_ticks.append(
            f'<text x="{x:.2f}" y="{top + plot_h + 24}" text-anchor="middle" font-family="monospace" font-size="12">{tick_time:.3f}</text>'
        )

    y_ticks = []
    for i in range(y_tick_count + 1):
        y = top + (plot_h * i / y_tick_count)
        grid_lines.append(
            f'<line x1="{left}" y1="{y:.2f}" x2="{left + plot_w}" y2="{y:.2f}" stroke="#dddddd" stroke-width="1"/>'
        )
        tick_value = _scale(i, y_tick_count, 0.0, y_min, y_max)
        y_ticks.append(
            f'<text x="{left - 10}" y="{y + 4:.2f}" text-anchor="end" font-family="monospace" font-size="12">{tick_value:.1f}</text>'
        )

    shift_levels = _aero_shift_levels(times, aero_apogee, max_levels=5)
    shift_lines = []
    shift_labels = []
    for idx, (shift_time, shift_level) in enumerate(shift_levels):
        y = _scale(shift_level, y_min, y_max, top + plot_h, top)
        shift_lines.append(
            f'<line x1="{left}" y1="{y:.2f}" x2="{left + plot_w}" y2="{y:.2f}" stroke="#ff7f0e" stroke-width="1.5" stroke-dasharray="7,4"/>'
        )
        label_y = y - 6 - (idx % 2) * 14
        shift_labels.append(
            f'<text x="{left + plot_w - 8}" y="{label_y:.2f}" text-anchor="end" font-family="monospace" font-size="11" fill="#b35a00">aero shift @ {shift_time:.3f}s: {shift_level:.2f}m</text>'
        )

    flap_x = _scale(flap_time, t_min, t_max, left, left + plot_w)
    flap_line = (
        f'<line x1="{flap_x:.2f}" y1="{top}" x2="{flap_x:.2f}" y2="{top + plot_h}" '
        'stroke="#000000" stroke-width="2" stroke-dasharray="10,5"/>'
    )
    flap_label = (
        f'<text x="{flap_x + 8:.2f}" y="{top + 18}" text-anchor="start" font-family="monospace" '
        f'font-size="13">Flap Open Ref: {flap_time:.3f}s</text>'
    )

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="white"/>
  <text x="{width / 2:.0f}" y="40" text-anchor="middle" font-family="monospace" font-size="26">Zoomed Replay: Altitude vs Aero vs Ballistic</text>
  <text x="{width / 2:.0f}" y="{height - 28}" text-anchor="middle" font-family="monospace" font-size="18">Time (s)</text>
  <text x="30" y="{height / 2:.0f}" text-anchor="middle" font-family="monospace" font-size="18" transform="rotate(-90 30 {height / 2:.0f})">Meters</text>
  {''.join(grid_lines)}
  {''.join(x_ticks)}
  {''.join(y_ticks)}
  <rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="none" stroke="black" stroke-width="2"/>
  {''.join(shift_lines)}
  {flap_line}
  <polyline fill="none" stroke="#1f77b4" stroke-width="1.8" points="{altitude_points}"/>
  <polyline fill="none" stroke="#d62728" stroke-width="1.8" points="{aero_points}"/>
  <polyline fill="none" stroke="#2ca02c" stroke-width="1.8" points="{ballistic_points}"/>
  {''.join(shift_labels)}
  {flap_label}
  <line x1="{left + 20}" y1="{top + 22}" x2="{left + 95}" y2="{top + 22}" stroke="#1f77b4" stroke-width="3"/>
  <text x="{left + 105}" y="{top + 28}" font-family="monospace" font-size="16">Altitude</text>
  <line x1="{left + 20}" y1="{top + 50}" x2="{left + 95}" y2="{top + 50}" stroke="#d62728" stroke-width="3"/>
  <text x="{left + 105}" y="{top + 56}" font-family="monospace" font-size="16">Aero Predictor</text>
  <line x1="{left + 20}" y1="{top + 78}" x2="{left + 95}" y2="{top + 78}" stroke="#2ca02c" stroke-width="3"/>
  <text x="{left + 105}" y="{top + 84}" font-family="monospace" font-size="16">Ballistic</text>
  <line x1="{left + 20}" y1="{top + 106}" x2="{left + 95}" y2="{top + 106}" stroke="#ff7f0e" stroke-width="2" stroke-dasharray="7,4"/>
  <text x="{left + 105}" y="{top + 112}" font-family="monospace" font-size="16">Aero Shift Level</text>
  <line x1="{left + 20}" y1="{top + 134}" x2="{left + 95}" y2="{top + 134}" stroke="#000000" stroke-width="2" stroke-dasharray="10,5"/>
  <text x="{left + 105}" y="{top + 140}" font-family="monospace" font-size="16">Flap Open Reference</text>
  <text x="{left}" y="{height - 56}" font-family="monospace" font-size="14">window=[{t_min:.3f}, {t_max:.3f}]</text>
  <text x="{left + 260}" y="{height - 56}" font-family="monospace" font-size="14">samples={len(times)}</text>
</svg>
"""

    output_path.write_text(svg)
    print(f"Saved plot to {output_path}")
    print(f"Window {t_min:.3f} to {t_max:.3f} s with {len(times)} samples.")


if __name__ == "__main__":
    main()
