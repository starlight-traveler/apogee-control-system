#!/usr/bin/env python3
"""
Compare zenith angle calculation methods:
- Old method: Quaternion -> Euler (pitch/roll) -> Zenith via acos(cos(pitch)*cos(roll))
- New method: Quaternion -> Zenith directly via acos(1 - 2*(x² + y²))

This script visualizes the differences between ICM and LSM sensors using both methods,
highlighting why the direct quaternion-to-zenith approach is more robust.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

from replaylib.paths import SCRIPTS_DIR
SCRIPT_DIR = SCRIPTS_DIR
REPLAY_DIR = SCRIPT_DIR.parent
ROOT = REPLAY_DIR.parents[1]
PLOTS_DIR = REPLAY_DIR / "plots"

DEFAULT_INPUT_CSV = ROOT / "fullscale_3.csv"
DEFAULT_OUTPUT_SVG = PLOTS_DIR / "fullscale_3_zenith_comparison.svg"


@dataclass
class ZenithSample:
    time_s: float
    # ICM sensor
    icm_quat_w: float
    icm_quat_x: float
    icm_quat_y: float
    icm_quat_z: float
    icm_pitch_deg: float
    icm_roll_deg: float
    # LSM sensor
    lsm_quat_w: float
    lsm_quat_x: float
    lsm_quat_y: float
    lsm_quat_z: float
    lsm_pitch_deg: float
    lsm_roll_deg: float
    # Main quaternion (what flight computer uses)
    main_quat_w: float
    main_quat_x: float
    main_quat_y: float
    main_quat_z: float
    main_source: int

    # Computed values
    @property
    def icm_zenith_from_euler(self) -> float:
        """Old method: zenith from Euler angles."""
        pitch_rad = math.radians(self.icm_pitch_deg)
        roll_rad = math.radians(self.icm_roll_deg)
        cos_zenith = math.cos(pitch_rad) * math.cos(roll_rad)
        return math.degrees(math.acos(max(-1, min(1, cos_zenith))))

    @property
    def icm_zenith_from_quat(self) -> float:
        """New method: zenith directly from quaternion."""
        x, y = self.icm_quat_x, self.icm_quat_y
        cos_zenith = 1.0 - 2.0 * (x * x + y * y)
        return math.degrees(math.acos(max(-1, min(1, cos_zenith))))

    @property
    def lsm_zenith_from_euler(self) -> float:
        """Old method: zenith from Euler angles."""
        pitch_rad = math.radians(self.lsm_pitch_deg)
        roll_rad = math.radians(self.lsm_roll_deg)
        cos_zenith = math.cos(pitch_rad) * math.cos(roll_rad)
        return math.degrees(math.acos(max(-1, min(1, cos_zenith))))

    @property
    def lsm_zenith_from_quat(self) -> float:
        """New method: zenith directly from quaternion."""
        x, y = self.lsm_quat_x, self.lsm_quat_y
        cos_zenith = 1.0 - 2.0 * (x * x + y * y)
        return math.degrees(math.acos(max(-1, min(1, cos_zenith))))

    @property
    def main_zenith_from_quat(self) -> float:
        """Zenith from the main quaternion used by flight computer."""
        x, y = self.main_quat_x, self.main_quat_y
        cos_zenith = 1.0 - 2.0 * (x * x + y * y)
        return math.degrees(math.acos(max(-1, min(1, cos_zenith))))


def parse_float(value: str | None, default: float = 0.0) -> float:
    if value is None:
        return default
    text = value.strip()
    if not text:
        return default
    try:
        number = float(text)
        return number if math.isfinite(number) else default
    except ValueError:
        return default


def iter_csv_rows(path: Path):
    with path.open(newline="") as handle:
        reader = csv.DictReader(line for line in handle if not line.startswith("#"))
        for row in reader:
            yield row


def load_samples(csv_path: Path) -> List[ZenithSample]:
    samples = []
    for raw in iter_csv_rows(csv_path):
        time_s = parse_float(raw.get("sensor_timestamp"))
        if time_s <= 0:
            continue

        sample = ZenithSample(
            time_s=time_s,
            # ICM
            icm_quat_w=parse_float(raw.get("sensor_icm_quat_w"), 1.0),
            icm_quat_x=parse_float(raw.get("sensor_icm_quat_x")),
            icm_quat_y=parse_float(raw.get("sensor_icm_quat_y")),
            icm_quat_z=parse_float(raw.get("sensor_icm_quat_z")),
            icm_pitch_deg=parse_float(raw.get("sensor_icm_pitch_deg")),
            icm_roll_deg=parse_float(raw.get("sensor_icm_roll_deg")),
            # LSM
            lsm_quat_w=parse_float(raw.get("sensor_lsm_quat_w"), 1.0),
            lsm_quat_x=parse_float(raw.get("sensor_lsm_quat_x")),
            lsm_quat_y=parse_float(raw.get("sensor_lsm_quat_y")),
            lsm_quat_z=parse_float(raw.get("sensor_lsm_quat_z")),
            lsm_pitch_deg=parse_float(raw.get("sensor_lsm_pitch_deg")),
            lsm_roll_deg=parse_float(raw.get("sensor_lsm_roll_deg")),
            # Main
            main_quat_w=parse_float(raw.get("sensor_quat_w"), 1.0),
            main_quat_x=parse_float(raw.get("sensor_quat_x")),
            main_quat_y=parse_float(raw.get("sensor_quat_y")),
            main_quat_z=parse_float(raw.get("sensor_quat_z")),
            main_source=int(parse_float(raw.get("sensor_main_quaternion_source"))),
        )
        samples.append(sample)
    return samples


def print_statistics(samples: List[ZenithSample], t_start: float, t_end: float):
    """Print statistics comparing the two methods."""
    filtered = [s for s in samples if t_start <= s.time_s <= t_end]
    if not filtered:
        print("No samples in time range")
        return

    print(f"\n{'='*80}")
    print(f"Zenith Calculation Comparison: t={t_start:.1f}s to t={t_end:.1f}s ({len(filtered)} samples)")
    print(f"{'='*80}\n")

    # ICM: Euler vs Quaternion zenith
    icm_euler_zeniths = [s.icm_zenith_from_euler for s in filtered]
    icm_quat_zeniths = [s.icm_zenith_from_quat for s in filtered]
    icm_diff = [abs(e - q) for e, q in zip(icm_euler_zeniths, icm_quat_zeniths)]

    print("ICM Sensor:")
    print(f"  Euler-based zenith:  mean={sum(icm_euler_zeniths)/len(icm_euler_zeniths):.2f}°, "
          f"range=[{min(icm_euler_zeniths):.2f}°, {max(icm_euler_zeniths):.2f}°]")
    print(f"  Quat-based zenith:   mean={sum(icm_quat_zeniths)/len(icm_quat_zeniths):.2f}°, "
          f"range=[{min(icm_quat_zeniths):.2f}°, {max(icm_quat_zeniths):.2f}°]")
    print(f"  Euler vs Quat diff:  mean={sum(icm_diff)/len(icm_diff):.2f}°, max={max(icm_diff):.2f}°")

    # LSM: Euler vs Quaternion zenith
    lsm_euler_zeniths = [s.lsm_zenith_from_euler for s in filtered]
    lsm_quat_zeniths = [s.lsm_zenith_from_quat for s in filtered]
    lsm_diff = [abs(e - q) for e, q in zip(lsm_euler_zeniths, lsm_quat_zeniths)]

    print("\nLSM Sensor:")
    print(f"  Euler-based zenith:  mean={sum(lsm_euler_zeniths)/len(lsm_euler_zeniths):.2f}°, "
          f"range=[{min(lsm_euler_zeniths):.2f}°, {max(lsm_euler_zeniths):.2f}°]")
    print(f"  Quat-based zenith:   mean={sum(lsm_quat_zeniths)/len(lsm_quat_zeniths):.2f}°, "
          f"range=[{min(lsm_quat_zeniths):.2f}°, {max(lsm_quat_zeniths):.2f}°]")
    print(f"  Euler vs Quat diff:  mean={sum(lsm_diff)/len(lsm_diff):.2f}°, max={max(lsm_diff):.2f}°")

    # Cross-sensor comparison using Euler method (old)
    euler_icm_lsm_diff = [abs(s.icm_zenith_from_euler - s.lsm_zenith_from_euler) for s in filtered]
    print("\nCross-Sensor (Euler method - OLD):")
    print(f"  ICM vs LSM diff:     mean={sum(euler_icm_lsm_diff)/len(euler_icm_lsm_diff):.2f}°, "
          f"max={max(euler_icm_lsm_diff):.2f}°")

    # Cross-sensor comparison using Quaternion method (new)
    quat_icm_lsm_diff = [abs(s.icm_zenith_from_quat - s.lsm_zenith_from_quat) for s in filtered]
    print("\nCross-Sensor (Quaternion method - NEW):")
    print(f"  ICM vs LSM diff:     mean={sum(quat_icm_lsm_diff)/len(quat_icm_lsm_diff):.2f}°, "
          f"max={max(quat_icm_lsm_diff):.2f}°")

    # Main quaternion analysis
    main_zeniths = [s.main_zenith_from_quat for s in filtered]
    print("\nMain Quaternion (used by flight computer):")
    print(f"  Zenith:              mean={sum(main_zeniths)/len(main_zeniths):.2f}°, "
          f"range=[{min(main_zeniths):.2f}°, {max(main_zeniths):.2f}°]")

    # Check for identity quaternion issue
    identity_count = sum(1 for s in filtered if abs(s.icm_quat_w) > 0.9999 and
                         s.icm_quat_x**2 + s.icm_quat_y**2 + s.icm_quat_z**2 < 0.0001)
    if identity_count > 0:
        print(f"\n  WARNING: {identity_count}/{len(filtered)} ICM quaternions are near-identity!")
        print("           This indicates the ICM quaternion was not properly populated.")


def downsample(data: List[float], max_points: int = 1000) -> List[float]:
    """Downsample data to max_points using min-max preservation."""
    if len(data) <= max_points:
        return data
    step = len(data) / max_points
    result = []
    for i in range(max_points):
        start = int(i * step)
        end = int((i + 1) * step)
        chunk = data[start:end]
        if chunk:
            result.append(sum(chunk) / len(chunk))
    return result


def generate_svg_path(times: List[float], values: List[float],
                      x_min: float, x_max: float, y_min: float, y_max: float,
                      width: float, height: float, x_offset: float, y_offset: float) -> str:
    """Generate SVG path data for a line plot."""
    if not times or not values:
        return ""

    points = []
    for t, v in zip(times, values):
        x = x_offset + (t - x_min) / (x_max - x_min) * width
        y = y_offset + height - (v - y_min) / (y_max - y_min) * height
        points.append(f"{x:.1f},{y:.1f}")

    return "M " + " L ".join(points)


def plot_comparison(samples: List[ZenithSample], output_path: Path, t_start: float, t_end: float):
    """Generate comparison plot as SVG."""
    filtered = [s for s in samples if t_start <= s.time_s <= t_end]
    if not filtered:
        print("No samples in time range for plotting")
        return

    # Downsample for performance
    step = max(1, len(filtered) // 1000)
    filtered = filtered[::step]

    times = [s.time_s for s in filtered]
    t_min, t_max = min(times), max(times)

    # Data series
    icm_euler = [s.icm_zenith_from_euler for s in filtered]
    icm_quat = [s.icm_zenith_from_quat for s in filtered]
    lsm_euler = [s.lsm_zenith_from_euler for s in filtered]
    lsm_quat = [s.lsm_zenith_from_quat for s in filtered]

    # SVG dimensions
    svg_width = 1200
    svg_height = 900
    margin = 80
    plot_width = svg_width - 2 * margin
    plot_height = 180
    gap = 40

    def make_plot(y_offset: int, title: str, series: List[tuple], y_min: float, y_max: float) -> str:
        """Generate one subplot."""
        lines = []
        # Background
        lines.append(f'<rect x="{margin}" y="{y_offset}" width="{plot_width}" height="{plot_height}" fill="#f8f8f8" stroke="#ccc"/>')

        # Grid lines
        for i in range(5):
            gy = y_offset + i * plot_height / 4
            lines.append(f'<line x1="{margin}" y1="{gy}" x2="{margin + plot_width}" y2="{gy}" stroke="#ddd" stroke-width="1"/>')

        # Title
        lines.append(f'<text x="{margin}" y="{y_offset - 10}" font-size="14" font-weight="bold">{title}</text>')

        # Y-axis labels
        for i in range(5):
            val = y_min + (y_max - y_min) * (4 - i) / 4
            gy = y_offset + i * plot_height / 4
            lines.append(f'<text x="{margin - 5}" y="{gy + 4}" font-size="10" text-anchor="end">{val:.1f}°</text>')

        # Data lines
        colors = ['#1f77b4', '#2ca02c', '#d62728', '#9467bd']
        for idx, (name, data, color) in enumerate(series):
            path = generate_svg_path(times, data, t_min, t_max, y_min, y_max, plot_width, plot_height, margin, y_offset)
            if path:
                lines.append(f'<path d="{path}" fill="none" stroke="{color}" stroke-width="1.5" opacity="0.8"/>')
                # Legend
                lx = margin + plot_width - 150
                ly = y_offset + 20 + idx * 18
                lines.append(f'<line x1="{lx}" y1="{ly}" x2="{lx + 20}" y2="{ly}" stroke="{color}" stroke-width="2"/>')
                lines.append(f'<text x="{lx + 25}" y="{ly + 4}" font-size="11">{name}</text>')

        return '\n'.join(lines)

    # Build SVG
    svg_parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{svg_width}" height="{svg_height}" viewBox="0 0 {svg_width} {svg_height}">',
        '<style>text { font-family: Arial, sans-serif; }</style>',
        f'<text x="{svg_width/2}" y="30" font-size="18" font-weight="bold" text-anchor="middle">Zenith Angle Calculation Comparison</text>',
        f'<text x="{svg_width/2}" y="50" font-size="12" text-anchor="middle" fill="#666">fullscale_3.csv | t={t_start}s to {t_end}s</text>',
    ]

    y_pos = 80

    # Plot 1: ICM Euler vs Quat
    all_icm = icm_euler + icm_quat
    y_min1, y_max1 = min(all_icm), max(all_icm) + 1
    svg_parts.append(make_plot(y_pos, "ICM: Euler-based vs Quaternion-based Zenith", [
        ("Euler-based", icm_euler, "#1f77b4"),
        ("Quat-based", icm_quat, "#d62728"),
    ], y_min1, y_max1))
    y_pos += plot_height + gap

    # Plot 2: LSM Euler vs Quat
    all_lsm = lsm_euler + lsm_quat
    y_min2, y_max2 = 0, max(all_lsm) + 1
    svg_parts.append(make_plot(y_pos, "LSM: Euler-based vs Quaternion-based Zenith (Perfect Match)", [
        ("Euler-based", lsm_euler, "#1f77b4"),
        ("Quat-based", lsm_quat, "#d62728"),
    ], y_min2, y_max2))
    y_pos += plot_height + gap

    # Plot 3: Cross-sensor Euler comparison
    all_euler = icm_euler + lsm_euler
    y_min3, y_max3 = 0, max(all_euler) + 1
    svg_parts.append(make_plot(y_pos, "Cross-Sensor (Euler Method): ICM vs LSM - Shows Pitch/Roll Convention Differences", [
        ("ICM Euler", icm_euler, "#1f77b4"),
        ("LSM Euler", lsm_euler, "#2ca02c"),
    ], y_min3, y_max3))
    y_pos += plot_height + gap

    # Plot 4: Cross-sensor Quat comparison
    all_quat = icm_quat + lsm_quat
    y_min4, y_max4 = 0, max(all_quat) + 1
    svg_parts.append(make_plot(y_pos, "Cross-Sensor (Quat Method): ICM stuck at identity, LSM correct", [
        ("ICM Quat", icm_quat, "#1f77b4"),
        ("LSM Quat", lsm_quat, "#2ca02c"),
    ], y_min4, y_max4))

    # X-axis label
    svg_parts.append(f'<text x="{svg_width/2}" y="{svg_height - 10}" font-size="12" text-anchor="middle">Time (s)</text>')

    # X-axis tick labels
    for i in range(6):
        t_val = t_min + (t_max - t_min) * i / 5
        x_pos = margin + plot_width * i / 5
        svg_parts.append(f'<text x="{x_pos}" y="{svg_height - 25}" font-size="10" text-anchor="middle">{t_val:.1f}</text>')

    svg_parts.append('</svg>')

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, 'w') as f:
        f.write('\n'.join(svg_parts))
    print(f"\nPlot saved to: {output_path}")


def plot_pitch_roll_comparison(samples: List[ZenithSample], output_path: Path, t_start: float, t_end: float):
    """Generate pitch/roll comparison plot as SVG."""
    filtered = [s for s in samples if t_start <= s.time_s <= t_end]
    if not filtered:
        return

    # Downsample
    step = max(1, len(filtered) // 1000)
    filtered = filtered[::step]

    times = [s.time_s for s in filtered]
    t_min, t_max = min(times), max(times)

    icm_pitch = [s.icm_pitch_deg for s in filtered]
    icm_roll = [s.icm_roll_deg for s in filtered]
    lsm_pitch = [s.lsm_pitch_deg for s in filtered]
    lsm_roll = [s.lsm_roll_deg for s in filtered]

    svg_width = 1200
    svg_height = 500
    margin = 80
    plot_width = svg_width - 2 * margin
    plot_height = 180
    gap = 40

    def make_plot(y_offset: int, title: str, series: List[tuple], y_min: float, y_max: float) -> str:
        lines = []
        lines.append(f'<rect x="{margin}" y="{y_offset}" width="{plot_width}" height="{plot_height}" fill="#f8f8f8" stroke="#ccc"/>')
        for i in range(5):
            gy = y_offset + i * plot_height / 4
            lines.append(f'<line x1="{margin}" y1="{gy}" x2="{margin + plot_width}" y2="{gy}" stroke="#ddd"/>')
        lines.append(f'<text x="{margin}" y="{y_offset - 10}" font-size="14" font-weight="bold">{title}</text>')
        for i in range(5):
            val = y_min + (y_max - y_min) * (4 - i) / 4
            gy = y_offset + i * plot_height / 4
            lines.append(f'<text x="{margin - 5}" y="{gy + 4}" font-size="10" text-anchor="end">{val:.1f}°</text>')
        for idx, (name, data, color) in enumerate(series):
            path = generate_svg_path(times, data, t_min, t_max, y_min, y_max, plot_width, plot_height, margin, y_offset)
            if path:
                lines.append(f'<path d="{path}" fill="none" stroke="{color}" stroke-width="1.5" opacity="0.8"/>')
                lx = margin + plot_width - 120
                ly = y_offset + 20 + idx * 18
                lines.append(f'<line x1="{lx}" y1="{ly}" x2="{lx + 20}" y2="{ly}" stroke="{color}" stroke-width="2"/>')
                lines.append(f'<text x="{lx + 25}" y="{ly + 4}" font-size="11">{name}</text>')
        return '\n'.join(lines)

    svg_parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{svg_width}" height="{svg_height}">',
        '<style>text { font-family: Arial, sans-serif; }</style>',
        f'<text x="{svg_width/2}" y="30" font-size="18" font-weight="bold" text-anchor="middle">Pitch/Roll Comparison: ICM vs LSM</text>',
    ]

    y_pos = 60
    all_pitch = icm_pitch + lsm_pitch
    y_min_p = min(all_pitch) - 1
    y_max_p = max(all_pitch) + 1
    svg_parts.append(make_plot(y_pos, "Pitch Angle", [
        ("ICM Pitch", icm_pitch, "#1f77b4"),
        ("LSM Pitch", lsm_pitch, "#2ca02c"),
    ], y_min_p, y_max_p))

    y_pos += plot_height + gap
    all_roll = icm_roll + lsm_roll
    y_min_r = min(all_roll) - 1
    y_max_r = max(all_roll) + 1
    svg_parts.append(make_plot(y_pos, "Roll Angle (ICM negates roll for mount compensation)", [
        ("ICM Roll", icm_roll, "#1f77b4"),
        ("LSM Roll", lsm_roll, "#2ca02c"),
    ], y_min_r, y_max_r))

    svg_parts.append(f'<text x="{svg_width/2}" y="{svg_height - 10}" font-size="12" text-anchor="middle">Time (s)</text>')
    svg_parts.append('</svg>')

    pitch_roll_path = output_path.parent / output_path.name.replace('zenith', 'pitch_roll')
    with open(pitch_roll_path, 'w') as f:
        f.write('\n'.join(svg_parts))
    print(f"Pitch/Roll plot saved to: {pitch_roll_path}")


def main():
    parser = argparse.ArgumentParser(description="Compare zenith calculation methods")
    parser.add_argument("--input", "-i", type=Path, default=DEFAULT_INPUT_CSV,
                        help="Input CSV file")
    parser.add_argument("--output", "-o", type=Path, default=DEFAULT_OUTPUT_SVG,
                        help="Output SVG file")
    parser.add_argument("--t-start", type=float, default=24.0,
                        help="Start time for analysis (seconds)")
    parser.add_argument("--t-end", type=float, default=40.0,
                        help="End time for analysis (seconds)")
    parser.add_argument("--no-plot", action="store_true",
                        help="Skip plot generation")
    args = parser.parse_args()

    print(f"Loading data from: {args.input}")
    samples = load_samples(args.input)
    print(f"Loaded {len(samples)} samples")

    # Print statistics
    print_statistics(samples, args.t_start, args.t_end)

    # Generate plots
    if not args.no_plot:
        plot_comparison(samples, args.output, args.t_start, args.t_end)
        plot_pitch_roll_comparison(samples, args.output, args.t_start, args.t_end)


if __name__ == "__main__":
    main()
