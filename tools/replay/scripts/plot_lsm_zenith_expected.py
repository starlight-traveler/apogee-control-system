#!/usr/bin/env python3
"""
Plot LSM zenith diagnostics against a simple expected trajectory proxy.

This script focuses on the non-ground portion of a decoded replay CSV and
renders a standalone SVG with:
  - raw LSM zenith from quaternion in [0, 180] degrees
  - folded LSM body-axis tilt min(z, 180-z) in [0, 90] degrees
  - expected trajectory tilt from the filtered state velocity vector

The expected proxy is intentionally conservative: it is just the tilt of the
flight path away from vertical, computed from state velocity. In `fullscale_3`
that proxy is effectively 0 deg for the ascent window because the replayed
state has no horizontal velocity component.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
REPLAY_DIR = SCRIPT_DIR.parent
ROOT = REPLAY_DIR.parents[1]
PLOTS_DIR = REPLAY_DIR / "plots"

DEFAULT_INPUT_CSV = ROOT / "fullscale_3.csv"
DEFAULT_OUTPUT_SVG = PLOTS_DIR / "fullscale_3_lsm_zenith_expected.svg"


@dataclass
class Sample:
    time_s: float
    status: str
    raw_zenith_deg: float
    axis_tilt_deg: float
    expected_tilt_deg: float | None
    altitude_m: float
    vertical_velocity_mps: float


def parse_float(value: str | None) -> float | None:
    if value is None:
        return None
    text = value.strip()
    if not text:
        return None
    try:
        number = float(text)
    except ValueError:
        return None
    return number if math.isfinite(number) else None


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def quaternion_zenith_deg(qx: float, qy: float) -> float:
    cos_zenith = clamp(1.0 - 2.0 * (qx * qx + qy * qy), -1.0, 1.0)
    return math.degrees(math.acos(cos_zenith))


def axis_tilt_from_velocity_deg(vx: float, vy: float, vz: float) -> float | None:
    speed_sq = vx * vx + vy * vy + vz * vz
    if speed_sq <= 1.0:
        return None
    horizontal_speed = math.hypot(vx, vy)
    return math.degrees(math.atan2(horizontal_speed, abs(vz)))


def load_samples(path: Path) -> list[Sample]:
    samples: list[Sample] = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(line for line in handle if not line.startswith("#"))
        for row in reader:
            status = (row.get("flight_status") or "").strip().lower()
            if status == "ground":
                continue
            time_s = parse_float(row.get("sensor_timestamp"))
            qx = parse_float(row.get("sensor_lsm_quat_x"))
            qy = parse_float(row.get("sensor_lsm_quat_y"))
            altitude_m = parse_float(row.get("state_position_z"))
            vz = parse_float(row.get("state_velocity_z"))
            vx = parse_float(row.get("state_velocity_x"))
            vy = parse_float(row.get("state_velocity_y"))
            if None in (time_s, qx, qy, altitude_m, vz, vx, vy):
                continue

            raw_zenith_deg = quaternion_zenith_deg(qx, qy)
            axis_tilt_deg = min(raw_zenith_deg, 180.0 - raw_zenith_deg)
            expected_tilt_deg = axis_tilt_from_velocity_deg(vx, vy, vz)

            samples.append(
                Sample(
                    time_s=time_s,
                    status=status,
                    raw_zenith_deg=raw_zenith_deg,
                    axis_tilt_deg=axis_tilt_deg,
                    expected_tilt_deg=expected_tilt_deg,
                    altitude_m=altitude_m,
                    vertical_velocity_mps=vz,
                )
            )
    return samples


def scale(value: float, src_lo: float, src_hi: float, dst_lo: float, dst_hi: float) -> float:
    if src_hi <= src_lo:
        return 0.5 * (dst_lo + dst_hi)
    ratio = (value - src_lo) / (src_hi - src_lo)
    return dst_lo + ratio * (dst_hi - dst_lo)


def polyline_points(
    xs: list[float],
    ys: list[float],
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    left: float,
    top: float,
    width: float,
    height: float,
) -> str:
    points: list[str] = []
    for x, y in zip(xs, ys):
        if not math.isfinite(y):
            continue
        px = scale(x, x_min, x_max, left, left + width)
        py = scale(y, y_min, y_max, top + height, top)
        points.append(f"{px:.2f},{py:.2f}")
    return " ".join(points)


def phase_runs(samples: list[Sample]) -> list[tuple[str, float, float]]:
    runs: list[tuple[str, float, float]] = []
    if not samples:
        return runs
    current_status = samples[0].status
    start_time = samples[0].time_s
    for sample in samples[1:]:
        if sample.status != current_status:
            runs.append((current_status, start_time, sample.time_s))
            current_status = sample.status
            start_time = sample.time_s
    runs.append((current_status, start_time, samples[-1].time_s))
    return runs


def summarize(samples: list[Sample]) -> dict[str, float]:
    expected_values = [sample.expected_tilt_deg for sample in samples if sample.expected_tilt_deg is not None]
    diffs = [
        abs(sample.axis_tilt_deg - sample.expected_tilt_deg)
        for sample in samples
        if sample.expected_tilt_deg is not None
    ]
    horizontal_zero_fraction = 0.0
    if expected_values:
        zero_like = sum(1 for value in expected_values if abs(value) <= 1.0e-6)
        horizontal_zero_fraction = zero_like / len(expected_values)

    return {
        "start_time_s": samples[0].time_s if samples else 0.0,
        "end_time_s": samples[-1].time_s if samples else 0.0,
        "mean_axis_tilt_deg": sum(sample.axis_tilt_deg for sample in samples) / len(samples) if samples else 0.0,
        "max_axis_tilt_deg": max((sample.axis_tilt_deg for sample in samples), default=0.0),
        "mean_abs_diff_deg": sum(diffs) / len(diffs) if diffs else 0.0,
        "max_abs_diff_deg": max(diffs, default=0.0),
        "expected_zero_fraction": horizontal_zero_fraction,
    }


def draw_panel(
    title: str,
    y_label: str,
    xs: list[float],
    series: list[tuple[str, list[float], str, str]],
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    left: float,
    top: float,
    width: float,
    height: float,
    phases: list[tuple[str, float, float]],
    x_axis: bool,
) -> str:
    elements: list[str] = []
    phase_colors = {
        "burn": "#fee2e2",
        "coast": "#dbeafe",
        "overshoot": "#ede9fe",
        "descent": "#dcfce7",
    }

    for status, start_time, end_time in phases:
        phase_left = scale(start_time, x_min, x_max, left, left + width)
        phase_right = scale(end_time, x_min, x_max, left, left + width)
        elements.append(
            f'<rect x="{phase_left:.2f}" y="{top:.2f}" width="{max(1.0, phase_right - phase_left):.2f}" '
            f'height="{height:.2f}" fill="{phase_colors.get(status, "#f3f4f6")}" opacity="0.35"/>'
        )

    for index in range(6):
        gy = top + index * height / 5.0
        elements.append(
            f'<line x1="{left:.2f}" y1="{gy:.2f}" x2="{left + width:.2f}" y2="{gy:.2f}" '
            'stroke="#e5dccd" stroke-width="1"/>'
        )
        y_value = y_max - index * (y_max - y_min) / 5.0
        elements.append(
            f'<text x="{left - 12:.2f}" y="{gy + 4:.2f}" text-anchor="end" '
            'font-size="12" fill="#1f2937">'
            f"{y_value:.0f}</text>"
        )

    for index in range(8):
        gx = left + index * width / 7.0
        elements.append(
            f'<line x1="{gx:.2f}" y1="{top:.2f}" x2="{gx:.2f}" y2="{top + height:.2f}" '
            'stroke="#e5dccd" stroke-width="1"/>'
        )
        if x_axis:
            x_value = x_min + index * (x_max - x_min) / 7.0
            elements.append(
                f'<text x="{gx:.2f}" y="{top + height + 22:.2f}" text-anchor="middle" '
                'font-size="12" fill="#1f2937">'
                f"{x_value - x_min:.1f}</text>"
            )

    elements.append(
        f'<rect x="{left:.2f}" y="{top:.2f}" width="{width:.2f}" height="{height:.2f}" '
        'fill="none" stroke="#5b534a" stroke-width="1.5"/>'
    )
    elements.append(
        f'<text x="{left:.2f}" y="{top - 14:.2f}" font-size="18" font-weight="700" fill="#1f2937">{title}</text>'
    )
    elements.append(
        f'<text x="{left - 56:.2f}" y="{top + height / 2.0:.2f}" transform="rotate(-90 {left - 56:.2f} {top + height / 2.0:.2f})" '
        'text-anchor="middle" font-size="14" fill="#1f2937">'
        f"{y_label}</text>"
    )

    legend_y = top + 18.0
    for name, ys, color, dash in series:
        points = polyline_points(xs, ys, x_min, x_max, y_min, y_max, left, top, width, height)
        if points:
            dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
            elements.append(
                f'<polyline fill="none" stroke="{color}" stroke-width="2.5"{dash_attr} points="{points}"/>'
            )
        elements.append(
            f'<line x1="{left + width - 340:.2f}" y1="{legend_y:.2f}" x2="{left + width - 310:.2f}" y2="{legend_y:.2f}" '
            f'stroke="{color}" stroke-width="3"{f" stroke-dasharray={chr(34)+dash+chr(34)}" if dash else ""}/>'
        )
        elements.append(
            f'<text x="{left + width - 300:.2f}" y="{legend_y + 5:.2f}" font-size="13" fill="#1f2937">{name}</text>'
        )
        legend_y += 22.0

    return "\n".join(elements)


def write_svg(samples: list[Sample], output_path: Path) -> dict[str, float]:
    summary = summarize(samples)
    xs = [sample.time_s for sample in samples]
    raw = [sample.raw_zenith_deg for sample in samples]
    axis = [sample.axis_tilt_deg for sample in samples]
    expected = [float("nan") if sample.expected_tilt_deg is None else sample.expected_tilt_deg for sample in samples]
    altitude = [sample.altitude_m for sample in samples]
    x_min = xs[0]
    x_max = xs[-1]

    width = 1500.0
    height = 1080.0
    left = 95.0
    plot_width = 1360.0
    panel_height = 255.0
    gap = 90.0
    top1 = 90.0
    top2 = top1 + panel_height + gap
    top3 = top2 + panel_height + gap
    phases = phase_runs(samples)

    elements = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{int(width)}" height="{int(height)}" viewBox="0 0 {int(width)} {int(height)}">',
        '<rect width="100%" height="100%" fill="#fffaf3"/>',
        '<style>text{font-family:Helvetica,Arial,sans-serif}</style>',
        '<text x="95" y="42" font-size="30" font-weight="700" fill="#1f2937">LSM Zenith Vs Expected Trajectory Tilt</text>',
        (
            '<text x="95" y="68" font-size="15" fill="#374151">'
            'Expected tilt proxy = flight-path tilt from filtered state velocity. '
            'In this replay the horizontal velocity is effectively zero, so the expected proxy stays at 0 deg.'
            "</text>"
        ),
    ]

    elements.append(
        draw_panel(
            title="Raw Quaternion Zenith",
            y_label="Zenith [deg]",
            xs=xs,
            series=[("LSM raw zenith", raw, "#d97706", "")],
            x_min=x_min,
            x_max=x_max,
            y_min=0.0,
            y_max=180.0,
            left=left,
            top=top1,
            width=plot_width,
            height=panel_height,
            phases=phases,
            x_axis=False,
        )
    )
    elements.append(
        draw_panel(
            title="Rocket-Relevant Axis Tilt",
            y_label="Tilt [deg]",
            xs=xs,
            series=[
                ("LSM folded axis tilt", axis, "#2563eb", ""),
                ("Expected trajectory tilt", expected, "#111827", "8 6"),
            ],
            x_min=x_min,
            x_max=x_max,
            y_min=0.0,
            y_max=90.0,
            left=left,
            top=top2,
            width=plot_width,
            height=panel_height,
            phases=phases,
            x_axis=False,
        )
    )
    elements.append(
        draw_panel(
            title="Altitude Context",
            y_label="Altitude [m]",
            xs=xs,
            series=[("State altitude z", altitude, "#059669", "")],
            x_min=x_min,
            x_max=x_max,
            y_min=min(altitude),
            y_max=max(altitude),
            left=left,
            top=top3,
            width=plot_width,
            height=panel_height,
            phases=phases,
            x_axis=True,
        )
    )

    footer_y = top3 + panel_height + 48.0
    elements.append(
        f'<rect x="{left + plot_width - 420:.2f}" y="{footer_y - 94:.2f}" width="420" height="86" rx="8" '
        'fill="#ffffff" stroke="#d6cfc2"/>'
    )
    elements.append(
        f'<text x="{left + plot_width - 404:.2f}" y="{footer_y - 66:.2f}" font-size="14" fill="#1f2937">'
        f'Mean folded tilt: {summary["mean_axis_tilt_deg"]:.2f} deg</text>'
    )
    elements.append(
        f'<text x="{left + plot_width - 404:.2f}" y="{footer_y - 42:.2f}" font-size="14" fill="#1f2937">'
        f'Max folded tilt: {summary["max_axis_tilt_deg"]:.2f} deg</text>'
    )
    elements.append(
        f'<text x="{left + plot_width - 404:.2f}" y="{footer_y - 18:.2f}" font-size="14" fill="#1f2937">'
        f'Mean |tilt - expected|: {summary["mean_abs_diff_deg"]:.2f} deg</text>'
    )
    elements.append(
        f'<text x="{left + plot_width - 404:.2f}" y="{footer_y + 6:.2f}" font-size="14" fill="#1f2937">'
        f'Expected proxy zero-like fraction: {100.0 * summary["expected_zero_fraction"]:.1f}%</text>'
    )
    elements.append(
        f'<text x="{left + plot_width / 2.0:.2f}" y="{height - 18:.2f}" text-anchor="middle" '
        'font-size="14" fill="#1f2937">Time since non-ground window start [s]</text>'
    )
    elements.append("</svg>")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(elements))
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot LSM zenith against a simple expected trajectory tilt proxy.")
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT_CSV, help="Input decoded replay CSV.")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT_SVG, help="Output SVG path.")
    args = parser.parse_args()

    samples = load_samples(args.input)
    if not samples:
        raise SystemExit("No non-ground samples found.")
    summary = write_svg(samples, args.output)

    print(f"wrote_svg={args.output}")
    print(f"window_start_s={summary['start_time_s']:.3f}")
    print(f"window_end_s={summary['end_time_s']:.3f}")
    print(f"mean_axis_tilt_deg={summary['mean_axis_tilt_deg']:.3f}")
    print(f"max_axis_tilt_deg={summary['max_axis_tilt_deg']:.3f}")
    print(f"mean_abs_diff_deg={summary['mean_abs_diff_deg']:.3f}")
    print(f"max_abs_diff_deg={summary['max_abs_diff_deg']:.3f}")
    print(f"expected_zero_fraction={summary['expected_zero_fraction']:.6f}")


if __name__ == "__main__":
    main()
