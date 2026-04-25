#!/usr/bin/env python3
"""
Plot folded tilt magnitude from quaternions and accel-derived zenith for all rails.

Solid lines are quaternion-derived tilt magnitude.
Dashed lines are accel-derived zenith magnitude.
WT901 quaternion data is rotated into body frame using the inferred mount rotation:
    body +X = sensor -Z
    body +Y = sensor -X
    body +Z = sensor +Y
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path


from replaylib.paths import SCRIPTS_DIR
SCRIPT_DIR = SCRIPTS_DIR
DEFAULT_INPUT_CSV = SCRIPT_DIR / "fullscale4.csv"
WT901_QUAT_ROTATION = (
    (0.0, 0.0, -1.0),
    (-1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
)
RAIL_COLORS = {
    "ICM": "#22577a",
    "LSM": "#2a9d8f",
    "BNO": "#d1495b",
    "WT901": "#d17b0f",
}


@dataclass
class Sample:
    time_s: float
    status: str
    icm_quat_deg: float | None
    lsm_quat_deg: float | None
    bno_quat_deg: float | None
    wt901_quat_deg: float | None
    icm_accel_deg: float | None
    lsm_accel_deg: float | None
    bno_accel_deg: float | None
    wt901_accel_deg: float | None


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


def parse_bool(value: str | None) -> bool:
    return (value or "").strip().lower() == "true"


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize3(vector: tuple[float, float, float]) -> tuple[float, float, float] | None:
    norm = math.sqrt(sum(component * component for component in vector))
    if norm <= 0.0 or not math.isfinite(norm):
        return None
    return tuple(component / norm for component in vector)


def apply_rotation(
    matrix: tuple[tuple[float, float, float], ...],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    return tuple(
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )


def folded_tilt_from_gravity(gravity: tuple[float, float, float]) -> float:
    return math.degrees(math.acos(clamp(abs(gravity[2]), 0.0, 1.0)))


def gravity_from_quaternion(
    w: float, x: float, y: float, z: float
) -> tuple[float, float, float] | None:
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 0.0 or not math.isfinite(norm):
        return None
    w /= norm
    x /= norm
    y /= norm
    z /= norm
    return normalize3(
        (
            2.0 * (x * z - w * y),
            2.0 * (w * x + y * z),
            w * w - x * x - y * y + z * z,
        )
    )


def read_quaternion_tilt(
    raw: dict[str, str],
    prefix: str,
    valid_key: str,
    rotation: tuple[tuple[float, float, float], ...] | None = None,
) -> float | None:
    if not parse_bool(raw.get(valid_key)):
        return None
    w = parse_float(raw.get(f"{prefix}_w"))
    x = parse_float(raw.get(f"{prefix}_x"))
    y = parse_float(raw.get(f"{prefix}_y"))
    z = parse_float(raw.get(f"{prefix}_z"))
    if None in (w, x, y, z):
        return None
    gravity = gravity_from_quaternion(w, x, y, z)
    if gravity is None:
        return None
    if rotation is not None:
        gravity = apply_rotation(rotation, gravity)
    return folded_tilt_from_gravity(gravity)


def read_accel_tilt(raw: dict[str, str], prefix: str) -> float | None:
    x = parse_float(raw.get(f"sensor_accel_{prefix}_x"))
    y = parse_float(raw.get(f"sensor_accel_{prefix}_y"))
    z = parse_float(raw.get(f"sensor_accel_{prefix}_z"))
    if None in (x, y, z):
        return None
    gravity = normalize3((x, y, z))
    if gravity is None:
        return None
    return folded_tilt_from_gravity(gravity)


def load_samples(path: Path) -> list[Sample]:
    samples: list[Sample] = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(line for line in handle if not line.startswith("#"))
        for raw in reader:
            time_s = parse_float(raw.get("sensor_timestamp"))
            if time_s is None:
                continue
            samples.append(
                Sample(
                    time_s=time_s,
                    status=(raw.get("flight_status") or "").strip().lower(),
                    icm_quat_deg=read_quaternion_tilt(raw, "sensor_icm_quat", "sensor_has_icm_quaternion"),
                    lsm_quat_deg=read_quaternion_tilt(raw, "sensor_lsm_quat", "sensor_has_lsm_quaternion"),
                    bno_quat_deg=read_quaternion_tilt(raw, "sensor_bno_quat", "sensor_has_bno_quaternion"),
                    wt901_quat_deg=read_quaternion_tilt(
                        raw,
                        "sensor_wt901_quat",
                        "sensor_has_wt901_quaternion",
                        WT901_QUAT_ROTATION,
                    ),
                    icm_accel_deg=read_accel_tilt(raw, "icm"),
                    lsm_accel_deg=read_accel_tilt(raw, "lsm"),
                    bno_accel_deg=read_accel_tilt(raw, "bno"),
                    wt901_accel_deg=read_accel_tilt(raw, "wt901"),
                )
            )
    return samples


def scale(value: float, src_lo: float, src_hi: float, dst_lo: float, dst_hi: float) -> float:
    if src_hi <= src_lo:
        return 0.5 * (dst_lo + dst_hi)
    ratio = (value - src_lo) / (src_hi - src_lo)
    return dst_lo + ratio * (dst_hi - dst_lo)


def decimation_stride(sample_count: int, max_points: int) -> int:
    if sample_count <= max_points:
        return 1
    return max(1, math.ceil(sample_count / max_points))


def build_polyline(
    samples: list[Sample],
    selector,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    left: float,
    top: float,
    width: float,
    height: float,
    max_points: int,
) -> str:
    if not samples:
        return ""
    stride = decimation_stride(len(samples), max_points)
    points: list[str] = []
    bottom = top + height
    selected = samples[::stride]
    if selected[-1] is not samples[-1]:
        selected.append(samples[-1])
    for sample in selected:
        value = selector(sample)
        if value is None or not math.isfinite(value):
            continue
        x = scale(sample.time_s, x_min, x_max, left, left + width)
        y = scale(value, y_min, y_max, bottom, top)
        points.append(f"{x:.2f},{y:.2f}")
    return " ".join(points)


def phase_runs(samples: list[Sample]) -> list[tuple[str, float, float]]:
    if not samples:
        return []
    runs: list[tuple[str, float, float]] = []
    current_status = samples[0].status
    start_time = samples[0].time_s
    for sample in samples[1:]:
        if sample.status != current_status:
            runs.append((current_status, start_time, sample.time_s))
            current_status = sample.status
            start_time = sample.time_s
    runs.append((current_status, start_time, samples[-1].time_s))
    return runs


def draw_panel(
    title: str,
    samples: list[Sample],
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    left: float,
    top: float,
    width: float,
    height: float,
) -> str:
    phase_colors = {
        "ground": "#f4f1ea",
        "burn": "#ffe5d9",
        "coast": "#e6f0ff",
        "overshoot": "#efe9fb",
        "descent": "#e3f7ea",
    }
    rail_series = [
        ("ICM quat", lambda sample: sample.icm_quat_deg, RAIL_COLORS["ICM"], None),
        ("ICM accel", lambda sample: sample.icm_accel_deg, RAIL_COLORS["ICM"], "8 6"),
        ("LSM quat", lambda sample: sample.lsm_quat_deg, RAIL_COLORS["LSM"], None),
        ("LSM accel", lambda sample: sample.lsm_accel_deg, RAIL_COLORS["LSM"], "8 6"),
        ("BNO quat", lambda sample: sample.bno_quat_deg, RAIL_COLORS["BNO"], None),
        ("BNO accel", lambda sample: sample.bno_accel_deg, RAIL_COLORS["BNO"], "8 6"),
        ("WT901 quat", lambda sample: sample.wt901_quat_deg, RAIL_COLORS["WT901"], None),
        ("WT901 accel", lambda sample: sample.wt901_accel_deg, RAIL_COLORS["WT901"], "8 6"),
    ]

    elements: list[str] = []
    card_x = left - 22.0
    card_y = top - 30.0
    card_width = width + 44.0
    card_height = height + 72.0
    elements.append(
        f'<rect x="{card_x:.2f}" y="{card_y:.2f}" width="{card_width:.2f}" height="{card_height:.2f}" '
        'rx="20" fill="#ffffff" stroke="#e5ddd1" stroke-width="1.2"/>'
    )

    for status, start_time, end_time in phase_runs(samples):
        px0 = scale(start_time, x_min, x_max, left, left + width)
        px1 = scale(end_time, x_min, x_max, left, left + width)
        elements.append(
            f'<rect x="{px0:.2f}" y="{top:.2f}" width="{max(1.0, px1 - px0):.2f}" '
            f'height="{height:.2f}" fill="{phase_colors.get(status, "#f8f5ef")}" opacity="0.55"/>'
        )

    for index in range(6):
        gy = top + index * height / 5.0
        gx = left + index * width / 5.0
        y_value = y_max - index * (y_max - y_min) / 5.0
        x_value = x_min + index * (x_max - x_min) / 5.0
        elements.append(
            f'<line x1="{left:.2f}" y1="{gy:.2f}" x2="{left + width:.2f}" y2="{gy:.2f}" '
            'stroke="#e7e1d6" stroke-width="1"/>'
        )
        elements.append(
            f'<line x1="{gx:.2f}" y1="{top:.2f}" x2="{gx:.2f}" y2="{top + height:.2f}" '
            'stroke="#efe9df" stroke-width="1"/>'
        )
        elements.append(
            f'<text x="{left - 12:.2f}" y="{gy + 4:.2f}" text-anchor="end" '
            'font-size="12" fill="#4a5568">'
            f'{y_value:.1f}</text>'
        )
        elements.append(
            f'<text x="{gx:.2f}" y="{top + height + 24:.2f}" text-anchor="middle" '
            'font-size="12" fill="#4a5568">'
            f'{x_value:.2f}</text>'
        )

    for label, selector, color, dash in rail_series:
        polyline = build_polyline(
            samples,
            selector,
            x_min,
            x_max,
            y_min,
            y_max,
            left,
            top,
            width,
            height,
            max_points=5000,
        )
        if polyline:
            dash_attr = f' stroke-dasharray="{dash}"' if dash is not None else ""
            opacity = "0.72" if dash is not None else "0.96"
            width_attr = "1.3" if dash is not None else "1.8"
            elements.append(
                f'<polyline fill="none" stroke="{color}" stroke-width="{width_attr}" stroke-linecap="round" '
                f'stroke-linejoin="round" opacity="{opacity}"{dash_attr} points="{polyline}"/>'
            )

    elements.append(
        f'<rect x="{left:.2f}" y="{top:.2f}" width="{width:.2f}" height="{height:.2f}" '
        'fill="none" stroke="#4b5563" stroke-width="1.1"/>'
    )
    elements.append(
        f'<text x="{left:.2f}" y="{top - 10:.2f}" font-size="17" fill="#1f2937">{title}</text>'
    )
    elements.append(
        f'<text x="{left - 62:.2f}" y="{top + height * 0.5:.2f}" text-anchor="middle" '
        'font-size="13" fill="#374151" transform="rotate(-90 '
        f'{left - 62:.2f} {top + height * 0.5:.2f})">Tilt Magnitude (deg)</text>'
    )
    return "\n".join(elements)


def mean(values: list[float]) -> float | None:
    return None if not values else sum(values) / len(values)


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        raise ValueError("percentile of empty sequence")
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round(fraction * (len(ordered) - 1)))))
    return ordered[index]


def format_stat(label: str, values: list[float]) -> str | None:
    value = mean(values)
    if value is None:
        return None
    return f"{label}: mean={value:.2f} deg"


def summarize_flight_window(samples: list[Sample]) -> list[str]:
    lines: list[str] = []
    series = [
        ("ICM quat", [sample.icm_quat_deg for sample in samples if sample.icm_quat_deg is not None]),
        ("LSM quat", [sample.lsm_quat_deg for sample in samples if sample.lsm_quat_deg is not None]),
        ("BNO quat", [sample.bno_quat_deg for sample in samples if sample.bno_quat_deg is not None]),
        ("WT901 quat", [sample.wt901_quat_deg for sample in samples if sample.wt901_quat_deg is not None]),
        ("ICM accel", [sample.icm_accel_deg for sample in samples if sample.icm_accel_deg is not None]),
        ("LSM accel", [sample.lsm_accel_deg for sample in samples if sample.lsm_accel_deg is not None]),
        ("BNO accel", [sample.bno_accel_deg for sample in samples if sample.bno_accel_deg is not None]),
        ("WT901 accel", [sample.wt901_accel_deg for sample in samples if sample.wt901_accel_deg is not None]),
    ]
    for label, values in series:
        line = format_stat(label, values)
        if line is not None:
            lines.append(line)
    return lines


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_csv", nargs="?", type=Path, default=DEFAULT_INPUT_CSV)
    parser.add_argument("-o", "--output", type=Path)
    args = parser.parse_args()

    samples = load_samples(args.input_csv)
    if not samples:
        raise SystemExit(f"No usable samples found in {args.input_csv}")

    if args.output is None:
        args.output = args.input_csv.with_name(args.input_csv.stem + "_tilt_magnitude_comparison.svg")

    args.output.parent.mkdir(parents=True, exist_ok=True)

    all_values = [
        value
        for sample in samples
        for value in (
            sample.icm_quat_deg,
            sample.lsm_quat_deg,
            sample.bno_quat_deg,
            sample.wt901_quat_deg,
            sample.icm_accel_deg,
            sample.lsm_accel_deg,
            sample.bno_accel_deg,
            sample.wt901_accel_deg,
        )
        if value is not None and math.isfinite(value)
    ]
    if not all_values:
        raise SystemExit("No usable tilt values found.")

    y_min = 0.0
    y_max = max(30.0, min(45.0, percentile(all_values, 0.999) + 2.0))

    full_x_min = samples[0].time_s
    full_x_max = samples[-1].time_s
    non_ground = [sample for sample in samples if sample.status != "ground"]
    flight_x_min = max(full_x_min, non_ground[0].time_s - 0.5) if non_ground else full_x_min
    flight_samples = [sample for sample in samples if flight_x_min <= sample.time_s <= full_x_max]

    width = 1720
    height = 1120
    left = 120
    right = 48
    top = 82
    panel_gap = 108
    panel_height = 320
    plot_width = width - left - right

    summary_lines = summarize_flight_window(flight_samples)
    summary_svg = []
    for index, line in enumerate(summary_lines[:8]):
        column = index // 4
        row = index % 4
        summary_svg.append(
            f'<text x="{left + column * 330:.2f}" y="{top + row * 19:.2f}" font-size="13" fill="#5b6472">{line}</text>'
        )

    rail_legend_svg = []
    for index, (label, color) in enumerate(RAIL_COLORS.items()):
        x = left + 940 + index * 150
        y = top + 5
        rail_legend_svg.append(
            f'<line x1="{x:.2f}" y1="{y:.2f}" x2="{x + 28:.2f}" y2="{y:.2f}" stroke="{color}" stroke-width="2.6" stroke-linecap="round"/>'
        )
        rail_legend_svg.append(
            f'<text x="{x + 36:.2f}" y="{y + 4:.2f}" font-size="13" fill="#334155">{label}</text>'
        )

    style_legend_svg = [
        f'<line x1="{left + 940:.2f}" y1="{top + 28:.2f}" x2="{left + 968:.2f}" y2="{top + 28:.2f}" '
        'stroke="#334155" stroke-width="2.2" stroke-linecap="round"/>',
        f'<text x="{left + 976:.2f}" y="{top + 32:.2f}" font-size="13" fill="#334155">Quaternion</text>',
        f'<line x1="{left + 1088:.2f}" y1="{top + 28:.2f}" x2="{left + 1116:.2f}" y2="{top + 28:.2f}" '
        'stroke="#334155" stroke-width="2.0" stroke-dasharray="8 6" stroke-linecap="round"/>',
        f'<text x="{left + 1124:.2f}" y="{top + 32:.2f}" font-size="13" fill="#334155">Accel</text>',
    ]

    full_panel_top = top + 112
    flight_panel_top = full_panel_top + panel_height + panel_gap

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <defs>
    <linearGradient id="bg" x1="0%" y1="0%" x2="100%" y2="100%">
      <stop offset="0%" stop-color="#fff8ee"/>
      <stop offset="100%" stop-color="#f7fbff"/>
    </linearGradient>
  </defs>
  <rect width="100%" height="100%" fill="url(#bg)"/>
  <text x="{width / 2:.0f}" y="34" text-anchor="middle" font-size="28" fill="#1f2937"
        font-family="Avenir Next, Avenir, Helvetica Neue, sans-serif">
    Tilt Magnitude Comparison Across Rails
  </text>
  <text x="{width / 2:.0f}" y="58" text-anchor="middle" font-size="14" fill="#4b5563"
        font-family="Avenir Next, Avenir, Helvetica Neue, sans-serif">
    Folded tilt magnitude [0, 90] deg. WT901 quaternion rotated into body frame. Y-axis scaled to the 99.5th percentile for readability.
  </text>
  {''.join(summary_svg)}
  {''.join(rail_legend_svg)}
  {''.join(style_legend_svg)}
  {draw_panel('Full Mission', samples, full_x_min, full_x_max, y_min, y_max, left, full_panel_top, plot_width, panel_height)}
  {draw_panel('Flight Window', flight_samples, flight_x_min, full_x_max, y_min, y_max, left, flight_panel_top, plot_width, panel_height)}
  <text x="{left + plot_width / 2:.2f}" y="{flight_panel_top + panel_height + 56:.2f}" text-anchor="middle" font-size="14" fill="#374151">
    Time (s)
  </text>
</svg>
"""

    args.output.write_text(svg, encoding="utf-8")
    print(f"Saved tilt comparison plot to {args.output}")
    for line in summary_lines:
        print(line)
    print(f"Y-axis max used: {y_max:.2f} deg")


if __name__ == "__main__":
    main()
