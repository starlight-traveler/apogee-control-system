#!/usr/bin/env python3
"""
Compare raw and mount-corrected WT901 zenith against the trusted rails.

The WT901 accel/gyro path in firmware already uses a fixed sensor-to-body mapping,
but the logged quaternion is left in the sensor frame. This script applies the
best constant body-frame rotation found from the flight data:

    body +X = sensor -Z
    body +Y = sensor -X
    body +Z = sensor +Y

That is a proper rotation:
    [[ 0,  0, -1],
     [-1,  0,  0],
     [ 0,  1,  0]]

The unfolded plot shows whether the rail flips upside-down.
The folded plot shows tilt magnitude only, ignoring a 180-degree ambiguity.
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

# Best proper WT901 quaternion mount rotation inferred from the flight data.
WT901_QUAT_ROTATION = (
    (0.0, 0.0, -1.0),
    (-1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
)


@dataclass
class Sample:
    time_s: float
    status: str
    icm_unfolded_deg: float | None
    lsm_unfolded_deg: float | None
    bno_unfolded_deg: float | None
    wt_raw_unfolded_deg: float | None
    wt_rotated_unfolded_deg: float | None
    wt_rotated_folded_deg: float | None


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


def quaternion_gravity_vector(
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


def apply_rotation(
    matrix: tuple[tuple[float, float, float], ...],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    return tuple(
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )


def zenith_from_gravity(gravity: tuple[float, float, float]) -> float:
    return math.degrees(math.acos(clamp(gravity[2], -1.0, 1.0)))


def folded(angle_deg: float) -> float:
    return min(angle_deg, 180.0 - angle_deg)


def read_quaternion_gravity(
    raw: dict[str, str], prefix: str, valid_key: str
) -> tuple[float, float, float] | None:
    if not parse_bool(raw.get(valid_key)):
        return None
    w = parse_float(raw.get(f"{prefix}_w"))
    x = parse_float(raw.get(f"{prefix}_x"))
    y = parse_float(raw.get(f"{prefix}_y"))
    z = parse_float(raw.get(f"{prefix}_z"))
    if None in (w, x, y, z):
        return None
    return quaternion_gravity_vector(w, x, y, z)


def load_samples(path: Path) -> list[Sample]:
    samples: list[Sample] = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(line for line in handle if not line.startswith("#"))
        for raw in reader:
            time_s = parse_float(raw.get("sensor_timestamp"))
            if time_s is None:
                continue
            status = (raw.get("flight_status") or "").strip().lower()
            icm_gravity = read_quaternion_gravity(raw, "sensor_icm_quat", "sensor_has_icm_quaternion")
            lsm_gravity = read_quaternion_gravity(raw, "sensor_lsm_quat", "sensor_has_lsm_quaternion")
            bno_gravity = read_quaternion_gravity(raw, "sensor_bno_quat", "sensor_has_bno_quaternion")
            wt_gravity = read_quaternion_gravity(raw, "sensor_wt901_quat", "sensor_has_wt901_quaternion")

            wt_rotated_unfolded_deg = None
            wt_rotated_folded_deg = None
            if wt_gravity is not None:
                wt_rotated = apply_rotation(WT901_QUAT_ROTATION, wt_gravity)
                wt_rotated_unfolded_deg = zenith_from_gravity(wt_rotated)
                wt_rotated_folded_deg = folded(wt_rotated_unfolded_deg)

            samples.append(
                Sample(
                    time_s=time_s,
                    status=status,
                    icm_unfolded_deg=zenith_from_gravity(icm_gravity) if icm_gravity is not None else None,
                    lsm_unfolded_deg=zenith_from_gravity(lsm_gravity) if lsm_gravity is not None else None,
                    bno_unfolded_deg=zenith_from_gravity(bno_gravity) if bno_gravity is not None else None,
                    wt_raw_unfolded_deg=zenith_from_gravity(wt_gravity) if wt_gravity is not None else None,
                    wt_rotated_unfolded_deg=wt_rotated_unfolded_deg,
                    wt_rotated_folded_deg=wt_rotated_folded_deg,
                )
            )
    return samples


def scale(value: float, src_lo: float, src_hi: float, dst_lo: float, dst_hi: float) -> float:
    if src_hi <= src_lo:
        return 0.5 * (dst_lo + dst_hi)
    ratio = (value - src_lo) / (src_hi - src_lo)
    return dst_lo + ratio * (dst_hi - dst_lo)


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
) -> str:
    points: list[str] = []
    bottom = top + height
    for sample in samples:
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
    series: list[tuple[str, object, str]],
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    left: float,
    top: float,
    width: float,
    height: float,
    y_label: str,
) -> str:
    phase_colors = {
        "ground": "#f3f4f6",
        "burn": "#fee2e2",
        "coast": "#dbeafe",
        "overshoot": "#ede9fe",
        "descent": "#dcfce7",
    }
    elements: list[str] = []

    for status, start_time, end_time in phase_runs(samples):
        px0 = scale(start_time, x_min, x_max, left, left + width)
        px1 = scale(end_time, x_min, x_max, left, left + width)
        elements.append(
            f'<rect x="{px0:.2f}" y="{top:.2f}" width="{max(1.0, px1 - px0):.2f}" '
            f'height="{height:.2f}" fill="{phase_colors.get(status, "#f3f4f6")}" opacity="0.35"/>'
        )

    for index in range(6):
        gy = top + index * height / 5.0
        gx = left + index * width / 5.0
        y_value = y_max - index * (y_max - y_min) / 5.0
        x_value = x_min + index * (x_max - x_min) / 5.0
        elements.append(
            f'<line x1="{left:.2f}" y1="{gy:.2f}" x2="{left + width:.2f}" y2="{gy:.2f}" '
            'stroke="#e5e7eb" stroke-width="1"/>'
        )
        elements.append(
            f'<line x1="{gx:.2f}" y1="{top:.2f}" x2="{gx:.2f}" y2="{top + height:.2f}" '
            'stroke="#e5e7eb" stroke-width="1"/>'
        )
        elements.append(
            f'<text x="{left - 10:.2f}" y="{gy + 4:.2f}" text-anchor="end" '
            'font-size="12" fill="#111827">'
            f'{y_value:.2f}</text>'
        )
        elements.append(
            f'<text x="{gx:.2f}" y="{top + height + 22:.2f}" text-anchor="middle" '
            'font-size="12" fill="#111827">'
            f'{x_value:.2f}</text>'
        )

    legend_y = top + 16.0
    legend_x = left + 12.0
    for index, (label, selector, color) in enumerate(series):
        polyline = build_polyline(samples, selector, x_min, x_max, y_min, y_max, left, top, width, height)
        if polyline:
            elements.append(
                f'<polyline fill="none" stroke="{color}" stroke-width="1.2" points="{polyline}"/>'
            )
        line_y = legend_y + index * 20.0
        elements.append(
            f'<line x1="{legend_x:.2f}" y1="{line_y:.2f}" x2="{legend_x + 30:.2f}" y2="{line_y:.2f}" '
            f'stroke="{color}" stroke-width="2"/>'
        )
        elements.append(
            f'<text x="{legend_x + 38:.2f}" y="{line_y + 4:.2f}" font-size="13" fill="#111827">{label}</text>'
        )

    elements.append(
        f'<rect x="{left:.2f}" y="{top:.2f}" width="{width:.2f}" height="{height:.2f}" '
        'fill="none" stroke="#111827" stroke-width="1.5"/>'
    )
    elements.append(f'<text x="{left:.2f}" y="{top - 12:.2f}" font-size="16" fill="#111827">{title}</text>')
    elements.append(
        f'<text x="{left - 56:.2f}" y="{top + height * 0.5:.2f}" text-anchor="middle" '
        'font-size="13" fill="#111827" transform="rotate(-90 '
        f'{left - 56:.2f} {top + height * 0.5:.2f})">{y_label}</text>'
    )
    return "\n".join(elements)


def mean(values: list[float]) -> float | None:
    return None if not values else sum(values) / len(values)


def format_mean(label: str, values: list[float]) -> str | None:
    value = mean(values)
    if value is None:
        return None
    return f"{label}: mean={value:.4f} deg, min={min(values):.4f}, max={max(values):.4f}"


def summarize(samples: list[Sample]) -> list[str]:
    lines: list[str] = []
    series = [
        ("ICM raw", [sample.icm_unfolded_deg for sample in samples if sample.icm_unfolded_deg is not None]),
        ("LSM raw", [sample.lsm_unfolded_deg for sample in samples if sample.lsm_unfolded_deg is not None]),
        ("BNO raw", [sample.bno_unfolded_deg for sample in samples if sample.bno_unfolded_deg is not None]),
        ("WT901 raw", [sample.wt_raw_unfolded_deg for sample in samples if sample.wt_raw_unfolded_deg is not None]),
        (
            "WT901 rotated",
            [sample.wt_rotated_unfolded_deg for sample in samples if sample.wt_rotated_unfolded_deg is not None],
        ),
        (
            "WT901 rotated folded",
            [sample.wt_rotated_folded_deg for sample in samples if sample.wt_rotated_folded_deg is not None],
        ),
    ]
    for label, values in series:
        line = format_mean(label, values)
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
        args.output = args.input_csv.with_name(args.input_csv.stem + "_wt901_rotated_zenith.svg")

    args.output.parent.mkdir(parents=True, exist_ok=True)

    unfolded_values = [
        value
        for sample in samples
        for value in (
            sample.icm_unfolded_deg,
            sample.lsm_unfolded_deg,
            sample.bno_unfolded_deg,
            sample.wt_raw_unfolded_deg,
            sample.wt_rotated_unfolded_deg,
        )
        if value is not None and math.isfinite(value)
    ]
    folded_values = [
        value
        for sample in samples
        for value in (
            sample.icm_unfolded_deg,
            sample.lsm_unfolded_deg,
            sample.bno_unfolded_deg,
            sample.wt_rotated_folded_deg,
        )
        if value is not None and math.isfinite(value)
    ]
    if not unfolded_values or not folded_values:
        raise SystemExit("No valid WT901 zenith values found.")

    unfolded_y_min = min(unfolded_values)
    unfolded_y_max = max(unfolded_values)
    unfolded_pad = max(0.5, 0.08 * (unfolded_y_max - unfolded_y_min if unfolded_y_max > unfolded_y_min else 1.0))
    unfolded_y_min -= unfolded_pad
    unfolded_y_max += unfolded_pad

    folded_y_min = 0.0
    folded_y_max = max(folded_values) + 2.0

    full_x_min = samples[0].time_s
    full_x_max = samples[-1].time_s
    non_ground = [sample for sample in samples if sample.status != "ground"]
    flight_x_min = max(full_x_min, non_ground[0].time_s - 0.5) if non_ground else full_x_min
    flight_samples = [sample for sample in samples if flight_x_min <= sample.time_s <= full_x_max]

    width = 1600
    height = 1600
    left = 120
    right = 40
    top = 90
    panel_gap = 90
    panel_height = 300
    plot_width = width - left - right

    summary_lines = summarize(samples)
    summary_svg = []
    for index, line in enumerate(summary_lines):
        summary_svg.append(
            f'<text x="{left:.2f}" y="{top + index * 18:.2f}" font-size="13" fill="#374151">{line}</text>'
        )

    full_unfold_top = top + 110
    flight_unfold_top = full_unfold_top + panel_height + panel_gap
    full_fold_top = flight_unfold_top + panel_height + panel_gap
    flight_fold_top = full_fold_top + panel_height + panel_gap

    unfolded_series = [
        ("ICM", lambda sample: sample.icm_unfolded_deg, "#1d4ed8"),
        ("LSM", lambda sample: sample.lsm_unfolded_deg, "#059669"),
        ("BNO", lambda sample: sample.bno_unfolded_deg, "#dc2626"),
        ("WT901 raw", lambda sample: sample.wt_raw_unfolded_deg, "#7c3aed"),
        ("WT901 rotated", lambda sample: sample.wt_rotated_unfolded_deg, "#ea580c"),
    ]
    folded_series = [
        ("ICM", lambda sample: sample.icm_unfolded_deg, "#1d4ed8"),
        ("LSM", lambda sample: sample.lsm_unfolded_deg, "#059669"),
        ("BNO", lambda sample: sample.bno_unfolded_deg, "#dc2626"),
        ("WT901 rotated folded", lambda sample: sample.wt_rotated_folded_deg, "#ea580c"),
    ]

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="#fffdf8"/>
  <text x="{width / 2:.0f}" y="34" text-anchor="middle" font-size="24" fill="#111827">
    WT901 Quaternion Zenith With Mount Rotation
  </text>
  <text x="{width / 2:.0f}" y="58" text-anchor="middle" font-size="14" fill="#374151">
    Applied rotation: body +X = sensor -Z, body +Y = sensor -X, body +Z = sensor +Y
  </text>
  {''.join(summary_svg)}
  {draw_panel('Full Mission - Unfolded', samples, unfolded_series, full_x_min, full_x_max, unfolded_y_min, unfolded_y_max, left, full_unfold_top, plot_width, panel_height, 'Zenith (deg)')}
  {draw_panel('Flight Window - Unfolded', flight_samples, unfolded_series, flight_x_min, full_x_max, unfolded_y_min, unfolded_y_max, left, flight_unfold_top, plot_width, panel_height, 'Zenith (deg)')}
  {draw_panel('Full Mission - Folded', samples, folded_series, full_x_min, full_x_max, folded_y_min, folded_y_max, left, full_fold_top, plot_width, panel_height, 'Tilt Magnitude (deg)')}
  {draw_panel('Flight Window - Folded', flight_samples, folded_series, flight_x_min, full_x_max, folded_y_min, folded_y_max, left, flight_fold_top, plot_width, panel_height, 'Tilt Magnitude (deg)')}
  <text x="{left + plot_width / 2:.2f}" y="{flight_fold_top + panel_height + 54:.2f}" text-anchor="middle" font-size="14" fill="#111827">
    Time (s)
  </text>
</svg>
"""

    args.output.write_text(svg, encoding="utf-8")
    print(f"Saved WT901 rotation plot to {args.output}")
    for line in summary_lines:
        print(line)


if __name__ == "__main__":
    main()
