#!/usr/bin/env python3
"""
Plot raw zenith angles derived directly from logged sensor quaternions.

The zenith calculation here is intentionally the direct quaternion method:
    cos(zenith) = 1 - 2 * (x^2 + y^2)
    zenith = acos(clamp(cos(zenith), -1, 1))

That produces the raw, unfolded zenith in degrees in [0, 180].
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path


from replaylib.paths import SCRIPTS_DIR
SCRIPT_DIR = SCRIPTS_DIR
REPLAY_DIR = SCRIPT_DIR.parent
DEFAULT_INPUT_CSV = SCRIPT_DIR / "fullscale4.csv"


@dataclass
class ZenithSample:
    time_s: float
    status: str
    icm_zenith_deg: float | None
    lsm_zenith_deg: float | None
    bno_zenith_deg: float | None
    wt901_zenith_deg: float | None


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


def quaternion_raw_zenith_deg(w: float, x: float, y: float, z: float) -> float | None:
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 0.0 or not math.isfinite(norm):
        return None
    x /= norm
    y /= norm
    cos_zenith = clamp(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0)
    return math.degrees(math.acos(cos_zenith))


def read_zenith(raw: dict[str, str], prefix: str, valid_key: str) -> float | None:
    if not parse_bool(raw.get(valid_key)):
        return None
    w = parse_float(raw.get(f"{prefix}_w"))
    x = parse_float(raw.get(f"{prefix}_x"))
    y = parse_float(raw.get(f"{prefix}_y"))
    z = parse_float(raw.get(f"{prefix}_z"))
    if None in (w, x, y, z):
        return None
    return quaternion_raw_zenith_deg(w, x, y, z)


def load_samples(path: Path) -> list[ZenithSample]:
    samples: list[ZenithSample] = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(line for line in handle if not line.startswith("#"))
        for raw in reader:
            time_s = parse_float(raw.get("sensor_timestamp"))
            if time_s is None:
                continue
            status = (raw.get("flight_status") or "").strip().lower()
            samples.append(
                ZenithSample(
                    time_s=time_s,
                    status=status,
                    icm_zenith_deg=read_zenith(raw, "sensor_icm_quat", "sensor_has_icm_quaternion"),
                    lsm_zenith_deg=read_zenith(raw, "sensor_lsm_quat", "sensor_has_lsm_quaternion"),
                    bno_zenith_deg=read_zenith(raw, "sensor_bno_quat", "sensor_has_bno_quaternion"),
                    wt901_zenith_deg=read_zenith(raw, "sensor_wt901_quat", "sensor_has_wt901_quaternion"),
                )
            )
    return samples


def scale(value: float, src_lo: float, src_hi: float, dst_lo: float, dst_hi: float) -> float:
    if src_hi <= src_lo:
        return 0.5 * (dst_lo + dst_hi)
    ratio = (value - src_lo) / (src_hi - src_lo)
    return dst_lo + ratio * (dst_hi - dst_lo)


def build_polyline(
    samples: list[ZenithSample],
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


def phase_runs(samples: list[ZenithSample]) -> list[tuple[str, float, float]]:
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
    samples: list[ZenithSample],
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
            f'{y_value:.3f}</text>'
        )
        elements.append(
            f'<text x="{gx:.2f}" y="{top + height + 22:.2f}" text-anchor="middle" '
            'font-size="12" fill="#111827">'
            f'{x_value:.2f}</text>'
        )

    series = [
        ("ICM", lambda sample: sample.icm_zenith_deg, "#1d4ed8"),
        ("LSM", lambda sample: sample.lsm_zenith_deg, "#059669"),
        ("BNO", lambda sample: sample.bno_zenith_deg, "#dc2626"),
        ("WT901", lambda sample: sample.wt901_zenith_deg, "#7c3aed"),
    ]
    legend_y = top + 16.0
    legend_x = left + 12.0
    for index, (label, selector, color) in enumerate(series):
        polyline = build_polyline(samples, selector, x_min, x_max, y_min, y_max, left, top, width, height)
        if polyline:
            elements.append(
                f'<polyline fill="none" stroke="{color}" stroke-width="1.1" points="{polyline}"/>'
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
    elements.append(
        f'<text x="{left:.2f}" y="{top - 12:.2f}" font-size="16" fill="#111827">{title}</text>'
    )
    elements.append(
        f'<text x="{left - 56:.2f}" y="{top + height * 0.5:.2f}" text-anchor="middle" '
        'font-size="13" fill="#111827" transform="rotate(-90 '
        f'{left - 56:.2f} {top + height * 0.5:.2f})">Raw Zenith (deg)</text>'
    )
    return "\n".join(elements)


def summarize(samples: list[ZenithSample]) -> list[str]:
    lines: list[str] = []
    series = [
        ("ICM", [sample.icm_zenith_deg for sample in samples if sample.icm_zenith_deg is not None]),
        ("LSM", [sample.lsm_zenith_deg for sample in samples if sample.lsm_zenith_deg is not None]),
        ("BNO", [sample.bno_zenith_deg for sample in samples if sample.bno_zenith_deg is not None]),
        ("WT901", [sample.wt901_zenith_deg for sample in samples if sample.wt901_zenith_deg is not None]),
    ]
    for label, values in series:
        if not values:
            continue
        lines.append(
            f"{label}: mean={sum(values) / len(values):.4f} deg, "
            f"min={min(values):.4f}, max={max(values):.4f}"
        )
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
        args.output = args.input_csv.with_name(args.input_csv.stem + "_raw_quaternion_zenith.svg")

    args.output.parent.mkdir(parents=True, exist_ok=True)

    all_values = [
        value
        for sample in samples
        for value in (
            sample.icm_zenith_deg,
            sample.lsm_zenith_deg,
            sample.bno_zenith_deg,
            sample.wt901_zenith_deg,
        )
        if value is not None and math.isfinite(value)
    ]
    if not all_values:
        raise SystemExit("No valid quaternion zenith values found.")

    y_min = min(all_values)
    y_max = max(all_values)
    y_pad = max(0.01, 0.15 * (y_max - y_min if y_max > y_min else 0.05))
    y_min -= y_pad
    y_max += y_pad

    full_x_min = samples[0].time_s
    full_x_max = samples[-1].time_s

    non_ground = [sample for sample in samples if sample.status != "ground"]
    if non_ground:
        flight_x_min = max(full_x_min, non_ground[0].time_s - 0.5)
    else:
        flight_x_min = full_x_min
    flight_x_max = full_x_max
    flight_samples = [sample for sample in samples if flight_x_min <= sample.time_s <= flight_x_max]

    width = 1600
    height = 1100
    left = 120
    right = 40
    top = 70
    panel_gap = 90
    panel_height = 380
    plot_width = width - left - right

    full_panel_top = top + 30
    flight_panel_top = full_panel_top + panel_height + panel_gap

    summary_lines = summarize(samples)
    summary_svg = []
    for index, line in enumerate(summary_lines):
        summary_svg.append(
            f'<text x="{left:.2f}" y="{top + index * 18:.2f}" font-size="13" fill="#374151">{line}</text>'
        )

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="#fffdf8"/>
  <text x="{width / 2:.0f}" y="30" text-anchor="middle" font-size="24" fill="#111827">
    Raw Quaternion Zenith From ICM / LSM / BNO / WT901
  </text>
  {''.join(summary_svg)}
  {draw_panel('Full Mission', samples, full_x_min, full_x_max, y_min, y_max, left, full_panel_top, plot_width, panel_height)}
  {draw_panel('Flight Window', flight_samples, flight_x_min, flight_x_max, y_min, y_max, left, flight_panel_top, plot_width, panel_height)}
  <text x="{left + plot_width / 2:.2f}" y="{flight_panel_top + panel_height + 50:.2f}" text-anchor="middle" font-size="14" fill="#111827">
    Time (s)
  </text>
</svg>
"""

    args.output.write_text(svg, encoding="utf-8")
    print(f"Saved raw zenith plot to {args.output}")
    for line in summary_lines:
        print(line)


if __name__ == "__main__":
    main()
