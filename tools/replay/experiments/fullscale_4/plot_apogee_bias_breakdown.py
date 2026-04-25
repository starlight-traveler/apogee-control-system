#!/usr/bin/env python3
"""
Break down the early apogee walk-down in `fullscale4.csv` into:
1. altitude-reference/state altitude error,
2. vertical-velocity/state acceleration error,
3. predictor drag penalty relative to a ballistic ceiling.

The script writes a compact SVG with four panels and prints the key summary
numbers to stdout.
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
DEFAULT_OUTPUT_SVG = SCRIPT_DIR / "fullscale4_apogee_bias_breakdown.svg"

G_FTPS2 = 32.174


@dataclass
class Sample:
    time_s: float
    status: str
    baro_agl_ft: float
    state_ft: float
    state_vz_fps: float
    baro_vz_fps: float
    apogee_ft: float
    flap_deg: float
    settling: bool
    ballistic_ft: float
    ballistic_alt_fix_ft: float
    ballistic_vel_fix_ft: float
    drag_penalty_ft: float


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


def load_samples(path: Path) -> list[Sample]:
    raw_rows: list[dict[str, str]] = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(line for line in handle if not line.startswith("#"))
        for row in reader:
            if row.get("has_filtered_state") != "True":
                continue
            raw_rows.append(row)
    if not raw_rows:
        raise SystemExit(f"No usable filtered-state rows found in {path}")

    pad_alt_ft: float | None = None
    for row in raw_rows:
        absolute_alt_ft = parse_float(row.get("sensor_altitude_feet"))
        if absolute_alt_ft is not None and absolute_alt_ft != 0.0:
            pad_alt_ft = absolute_alt_ft
            break
    if pad_alt_ft is None:
        raise SystemExit(f"Could not determine pad altitude from {path}")

    pre_samples: list[dict[str, float | str | bool]] = []
    for row in raw_rows:
        time_s = parse_float(row.get("sensor_timestamp"))
        absolute_alt_ft = parse_float(row.get("sensor_altitude_feet"))
        state_ft = parse_float(row.get("state_altitude_agl_feet"))
        state_vz_fps = parse_float(row.get("state_vertical_velocity_fps"))
        apogee_ft = parse_float(row.get("state_apogee_estimate_feet"))
        flap_deg = parse_float(row.get("sensor_flap_command_deg"))
        if (
            time_s is None
            or absolute_alt_ft is None
            or state_ft is None
            or state_vz_fps is None
            or apogee_ft is None
        ):
            continue
        pre_samples.append(
            {
                "time_s": time_s,
                "status": (row.get("flight_status") or "").strip().lower(),
                "baro_agl_ft": max(0.0, absolute_alt_ft - pad_alt_ft),
                "state_ft": state_ft,
                "state_vz_fps": state_vz_fps,
                "apogee_ft": apogee_ft,
                "flap_deg": 0.0 if flap_deg is None else flap_deg,
                "settling": row.get("sensor_actuation_is_settling", "False") == "True",
            }
        )

    if not pre_samples:
        raise SystemExit(f"No complete numeric rows found in {path}")

    times = [float(sample["time_s"]) for sample in pre_samples]
    baro_agl = [float(sample["baro_agl_ft"]) for sample in pre_samples]
    baro_vz = centered_slopes(times, baro_agl, window_seconds=0.30)

    samples: list[Sample] = []
    for sample, vz_baro in zip(pre_samples, baro_vz):
        state_ft = float(sample["state_ft"])
        state_vz_fps = float(sample["state_vz_fps"])
        ballistic_ft = state_ft + (state_vz_fps * state_vz_fps) / (2.0 * G_FTPS2)
        ballistic_alt_fix_ft = float(sample["baro_agl_ft"]) + (state_vz_fps * state_vz_fps) / (2.0 * G_FTPS2)
        ballistic_vel_fix_ft = state_ft + (vz_baro * vz_baro) / (2.0 * G_FTPS2)
        apogee_ft = float(sample["apogee_ft"])
        samples.append(
            Sample(
                time_s=float(sample["time_s"]),
                status=str(sample["status"]),
                baro_agl_ft=float(sample["baro_agl_ft"]),
                state_ft=state_ft,
                state_vz_fps=state_vz_fps,
                baro_vz_fps=vz_baro,
                apogee_ft=apogee_ft,
                flap_deg=float(sample["flap_deg"]),
                settling=bool(sample["settling"]),
                ballistic_ft=ballistic_ft,
                ballistic_alt_fix_ft=ballistic_alt_fix_ft,
                ballistic_vel_fix_ft=ballistic_vel_fix_ft,
                drag_penalty_ft=apogee_ft - ballistic_ft,
            )
        )
    return samples


def centered_slopes(times: list[float], values: list[float], window_seconds: float) -> list[float]:
    if len(times) != len(values):
        raise ValueError("times and values must have the same length")
    if len(times) < 2:
        return [0.0 for _ in times]

    half_window = max(window_seconds * 0.5, 1.0e-3)
    slopes: list[float] = []
    for index, time_s in enumerate(times):
        left = index
        right = index
        while left > 0 and times[left - 1] >= time_s - half_window:
            left -= 1
        while right + 1 < len(times) and times[right + 1] <= time_s + half_window:
            right += 1
        xs = times[left : right + 1]
        ys = values[left : right + 1]
        mean_x = sum(xs) / len(xs)
        mean_y = sum(ys) / len(ys)
        denom = sum((x - mean_x) * (x - mean_x) for x in xs)
        if denom <= 1.0e-12:
            slopes.append(0.0)
            continue
        numer = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
        slopes.append(numer / denom)
    return slopes


def phase_runs(samples: list[Sample]) -> list[tuple[str, float, float]]:
    if not samples:
        return []
    runs: list[tuple[str, float, float]] = []
    current_status = samples[0].status
    run_start = samples[0].time_s
    for sample in samples[1:]:
        if sample.status != current_status:
            runs.append((current_status, run_start, sample.time_s))
            current_status = sample.status
            run_start = sample.time_s
    runs.append((current_status, run_start, samples[-1].time_s))
    return runs


def mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def scale(value: float, src_lo: float, src_hi: float, dst_lo: float, dst_hi: float) -> float:
    if src_hi <= src_lo:
        return 0.5 * (dst_lo + dst_hi)
    ratio = (value - src_lo) / (src_hi - src_lo)
    return dst_lo + ratio * (dst_hi - dst_lo)


def build_polyline(
    points: list[tuple[float, float]],
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    left: float,
    top: float,
    width: float,
    height: float,
) -> str:
    bottom = top + height
    svg_points: list[str] = []
    for time_s, value in points:
        if not math.isfinite(value):
            continue
        x = scale(time_s, x_min, x_max, left, left + width)
        y = scale(value, y_min, y_max, bottom, top)
        svg_points.append(f"{x:.2f},{y:.2f}")
    return " ".join(svg_points)


def draw_panel(
    title: str,
    unit_label: str,
    samples: list[Sample],
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    left: float,
    top: float,
    width: float,
    height: float,
    series: list[tuple[str, list[tuple[float, float]], str, str]],
    markers: list[tuple[str, float, str]],
) -> str:
    bottom = top + height
    phase_colors = {
        "ground": "#f3f4f6",
        "burn": "#fee2e2",
        "coast": "#dbeafe",
        "overshoot": "#ede9fe",
        "descent": "#dcfce7",
    }
    elements: list[str] = []

    for status, run_start, run_end in phase_runs(samples):
        x0 = scale(run_start, x_min, x_max, left, left + width)
        x1 = scale(run_end, x_min, x_max, left, left + width)
        elements.append(
            f'<rect x="{x0:.2f}" y="{top:.2f}" width="{max(1.0, x1 - x0):.2f}" '
            f'height="{height:.2f}" fill="{phase_colors.get(status, "#f3f4f6")}" opacity="0.38"/>'
        )

    for index in range(6):
        grid_y = top + index * height / 5.0
        grid_x = left + index * width / 5.0
        y_value = y_max - index * (y_max - y_min) / 5.0
        x_value = x_min + index * (x_max - x_min) / 5.0
        elements.append(
            f'<line x1="{left:.2f}" y1="{grid_y:.2f}" x2="{left + width:.2f}" y2="{grid_y:.2f}" '
            'stroke="#d6d9df" stroke-width="1"/>'
        )
        elements.append(
            f'<line x1="{grid_x:.2f}" y1="{top:.2f}" x2="{grid_x:.2f}" y2="{bottom:.2f}" '
            'stroke="#e5e7eb" stroke-width="1"/>'
        )
        elements.append(
            f'<text x="{left - 12:.2f}" y="{grid_y + 4:.2f}" text-anchor="end" '
            'font-size="12" fill="#111827">'
            f"{y_value:.0f}</text>"
        )
        elements.append(
            f'<text x="{grid_x:.2f}" y="{bottom + 22:.2f}" text-anchor="middle" '
            'font-size="12" fill="#111827">'
            f"{x_value:.2f}</text>"
        )

    for label, time_s, color in markers:
        x = scale(time_s, x_min, x_max, left, left + width)
        elements.append(
            f'<line x1="{x:.2f}" y1="{top:.2f}" x2="{x:.2f}" y2="{bottom:.2f}" '
            f'stroke="{color}" stroke-width="1.6" stroke-dasharray="5 5"/>'
        )
        elements.append(
            f'<text x="{x + 4:.2f}" y="{top + 14:.2f}" font-size="11" fill="{color}">{label}</text>'
        )

    legend_x = left + 12.0
    legend_y = top + 18.0
    for index, (label, points, color, dash) in enumerate(series):
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        polyline = build_polyline(points, x_min, x_max, y_min, y_max, left, top, width, height)
        if polyline:
            elements.append(
                f'<polyline fill="none" stroke="{color}" stroke-width="2.0"{dash_attr} points="{polyline}"/>'
            )
        line_y = legend_y + 19.0 * index
        elements.append(
            f'<line x1="{legend_x:.2f}" y1="{line_y:.2f}" x2="{legend_x + 30:.2f}" y2="{line_y:.2f}" '
            f'stroke="{color}" stroke-width="2.8"{dash_attr}/>'
        )
        elements.append(
            f'<text x="{legend_x + 38:.2f}" y="{line_y + 4:.2f}" font-size="13" fill="#111827">{label}</text>'
        )

    elements.append(
        f'<rect x="{left:.2f}" y="{top:.2f}" width="{width:.2f}" height="{height:.2f}" '
        'fill="none" stroke="#0f172a" stroke-width="1.5"/>'
    )
    elements.append(
        f'<text x="{left:.2f}" y="{top - 16:.2f}" font-size="18" font-weight="700" fill="#111827">{title}</text>'
    )
    elements.append(
        f'<text x="{left + width:.2f}" y="{top - 16:.2f}" text-anchor="end" '
        'font-size="13" fill="#475569">'
        f"{unit_label}</text>"
    )
    return "\n".join(elements)


def write_svg(output_path: Path, samples: list[Sample]) -> None:
    burn_start = next(sample for sample in samples if sample.status == "burn")
    coast_start = next(sample for sample in samples if sample.status == "coast")
    first_settle = next(
        sample
        for sample in samples
        if sample.status == "coast" and (sample.settling or abs(sample.flap_deg) > 0.01)
    )
    state_apogee = max(samples, key=lambda sample: sample.state_ft)
    baro_apogee = max(samples, key=lambda sample: sample.baro_agl_ft)
    pre_settle_coast = [sample for sample in samples if sample.status == "coast" and sample.time_s < first_settle.time_s]

    markers = [
        ("burn", burn_start.time_s, "#dc2626"),
        ("coast", coast_start.time_s, "#2563eb"),
        ("settle", first_settle.time_s, "#7c3aed"),
        ("state ap", state_apogee.time_s, "#059669"),
    ]

    x_min = burn_start.time_s - 0.15
    x_max = baro_apogee.time_s + 0.10

    apogee_series = [
        ("Logged apogee", [(sample.time_s, sample.apogee_ft) for sample in samples], "#0f4c81", ""),
        ("Ballistic from state", [(sample.time_s, sample.ballistic_ft) for sample in samples], "#f97316", ""),
        ("Ballistic with baro vz", [(sample.time_s, sample.ballistic_vel_fix_ft) for sample in samples], "#059669", ""),
        ("Ballistic with baro alt", [(sample.time_s, sample.ballistic_alt_fix_ft) for sample in samples], "#475569", "6 5"),
    ]
    drag_series = [
        ("Drag penalty = logged - ballistic", [(sample.time_s, sample.drag_penalty_ft) for sample in samples], "#7c3aed", ""),
    ]
    altitude_bias_series = [
        ("Baro AGL - state altitude", [(sample.time_s, sample.baro_agl_ft - sample.state_ft) for sample in samples], "#1d4ed8", ""),
    ]
    velocity_bias_series = [
        ("Baro vz - state vz", [(sample.time_s, sample.baro_vz_fps - sample.state_vz_fps) for sample in samples], "#dc2626", ""),
    ]

    apogee_max = max(sample.ballistic_vel_fix_ft for sample in samples)
    apogee_y_max = math.ceil((apogee_max + 80.0) / 100.0) * 100.0
    drag_min = min(sample.drag_penalty_ft for sample in samples)
    altitude_bias_max = max(abs(sample.baro_agl_ft - sample.state_ft) for sample in samples)
    velocity_bias_max = max(abs(sample.baro_vz_fps - sample.state_vz_fps) for sample in samples)

    pre_alt_bias = [sample.baro_agl_ft - sample.state_ft for sample in pre_settle_coast]
    pre_vel_bias = [sample.baro_vz_fps - sample.state_vz_fps for sample in pre_settle_coast]
    coast_alt_uplift = coast_start.ballistic_alt_fix_ft - coast_start.ballistic_ft
    coast_vel_uplift = coast_start.ballistic_vel_fix_ft - coast_start.ballistic_ft
    settle_alt_uplift = first_settle.ballistic_alt_fix_ft - first_settle.ballistic_ft
    settle_vel_uplift = first_settle.ballistic_vel_fix_ft - first_settle.ballistic_ft
    logged_delta = first_settle.apogee_ft - coast_start.apogee_ft
    ballistic_delta = first_settle.ballistic_ft - coast_start.ballistic_ft
    drag_penalty_change = first_settle.drag_penalty_ft - coast_start.drag_penalty_ft

    subtitle_lines = [
        f"Pre-settling coast altitude bias: mean {mean(pre_alt_bias):.1f} ft, max {max(abs(value) for value in pre_alt_bias):.1f} ft",
        f"Pre-settling coast velocity bias: mean {mean(pre_vel_bias):.1f} fps, max {max(abs(value) for value in pre_vel_bias):.1f} fps",
        f"Coast start uplift if fixed: altitude {coast_alt_uplift:.1f} ft, velocity {coast_vel_uplift:.1f} ft",
        f"First settling uplift if fixed: altitude {settle_alt_uplift:.1f} ft, velocity {settle_vel_uplift:.1f} ft",
        f"Coast start -> settling: logged {logged_delta:.1f} ft, ballistic {ballistic_delta:.1f} ft, drag penalty change {drag_penalty_change:.1f} ft",
    ]

    width = 1580
    height = 1160
    margin_left = 92.0
    margin_right = 42.0
    gap_x = 54.0
    gap_y = 72.0
    panel_width = (width - margin_left - margin_right - gap_x) / 2.0
    panel_height = 305.0
    top_row_y = 205.0
    bottom_row_y = top_row_y + panel_height + gap_y

    elements = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        '<text x="790" y="42" text-anchor="middle" font-size="30" font-weight="700" fill="#0f172a">Fullscale 4 Apogee Bias Breakdown</text>',
        '<text x="790" y="69" text-anchor="middle" font-size="15" fill="#334155">Early walk-down decomposition: altitude reference vs velocity state vs predictor drag penalty</text>',
    ]

    for index, line in enumerate(subtitle_lines):
        elements.append(
            f'<text x="790" y="{96 + index * 18:.2f}" text-anchor="middle" '
            'font-size="14" fill="#475569">'
            f"{line}</text>"
        )

    elements.append(
        draw_panel(
            title="Apogee Decomposition",
            unit_label="feet",
            samples=samples,
            x_min=x_min,
            x_max=x_max,
            y_min=0.0,
            y_max=apogee_y_max,
            left=margin_left,
            top=top_row_y,
            width=panel_width,
            height=panel_height,
            series=apogee_series,
            markers=markers,
        )
    )
    elements.append(
        draw_panel(
            title="Predictor Drag Penalty",
            unit_label="feet",
            samples=samples,
            x_min=x_min,
            x_max=x_max,
            y_min=math.floor((drag_min - 50.0) / 50.0) * 50.0,
            y_max=50.0,
            left=margin_left + panel_width + gap_x,
            top=top_row_y,
            width=panel_width,
            height=panel_height,
            series=drag_series,
            markers=markers,
        )
    )
    elements.append(
        draw_panel(
            title="Altitude Bias",
            unit_label="feet",
            samples=samples,
            x_min=x_min,
            x_max=x_max,
            y_min=-10.0,
            y_max=math.ceil((altitude_bias_max + 20.0) / 20.0) * 20.0,
            left=margin_left,
            top=bottom_row_y,
            width=panel_width,
            height=panel_height,
            series=altitude_bias_series,
            markers=markers,
        )
    )
    elements.append(
        draw_panel(
            title="Vertical Velocity Bias",
            unit_label="ft/s",
            samples=samples,
            x_min=x_min,
            x_max=x_max,
            y_min=-40.0,
            y_max=math.ceil((velocity_bias_max + 40.0) / 40.0) * 40.0,
            left=margin_left + panel_width + gap_x,
            top=bottom_row_y,
            width=panel_width,
            height=panel_height,
            series=velocity_bias_series,
            markers=markers,
        )
    )

    elements.append(
        '<text x="790" y="1126" text-anchor="middle" font-size="13" fill="#64748b">'
        'Pad reference uses the flight code convention: first nonzero barometer sample. Baro velocity is a centered 0.30 s regression slope.'
        '</text>'
    )
    elements.append("</svg>")
    output_path.write_text("\n".join(elements))


def print_summary(samples: list[Sample]) -> None:
    burn_start = next(sample for sample in samples if sample.status == "burn")
    coast_start = next(sample for sample in samples if sample.status == "coast")
    first_settle = next(
        sample
        for sample in samples
        if sample.status == "coast" and (sample.settling or abs(sample.flap_deg) > 0.01)
    )
    state_apogee = max(samples, key=lambda sample: sample.state_ft)
    baro_apogee = max(samples, key=lambda sample: sample.baro_agl_ft)
    pre_settle_coast = [sample for sample in samples if sample.status == "coast" and sample.time_s < first_settle.time_s]

    pre_alt_bias = [sample.baro_agl_ft - sample.state_ft for sample in pre_settle_coast]
    pre_vel_bias = [sample.baro_vz_fps - sample.state_vz_fps for sample in pre_settle_coast]

    print("Key Samples")
    for label, sample in (
        ("burn_start", burn_start),
        ("coast_start", coast_start),
        ("first_settle", first_settle),
        ("state_apogee", state_apogee),
        ("baro_apogee", baro_apogee),
    ):
        print(
            f"  {label}: t={sample.time_s:.3f}s "
            f"state={sample.state_ft:.1f}ft "
            f"baro={sample.baro_agl_ft:.1f}ft "
            f"vz={sample.state_vz_fps:.1f}fps "
            f"baro_vz={sample.baro_vz_fps:.1f}fps "
            f"ap={sample.apogee_ft:.1f}ft"
        )

    print("Bias Summary")
    print(
        f"  pre-settling altitude bias: mean={mean(pre_alt_bias):.1f}ft "
        f"max={max(abs(value) for value in pre_alt_bias):.1f}ft"
    )
    print(
        f"  pre-settling velocity bias: mean={mean(pre_vel_bias):.1f}fps "
        f"max={max(abs(value) for value in pre_vel_bias):.1f}fps"
    )

    logged_delta = first_settle.apogee_ft - coast_start.apogee_ft
    ballistic_delta = first_settle.ballistic_ft - coast_start.ballistic_ft
    altitude_component = first_settle.state_ft - coast_start.state_ft
    velocity_component = (
        (first_settle.state_vz_fps * first_settle.state_vz_fps)
        - (coast_start.state_vz_fps * coast_start.state_vz_fps)
    ) / (2.0 * G_FTPS2)
    drag_penalty_change = first_settle.drag_penalty_ft - coast_start.drag_penalty_ft

    print("Early Walk-Down Decomposition")
    print(f"  coast -> first settling logged apogee change: {logged_delta:.1f}ft")
    print(f"  coast -> first settling ballistic change: {ballistic_delta:.1f}ft")
    print(f"    altitude component: {altitude_component:.1f}ft")
    print(f"    velocity component: {velocity_component:.1f}ft")
    print(f"  drag penalty change over same interval: {drag_penalty_change:.1f}ft")

    coast_alt_uplift = coast_start.ballistic_alt_fix_ft - coast_start.ballistic_ft
    coast_vel_uplift = coast_start.ballistic_vel_fix_ft - coast_start.ballistic_ft
    settle_alt_uplift = first_settle.ballistic_alt_fix_ft - first_settle.ballistic_ft
    settle_vel_uplift = first_settle.ballistic_vel_fix_ft - first_settle.ballistic_ft
    print("Ballistic Sensitivity")
    print(f"  coast start uplift if only altitude is fixed: {coast_alt_uplift:.1f}ft")
    print(f"  coast start uplift if only velocity is fixed: {coast_vel_uplift:.1f}ft")
    print(f"  first settling uplift if only altitude is fixed: {settle_alt_uplift:.1f}ft")
    print(f"  first settling uplift if only velocity is fixed: {settle_vel_uplift:.1f}ft")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT_CSV, help="Input decoded CSV.")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT_SVG, help="Output SVG path.")
    args = parser.parse_args()

    samples = load_samples(args.input)
    write_svg(args.output, samples)
    print_summary(samples)
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
