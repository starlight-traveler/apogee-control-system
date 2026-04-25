#!/usr/bin/env python3
"""
Counterfactual apogee plot for Fullscale 4:

- Logged apogee estimate
- Velocity-only counterfactual:
    keep the logged drag penalty, replace state vz with baro-derived climb rate
- Altitude+velocity upper bound:
    keep the logged drag penalty, replace both altitude and vz with baro-derived values

This isolates whether fixing early-coast vertical velocity would have changed
the predictor's convergence, without depending on the hosted replay predictor.
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
DEFAULT_OUTPUT_SVG = SCRIPT_DIR / "fullscale4_baro_vz_counterfactual_apogee.svg"

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
    drag_penalty_ft: float
    apogee_vz_only_ft: float
    apogee_alt_vz_ft: float


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

    times = [float(sample["time_s"]) for sample in pre_samples]
    baro_agl = [float(sample["baro_agl_ft"]) for sample in pre_samples]
    baro_vz = centered_slopes(times, baro_agl, window_seconds=0.30)

    samples: list[Sample] = []
    for sample, vz_baro in zip(pre_samples, baro_vz):
        state_ft = float(sample["state_ft"])
        state_vz_fps = float(sample["state_vz_fps"])
        apogee_ft = float(sample["apogee_ft"])
        ballistic_state_ft = state_ft + (state_vz_fps * state_vz_fps) / (2.0 * G_FTPS2)
        drag_penalty_ft = apogee_ft - ballistic_state_ft
        apogee_vz_only_ft = state_ft + (vz_baro * vz_baro) / (2.0 * G_FTPS2) + drag_penalty_ft
        apogee_alt_vz_ft = float(sample["baro_agl_ft"]) + (vz_baro * vz_baro) / (2.0 * G_FTPS2) + drag_penalty_ft
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
                drag_penalty_ft=drag_penalty_ft,
                apogee_vz_only_ft=apogee_vz_only_ft,
                apogee_alt_vz_ft=apogee_alt_vz_ft,
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
    reference_lines: list[tuple[str, float, str]],
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

    for label, value, color in reference_lines:
        y = scale(value, y_min, y_max, bottom, top)
        elements.append(
            f'<line x1="{left:.2f}" y1="{y:.2f}" x2="{left + width:.2f}" y2="{y:.2f}" '
            f'stroke="{color}" stroke-width="1.5" stroke-dasharray="6 5"/>'
        )
        elements.append(
            f'<text x="{left + width - 6:.2f}" y="{y - 6:.2f}" text-anchor="end" '
            f'font-size="12" fill="{color}">{label}</text>'
        )

    for label, time_s, color in markers:
        x = scale(time_s, x_min, x_max, left, left + width)
        elements.append(
            f'<line x1="{x:.2f}" y1="{top:.2f}" x2="{x:.2f}" y2="{bottom:.2f}" '
            f'stroke="{color}" stroke-width="1.4" stroke-dasharray="4 5"/>'
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
    coast_samples = [sample for sample in samples if sample.status == "coast"]

    markers = [
        ("burn", burn_start.time_s, "#dc2626"),
        ("coast", coast_start.time_s, "#2563eb"),
        ("settle", first_settle.time_s, "#7c3aed"),
        ("state ap", state_apogee.time_s, "#059669"),
    ]

    x_min = burn_start.time_s - 0.15
    x_max = baro_apogee.time_s + 0.10

    max_apogee = max(sample.apogee_alt_vz_ft for sample in coast_samples)
    apogee_y_max = math.ceil((max_apogee + 80.0) / 100.0) * 100.0
    uplift_values = [sample.apogee_vz_only_ft - sample.apogee_ft for sample in coast_samples]
    uplift_upper_values = [sample.apogee_alt_vz_ft - sample.apogee_ft for sample in coast_samples]
    uplift_y_max = math.ceil((max(uplift_upper_values) + 40.0) / 50.0) * 50.0
    velocity_y_max = math.ceil((max(sample.baro_vz_fps for sample in coast_samples) + 40.0) / 50.0) * 50.0

    subtitle_lines = [
        f"Coast start: logged {coast_start.apogee_ft:.1f} ft, vz-only {coast_start.apogee_vz_only_ft:.1f} ft, alt+vz {coast_start.apogee_alt_vz_ft:.1f} ft",
        f"First settling: logged {first_settle.apogee_ft:.1f} ft, vz-only {first_settle.apogee_vz_only_ft:.1f} ft, alt+vz {first_settle.apogee_alt_vz_ft:.1f} ft",
        f"State apogee: logged {state_apogee.apogee_ft:.1f} ft, vz-only {state_apogee.apogee_vz_only_ft:.1f} ft, alt+vz {state_apogee.apogee_alt_vz_ft:.1f} ft",
        f"Final coast sample: vz-only uplift {coast_samples[-1].apogee_vz_only_ft - coast_samples[-1].apogee_ft:.1f} ft, alt+vz uplift {coast_samples[-1].apogee_alt_vz_ft - coast_samples[-1].apogee_ft:.1f} ft",
    ]

    width = 1520
    height = 1180
    margin_left = 92.0
    margin_right = 42.0
    panel_width = width - margin_left - margin_right
    panel_height = 255.0
    gap_y = 62.0
    top_y = 175.0

    elements = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        '<text x="760" y="42" text-anchor="middle" font-size="30" font-weight="700" fill="#0f172a">Baro-vz Counterfactual Apogee</text>',
        '<text x="760" y="69" text-anchor="middle" font-size="15" fill="#334155">Logged predictor with drag penalty held fixed, replacing vertical velocity from the barometer slope</text>',
    ]

    for index, line in enumerate(subtitle_lines):
        elements.append(
            f'<text x="760" y="{96 + index * 18:.2f}" text-anchor="middle" '
            'font-size="14" fill="#475569">'
            f"{line}</text>"
        )

    elements.append(
        draw_panel(
            title="Apogee Traces",
            unit_label="feet",
            samples=samples,
            x_min=x_min,
            x_max=x_max,
            y_min=0.0,
            y_max=apogee_y_max,
            left=margin_left,
            top=top_y,
            width=panel_width,
            height=panel_height,
            series=[
                ("Logged apogee", [(sample.time_s, sample.apogee_ft) for sample in samples], "#0f4c81", ""),
                ("vz-only counterfactual", [(sample.time_s, sample.apogee_vz_only_ft) for sample in samples], "#dc2626", ""),
                ("alt+vz upper bound", [(sample.time_s, sample.apogee_alt_vz_ft) for sample in samples], "#059669", "7 5"),
            ],
            reference_lines=[("Baro peak", baro_apogee.baro_agl_ft, "#7c3aed")],
            markers=markers,
        )
    )
    elements.append(
        draw_panel(
            title="Vertical Velocity",
            unit_label="ft/s",
            samples=samples,
            x_min=x_min,
            x_max=x_max,
            y_min=-50.0,
            y_max=velocity_y_max,
            left=margin_left,
            top=top_y + panel_height + gap_y,
            width=panel_width,
            height=panel_height,
            series=[
                ("State vz", [(sample.time_s, sample.state_vz_fps) for sample in samples], "#2563eb", ""),
                ("Baro-derived vz", [(sample.time_s, sample.baro_vz_fps) for sample in samples], "#f97316", ""),
            ],
            reference_lines=[],
            markers=markers,
        )
    )
    elements.append(
        draw_panel(
            title="Counterfactual Uplift Over Logged",
            unit_label="feet",
            samples=samples,
            x_min=x_min,
            x_max=x_max,
            y_min=-20.0,
            y_max=uplift_y_max,
            left=margin_left,
            top=top_y + 2.0 * (panel_height + gap_y),
            width=panel_width,
            height=panel_height,
            series=[
                ("vz-only uplift", [(sample.time_s, sample.apogee_vz_only_ft - sample.apogee_ft) for sample in samples], "#dc2626", ""),
                ("alt+vz uplift", [(sample.time_s, sample.apogee_alt_vz_ft - sample.apogee_ft) for sample in samples], "#059669", "7 5"),
            ],
            reference_lines=[("0 ft", 0.0, "#64748b")],
            markers=markers,
        )
    )

    elements.append(
        '<text x="760" y="1148" text-anchor="middle" font-size="13" fill="#64748b">'
        'Baro velocity is a centered 0.30 s regression slope. This is a counterfactual isolating vz and state altitude, not a re-run of the full flight computer.'
        '</text>'
    )
    elements.append("</svg>")
    output_path.write_text("\n".join(elements))


def print_summary(samples: list[Sample]) -> None:
    coast_start = next(sample for sample in samples if sample.status == "coast")
    first_settle = next(
        sample
        for sample in samples
        if sample.status == "coast" and (sample.settling or abs(sample.flap_deg) > 0.01)
    )
    state_apogee = max(samples, key=lambda sample: sample.state_ft)
    baro_apogee = max(samples, key=lambda sample: sample.baro_agl_ft)
    coast_samples = [sample for sample in samples if sample.status == "coast"]

    print("Counterfactual Markers")
    for label, sample in (
        ("coast_start", coast_start),
        ("first_settle", first_settle),
        ("state_apogee", state_apogee),
        ("baro_apogee", baro_apogee),
    ):
        print(
            f"  {label}: t={sample.time_s:.3f}s "
            f"logged={sample.apogee_ft:.1f}ft "
            f"vz_only={sample.apogee_vz_only_ft:.1f}ft "
            f"alt_vz={sample.apogee_alt_vz_ft:.1f}ft "
            f"state_vz={sample.state_vz_fps:.1f}fps "
            f"baro_vz={sample.baro_vz_fps:.1f}fps"
        )

    print("Convergence")
    print(
        f"  final coast sample vz-only uplift: "
        f"{coast_samples[-1].apogee_vz_only_ft - coast_samples[-1].apogee_ft:.1f}ft"
    )
    print(
        f"  final coast sample alt+vz uplift: "
        f"{coast_samples[-1].apogee_alt_vz_ft - coast_samples[-1].apogee_ft:.1f}ft"
    )
    print(
        f"  baro peak reference: {baro_apogee.baro_agl_ft:.1f}ft, "
        f"logged final apogee: {coast_samples[-1].apogee_ft:.1f}ft"
    )


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
