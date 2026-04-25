#!/usr/bin/env python3
"""
Plot fullscale 4 with actuation-predictor altitude seeded from baro AGL.

This mirrors the flight-code change that leaves the estimator state alone but
uses fresh baro AGL as the apogee predictor altitude seed during coast when the
baro/state agreement gate is satisfied.
"""

from __future__ import annotations

import argparse
import csv
import html
import math
import os
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path

from plot_no_deweight_apogee_replay import find_replay_binary
from replay_legacy_frame_adapter import HISTORICAL_FULLSCALE_CP_OFFSET_M
from replay_legacy_frame_adapter import HISTORICAL_FULLSCALE_DRY_MASS_KG
from replay_legacy_frame_adapter import HISTORICAL_FULLSCALE_MOI_KGM2


from replaylib.paths import SCRIPTS_DIR
SCRIPT_DIR = SCRIPTS_DIR
ROOT = SCRIPT_DIR.parents[2]
DEFAULT_INPUT_CSV = ROOT / "tools" / "replay" / "data" / "fullscale_4.csv"
DEFAULT_OUTPUT_CSV = ROOT / "tools" / "replay" / "data" / "fullscale_4_baro_altitude_seed_comparison.csv"
DEFAULT_OUTPUT_SVG = ROOT / "tools" / "replay" / "plots" / "fullscale_4_baro_altitude_seed_comparison.svg"

FEET_TO_METERS = 0.3048
METERS_TO_FEET = 1.0 / FEET_TO_METERS


@dataclass
class LogSample:
    time_s: float
    status: str
    baro_agl_ft: float
    state_agl_ft: float
    flap_effective_deg: float


@dataclass
class ReplaySample:
    time_s: float
    status: str
    altitude_ft: float
    apogee_ft: float | None
    velocity_fps: float


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


def load_log_samples(path: Path) -> list[LogSample]:
    rows: list[LogSample] = []
    pad_altitude_ft: float | None = None
    with path.open(newline="") as handle:
        reader = csv.DictReader(line for line in handle if not line.startswith("#"))
        for row in reader:
            time_s = parse_float(row.get("sensor_timestamp"))
            altitude_ft = parse_float(row.get("sensor_altitude_feet"))
            state_agl_ft = parse_float(row.get("state_altitude_agl_feet"))
            flap_effective_deg = parse_float(row.get("sensor_flap_effective_deg")) or 0.0
            if time_s is None or altitude_ft is None or state_agl_ft is None:
                continue
            if pad_altitude_ft is None:
                pad_altitude_ft = altitude_ft
            rows.append(
                LogSample(
                    time_s=time_s,
                    status=(row.get("flight_status") or "").strip().lower(),
                    baro_agl_ft=max(0.0, altitude_ft - pad_altitude_ft),
                    state_agl_ft=state_agl_ft,
                    flap_effective_deg=flap_effective_deg,
                )
            )
    if not rows:
        raise SystemExit(f"No usable rows found in {path}")
    return rows


def write_baro_seed_input(input_csv: Path, output_csv: Path) -> None:
    with input_csv.open(newline="") as source, output_csv.open("w", newline="") as dest:
        reader = csv.DictReader(line for line in source if not line.startswith("#"))
        if reader.fieldnames is None:
            raise SystemExit(f"No header found in {input_csv}")
        writer = csv.DictWriter(dest, fieldnames=reader.fieldnames)
        writer.writeheader()
        pad_altitude_ft: float | None = None
        for row in reader:
            altitude_ft = parse_float(row.get("sensor_altitude_feet"))
            if altitude_ft is not None:
                if pad_altitude_ft is None:
                    pad_altitude_ft = altitude_ft
                row["state_altitude_agl_feet"] = f"{max(0.0, altitude_ft - pad_altitude_ft):.9f}"
            writer.writerow(row)


def run_replay(replay_binary: Path, input_csv: Path) -> list[ReplaySample]:
    command = [
        str(replay_binary),
        str(input_csv),
        f"--dry-mass-kg={HISTORICAL_FULLSCALE_DRY_MASS_KG}",
        f"--cp-offset-m={HISTORICAL_FULLSCALE_CP_OFFSET_M}",
        f"--moment-of-inertia-kgm2={HISTORICAL_FULLSCALE_MOI_KGM2}",
        "--seeded-flap-source=zero",
        "--include-raw=altitude_agl_m",
    ]
    result = subprocess.run(command, cwd=ROOT, text=True, capture_output=True, check=False)
    if result.returncode != 0:
        raise SystemExit(result.stderr.strip() or result.stdout.strip() or "acs_replay failed")

    header: list[str] | None = None
    samples: list[ReplaySample] = []
    for line in result.stdout.splitlines():
        if line.startswith("time_s,"):
            header = line.split(",")
            continue
        if header is None or not line or not line[0].isdigit():
            continue
        row = dict(zip(header, line.split(",")))
        time_s = parse_float(row.get("time_s"))
        altitude_m = parse_float(row.get("altitude_m"))
        velocity_mps = parse_float(row.get("velocity_mps"))
        apogee_m = parse_float(row.get("apogee_prediction_m"))
        if time_s is None or altitude_m is None or velocity_mps is None:
            continue
        samples.append(
            ReplaySample(
                time_s=time_s,
                status=(row.get("status") or "").strip(),
                altitude_ft=altitude_m * METERS_TO_FEET,
                apogee_ft=None if apogee_m is None else apogee_m * METERS_TO_FEET,
                velocity_fps=velocity_mps * METERS_TO_FEET,
            )
        )
    return samples


def write_summary_csv(path: Path,
                      log_samples: list[LogSample],
                      original: list[ReplaySample],
                      baro_seed: list[ReplaySample],
                      actual_apogee_ft: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    original_by_time = {round(sample.time_s, 4): sample for sample in original}
    baro_by_time = {round(sample.time_s, 4): sample for sample in baro_seed}
    with path.open("w", newline="") as handle:
        fields = [
            "time_s",
            "status",
            "baro_agl_ft",
            "state_agl_ft",
            "flap_effective_deg",
            "original_seed_apogee_ft",
            "baro_altitude_seed_apogee_ft",
            "actual_apogee_ft",
            "original_error_ft",
            "baro_altitude_seed_error_ft",
        ]
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for logged in log_samples:
            key = round(logged.time_s, 4)
            original_apogee = original_by_time.get(key).apogee_ft if key in original_by_time else None
            baro_apogee = baro_by_time.get(key).apogee_ft if key in baro_by_time else None
            writer.writerow(
                {
                    "time_s": f"{logged.time_s:.6f}",
                    "status": logged.status,
                    "baro_agl_ft": f"{logged.baro_agl_ft:.6f}",
                    "state_agl_ft": f"{logged.state_agl_ft:.6f}",
                    "flap_effective_deg": f"{logged.flap_effective_deg:.6f}",
                    "original_seed_apogee_ft": "" if original_apogee is None else f"{original_apogee:.6f}",
                    "baro_altitude_seed_apogee_ft": "" if baro_apogee is None else f"{baro_apogee:.6f}",
                    "actual_apogee_ft": f"{actual_apogee_ft:.6f}",
                    "original_error_ft": "" if original_apogee is None else f"{original_apogee - actual_apogee_ft:.6f}",
                    "baro_altitude_seed_error_ft": "" if baro_apogee is None else f"{baro_apogee - actual_apogee_ft:.6f}",
                }
            )


def scale(value: float, src_lo: float, src_hi: float, dst_lo: float, dst_hi: float) -> float:
    if src_hi <= src_lo:
        return 0.5 * (dst_lo + dst_hi)
    return dst_lo + (value - src_lo) * (dst_hi - dst_lo) / (src_hi - src_lo)


def thin(points: list[tuple[float, float]], limit: int = 1800) -> list[tuple[float, float]]:
    if len(points) <= limit:
        return points
    step = max(1, len(points) // limit)
    thinned = points[::step]
    if thinned[-1] != points[-1]:
        thinned.append(points[-1])
    return thinned


def polyline(points: list[tuple[float, float]],
             x_min: float,
             x_max: float,
             y_min: float,
             y_max: float,
             left: float,
             top: float,
             width: float,
             height: float) -> str:
    bottom = top + height
    out: list[str] = []
    for x_value, y_value in thin(points):
        if not math.isfinite(x_value) or not math.isfinite(y_value):
            continue
        x = scale(x_value, x_min, x_max, left, left + width)
        y = scale(y_value, y_min, y_max, bottom, top)
        out.append(f"{x:.2f},{y:.2f}")
    return " ".join(out)


def ticks(y_min: float, y_max: float, step: float) -> list[float]:
    first = math.ceil(y_min / step) * step
    values: list[float] = []
    value = first
    while value <= y_max + 0.5 * step:
        values.append(value)
        value += step
    return values


def draw_panel(title: str,
               x_min: float,
               x_max: float,
               y_min: float,
               y_max: float,
               y_step: float,
               left: float,
               top: float,
               width: float,
               height: float,
               series: list[tuple[str, list[tuple[float, float]], str]],
               reference_lines: list[tuple[str, float, str]],
               vertical_lines: list[tuple[str, float, str]]) -> str:
    bottom = top + height
    parts = [
        f'<text x="{left:.0f}" y="{top - 14:.0f}" class="title">{html.escape(title)}</text>',
        f'<rect x="{left:.0f}" y="{top:.0f}" width="{width:.0f}" height="{height:.0f}" class="panel"/>',
    ]
    for tick in ticks(y_min, y_max, y_step):
        y = scale(tick, y_min, y_max, bottom, top)
        parts.append(f'<line x1="{left:.0f}" y1="{y:.2f}" x2="{left + width:.0f}" y2="{y:.2f}" class="grid"/>')
        parts.append(f'<text x="{left - 8:.0f}" y="{y + 4:.2f}" class="axis" text-anchor="end">{tick:.0f}</text>')
    for second in range(math.ceil(x_min), math.floor(x_max) + 1):
        x = scale(second, x_min, x_max, left, left + width)
        parts.append(f'<line x1="{x:.2f}" y1="{top:.0f}" x2="{x:.2f}" y2="{bottom:.0f}" class="grid vertical"/>')
        parts.append(f'<text x="{x:.2f}" y="{bottom + 18:.0f}" class="axis" text-anchor="middle">{second}</text>')
    for label, value, color in reference_lines:
        if y_min <= value <= y_max:
            y = scale(value, y_min, y_max, bottom, top)
            parts.append(f'<line x1="{left:.0f}" y1="{y:.2f}" x2="{left + width:.0f}" y2="{y:.2f}" stroke="{color}" stroke-width="1.4" stroke-dasharray="6 5"/>')
            parts.append(f'<text x="{left + width - 8:.0f}" y="{y - 6:.2f}" class="note" fill="{color}" text-anchor="end">{html.escape(label)}</text>')
    for label, time_s, color in vertical_lines:
        if x_min <= time_s <= x_max:
            x = scale(time_s, x_min, x_max, left, left + width)
            parts.append(f'<line x1="{x:.2f}" y1="{top:.0f}" x2="{x:.2f}" y2="{bottom:.0f}" stroke="{color}" stroke-width="1.2" stroke-dasharray="5 4"/>')
            parts.append(f'<text x="{x + 5:.2f}" y="{top + 14:.0f}" class="note" fill="{color}">{html.escape(label)}</text>')
    for _, points, color in series:
        svg_points = polyline(points, x_min, x_max, y_min, y_max, left, top, width, height)
        if svg_points:
            parts.append(f'<polyline points="{svg_points}" fill="none" stroke="{color}" stroke-width="2.1" stroke-linejoin="round" stroke-linecap="round"/>')
    return "\n".join(parts)


def write_svg(path: Path,
              log_samples: list[LogSample],
              original: list[ReplaySample],
              baro_seed: list[ReplaySample],
              actual_apogee_ft: float,
              actual_apogee_time_s: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    x_min = 187.6
    x_max = 191.8
    window_logs = [sample for sample in log_samples if x_min <= sample.time_s <= x_max]
    window_original = [sample for sample in original if x_min <= sample.time_s <= x_max]
    window_baro = [sample for sample in baro_seed if x_min <= sample.time_s <= x_max]

    altitude_series = [
        ("baro AGL", [(sample.time_s, sample.baro_agl_ft) for sample in window_logs], "#111827"),
        ("state AGL", [(sample.time_s, sample.state_agl_ft) for sample in window_logs], "#64748b"),
        ("original seeded apogee", [(sample.time_s, sample.apogee_ft) for sample in window_original if sample.apogee_ft is not None], "#dc2626"),
        ("baro-alt seeded apogee", [(sample.time_s, sample.apogee_ft) for sample in window_baro if sample.apogee_ft is not None], "#059669"),
    ]
    error_series = [
        ("original error", [(sample.time_s, sample.apogee_ft - actual_apogee_ft) for sample in window_original if sample.apogee_ft is not None], "#dc2626"),
        ("baro-alt error", [(sample.time_s, sample.apogee_ft - actual_apogee_ft) for sample in window_baro if sample.apogee_ft is not None], "#059669"),
    ]
    state_delta_series = [
        ("state - baro", [(sample.time_s, sample.state_agl_ft - sample.baro_agl_ft) for sample in window_logs], "#7c3aed"),
    ]
    flap_series = [
        ("effective flap", [(sample.time_s, sample.flap_effective_deg) for sample in window_logs], "#0f766e"),
    ]

    width = 1280
    height = 1120
    left = 96
    panel_width = 1100
    panel_height = 210
    gap = 70
    top1 = 94
    top2 = top1 + panel_height + gap
    top3 = top2 + panel_height + gap
    top4 = top3 + panel_height + gap
    vertical_lines = [("baro apogee", actual_apogee_time_s, "#047857")]

    parts = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        "<style>",
        "text { font-family: Inter, Helvetica, Arial, sans-serif; }",
        ".heading { font-size: 22px; font-weight: 700; fill: #111827; }",
        ".sub { font-size: 13px; fill: #4b5563; }",
        ".title { font-size: 15px; font-weight: 700; fill: #111827; }",
        ".axis { font-size: 11px; fill: #4b5563; }",
        ".note { font-size: 11px; fill: #4b5563; }",
        ".panel { fill: #fff; stroke: #d1d5db; stroke-width: 1; }",
        ".grid { stroke: #e5e7eb; stroke-width: 1; }",
        ".vertical { stroke: #f3f4f6; }",
        "</style>",
        f'<rect x="0" y="0" width="{width}" height="{height}" fill="#f8fafc"/>',
        '<text x="48" y="42" class="heading">Fullscale 4 Baro-Altitude Predictor Seed</text>',
        f'<text x="48" y="64" class="sub">Actual baro apogee {actual_apogee_ft:.1f} ft AGL. Green replaces only predictor seed altitude with fresh baro AGL.</text>',
    ]

    legend = [
        ("baro AGL", "#111827"),
        ("state AGL", "#64748b"),
        ("original", "#dc2626"),
        ("baro seed", "#059669"),
    ]
    for index, (label, color) in enumerate(legend):
        x = 770 + index * 115
        parts.append(f'<line x1="{x}" y1="38" x2="{x + 28}" y2="38" stroke="{color}" stroke-width="3"/>')
        parts.append(f'<text x="{x + 36}" y="42" class="sub">{html.escape(label)}</text>')

    parts.append(draw_panel("Altitude and predicted apogee (ft AGL)", x_min, x_max, 2450, 2925, 100, left, top1, panel_width, panel_height, altitude_series, [("actual apogee", actual_apogee_ft, "#047857")], vertical_lines))
    parts.append(draw_panel("Prediction error vs actual apogee (ft)", x_min, x_max, -200, 50, 50, left, top2, panel_width, panel_height, error_series, [("zero error", 0, "#047857")], vertical_lines))
    parts.append(draw_panel("Estimator altitude error: state AGL - baro AGL (ft)", x_min, x_max, -200, 25, 25, left, top3, panel_width, panel_height, state_delta_series, [( "zero", 0, "#047857")], vertical_lines))
    parts.append(draw_panel("Effective flap angle (deg)", x_min, x_max, 0, 45, 5, left, top4, panel_width, panel_height, flap_series, [], vertical_lines))
    parts.append("</svg>")
    path.write_text("\n".join(parts) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-csv", type=Path, default=DEFAULT_INPUT_CSV)
    parser.add_argument("--output-csv", type=Path, default=DEFAULT_OUTPUT_CSV)
    parser.add_argument("--output-svg", type=Path, default=DEFAULT_OUTPUT_SVG)
    parser.add_argument("--replay-binary", type=Path)
    args = parser.parse_args()

    replay_binary = find_replay_binary(args.replay_binary)
    log_samples = load_log_samples(args.input_csv)
    actual_sample = max(log_samples, key=lambda sample: sample.baro_agl_ft)

    original = run_replay(replay_binary, args.input_csv)
    with tempfile.NamedTemporaryFile(suffix="_fullscale4_baro_seed.csv", dir="/tmp", delete=False) as handle:
        baro_seed_csv = Path(handle.name)
    try:
        write_baro_seed_input(args.input_csv, baro_seed_csv)
        baro_seed = run_replay(replay_binary, baro_seed_csv)
    finally:
        try:
            os.unlink(baro_seed_csv)
        except OSError:
            pass

    write_summary_csv(args.output_csv, log_samples, original, baro_seed, actual_sample.baro_agl_ft)
    write_svg(args.output_svg, log_samples, original, baro_seed, actual_sample.baro_agl_ft, actual_sample.time_s)

    print(f"Actual baro apogee: {actual_sample.baro_agl_ft:.1f} ft at t={actual_sample.time_s:.3f}s")
    for target_time_s in (188.2, 189.0, 190.0, 191.0):
        original_row = min(original, key=lambda sample: abs(sample.time_s - target_time_s))
        baro_row = min(baro_seed, key=lambda sample: abs(sample.time_s - target_time_s))
        print(
            f"t={target_time_s:.1f}s original={original_row.apogee_ft:.1f}ft "
            f"({original_row.apogee_ft - actual_sample.baro_agl_ft:+.1f}), "
            f"baro-seed={baro_row.apogee_ft:.1f}ft "
            f"({baro_row.apogee_ft - actual_sample.baro_agl_ft:+.1f})"
        )
    print(f"Wrote CSV: {args.output_csv}")
    print(f"Wrote SVG: {args.output_svg}")


if __name__ == "__main__":
    main()
