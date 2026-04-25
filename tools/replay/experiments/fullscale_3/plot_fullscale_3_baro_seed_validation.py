#!/usr/bin/env python3
"""
Build an altitude-seeded fullscale 3 replay and plot the current apogee model.

The raw fullscale 3 altitude is usable, but the hosted replay cannot reconstruct
the old estimator state cleanly from the legacy IMU path. This script leaves the
source log untouched and creates explicit baro-seeded validation inputs:

- state_position_z comes from raw altitude AGL.
- state_velocity_z comes from a local linear fit of raw altitude.
- state_zenith_deg is fixed at 0 for a vertical, altitude-only predictor check.

The default output includes both a centered fit, useful as an offline repaired
state, and a trailing fit, useful as a causal sanity check.
"""

from __future__ import annotations

import argparse
import csv
import html
import math
import os
import subprocess
from dataclasses import dataclass
from pathlib import Path

from plot_no_deweight_apogee_replay import find_replay_binary
from replay_legacy_frame_adapter import HISTORICAL_FULLSCALE_CP_OFFSET_M
from replay_legacy_frame_adapter import HISTORICAL_FULLSCALE_DRY_MASS_KG
from replay_legacy_frame_adapter import HISTORICAL_FULLSCALE_MOI_KGM2


from replaylib.paths import SCRIPTS_DIR
SCRIPT_DIR = SCRIPTS_DIR
ROOT = SCRIPT_DIR.parents[2]

DEFAULT_INPUT_CSV = ROOT / "tools" / "replay" / "data" / "fullscale_3_bno_from_icm_sensor_only.csv"
DEFAULT_SEED_PREFIX = ROOT / "tools" / "replay" / "data" / "fullscale_3_baro_seeded"
DEFAULT_OUTPUT_CSV = ROOT / "tools" / "replay" / "data" / "fullscale_3_baro_seeded_validation.csv"
DEFAULT_OUTPUT_SVG = ROOT / "tools" / "replay" / "plots" / "fullscale_3_baro_seeded_validation.svg"

FEET_TO_METERS = 0.3048
METERS_TO_FEET = 1.0 / FEET_TO_METERS
MPS_TO_FPS = METERS_TO_FEET
G_MPS2 = 9.80665


@dataclass
class AltitudeSample:
    time_s: float
    altitude_ft: float
    altitude_agl_m: float


@dataclass
class ReplaySample:
    source: str
    time_s: float
    altitude_m: float
    velocity_mps: float
    apogee_prediction_m: float
    status: str
    altitude_agl_m: float | None
    zenith_deg: float | None
    horizontal_velocity_mps: float | None
    actual_apogee_m: float

    @property
    def prediction_error_m(self) -> float:
        return self.apogee_prediction_m - self.actual_apogee_m


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


def load_altitude_samples(path: Path) -> list[AltitudeSample]:
    rows: list[tuple[float, float]] = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(line for line in handle if not line.startswith("#"))
        for row in reader:
            time_s = parse_float(row.get("timestamp") or row.get("time_s") or row.get("sensor_timestamp"))
            altitude_ft = parse_float(row.get("altitude_feet") or row.get("altitude_ft") or row.get("sensor_altitude_feet"))
            if time_s is None or altitude_ft is None:
                continue
            rows.append((time_s, altitude_ft))
    if not rows:
        raise SystemExit(f"No timestamp/altitude rows found in {path}")

    pad_altitude_ft = rows[0][1]
    return [
        AltitudeSample(time_s=time_s,
                       altitude_ft=altitude_ft,
                       altitude_agl_m=(altitude_ft - pad_altitude_ft) * FEET_TO_METERS)
        for time_s, altitude_ft in rows
    ]


def local_linear_slopes(samples: list[AltitudeSample], window_s: float, mode: str) -> list[float]:
    if window_s <= 0.0:
        raise SystemExit("--velocity-window-s must be positive")
    if mode not in {"centered", "trailing"}:
        raise SystemExit(f"Unsupported slope mode: {mode}")

    times = [sample.time_s for sample in samples]
    altitudes = [sample.altitude_agl_m for sample in samples]
    count = len(samples)

    prefix_t = [0.0] * (count + 1)
    prefix_z = [0.0] * (count + 1)
    prefix_tz = [0.0] * (count + 1)
    prefix_t2 = [0.0] * (count + 1)
    for index, (time_s, altitude_m) in enumerate(zip(times, altitudes)):
        prefix_t[index + 1] = prefix_t[index] + time_s
        prefix_z[index + 1] = prefix_z[index] + altitude_m
        prefix_tz[index + 1] = prefix_tz[index] + time_s * altitude_m
        prefix_t2[index + 1] = prefix_t2[index] + time_s * time_s

    slopes = [0.0] * count
    lo = 0
    hi = 0
    for index, time_s in enumerate(times):
        if mode == "centered":
            lo_time = time_s - 0.5 * window_s
            hi_time = time_s + 0.5 * window_s
        else:
            lo_time = time_s - window_s
            hi_time = time_s

        while lo < count and times[lo] < lo_time:
            lo += 1
        while hi < count and times[hi] <= hi_time:
            hi += 1

        n = hi - lo
        if n < 4:
            continue
        sum_t = prefix_t[hi] - prefix_t[lo]
        sum_z = prefix_z[hi] - prefix_z[lo]
        sum_tz = prefix_tz[hi] - prefix_tz[lo]
        sum_t2 = prefix_t2[hi] - prefix_t2[lo]
        denominator = n * sum_t2 - sum_t * sum_t
        if abs(denominator) > 1.0e-12:
            slopes[index] = (n * sum_tz - sum_t * sum_z) / denominator
    return slopes


def flight_status(altitude_agl_m: float,
                  velocity_mps: float,
                  time_s: float,
                  coast_start_s: float,
                  liftoff_altitude_m: float) -> str:
    if altitude_agl_m < liftoff_altitude_m:
        return "ground"
    if time_s < coast_start_s:
        return "burn"
    if velocity_mps > 0.0:
        return "coast"
    return "descent"


def seed_path_for_mode(seed_prefix: Path, mode: str) -> Path:
    return seed_prefix.with_name(f"{seed_prefix.name}_{mode}_input.csv")


def write_seed_csv(path: Path,
                   samples: list[AltitudeSample],
                   velocities_mps: list[float],
                   coast_start_s: float,
                   liftoff_altitude_m: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "timestamp",
        "altitude_feet",
        "has_filtered_state",
        "flight_status",
        "state_time",
        "state_position_x",
        "state_position_y",
        "state_position_z",
        "state_velocity_x",
        "state_velocity_y",
        "state_velocity_z",
        "state_zenith_deg",
        "state_apogee_estimate",
        "flap_command_deg",
        "flap_effective_deg",
    ]
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for sample, velocity_mps in zip(samples, velocities_mps):
            status = flight_status(sample.altitude_agl_m,
                                   velocity_mps,
                                   sample.time_s,
                                   coast_start_s,
                                   liftoff_altitude_m)
            writer.writerow(
                {
                    "timestamp": f"{sample.time_s:.9f}",
                    "altitude_feet": f"{sample.altitude_ft:.6f}",
                    "has_filtered_state": "1",
                    "flight_status": status,
                    "state_time": f"{sample.time_s:.9f}",
                    "state_position_x": "0.0",
                    "state_position_y": "0.0",
                    "state_position_z": f"{sample.altitude_agl_m:.9f}",
                    "state_velocity_x": "0.0",
                    "state_velocity_y": "0.0",
                    "state_velocity_z": f"{velocity_mps:.9f}",
                    "state_zenith_deg": "0.0",
                    "state_apogee_estimate": f"{sample.altitude_agl_m:.9f}",
                    "flap_command_deg": "0.0",
                    "flap_effective_deg": "0.0",
                }
            )


def parse_replay_stdout(source: str, stdout: str, actual_apogee_m: float) -> list[ReplaySample]:
    lines = stdout.splitlines()
    header_index = next((index for index, line in enumerate(lines) if line.startswith("time_s,")), None)
    if header_index is None:
        raise SystemExit(f"Replay output for {source} did not contain a CSV header")

    parsed: list[ReplaySample] = []
    reader = csv.DictReader(lines[header_index:])
    for row in reader:
        time_s = parse_float(row.get("time_s"))
        altitude_m = parse_float(row.get("altitude_m"))
        velocity_mps = parse_float(row.get("velocity_mps"))
        apogee_prediction_m = parse_float(row.get("apogee_prediction_m"))
        if time_s is None or altitude_m is None or velocity_mps is None or apogee_prediction_m is None:
            continue
        parsed.append(
            ReplaySample(
                source=source,
                time_s=time_s,
                altitude_m=altitude_m,
                velocity_mps=velocity_mps,
                apogee_prediction_m=apogee_prediction_m,
                status=(row.get("status") or "").strip(),
                altitude_agl_m=parse_float(row.get("altitude_agl_m")),
                zenith_deg=parse_float(row.get("zenith_deg")),
                horizontal_velocity_mps=parse_float(row.get("horizontal_velocity_mps")),
                actual_apogee_m=actual_apogee_m,
            )
        )
    return parsed


def run_replay(binary: Path,
               seed_csv: Path,
               source: str,
               actual_apogee_m: float,
               dry_mass_kg: float,
               cp_offset_m: float,
               moment_of_inertia_kgm2: float) -> list[ReplaySample]:
    command = [
        str(binary),
        str(seed_csv),
        "--dry-mass-kg",
        str(dry_mass_kg),
        "--cp-offset-m",
        str(cp_offset_m),
        "--moment-of-inertia-kgm2",
        str(moment_of_inertia_kgm2),
        "--include-raw",
        "altitude_agl_m,zenith_deg,horizontal_velocity_mps",
    ]
    result = subprocess.run(command, cwd=ROOT, text=True, capture_output=True, check=False)
    if result.returncode != 0:
        raise SystemExit(
            f"acs_replay failed for {source} with exit code {result.returncode}\n"
            f"stdout:\n{result.stdout}\n"
            f"stderr:\n{result.stderr}"
        )
    return parse_replay_stdout(source, result.stdout, actual_apogee_m)


def write_replay_csv(path: Path, runs: dict[str, list[ReplaySample]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "source",
        "time_s",
        "altitude_m",
        "velocity_mps",
        "apogee_prediction_m",
        "status",
        "altitude_agl_m",
        "zenith_deg",
        "horizontal_velocity_mps",
        "actual_apogee_m",
        "prediction_error_m",
    ]
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for rows in runs.values():
            for sample in rows:
                writer.writerow(
                    {
                        "source": sample.source,
                        "time_s": f"{sample.time_s:.6f}",
                        "altitude_m": f"{sample.altitude_m:.6f}",
                        "velocity_mps": f"{sample.velocity_mps:.6f}",
                        "apogee_prediction_m": f"{sample.apogee_prediction_m:.6f}",
                        "status": sample.status,
                        "altitude_agl_m": "" if sample.altitude_agl_m is None else f"{sample.altitude_agl_m:.6f}",
                        "zenith_deg": "" if sample.zenith_deg is None else f"{sample.zenith_deg:.6f}",
                        "horizontal_velocity_mps": "" if sample.horizontal_velocity_mps is None else f"{sample.horizontal_velocity_mps:.6f}",
                        "actual_apogee_m": f"{sample.actual_apogee_m:.6f}",
                        "prediction_error_m": f"{sample.prediction_error_m:.6f}",
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
    for time_s, value in thin(points):
        if not math.isfinite(time_s) or not math.isfinite(value):
            continue
        x = scale(time_s, x_min, x_max, left, left + width)
        y = scale(value, y_min, y_max, bottom, top)
        out.append(f"{x:.2f},{y:.2f}")
    return " ".join(out)


def nice_ticks(y_min: float, y_max: float, count: int = 5) -> list[float]:
    if y_max <= y_min:
        return [y_min]
    raw_step = (y_max - y_min) / max(1, count - 1)
    magnitude = 10 ** math.floor(math.log10(raw_step))
    normalized = raw_step / magnitude
    if normalized <= 1.5:
        step = magnitude
    elif normalized <= 3.0:
        step = 2.0 * magnitude
    elif normalized <= 7.0:
        step = 5.0 * magnitude
    else:
        step = 10.0 * magnitude
    first = math.ceil(y_min / step) * step
    ticks = []
    value = first
    while value <= y_max + 0.5 * step:
        ticks.append(value)
        value += step
    return ticks


def draw_panel(title: str,
               x_min: float,
               x_max: float,
               y_min: float,
               y_max: float,
               left: float,
               top: float,
               width: float,
               height: float,
               series: list[tuple[str, list[tuple[float, float]], str]],
               reference_lines: list[tuple[str, float, str]],
               vertical_lines: list[tuple[str, float, str]]) -> str:
    bottom = top + height
    lines: list[str] = []
    lines.append(f'<text x="{left:.0f}" y="{top - 14:.0f}" class="title">{html.escape(title)}</text>')
    lines.append(f'<rect x="{left:.0f}" y="{top:.0f}" width="{width:.0f}" height="{height:.0f}" class="panel"/>')

    for tick in nice_ticks(y_min, y_max):
        y = scale(tick, y_min, y_max, bottom, top)
        lines.append(f'<line x1="{left:.0f}" y1="{y:.2f}" x2="{left + width:.0f}" y2="{y:.2f}" class="grid"/>')
        lines.append(f'<text x="{left - 8:.0f}" y="{y + 4:.2f}" class="axis" text-anchor="end">{tick:.0f}</text>')

    for second in range(math.ceil(x_min), math.floor(x_max) + 1):
        if second % 2:
            continue
        x = scale(second, x_min, x_max, left, left + width)
        lines.append(f'<line x1="{x:.2f}" y1="{top:.0f}" x2="{x:.2f}" y2="{bottom:.0f}" class="grid vertical"/>')
        lines.append(f'<text x="{x:.2f}" y="{bottom + 18:.0f}" class="axis" text-anchor="middle">{second:.0f}</text>')

    for label, value, color in reference_lines:
        if y_min <= value <= y_max:
            y = scale(value, y_min, y_max, bottom, top)
            lines.append(
                f'<line x1="{left:.0f}" y1="{y:.2f}" x2="{left + width:.0f}" y2="{y:.2f}" '
                f'stroke="{color}" stroke-width="1.5" stroke-dasharray="6 5"/>'
            )
            lines.append(f'<text x="{left + width - 8:.0f}" y="{y - 6:.2f}" fill="{color}" class="note" text-anchor="end">{html.escape(label)}</text>')

    for label, time_s, color in vertical_lines:
        if x_min <= time_s <= x_max:
            x = scale(time_s, x_min, x_max, left, left + width)
            lines.append(
                f'<line x1="{x:.2f}" y1="{top:.0f}" x2="{x:.2f}" y2="{bottom:.0f}" '
                f'stroke="{color}" stroke-width="1.2" stroke-dasharray="5 4"/>'
            )
            lines.append(f'<text x="{x + 5:.2f}" y="{top + 14:.0f}" fill="{color}" class="note">{html.escape(label)}</text>')

    for label, points, color in series:
        path_points = polyline(points, x_min, x_max, y_min, y_max, left, top, width, height)
        if path_points:
            lines.append(
                f'<polyline points="{path_points}" fill="none" stroke="{color}" '
                f'stroke-width="2.0" stroke-linejoin="round" stroke-linecap="round"/>'
            )
    return "\n".join(lines)


def write_svg(path: Path,
              altitude_samples: list[AltitudeSample],
              velocities_by_mode: dict[str, list[float]],
              runs: dict[str, list[ReplaySample]],
              actual_apogee_m: float,
              actual_apogee_time_s: float,
              coast_start_s: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    colors = {"centered": "#2563eb", "trailing": "#ea580c"}
    x_start = coast_start_s - 3.0
    x_end = actual_apogee_time_s + 2.0

    flight_altitude = [
        (sample.time_s, sample.altitude_agl_m * METERS_TO_FEET)
        for sample in altitude_samples
        if x_start <= sample.time_s <= x_end
    ]
    altitude_series: list[tuple[str, list[tuple[float, float]], str]] = [
        ("raw altitude", flight_altitude, "#111827")
    ]
    velocity_series: list[tuple[str, list[tuple[float, float]], str]] = []
    error_series: list[tuple[str, list[tuple[float, float]], str]] = []

    y_alt_values = [value for _, value in flight_altitude]
    y_error_values: list[float] = [0.0]
    y_velocity_values: list[float] = [0.0]
    for mode, rows in runs.items():
        color = colors.get(mode, "#64748b")
        prediction_points = [
            (row.time_s, row.apogee_prediction_m * METERS_TO_FEET)
            for row in rows
            if x_start <= row.time_s <= x_end and row.status in {"burn", "coast", "descent"}
        ]
        error_points = [
            (row.time_s, row.prediction_error_m * METERS_TO_FEET)
            for row in rows
            if x_start <= row.time_s <= x_end and row.status in {"burn", "coast", "descent"}
        ]
        altitude_series.append((f"{mode} predictor", prediction_points, color))
        error_series.append((f"{mode} error", error_points, color))
        y_alt_values.extend(value for _, value in prediction_points)
        y_error_values.extend(value for _, value in error_points)

    for mode, velocities in velocities_by_mode.items():
        color = colors.get(mode, "#64748b")
        points = [
            (sample.time_s, velocity * MPS_TO_FPS)
            for sample, velocity in zip(altitude_samples, velocities)
            if x_start <= sample.time_s <= x_end
        ]
        velocity_series.append((f"{mode} seed vz", points, color))
        y_velocity_values.extend(value for _, value in points)

    y_alt_min = 0.0
    y_alt_max = max(5000.0, math.ceil((max(y_alt_values) + 250.0) / 250.0) * 250.0)
    y_error_min = math.floor((min(y_error_values) - 250.0) / 250.0) * 250.0
    y_error_max = math.ceil((max(y_error_values) + 250.0) / 250.0) * 250.0
    y_velocity_min = math.floor((min(y_velocity_values) - 100.0) / 100.0) * 100.0
    y_velocity_max = math.ceil((max(y_velocity_values) + 100.0) / 100.0) * 100.0

    width = 1280
    height = 960
    left = 96
    panel_width = 1100
    panel_height = 230
    panel_gap = 78
    top1 = 92
    top2 = top1 + panel_height + panel_gap
    top3 = top2 + panel_height + panel_gap
    vertical_lines = [
        ("coast seed", coast_start_s, "#7c3aed"),
        ("raw apogee", actual_apogee_time_s, "#047857"),
    ]

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
        ".panel { fill: #ffffff; stroke: #d1d5db; stroke-width: 1; }",
        ".grid { stroke: #e5e7eb; stroke-width: 1; }",
        ".vertical { stroke: #f3f4f6; }",
        "</style>",
        f'<rect x="0" y="0" width="{width}" height="{height}" fill="#f8fafc"/>',
        '<text x="48" y="42" class="heading">Fullscale 3 Baro-Seeded Current Predictor Validation</text>',
        (
            f'<text x="48" y="64" class="sub">Raw altitude apogee '
            f'{actual_apogee_m * METERS_TO_FEET:.1f} ft at t={actual_apogee_time_s:.3f}s; '
            f"centered fit is offline, trailing fit is causal.</text>"
        ),
    ]

    legend_x = 795
    legend_y = 36
    legend_items = [("raw altitude", "#111827"), ("centered", "#2563eb"), ("trailing", "#ea580c")]
    for offset, (label, color) in enumerate(legend_items):
        x = legend_x + offset * 145
        parts.append(f'<line x1="{x}" y1="{legend_y}" x2="{x + 30}" y2="{legend_y}" stroke="{color}" stroke-width="3"/>')
        parts.append(f'<text x="{x + 38}" y="{legend_y + 4}" class="sub">{html.escape(label)}</text>')

    parts.append(
        draw_panel(
            "Altitude and predicted apogee (ft AGL)",
            x_start,
            x_end,
            y_alt_min,
            y_alt_max,
            left,
            top1,
            panel_width,
            panel_height,
            altitude_series,
            [("actual apogee", actual_apogee_m * METERS_TO_FEET, "#047857")],
            vertical_lines,
        )
    )
    parts.append(
        draw_panel(
            "Prediction error vs raw apogee (ft)",
            x_start,
            x_end,
            y_error_min,
            y_error_max,
            left,
            top2,
            panel_width,
            panel_height,
            error_series,
            [("zero error", 0.0, "#047857")],
            vertical_lines,
        )
    )
    parts.append(
        draw_panel(
            "Seeded vertical velocity from raw altitude (ft/s)",
            x_start,
            x_end,
            y_velocity_min,
            y_velocity_max,
            left,
            top3,
            panel_width,
            panel_height,
            velocity_series,
            [],
            vertical_lines,
        )
    )
    parts.append("</svg>")
    path.write_text("\n".join(parts) + "\n", encoding="utf-8")


def nearest_sample(rows: list[ReplaySample], target_time_s: float) -> ReplaySample:
    return min(rows, key=lambda row: abs(row.time_s - target_time_s))


def summarize_mode(rows: list[ReplaySample], actual_apogee_m: float, start_s: float, end_s: float) -> tuple[float, float, float]:
    window = [
        abs(row.prediction_error_m)
        for row in rows
        if start_s <= row.time_s <= end_s and row.status in {"coast", "descent"}
    ]
    if not window:
        return math.nan, math.nan, math.nan
    signed = [
        row.prediction_error_m
        for row in rows
        if start_s <= row.time_s <= end_s and row.status in {"coast", "descent"}
    ]
    return sum(window) / len(window), min(signed), max(signed)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-csv", type=Path, default=DEFAULT_INPUT_CSV)
    parser.add_argument("--seed-prefix", type=Path, default=DEFAULT_SEED_PREFIX)
    parser.add_argument("--output-csv", type=Path, default=DEFAULT_OUTPUT_CSV)
    parser.add_argument("--output-svg", type=Path, default=DEFAULT_OUTPUT_SVG)
    parser.add_argument("--replay-binary", type=Path)
    parser.add_argument("--modes", nargs="+", choices=("centered", "trailing"), default=["centered", "trailing"])
    parser.add_argument("--velocity-window-s", type=float, default=6.0)
    parser.add_argument("--coast-start-s", type=float, default=331.679138)
    parser.add_argument("--liftoff-altitude-m", type=float, default=40.0)
    parser.add_argument("--dry-mass-kg", type=float, default=HISTORICAL_FULLSCALE_DRY_MASS_KG)
    parser.add_argument("--cp-offset-m", type=float, default=HISTORICAL_FULLSCALE_CP_OFFSET_M)
    parser.add_argument("--moment-of-inertia-kgm2", type=float, default=HISTORICAL_FULLSCALE_MOI_KGM2)
    args = parser.parse_args()

    samples = load_altitude_samples(args.input_csv)
    actual_sample = max(samples, key=lambda sample: sample.altitude_agl_m)
    actual_apogee_m = actual_sample.altitude_agl_m
    replay_binary = find_replay_binary(args.replay_binary)

    velocities_by_mode: dict[str, list[float]] = {}
    runs: dict[str, list[ReplaySample]] = {}
    for mode in args.modes:
        velocities = local_linear_slopes(samples, args.velocity_window_s, mode)
        seed_csv = seed_path_for_mode(args.seed_prefix, mode)
        write_seed_csv(seed_csv, samples, velocities, args.coast_start_s, args.liftoff_altitude_m)
        runs[mode] = run_replay(
            replay_binary,
            seed_csv,
            mode,
            actual_apogee_m,
            args.dry_mass_kg,
            args.cp_offset_m,
            args.moment_of_inertia_kgm2,
        )
        velocities_by_mode[mode] = velocities

    write_replay_csv(args.output_csv, runs)
    write_svg(args.output_svg,
              samples,
              velocities_by_mode,
              runs,
              actual_apogee_m,
              actual_sample.time_s,
              args.coast_start_s)

    print(f"Input altitude log: {args.input_csv}")
    print(f"Raw altitude apogee: {actual_apogee_m:.3f} m ({actual_apogee_m * METERS_TO_FEET:.1f} ft) at t={actual_sample.time_s:.3f} s")
    for mode, rows in runs.items():
        mae_338, lo_338, hi_338 = summarize_mode(rows, actual_apogee_m, 338.0, actual_sample.time_s)
        mae_333, lo_333, hi_333 = summarize_mode(rows, actual_apogee_m, 333.0, actual_sample.time_s)
        print(
            f"{mode}: mean abs error 333s-apogee={mae_333:.2f} m "
            f"({mae_333 * METERS_TO_FEET:.1f} ft), "
            f"338s-apogee={mae_338:.2f} m ({mae_338 * METERS_TO_FEET:.1f} ft), "
            f"338s-apogee range=[{lo_338:.2f}, {hi_338:.2f}] m"
        )
        for target_time_s in (332.0, 335.0, 338.0, 340.0, 342.0, 344.0):
            row = nearest_sample(rows, target_time_s)
            print(
                f"  t={row.time_s:.2f}s {row.status:7s} "
                f"alt={row.altitude_m:.1f}m apogee={row.apogee_prediction_m:.1f}m "
                f"err={row.prediction_error_m:+.1f}m"
            )
    print(f"Wrote replay CSV: {args.output_csv}")
    print(f"Wrote plot: {args.output_svg}")


if __name__ == "__main__":
    main()
