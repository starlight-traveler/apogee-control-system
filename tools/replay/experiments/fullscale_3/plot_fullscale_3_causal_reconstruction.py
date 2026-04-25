#!/usr/bin/env python3
"""
Build causal fullscale 3 apogee reconstructions from baro, ICM, and LSM.

This is intentionally an offline validation script, not flight firmware. It
uses only samples at or before the current timestamp to estimate vertical
velocity, then feeds that state into the current hosted `acs_replay` predictor.

Modes:
- baro_trailing: raw altitude AGL plus trailing local-linear baro velocity.
- icm_baro: raw altitude AGL plus ICM vertical accel integrated with baro correction.
- lsm_baro: raw altitude AGL plus LSM vertical accel integrated with baro correction.
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

DEFAULT_INPUT_CSV = ROOT / "tools" / "replay" / "data" / "fullscale_3_historical.csv"
DEFAULT_SEED_PREFIX = ROOT / "tools" / "replay" / "data" / "fullscale_3_causal_reconstruction"
DEFAULT_OUTPUT_CSV = ROOT / "tools" / "replay" / "data" / "fullscale_3_causal_reconstruction.csv"
DEFAULT_OUTPUT_SVG = ROOT / "tools" / "replay" / "plots" / "fullscale_3_causal_reconstruction.svg"

FEET_TO_METERS = 0.3048
METERS_TO_FEET = 1.0 / FEET_TO_METERS
MPS_TO_FPS = METERS_TO_FEET
G_MPS2 = 9.80665


@dataclass
class RawSample:
    time_s: float
    altitude_ft: float
    altitude_agl_m: float
    status: str
    flap_command_deg: float
    icm_accel: tuple[float, float, float] | None
    lsm_accel: tuple[float, float, float] | None
    icm_quat: tuple[float, float, float, float] | None
    lsm_quat: tuple[float, float, float, float] | None


@dataclass
class SeededSample:
    source: str
    time_s: float
    altitude_m: float
    velocity_mps: float
    zenith_deg: float
    flap_command_deg: float
    status: str


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


def parse_vec3(row: dict[str, str], prefix: str) -> tuple[float, float, float] | None:
    values = tuple(parse_float(row.get(f"{prefix}_{axis}")) for axis in ("x", "y", "z"))
    if any(value is None for value in values):
        return None
    return values  # type: ignore[return-value]


def parse_quat(row: dict[str, str], prefix: str) -> tuple[float, float, float, float] | None:
    values = tuple(parse_float(row.get(f"{prefix}_{axis}")) for axis in ("w", "x", "y", "z"))
    if any(value is None for value in values):
        return None
    normalized = normalize_quat(values)  # type: ignore[arg-type]
    return normalized


def normalize_quat(quat: tuple[float, float, float, float]) -> tuple[float, float, float, float] | None:
    norm = math.sqrt(sum(value * value for value in quat))
    if norm <= 1.0e-9 or not math.isfinite(norm):
        return None
    return tuple(value / norm for value in quat)  # type: ignore[return-value]


def quaternion_zenith_deg(quat: tuple[float, float, float, float] | None) -> float:
    if quat is None:
        return 0.0
    _, x, y, _ = quat
    cos_zenith = max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))
    return math.degrees(math.acos(abs(cos_zenith)))


def gravity_body_from_quaternion(quat: tuple[float, float, float, float] | None) -> tuple[float, float, float] | None:
    if quat is None:
        return None
    w, x, y, z = quat
    gx = -2.0 * (x * z - w * y)
    gy = -2.0 * (y * z + w * x)
    gz = -(1.0 - 2.0 * (x * x + y * y))
    if gz < 0.0:
        gx = -gx
        gy = -gy
        gz = -gz
    magnitude = math.sqrt(gx * gx + gy * gy + gz * gz)
    if magnitude <= 1.0e-9 or not math.isfinite(magnitude):
        return None
    return gx / magnitude, gy / magnitude, gz / magnitude


def dot3(lhs: tuple[float, float, float], rhs: tuple[float, float, float]) -> float:
    return lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2]


def vec3_magnitude(vec: tuple[float, float, float]) -> float:
    return math.sqrt(dot3(vec, vec))


def load_raw_samples(path: Path) -> list[RawSample]:
    rows: list[dict[str, str]] = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(line for line in handle if not line.startswith("#"))
        for row in reader:
            rows.append(row)
    if not rows:
        raise SystemExit(f"No rows found in {path}")

    first_altitude = parse_float(rows[0].get("sensor_altitude_feet"))
    if first_altitude is None:
        raise SystemExit(f"First row in {path} has no sensor_altitude_feet")

    samples: list[RawSample] = []
    for row in rows:
        time_s = parse_float(row.get("sensor_timestamp"))
        altitude_ft = parse_float(row.get("sensor_altitude_feet"))
        if time_s is None or altitude_ft is None:
            continue
        samples.append(
            RawSample(
                time_s=time_s,
                altitude_ft=altitude_ft,
                altitude_agl_m=(altitude_ft - first_altitude) * FEET_TO_METERS,
                status=(row.get("flight_status") or "").strip().lower(),
                flap_command_deg=parse_float(row.get("sensor_auto_cmd_deg")) or 0.0,
                icm_accel=parse_vec3(row, "sensor_accel_icm"),
                lsm_accel=parse_vec3(row, "sensor_accel_lsm"),
                icm_quat=parse_quat(row, "sensor_icm_quat"),
                lsm_quat=parse_quat(row, "sensor_lsm_quat"),
            )
        )
    if not samples:
        raise SystemExit(f"No usable timestamp/altitude rows found in {path}")
    return samples


def local_linear_trailing_velocity(samples: list[RawSample], window_s: float) -> list[float]:
    if window_s <= 0.0:
        raise SystemExit("--baro-window-s must be positive")

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

    velocities = [0.0] * count
    lo = 0
    hi = 0
    for index, time_s in enumerate(times):
        while lo < count and times[lo] < time_s - window_s:
            lo += 1
        while hi < count and times[hi] <= time_s:
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
            velocities[index] = (n * sum_tz - sum_t * sum_z) / denominator
    return velocities


def vertical_accel_from_rail(sample: RawSample, rail: str, max_accel_mag_mps2: float) -> float | None:
    accel = sample.icm_accel if rail == "icm" else sample.lsm_accel
    quat = sample.icm_quat if rail == "icm" else sample.lsm_quat
    gravity_body = gravity_body_from_quaternion(quat)
    if accel is None or gravity_body is None:
        return None
    magnitude = vec3_magnitude(accel)
    if magnitude < 0.1 or magnitude > max_accel_mag_mps2:
        return None
    return dot3(accel, gravity_body) - G_MPS2


def estimate_accel_bias(samples: list[RawSample],
                        rail: str,
                        max_accel_mag_mps2: float,
                        ground_end_s: float) -> float:
    values = [
        vertical_accel_from_rail(sample, rail, max_accel_mag_mps2)
        for sample in samples
        if sample.status == "ground" and sample.time_s <= ground_end_s
    ]
    finite = [value for value in values if value is not None and math.isfinite(value)]
    if not finite:
        return 0.0
    return sum(finite) / len(finite)


def reconstruct_imu_velocity(samples: list[RawSample],
                             rail: str,
                             baro_velocity_mps: list[float],
                             coast_start_s: float,
                             ground_bias_end_s: float,
                             alpha: float,
                             beta: float,
                             max_accel_mag_mps2: float) -> list[float]:
    bias = estimate_accel_bias(samples, rail, max_accel_mag_mps2, ground_bias_end_s)
    velocities = [0.0] * len(samples)
    filter_altitude_m = samples[0].altitude_agl_m
    filter_velocity_mps = baro_velocity_mps[0]
    last_time_s = samples[0].time_s
    active = False

    for index, sample in enumerate(samples):
        if sample.time_s < coast_start_s:
            velocities[index] = baro_velocity_mps[index]
            filter_altitude_m = sample.altitude_agl_m
            filter_velocity_mps = baro_velocity_mps[index]
            last_time_s = sample.time_s
            continue

        if not active:
            active = True
            filter_altitude_m = sample.altitude_agl_m
            filter_velocity_mps = baro_velocity_mps[index]
            last_time_s = sample.time_s
            velocities[index] = filter_velocity_mps
            continue

        dt = sample.time_s - last_time_s
        last_time_s = sample.time_s
        if dt <= 0.0 or dt > 0.25:
            velocities[index] = filter_velocity_mps
            filter_altitude_m = sample.altitude_agl_m
            continue

        vertical_accel_mps2 = vertical_accel_from_rail(sample, rail, max_accel_mag_mps2)
        if vertical_accel_mps2 is None:
            vertical_accel_mps2 = 0.0
        vertical_accel_mps2 -= bias

        predicted_altitude = (
            filter_altitude_m
            + filter_velocity_mps * dt
            + 0.5 * vertical_accel_mps2 * dt * dt
        )
        predicted_velocity = filter_velocity_mps + vertical_accel_mps2 * dt
        residual = sample.altitude_agl_m - predicted_altitude

        filter_altitude_m = predicted_altitude + alpha * residual
        filter_velocity_mps = predicted_velocity + beta * residual / max(dt, 0.005)
        if not math.isfinite(filter_velocity_mps) or abs(filter_velocity_mps) > 450.0:
            filter_velocity_mps = baro_velocity_mps[index]
            filter_altitude_m = sample.altitude_agl_m
        velocities[index] = filter_velocity_mps
    return velocities


def derived_status(sample: RawSample, velocity_mps: float, coast_start_s: float, liftoff_altitude_m: float) -> str:
    if sample.altitude_agl_m < liftoff_altitude_m:
        return "ground"
    if sample.time_s < coast_start_s:
        return "burn"
    if velocity_mps > 0.0:
        return "coast"
    return "descent"


def make_seeded_samples(samples: list[RawSample],
                        source: str,
                        velocities_mps: list[float],
                        coast_start_s: float,
                        liftoff_altitude_m: float,
                        zero_zenith: bool) -> list[SeededSample]:
    seeded: list[SeededSample] = []
    for sample, velocity_mps in zip(samples, velocities_mps):
        if source == "icm_baro" and not zero_zenith:
            zenith_deg = quaternion_zenith_deg(sample.icm_quat)
        elif source == "lsm_baro" and not zero_zenith:
            zenith_deg = quaternion_zenith_deg(sample.lsm_quat)
        else:
            zenith_deg = 0.0
        seeded.append(
            SeededSample(
                source=source,
                time_s=sample.time_s,
                altitude_m=sample.altitude_agl_m,
                velocity_mps=velocity_mps,
                zenith_deg=zenith_deg,
                flap_command_deg=sample.flap_command_deg,
                status=derived_status(sample, velocity_mps, coast_start_s, liftoff_altitude_m),
            )
        )
    return seeded


def seed_path_for_source(seed_prefix: Path, source: str) -> Path:
    return seed_prefix.with_name(f"{seed_prefix.name}_{source}_input.csv")


def write_seed_csv(path: Path, rows: list[SeededSample]) -> None:
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
        for row in rows:
            writer.writerow(
                {
                    "timestamp": f"{row.time_s:.9f}",
                    "altitude_feet": f"{row.altitude_m * METERS_TO_FEET:.6f}",
                    "has_filtered_state": "1",
                    "flight_status": row.status,
                    "state_time": f"{row.time_s:.9f}",
                    "state_position_x": "0.0",
                    "state_position_y": "0.0",
                    "state_position_z": f"{row.altitude_m:.9f}",
                    "state_velocity_x": "0.0",
                    "state_velocity_y": "0.0",
                    "state_velocity_z": f"{row.velocity_mps:.9f}",
                    "state_zenith_deg": f"{row.zenith_deg:.9f}",
                    "state_apogee_estimate": f"{row.altitude_m:.9f}",
                    "flap_command_deg": f"{row.flap_command_deg:.9f}",
                    "flap_effective_deg": f"{row.flap_command_deg:.9f}",
                }
            )


def parse_replay_stdout(source: str, stdout: str, actual_apogee_m: float) -> list[ReplaySample]:
    lines = stdout.splitlines()
    header_index = next((index for index, line in enumerate(lines) if line.startswith("time_s,")), None)
    if header_index is None:
        raise SystemExit(f"Replay output for {source} did not contain a CSV header")

    rows: list[ReplaySample] = []
    reader = csv.DictReader(lines[header_index:])
    for row in reader:
        time_s = parse_float(row.get("time_s"))
        altitude_m = parse_float(row.get("altitude_m"))
        velocity_mps = parse_float(row.get("velocity_mps"))
        apogee_prediction_m = parse_float(row.get("apogee_prediction_m"))
        if None in (time_s, altitude_m, velocity_mps, apogee_prediction_m):
            continue
        rows.append(
            ReplaySample(
                source=source,
                time_s=time_s,  # type: ignore[arg-type]
                altitude_m=altitude_m,  # type: ignore[arg-type]
                velocity_mps=velocity_mps,  # type: ignore[arg-type]
                apogee_prediction_m=apogee_prediction_m,  # type: ignore[arg-type]
                status=(row.get("status") or "").strip(),
                altitude_agl_m=parse_float(row.get("altitude_agl_m")),
                zenith_deg=parse_float(row.get("zenith_deg")),
                horizontal_velocity_mps=parse_float(row.get("horizontal_velocity_mps")),
                actual_apogee_m=actual_apogee_m,
            )
        )
    return rows


def run_replay(binary: Path,
               seed_csv: Path,
               source: str,
               actual_apogee_m: float,
               dry_mass_kg: float,
               cp_offset_m: float,
               moment_of_inertia_kgm2: float,
               seeded_flap_source: str) -> list[ReplaySample]:
    command = [
        str(binary),
        str(seed_csv),
        "--dry-mass-kg",
        str(dry_mass_kg),
        "--cp-offset-m",
        str(cp_offset_m),
        "--moment-of-inertia-kgm2",
        str(moment_of_inertia_kgm2),
        "--seeded-flap-source",
        seeded_flap_source,
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


def write_replay_csv(path: Path,
                     seeded_runs: dict[str, list[SeededSample]],
                     replay_runs: dict[str, list[ReplaySample]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "source",
        "time_s",
        "seed_altitude_m",
        "seed_velocity_mps",
        "seed_zenith_deg",
        "seed_flap_command_deg",
        "apogee_prediction_m",
        "status",
        "altitude_agl_m",
        "horizontal_velocity_mps",
        "actual_apogee_m",
        "prediction_error_m",
    ]
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for source, replay_rows in replay_runs.items():
            seed_rows = seeded_runs[source]
            for seed, replay in zip(seed_rows, replay_rows):
                writer.writerow(
                    {
                        "source": source,
                        "time_s": f"{replay.time_s:.6f}",
                        "seed_altitude_m": f"{seed.altitude_m:.6f}",
                        "seed_velocity_mps": f"{seed.velocity_mps:.6f}",
                        "seed_zenith_deg": f"{seed.zenith_deg:.6f}",
                        "seed_flap_command_deg": f"{seed.flap_command_deg:.6f}",
                        "apogee_prediction_m": f"{replay.apogee_prediction_m:.6f}",
                        "status": replay.status,
                        "altitude_agl_m": "" if replay.altitude_agl_m is None else f"{replay.altitude_agl_m:.6f}",
                        "horizontal_velocity_mps": "" if replay.horizontal_velocity_mps is None else f"{replay.horizontal_velocity_mps:.6f}",
                        "actual_apogee_m": f"{replay.actual_apogee_m:.6f}",
                        "prediction_error_m": f"{replay.prediction_error_m:.6f}",
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
    output: list[str] = []
    for time_s, value in thin(points):
        if not math.isfinite(time_s) or not math.isfinite(value):
            continue
        x = scale(time_s, x_min, x_max, left, left + width)
        y = scale(value, y_min, y_max, bottom, top)
        output.append(f"{x:.2f},{y:.2f}")
    return " ".join(output)


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
    ticks: list[float] = []
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
    lines: list[str] = [
        f'<text x="{left:.0f}" y="{top - 14:.0f}" class="title">{html.escape(title)}</text>',
        f'<rect x="{left:.0f}" y="{top:.0f}" width="{width:.0f}" height="{height:.0f}" class="panel"/>',
    ]
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

    for _, points, color in series:
        svg_points = polyline(points, x_min, x_max, y_min, y_max, left, top, width, height)
        if svg_points:
            lines.append(
                f'<polyline points="{svg_points}" fill="none" stroke="{color}" '
                f'stroke-width="2.0" stroke-linejoin="round" stroke-linecap="round"/>'
            )
    return "\n".join(lines)


def write_svg(path: Path,
              samples: list[RawSample],
              seeded_runs: dict[str, list[SeededSample]],
              replay_runs: dict[str, list[ReplaySample]],
              actual_apogee_m: float,
              actual_apogee_time_s: float,
              coast_start_s: float,
              seeded_flap_source: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    colors = {
        "baro_trailing": "#111827",
        "icm_baro": "#2563eb",
        "lsm_baro": "#ea580c",
    }
    labels = {
        "baro_trailing": "baro trailing",
        "icm_baro": "ICM+baro",
        "lsm_baro": "LSM+baro",
    }
    x_start = coast_start_s - 2.0
    x_end = actual_apogee_time_s + 1.5

    altitude_points = [
        (sample.time_s, sample.altitude_agl_m * METERS_TO_FEET)
        for sample in samples
        if x_start <= sample.time_s <= x_end
    ]
    altitude_series = [("raw altitude", altitude_points, "#0f172a")]
    error_series: list[tuple[str, list[tuple[float, float]], str]] = []
    velocity_series: list[tuple[str, list[tuple[float, float]], str]] = []
    zenith_series: list[tuple[str, list[tuple[float, float]], str]] = []

    y_alt_values = [value for _, value in altitude_points]
    y_err_values = [0.0]
    y_vel_values = [0.0]
    y_zen_values = [0.0]
    for source, replay_rows in replay_runs.items():
        color = colors[source]
        prediction = [
            (row.time_s, row.apogee_prediction_m * METERS_TO_FEET)
            for row in replay_rows
            if x_start <= row.time_s <= x_end and row.status in {"burn", "coast", "descent"}
        ]
        error = [
            (row.time_s, row.prediction_error_m * METERS_TO_FEET)
            for row in replay_rows
            if x_start <= row.time_s <= x_end and row.status in {"burn", "coast", "descent"}
        ]
        velocity = [
            (row.time_s, row.velocity_mps * MPS_TO_FPS)
            for row in replay_rows
            if x_start <= row.time_s <= x_end
        ]
        zenith = [
            (seed.time_s, seed.zenith_deg)
            for seed in seeded_runs[source]
            if x_start <= seed.time_s <= x_end
        ]
        altitude_series.append((labels[source], prediction, color))
        error_series.append((labels[source], error, color))
        velocity_series.append((labels[source], velocity, color))
        zenith_series.append((labels[source], zenith, color))
        y_alt_values.extend(value for _, value in prediction)
        y_err_values.extend(value for _, value in error)
        y_vel_values.extend(value for _, value in velocity)
        y_zen_values.extend(value for _, value in zenith)

    width = 1280
    height = 1160
    left = 96
    panel_width = 1100
    panel_height = 210
    panel_gap = 70
    top1 = 98
    top2 = top1 + panel_height + panel_gap
    top3 = top2 + panel_height + panel_gap
    top4 = top3 + panel_height + panel_gap
    vertical_lines = [
        ("coast seed", coast_start_s, "#7c3aed"),
        ("raw apogee", actual_apogee_time_s, "#047857"),
    ]

    y_alt_max = max(5200.0, math.ceil((max(y_alt_values) + 250.0) / 250.0) * 250.0)
    y_err_min = math.floor((min(y_err_values) - 250.0) / 250.0) * 250.0
    y_err_max = math.ceil((max(y_err_values) + 250.0) / 250.0) * 250.0
    y_vel_min = math.floor((min(y_vel_values) - 100.0) / 100.0) * 100.0
    y_vel_max = math.ceil((max(y_vel_values) + 100.0) / 100.0) * 100.0
    y_zen_max = max(10.0, math.ceil((max(y_zen_values) + 5.0) / 5.0) * 5.0)

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
        '<text x="48" y="42" class="heading">Fullscale 3 Causal Apogee Reconstruction</text>',
        (
            f'<text x="48" y="64" class="sub">Raw AGL apogee '
            f'{actual_apogee_m * METERS_TO_FEET:.1f} ft at t={actual_apogee_time_s:.3f}s; '
            f"seeded flap source: {html.escape(seeded_flap_source)}.</text>"
        ),
    ]

    legend_x = 725
    legend_y = 38
    for offset, source in enumerate(("baro_trailing", "icm_baro", "lsm_baro")):
        x = legend_x + offset * 150
        parts.append(f'<line x1="{x}" y1="{legend_y}" x2="{x + 30}" y2="{legend_y}" stroke="{colors[source]}" stroke-width="3"/>')
        parts.append(f'<text x="{x + 38}" y="{legend_y + 4}" class="sub">{html.escape(labels[source])}</text>')

    parts.append(draw_panel(
        "Altitude and predicted apogee (ft AGL)",
        x_start,
        x_end,
        0.0,
        y_alt_max,
        left,
        top1,
        panel_width,
        panel_height,
        altitude_series,
        [("actual apogee", actual_apogee_m * METERS_TO_FEET, "#047857")],
        vertical_lines,
    ))
    parts.append(draw_panel(
        "Prediction error vs raw apogee (ft)",
        x_start,
        x_end,
        y_err_min,
        y_err_max,
        left,
        top2,
        panel_width,
        panel_height,
        error_series,
        [("zero error", 0.0, "#047857")],
        vertical_lines,
    ))
    parts.append(draw_panel(
        "Causal seed vertical velocity (ft/s)",
        x_start,
        x_end,
        y_vel_min,
        y_vel_max,
        left,
        top3,
        panel_width,
        panel_height,
        velocity_series,
        [],
        vertical_lines,
    ))
    parts.append(draw_panel(
        "Seed zenith from rail quaternion (deg)",
        x_start,
        x_end,
        0.0,
        y_zen_max,
        left,
        top4,
        panel_width,
        panel_height,
        zenith_series,
        [],
        vertical_lines,
    ))
    parts.append("</svg>")
    path.write_text("\n".join(parts) + "\n", encoding="utf-8")


def nearest_sample(rows: list[ReplaySample], target_time_s: float) -> ReplaySample:
    return min(rows, key=lambda row: abs(row.time_s - target_time_s))


def summarize(rows: list[ReplaySample], start_s: float, end_s: float) -> tuple[float, float, float]:
    values = [
        row.prediction_error_m
        for row in rows
        if start_s <= row.time_s <= end_s and row.status in {"coast", "descent"}
    ]
    if not values:
        return math.nan, math.nan, math.nan
    return sum(abs(value) for value in values) / len(values), min(values), max(values)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-csv", type=Path, default=DEFAULT_INPUT_CSV)
    parser.add_argument("--seed-prefix", type=Path, default=DEFAULT_SEED_PREFIX)
    parser.add_argument("--output-csv", type=Path, default=DEFAULT_OUTPUT_CSV)
    parser.add_argument("--output-svg", type=Path, default=DEFAULT_OUTPUT_SVG)
    parser.add_argument("--replay-binary", type=Path)
    parser.add_argument("--baro-window-s", type=float, default=1.0)
    parser.add_argument("--coast-start-s", type=float, default=331.679138)
    parser.add_argument("--liftoff-altitude-m", type=float, default=40.0)
    parser.add_argument("--ground-bias-end-s", type=float, default=328.0)
    parser.add_argument("--imu-alpha", type=float, default=0.10)
    parser.add_argument("--imu-beta", type=float, default=0.03)
    parser.add_argument("--max-accel-mag-mps2", type=float, default=80.0)
    parser.add_argument("--seeded-flap-source", choices=("zero", "logged-command", "logged-effective"), default="zero")
    parser.add_argument("--zero-zenith", action="store_true")
    parser.add_argument("--dry-mass-kg", type=float, default=HISTORICAL_FULLSCALE_DRY_MASS_KG)
    parser.add_argument("--cp-offset-m", type=float, default=HISTORICAL_FULLSCALE_CP_OFFSET_M)
    parser.add_argument("--moment-of-inertia-kgm2", type=float, default=HISTORICAL_FULLSCALE_MOI_KGM2)
    args = parser.parse_args()

    samples = load_raw_samples(args.input_csv)
    actual_sample = max(samples, key=lambda sample: sample.altitude_agl_m)
    actual_apogee_m = actual_sample.altitude_agl_m
    replay_binary = find_replay_binary(args.replay_binary)

    baro_velocity_mps = local_linear_trailing_velocity(samples, args.baro_window_s)
    velocities = {
        "baro_trailing": baro_velocity_mps,
        "icm_baro": reconstruct_imu_velocity(
            samples,
            "icm",
            baro_velocity_mps,
            args.coast_start_s,
            args.ground_bias_end_s,
            args.imu_alpha,
            args.imu_beta,
            args.max_accel_mag_mps2,
        ),
        "lsm_baro": reconstruct_imu_velocity(
            samples,
            "lsm",
            baro_velocity_mps,
            args.coast_start_s,
            args.ground_bias_end_s,
            args.imu_alpha,
            args.imu_beta,
            args.max_accel_mag_mps2,
        ),
    }

    seeded_runs: dict[str, list[SeededSample]] = {}
    replay_runs: dict[str, list[ReplaySample]] = {}
    for source, source_velocities in velocities.items():
        seeded = make_seeded_samples(
            samples,
            source,
            source_velocities,
            args.coast_start_s,
            args.liftoff_altitude_m,
            args.zero_zenith,
        )
        seed_csv = seed_path_for_source(args.seed_prefix, source)
        write_seed_csv(seed_csv, seeded)
        replay_runs[source] = run_replay(
            replay_binary,
            seed_csv,
            source,
            actual_apogee_m,
            args.dry_mass_kg,
            args.cp_offset_m,
            args.moment_of_inertia_kgm2,
            args.seeded_flap_source,
        )
        seeded_runs[source] = seeded

    write_replay_csv(args.output_csv, seeded_runs, replay_runs)
    write_svg(
        args.output_svg,
        samples,
        seeded_runs,
        replay_runs,
        actual_apogee_m,
        actual_sample.time_s,
        args.coast_start_s,
        args.seeded_flap_source,
    )

    print(f"Input log: {args.input_csv}")
    print(f"Raw altitude apogee: {actual_apogee_m:.3f} m ({actual_apogee_m * METERS_TO_FEET:.1f} ft) at t={actual_sample.time_s:.3f} s")
    print(f"Seeded flap source: {args.seeded_flap_source}")
    for source, rows in replay_runs.items():
        mae_335, low_335, high_335 = summarize(rows, 335.0, actual_sample.time_s)
        mae_338, low_338, high_338 = summarize(rows, 338.0, actual_sample.time_s)
        print(
            f"{source}: mean abs error 335s-apogee={mae_335:.2f} m "
            f"({mae_335 * METERS_TO_FEET:.1f} ft), "
            f"338s-apogee={mae_338:.2f} m ({mae_338 * METERS_TO_FEET:.1f} ft), "
            f"338s range=[{low_338:.2f}, {high_338:.2f}] m"
        )
        for target_time_s in (332.0, 335.0, 338.0, 340.0, 342.0, 344.0):
            row = nearest_sample(rows, target_time_s)
            print(
                f"  t={row.time_s:.2f}s {row.status:7s} "
                f"alt={row.altitude_m:.1f}m vz={row.velocity_mps:.1f}m/s "
                f"apogee={row.apogee_prediction_m:.1f}m err={row.prediction_error_m:+.1f}m"
            )
    print(f"Wrote replay CSV: {args.output_csv}")
    print(f"Wrote plot: {args.output_svg}")


if __name__ == "__main__":
    main()
