#!/usr/bin/env python3
"""
Generate replay validation plots for the historical `fullscale_4.csv` flight.

This script acknowledges that `fullscale_4.csv` is a legacy log whose raw body
frame does not line up with the current hosted replay conventions. Rather than
pretending there is a single trusted transform, it:

1. Uses the logged filtered state as a seeded baseline for the current apogee
   predictor.
2. Applies one or more explicit legacy-frame presets to the raw rails.
3. Runs hosted `acs_replay` in full estimator mode on each transformed CSV.
4. Plots the resulting altitude, apogee prediction, flight-path geometry, Mach,
   and flap motion in a single comparison figure.

The goal is to validate two separate questions:

- Is the current apogee predictor reasonable when seeded from the historical
  filtered state?
- Does the hosted replay recover a physically plausible coast flight path from
  the transformed raw rails?
"""

from __future__ import annotations

import argparse
import csv
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
from replay_legacy_frame_adapter import PRESETS_BY_NAME
from replay_legacy_frame_adapter import Preset
from replay_legacy_frame_adapter import write_transformed_csv


from replaylib.paths import SCRIPTS_DIR
SCRIPT_DIR = SCRIPTS_DIR
ROOT = SCRIPT_DIR.parents[2]

DEFAULT_INPUT_CSV = ROOT / "tools" / "replay" / "data" / "fullscale_4.csv"
DEFAULT_OUTPUT_SVG = ROOT / "tools" / "replay" / "plots" / "fullscale_4_validation.svg"
DEFAULT_OUTPUT_CSV = ROOT / "tools" / "replay" / "data" / "fullscale_4_validation.csv"
DEFAULT_PRESETS = ("best-coast", "lsm-track")
MAX_POLYLINE_POINTS = 1800

FEET_TO_METERS = 0.3048
METERS_TO_FEET = 1.0 / FEET_TO_METERS

REPLAY_EXTRA_FIELDS = (
    "altitude_agl_m",
    "horizontal_velocity_mps",
    "airspeed_rel_mps",
    "mach",
    "zenith_deg",
    "aoa_deg",
    "aoa_abs_deg",
)


@dataclass
class LoggedSample:
    time_s: float
    status: str
    baro_agl_ft: float | None
    state_agl_ft: float | None
    state_apogee_ft: float | None
    state_zenith_deg: float | None
    flap_command_deg: float | None
    flap_effective_deg: float | None
    settling: bool


@dataclass
class ReplaySample:
    time_s: float
    altitude_ft: float
    altitude_agl_ft: float | None
    velocity_fps: float
    apogee_ft: float
    status: str
    horizontal_velocity_fps: float | None
    airspeed_rel_fps: float | None
    mach: float | None
    zenith_deg: float | None
    aoa_deg: float | None
    aoa_abs_deg: float | None


@dataclass
class ReplayRun:
    label: str
    mode: str
    color: str
    samples: list[ReplaySample]


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
    if value is None:
        return False
    lowered = value.strip().lower()
    return lowered in {"1", "true", "yes"}


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def sanitize_filename(text: str) -> str:
    out: list[str] = []
    for ch in text:
        if ch.isalnum():
            out.append(ch)
        elif ch in ("-", "_"):
            out.append(ch)
        else:
            out.append("_")
    return ("".join(out).strip("_")) or "output"


def phase_runs(samples: list[LoggedSample]) -> list[tuple[str, float, float]]:
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


def settling_runs(samples: list[LoggedSample]) -> list[tuple[float, float]]:
    runs: list[tuple[float, float]] = []
    active_start: float | None = None
    previous_time = samples[0].time_s if samples else 0.0
    for sample in samples:
        if sample.settling and active_start is None:
            active_start = sample.time_s
        elif not sample.settling and active_start is not None:
            runs.append((active_start, sample.time_s))
            active_start = None
        previous_time = sample.time_s
    if active_start is not None:
        runs.append((active_start, previous_time))
    return runs


def scale(value: float, src_lo: float, src_hi: float, dst_lo: float, dst_hi: float) -> float:
    if src_hi <= src_lo:
        return 0.5 * (dst_lo + dst_hi)
    ratio = (value - src_lo) / (src_hi - src_lo)
    return dst_lo + ratio * (dst_hi - dst_lo)


def thin_points(points: list[tuple[float, float]], limit: int = MAX_POLYLINE_POINTS) -> list[tuple[float, float]]:
    if len(points) <= limit:
        return points
    if limit <= 2:
        return [points[0], points[-1]]

    thinned: list[tuple[float, float]] = []
    previous_index = -1
    scale_index = (len(points) - 1) / float(limit - 1)
    for output_index in range(limit):
        source_index = int(round(output_index * scale_index))
        if source_index != previous_index:
            thinned.append(points[source_index])
            previous_index = source_index
    if thinned[-1] != points[-1]:
        thinned.append(points[-1])
    return thinned


def build_polyline(
    points: list[tuple[float, float | None]],
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    left: float,
    top: float,
    width: float,
    height: float,
) -> str:
    filtered_points: list[tuple[float, float]] = []
    for time_s, value in points:
        if value is None or not math.isfinite(value) or not math.isfinite(time_s):
            continue
        if time_s < x_min or time_s > x_max:
            continue
        filtered_points.append((time_s, value))

    svg_points: list[str] = []
    bottom = top + height
    for time_s, value in thin_points(filtered_points):
        x = scale(time_s, x_min, x_max, left, left + width)
        y = scale(value, y_min, y_max, bottom, top)
        svg_points.append(f"{x:.2f},{y:.2f}")
    return " ".join(svg_points)


def draw_panel(
    title: str,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    left: float,
    top: float,
    width: float,
    height: float,
    phase_source: list[LoggedSample],
    series: list[tuple[str, list[tuple[float, float | None]], str, str]],
    reference_lines: list[tuple[str, float, str]],
    vertical_markers: list[tuple[str, float, str]],
    x_spans: list[tuple[str, float, float, str, float]],
) -> str:
    phase_colors = {
        "ground": "#f3f4f6",
        "burn": "#fee2e2",
        "coast": "#dbeafe",
        "overshoot": "#ede9fe",
        "descent": "#dcfce7",
    }
    elements: list[str] = []
    bottom = top + height

    for status, run_start, run_end in phase_runs(phase_source):
        x0 = scale(run_start, x_min, x_max, left, left + width)
        x1 = scale(run_end, x_min, x_max, left, left + width)
        elements.append(
            f'<rect x="{x0:.2f}" y="{top:.2f}" width="{max(1.0, x1 - x0):.2f}" '
            f'height="{height:.2f}" fill="{phase_colors.get(status, "#f3f4f6")}" opacity="0.30"/>'
        )

    for label, span_start, span_end, color, opacity in x_spans:
        x0 = scale(span_start, x_min, x_max, left, left + width)
        x1 = scale(span_end, x_min, x_max, left, left + width)
        elements.append(
            f'<rect x="{x0:.2f}" y="{top:.2f}" width="{max(1.0, x1 - x0):.2f}" '
            f'height="{height:.2f}" fill="{color}" opacity="{opacity:.2f}"/>'
        )
        elements.append(
            f'<text x="{x0 + 6:.2f}" y="{top + 18:.2f}" font-size="11" fill="{color}">{label}</text>'
        )

    for index in range(6):
        grid_y = top + index * height / 5.0
        grid_x = left + index * width / 5.0
        y_value = y_max - index * (y_max - y_min) / 5.0
        x_value = x_min + index * (x_max - x_min) / 5.0
        elements.append(
            f'<line x1="{left:.2f}" y1="{grid_y:.2f}" x2="{left + width:.2f}" y2="{grid_y:.2f}" '
            'stroke="#d1d5db" stroke-width="1"/>'
        )
        elements.append(
            f'<line x1="{grid_x:.2f}" y1="{top:.2f}" x2="{grid_x:.2f}" y2="{bottom:.2f}" '
            'stroke="#e5e7eb" stroke-width="1"/>'
        )
        elements.append(
            f'<text x="{left - 10:.2f}" y="{grid_y + 4:.2f}" text-anchor="end" '
            'font-size="12" fill="#111827">'
            f"{y_value:.1f}</text>"
        )
        elements.append(
            f'<text x="{grid_x:.2f}" y="{bottom + 22:.2f}" text-anchor="middle" '
            'font-size="12" fill="#111827">'
            f"{x_value:.2f}</text>"
        )

    for label, value, color in reference_lines:
        if not math.isfinite(value):
            continue
        y = scale(value, y_min, y_max, bottom, top)
        elements.append(
            f'<line x1="{left:.2f}" y1="{y:.2f}" x2="{left + width:.2f}" y2="{y:.2f}" '
            f'stroke="{color}" stroke-width="1.5" stroke-dasharray="6 5"/>'
        )
        elements.append(
            f'<text x="{left + width - 6:.2f}" y="{y - 6:.2f}" text-anchor="end" '
            f'font-size="12" fill="{color}">{label}</text>'
        )

    for label, value, color in vertical_markers:
        if value < x_min or value > x_max:
            continue
        x = scale(value, x_min, x_max, left, left + width)
        elements.append(
            f'<line x1="{x:.2f}" y1="{top:.2f}" x2="{x:.2f}" y2="{bottom:.2f}" '
            f'stroke="{color}" stroke-width="1.4" stroke-dasharray="5 4"/>'
        )
        elements.append(
            f'<text x="{x + 4:.2f}" y="{top + 16:.2f}" font-size="11" fill="{color}">{label}</text>'
        )

    legend_x = left + 14.0
    legend_y = top + 18.0
    for index, (label, points, color, dash) in enumerate(series):
        polyline = build_polyline(points, x_min, x_max, y_min, y_max, left, top, width, height)
        if polyline:
            dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
            elements.append(
                f'<polyline fill="none" stroke="{color}" stroke-width="1.8"{dash_attr} points="{polyline}"/>'
            )
        line_y = legend_y + 18.0 * index
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        elements.append(
            f'<line x1="{legend_x:.2f}" y1="{line_y:.2f}" x2="{legend_x + 30:.2f}" y2="{line_y:.2f}" '
            f'stroke="{color}" stroke-width="2.4"{dash_attr}/>'
        )
        elements.append(
            f'<text x="{legend_x + 38:.2f}" y="{line_y + 4:.2f}" font-size="12" fill="#111827">{label}</text>'
        )

    elements.append(
        f'<rect x="{left:.2f}" y="{top:.2f}" width="{width:.2f}" height="{height:.2f}" '
        'fill="none" stroke="#111827" stroke-width="1.5"/>'
    )
    elements.append(
        f'<text x="{left:.2f}" y="{top - 16:.2f}" font-size="18" font-weight="700" fill="#111827">{title}</text>'
    )
    return "\n".join(elements)


def load_logged_samples(path: Path) -> list[LoggedSample]:
    rows: list[dict[str, str]] = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(line for line in handle if not line.startswith("#"))
        for raw in reader:
            rows.append(raw)
    if not rows:
        raise SystemExit(f"No usable rows found in {path}")

    pad_alt_ft: float | None = None
    for raw in rows:
        altitude_ft = parse_float(raw.get("sensor_altitude_feet"))
        if altitude_ft is not None and altitude_ft != 0.0:
            pad_alt_ft = altitude_ft
            break
    if pad_alt_ft is None:
        raise SystemExit(f"Could not determine pad altitude from {path}")

    samples: list[LoggedSample] = []
    for raw in rows:
        time_s = parse_float(raw.get("sensor_timestamp"))
        if time_s is None:
            continue
        absolute_alt_ft = parse_float(raw.get("sensor_altitude_feet"))
        baro_agl_ft = None if absolute_alt_ft is None else max(0.0, absolute_alt_ft - pad_alt_ft)
        samples.append(
            LoggedSample(
                time_s=time_s,
                status=(raw.get("flight_status") or "").strip().lower(),
                baro_agl_ft=baro_agl_ft,
                state_agl_ft=parse_float(raw.get("state_altitude_agl_feet")),
                state_apogee_ft=parse_float(raw.get("state_apogee_estimate_feet")),
                state_zenith_deg=parse_float(raw.get("state_zenith_deg")),
                flap_command_deg=parse_float(raw.get("sensor_flap_command_deg")),
                flap_effective_deg=parse_float(raw.get("sensor_flap_effective_deg")),
                settling=parse_bool(raw.get("sensor_actuation_is_settling")),
            )
        )
    return samples


def _parse_replay_stdout(stdout: str) -> list[ReplaySample]:
    lines = stdout.splitlines()
    try:
        header_index = next(index for index, line in enumerate(lines) if line.startswith("time_s,"))
    except StopIteration as exc:
        raise SystemExit("acs_replay did not emit replay CSV output") from exc

    csv_lines = [lines[header_index]]
    for line in lines[header_index + 1 :]:
        if line.startswith("Samples processed:"):
            break
        if line.strip():
            csv_lines.append(line)
    reader = csv.DictReader(csv_lines)
    samples: list[ReplaySample] = []
    for raw in reader:
        time_s = parse_float(raw.get("time_s"))
        altitude_m = parse_float(raw.get("altitude_m"))
        velocity_mps = parse_float(raw.get("velocity_mps"))
        apogee_m = parse_float(raw.get("apogee_prediction_m"))
        if time_s is None or altitude_m is None or velocity_mps is None or apogee_m is None:
            continue
        altitude_agl_m = parse_float(raw.get("altitude_agl_m"))
        horizontal_velocity_mps = parse_float(raw.get("horizontal_velocity_mps"))
        airspeed_rel_mps = parse_float(raw.get("airspeed_rel_mps"))
        samples.append(
            ReplaySample(
                time_s=time_s,
                altitude_ft=altitude_m * METERS_TO_FEET,
                altitude_agl_ft=None if altitude_agl_m is None else altitude_agl_m * METERS_TO_FEET,
                velocity_fps=velocity_mps * METERS_TO_FEET,
                apogee_ft=apogee_m * METERS_TO_FEET,
                status=(raw.get("status") or "").strip().lower(),
                horizontal_velocity_fps=None
                if horizontal_velocity_mps is None
                else horizontal_velocity_mps * METERS_TO_FEET,
                airspeed_rel_fps=None if airspeed_rel_mps is None else airspeed_rel_mps * METERS_TO_FEET,
                mach=parse_float(raw.get("mach")),
                zenith_deg=parse_float(raw.get("zenith_deg")),
                aoa_deg=parse_float(raw.get("aoa_deg")),
                aoa_abs_deg=parse_float(raw.get("aoa_abs_deg")),
            )
        )
    if not samples:
        raise SystemExit("acs_replay emitted no usable sample rows")
    return samples


def run_replay(replay_binary: Path, input_csv: Path, extra_args: list[str]) -> list[ReplaySample]:
    command = [
        str(replay_binary),
        str(input_csv),
        f"--dry-mass-kg={HISTORICAL_FULLSCALE_DRY_MASS_KG}",
        f"--cp-offset-m={HISTORICAL_FULLSCALE_CP_OFFSET_M}",
        f"--moment-of-inertia-kgm2={HISTORICAL_FULLSCALE_MOI_KGM2}",
        f"--include-raw={','.join(REPLAY_EXTRA_FIELDS)}",
        *extra_args,
    ]
    result = subprocess.run(
        command,
        cwd=str(ROOT),
        text=True,
        capture_output=True,
        check=False,
    )
    if result.returncode != 0:
        raise SystemExit(result.stderr.strip() or result.stdout.strip() or "acs_replay failed")
    return _parse_replay_stdout(result.stdout)


def run_seeded_variant(
    replay_binary: Path,
    input_csv: Path,
    *,
    label: str,
    mode: str,
    color: str,
    seeded_flap_source: str | None,
) -> ReplayRun:
    extra_args: list[str] = []
    if seeded_flap_source is not None:
        extra_args.append(f"--seeded-flap-source={seeded_flap_source}")
    return ReplayRun(
        label=label,
        mode=mode,
        color=color,
        samples=run_replay(replay_binary, input_csv, extra_args),
    )


def run_hosted_preset(replay_binary: Path, input_csv: Path, preset: Preset, color: str) -> ReplayRun:
    with tempfile.NamedTemporaryFile(
        suffix=f"_{sanitize_filename(preset.name)}.csv",
        prefix=f"{sanitize_filename(input_csv.stem)}_validation_",
        dir="/tmp",
        delete=False,
    ) as handle:
        temp_csv = Path(handle.name)
    try:
        write_transformed_csv(input_csv, preset, temp_csv)
        samples = run_replay(replay_binary, temp_csv, ["--ignore-logged-state", "--rebuild-main-quaternion"])
    finally:
        try:
            os.unlink(temp_csv)
        except OSError:
            pass
    return ReplayRun(label=f"Hosted replay ({preset.name})", mode=preset.name, color=color, samples=samples)


def compute_baro_peak(logged_samples: list[LoggedSample]) -> tuple[float, float]:
    peak_sample = max(
        (sample for sample in logged_samples if sample.baro_agl_ft is not None and math.isfinite(sample.baro_agl_ft)),
        key=lambda sample: sample.baro_agl_ft if sample.baro_agl_ft is not None else -1.0,
    )
    return peak_sample.time_s, peak_sample.baro_agl_ft if peak_sample.baro_agl_ft is not None else 0.0


def first_time_where(samples: list[LoggedSample], predicate) -> float | None:
    for sample in samples:
        if predicate(sample):
            return sample.time_s
    return None


def replay_series(samples: list[ReplaySample], getter) -> list[tuple[float, float | None]]:
    return [(sample.time_s, getter(sample)) for sample in samples]


def logged_series(samples: list[LoggedSample], getter) -> list[tuple[float, float | None]]:
    return [(sample.time_s, getter(sample)) for sample in samples]


def write_summary_csv(
    output_csv: Path,
    logged_samples: list[LoggedSample],
    seeded_runs: list[ReplayRun],
    hosted_runs: list[ReplayRun],
) -> None:
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    time_keys = [round(sample.time_s, 4) for sample in logged_samples]
    seeded_by_mode = {
        run.mode: {round(sample.time_s, 4): sample for sample in run.samples}
        for run in seeded_runs
    }
    hosted_by_mode = {
        run.mode: {round(sample.time_s, 4): sample for sample in run.samples}
        for run in hosted_runs
    }

    headers = [
        "time_s",
        "status",
        "baro_agl_ft",
        "logged_state_agl_ft",
        "logged_state_apogee_ft",
        "logged_state_zenith_deg",
        "flap_command_deg",
        "flap_effective_deg",
        "settling",
    ]
    for run in seeded_runs:
        headers.append(f"{run.mode.replace('-', '_')}_apogee_ft")
    for run in hosted_runs:
        prefix = run.mode.replace("-", "_")
        headers.extend(
            [
                f"{prefix}_altitude_agl_ft",
                f"{prefix}_apogee_ft",
                f"{prefix}_zenith_deg",
                f"{prefix}_aoa_abs_deg",
                f"{prefix}_mach",
            ]
        )

    with output_csv.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(headers)
        for sample, time_key in zip(logged_samples, time_keys):
            row: list[object] = [
                sample.time_s,
                sample.status,
                sample.baro_agl_ft,
                sample.state_agl_ft,
                sample.state_apogee_ft,
                sample.state_zenith_deg,
                sample.flap_command_deg,
                sample.flap_effective_deg,
                int(sample.settling),
            ]
            for run in seeded_runs:
                seeded = seeded_by_mode[run.mode].get(time_key)
                row.append("" if seeded is None else seeded.apogee_ft)
            for run in hosted_runs:
                replay_sample = hosted_by_mode[run.mode].get(time_key)
                row.extend(
                    [
                        "" if replay_sample is None else replay_sample.altitude_agl_ft,
                        "" if replay_sample is None else replay_sample.apogee_ft,
                        "" if replay_sample is None else replay_sample.zenith_deg,
                        "" if replay_sample is None else replay_sample.aoa_abs_deg,
                        "" if replay_sample is None else replay_sample.mach,
                    ]
                )
            writer.writerow(row)


def write_svg(
    output_svg: Path,
    logged_samples: list[LoggedSample],
    seeded_runs: list[ReplayRun],
    hosted_runs: list[ReplayRun],
) -> None:
    output_svg.parent.mkdir(parents=True, exist_ok=True)

    width = 1560
    height = 1660
    margin_left = 110.0
    panel_width = 1320.0
    panel_height = 185.0
    panel_gap = 62.0

    baro_apogee_time_s, baro_apogee_ft = compute_baro_peak(logged_samples)
    first_coast_s = first_time_where(logged_samples, lambda sample: sample.status in {"coast", "overshoot"})
    first_flap_command_s = first_time_where(
        logged_samples, lambda sample: (sample.flap_command_deg or 0.0) > 0.5
    )
    first_flap_effective_s = first_time_where(
        logged_samples, lambda sample: (sample.flap_effective_deg or 0.0) > 0.5
    )
    zoom_start = logged_samples[0].time_s if first_coast_s is None else max(logged_samples[0].time_s, first_coast_s - 2.0)
    zoom_end = min(logged_samples[-1].time_s, baro_apogee_time_s + 3.0)

    markers = [
        ("coast", first_coast_s if first_coast_s is not None else zoom_start, "#2563eb"),
        ("cmd", first_flap_command_s if first_flap_command_s is not None else zoom_start, "#b45309"),
        ("eff", first_flap_effective_s if first_flap_effective_s is not None else zoom_start, "#047857"),
        ("baro apogee", baro_apogee_time_s, "#dc2626"),
    ]
    settling_spans = [("settling", start, end, "#fb923c", 0.14) for start, end in settling_runs(logged_samples)]

    overview_alt_points = logged_series(logged_samples, lambda sample: sample.baro_agl_ft)
    overview_state_alt_points = logged_series(logged_samples, lambda sample: sample.state_agl_ft)
    overview_state_apogee_points = logged_series(logged_samples, lambda sample: sample.state_apogee_ft)

    altitude_series = [
        ("Raw baro AGL", overview_alt_points, "#111827", ""),
        ("Logged state AGL", overview_state_alt_points, "#4b5563", ""),
        ("Logged state apogee", overview_state_apogee_points, "#6b7280", "8 5"),
    ]
    for run in seeded_runs:
        altitude_series.append((run.label, replay_series(run.samples, lambda sample: sample.apogee_ft), run.color, "8 4"))
    for run in hosted_runs:
        altitude_series.append(
            (
                f"{run.label} altitude",
                replay_series(run.samples, lambda sample: sample.altitude_agl_ft),
                run.color,
                "",
            )
        )
        altitude_series.append(
            (
                f"{run.label} apogee",
                replay_series(run.samples, lambda sample: sample.apogee_ft),
                run.color,
                "7 4",
            )
        )

    error_series = [
        (
            "Logged state error",
            logged_series(
                logged_samples,
                lambda sample: None
                if sample.state_apogee_ft is None
                else sample.state_apogee_ft - baro_apogee_ft,
            ),
            "#6b7280",
            "",
        ),
    ]
    for run in seeded_runs:
        error_series.append(
            (
                f"{run.label} error",
                replay_series(run.samples, lambda sample: sample.apogee_ft - baro_apogee_ft),
                run.color,
                "",
            )
        )
    for run in hosted_runs:
        error_series.append(
            (
                f"{run.label} error",
                replay_series(run.samples, lambda sample: sample.apogee_ft - baro_apogee_ft),
                run.color,
                "",
            )
        )

    zenith_series = [
        ("Logged state zenith", logged_series(logged_samples, lambda sample: sample.state_zenith_deg), "#111827", ""),
    ]
    for run in hosted_runs:
        zenith_series.append(
            (
                f"{run.label} zenith",
                replay_series(run.samples, lambda sample: sample.zenith_deg),
                run.color,
                "",
            )
        )
        zenith_series.append(
            (
                f"{run.label} |AoA|",
                replay_series(run.samples, lambda sample: sample.aoa_abs_deg),
                run.color,
                "7 4",
            )
        )

    mach_series = []
    for run in hosted_runs:
        mach_series.append(
            (
                f"{run.label} Mach",
                replay_series(run.samples, lambda sample: sample.mach),
                run.color,
                "",
            )
        )

    flap_series = [
        ("Flap command", logged_series(logged_samples, lambda sample: sample.flap_command_deg), "#b45309", ""),
        ("Flap effective", logged_series(logged_samples, lambda sample: sample.flap_effective_deg), "#047857", ""),
    ]

    max_altitude_ft = max(
        value
        for _, value in overview_alt_points
        if value is not None and math.isfinite(value)
    )
    for run in seeded_runs:
        for _, value in replay_series(run.samples, lambda sample: sample.apogee_ft):
            if value is not None and math.isfinite(value):
                max_altitude_ft = max(max_altitude_ft, value)
    for run in hosted_runs:
        for _, value in replay_series(run.samples, lambda sample: sample.apogee_ft):
            if value is not None and math.isfinite(value):
                max_altitude_ft = max(max_altitude_ft, value)
    for _, value in overview_state_apogee_points:
        if value is not None and math.isfinite(value):
            max_altitude_ft = max(max_altitude_ft, value)

    error_values = [
        value
        for _, value in sum((series[1] for series in error_series), [])
        if value is not None and math.isfinite(value)
    ]
    error_extent = max(50.0, max(abs(value) for value in error_values)) if error_values else 50.0

    zenith_values = [
        value
        for _, value in sum((series[1] for series in zenith_series), [])
        if value is not None and math.isfinite(value)
    ]
    zenith_max = max(20.0, max(zenith_values)) if zenith_values else 20.0

    mach_values = [
        value
        for _, value in sum((series[1] for series in mach_series), [])
        if value is not None and math.isfinite(value)
    ]
    mach_max = max(0.8, max(mach_values) * 1.1) if mach_values else 0.8

    flap_max = max(
        10.0,
        max(
            (
                value
                for _, value in sum((series[1] for series in flap_series), [])
                if value is not None and math.isfinite(value)
            ),
            default=10.0,
        )
        * 1.15,
    )

    subtitle_lines = [
        f"Baro apogee: {baro_apogee_ft:.1f} ft at t={baro_apogee_time_s:.2f}s",
        f"Zoom window: {zoom_start:.2f}s to {zoom_end:.2f}s | coast at {first_coast_s:.2f}s"
        if first_coast_s is not None
        else f"Zoom window: {zoom_start:.2f}s to {zoom_end:.2f}s",
    ]
    for run in seeded_runs:
        subtitle_lines.append(f"{run.label} peak: {max(sample.apogee_ft for sample in run.samples):.1f} ft")
    for run in hosted_runs:
        peak_apogee_ft = max(sample.apogee_ft for sample in run.samples)
        first_coast_apogee_ft = next(
            (sample.apogee_ft for sample in run.samples if sample.status == "coast"),
            float("nan"),
        )
        max_mach = max((sample.mach for sample in run.samples if sample.mach is not None), default=float("nan"))
        max_aoa = max(
            (sample.aoa_abs_deg for sample in run.samples if sample.aoa_abs_deg is not None),
            default=float("nan"),
        )
        subtitle_lines.append(
            f"{run.label}: first coast apogee {first_coast_apogee_ft:.1f} ft | peak {peak_apogee_ft:.1f} ft | "
            f"max Mach {max_mach:.2f} | max |AoA| {max_aoa:.1f} deg"
        )

    elements = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        f'<text x="{width / 2:.2f}" y="42" text-anchor="middle" font-family="monospace" '
        'font-size="28" font-weight="700" fill="#111827">fullscale_4 replay validation</text>',
    ]
    for index, line in enumerate(subtitle_lines):
        elements.append(
            f'<text x="{width / 2:.2f}" y="{72 + index * 18:.2f}" text-anchor="middle" '
            'font-family="monospace" font-size="14" fill="#374151">'
            f"{line}</text>"
        )

    panel_top = 175.0
    panel_specs = [
        (
            "Altitude and apogee overview (ft AGL)",
            logged_samples[0].time_s,
            logged_samples[-1].time_s,
            0.0,
            math.ceil((max_altitude_ft + 100.0) / 100.0) * 100.0,
            altitude_series,
            [("Baro apogee", baro_apogee_ft, "#dc2626")],
            [],
        ),
        (
            "Coast zoom: altitude and apogee (ft AGL)",
            zoom_start,
            zoom_end,
            max(0.0, baro_apogee_ft * -0.02),
            math.ceil((max_altitude_ft + 50.0) / 50.0) * 50.0,
            altitude_series,
            [("Baro apogee", baro_apogee_ft, "#dc2626")],
            [],
        ),
        (
            "Coast zoom: apogee prediction error vs actual (ft)",
            zoom_start,
            zoom_end,
            -error_extent * 1.1,
            error_extent * 1.1,
            error_series,
            [("Zero error", 0.0, "#111827")],
            [],
        ),
        (
            "Coast zoom: zenith and |AoA| (deg)",
            zoom_start,
            zoom_end,
            0.0,
            math.ceil((zenith_max + 2.0) / 2.0) * 2.0,
            zenith_series,
            [("CFD AoA limit", 10.0, "#dc2626")],
            [],
        ),
        (
            "Coast zoom: flap command/effective (deg)",
            zoom_start,
            zoom_end,
            0.0,
            math.ceil((flap_max + 2.0) / 2.0) * 2.0,
            flap_series,
            [],
            settling_spans,
        ),
        (
            "Coast zoom: Mach",
            zoom_start,
            zoom_end,
            0.0,
            max(0.8, math.ceil(mach_max * 10.0) / 10.0),
            mach_series,
            [("CFD Mach limit", 0.7, "#dc2626")],
            [],
        ),
    ]

    for title, x_min, x_max, y_min, y_max, series, refs, spans in panel_specs:
        elements.append(
            draw_panel(
                title=title,
                x_min=x_min,
                x_max=x_max,
                y_min=y_min,
                y_max=y_max,
                left=margin_left,
                top=panel_top,
                width=panel_width,
                height=panel_height,
                phase_source=logged_samples,
                series=series,
                reference_lines=refs,
                vertical_markers=markers,
                x_spans=spans,
            )
        )
        panel_top += panel_height + panel_gap

    elements.append("</svg>")
    output_svg.write_text("\n".join(elements))


def write_clean_svg(
    output_svg: Path,
    logged_samples: list[LoggedSample],
    seeded_run: ReplayRun,
) -> None:
    output_svg.parent.mkdir(parents=True, exist_ok=True)

    width = 1280
    height = 720
    margin_left = 96.0
    panel_width = 1100.0
    panel_height = 215.0
    panel_gap = 78.0

    baro_apogee_time_s, baro_apogee_ft = compute_baro_peak(logged_samples)
    first_coast_s = first_time_where(logged_samples, lambda sample: sample.status in {"coast", "overshoot"})
    zoom_start = logged_samples[0].time_s if first_coast_s is None else max(logged_samples[0].time_s, first_coast_s - 2.0)
    zoom_end = min(logged_samples[-1].time_s, baro_apogee_time_s + 0.5)

    baro_points = logged_series(logged_samples, lambda sample: sample.baro_agl_ft)
    state_altitude_points = logged_series(logged_samples, lambda sample: sample.state_agl_ft)
    predictor_points = replay_series(seeded_run.samples, lambda sample: sample.apogee_ft)
    predictor_error_points = replay_series(seeded_run.samples, lambda sample: sample.apogee_ft - baro_apogee_ft)

    altitude_series = [
        ("Baro AGL", baro_points, "#111827", ""),
        ("Logged state AGL", state_altitude_points, "#64748b", ""),
        ("Seeded predictor", predictor_points, "#059669", "7 4"),
    ]
    error_series = [
        ("Seeded predictor error", predictor_error_points, "#059669", ""),
    ]

    y_alt_values = [
        value
        for _, value in baro_points + state_altitude_points + predictor_points
        if value is not None and math.isfinite(value)
    ]
    y_alt_max = math.ceil((max(y_alt_values, default=baro_apogee_ft) + 100.0) / 100.0) * 100.0
    error_values = [
        value
        for time_s, value in predictor_error_points
        if value is not None and math.isfinite(value) and zoom_start <= time_s <= zoom_end
    ]
    error_extent = max(25.0, max((abs(value) for value in error_values), default=25.0))

    predictor_peak_ft = max(sample.apogee_ft for sample in seeded_run.samples)
    predictor_peak_error_ft = predictor_peak_ft - baro_apogee_ft
    vertical_markers = [
        ("coast", first_coast_s if first_coast_s is not None else zoom_start, "#2563eb"),
        ("baro apogee", baro_apogee_time_s, "#dc2626"),
    ]

    elements = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        f'<text x="{width / 2:.2f}" y="42" text-anchor="middle" font-family="monospace" '
        'font-size="26" font-weight="700" fill="#111827">fullscale_4 current predictor validation</text>',
        f'<text x="{width / 2:.2f}" y="72" text-anchor="middle" font-family="monospace" '
        f'font-size="14" fill="#374151">Baro apogee: {baro_apogee_ft:.1f} ft at t={baro_apogee_time_s:.2f}s | '
        f"Seeded predictor peak: {predictor_peak_ft:.1f} ft ({predictor_peak_error_ft:+.1f} ft)</text>",
        f'<text x="{width / 2:.2f}" y="94" text-anchor="middle" font-family="monospace" '
        f'font-size="13" fill="#64748b">Clean view: measured baro/state data plus the selected seeded predictor only.</text>',
    ]

    panel_top = 138.0
    elements.append(
        draw_panel(
            title="Coast altitude and predicted apogee (ft AGL)",
            x_min=zoom_start,
            x_max=zoom_end,
            y_min=0.0,
            y_max=y_alt_max,
            left=margin_left,
            top=panel_top,
            width=panel_width,
            height=panel_height,
            phase_source=logged_samples,
            series=altitude_series,
            reference_lines=[("Baro apogee", baro_apogee_ft, "#dc2626")],
            vertical_markers=vertical_markers,
            x_spans=[],
        )
    )
    panel_top += panel_height + panel_gap
    elements.append(
        draw_panel(
            title="Seeded predictor error vs baro apogee (ft)",
            x_min=zoom_start,
            x_max=zoom_end,
            y_min=-1.1 * error_extent,
            y_max=1.1 * error_extent,
            left=margin_left,
            top=panel_top,
            width=panel_width,
            height=panel_height,
            phase_source=logged_samples,
            series=error_series,
            reference_lines=[("Zero error", 0.0, "#111827")],
            vertical_markers=vertical_markers,
            x_spans=[],
        )
    )

    elements.append("</svg>")
    output_svg.write_text("\n".join(elements) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_csv", nargs="?", type=Path, default=DEFAULT_INPUT_CSV)
    parser.add_argument("--replay-binary", type=Path, default=None)
    parser.add_argument(
        "--presets",
        default=",".join(DEFAULT_PRESETS),
        help="Comma-separated legacy transform presets to compare.",
    )
    parser.add_argument("--output-svg", type=Path, default=DEFAULT_OUTPUT_SVG)
    parser.add_argument("--output-csv", type=Path, default=DEFAULT_OUTPUT_CSV)
    parser.add_argument(
        "--clean",
        action="store_true",
        help="Write a compact current-code figure with only logged data and the selected seeded predictor.",
    )
    parser.add_argument(
        "--clean-seeded-flap-source",
        choices=("none", "logged-effective", "logged-command"),
        default="none",
        help="Flap source for the clean seeded predictor. Default keeps the historical state seed unmodified.",
    )
    args = parser.parse_args()

    preset_names = [] if args.clean else [name.strip() for name in args.presets.split(",") if name.strip()]
    if not args.clean:
        if not preset_names:
            raise SystemExit("At least one preset must be selected.")
        unknown_presets = [name for name in preset_names if name not in PRESETS_BY_NAME]
        if unknown_presets:
            raise SystemExit(f"Unknown preset(s): {', '.join(sorted(unknown_presets))}")

    replay_binary = find_replay_binary(args.replay_binary)
    logged_samples = load_logged_samples(args.input_csv)
    if args.clean:
        seeded_flap_source = None if args.clean_seeded_flap_source == "none" else args.clean_seeded_flap_source
        seeded_runs = [
            run_seeded_variant(
                replay_binary,
                args.input_csv,
                label="Seeded predictor",
                mode="seeded-no-flap" if seeded_flap_source is None else f"seeded-{seeded_flap_source}",
                color="#059669",
                seeded_flap_source=seeded_flap_source,
            )
        ]
    else:
        seeded_runs = [
            run_seeded_variant(
                replay_binary,
                args.input_csv,
                label="Seeded predictor (no flap)",
                mode="seeded-no-flap",
                color="#d97706",
                seeded_flap_source=None,
            ),
            run_seeded_variant(
                replay_binary,
                args.input_csv,
                label="Seeded predictor (+logged flap)",
                mode="seeded-logged-effective",
                color="#059669",
                seeded_flap_source="logged-effective",
            ),
        ]

    hosted_colors = ["#2563eb", "#7c3aed", "#0891b2", "#dc2626"]
    hosted_runs: list[ReplayRun] = []
    for index, preset_name in enumerate(preset_names):
        hosted_runs.append(
            run_hosted_preset(
                replay_binary,
                args.input_csv,
                PRESETS_BY_NAME[preset_name],
                hosted_colors[index % len(hosted_colors)],
            )
        )

    write_summary_csv(args.output_csv, logged_samples, seeded_runs, hosted_runs)
    if args.clean:
        write_clean_svg(args.output_svg, logged_samples, seeded_runs[0])
    else:
        write_svg(args.output_svg, logged_samples, seeded_runs, hosted_runs)

    baro_apogee_time_s, baro_apogee_ft = compute_baro_peak(logged_samples)
    print(f"Input CSV: {args.input_csv}")
    print(f"Replay binary: {replay_binary}")
    print(f"Baro apogee: {baro_apogee_ft:.2f} ft at t={baro_apogee_time_s:.3f}s")
    for run in seeded_runs:
        print(f"{run.label} peak apogee: {max(sample.apogee_ft for sample in run.samples):.2f} ft")
    for run in hosted_runs:
        first_coast_apogee_ft = next(
            (sample.apogee_ft for sample in run.samples if sample.status == "coast"),
            float("nan"),
        )
        print(
            f"{run.label}: first coast apogee {first_coast_apogee_ft:.2f} ft | "
            f"peak {max(sample.apogee_ft for sample in run.samples):.2f} ft"
        )
    print(f"Wrote {args.output_svg}")
    print(f"Wrote {args.output_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
