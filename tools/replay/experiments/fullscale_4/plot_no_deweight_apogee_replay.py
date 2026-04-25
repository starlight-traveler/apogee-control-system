#!/usr/bin/env python3
"""
Run the hosted replay flight computer on a logged CSV with baro deweighting
disabled, then compare the resulting apogee trace against the logged state
estimate and raw baro AGL.

This wrapper does three important things before invoking `acs_replay`:
1. Drops logged `state_*` columns so replay is forced through FlightComputer.
2. Rotates the logged IMU accel/gyro rails so body +X is the axial axis the
   estimator expects.
3. Leaves baro weighting at the hosted replay default, which is effectively
   "no flap transient deweighting".
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


from replaylib.paths import SCRIPTS_DIR
SCRIPT_DIR = SCRIPTS_DIR
REPLAY_DIR = SCRIPT_DIR.parent
ROOT = REPLAY_DIR.parents[1]

DEFAULT_INPUT_CSV = SCRIPT_DIR / "fullscale4.csv"
DEFAULT_OUTPUT_SVG = SCRIPT_DIR / "fullscale4_no_deweight_replay.svg"

FEET_TO_METERS = 0.3048
METERS_TO_FEET = 1.0 / FEET_TO_METERS


@dataclass
class LoggedSample:
    time_s: float
    status: str
    baro_agl_ft: float | None
    state_agl_ft: float | None
    state_apogee_ft: float | None


@dataclass
class ReplaySample:
    time_s: float
    altitude_ft: float
    velocity_fps: float
    apogee_ft: float
    status: str


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


def sanitize_filename(text: str) -> str:
    out = []
    for ch in text:
        if ch.isalnum():
            out.append(ch)
        elif ch in ("-", "_"):
            out.append(ch)
        else:
            out.append("_")
    return "".join(out).strip("_") or "output"


def find_replay_binary(explicit: Path | None) -> Path:
    candidates: list[Path] = []
    if explicit is not None:
        candidates.append(explicit)
    candidates.extend(
        [
            ROOT / "tools" / "build" / "bin" / "acs_replay",
            ROOT / "tools" / "replay" / "build" / "bin" / "acs_replay",
        ]
    )
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate
    raise SystemExit(
        "Could not locate `acs_replay`. Build it first with "
        "`cmake --build tools/build --target acs_replay`."
    )


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
        state_agl_ft = parse_float(raw.get("state_altitude_agl_feet"))
        state_apogee_ft = parse_float(raw.get("state_apogee_estimate_feet"))
        baro_agl_ft = None
        if absolute_alt_ft is not None:
            baro_agl_ft = max(0.0, absolute_alt_ft - pad_alt_ft)
        samples.append(
            LoggedSample(
                time_s=time_s,
                status=(raw.get("flight_status") or "").strip().lower(),
                baro_agl_ft=baro_agl_ft,
                state_agl_ft=state_agl_ft,
                state_apogee_ft=state_apogee_ft,
            )
        )
    return samples


def write_replay_input_csv(src: Path) -> Path:
    fd, temp_path = tempfile.mkstemp(
        suffix=".csv",
        prefix=f"{sanitize_filename(src.stem)}_no_deweight_",
        dir="/tmp",
    )
    os.close(fd)
    dst = Path(temp_path)

    with src.open(newline="") as handle_in, dst.open("w", newline="") as handle_out:
        reader = csv.DictReader(handle_in)
        if reader.fieldnames is None:
            raise SystemExit(f"Missing CSV header in {src}")

        keep_headers = [
            header
            for header in reader.fieldnames
            if not (
                header.startswith("state_")
                or header in ("has_filtered_state", "flight_status", "flight_status_raw")
            )
        ]
        writer = csv.DictWriter(handle_out, fieldnames=keep_headers)
        writer.writeheader()

        imu_prefixes = (
            "sensor_accel_icm_",
            "sensor_accel_bno_",
            "sensor_accel_lsm_",
            "sensor_gyro_",
            "sensor_gyro_bno_",
            "sensor_gyro_lsm_",
        )
        for raw in reader:
            row = dict(raw)
            for prefix in imu_prefixes:
                x_key = f"{prefix}x"
                y_key = f"{prefix}y"
                z_key = f"{prefix}z"
                if x_key not in row or y_key not in row or z_key not in row:
                    continue
                x_value = row.get(x_key, "")
                y_value = row.get(y_key, "")
                z_value = row.get(z_key, "")
                row[x_key] = z_value
                row[y_key] = y_value
                row[z_key] = x_value
            writer.writerow({header: row.get(header, "") for header in keep_headers})

    return dst


def run_hosted_replay(replay_binary: Path, replay_input_csv: Path) -> list[ReplaySample]:
    result = subprocess.run(
        [str(replay_binary), str(replay_input_csv), "--ignore-logged-state"],
        cwd=str(ROOT),
        text=True,
        capture_output=True,
        check=False,
    )
    if result.returncode != 0:
        raise SystemExit(result.stderr.strip() or result.stdout.strip() or "acs_replay failed")

    lines = result.stdout.splitlines()
    try:
        header_index = next(index for index, line in enumerate(lines) if line.startswith("time_s,"))
    except StopIteration as exc:
        raise SystemExit("acs_replay did not emit replay CSV output") from exc

    headers = lines[header_index].split(",")
    if headers[:5] != ["time_s", "altitude_m", "velocity_mps", "apogee_prediction_m", "status"]:
        raise SystemExit("Unexpected acs_replay output header")

    samples: list[ReplaySample] = []
    for line in lines[header_index + 1 :]:
        if line.startswith("Samples processed:"):
            break
        parts = line.split(",")
        if len(parts) != len(headers):
            continue
        try:
            time_s = float(parts[0])
            altitude_ft = float(parts[1]) * METERS_TO_FEET
            velocity_fps = float(parts[2]) * METERS_TO_FEET
            apogee_ft = float(parts[3]) * METERS_TO_FEET
        except ValueError:
            continue
        samples.append(
            ReplaySample(
                time_s=time_s,
                altitude_ft=altitude_ft,
                velocity_fps=velocity_fps,
                apogee_ft=apogee_ft,
                status=parts[4].strip().lower(),
            )
        )

    if not samples:
        raise SystemExit("acs_replay emitted no usable sample rows")
    return samples


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


def scale(value: float, src_lo: float, src_hi: float, dst_lo: float, dst_hi: float) -> float:
    if src_hi <= src_lo:
        return 0.5 * (dst_lo + dst_hi)
    ratio = (value - src_lo) / (src_hi - src_lo)
    return dst_lo + ratio * (dst_hi - dst_lo)


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
    svg_points: list[str] = []
    bottom = top + height
    for time_s, value in points:
        if value is None or not math.isfinite(value):
            continue
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
) -> str:
    phase_colors = {
        "ground": "#f3f4f6",
        "burn": "#fee2e2",
        "coast": "#dbeafe",
        "overshoot": "#ede9fe",
        "descent": "#dcfce7",
    }
    bottom = top + height
    elements: list[str] = []

    for status, run_start, run_end in phase_runs(phase_source):
        x0 = scale(run_start, x_min, x_max, left, left + width)
        x1 = scale(run_end, x_min, x_max, left, left + width)
        elements.append(
            f'<rect x="{x0:.2f}" y="{top:.2f}" width="{max(1.0, x1 - x0):.2f}" '
            f'height="{height:.2f}" fill="{phase_colors.get(status, "#f3f4f6")}" opacity="0.33"/>'
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
            f"{y_value:.0f}</text>"
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

    legend_x = left + 14.0
    legend_y = top + 16.0
    for index, (label, points, color, dash) in enumerate(series):
        polyline = build_polyline(points, x_min, x_max, y_min, y_max, left, top, width, height)
        if polyline:
            dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
            elements.append(
                f'<polyline fill="none" stroke="{color}" stroke-width="1.8"{dash_attr} points="{polyline}"/>'
            )
        line_y = legend_y + 20.0 * index
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        elements.append(
            f'<line x1="{legend_x:.2f}" y1="{line_y:.2f}" x2="{legend_x + 32:.2f}" y2="{line_y:.2f}" '
            f'stroke="{color}" stroke-width="2.6"{dash_attr}/>'
        )
        elements.append(
            f'<text x="{legend_x + 40:.2f}" y="{line_y + 4:.2f}" font-size="13" fill="#111827">{label}</text>'
        )

    elements.append(
        f'<rect x="{left:.2f}" y="{top:.2f}" width="{width:.2f}" height="{height:.2f}" '
        'fill="none" stroke="#111827" stroke-width="1.5"/>'
    )
    elements.append(
        f'<text x="{left:.2f}" y="{top - 16:.2f}" font-size="18" font-weight="700" fill="#111827">{title}</text>'
    )
    return "\n".join(elements)


def write_svg(
    output_path: Path,
    logged_samples: list[LoggedSample],
    replay_samples: list[ReplaySample],
    zoom_start: float,
    zoom_end: float,
) -> None:
    width = 1440
    height = 1080
    margin_left = 100.0
    panel_width = 1260.0
    panel_height = 250.0

    logged_times = [sample.time_s for sample in logged_samples]
    x_overview_min = logged_times[0]
    x_overview_max = logged_times[-1]

    logged_baro_points = [(sample.time_s, sample.baro_agl_ft) for sample in logged_samples]
    logged_state_alt_points = [(sample.time_s, sample.state_agl_ft) for sample in logged_samples]
    logged_state_apogee_points = [(sample.time_s, sample.state_apogee_ft) for sample in logged_samples]
    replay_alt_points = [(sample.time_s, sample.altitude_ft) for sample in replay_samples]
    replay_apogee_points = [(sample.time_s, sample.apogee_ft) for sample in replay_samples]

    max_alt_ft = max(
        value
        for _, value in logged_baro_points + logged_state_alt_points + replay_alt_points
        if value is not None and math.isfinite(value)
    )
    max_apogee_ft = max(
        value
        for _, value in logged_state_apogee_points + replay_apogee_points
        if value is not None and math.isfinite(value)
    )
    baro_peak_ft = max(
        value for _, value in logged_baro_points if value is not None and math.isfinite(value)
    )

    overview_alt_y_max = math.ceil((max_alt_ft + 100.0) / 100.0) * 100.0
    zoom_alt_y_max = overview_alt_y_max
    apogee_y_max = math.ceil((max(max_apogee_ft, baro_peak_ft) + 100.0) / 100.0) * 100.0

    logged_final_apogee_ft = next(
        sample.state_apogee_ft
        for sample in reversed(logged_samples)
        if sample.state_apogee_ft is not None and math.isfinite(sample.state_apogee_ft)
    )
    replay_final_apogee_ft = next(
        sample.apogee_ft
        for sample in reversed(replay_samples)
        if math.isfinite(sample.apogee_ft)
    )
    replay_peak_apogee_ft = max(sample.apogee_ft for sample in replay_samples if math.isfinite(sample.apogee_ft))

    subtitle_lines = [
        f"Logged final state apogee: {logged_final_apogee_ft:.1f} ft",
        f"Replay no-deweight final apogee: {replay_final_apogee_ft:.1f} ft",
        f"Replay no-deweight peak predicted apogee: {replay_peak_apogee_ft:.1f} ft",
        f"Raw baro AGL peak: {baro_peak_ft:.1f} ft",
    ]

    elements = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        f'<text x="{width / 2:.2f}" y="42" text-anchor="middle" font-family="monospace" '
        'font-size="28" font-weight="700" fill="#111827">No-Deweight Hosted Replay vs Logged Apogee</text>',
    ]

    for index, line in enumerate(subtitle_lines):
        elements.append(
            f'<text x="{width / 2:.2f}" y="{74 + index * 18:.2f}" text-anchor="middle" '
            'font-family="monospace" font-size="15" fill="#374151">'
            f"{line}</text>"
        )

    elements.append(
        draw_panel(
            title="Altitude AGL Overview (ft)",
            x_min=x_overview_min,
            x_max=x_overview_max,
            y_min=0.0,
            y_max=overview_alt_y_max,
            left=margin_left,
            top=160.0,
            width=panel_width,
            height=panel_height,
            phase_source=logged_samples,
            series=[
                ("Baro AGL", logged_baro_points, "#111827", ""),
                ("Logged State Alt", logged_state_alt_points, "#2563eb", ""),
                ("Replay Alt (No Deweight)", replay_alt_points, "#d97706", ""),
            ],
            reference_lines=[],
        )
    )
    elements.append(
        draw_panel(
            title="Altitude AGL Flight Window (ft)",
            x_min=zoom_start,
            x_max=zoom_end,
            y_min=0.0,
            y_max=zoom_alt_y_max,
            left=margin_left,
            top=490.0,
            width=panel_width,
            height=panel_height,
            phase_source=logged_samples,
            series=[
                ("Baro AGL", logged_baro_points, "#111827", ""),
                ("Logged State Alt", logged_state_alt_points, "#2563eb", ""),
                ("Replay Alt (No Deweight)", replay_alt_points, "#d97706", ""),
            ],
            reference_lines=[],
        )
    )
    elements.append(
        draw_panel(
            title="Apogee Prediction Flight Window (ft)",
            x_min=zoom_start,
            x_max=zoom_end,
            y_min=0.0,
            y_max=apogee_y_max,
            left=margin_left,
            top=820.0,
            width=panel_width,
            height=panel_height - 40.0,
            phase_source=logged_samples,
            series=[
                ("Logged State Apogee", logged_state_apogee_points, "#1d4ed8", ""),
                ("Replay Apogee (No Deweight)", replay_apogee_points, "#dc2626", ""),
            ],
            reference_lines=[("Baro AGL Peak", baro_peak_ft, "#059669")],
        )
    )
    elements.append("</svg>")

    output_path.write_text("\n".join(elements))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run hosted replay with no baro deweighting and plot the resulting apogee trace."
    )
    parser.add_argument("input_csv", nargs="?", type=Path, default=DEFAULT_INPUT_CSV)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT_SVG)
    parser.add_argument("--replay-binary", type=Path, default=None)
    parser.add_argument("--zoom-start", type=float, default=178.5)
    parser.add_argument("--zoom-end", type=float, default=191.7)
    args = parser.parse_args()

    replay_binary = find_replay_binary(args.replay_binary)
    logged_samples = load_logged_samples(args.input_csv)
    temp_csv = write_replay_input_csv(args.input_csv)
    try:
        replay_samples = run_hosted_replay(replay_binary, temp_csv)
    finally:
        try:
            temp_csv.unlink()
        except OSError:
            pass

    write_svg(args.output, logged_samples, replay_samples, args.zoom_start, args.zoom_end)

    logged_final_apogee_ft = next(
        sample.state_apogee_ft
        for sample in reversed(logged_samples)
        if sample.state_apogee_ft is not None and math.isfinite(sample.state_apogee_ft)
    )
    replay_final_apogee_ft = next(
        sample.apogee_ft for sample in reversed(replay_samples) if math.isfinite(sample.apogee_ft)
    )
    replay_peak_apogee_ft = max(sample.apogee_ft for sample in replay_samples if math.isfinite(sample.apogee_ft))
    baro_peak_ft = max(
        sample.baro_agl_ft
        for sample in logged_samples
        if sample.baro_agl_ft is not None and math.isfinite(sample.baro_agl_ft)
    )

    print(f"Input CSV: {args.input_csv}")
    print(f"Replay binary: {replay_binary}")
    print(f"Output SVG: {args.output}")
    print(f"Logged final state apogee: {logged_final_apogee_ft:.2f} ft")
    print(f"Replay final no-deweight apogee: {replay_final_apogee_ft:.2f} ft")
    print(f"Replay peak no-deweight apogee: {replay_peak_apogee_ft:.2f} ft")
    print(f"Baro AGL peak: {baro_peak_ft:.2f} ft")


if __name__ == "__main__":
    main()
