#!/usr/bin/env python3
"""Plot fullscale_4 flap actuation against the hosted replay overshoot transition."""

from __future__ import annotations

import argparse
import csv
import math
import os
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path

from plot_fullscale_4_validation import DEFAULT_INPUT_CSV
from plot_fullscale_4_validation import compute_baro_peak
from plot_fullscale_4_validation import find_replay_binary
from plot_fullscale_4_validation import first_time_where
from plot_fullscale_4_validation import load_logged_samples
from plot_fullscale_4_validation import logged_series
from plot_fullscale_4_validation import phase_runs
from plot_fullscale_4_validation import sanitize_filename
from plot_fullscale_4_validation import scale
from plot_fullscale_4_validation import settling_runs
from plot_fullscale_4_validation import thin_points
from replay_legacy_frame_adapter import HISTORICAL_FULLSCALE_CP_OFFSET_M
from replay_legacy_frame_adapter import HISTORICAL_FULLSCALE_DRY_MASS_KG
from replay_legacy_frame_adapter import HISTORICAL_FULLSCALE_MOI_KGM2
from replay_legacy_frame_adapter import PRESETS_BY_NAME
from replay_legacy_frame_adapter import write_transformed_csv


from replaylib.paths import SCRIPTS_DIR
SCRIPT_DIR = SCRIPTS_DIR
ROOT = SCRIPT_DIR.parents[2]
DEFAULT_OUTPUT_SVG = ROOT / "tools" / "replay" / "plots" / "fullscale_4_overshoot_actuation.svg"
DEFAULT_APOGEE_TARGET_M = 822.96


@dataclass
class ReplayStatusSample:
    time_s: float
    status: str


def parse_replay_status(stdout: str) -> list[ReplayStatusSample]:
    lines = stdout.splitlines()
    try:
        header_index = next(index for index, line in enumerate(lines) if line.startswith("time_s,"))
    except StopIteration as exc:
        raise SystemExit("acs_replay did not emit replay CSV output") from exc

    samples: list[ReplayStatusSample] = []
    reader = csv.DictReader(line for line in lines[header_index:] if line and not line.startswith("Samples processed:"))
    for raw in reader:
        try:
            time_s = float(raw["time_s"])
        except (KeyError, TypeError, ValueError):
            continue
        samples.append(ReplayStatusSample(time_s=time_s, status=(raw.get("status") or "").strip().lower()))
    if not samples:
        raise SystemExit("acs_replay emitted no status samples")
    return samples


def run_hosted_status(
    replay_binary: Path,
    input_csv: Path,
    preset_name: str,
    apogee_target_m: float,
) -> list[ReplayStatusSample]:
    preset = PRESETS_BY_NAME[preset_name]
    with tempfile.NamedTemporaryFile(
        suffix=f"_{sanitize_filename(preset.name)}.csv",
        prefix=f"{sanitize_filename(input_csv.stem)}_overshoot_actuation_",
        dir="/tmp",
        delete=False,
    ) as handle:
        temp_csv = Path(handle.name)
    try:
        write_transformed_csv(input_csv, preset, temp_csv)
        command = [
            str(replay_binary),
            str(temp_csv),
            f"--dry-mass-kg={HISTORICAL_FULLSCALE_DRY_MASS_KG}",
            f"--cp-offset-m={HISTORICAL_FULLSCALE_CP_OFFSET_M}",
            f"--moment-of-inertia-kgm2={HISTORICAL_FULLSCALE_MOI_KGM2}",
            "--apogee-target",
            str(apogee_target_m),
            "--ignore-logged-state",
            "--rebuild-main-quaternion",
        ]
        result = subprocess.run(command, cwd=str(ROOT), text=True, capture_output=True, check=False)
    finally:
        try:
            os.unlink(temp_csv)
        except OSError:
            pass
    if result.returncode != 0:
        raise SystemExit(result.stderr.strip() or result.stdout.strip() or "acs_replay failed")
    return parse_replay_status(result.stdout)


def status_runs(samples: list[ReplayStatusSample]) -> list[tuple[str, float, float]]:
    runs: list[tuple[str, float, float]] = []
    if not samples:
        return runs
    current = samples[0].status
    start = samples[0].time_s
    for sample in samples[1:]:
        if sample.status != current:
            runs.append((current, start, sample.time_s))
            current = sample.status
            start = sample.time_s
    runs.append((current, start, samples[-1].time_s))
    return runs


def first_status_time(samples: list[ReplayStatusSample], status: str) -> float | None:
    return next((sample.time_s for sample in samples if sample.status == status), None)


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
    filtered = [
        (time_s, value)
        for time_s, value in points
        if value is not None and math.isfinite(time_s) and math.isfinite(value) and x_min <= time_s <= x_max
    ]
    bottom = top + height
    svg_points: list[str] = []
    for time_s, value in thin_points(filtered, 1800):
        x = scale(time_s, x_min, x_max, left, left + width)
        y = scale(value, y_min, y_max, bottom, top)
        svg_points.append(f"{x:.2f},{y:.2f}")
    return " ".join(svg_points)


def write_svg(
    output_svg: Path,
    input_csv: Path,
    preset_name: str,
    replay_binary: Path,
    apogee_target_m: float,
) -> dict[str, float | None]:
    logged_samples = load_logged_samples(input_csv)
    replay_samples = run_hosted_status(replay_binary, input_csv, preset_name, apogee_target_m)

    baro_apogee_time_s, baro_apogee_ft = compute_baro_peak(logged_samples)
    replay_coast_s = first_status_time(replay_samples, "coast")
    replay_overshoot_s = first_status_time(replay_samples, "overshoot")
    replay_descent_s = first_status_time(replay_samples, "descent")
    first_flap_command_s = first_time_where(logged_samples, lambda sample: (sample.flap_command_deg or 0.0) > 0.5)
    first_flap_effective_s = first_time_where(logged_samples, lambda sample: (sample.flap_effective_deg or 0.0) > 0.5)

    zoom_anchor = replay_coast_s or first_flap_command_s or logged_samples[0].time_s
    x_min = max(logged_samples[0].time_s, zoom_anchor - 1.5)
    x_max = min(logged_samples[-1].time_s, max(baro_apogee_time_s + 1.5, (replay_descent_s or baro_apogee_time_s) + 0.5))

    flap_command = logged_series(logged_samples, lambda sample: sample.flap_command_deg)
    flap_effective = logged_series(logged_samples, lambda sample: sample.flap_effective_deg)
    flap_values = [
        value
        for _, value in flap_command + flap_effective
        if value is not None and math.isfinite(value)
    ]
    y_max = max(5.0, math.ceil((max(flap_values, default=0.0) + 2.0) / 5.0) * 5.0)

    width = 1280
    height = 640
    left = 96.0
    top = 132.0
    plot_width = 1080.0
    plot_height = 360.0
    bottom = top + plot_height

    phase_colors = {
        "ground": "#f3f4f6",
        "burn": "#fee2e2",
        "coast": "#dbeafe",
        "overshoot": "#ede9fe",
        "descent": "#dcfce7",
    }
    elements = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        f'<text x="{width / 2:.2f}" y="42" text-anchor="middle" font-family="monospace" '
        'font-size="26" font-weight="700" fill="#111827">fullscale_4 overshoot actuation</text>',
        f'<text x="{width / 2:.2f}" y="72" text-anchor="middle" font-family="monospace" '
        f'font-size="14" fill="#374151">Hosted replay preset: {preset_name} | '
        f'target {apogee_target_m:.2f} m | baro apogee {baro_apogee_ft:.1f} ft at t={baro_apogee_time_s:.2f}s</text>',
        f'<text x="{width / 2:.2f}" y="94" text-anchor="middle" font-family="monospace" '
        f'font-size="13" fill="#64748b">Overshoot marker comes from rebuilt acs_replay status; '
        f'flap angles are logged command/effective rails.</text>',
    ]

    for status, run_start, run_end in status_runs(replay_samples):
        if run_end < x_min or run_start > x_max:
            continue
        x0 = scale(max(run_start, x_min), x_min, x_max, left, left + plot_width)
        x1 = scale(min(run_end, x_max), x_min, x_max, left, left + plot_width)
        elements.append(
            f'<rect x="{x0:.2f}" y="{top:.2f}" width="{max(1.0, x1 - x0):.2f}" '
            f'height="{plot_height:.2f}" fill="{phase_colors.get(status, "#f3f4f6")}" opacity="0.42"/>'
        )

    for start, end in settling_runs(logged_samples):
        if end < x_min or start > x_max:
            continue
        x0 = scale(max(start, x_min), x_min, x_max, left, left + plot_width)
        x1 = scale(min(end, x_max), x_min, x_max, left, left + plot_width)
        elements.append(
            f'<rect x="{x0:.2f}" y="{top:.2f}" width="{max(1.0, x1 - x0):.2f}" '
            f'height="{plot_height:.2f}" fill="#fb923c" opacity="0.14"/>'
        )

    for index in range(6):
        y = top + index * plot_height / 5.0
        x = left + index * plot_width / 5.0
        y_value = y_max - index * y_max / 5.0
        x_value = x_min + index * (x_max - x_min) / 5.0
        elements.append(
            f'<line x1="{left:.2f}" y1="{y:.2f}" x2="{left + plot_width:.2f}" y2="{y:.2f}" '
            'stroke="#d1d5db" stroke-width="1"/>'
        )
        elements.append(
            f'<line x1="{x:.2f}" y1="{top:.2f}" x2="{x:.2f}" y2="{bottom:.2f}" '
            'stroke="#e5e7eb" stroke-width="1"/>'
        )
        elements.append(
            f'<text x="{left - 10:.2f}" y="{y + 4:.2f}" text-anchor="end" font-size="12" '
            f'fill="#111827">{y_value:.1f}</text>'
        )
        elements.append(
            f'<text x="{x:.2f}" y="{bottom + 22:.2f}" text-anchor="middle" font-size="12" '
            f'fill="#111827">{x_value:.2f}</text>'
        )

    markers = [
        ("coast", replay_coast_s, "#2563eb"),
        ("overshoot", replay_overshoot_s, "#7c3aed"),
        ("descent", replay_descent_s, "#16a34a"),
        ("cmd > 0.5 deg", first_flap_command_s, "#b45309"),
        ("eff > 0.5 deg", first_flap_effective_s, "#047857"),
        ("baro apogee", baro_apogee_time_s, "#dc2626"),
    ]
    for label, time_s, color in markers:
        if time_s is None or time_s < x_min or time_s > x_max:
            continue
        x = scale(time_s, x_min, x_max, left, left + plot_width)
        elements.append(
            f'<line x1="{x:.2f}" y1="{top:.2f}" x2="{x:.2f}" y2="{bottom:.2f}" '
            f'stroke="{color}" stroke-width="1.5" stroke-dasharray="5 4"/>'
        )
        elements.append(f'<text x="{x + 5:.2f}" y="{top + 18:.2f}" font-size="11" fill="{color}">{label}</text>')

    series = [
        ("Flap command", flap_command, "#b45309", ""),
        ("Flap effective", flap_effective, "#047857", ""),
    ]
    for label, points, color, dash in series:
        polyline = build_polyline(points, x_min, x_max, 0.0, y_max, left, top, plot_width, plot_height)
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        elements.append(f'<polyline fill="none" stroke="{color}" stroke-width="2.2"{dash_attr} points="{polyline}"/>')

    legend_x = left + 16.0
    legend_y = bottom + 58.0
    legend_items = [
        ("Flap command", "#b45309", ""),
        ("Flap effective", "#047857", ""),
        ("Replay coast", "#dbeafe", "box"),
        ("Replay overshoot", "#ede9fe", "box"),
        ("Actuator settling", "#fb923c", "box"),
    ]
    for index, (label, color, kind) in enumerate(legend_items):
        x = legend_x + index * 205.0
        if kind == "box":
            elements.append(f'<rect x="{x:.2f}" y="{legend_y - 11:.2f}" width="28" height="12" fill="{color}" opacity="0.60"/>')
        else:
            elements.append(f'<line x1="{x:.2f}" y1="{legend_y - 5:.2f}" x2="{x + 30:.2f}" y2="{legend_y - 5:.2f}" stroke="{color}" stroke-width="2.4"/>')
        elements.append(f'<text x="{x + 38:.2f}" y="{legend_y:.2f}" font-size="12" fill="#111827">{label}</text>')

    elements.append(
        f'<rect x="{left:.2f}" y="{top:.2f}" width="{plot_width:.2f}" height="{plot_height:.2f}" '
        'fill="none" stroke="#111827" stroke-width="1.5"/>'
    )
    elements.append(f'<text x="{left + plot_width / 2:.2f}" y="{height - 34:.2f}" text-anchor="middle" font-size="13" fill="#374151">Time (s)</text>')
    elements.append(f'<text x="32" y="{top + plot_height / 2:.2f}" text-anchor="middle" font-size="13" fill="#374151" transform="rotate(-90 32 {top + plot_height / 2:.2f})">Flap angle (deg)</text>')
    elements.append("</svg>")

    output_svg.parent.mkdir(parents=True, exist_ok=True)
    output_svg.write_text("\n".join(elements) + "\n", encoding="utf-8")
    return {
        "baro_apogee_time_s": baro_apogee_time_s,
        "replay_coast_s": replay_coast_s,
        "replay_overshoot_s": replay_overshoot_s,
        "replay_descent_s": replay_descent_s,
        "first_flap_command_s": first_flap_command_s,
        "first_flap_effective_s": first_flap_effective_s,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_csv", nargs="?", type=Path, default=DEFAULT_INPUT_CSV)
    parser.add_argument("--replay-binary", type=Path, default=None)
    parser.add_argument("--preset", choices=sorted(PRESETS_BY_NAME), default="best-coast")
    parser.add_argument(
        "--apogee-target-m",
        type=float,
        default=DEFAULT_APOGEE_TARGET_M,
        help="Replay target apogee in meters. Defaults to the current firmware setting.",
    )
    parser.add_argument("--output-svg", type=Path, default=DEFAULT_OUTPUT_SVG)
    args = parser.parse_args()

    replay_binary = find_replay_binary(args.replay_binary)
    summary = write_svg(args.output_svg, args.input_csv, args.preset, replay_binary, args.apogee_target_m)
    print(f"Input CSV: {args.input_csv}")
    print(f"Replay binary: {replay_binary}")
    print(f"Preset: {args.preset}")
    print(f"Apogee target: {args.apogee_target_m:.2f} m")
    for key, value in summary.items():
        if value is not None:
            print(f"{key}: {value:.3f}s")
        else:
            print(f"{key}: none")
    print(f"Wrote {args.output_svg}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
