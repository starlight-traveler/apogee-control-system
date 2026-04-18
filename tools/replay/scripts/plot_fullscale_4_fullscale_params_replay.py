#!/usr/bin/env python3
"""
Plot the historical fullscale_4 replay against the current predictor using the
original full-scale vehicle parameters.

This intentionally runs the replay in seeded mode. For this historical CSV, the
hosted `--ignore-logged-state` rebuild path still does not reconstruct a usable
flight state under the current estimator conventions, so the meaningful
comparison is:

1. Raw baro AGL from the flight log.
2. Logged fused altitude and logged apogee estimate from the flight log.
3. Current replay predictor with full-scale parameters.
4. Current replay predictor with full-scale parameters plus rebuilt zenith.
"""

from __future__ import annotations

import csv
import math
import os
import subprocess
from dataclasses import dataclass
from pathlib import Path

from plot_no_deweight_apogee_replay import draw_panel
from plot_no_deweight_apogee_replay import find_replay_binary
from plot_no_deweight_apogee_replay import load_logged_samples


SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parents[2]

DEFAULT_INPUT_CSV = SCRIPT_DIR / "fullscale_4.csv"
DEFAULT_OUTPUT_SVG = SCRIPT_DIR / "fullscale_4_fullscale_params_replay.svg"
DEFAULT_OUTPUT_CSV = SCRIPT_DIR / "fullscale_4_fullscale_params_replay.csv"

FULLSCALE_DRY_MASS_KG = 18.24
FULLSCALE_CP_OFFSET_M = 0.4437
FULLSCALE_MOI_KGM2 = 8.28
FEET_TO_METERS = 0.3048
METERS_TO_FEET = 1.0 / FEET_TO_METERS


@dataclass
class ReplaySample:
    time_s: float
    altitude_ft: float
    velocity_fps: float
    apogee_ft: float
    status: str


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


def compute_baro_peak_ft(logged_samples) -> float:
    return max(
        sample.baro_agl_ft
        for sample in logged_samples
        if sample.baro_agl_ft is not None and math.isfinite(sample.baro_agl_ft)
    )


def run_seeded_replay(
    replay_binary: Path, input_csv: Path, rebuild_main_quaternion: bool
) -> list[ReplaySample]:
    command = [
        str(replay_binary),
        str(input_csv),
        f"--dry-mass-kg={FULLSCALE_DRY_MASS_KG}",
        f"--cp-offset-m={FULLSCALE_CP_OFFSET_M}",
        f"--moment-of-inertia-kgm2={FULLSCALE_MOI_KGM2}",
    ]
    if rebuild_main_quaternion:
        command.append("--rebuild-main-quaternion")

    result = subprocess.run(
        command,
        cwd=str(ROOT),
        text=True,
        capture_output=True,
        check=False,
    )
    if result.returncode != 0:
        raise SystemExit(result.stderr.strip() or result.stdout.strip() or "acs_replay failed")

    lines = result.stdout.splitlines()
    try:
        header_index = next(i for i, line in enumerate(lines) if line.startswith("time_s,"))
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


def summarize_replay(samples: list[ReplaySample]) -> tuple[ReplaySample | None, ReplaySample | None]:
    first_coast = next((sample for sample in samples if sample.status == "coast"), None)
    peak = max(samples, key=lambda sample: sample.apogee_ft) if samples else None
    return first_coast, peak


def write_summary_csv(
    output_csv: Path,
    logged_samples,
    seeded_samples: list[ReplaySample],
    rebuilt_samples: list[ReplaySample],
) -> None:
    logged_by_time = {sample.time_s: sample for sample in logged_samples}
    rebuilt_by_time = {sample.time_s: sample for sample in rebuilt_samples}

    with output_csv.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "time_s",
                "status",
                "baro_agl_ft",
                "logged_state_agl_ft",
                "logged_state_apogee_ft",
                "seeded_altitude_ft",
                "seeded_velocity_fps",
                "seeded_apogee_ft",
                "rebuilt_zenith_altitude_ft",
                "rebuilt_zenith_velocity_fps",
                "rebuilt_zenith_apogee_ft",
            ]
        )
        for seeded in seeded_samples:
            logged = logged_by_time.get(seeded.time_s)
            rebuilt = rebuilt_by_time.get(seeded.time_s)
            writer.writerow(
                [
                    seeded.time_s,
                    seeded.status,
                    "" if logged is None else logged.baro_agl_ft,
                    "" if logged is None else logged.state_agl_ft,
                    "" if logged is None else logged.state_apogee_ft,
                    seeded.altitude_ft,
                    seeded.velocity_fps,
                    seeded.apogee_ft,
                    "" if rebuilt is None else rebuilt.altitude_ft,
                    "" if rebuilt is None else rebuilt.velocity_fps,
                    "" if rebuilt is None else rebuilt.apogee_ft,
                ]
            )


def write_svg(
    output_svg: Path,
    logged_samples,
    seeded_samples: list[ReplaySample],
    rebuilt_samples: list[ReplaySample],
) -> None:
    width = 1480
    height = 1180
    margin_left = 110.0
    panel_width = 1280.0
    panel_height = 260.0
    panel_gap = 90.0

    x_all_min = logged_samples[0].time_s
    x_all_max = logged_samples[-1].time_s
    x_zoom_min = 176.0
    x_zoom_max = 194.0

    baro_points = [(sample.time_s, sample.baro_agl_ft) for sample in logged_samples]
    state_alt_points = [(sample.time_s, sample.state_agl_ft) for sample in logged_samples]
    state_apogee_points = [(sample.time_s, sample.state_apogee_ft) for sample in logged_samples]
    seeded_apogee_points = [(sample.time_s, sample.apogee_ft) for sample in seeded_samples]
    rebuilt_apogee_points = [(sample.time_s, sample.apogee_ft) for sample in rebuilt_samples]
    seeded_alt_points = [(sample.time_s, sample.altitude_ft) for sample in seeded_samples]

    peak_baro_ft = compute_baro_peak_ft(logged_samples)
    logged_peak_apogee_ft = max(
        sample.state_apogee_ft
        for sample in logged_samples
        if sample.state_apogee_ft is not None and math.isfinite(sample.state_apogee_ft)
    )

    overview_y_max = max(
        peak_baro_ft,
        logged_peak_apogee_ft,
        max(sample.apogee_ft for sample in seeded_samples),
        max(sample.apogee_ft for sample in rebuilt_samples),
    ) * 1.05
    overview_y_min = 0.0

    error_points_logged = [
        (sample.time_s, None if sample.state_apogee_ft is None else sample.state_apogee_ft - peak_baro_ft)
        for sample in logged_samples
    ]
    error_points_seeded = [(sample.time_s, sample.apogee_ft - peak_baro_ft) for sample in seeded_samples]
    error_points_rebuilt = [(sample.time_s, sample.apogee_ft - peak_baro_ft) for sample in rebuilt_samples]
    error_y_min = min(
        value
        for _, value in error_points_logged + error_points_seeded + error_points_rebuilt
        if value is not None and math.isfinite(value)
    )
    error_y_max = max(
        value
        for _, value in error_points_logged + error_points_seeded + error_points_rebuilt
        if value is not None and math.isfinite(value)
    )
    padding = 0.1 * max(abs(error_y_min), abs(error_y_max), 50.0)
    error_y_min -= padding
    error_y_max += padding

    panels = []
    panels.append(
        draw_panel(
            "Full Flight Overview: Logged Flight vs Current Full-Scale Replay",
            x_all_min,
            x_all_max,
            overview_y_min,
            overview_y_max,
            margin_left,
            90.0,
            panel_width,
            panel_height,
            logged_samples,
            [
                ("Raw baro AGL", baro_points, "#1d4ed8", ""),
                ("Logged fused altitude", state_alt_points, "#7c3aed", "8 5"),
                ("Seeded replay altitude", seeded_alt_points, "#0f766e", ""),
                ("Logged apogee estimate", state_apogee_points, "#9a3412", "6 5"),
                ("Seeded replay apogee", seeded_apogee_points, "#dc2626", ""),
                ("Seeded replay apogee + rebuilt zenith", rebuilt_apogee_points, "#16a34a", ""),
            ],
            [
                ("Raw baro apogee", peak_baro_ft, "#111827"),
            ],
        )
    )
    panels.append(
        draw_panel(
            "Coast Zoom: Apogee Prediction",
            x_zoom_min,
            x_zoom_max,
            overview_y_min,
            overview_y_max,
            margin_left,
            90.0 + panel_height + panel_gap,
            panel_width,
            panel_height,
            logged_samples,
            [
                ("Raw baro AGL", baro_points, "#1d4ed8", ""),
                ("Logged fused altitude", state_alt_points, "#7c3aed", "8 5"),
                ("Logged apogee estimate", state_apogee_points, "#9a3412", "6 5"),
                ("Seeded replay apogee", seeded_apogee_points, "#dc2626", ""),
                ("Seeded replay apogee + rebuilt zenith", rebuilt_apogee_points, "#16a34a", ""),
            ],
            [
                ("Raw baro apogee", peak_baro_ft, "#111827"),
            ],
        )
    )
    panels.append(
        draw_panel(
            "Prediction Error Relative to Raw Baro Apogee",
            x_zoom_min,
            x_zoom_max,
            error_y_min,
            error_y_max,
            margin_left,
            90.0 + 2.0 * (panel_height + panel_gap),
            panel_width,
            panel_height,
            logged_samples,
            [
                ("Logged apogee error", error_points_logged, "#9a3412", "6 5"),
                ("Seeded replay error", error_points_seeded, "#dc2626", ""),
                ("Seeded replay error + rebuilt zenith", error_points_rebuilt, "#16a34a", ""),
            ],
            [
                ("Zero error", 0.0, "#111827"),
            ],
        )
    )

    first_seeded_coast, peak_seeded = summarize_replay(seeded_samples)
    first_rebuilt_coast, peak_rebuilt = summarize_replay(rebuilt_samples)

    note_lines = [
        "Replay settings:",
        f"dry_mass_kg={FULLSCALE_DRY_MASS_KG}, cp_offset_m={FULLSCALE_CP_OFFSET_M}, moi_kgm2={FULLSCALE_MOI_KGM2}",
        f"raw_baro_peak={peak_baro_ft * FEET_TO_METERS:.2f} m ({peak_baro_ft:.1f} ft)",
        "",
        "Seeded replay:",
        "uses current predictor against logged historical flight state",
        f"first coast prediction={0.0 if first_seeded_coast is None else first_seeded_coast.apogee_ft * FEET_TO_METERS:.2f} m",
        f"peak prediction={0.0 if peak_seeded is None else peak_seeded.apogee_ft * FEET_TO_METERS:.2f} m",
        "",
        "Seeded replay + rebuilt zenith:",
        f"first coast prediction={0.0 if first_rebuilt_coast is None else first_rebuilt_coast.apogee_ft * FEET_TO_METERS:.2f} m",
        f"peak prediction={0.0 if peak_rebuilt is None else peak_rebuilt.apogee_ft * FEET_TO_METERS:.2f} m",
        "",
        "Current hosted replay with --ignore-logged-state still stays in ground on this old CSV,",
        "so the useful historical comparison is the seeded replay, not the full estimator rebuild.",
    ]

    note_svg = []
    note_x = margin_left
    note_y = 90.0 + 3.0 * (panel_height + panel_gap) - 20.0
    note_svg.append(
        f'<text x="{note_x:.2f}" y="{note_y:.2f}" font-size="18" font-weight="700" fill="#111827">'
        "Interpretation</text>"
    )
    for index, line in enumerate(note_lines):
        y = note_y + 28.0 + 20.0 * index
        note_svg.append(
            f'<text x="{note_x:.2f}" y="{y:.2f}" font-size="14" fill="#111827">{line}</text>'
        )

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="#f8fafc"/>
  <text x="{margin_left:.2f}" y="44" font-size="28" font-weight="700" fill="#0f172a">fullscale_4 replay with historical full-scale vehicle parameters</text>
  <text x="{margin_left:.2f}" y="68" font-size="15" fill="#334155">Current predictor vs historical flight data</text>
  {' '.join(panels)}
  {' '.join(note_svg)}
</svg>
"""
    output_svg.write_text(svg)


def main() -> None:
    replay_binary = find_replay_binary(None)
    logged_samples = load_logged_samples(DEFAULT_INPUT_CSV)
    seeded_samples = run_seeded_replay(replay_binary, DEFAULT_INPUT_CSV, rebuild_main_quaternion=False)
    rebuilt_samples = run_seeded_replay(replay_binary, DEFAULT_INPUT_CSV, rebuild_main_quaternion=True)

    write_summary_csv(DEFAULT_OUTPUT_CSV, logged_samples, seeded_samples, rebuilt_samples)
    write_svg(DEFAULT_OUTPUT_SVG, logged_samples, seeded_samples, rebuilt_samples)

    first_seeded_coast, peak_seeded = summarize_replay(seeded_samples)
    first_rebuilt_coast, peak_rebuilt = summarize_replay(rebuilt_samples)
    peak_baro_ft = compute_baro_peak_ft(logged_samples)

    print(f"Wrote {DEFAULT_OUTPUT_CSV}")
    print(f"Wrote {DEFAULT_OUTPUT_SVG}")
    print(f"Raw baro peak: {peak_baro_ft * FEET_TO_METERS:.3f} m")
    if first_seeded_coast is not None:
        print(f"Seeded first coast prediction: {first_seeded_coast.apogee_ft * FEET_TO_METERS:.3f} m")
    if peak_seeded is not None:
        print(f"Seeded peak prediction: {peak_seeded.apogee_ft * FEET_TO_METERS:.3f} m")
    if first_rebuilt_coast is not None:
        print(
            "Seeded + rebuilt zenith first coast prediction: "
            f"{first_rebuilt_coast.apogee_ft * FEET_TO_METERS:.3f} m"
        )
    if peak_rebuilt is not None:
        print(f"Seeded + rebuilt zenith peak prediction: {peak_rebuilt.apogee_ft * FEET_TO_METERS:.3f} m")


if __name__ == "__main__":
    main()
