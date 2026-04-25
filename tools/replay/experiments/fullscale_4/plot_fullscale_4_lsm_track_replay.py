#!/usr/bin/env python3
"""
Plot the `lsm-track` legacy-frame replay for fullscale_4 against the historical
flight data.

This uses the current `acs_replay` hosted replay path (`--ignore-logged-state`)
after transforming the historical raw rails into the selected legacy adapter
frame. The comparison is against:

1. Raw baro AGL from the flight log.
2. Logged fused altitude and logged apogee estimate from the historical log.
3. Seeded replay with historical full-scale parameters.
4. Full hosted replay using the `lsm-track` legacy-frame adapter preset.
"""

from __future__ import annotations

import csv
import math
import os
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path

from plot_no_deweight_apogee_replay import draw_panel
from plot_no_deweight_apogee_replay import find_replay_binary
from plot_no_deweight_apogee_replay import load_logged_samples
from replay_legacy_frame_adapter import HISTORICAL_APOGEE_TIME_S
from replay_legacy_frame_adapter import HISTORICAL_BARO_APOGEE_M
from replay_legacy_frame_adapter import HISTORICAL_FULLSCALE_CP_OFFSET_M
from replay_legacy_frame_adapter import HISTORICAL_FULLSCALE_DRY_MASS_KG
from replay_legacy_frame_adapter import HISTORICAL_FULLSCALE_MOI_KGM2
from replay_legacy_frame_adapter import PRESETS_BY_NAME
from replay_legacy_frame_adapter import write_transformed_csv


from replaylib.paths import SCRIPTS_DIR
SCRIPT_DIR = SCRIPTS_DIR
ROOT = SCRIPT_DIR.parents[2]

DEFAULT_INPUT_CSV = SCRIPT_DIR / "fullscale_4.csv"
DEFAULT_OUTPUT_SVG = SCRIPT_DIR / "fullscale_4_lsm_track_replay.svg"
DEFAULT_OUTPUT_CSV = SCRIPT_DIR / "fullscale_4_lsm_track_replay.csv"
DEFAULT_PRESET = "lsm-track"

FEET_TO_METERS = 0.3048
METERS_TO_FEET = 1.0 / FEET_TO_METERS


@dataclass
class ReplaySample:
    time_s: float
    altitude_ft: float
    velocity_fps: float
    apogee_ft: float
    status: str


def run_replay(
    replay_binary: Path,
    input_csv: Path,
    *,
    ignore_logged_state: bool,
    rebuild_main_quaternion: bool,
) -> list[ReplaySample]:
    command = [
        str(replay_binary),
        str(input_csv),
        f"--dry-mass-kg={HISTORICAL_FULLSCALE_DRY_MASS_KG}",
        f"--cp-offset-m={HISTORICAL_FULLSCALE_CP_OFFSET_M}",
        f"--moment-of-inertia-kgm2={HISTORICAL_FULLSCALE_MOI_KGM2}",
    ]
    if ignore_logged_state:
        command.append("--ignore-logged-state")
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


def compute_baro_peak_ft(logged_samples) -> float:
    return max(
        sample.baro_agl_ft
        for sample in logged_samples
        if sample.baro_agl_ft is not None and math.isfinite(sample.baro_agl_ft)
    )


def first_coast_apogee_ft(samples: list[ReplaySample]) -> float | None:
    for sample in samples:
        if sample.status == "coast":
            return sample.apogee_ft
    return None


def altitude_near_time_ft(samples: list[ReplaySample], target_time_s: float, tolerance_s: float = 0.02) -> float | None:
    candidates = [sample.altitude_ft for sample in samples if abs(sample.time_s - target_time_s) <= tolerance_s]
    if not candidates:
        return None
    return candidates[-1]


def write_summary_csv(
    output_csv: Path,
    logged_samples,
    seeded_samples: list[ReplaySample],
    lsm_track_samples: list[ReplaySample],
) -> None:
    logged_by_time = {sample.time_s: sample for sample in logged_samples}
    lsm_track_by_time = {sample.time_s: sample for sample in lsm_track_samples}

    with output_csv.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "time_s",
                "status",
                "baro_agl_ft",
                "logged_state_agl_ft",
                "logged_state_apogee_ft",
                "seeded_replay_altitude_ft",
                "seeded_replay_apogee_ft",
                "lsm_track_altitude_ft",
                "lsm_track_apogee_ft",
            ]
        )
        for seeded in seeded_samples:
            logged = logged_by_time.get(seeded.time_s)
            lsm_track = lsm_track_by_time.get(seeded.time_s)
            writer.writerow(
                [
                    seeded.time_s,
                    seeded.status,
                    "" if logged is None else logged.baro_agl_ft,
                    "" if logged is None else logged.state_agl_ft,
                    "" if logged is None else logged.state_apogee_ft,
                    seeded.altitude_ft,
                    seeded.apogee_ft,
                    "" if lsm_track is None else lsm_track.altitude_ft,
                    "" if lsm_track is None else lsm_track.apogee_ft,
                ]
            )


def write_svg(
    output_svg: Path,
    logged_samples,
    seeded_samples: list[ReplaySample],
    lsm_track_samples: list[ReplaySample],
) -> None:
    width = 1500
    height = 1180
    margin_left = 110.0
    panel_width = 1280.0
    panel_height = 260.0

    x_all_min = logged_samples[0].time_s
    x_all_max = logged_samples[-1].time_s
    x_zoom_min = 176.0
    x_zoom_max = 194.0

    baro_points = [(sample.time_s, sample.baro_agl_ft) for sample in logged_samples]
    state_alt_points = [(sample.time_s, sample.state_agl_ft) for sample in logged_samples]
    state_apogee_points = [(sample.time_s, sample.state_apogee_ft) for sample in logged_samples]
    seeded_alt_points = [(sample.time_s, sample.altitude_ft) for sample in seeded_samples]
    seeded_apogee_points = [(sample.time_s, sample.apogee_ft) for sample in seeded_samples]
    lsm_track_alt_points = [(sample.time_s, sample.altitude_ft) for sample in lsm_track_samples]
    lsm_track_apogee_points = [(sample.time_s, sample.apogee_ft) for sample in lsm_track_samples]

    peak_baro_ft = compute_baro_peak_ft(logged_samples)
    overview_alt_max = max(
        value
        for _, value in baro_points + state_alt_points + seeded_alt_points + lsm_track_alt_points
        if value is not None and math.isfinite(value)
    )
    overview_apogee_max = max(
        value
        for _, value in state_apogee_points + seeded_apogee_points + lsm_track_apogee_points
        if value is not None and math.isfinite(value)
    )

    subtitle_lines = [
        f"Historical raw baro apogee: {peak_baro_ft:.1f} ft ({HISTORICAL_BARO_APOGEE_M:.1f} m)",
        f"Seeded replay first coast apogee: {first_coast_apogee_ft(seeded_samples):.1f} ft",
        f"LSM-track first coast apogee: {first_coast_apogee_ft(lsm_track_samples):.1f} ft",
        f"LSM-track altitude near {HISTORICAL_APOGEE_TIME_S:.3f}s: {altitude_near_time_ft(lsm_track_samples, HISTORICAL_APOGEE_TIME_S):.1f} ft",
    ]

    elements = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        f'<text x="{width / 2:.2f}" y="42" text-anchor="middle" font-family="monospace" '
        'font-size="28" font-weight="700" fill="#111827">fullscale_4 LSM-Track Legacy Replay</text>',
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
            x_min=x_all_min,
            x_max=x_all_max,
            y_min=0.0,
            y_max=math.ceil((overview_alt_max + 100.0) / 100.0) * 100.0,
            left=margin_left,
            top=160.0,
            width=panel_width,
            height=panel_height,
            phase_source=logged_samples,
            series=[
                ("Raw Baro AGL", baro_points, "#111827", ""),
                ("Logged State Alt", state_alt_points, "#2563eb", ""),
                ("Seeded Replay Alt", seeded_alt_points, "#dc2626", "8 4"),
                ("LSM-Track Replay Alt", lsm_track_alt_points, "#059669", ""),
            ],
            reference_lines=[],
        )
    )

    elements.append(
        draw_panel(
            title="Altitude AGL Flight Window (ft)",
            x_min=x_zoom_min,
            x_max=x_zoom_max,
            y_min=0.0,
            y_max=math.ceil((overview_alt_max + 100.0) / 100.0) * 100.0,
            left=margin_left,
            top=490.0,
            width=panel_width,
            height=panel_height,
            phase_source=logged_samples,
            series=[
                ("Raw Baro AGL", baro_points, "#111827", ""),
                ("Logged State Alt", state_alt_points, "#2563eb", ""),
                ("Seeded Replay Alt", seeded_alt_points, "#dc2626", "8 4"),
                ("LSM-Track Replay Alt", lsm_track_alt_points, "#059669", ""),
            ],
            reference_lines=[],
        )
    )

    elements.append(
        draw_panel(
            title="Apogee Prediction Flight Window (ft)",
            x_min=x_zoom_min,
            x_max=x_zoom_max,
            y_min=0.0,
            y_max=math.ceil((overview_apogee_max + 100.0) / 100.0) * 100.0,
            left=margin_left,
            top=820.0,
            width=panel_width,
            height=panel_height - 40.0,
            phase_source=logged_samples,
            series=[
                ("Logged State Apogee", state_apogee_points, "#1d4ed8", ""),
                ("Seeded Replay Apogee", seeded_apogee_points, "#dc2626", "8 4"),
                ("LSM-Track Replay Apogee", lsm_track_apogee_points, "#059669", ""),
            ],
            reference_lines=[("Baro AGL Peak", peak_baro_ft, "#7c3aed")],
        )
    )

    elements.append("</svg>")
    output_svg.write_text("\n".join(elements))


def main() -> int:
    input_csv = DEFAULT_INPUT_CSV
    output_svg = DEFAULT_OUTPUT_SVG
    output_csv = DEFAULT_OUTPUT_CSV

    replay_binary = find_replay_binary(None)
    logged_samples = load_logged_samples(input_csv)
    seeded_samples = run_replay(
        replay_binary,
        input_csv,
        ignore_logged_state=False,
        rebuild_main_quaternion=False,
    )

    preset = PRESETS_BY_NAME[DEFAULT_PRESET]
    with tempfile.NamedTemporaryFile(
        suffix=f"_{preset.name}.csv",
        prefix=f"{input_csv.stem}_legacy_plot_",
        dir="/tmp",
        delete=False,
    ) as handle:
        transformed_csv = Path(handle.name)
    try:
        write_transformed_csv(input_csv, preset, transformed_csv)
        lsm_track_samples = run_replay(
            replay_binary,
            transformed_csv,
            ignore_logged_state=True,
            rebuild_main_quaternion=True,
        )
    finally:
        try:
            os.unlink(transformed_csv)
        except OSError:
            pass

    write_summary_csv(output_csv, logged_samples, seeded_samples, lsm_track_samples)
    write_svg(output_svg, logged_samples, seeded_samples, lsm_track_samples)

    print(f"Wrote {output_csv}")
    print(f"Wrote {output_svg}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
