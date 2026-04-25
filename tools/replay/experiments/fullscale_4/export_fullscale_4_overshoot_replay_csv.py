#!/usr/bin/env python3
"""Export fullscale_4 hosted replay rows with overshoot detection and flap rails."""

from __future__ import annotations

import argparse
import bisect
import csv
import math
import os
import subprocess
import tempfile
from pathlib import Path

from plot_fullscale_4_overshoot_actuation import DEFAULT_APOGEE_TARGET_M
from plot_fullscale_4_validation import DEFAULT_INPUT_CSV
from plot_fullscale_4_validation import find_replay_binary
from plot_fullscale_4_validation import parse_float
from plot_fullscale_4_validation import sanitize_filename
from replay_legacy_frame_adapter import HISTORICAL_FULLSCALE_CP_OFFSET_M
from replay_legacy_frame_adapter import HISTORICAL_FULLSCALE_DRY_MASS_KG
from replay_legacy_frame_adapter import HISTORICAL_FULLSCALE_MOI_KGM2
from replay_legacy_frame_adapter import PRESETS_BY_NAME
from replay_legacy_frame_adapter import write_transformed_csv


from replaylib.paths import SCRIPTS_DIR
SCRIPT_DIR = SCRIPTS_DIR
ROOT = SCRIPT_DIR.parents[2]
DEFAULT_OUTPUT_CSV = ROOT / "tools" / "replay" / "data" / "fullscale_4_overshoot_replay.csv"
FEET_TO_METERS = 0.3048


def load_logged_samples(input_csv: Path) -> list[tuple[float, dict[str, float | str | None]]]:
    rows: list[dict[str, str]] = []
    with input_csv.open(newline="") as handle:
        reader = csv.DictReader(line for line in handle if not line.startswith("#"))
        rows.extend(reader)

    pad_altitude_ft: float | None = None
    for row in rows:
        altitude_ft = parse_float(row.get("sensor_altitude_feet"))
        if altitude_ft is not None and altitude_ft != 0.0:
            pad_altitude_ft = altitude_ft
            break
    if pad_altitude_ft is None:
        raise SystemExit(f"Could not determine pad altitude from {input_csv}")

    samples: list[tuple[float, dict[str, float | str | None]]] = []
    for row in rows:
        time_s = parse_float(row.get("sensor_timestamp"))
        if time_s is None:
            continue
        altitude_ft = parse_float(row.get("sensor_altitude_feet"))
        baro_agl_ft = None if altitude_ft is None else max(0.0, altitude_ft - pad_altitude_ft)
        samples.append((time_s, {
            "logged_status": (row.get("flight_status") or "").strip().lower(),
            "baro_agl_m": None if baro_agl_ft is None else baro_agl_ft * FEET_TO_METERS,
            "logged_state_agl_m": None
            if parse_float(row.get("state_altitude_agl_feet")) is None
            else parse_float(row.get("state_altitude_agl_feet")) * FEET_TO_METERS,
            "logged_state_apogee_m": None
            if parse_float(row.get("state_apogee_estimate_feet")) is None
            else parse_float(row.get("state_apogee_estimate_feet")) * FEET_TO_METERS,
            "logged_flap_command_deg": parse_float(row.get("sensor_flap_command_deg")),
            "logged_flap_effective_deg": parse_float(row.get("sensor_flap_effective_deg")),
            "logged_actuation_is_settling": parse_float(row.get("sensor_actuation_is_settling")),
        }))
    return samples


def nearest_logged_sample(
    samples: list[tuple[float, dict[str, float | str | None]]],
    times: list[float],
    time_s: float,
    tolerance_s: float = 0.002,
) -> dict[str, float | str | None]:
    index = bisect.bisect_left(times, time_s)
    candidates = []
    if index < len(samples):
        candidates.append(samples[index])
    if index > 0:
        candidates.append(samples[index - 1])
    if not candidates:
        return {}
    best_time, best_sample = min(candidates, key=lambda item: abs(item[0] - time_s))
    if abs(best_time - time_s) > tolerance_s:
        return {}
    return best_sample


def run_replay_stdout(replay_binary: Path, input_csv: Path, preset_name: str, apogee_target_m: float) -> str:
    preset = PRESETS_BY_NAME[preset_name]
    with tempfile.NamedTemporaryFile(
        suffix=f"_{sanitize_filename(preset.name)}.csv",
        prefix=f"{sanitize_filename(input_csv.stem)}_overshoot_export_",
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
            "--include-raw=altitude_agl_m,apogee_error_m,altimeter_raw_m",
        ]
        result = subprocess.run(command, cwd=str(ROOT), text=True, capture_output=True, check=False)
    finally:
        try:
            os.unlink(temp_csv)
        except OSError:
            pass
    if result.returncode != 0:
        raise SystemExit(result.stderr.strip() or result.stdout.strip() or "acs_replay failed")
    return result.stdout


def replay_rows(stdout: str) -> list[dict[str, str]]:
    lines = stdout.splitlines()
    try:
        header_index = next(index for index, line in enumerate(lines) if line.startswith("time_s,"))
    except StopIteration as exc:
        raise SystemExit("acs_replay did not emit replay CSV output") from exc

    csv_lines: list[str] = []
    for line in lines[header_index:]:
        if line.startswith("Samples processed:"):
            break
        if line.strip():
            csv_lines.append(line)
    rows = list(csv.DictReader(csv_lines))
    if not rows:
        raise SystemExit("acs_replay emitted no usable rows")
    return rows


def fmt(value: float | str | None) -> str:
    if value is None:
        return ""
    if isinstance(value, str):
        return value
    if not math.isfinite(value):
        return ""
    return f"{value:.9g}"


def write_export_csv(
    output_csv: Path,
    input_csv: Path,
    replay_binary: Path,
    preset_name: str,
    apogee_target_m: float,
) -> tuple[float | None, int, float | None]:
    logged_samples = load_logged_samples(input_csv)
    logged_times = [sample_time for sample_time, _ in logged_samples]
    rows = replay_rows(run_replay_stdout(replay_binary, input_csv, preset_name, apogee_target_m))
    output_csv.parent.mkdir(parents=True, exist_ok=True)

    overshoot_detection_time: float | None = None
    max_logged_flap_in_overshoot: float | None = None
    previous_status = ""
    emitted = 0

    headers = [
        "time_s",
        "replay_status",
        "overshoot_detected_this_row",
        "seconds_since_overshoot",
        "replay_altitude_m",
        "replay_altitude_agl_m",
        "replay_velocity_mps",
        "replay_apogee_prediction_m",
        "replay_apogee_error_m",
        "apogee_target_m",
        "absolute_baro_at_or_above_target",
        "replay_agl_at_or_above_target",
        "altimeter_raw_m",
        "baro_agl_m",
        "replay_altitude_minus_baro_m",
        "logged_status",
        "logged_state_agl_m",
        "logged_state_apogee_m",
        "logged_flap_command_deg",
        "logged_flap_effective_deg",
        "logged_actuation_is_settling",
        "logged_flap_command_is_45_deg",
        "forced_overshoot_flap_command_deg",
    ]

    with output_csv.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(headers)
        for row in rows:
            time_s = parse_float(row.get("time_s"))
            if time_s is None:
                continue
            status = (row.get("status") or "").strip().lower()
            replay_altitude_m = parse_float(row.get("altitude_m"))
            replay_altitude_agl_m = parse_float(row.get("altitude_agl_m"))
            replay_velocity_mps = parse_float(row.get("velocity_mps"))
            replay_apogee_m = parse_float(row.get("apogee_prediction_m"))
            replay_apogee_error_m = parse_float(row.get("apogee_error_m"))
            altimeter_raw_m = parse_float(row.get("altimeter_raw_m"))
            logged = nearest_logged_sample(logged_samples, logged_times, time_s)
            baro_agl_m = logged.get("baro_agl_m")
            flap_command = logged.get("logged_flap_command_deg")
            flap_effective = logged.get("logged_flap_effective_deg")

            detected_this_row = status == "overshoot" and previous_status != "overshoot"
            if detected_this_row and overshoot_detection_time is None:
                overshoot_detection_time = time_s
            seconds_since_overshoot = None if overshoot_detection_time is None else time_s - overshoot_detection_time
            replay_minus_baro = (
                None
                if replay_altitude_agl_m is None or not isinstance(baro_agl_m, float)
                else replay_altitude_agl_m - baro_agl_m
            )
            command_is_45 = (
                isinstance(flap_command, float) and
                abs(flap_command - 45.0) <= 0.25
            )
            forced_overshoot_command = 45.0 if status == "overshoot" else None
            if status == "overshoot" and isinstance(flap_command, float):
                max_logged_flap_in_overshoot = (
                    flap_command
                    if max_logged_flap_in_overshoot is None
                    else max(max_logged_flap_in_overshoot, flap_command)
                )

            writer.writerow(
                [
                    fmt(time_s),
                    status,
                    int(detected_this_row),
                    fmt(seconds_since_overshoot),
                    fmt(replay_altitude_m),
                    fmt(replay_altitude_agl_m),
                    fmt(replay_velocity_mps),
                    fmt(replay_apogee_m),
                    fmt(replay_apogee_error_m),
                    fmt(apogee_target_m),
                    int(altimeter_raw_m is not None and altimeter_raw_m >= apogee_target_m),
                    int(replay_altitude_agl_m is not None and replay_altitude_agl_m >= apogee_target_m),
                    fmt(altimeter_raw_m),
                    fmt(baro_agl_m),
                    fmt(replay_minus_baro),
                    fmt(logged.get("logged_status")),
                    fmt(logged.get("logged_state_agl_m")),
                    fmt(logged.get("logged_state_apogee_m")),
                    fmt(flap_command),
                    fmt(flap_effective),
                    fmt(logged.get("logged_actuation_is_settling")),
                    int(command_is_45),
                    fmt(forced_overshoot_command),
                ]
            )
            previous_status = status
            emitted += 1

    return overshoot_detection_time, emitted, max_logged_flap_in_overshoot


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_csv", nargs="?", type=Path, default=DEFAULT_INPUT_CSV)
    parser.add_argument("--replay-binary", type=Path, default=None)
    parser.add_argument("--preset", choices=sorted(PRESETS_BY_NAME), default="best-coast")
    parser.add_argument("--apogee-target-m", type=float, default=DEFAULT_APOGEE_TARGET_M)
    parser.add_argument("--output-csv", type=Path, default=DEFAULT_OUTPUT_CSV)
    args = parser.parse_args()

    replay_binary = find_replay_binary(args.replay_binary)
    overshoot_time, rows, max_flap = write_export_csv(
        args.output_csv,
        args.input_csv,
        replay_binary,
        args.preset,
        args.apogee_target_m,
    )
    print(f"Input CSV: {args.input_csv}")
    print(f"Replay binary: {replay_binary}")
    print(f"Preset: {args.preset}")
    print(f"Apogee target: {args.apogee_target_m:.2f} m")
    print(f"Rows written: {rows}")
    print(f"Overshoot detected: {'none' if overshoot_time is None else f'{overshoot_time:.3f}s'}")
    print(f"Max logged flap command while replay status is overshoot: {fmt(max_flap)} deg")
    print(f"Wrote {args.output_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
