#!/usr/bin/env python3
"""
Apply a legacy body-frame remap to a historical replay CSV, then run acs_replay.

This is for older logs like `fullscale_4.csv` where raw accel/gyro/quaternion
fields do not line up with the current estimator body-frame convention.

It supports a few empirically useful presets and can also sweep them to compare:
  - first non-ground transition
  - first coast apogee prediction
  - altitude near historical apogee time
  - peak predicted apogee

The goal is not to silently "fix" old logs. The goal is to make the remap
explicit and reproducible while we validate which historical convention is
closest to the current filter.
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
from typing import Iterable


from replaylib.paths import ROOT
DEFAULT_INPUT = ROOT / "tools" / "replay" / "scripts" / "fullscale_4.csv"
DEFAULT_REPLAY = ROOT / "tools" / "build" / "bin" / "acs_replay"

HISTORICAL_FULLSCALE_DRY_MASS_KG = 18.24
HISTORICAL_FULLSCALE_CP_OFFSET_M = 0.4437
HISTORICAL_FULLSCALE_MOI_KGM2 = 8.28
HISTORICAL_BARO_APOGEE_M = 867.881
HISTORICAL_APOGEE_TIME_S = 191.154


@dataclass(frozen=True)
class Preset:
    name: str
    description: str
    rotation: tuple[tuple[int, int, int], tuple[int, int, int], tuple[int, int, int]]
    quaternion_mode: str
    force_main_source: int | None = None


PRESETS: tuple[Preset, ...] = (
    Preset(
        name="pure-basis-a",
        description="Proper body-basis remap, conservative. Leaves ground and stays low.",
        rotation=((0, 0, 1), (0, -1, 0), (1, 0, 0)),
        quaternion_mode="R_old_Mt",
    ),
    Preset(
        name="pure-basis-b",
        description="Alternate proper body-basis remap, conservative. Leaves ground and stays lower.",
        rotation=((0, 0, 1), (0, 1, 0), (-1, 0, 0)),
        quaternion_mode="R_old_Mt",
    ),
    Preset(
        name="best-coast",
        description="Best first-coast match found so far. Coast seed is close; later altitude drifts high.",
        rotation=((0, 0, 1), (-1, 0, 0), (0, -1, 0)),
        quaternion_mode="M_R_old_Mt",
    ),
    Preset(
        name="best-coast-alt",
        description="Alternate near-coast match found so far. Similar divergence later in flight.",
        rotation=((0, 0, 1), (-1, 0, 0), (0, -1, 0)),
        quaternion_mode="Mt_R_old",
    ),
    Preset(
        name="lsm-track",
        description="Force rebuilt main quaternion to transformed LSM. Tracks historical altitude far better near apogee, but coast apogee call stays low.",
        rotation=((0, 0, 1), (-1, 0, 0), (0, -1, 0)),
        quaternion_mode="Mt_R_old",
        force_main_source=3,
    ),
)

PRESETS_BY_NAME = {preset.name: preset for preset in PRESETS}


VECTOR_FIELDS = (
    ("sensor_accel_icm_x", "sensor_accel_icm_y", "sensor_accel_icm_z"),
    ("sensor_gyro_x", "sensor_gyro_y", "sensor_gyro_z"),
    ("sensor_accel_lsm_x", "sensor_accel_lsm_y", "sensor_accel_lsm_z"),
    ("sensor_gyro_lsm_x", "sensor_gyro_lsm_y", "sensor_gyro_lsm_z"),
)

QUATERNION_FIELDS = (
    ("sensor_quat_w", "sensor_quat_x", "sensor_quat_y", "sensor_quat_z"),
    ("sensor_icm_quat_w", "sensor_icm_quat_x", "sensor_icm_quat_y", "sensor_icm_quat_z"),
    ("sensor_lsm_quat_w", "sensor_lsm_quat_x", "sensor_lsm_quat_y", "sensor_lsm_quat_z"),
)


def transpose(matrix: tuple[tuple[int, int, int], tuple[int, int, int], tuple[int, int, int]]) -> list[list[float]]:
    return [[float(matrix[row][col]) for row in range(3)] for col in range(3)]


def as_float_matrix(
    matrix: tuple[tuple[int, int, int], tuple[int, int, int], tuple[int, int, int]]
) -> list[list[float]]:
    return [[float(value) for value in row] for row in matrix]


def matmul(lhs: list[list[float]], rhs: list[list[float]]) -> list[list[float]]:
    return [[sum(lhs[row][k] * rhs[k][col] for k in range(3)) for col in range(3)] for row in range(3)]


def apply_matrix(matrix: list[list[float]], vector: Iterable[float]) -> list[float]:
    values = list(vector)
    return [sum(matrix[row][k] * values[k] for k in range(3)) for row in range(3)]


def quat_to_matrix(w: float, x: float, y: float, z: float) -> list[list[float]] | None:
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1.0e-12:
        return None
    w /= norm
    x /= norm
    y /= norm
    z /= norm
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]


def matrix_to_quat(matrix: list[list[float]]) -> tuple[float, float, float, float]:
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (matrix[2][1] - matrix[1][2]) / s
        y = (matrix[0][2] - matrix[2][0]) / s
        z = (matrix[1][0] - matrix[0][1]) / s
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        s = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        w = (matrix[2][1] - matrix[1][2]) / s
        x = 0.25 * s
        y = (matrix[0][1] + matrix[1][0]) / s
        z = (matrix[0][2] + matrix[2][0]) / s
    elif matrix[1][1] > matrix[2][2]:
        s = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        w = (matrix[0][2] - matrix[2][0]) / s
        x = (matrix[0][1] + matrix[1][0]) / s
        y = 0.25 * s
        z = (matrix[1][2] + matrix[2][1]) / s
    else:
        s = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        w = (matrix[1][0] - matrix[0][1]) / s
        x = (matrix[0][2] + matrix[2][0]) / s
        y = (matrix[1][2] + matrix[2][1]) / s
        z = 0.25 * s
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    return (w / norm, x / norm, y / norm, z / norm)


def remap_quaternion(
    rotation_matrix: list[list[float]],
    quaternion_mode: str,
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float, float] | None:
    rotation_old = quat_to_matrix(*quaternion)
    if rotation_old is None:
        return None
    rotation_matrix_t = transpose(tuple(tuple(int(value) for value in row) for row in rotation_matrix))
    if quaternion_mode == "R_old_Mt":
        rotation_new = matmul(rotation_old, rotation_matrix_t)
    elif quaternion_mode == "M_R_old":
        rotation_new = matmul(rotation_matrix, rotation_old)
    elif quaternion_mode == "M_R_old_Mt":
        rotation_new = matmul(matmul(rotation_matrix, rotation_old), rotation_matrix_t)
    elif quaternion_mode == "Mt_R_old":
        rotation_new = matmul(rotation_matrix_t, rotation_old)
    else:
        raise ValueError(f"unsupported quaternion mode: {quaternion_mode}")
    return matrix_to_quat(rotation_new)


@dataclass
class ReplaySummary:
    first_non_ground_time_s: float | None
    first_non_ground_status: str | None
    first_coast_apogee_m: float | None
    altitude_near_apogee_m: float | None
    peak_apogee_m: float | None
    replay_stdout: str


def run_replay(
    replay_binary: Path,
    input_csv: Path,
    dry_mass_kg: float,
    cp_offset_m: float,
    moi_kgm2: float,
) -> ReplaySummary:
    result = subprocess.run(
        [
            str(replay_binary),
            str(input_csv),
            "--ignore-logged-state",
            "--rebuild-main-quaternion",
            f"--dry-mass-kg={dry_mass_kg}",
            f"--cp-offset-m={cp_offset_m}",
            f"--moment-of-inertia-kgm2={moi_kgm2}",
        ],
        cwd=str(ROOT),
        text=True,
        capture_output=True,
        check=False,
    )
    text = result.stdout
    first_non_ground_time_s: float | None = None
    first_non_ground_status: str | None = None
    first_coast_apogee_m: float | None = None
    altitude_near_apogee_m: float | None = None
    peak_apogee_m: float | None = None
    for line in text.splitlines():
        if not line or line.startswith("time_s,"):
            continue
        if line.startswith("Samples processed:"):
            break
        parts = line.split(",")
        if len(parts) < 5:
            continue
        try:
            time_s = float(parts[0])
            altitude_m = float(parts[1])
            apogee_prediction_m = float(parts[3])
        except ValueError:
            continue
        status = parts[4].strip().lower()
        if status != "ground" and first_non_ground_time_s is None:
            first_non_ground_time_s = time_s
            first_non_ground_status = status
        if status == "coast" and first_coast_apogee_m is None:
            first_coast_apogee_m = apogee_prediction_m
        if abs(time_s - HISTORICAL_APOGEE_TIME_S) <= 0.02:
            altitude_near_apogee_m = altitude_m
        if peak_apogee_m is None or apogee_prediction_m > peak_apogee_m:
            peak_apogee_m = apogee_prediction_m
    return ReplaySummary(
        first_non_ground_time_s=first_non_ground_time_s,
        first_non_ground_status=first_non_ground_status,
        first_coast_apogee_m=first_coast_apogee_m,
        altitude_near_apogee_m=altitude_near_apogee_m,
        peak_apogee_m=peak_apogee_m,
        replay_stdout=text,
    )


def write_transformed_csv(input_csv: Path, preset: Preset, output_csv: Path) -> None:
    rotation_matrix = as_float_matrix(preset.rotation)
    with input_csv.open(newline="") as handle_in, output_csv.open("w", newline="") as handle_out:
        reader = csv.DictReader(handle_in)
        if reader.fieldnames is None:
            raise SystemExit(f"Missing CSV header in {input_csv}")
        writer = csv.DictWriter(handle_out, fieldnames=reader.fieldnames)
        writer.writeheader()
        for raw in reader:
            row = dict(raw)
            if preset.force_main_source is not None:
                row["sensor_main_quaternion_source"] = str(preset.force_main_source)
            for x_key, y_key, z_key in VECTOR_FIELDS:
                if x_key not in row or y_key not in row or z_key not in row:
                    continue
                try:
                    vector = [float(row[x_key]), float(row[y_key]), float(row[z_key])]
                except ValueError:
                    continue
                transformed = apply_matrix(rotation_matrix, vector)
                row[x_key], row[y_key], row[z_key] = [f"{value:.9g}" for value in transformed]
            for w_key, x_key, y_key, z_key in QUATERNION_FIELDS:
                if any(key not in row for key in (w_key, x_key, y_key, z_key)):
                    continue
                try:
                    quat = (float(row[w_key]), float(row[x_key]), float(row[y_key]), float(row[z_key]))
                except ValueError:
                    continue
                transformed_quat = remap_quaternion(rotation_matrix, preset.quaternion_mode, quat)
                if transformed_quat is None:
                    continue
                row[w_key], row[x_key], row[y_key], row[z_key] = [f"{value:.9g}" for value in transformed_quat]
            writer.writerow(row)


def resolve_replay_binary(path: str | None) -> Path:
    if path:
        binary = Path(path)
    else:
        binary = DEFAULT_REPLAY
    if not binary.is_file():
        raise SystemExit(
            f"Could not find acs_replay at {binary}. Build it with "
            "`cmake --build tools/build --target acs_replay` first."
        )
    return binary


def default_output_path(input_csv: Path, preset_name: str) -> Path:
    return input_csv.with_name(f"{input_csv.stem}_{preset_name}_legacy_adapter.csv")


def print_summary(name: str, summary: ReplaySummary) -> None:
    def fmt(value: float | None) -> str:
        return "n/a" if value is None else f"{value:.3f}"

    print(
        f"{name}: first_non_ground={fmt(summary.first_non_ground_time_s)}"
        f"/{summary.first_non_ground_status or 'n/a'} "
        f"first_coast_apogee={fmt(summary.first_coast_apogee_m)} m "
        f"alt_at_{HISTORICAL_APOGEE_TIME_S:.3f}s={fmt(summary.altitude_near_apogee_m)} m "
        f"peak_apogee={fmt(summary.peak_apogee_m)} m"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_csv", nargs="?", default=str(DEFAULT_INPUT))
    parser.add_argument("--replay-binary", default=str(DEFAULT_REPLAY))
    parser.add_argument("--preset", choices=sorted(PRESETS_BY_NAME), default="best-coast")
    parser.add_argument("--output-csv", help="Where to write the transformed CSV.")
    parser.add_argument("--sweep-presets", action="store_true", help="Run all built-in presets and print a summary table.")
    parser.add_argument("--dry-mass-kg", type=float, default=HISTORICAL_FULLSCALE_DRY_MASS_KG)
    parser.add_argument("--cp-offset-m", type=float, default=HISTORICAL_FULLSCALE_CP_OFFSET_M)
    parser.add_argument("--moment-of-inertia-kgm2", type=float, default=HISTORICAL_FULLSCALE_MOI_KGM2)
    args = parser.parse_args()

    input_csv = Path(args.input_csv)
    replay_binary = resolve_replay_binary(args.replay_binary)

    if args.sweep_presets:
        for preset in PRESETS:
            with tempfile.NamedTemporaryFile(
                suffix=f"_{preset.name}.csv",
                prefix=f"{input_csv.stem}_legacy_",
                dir="/tmp",
                delete=False,
            ) as handle:
                temp_path = Path(handle.name)
            try:
                write_transformed_csv(input_csv, preset, temp_path)
                summary = run_replay(
                    replay_binary,
                    temp_path,
                    args.dry_mass_kg,
                    args.cp_offset_m,
                    args.moment_of_inertia_kgm2,
                )
                print_summary(preset.name, summary)
            finally:
                try:
                    os.unlink(temp_path)
                except OSError:
                    pass
        print(f"reference_baro_apogee={HISTORICAL_BARO_APOGEE_M:.3f} m")
        return 0

    preset = PRESETS_BY_NAME[args.preset]
    output_csv = Path(args.output_csv) if args.output_csv else default_output_path(input_csv, preset.name)
    write_transformed_csv(input_csv, preset, output_csv)
    summary = run_replay(
        replay_binary,
        output_csv,
        args.dry_mass_kg,
        args.cp_offset_m,
        args.moment_of_inertia_kgm2,
    )
    print_summary(preset.name, summary)
    print(f"wrote {output_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
