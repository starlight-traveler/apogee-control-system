#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def normalize(q: list[float]) -> list[float]:
    norm = math.sqrt(sum(v * v for v in q))
    if not math.isfinite(norm) or norm < 1.0e-12:
        return [1.0, 0.0, 0.0, 0.0]
    return [v / norm for v in q]


def dot(a: list[float], b: list[float]) -> float:
    a = normalize(a)
    b = normalize(b)
    return sum(x * y for x, y in zip(a, b))


def slerp(a: list[float], b: list[float], t: float) -> list[float]:
    a = normalize(a)
    b = normalize(b)
    d = dot(a, b)
    if d < 0.0:
        b = [-v for v in b]
        d = -d
    if d > 0.9995:
        return normalize([(1.0 - t) * x + t * y for x, y in zip(a, b)])
    theta_0 = math.acos(max(-1.0, min(1.0, d)))
    sin_theta_0 = math.sin(theta_0)
    scale_a = math.sin((1.0 - t) * theta_0) / sin_theta_0
    scale_b = math.sin(t * theta_0) / sin_theta_0
    return [scale_a * x + scale_b * y for x, y in zip(a, b)]


def quat_to_euler_rad(q: list[float]) -> tuple[float, float, float]:
    w, x, y, z = normalize(q)
    r11 = 2.0 * w * w - 1.0 + 2.0 * x * x
    r21 = 2.0 * (x * y - w * z)
    r31 = 2.0 * (x * z + w * y)
    r32 = 2.0 * (y * z - w * x)
    r33 = 2.0 * w * w - 1.0 + 2.0 * z * z

    roll = math.atan2(r32, r33)
    denom = max(0.0, 1.0 - r31 * r31)
    root = math.sqrt(denom)
    if root != 0.0:
        pitch = -math.atan(r31 / root)
    else:
        pitch = (-1.0 if r31 >= 0.0 else 1.0) * (math.pi * 0.5)
    yaw = math.atan2(r21, r11)
    return yaw, pitch, roll


def quat_from_pitch_roll_deg(pitch_deg: float, roll_deg: float) -> list[float]:
    half_pitch = math.radians(pitch_deg) * 0.5
    half_roll = math.radians(roll_deg) * 0.5
    sin_pitch = math.sin(half_pitch)
    cos_pitch = math.cos(half_pitch)
    sin_roll = math.sin(half_roll)
    cos_roll = math.cos(half_roll)
    return normalize(
        [
            cos_pitch * cos_roll,
            -cos_pitch * sin_roll,
            -sin_pitch * cos_roll,
            -sin_pitch * sin_roll,
        ]
    )


def tilt_quaternion_from_quaternion(q: list[float]) -> list[float]:
    _, pitch_rad, roll_rad = quat_to_euler_rad(q)
    return quat_from_pitch_roll_deg(math.degrees(pitch_rad), math.degrees(roll_rad))


def euler_to_zenith_deg(pitch_rad: float, roll_rad: float) -> float:
    value = max(-1.0, min(1.0, math.cos(pitch_rad) * math.cos(roll_rad)))
    return math.degrees(math.acos(abs(value)))


def compute_source_label(code: int) -> str:
    return {
        0: "None",
        1: "BNO",
        2: "ICM",
        3: "LSM",
        4: "Blended",
    }.get(code, str(code))


def parse_quat(row: dict[str, str], prefix: str) -> list[float]:
    return [float(row[f"{prefix}_{axis}"]) for axis in "wxyz"]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input_csv",
        nargs="?",
        type=Path,
        default=Path(__file__).with_name("fullscale_4.csv"),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).with_name("fullscale_4_recalculated_zenith.csv"),
    )
    args = parser.parse_args()

    rows = list(csv.DictReader(args.input_csv.open(newline="", encoding="utf-8")))
    base_fields = list(rows[0].keys())
    extra_fields = [
        "recalc_main_quat_w",
        "recalc_main_quat_x",
        "recalc_main_quat_y",
        "recalc_main_quat_z",
        "recalc_main_source_label",
        "recalc_state_zenith_deg",
        "recalc_state_zenith_delta_deg",
    ]

    recalculated_zeniths: list[float] = []
    deltas: list[float] = []

    with args.output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=base_fields + extra_fields)
        writer.writeheader()

        for row in rows:
            source_code = int(float(row["sensor_main_quaternion_source"]))
            icm_quat = tilt_quaternion_from_quaternion(parse_quat(row, "sensor_icm_quat"))
            lsm_raw_quat = parse_quat(row, "sensor_lsm_quat")
            lsm_tilt_quat = tilt_quaternion_from_quaternion(lsm_raw_quat)

            if source_code == 4:
                selected = slerp(icm_quat, lsm_tilt_quat, 0.5)
            elif source_code == 3:
                selected = lsm_tilt_quat
            elif source_code == 2:
                selected = icm_quat
            else:
                selected = tilt_quaternion_from_quaternion(parse_quat(row, "sensor_quat"))

            final_main_quat = tilt_quaternion_from_quaternion(selected)
            _, pitch_rad, roll_rad = quat_to_euler_rad(final_main_quat)
            recalculated_zenith_deg = euler_to_zenith_deg(pitch_rad, roll_rad)
            original_zenith_deg = float(row["state_zenith_deg"])
            delta_deg = recalculated_zenith_deg - original_zenith_deg

            row["recalc_main_quat_w"] = f"{final_main_quat[0]:.9f}"
            row["recalc_main_quat_x"] = f"{final_main_quat[1]:.9f}"
            row["recalc_main_quat_y"] = f"{final_main_quat[2]:.9f}"
            row["recalc_main_quat_z"] = f"{final_main_quat[3]:.9f}"
            row["recalc_main_source_label"] = compute_source_label(source_code)
            row["recalc_state_zenith_deg"] = f"{recalculated_zenith_deg:.9f}"
            row["recalc_state_zenith_delta_deg"] = f"{delta_deg:.9f}"
            writer.writerow(row)

            recalculated_zeniths.append(recalculated_zenith_deg)
            deltas.append(delta_deg)

    print(f"Wrote {args.output}")
    print(f"Rows: {len(rows)}")
    print(f"Mean recalculated zenith: {sum(recalculated_zeniths) / len(recalculated_zeniths):.6f} deg")
    print(f"Mean zenith delta: {sum(deltas) / len(deltas):.6f} deg")
    print(f"Max zenith delta: {max(deltas):.6f} deg")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
