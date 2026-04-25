#!/usr/bin/env python3

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path


from replaylib.paths import SCRIPTS_DIR
SCRIPT_DIR = SCRIPTS_DIR
REPLAY_DIR = SCRIPT_DIR.parent
ROOT = REPLAY_DIR.parents[1]
DATA_DIR = REPLAY_DIR / "data"
PLOTS_DIR = REPLAY_DIR / "plots"

DEFAULT_INPUT_CSV = ROOT / "fullscale_3.csv"
DEFAULT_RECOVERY_CSV = DATA_DIR / "BlRv_LASS_alt HR_10-12-2025_16_01_16.csv"
DEFAULT_OUTPUT_CSV = DATA_DIR / "fullscale_3_attitude_corrected.csv"
DEFAULT_OUTPUT_JSON = DATA_DIR / "fullscale_3_attitude_calibration_summary.json"
DEFAULT_OUTPUT_SVG = PLOTS_DIR / "fullscale_3_attitude_calibrated_compare.svg"


@dataclass
class AcsSample:
    time_s: float
    status: str
    icm_quat: tuple[float, float, float, float] | None
    lsm_quat: tuple[float, float, float, float] | None


@dataclass
class RecoverySample:
    flight_time_s: float
    quat: tuple[float, float, float, float]


@dataclass
class SensorCalibration:
    name: str
    variant: str
    mount_quat: tuple[float, float, float, float]
    pair_count: int
    body_z_rms_before_deg: float
    body_z_rms_after_deg: float
    pitch_rms_before_deg: float
    pitch_rms_after_deg: float
    roll_rms_before_deg: float
    roll_rms_after_deg: float


VARIANTS = ("as_is", "conjugate")


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


def normalize_quat(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(value * value for value in q))
    if norm <= 0.0 or not math.isfinite(norm):
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(value / norm for value in q)


def quat_conjugate(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    return (q[0], -q[1], -q[2], -q[3])


def quat_multiply(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_apply_variant(
    q: tuple[float, float, float, float],
    variant: str,
) -> tuple[float, float, float, float]:
    if variant == "conjugate":
        return quat_conjugate(q)
    return q


def quat_to_pitch_roll_deg(q: tuple[float, float, float, float]) -> tuple[float, float]:
    w, x, y, z = normalize_quat(q)
    r31 = 2.0 * (x * z + w * y)
    r32 = 2.0 * (y * z - w * x)
    r33 = 2.0 * w * w - 1.0 + 2.0 * z * z

    roll = math.atan2(r32, r33)
    denom = 1.0 - r31 * r31
    root = math.sqrt(denom) if denom >= 0.0 else math.sqrt(abs(denom))
    if root != 0.0:
        pitch = -math.atan(r31 / root)
    else:
        pitch = (-1.0 if r31 >= 0.0 else 1.0) * (math.pi * 0.5)
    return (math.degrees(pitch), math.degrees(roll))


def quat_to_body_z_axis(q: tuple[float, float, float, float]) -> tuple[float, float, float]:
    w, x, y, z = normalize_quat(q)
    return (
        2.0 * (x * z + w * y),
        2.0 * (y * z - w * x),
        2.0 * w * w - 1.0 + 2.0 * z * z,
    )


def rotate_vector(
    q: tuple[float, float, float, float],
    v: tuple[float, float, float],
) -> tuple[float, float, float]:
    qn = normalize_quat(q)
    pure = (0.0, v[0], v[1], v[2])
    rotated = quat_multiply(quat_multiply(qn, pure), quat_conjugate(qn))
    return (rotated[1], rotated[2], rotated[3])


def angle_between_deg(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    norm_a = math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
    norm_b = math.sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2])
    if norm_a <= 0.0 or norm_b <= 0.0:
        return 180.0
    cosine = max(-1.0, min(1.0, dot / (norm_a * norm_b)))
    return math.degrees(math.acos(cosine))


def wrap_deg(delta_deg: float) -> float:
    wrapped = (delta_deg + 180.0) % 360.0 - 180.0
    return wrapped


def rms(values: list[float]) -> float:
    if not values:
        return math.inf
    return math.sqrt(sum(value * value for value in values) / len(values))


def power_iteration_symmetric(matrix: list[list[float]], iterations: int = 60) -> tuple[float, float, float, float]:
    vector = [1.0, 0.0, 0.0, 0.0]
    for _ in range(iterations):
        next_vector = [
            sum(matrix[row][col] * vector[col] for col in range(4))
            for row in range(4)
        ]
        norm = math.sqrt(sum(value * value for value in next_vector))
        if norm <= 1.0e-12:
            break
        vector = [value / norm for value in next_vector]
    return normalize_quat((vector[0], vector[1], vector[2], vector[3]))


def fit_mount_quat_from_vectors(
    sensor_vectors: list[tuple[float, float, float]],
    recovery_vectors: list[tuple[float, float, float]],
) -> tuple[float, float, float, float]:
    if len(sensor_vectors) != len(recovery_vectors) or not sensor_vectors:
        return (1.0, 0.0, 0.0, 0.0)

    b00 = b01 = b02 = 0.0
    b10 = b11 = b12 = 0.0
    b20 = b21 = b22 = 0.0
    for sensor_vec, recovery_vec in zip(sensor_vectors, recovery_vectors):
        b00 += recovery_vec[0] * sensor_vec[0]
        b01 += recovery_vec[0] * sensor_vec[1]
        b02 += recovery_vec[0] * sensor_vec[2]
        b10 += recovery_vec[1] * sensor_vec[0]
        b11 += recovery_vec[1] * sensor_vec[1]
        b12 += recovery_vec[1] * sensor_vec[2]
        b20 += recovery_vec[2] * sensor_vec[0]
        b21 += recovery_vec[2] * sensor_vec[1]
        b22 += recovery_vec[2] * sensor_vec[2]

    sigma = b00 + b11 + b22
    s00 = 2.0 * b00
    s01 = b01 + b10
    s02 = b02 + b20
    s11 = 2.0 * b11
    s12 = b12 + b21
    s22 = 2.0 * b22
    z0 = b12 - b21
    z1 = b20 - b02
    z2 = b01 - b10
    matrix = [
        [sigma, z0, z1, z2],
        [z0, s00 - sigma, s01, s02],
        [z1, s01, s11 - sigma, s12],
        [z2, s02, s12, s22 - sigma],
    ]
    return power_iteration_symmetric(matrix)


def load_acs_samples(path: Path) -> list[AcsSample]:
    rows: list[AcsSample] = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(line for line in handle if not line.startswith("#"))
        for raw in reader:
            time_s = parse_float(raw.get("sensor_timestamp") or raw.get("state_time") or raw.get("timestamp"))
            if time_s is None:
                continue

            def read_quat(prefix: str, valid_name: str) -> tuple[float, float, float, float] | None:
                if raw.get(valid_name) != "True":
                    return None
                parts = [
                    parse_float(raw.get(f"{prefix}_w")),
                    parse_float(raw.get(f"{prefix}_x")),
                    parse_float(raw.get(f"{prefix}_y")),
                    parse_float(raw.get(f"{prefix}_z")),
                ]
                if any(part is None for part in parts):
                    return None
                return normalize_quat((parts[0], parts[1], parts[2], parts[3]))

            rows.append(
                AcsSample(
                    time_s=time_s,
                    status=(raw.get("flight_status") or "").strip().lower(),
                    icm_quat=read_quat("sensor_icm_quat", "sensor_has_icm_quaternion"),
                    lsm_quat=read_quat("sensor_lsm_quat", "sensor_has_lsm_quaternion"),
                )
            )
    if not rows:
        raise SystemExit(f"No ACS samples found in {path}")
    return rows


def load_recovery_samples(path: Path) -> list[RecoverySample]:
    rows: list[RecoverySample] = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            flight_time_s = parse_float(raw.get("Flight_Time_(s)"))
            q1 = parse_float(raw.get("Quat_1"))
            q2 = parse_float(raw.get("Quat_2"))
            q3 = parse_float(raw.get("Quat_3"))
            q4 = parse_float(raw.get("Quat_4"))
            if None in (flight_time_s, q1, q2, q3, q4):
                continue
            rows.append(
                RecoverySample(
                    flight_time_s=flight_time_s,
                    quat=normalize_quat((q1, q2, q3, q4)),
                )
            )
    if not rows:
        raise SystemExit(f"No recovery samples found in {path}")
    return rows


def flight_window(samples: list[AcsSample], padding_s: float) -> tuple[float, float, float]:
    launch_time_s = next((sample.time_s for sample in samples if sample.status and sample.status != "ground"), samples[0].time_s)
    end_time_s = next((sample.time_s for sample in reversed(samples) if sample.status and sample.status != "ground"), samples[-1].time_s)
    return (launch_time_s - padding_s, end_time_s + padding_s, launch_time_s)


def decimate_sensor_samples(
    samples: list[AcsSample],
    sensor_name: str,
    window_start_s: float,
    window_end_s: float,
    stride: int,
) -> list[tuple[float, tuple[float, float, float, float]]]:
    valid: list[tuple[float, tuple[float, float, float, float]]] = []
    for sample in samples:
        if sample.time_s < window_start_s or sample.time_s > window_end_s:
            continue
        quat = sample.icm_quat if sensor_name == "icm" else sample.lsm_quat
        if quat is None:
            continue
        valid.append((sample.time_s, quat))
    if stride <= 1:
        return valid
    reduced = valid[::stride]
    if reduced and reduced[-1] != valid[-1]:
        reduced.append(valid[-1])
    return reduced


def nearest_recovery_sample(
    recovery_times: list[float],
    recovery_samples: list[RecoverySample],
    target_flight_time_s: float,
    tolerance_s: float,
) -> RecoverySample | None:
    index = bisect.bisect_left(recovery_times, target_flight_time_s)
    candidates: list[RecoverySample] = []
    if index < len(recovery_samples):
        candidates.append(recovery_samples[index])
    if index > 0:
        candidates.append(recovery_samples[index - 1])
    if not candidates:
        return None
    best = min(candidates, key=lambda sample: abs(sample.flight_time_s - target_flight_time_s))
    if abs(best.flight_time_s - target_flight_time_s) > tolerance_s:
        return None
    return best


def build_pairs(
    sensor_samples: list[tuple[float, tuple[float, float, float, float]]],
    recovery_samples: list[RecoverySample],
    recovery_times: list[float],
    launch_time_s: float,
    offset_s: float,
    variant: str,
    tolerance_s: float,
) -> list[tuple[tuple[float, float, float, float], tuple[float, float, float, float]]]:
    pairs: list[tuple[tuple[float, float, float, float], tuple[float, float, float, float]]] = []
    recovery_launch_time_s = launch_time_s + offset_s
    for acs_time_s, quat in sensor_samples:
        recovery_sample = nearest_recovery_sample(
            recovery_times,
            recovery_samples,
            acs_time_s - recovery_launch_time_s,
            tolerance_s,
        )
        if recovery_sample is None:
            continue
        pairs.append((quat_apply_variant(quat, variant), recovery_sample.quat))
    return pairs


def evaluate_pairs(
    pairs: list[tuple[tuple[float, float, float, float], tuple[float, float, float, float]]],
    mount_quat: tuple[float, float, float, float],
) -> tuple[float, float, float, float, float, float]:
    body_before: list[float] = []
    body_after: list[float] = []
    pitch_before: list[float] = []
    pitch_after: list[float] = []
    roll_before: list[float] = []
    roll_after: list[float] = []

    for sensor_quat, recovery_quat in pairs:
        sensor_body_z = quat_to_body_z_axis(sensor_quat)
        recovery_body_z = quat_to_body_z_axis(recovery_quat)
        corrected_sensor_quat = normalize_quat(quat_multiply(mount_quat, sensor_quat))
        corrected_body_z = quat_to_body_z_axis(corrected_sensor_quat)

        sensor_pitch, sensor_roll = quat_to_pitch_roll_deg(sensor_quat)
        corrected_pitch, corrected_roll = quat_to_pitch_roll_deg(corrected_sensor_quat)
        recovery_pitch, recovery_roll = quat_to_pitch_roll_deg(recovery_quat)

        body_before.append(angle_between_deg(sensor_body_z, recovery_body_z))
        body_after.append(angle_between_deg(corrected_body_z, recovery_body_z))
        pitch_before.append(wrap_deg(sensor_pitch - recovery_pitch))
        pitch_after.append(wrap_deg(corrected_pitch - recovery_pitch))
        roll_before.append(wrap_deg(sensor_roll - recovery_roll))
        roll_after.append(wrap_deg(corrected_roll - recovery_roll))

    return (
        rms(body_before),
        rms(body_after),
        rms(pitch_before),
        rms(pitch_after),
        rms(roll_before),
        rms(roll_after),
    )


def calibrate_sensor(
    sensor_name: str,
    sensor_samples: list[tuple[float, tuple[float, float, float, float]]],
    recovery_samples: list[RecoverySample],
    recovery_times: list[float],
    launch_time_s: float,
    offset_s: float,
    tolerance_s: float,
) -> SensorCalibration | None:
    best: SensorCalibration | None = None
    for variant in VARIANTS:
        pairs = build_pairs(sensor_samples, recovery_samples, recovery_times, launch_time_s, offset_s, variant, tolerance_s)
        if len(pairs) < 20:
            continue
        sensor_vectors = [quat_to_body_z_axis(sensor_quat) for sensor_quat, _ in pairs]
        recovery_vectors = [quat_to_body_z_axis(recovery_quat) for _, recovery_quat in pairs]
        mount_quat = fit_mount_quat_from_vectors(sensor_vectors, recovery_vectors)
        metrics = evaluate_pairs(pairs, mount_quat)
        candidate = SensorCalibration(
            name=sensor_name,
            variant=variant,
            mount_quat=mount_quat,
            pair_count=len(pairs),
            body_z_rms_before_deg=metrics[0],
            body_z_rms_after_deg=metrics[1],
            pitch_rms_before_deg=metrics[2],
            pitch_rms_after_deg=metrics[3],
            roll_rms_before_deg=metrics[4],
            roll_rms_after_deg=metrics[5],
        )
        if best is None or candidate.body_z_rms_after_deg < best.body_z_rms_after_deg:
            best = candidate
    return best


def search_time_offset(
    acs_samples: list[AcsSample],
    recovery_samples: list[RecoverySample],
    launch_time_s: float,
    window_start_s: float,
    window_end_s: float,
    tolerance_s: float,
    offset_min_s: float,
    offset_max_s: float,
) -> float:
    recovery_times = [sample.flight_time_s for sample in recovery_samples]
    sensor_sets = {
        "icm": decimate_sensor_samples(acs_samples, "icm", window_start_s, window_end_s, stride=25),
        "lsm": decimate_sensor_samples(acs_samples, "lsm", window_start_s, window_end_s, stride=25),
    }

    def score_offset(offset_s: float) -> float:
        scores: list[float] = []
        for sensor_name in ("icm", "lsm"):
            calibration = calibrate_sensor(
                sensor_name,
                sensor_sets[sensor_name],
                recovery_samples,
                recovery_times,
                launch_time_s,
                offset_s,
                tolerance_s,
            )
            if calibration is not None:
                scores.append(calibration.body_z_rms_after_deg)
        return sum(scores) / len(scores) if scores else math.inf

    coarse_step = 0.05
    coarse_candidates = []
    offset = offset_min_s
    while offset <= offset_max_s + 1.0e-9:
        coarse_candidates.append(offset)
        offset += coarse_step
    best_offset = min(coarse_candidates, key=score_offset)

    fine_candidates = []
    fine_step = 0.002
    fine_min = max(offset_min_s, best_offset - coarse_step)
    fine_max = min(offset_max_s, best_offset + coarse_step)
    offset = fine_min
    while offset <= fine_max + 1.0e-9:
        fine_candidates.append(offset)
        offset += fine_step
    return min(fine_candidates, key=score_offset)


def write_summary(path: Path, offset_s: float, calibrations: list[SensorCalibration]) -> None:
    payload = {
        "time_offset_seconds": offset_s,
        "sensors": {
            calibration.name: {
                "variant": calibration.variant,
                "mount_quaternion_wxyz": list(calibration.mount_quat),
                "pair_count": calibration.pair_count,
                "body_z_rms_before_deg": calibration.body_z_rms_before_deg,
                "body_z_rms_after_deg": calibration.body_z_rms_after_deg,
                "pitch_rms_before_deg": calibration.pitch_rms_before_deg,
                "pitch_rms_after_deg": calibration.pitch_rms_after_deg,
                "roll_rms_before_deg": calibration.roll_rms_before_deg,
                "roll_rms_after_deg": calibration.roll_rms_after_deg,
            }
            for calibration in calibrations
        },
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2))


def format_float(value: float | None) -> str:
    if value is None or not math.isfinite(value):
        return ""
    return f"{value:.9f}"


def write_corrected_csv(
    path: Path,
    acs_samples: list[AcsSample],
    recovery_samples: list[RecoverySample],
    launch_time_s: float,
    offset_s: float,
    calibrations: dict[str, SensorCalibration],
) -> None:
    recovery_times = [sample.flight_time_s for sample in recovery_samples]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "time_s",
                "status",
                "recovery_flight_time_s",
                "recovery_pitch_deg",
                "recovery_roll_deg",
                "icm_pitch_deg",
                "icm_roll_deg",
                "icm_corrected_pitch_deg",
                "icm_corrected_roll_deg",
                "icm_corrected_quat_w",
                "icm_corrected_quat_x",
                "icm_corrected_quat_y",
                "icm_corrected_quat_z",
                "lsm_pitch_deg",
                "lsm_roll_deg",
                "lsm_corrected_pitch_deg",
                "lsm_corrected_roll_deg",
                "lsm_corrected_quat_w",
                "lsm_corrected_quat_x",
                "lsm_corrected_quat_y",
                "lsm_corrected_quat_z",
            ]
        )

        recovery_launch_time_s = launch_time_s + offset_s
        for sample in acs_samples:
            recovery_sample = nearest_recovery_sample(
                recovery_times,
                recovery_samples,
                sample.time_s - recovery_launch_time_s,
                tolerance_s=0.02,
            )
            recovery_pitch = recovery_roll = None
            recovery_flight_time_s = None
            if recovery_sample is not None:
                recovery_flight_time_s = recovery_sample.flight_time_s
                recovery_pitch, recovery_roll = quat_to_pitch_roll_deg(recovery_sample.quat)

            row: list[str] = [
                format_float(sample.time_s),
                sample.status,
                format_float(recovery_flight_time_s),
                format_float(recovery_pitch),
                format_float(recovery_roll),
            ]

            for sensor_name, quat in (("icm", sample.icm_quat), ("lsm", sample.lsm_quat)):
                if quat is None or sensor_name not in calibrations:
                    row.extend(["", "", "", "", "", "", "", ""])
                    continue
                calibration = calibrations[sensor_name]
                sensor_quat = quat_apply_variant(quat, calibration.variant)
                corrected_quat = normalize_quat(quat_multiply(calibration.mount_quat, sensor_quat))
                pitch_deg, roll_deg = quat_to_pitch_roll_deg(sensor_quat)
                corrected_pitch_deg, corrected_roll_deg = quat_to_pitch_roll_deg(corrected_quat)
                row.extend(
                    [
                        format_float(pitch_deg),
                        format_float(roll_deg),
                        format_float(corrected_pitch_deg),
                        format_float(corrected_roll_deg),
                        format_float(corrected_quat[0]),
                        format_float(corrected_quat[1]),
                        format_float(corrected_quat[2]),
                        format_float(corrected_quat[3]),
                    ]
                )
            writer.writerow(row)


def decimate_points(times: list[float], values: list[float], max_points: int = 3000) -> tuple[list[float], list[float]]:
    if len(times) <= max_points:
        return times, values
    step = max(1, len(times) // max_points)
    reduced_times = times[::step]
    reduced_values = values[::step]
    if reduced_times[-1] != times[-1]:
        reduced_times.append(times[-1])
        reduced_values.append(values[-1])
    return reduced_times, reduced_values


def scale(value: float, lower: float, upper: float, out_lower: float, out_upper: float) -> float:
    if upper <= lower:
        return (out_lower + out_upper) * 0.5
    return out_lower + (value - lower) * (out_upper - out_lower) / (upper - lower)


def polyline_points(
    xs: list[float],
    ys: list[float],
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    left: float,
    top: float,
    width: float,
    height: float,
) -> str:
    bottom = top + height
    points: list[str] = []
    for x, y in zip(xs, ys):
        if not (math.isfinite(x) and math.isfinite(y)):
            continue
        px = scale(x, x_min, x_max, left, left + width)
        py = scale(y, y_min, y_max, bottom, top)
        points.append(f"{px:.2f},{py:.2f}")
    return " ".join(points)


def padded_range(values: list[float], include_zero: bool = False) -> tuple[float, float]:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return (-1.0, 1.0)
    lower = min(finite)
    upper = max(finite)
    if include_zero:
        lower = min(lower, 0.0)
        upper = max(upper, 0.0)
    if upper <= lower:
        pad = max(1.0, abs(lower) * 0.1)
        return (lower - pad, upper + pad)
    pad = max(1.0, (upper - lower) * 0.08)
    return (lower - pad, upper + pad)


def build_series(
    acs_samples: list[AcsSample],
    recovery_samples: list[RecoverySample],
    launch_time_s: float,
    offset_s: float,
    calibration: SensorCalibration,
    sensor_name: str,
) -> tuple[list[float], list[float], list[float], list[float], list[float]]:
    recovery_times = [sample.flight_time_s for sample in recovery_samples]
    abs_times: list[float] = []
    original_values_pitch: list[float] = []
    corrected_values_pitch: list[float] = []
    original_values_roll: list[float] = []
    corrected_values_roll: list[float] = []
    recovery_pitch_values: list[float] = []
    recovery_roll_values: list[float] = []
    recovery_abs_times: list[float] = []

    recovery_launch_time_s = launch_time_s + offset_s
    for sample in acs_samples:
        quat = sample.icm_quat if sensor_name == "icm" else sample.lsm_quat
        if quat is None:
            continue
        recovery_sample = nearest_recovery_sample(
            recovery_times,
            recovery_samples,
            sample.time_s - recovery_launch_time_s,
            tolerance_s=0.02,
        )
        if recovery_sample is None:
            continue
        sensor_quat = quat_apply_variant(quat, calibration.variant)
        corrected_quat = normalize_quat(quat_multiply(calibration.mount_quat, sensor_quat))
        original_pitch, original_roll = quat_to_pitch_roll_deg(sensor_quat)
        corrected_pitch, corrected_roll = quat_to_pitch_roll_deg(corrected_quat)
        recovery_pitch, recovery_roll = quat_to_pitch_roll_deg(recovery_sample.quat)

        abs_times.append(sample.time_s)
        original_values_pitch.append(original_pitch)
        corrected_values_pitch.append(corrected_pitch)
        original_values_roll.append(original_roll)
        corrected_values_roll.append(corrected_roll)
        recovery_pitch_values.append(recovery_pitch)
        recovery_roll_values.append(recovery_roll)
        recovery_abs_times.append(recovery_launch_time_s + recovery_sample.flight_time_s)

    return (
        abs_times,
        original_values_pitch,
        corrected_values_pitch,
        original_values_roll,
        corrected_values_roll,
        recovery_abs_times,
        recovery_pitch_values,
        recovery_roll_values,
    )


def draw_panel(
    title: str,
    series: list[tuple[str, list[float], list[float], str]],
    left: float,
    top: float,
    width: float,
    height: float,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    show_x_ticks: bool,
) -> str:
    elements = [
        f'<rect x="{left}" y="{top}" width="{width}" height="{height}" fill="#fffdfa" stroke="#1f2933" stroke-width="1.2"/>',
        f'<text x="{left}" y="{top - 12}" font-family="monospace" font-size="18" fill="#1f2933">{title}</text>',
    ]
    for i in range(6):
        tick = y_min + (y_max - y_min) * i / 5.0
        y = scale(tick, y_min, y_max, top + height, top)
        elements.append(
            f'<line x1="{left}" y1="{y:.2f}" x2="{left + width}" y2="{y:.2f}" stroke="#d9dde3" stroke-width="1"/>'
        )
        elements.append(
            f'<text x="{left - 10}" y="{y + 4:.2f}" text-anchor="end" font-family="monospace" font-size="12">{tick:.1f}</text>'
        )
    for i in range(6):
        tick = x_min + (x_max - x_min) * i / 5.0
        x = scale(tick, x_min, x_max, left, left + width)
        elements.append(
            f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{top + height}" stroke="#eceff3" stroke-width="1"/>'
        )
        if show_x_ticks:
            elements.append(
                f'<text x="{x:.2f}" y="{top + height + 22}" text-anchor="middle" font-family="monospace" font-size="12">{tick:.1f}</text>'
            )
    for index, (label, xs, ys, color) in enumerate(series):
        points = polyline_points(xs, ys, x_min, x_max, y_min, y_max, left, top, width, height)
        elements.append(f'<polyline fill="none" stroke="{color}" stroke-width="1.6" points="{points}"/>')
        ly = top + 16 + index * 18
        elements.append(f'<line x1="{left + 14}" y1="{ly}" x2="{left + 36}" y2="{ly}" stroke="{color}" stroke-width="2.4"/>')
        elements.append(f'<text x="{left + 44}" y="{ly + 4}" font-family="monospace" font-size="12">{label}</text>')
    if show_x_ticks:
        elements.append(
            f'<text x="{left + width / 2:.2f}" y="{top + height + 44}" text-anchor="middle" font-family="monospace" font-size="13">Time [s]</text>'
        )
    return "\n".join(elements)


def write_svg(
    path: Path,
    acs_samples: list[AcsSample],
    recovery_samples: list[RecoverySample],
    launch_time_s: float,
    offset_s: float,
    calibrations: dict[str, SensorCalibration],
) -> None:
    icm = build_series(acs_samples, recovery_samples, launch_time_s, offset_s, calibrations["icm"], "icm")
    lsm = build_series(acs_samples, recovery_samples, launch_time_s, offset_s, calibrations["lsm"], "lsm")
    x_values = icm[0] + lsm[0]
    x_min = min(x_values)
    x_max = max(x_values)

    pitch_range = padded_range(icm[1] + icm[2] + icm[6] + lsm[1] + lsm[2] + lsm[6])
    roll_range = padded_range(icm[3] + icm[4] + icm[7] + lsm[3] + lsm[4] + lsm[7])

    width = 1700
    height = 1500
    left = 110
    right = 44
    top = 80
    bottom = 70
    gap = 58
    panel_count = 4
    plot_width = width - left - right
    plot_height = (height - top - bottom - gap * (panel_count - 1)) / panel_count

    panels = [
        (
            "ICM Pitch vs Recovery",
            [
                ("ICM Original", *decimate_points(icm[0], icm[1]), "#1f77b4"),
                ("ICM Corrected", *decimate_points(icm[0], icm[2]), "#d62728"),
                ("Recovery", *decimate_points(icm[5], icm[6]), "#111827"),
            ],
            pitch_range,
        ),
        (
            "ICM Roll vs Recovery",
            [
                ("ICM Original", *decimate_points(icm[0], icm[3]), "#1f77b4"),
                ("ICM Corrected", *decimate_points(icm[0], icm[4]), "#d62728"),
                ("Recovery", *decimate_points(icm[5], icm[7]), "#111827"),
            ],
            roll_range,
        ),
        (
            "LSM Pitch vs Recovery",
            [
                ("LSM Original", *decimate_points(lsm[0], lsm[1]), "#ea580c"),
                ("LSM Corrected", *decimate_points(lsm[0], lsm[2]), "#16a34a"),
                ("Recovery", *decimate_points(lsm[5], lsm[6]), "#111827"),
            ],
            pitch_range,
        ),
        (
            "LSM Roll vs Recovery",
            [
                ("LSM Original", *decimate_points(lsm[0], lsm[3]), "#ea580c"),
                ("LSM Corrected", *decimate_points(lsm[0], lsm[4]), "#16a34a"),
                ("Recovery", *decimate_points(lsm[5], lsm[7]), "#111827"),
            ],
            roll_range,
        ),
    ]

    elements = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '  <rect width="100%" height="100%" fill="#f7f4ed"/>',
        '  <text x="110" y="38" font-family="monospace" font-size="24" fill="#1f2933">Attitude Calibrated To Recovery Reference</text>',
    ]
    for index, (title, series, y_range) in enumerate(panels):
        panel_top = top + index * (plot_height + gap)
        elements.append(
            draw_panel(
                title,
                series,
                left,
                panel_top,
                plot_width,
                plot_height,
                x_min,
                x_max,
                y_range[0],
                y_range[1],
                index == len(panels) - 1,
            )
        )
    elements.append("</svg>")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(elements))


def main() -> None:
    parser = argparse.ArgumentParser(description="Time-align ACS attitude to recovery quaternions and emit corrected data.")
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT_CSV, help="ACS CSV input path.")
    parser.add_argument("--recovery", type=Path, default=DEFAULT_RECOVERY_CSV, help="Recovery CSV with quaternions.")
    parser.add_argument("--output-csv", type=Path, default=DEFAULT_OUTPUT_CSV, help="Corrected attitude CSV output path.")
    parser.add_argument("--output-json", type=Path, default=DEFAULT_OUTPUT_JSON, help="Calibration summary JSON output path.")
    parser.add_argument("--output-svg", type=Path, default=DEFAULT_OUTPUT_SVG, help="Comparison SVG output path.")
    parser.add_argument("--padding-seconds", type=float, default=1.5, help="Extra padding around the ACS flight window.")
    parser.add_argument("--offset-min", type=float, default=-2.0, help="Minimum recovery time offset to search.")
    parser.add_argument("--offset-max", type=float, default=2.0, help="Maximum recovery time offset to search.")
    parser.add_argument("--match-tolerance", type=float, default=0.02, help="Maximum time mismatch when pairing ACS and recovery samples.")
    args = parser.parse_args()

    acs_samples = load_acs_samples(args.input)
    recovery_samples = load_recovery_samples(args.recovery)
    window_start_s, window_end_s, launch_time_s = flight_window(acs_samples, args.padding_seconds)
    best_offset_s = search_time_offset(
        acs_samples,
        recovery_samples,
        launch_time_s,
        window_start_s,
        window_end_s,
        args.match_tolerance,
        args.offset_min,
        args.offset_max,
    )

    recovery_times = [sample.flight_time_s for sample in recovery_samples]
    calibrations: list[SensorCalibration] = []
    calibration_map: dict[str, SensorCalibration] = {}
    for sensor_name in ("icm", "lsm"):
        sensor_samples = decimate_sensor_samples(acs_samples, sensor_name, window_start_s, window_end_s, stride=5)
        calibration = calibrate_sensor(
            sensor_name,
            sensor_samples,
            recovery_samples,
            recovery_times,
            launch_time_s,
            best_offset_s,
            args.match_tolerance,
        )
        if calibration is None:
            raise SystemExit(f"Unable to calibrate {sensor_name} against recovery attitude.")
        calibrations.append(calibration)
        calibration_map[sensor_name] = calibration

    write_summary(args.output_json, best_offset_s, calibrations)
    write_corrected_csv(args.output_csv, acs_samples, recovery_samples, launch_time_s, best_offset_s, calibration_map)
    write_svg(args.output_svg, acs_samples, recovery_samples, launch_time_s, best_offset_s, calibration_map)

    print(f"Wrote {args.output_csv}")
    print(f"Wrote {args.output_json}")
    print(f"Wrote {args.output_svg}")
    print(f"time_offset_s={best_offset_s:.6f}")
    for calibration in calibrations:
        print(
            f"{calibration.name}: variant={calibration.variant} pairs={calibration.pair_count} "
            f"body_z_rms_before_deg={calibration.body_z_rms_before_deg:.3f} "
            f"body_z_rms_after_deg={calibration.body_z_rms_after_deg:.3f} "
            f"pitch_rms_before_deg={calibration.pitch_rms_before_deg:.3f} "
            f"pitch_rms_after_deg={calibration.pitch_rms_after_deg:.3f} "
            f"roll_rms_before_deg={calibration.roll_rms_before_deg:.3f} "
            f"roll_rms_after_deg={calibration.roll_rms_after_deg:.3f}"
        )


if __name__ == "__main__":
    main()
