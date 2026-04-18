#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
REPLAY_DIR = SCRIPT_DIR.parent
ROOT = REPLAY_DIR.parents[1]
PLOTS_DIR = REPLAY_DIR / "plots"

DEFAULT_INPUT_CSV = ROOT / "fullscale_3.csv"
DEFAULT_OUTPUT_SVG = PLOTS_DIR / "fullscale_3_estimator_diagnostics.svg"
DEFAULT_ATTITUDE_SVG = PLOTS_DIR / "fullscale_3_attitude_compare.svg"
DEFAULT_RECOVERY_CSV = REPLAY_DIR / "data" / "BlRv_LASS_alt HR_10-12-2025_16_01_16.csv"

FEET_TO_METERS = 0.3048
SIGMA_ALT_M = 1.0
APOGEE_TARGET_M = 1540.0


@dataclass
class DiagnosticRow:
    time_s: float
    status: str
    baro_agl_m: float
    state_z_m: float
    baro_vz_mps: float
    state_vz_mps: float
    altitude_residual_m: float
    velocity_residual_mps: float
    baro_innovation_m: float
    baro_min_gate_halfwidth_m: float
    baro_min_gate_margin_m: float
    altimeter_sigma_scale: float
    altimeter_gate_sigma: float
    flap_cmd_deg: float
    predicted_apogee_m: float
    predicted_apogee_error_m: float
    bno_z_mps2: float
    icm_z_mps2: float
    state_acceleration_z_mps2: float
    state_inertial_acceleration_z_mps2: float
    icm_pitch_deg: float
    icm_roll_deg: float
    lsm_pitch_deg: float
    lsm_roll_deg: float
    bno_pitch_deg: float
    bno_roll_deg: float


@dataclass
class RecoveryRow:
    time_s: float
    pitch_deg: float
    roll_deg: float


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


def iter_csv_rows(path: Path):
    with path.open(newline="") as handle:
        reader = csv.DictReader(line for line in handle if not line.startswith("#"))
        for row in reader:
            yield row


def pick_float(raw: dict[str, str], *names: str) -> float | None:
    for name in names:
        value = parse_float(raw.get(name))
        if value is not None:
            return value
    return None


def quaternion_to_pitch_roll_deg(w: float, x: float, y: float, z: float) -> tuple[float, float]:
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 0.0 or not math.isfinite(norm):
        return (math.nan, math.nan)
    w /= norm
    x /= norm
    y /= norm
    z /= norm

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


def compute_smoothed_slope(times: list[float], values: list[float], window_seconds: float = 0.30) -> list[float]:
    if len(times) != len(values):
        raise ValueError("times and values must have the same length")
    if len(times) < 2:
        return [0.0 for _ in times]

    slopes: list[float] = []
    left = 0
    right = 0
    half_window = max(window_seconds * 0.5, 1.0e-3)

    for time_s in times:
        while left + 1 < len(times) and times[left] < time_s - half_window:
            left += 1
        while right + 1 < len(times) and times[right + 1] <= time_s + half_window:
            right += 1

        start = max(0, left - 1)
        stop = min(len(times), right + 2)
        xs = times[start:stop]
        ys = values[start:stop]
        if len(xs) < 2:
            slopes.append(0.0)
            continue

        mean_x = sum(xs) / len(xs)
        mean_y = sum(ys) / len(ys)
        denom = sum((x - mean_x) * (x - mean_x) for x in xs)
        if denom <= 1.0e-12:
            slopes.append(0.0)
            continue
        numer = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
        slopes.append(numer / denom)

    return slopes


def load_rows(path: Path) -> list[DiagnosticRow]:
    base_altitude_feet: float | None = None
    base_state_z_m: float | None = None
    state_z_is_agl: bool | None = None
    samples: list[dict[str, float | str]] = []

    for raw in iter_csv_rows(path):
        time_s = pick_float(raw, "state_time", "time_s", "sensor_timestamp", "timestamp")
        altitude_feet = pick_float(raw, "sensor_altitude_feet", "altitude_feet")
        if time_s is None or altitude_feet is None:
            continue

        if base_altitude_feet is None:
            base_altitude_feet = altitude_feet

        baro_agl_m = (altitude_feet - base_altitude_feet) * FEET_TO_METERS
        state_agl_feet = pick_float(raw, "state_altitude_agl_feet")
        if state_agl_feet is not None:
            state_z_m = state_agl_feet * FEET_TO_METERS
        else:
            state_z_m = pick_float(raw, "state_position_z", "altitude_m", "altitude_meters")
        state_vz_mps = pick_float(raw, "state_velocity_z", "velocity_mps", "vertical_velocity")
        altimeter_sigma_scale = pick_float(raw, "sensor_altimeter_sigma_scale")
        altimeter_gate_sigma = pick_float(raw, "sensor_altimeter_gate_sigma")

        if state_z_m is not None and state_z_is_agl is None:
            if state_agl_feet is not None:
                state_z_is_agl = True
                base_state_z_m = 0.0
            else:
                first_baro_abs_m = altitude_feet * FEET_TO_METERS
                state_z_is_agl = abs(state_z_m - first_baro_abs_m) > 30.0
                base_state_z_m = 0.0 if state_z_is_agl else state_z_m

        bno_pitch_deg = math.nan
        bno_roll_deg = math.nan
        bno_q = [
            pick_float(raw, "sensor_bno_quat_w"),
            pick_float(raw, "sensor_bno_quat_x"),
            pick_float(raw, "sensor_bno_quat_y"),
            pick_float(raw, "sensor_bno_quat_z"),
        ]
        if all(value is not None for value in bno_q):
            bno_pitch_deg, bno_roll_deg = quaternion_to_pitch_roll_deg(
                bno_q[0], bno_q[1], bno_q[2], bno_q[3]
            )

        samples.append(
            {
                "time_s": time_s,
                "status": (raw.get("flight_status") or raw.get("status") or "").strip().lower(),
                "baro_agl_m": baro_agl_m,
                "state_z_m": 0.0 if state_z_m is None else state_z_m,
                "state_vz_mps": 0.0 if state_vz_mps is None else state_vz_mps,
                "altimeter_sigma_scale": 1.0 if altimeter_sigma_scale is None else altimeter_sigma_scale,
                "altimeter_gate_sigma": 0.0 if altimeter_gate_sigma is None else altimeter_gate_sigma,
                "flap_cmd_deg": 0.0 if pick_float(raw, "sensor_auto_cmd_deg", "auto_cmd_deg", "command_deg") is None else pick_float(raw, "sensor_auto_cmd_deg", "auto_cmd_deg", "command_deg"),
                "predicted_apogee_m": 0.0 if pick_float(raw, "state_apogee_estimate", "sensor_optimizer_best_predicted_apogee_m", "apogee_prediction_m", "apogee_estimate") is None else pick_float(raw, "state_apogee_estimate", "sensor_optimizer_best_predicted_apogee_m", "apogee_prediction_m", "apogee_estimate"),
                "bno_z_mps2": 0.0 if pick_float(raw, "sensor_accel_bno_z", "accel_bno_z") is None else pick_float(raw, "sensor_accel_bno_z", "accel_bno_z"),
                "icm_z_mps2": 0.0 if pick_float(raw, "sensor_accel_icm_z", "accel_icm_z") is None else pick_float(raw, "sensor_accel_icm_z", "accel_icm_z"),
                "icm_pitch_deg": math.nan if pick_float(raw, "sensor_icm_pitch_deg") is None else pick_float(raw, "sensor_icm_pitch_deg"),
                "icm_roll_deg": math.nan if pick_float(raw, "sensor_icm_roll_deg") is None else pick_float(raw, "sensor_icm_roll_deg"),
                "lsm_pitch_deg": math.nan if pick_float(raw, "sensor_lsm_pitch_deg") is None else pick_float(raw, "sensor_lsm_pitch_deg"),
                "lsm_roll_deg": math.nan if pick_float(raw, "sensor_lsm_roll_deg") is None else pick_float(raw, "sensor_lsm_roll_deg"),
                "bno_pitch_deg": bno_pitch_deg,
                "bno_roll_deg": bno_roll_deg,
            }
        )

    if not samples:
        raise SystemExit(f"No usable rows found in {path}")

    if state_z_is_agl is False and base_state_z_m is not None:
        for sample in samples:
            sample["state_z_m"] = float(sample["state_z_m"]) - base_state_z_m

    times = [float(sample["time_s"]) for sample in samples]
    baro_agl = [float(sample["baro_agl_m"]) for sample in samples]
    state_z = [float(sample["state_z_m"]) for sample in samples]
    state_vz = [float(sample["state_vz_mps"]) for sample in samples]
    baro_vz = compute_smoothed_slope(times, baro_agl)

    rows: list[DiagnosticRow] = []
    for sample, smoothed_vz in zip(samples, baro_vz):
        innovation = float(sample["baro_agl_m"]) - float(sample["state_z_m"])
        min_gate_halfwidth = float(sample["altimeter_gate_sigma"]) * math.sqrt(
            max(1.0, float(sample["altimeter_sigma_scale"])) * SIGMA_ALT_M * SIGMA_ALT_M
        )
        rows.append(
            DiagnosticRow(
                time_s=float(sample["time_s"]),
                status=str(sample["status"]),
                baro_agl_m=float(sample["baro_agl_m"]),
                state_z_m=float(sample["state_z_m"]),
                baro_vz_mps=smoothed_vz,
                state_vz_mps=float(sample["state_vz_mps"]),
                altitude_residual_m=float(sample["state_z_m"]) - float(sample["baro_agl_m"]),
                velocity_residual_mps=float(sample["state_vz_mps"]) - smoothed_vz,
                baro_innovation_m=innovation,
                baro_min_gate_halfwidth_m=min_gate_halfwidth,
                baro_min_gate_margin_m=min_gate_halfwidth - abs(innovation),
                altimeter_sigma_scale=float(sample["altimeter_sigma_scale"]),
                altimeter_gate_sigma=float(sample["altimeter_gate_sigma"]),
                flap_cmd_deg=float(sample["flap_cmd_deg"]),
                predicted_apogee_m=float(sample["predicted_apogee_m"]),
                predicted_apogee_error_m=float(sample["predicted_apogee_m"]) - APOGEE_TARGET_M,
                bno_z_mps2=float(sample["bno_z_mps2"]),
                icm_z_mps2=float(sample["icm_z_mps2"]),
                state_acceleration_z_mps2=0.0
                if pick_float(raw, "state_acceleration_z") is None
                else pick_float(raw, "state_acceleration_z"),
                state_inertial_acceleration_z_mps2=0.0
                if pick_float(raw, "state_inertial_acceleration_z") is None
                else pick_float(raw, "state_inertial_acceleration_z"),
                icm_pitch_deg=float(sample["icm_pitch_deg"]),
                icm_roll_deg=float(sample["icm_roll_deg"]),
                lsm_pitch_deg=float(sample["lsm_pitch_deg"]),
                lsm_roll_deg=float(sample["lsm_roll_deg"]),
                bno_pitch_deg=float(sample["bno_pitch_deg"]),
                bno_roll_deg=float(sample["bno_roll_deg"]),
            )
        )
    return rows


def load_recovery_rows(path: Path, launch_time_s: float, window_start_s: float, window_end_s: float) -> list[RecoveryRow]:
    if not path.exists():
        return []

    rows: list[RecoveryRow] = []
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
            absolute_time_s = launch_time_s + flight_time_s
            if absolute_time_s < window_start_s or absolute_time_s > window_end_s:
                continue
            pitch_deg, roll_deg = quaternion_to_pitch_roll_deg(q1, q2, q3, q4)
            rows.append(RecoveryRow(time_s=absolute_time_s, pitch_deg=pitch_deg, roll_deg=roll_deg))
    return rows


def select_flight_window(rows: list[DiagnosticRow], padding_s: float) -> list[DiagnosticRow]:
    if not rows:
        return rows

    launch_time_s: float | None = None
    end_time_s: float | None = None
    for row in rows:
        if row.status and row.status != "ground":
            launch_time_s = row.time_s
            break
        if row.baro_agl_m > 5.0:
            launch_time_s = row.time_s
            break

    for row in reversed(rows):
        if row.status and row.status != "ground":
            end_time_s = row.time_s
            break
        if row.baro_agl_m > 5.0:
            end_time_s = row.time_s
            break

    if launch_time_s is None:
        launch_time_s = rows[0].time_s
    if end_time_s is None:
        end_time_s = rows[-1].time_s

    start_time_s = launch_time_s - padding_s
    stop_time_s = end_time_s + padding_s
    windowed = [row for row in rows if start_time_s <= row.time_s <= stop_time_s]
    return windowed if windowed else rows


def decimate_rows(rows: list[DiagnosticRow], max_points: int = 5000) -> list[DiagnosticRow]:
    if len(rows) <= max_points:
        return rows
    step = max(1, len(rows) // max_points)
    reduced = rows[::step]
    if reduced[-1] != rows[-1]:
        reduced.append(rows[-1])
    return reduced


def scale(value: float, lower: float, upper: float, out_lower: float, out_upper: float) -> float:
    if upper <= lower:
        return (out_lower + out_upper) * 0.5
    fraction = (value - lower) / (upper - lower)
    return out_lower + fraction * (out_upper - out_lower)


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


def format_label(value: float) -> str:
    magnitude = max(abs(value), 1.0)
    if magnitude >= 1000.0:
        return f"{value:.0f}"
    if magnitude >= 100.0:
        return f"{value:.1f}"
    return f"{value:.2f}"


def build_axis_ticks(lower: float, upper: float, count: int = 6) -> list[float]:
    if not (math.isfinite(lower) and math.isfinite(upper)):
        return [0.0]
    if upper <= lower:
        return [lower]
    return [lower + (upper - lower) * i / (count - 1) for i in range(count)]


def padded_range(values: list[float], include_zero: bool = False, pad_fraction: float = 0.08) -> tuple[float, float]:
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
    pad = max(1.0e-6, (upper - lower) * pad_fraction)
    return (lower - pad, upper + pad)


def draw_panel(
    title: str,
    x_label: str,
    y_label: str,
    series: list[tuple[str, list[float], str, float]],
    times: list[float],
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    left: float,
    top: float,
    plot_width: float,
    plot_height: float,
    markers: list[tuple[float, str]],
    show_x_ticks: bool,
) -> str:
    elements: list[str] = []
    elements.append(
        f'<rect x="{left}" y="{top}" width="{plot_width}" height="{plot_height}" fill="#fffdfa" stroke="#1f2933" stroke-width="1.2"/>'
    )
    elements.append(
        f'<text x="{left}" y="{top - 12}" font-family="monospace" font-size="18" fill="#1f2933">{title}</text>'
    )

    for tick in build_axis_ticks(y_min, y_max):
        y = scale(tick, y_min, y_max, top + plot_height, top)
        elements.append(
            f'<line x1="{left}" y1="{y:.2f}" x2="{left + plot_width}" y2="{y:.2f}" stroke="#d9dde3" stroke-width="1"/>'
        )
        elements.append(
            f'<text x="{left - 10}" y="{y + 4:.2f}" text-anchor="end" font-family="monospace" font-size="12" fill="#334155">{format_label(tick)}</text>'
        )

    for tick in build_axis_ticks(x_min, x_max):
        x = scale(tick, x_min, x_max, left, left + plot_width)
        elements.append(
            f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{top + plot_height}" stroke="#eceff3" stroke-width="1"/>'
        )
        if show_x_ticks:
            elements.append(
                f'<text x="{x:.2f}" y="{top + plot_height + 22}" text-anchor="middle" font-family="monospace" font-size="12" fill="#334155">{tick:.1f}</text>'
            )

    for marker_time, color in markers:
        x = scale(marker_time, x_min, x_max, left, left + plot_width)
        elements.append(
            f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{top + plot_height}" stroke="{color}" stroke-width="1.2" stroke-dasharray="5,4"/>'
        )

    for label, values, color, stroke_width in series:
        elements.append(
            f'<polyline fill="none" stroke="{color}" stroke-width="{stroke_width}" points="'
            f'{polyline_points(times, values, x_min, x_max, y_min, y_max, left, top, plot_width, plot_height)}"/>'
        )

    legend_x = left + 14
    legend_y = top + 14
    legend_spacing = 20
    for idx, (label, _, color, stroke_width) in enumerate(series):
        y = legend_y + idx * legend_spacing
        elements.append(
            f'<line x1="{legend_x}" y1="{y}" x2="{legend_x + 22}" y2="{y}" stroke="{color}" stroke-width="{stroke_width + 0.6}"/>'
        )
        elements.append(
            f'<text x="{legend_x + 30}" y="{y + 4}" font-family="monospace" font-size="12" fill="#1f2933">{label}</text>'
        )

    if markers:
        y = legend_y + len(series) * legend_spacing
        seen: set[str] = set()
        for _, color in markers:
            if color in seen:
                continue
            seen.add(color)
            label = "First flap command" if color == "#b91c1c" else "First large divergence"
            elements.append(
                f'<line x1="{legend_x}" y1="{y}" x2="{legend_x + 22}" y2="{y}" stroke="{color}" stroke-width="1.2" stroke-dasharray="5,4"/>'
            )
            elements.append(
                f'<text x="{legend_x + 30}" y="{y + 4}" font-family="monospace" font-size="12" fill="#1f2933">{label}</text>'
            )
            y += legend_spacing

    elements.append(
        f'<text x="{left - 58}" y="{top + plot_height / 2:.2f}" text-anchor="middle" font-family="monospace" font-size="13" fill="#1f2933" transform="rotate(-90 {left - 58} {top + plot_height / 2:.2f})">{y_label}</text>'
    )
    if show_x_ticks:
        elements.append(
            f'<text x="{left + plot_width / 2:.2f}" y="{top + plot_height + 44}" text-anchor="middle" font-family="monospace" font-size="13" fill="#1f2933">{x_label}</text>'
        )
    return "\n".join(elements)


def summarize(rows: list[DiagnosticRow]) -> dict[str, float]:
    first_flap_time = next((row.time_s for row in rows if abs(row.flap_cmd_deg) > 0.1), math.nan)
    first_large_divergence = next((row.time_s for row in rows if abs(row.altitude_residual_m) > 50.0), math.nan)
    min_gate_margin = min(row.baro_min_gate_margin_m for row in rows if math.isfinite(row.baro_min_gate_margin_m))
    return {
        "window_start_s": rows[0].time_s,
        "window_end_s": rows[-1].time_s,
        "first_flap_time_s": first_flap_time,
        "first_large_divergence_s": first_large_divergence,
        "min_gate_margin_m": min_gate_margin,
    }


def write_residual_svg(rows: list[DiagnosticRow], output_path: Path, title: str) -> None:
    sampled = decimate_rows(rows)
    times = [row.time_s for row in sampled]
    altitude_residual = [row.altitude_residual_m for row in sampled]
    velocity_residual = [row.velocity_residual_mps for row in sampled]
    innovation = [row.baro_innovation_m for row in sampled]
    gate_pos = [row.baro_min_gate_halfwidth_m for row in sampled]
    gate_neg = [-row.baro_min_gate_halfwidth_m for row in sampled]
    state_accel_z = [row.state_acceleration_z_mps2 for row in sampled]
    inertial_accel_z = [row.state_inertial_acceleration_z_mps2 for row in sampled]
    apogee_error = [row.predicted_apogee_error_m for row in sampled]

    summary = summarize(sampled)
    markers: list[tuple[float, str]] = []
    if math.isfinite(summary["first_flap_time_s"]):
        markers.append((summary["first_flap_time_s"], "#b91c1c"))
    if math.isfinite(summary["first_large_divergence_s"]):
        markers.append((summary["first_large_divergence_s"], "#0f766e"))

    x_min = min(times)
    x_max = max(times)
    width = 1700
    height = 1840
    left = 110
    right = 44
    top = 80
    bottom = 70
    gap = 58
    panel_count = 5
    plot_width = width - left - right
    plot_height = (height - top - bottom - gap * (panel_count - 1)) / panel_count

    panels = [
        (
            "Altitude Residual",
            "Time [s]",
            "state_z - baro_agl [m]",
            [("Altitude Residual", altitude_residual, "#d62728", 1.8)],
            padded_range(altitude_residual, include_zero=True),
        ),
        (
            "Velocity Residual",
            "Time [s]",
            "state_vz - baro_vz [m/s]",
            [("Velocity Residual", velocity_residual, "#1f77b4", 1.8)],
            padded_range(velocity_residual, include_zero=True),
        ),
        (
            "Baro Innovation And Min Gate Width",
            "Time [s]",
            "meters",
            [
                ("Innovation (baro - state)", innovation, "#111827", 1.7),
                ("+ Min Gate Width", gate_pos, "#2ca02c", 1.5),
                ("- Min Gate Width", gate_neg, "#2ca02c", 1.5),
            ],
            padded_range(innovation + gate_pos + gate_neg, include_zero=True),
        ),
        (
            "Estimator Vertical Acceleration",
            "Time [s]",
            "m/s^2",
            [
                ("State Accel Z", state_accel_z, "#d97706", 1.7),
                ("State Inertial Accel Z", inertial_accel_z, "#0f766e", 1.7),
            ],
            padded_range(state_accel_z + inertial_accel_z, include_zero=True),
        ),
        (
            "Predicted Apogee Error",
            "Time [s]",
            "predicted_apogee - 1540 [m]",
            [("Predicted Apogee Error", apogee_error, "#7c2d12", 1.8)],
            padded_range(apogee_error, include_zero=True),
        ),
    ]

    elements = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '  <rect width="100%" height="100%" fill="#f7f4ed"/>',
        f'  <text x="{left}" y="38" font-family="monospace" font-size="24" fill="#1f2933">{title}</text>',
    ]

    for index, (panel_title, x_label, y_label, series, y_range) in enumerate(panels):
        panel_top = top + index * (plot_height + gap)
        elements.append(
            draw_panel(
                panel_title,
                x_label,
                y_label,
                series,
                times,
                x_min,
                x_max,
                y_range[0],
                y_range[1],
                left,
                panel_top,
                plot_width,
                plot_height,
                markers,
                index == len(panels) - 1,
            )
        )

    elements.append("</svg>")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(elements))


def write_attitude_svg(
    rows: list[DiagnosticRow],
    output_path: Path,
    title: str,
    recovery_rows: list[RecoveryRow] | None = None,
) -> None:
    sampled = decimate_rows(rows)
    times = [row.time_s for row in sampled]
    summary = summarize(sampled)
    markers: list[tuple[float, str]] = []
    if math.isfinite(summary["first_flap_time_s"]):
        markers.append((summary["first_flap_time_s"], "#b91c1c"))
    if math.isfinite(summary["first_large_divergence_s"]):
        markers.append((summary["first_large_divergence_s"], "#0f766e"))

    x_min = min(times)
    x_max = max(times)
    width = 1700
    height = 900
    left = 110
    right = 44
    top = 80
    bottom = 70
    gap = 58
    panel_count = 2
    plot_width = width - left - right
    plot_height = (height - top - bottom - gap * (panel_count - 1)) / panel_count

    recovery_rows = [] if recovery_rows is None else recovery_rows
    recovery_times = [row.time_s for row in recovery_rows]
    recovery_pitch = [row.pitch_deg for row in recovery_rows]
    recovery_roll = [row.roll_deg for row in recovery_rows]

    pitch_values = (
        [row.icm_pitch_deg for row in sampled]
        + [row.lsm_pitch_deg for row in sampled]
        + [row.bno_pitch_deg for row in sampled]
        + recovery_pitch
    )
    roll_values = (
        [row.icm_roll_deg for row in sampled]
        + [row.lsm_roll_deg for row in sampled]
        + [row.bno_roll_deg for row in sampled]
        + recovery_roll
    )
    panels = [
        (
            "Pitch Comparison",
            "Time [s]",
            "pitch [deg]",
            [
                ("ICM Pitch", [row.icm_pitch_deg for row in sampled], "#1f77b4", 1.7),
                ("LSM Pitch", [row.lsm_pitch_deg for row in sampled], "#ea580c", 1.7),
                ("BNO Pitch", [row.bno_pitch_deg for row in sampled], "#7c3aed", 1.7),
            ],
            padded_range(pitch_values, include_zero=False),
        ),
        (
            "Roll Comparison",
            "Time [s]",
            "roll [deg]",
            [
                ("ICM Roll", [row.icm_roll_deg for row in sampled], "#1f77b4", 1.7),
                ("LSM Roll", [row.lsm_roll_deg for row in sampled], "#ea580c", 1.7),
                ("BNO Roll", [row.bno_roll_deg for row in sampled], "#7c3aed", 1.7),
            ],
            padded_range(roll_values, include_zero=False),
        ),
    ]

    elements = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '  <rect width="100%" height="100%" fill="#f7f4ed"/>',
        f'  <text x="{left}" y="38" font-family="monospace" font-size="24" fill="#1f2933">{title}</text>',
    ]
    for index, (panel_title, x_label, y_label, series, y_range) in enumerate(panels):
        panel_top = top + index * (plot_height + gap)
        if recovery_rows:
            elements.append(
                draw_panel(
                    panel_title,
                    x_label,
                    y_label,
                    series,
                    times,
                    x_min,
                    x_max,
                    y_range[0],
                    y_range[1],
                    left,
                    panel_top,
                    plot_width,
                    plot_height,
                    markers,
                    index == len(panels) - 1,
                )
            )
            overlay_points = polyline_points(
                recovery_times,
                recovery_pitch if index == 0 else recovery_roll,
                x_min,
                x_max,
                y_range[0],
                y_range[1],
                left,
                panel_top,
                plot_width,
                plot_height,
            )
            if overlay_points:
                elements.append(
                    f'<polyline fill="none" stroke="#111827" stroke-width="1.8" points="{overlay_points}"/>'
                )
                legend_y = panel_top + 14 + len(series) * 20
                elements.append(
                    f'<line x1="{left + 14}" y1="{legend_y}" x2="{left + 36}" y2="{legend_y}" stroke="#111827" stroke-width="2.4"/>'
                )
                elements.append(
                    f'<text x="{left + 44}" y="{legend_y + 4}" font-family="monospace" font-size="12" fill="#1f2933">Recovery {"Pitch" if index == 0 else "Roll"}</text>'
                )
            continue
        elements.append(
            draw_panel(
                panel_title,
                x_label,
                y_label,
                series,
                times,
                x_min,
                x_max,
                y_range[0],
                y_range[1],
                left,
                panel_top,
                plot_width,
                plot_height,
                markers,
                index == len(panels) - 1,
            )
        )
    elements.append("</svg>")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(elements))


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot estimator residuals and attitude comparisons from a flight CSV.")
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT_CSV, help="Input CSV path.")
    parser.add_argument("--svg", type=Path, default=DEFAULT_OUTPUT_SVG, help="Residual diagnostic SVG path.")
    parser.add_argument("--attitude-svg", type=Path, default=DEFAULT_ATTITUDE_SVG, help="Attitude comparison SVG path.")
    parser.add_argument("--recovery-csv", type=Path, default=DEFAULT_RECOVERY_CSV, help="Recovery altimeter CSV with quaternions.")
    parser.add_argument("--padding-seconds", type=float, default=1.5, help="Padding before launch and after the last in-flight sample.")
    args = parser.parse_args()

    rows = select_flight_window(load_rows(args.input), max(0.0, args.padding_seconds))
    launch_time_s = next((row.time_s for row in rows if row.status and row.status != "ground"), rows[0].time_s)
    recovery_rows = load_recovery_rows(args.recovery_csv, launch_time_s, rows[0].time_s, rows[-1].time_s)
    write_residual_svg(rows, args.svg, "Estimator Residual Diagnostics")
    write_attitude_svg(rows, args.attitude_svg, "ICM vs LSM vs BNO vs Recovery Attitude", recovery_rows)
    summary = summarize(rows)

    print(f"Wrote {args.svg}")
    print(f"Wrote {args.attitude_svg}")
    print(f"window_start_s={summary['window_start_s']:.3f}")
    print(f"window_end_s={summary['window_end_s']:.3f}")
    if math.isfinite(summary["first_flap_time_s"]):
        print(f"first_flap_time_s={summary['first_flap_time_s']:.3f}")
    if math.isfinite(summary["first_large_divergence_s"]):
        print(f"first_large_divergence_s={summary['first_large_divergence_s']:.3f}")
    print(f"min_gate_margin_m={summary['min_gate_margin_m']:.3f}")


if __name__ == "__main__":
    main()
