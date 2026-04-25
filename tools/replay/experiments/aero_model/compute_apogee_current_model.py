#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path


from replaylib.paths import SCRIPTS_DIR
SCRIPT_DIR = SCRIPTS_DIR
REPLAY_DIR = SCRIPT_DIR.parent
ROOT = REPLAY_DIR.parents[1]
DATA_DIR = REPLAY_DIR / "data"
PLOTS_DIR = REPLAY_DIR / "plots"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from python.apogee_predictor_sim import CP_CG_M
from python.apogee_predictor_sim import DRY_MASS
from python.apogee_predictor_sim import ForceTable
from python.apogee_predictor_sim import G
from python.apogee_predictor_sim import GAMMA
from python.apogee_predictor_sim import MAX_STEPS
from python.apogee_predictor_sim import MOMENT_INERTIA
from python.apogee_predictor_sim import PRED_DT
from python.apogee_predictor_sim import R_GAS
from python.apogee_predictor_sim import gradient_wind
from python.apogee_predictor_sim import interp_axis
from python.apogee_predictor_sim import load_force_table
from python.apogee_predictor_sim import temperature_k
from python.apogee_predictor_sim import trilinear
from python.apogee_predictor_sim import wrap_to_pi


INPUT_CSV = DATA_DIR / "output.csv"
CFD_CSV = ROOT / "lib" / "cfd.csv"
OUTPUT_CSV = DATA_DIR / "output_current_model_prediction.csv"
OUTPUT_SVG = PLOTS_DIR / "output_current_model_prediction.svg"

SEA_LEVEL_PRESSURE_PA = 101325.0
SEA_LEVEL_TEMPERATURE_K = 288.15
REFERENCE_DENSITY_KG_PER_M3 = 1.225

PREDICTOR_HORIZONTAL_ACCEL_LIMIT_MPS2 = 12.0
PREDICTOR_HORIZONTAL_DECAY_TAU_SECONDS = 1.75
PREDICTOR_MAX_SEED_ZENITH_DEG = 20.0
PREDICTOR_MAX_SEED_ANGULAR_RATE_RAD_PER_SEC = 1.5
PREDICTOR_MAX_HORIZONTAL_SPEED_MPS = 65.0
PREDICTOR_MIN_HORIZONTAL_SPEED_CAP_MPS = 6.0
PREDICTOR_HORIZONTAL_SPEED_MARGIN_MPS = 3.0

ADAPTIVE_AXIAL_DRAG_SCALE_MIN = 1.00
ADAPTIVE_AXIAL_DRAG_SCALE_MAX = 1.35
ADAPTIVE_AXIAL_DRAG_RESIDUAL_CLAMP_MPS2 = 8.0
ADAPTIVE_AXIAL_DRAG_TAU_SECONDS = 0.45
ADAPTIVE_AXIAL_ACCEL_MIN_ABS_MPS2 = 0.75

ENABLE_MACH_DEPENDENT_DRAG = True
ENABLE_DENSITY_SCALING = True
MACH_BIN_EDGES = [0.3, 0.6, 0.9, 1.2]
MACH_DRAG_SCALE_MIN = 0.85
MACH_DRAG_SCALE_MAX = 1.50
MACH_DRAG_ADAPT_TAU_SECONDS_EARLY = 0.25
MACH_DRAG_ADAPT_TAU_SECONDS_LATE = 1.2
MACH_DRAG_ADAPT_TAU_TRANSITION_START = 4.0
MACH_DRAG_ADAPT_TAU_TRANSITION_END = 1.5


@dataclass
class ReplayRow:
    time_s: float
    status: str
    altitude_m: float
    vertical_velocity_mps: float
    measured_accel_z_mps2: float
    inertial_ax_mps2: float
    inertial_ay_mps2: float
    zenith_rad: float
    has_quaternion: bool
    logged_apogee_m: float


@dataclass
class ApogeeState:
    altitude_m: float
    horizontal_m: float
    vertical_v: float
    horizontal_v: float
    zenith: float
    angular_v: float
    acs_deg: float = 0.0


@dataclass
class PredictorHorizontalVelocityTracker:
    vx: float = 0.0
    vy: float = 0.0


class EnvironmentModel:
    def __init__(self) -> None:
        self._gradient_wind = gradient_wind()
        self._wind_offset = [0.0, 0.0, 0.0]
        self._density_correction = 1.0

    def temperature_kelvin(self, altitude_m: float) -> float:
        return temperature_k(altitude_m)

    def pressure_pa(self, altitude_m: float) -> float:
        if not ENABLE_DENSITY_SCALING:
            return SEA_LEVEL_PRESSURE_PA
        temp = self.temperature_kelvin(altitude_m)
        if temp <= 0.0 or SEA_LEVEL_TEMPERATURE_K <= 0.0:
            return SEA_LEVEL_PRESSURE_PA
        return SEA_LEVEL_PRESSURE_PA * math.pow(temp / SEA_LEVEL_TEMPERATURE_K, 5.2561)

    def density_ratio(self, altitude_m: float) -> float:
        if not ENABLE_DENSITY_SCALING:
            return 1.0
        temp = self.temperature_kelvin(altitude_m)
        if temp <= 0.0:
            return 1.0
        density = self.pressure_pa(altitude_m) / (R_GAS * temp)
        return (density / REFERENCE_DENSITY_KG_PER_M3) * self._density_correction

    def effective_wind(self) -> tuple[float, float, float]:
        return (
            self._gradient_wind[0] + self._wind_offset[0],
            self._gradient_wind[1] + self._wind_offset[1],
            self._gradient_wind[2] + self._wind_offset[2],
        )


class MachDependentDragScale:
    def __init__(self) -> None:
        self.scales = [1.0 for _ in MACH_BIN_EDGES]

    def interpolate_scale(self, mach: float) -> float:
        lower_idx = 0
        for i in range(len(MACH_BIN_EDGES) - 1):
            if mach >= MACH_BIN_EDGES[i]:
                lower_idx = i
        upper_idx = min(lower_idx + 1, len(MACH_BIN_EDGES) - 1)
        if lower_idx == upper_idx:
            return self.scales[lower_idx]
        lower_mach = MACH_BIN_EDGES[lower_idx]
        upper_mach = MACH_BIN_EDGES[upper_idx]
        denom = upper_mach - lower_mach
        if denom <= 0.0:
            return self.scales[lower_idx]
        t = clamp((mach - lower_mach) / denom, 0.0, 1.0)
        return self.scales[lower_idx] + t * (self.scales[upper_idx] - self.scales[lower_idx])

    def adapt_scale(self, mach: float, target_scale: float, alpha: float) -> None:
        lower_idx = 0
        for i in range(len(MACH_BIN_EDGES) - 1):
            if mach >= MACH_BIN_EDGES[i]:
                lower_idx = i
        upper_idx = min(lower_idx + 1, len(MACH_BIN_EDGES) - 1)
        if lower_idx == upper_idx:
            self.scales[lower_idx] = clamp(
                self.scales[lower_idx] + alpha * (target_scale - self.scales[lower_idx]),
                MACH_DRAG_SCALE_MIN,
                MACH_DRAG_SCALE_MAX,
            )
            return
        lower_mach = MACH_BIN_EDGES[lower_idx]
        upper_mach = MACH_BIN_EDGES[upper_idx]
        denom = upper_mach - lower_mach
        if denom <= 0.0:
            self.scales[lower_idx] = clamp(
                self.scales[lower_idx] + alpha * (target_scale - self.scales[lower_idx]),
                MACH_DRAG_SCALE_MIN,
                MACH_DRAG_SCALE_MAX,
            )
            return
        t = clamp((mach - lower_mach) / denom, 0.0, 1.0)
        w_lower = 1.0 - t
        w_upper = t
        self.scales[lower_idx] = clamp(
            self.scales[lower_idx] + alpha * w_lower * (target_scale - self.scales[lower_idx]),
            MACH_DRAG_SCALE_MIN,
            MACH_DRAG_SCALE_MAX,
        )
        self.scales[upper_idx] = clamp(
            self.scales[upper_idx] + alpha * w_upper * (target_scale - self.scales[upper_idx]),
            MACH_DRAG_SCALE_MIN,
            MACH_DRAG_SCALE_MAX,
        )


class ApogeePredictor:
    def __init__(self, environment: EnvironmentModel, force_table: ForceTable) -> None:
        self.environment = environment
        self.force_table = force_table
        self.axial_drag_scale = 1.0
        self.mach_drag_scale = MachDependentDragScale()

    @staticmethod
    def compute_adaptive_tau(time_to_apogee_s: float) -> float:
        if time_to_apogee_s >= MACH_DRAG_ADAPT_TAU_TRANSITION_START:
            return MACH_DRAG_ADAPT_TAU_SECONDS_EARLY
        if time_to_apogee_s <= MACH_DRAG_ADAPT_TAU_TRANSITION_END:
            return MACH_DRAG_ADAPT_TAU_SECONDS_LATE
        t = (
            (time_to_apogee_s - MACH_DRAG_ADAPT_TAU_TRANSITION_END)
            / (MACH_DRAG_ADAPT_TAU_TRANSITION_START - MACH_DRAG_ADAPT_TAU_TRANSITION_END)
        )
        return MACH_DRAG_ADAPT_TAU_SECONDS_LATE + t * (
            MACH_DRAG_ADAPT_TAU_SECONDS_EARLY - MACH_DRAG_ADAPT_TAU_SECONDS_LATE
        )

    def compute_acceleration(self, state: ApogeeState) -> tuple[float, float, float, float]:
        rel_x = state.vertical_v
        rel_y = state.horizontal_v
        rel_z = 0.0
        wind = self.environment.effective_wind()
        rel_x -= wind[0]
        rel_y -= wind[1]
        rel_z -= wind[2]

        temperature = self.environment.temperature_kelvin(state.altitude_m)
        density_ratio = self.environment.density_ratio(state.altitude_m)
        speed_of_sound = math.sqrt(GAMMA * R_GAS * temperature) if temperature > 0.0 else 0.0
        rel_speed_sq = rel_x * rel_x + rel_y * rel_y + rel_z * rel_z
        mach = math.sqrt(rel_speed_sq) / speed_of_sound if speed_of_sound > 0.0 and rel_speed_sq > 0.0 else 0.0

        gravity_x = -G
        linear_x = gravity_x
        linear_y = 0.0
        angular = 0.0
        axial_linear_x = 0.0

        min_aero_speed = speed_of_sound * 0.025
        if rel_speed_sq < (min_aero_speed * min_aero_speed):
            return linear_x, linear_y, angular, axial_linear_x

        velocity_angle = math.atan2(rel_y, rel_x)
        signed_atk = wrap_to_pi(state.zenith - velocity_angle)
        lift_state = signed_atk >= 0.0
        atk_deg = math.degrees(abs(signed_atk))

        i0, i1, ti = interp_axis(self.force_table.acs_angles, state.acs_deg)
        j0, j1, tj = interp_axis(self.force_table.atk_angles, atk_deg)
        k0, k1, tk = interp_axis(self.force_table.mach_numbers, mach)
        axial_force = trilinear(
            self.force_table.axial_forces,
            i0,
            i1,
            j0,
            j1,
            k0,
            k1,
            ti,
            tj,
            tk,
        )
        normal_force = trilinear(
            self.force_table.normal_forces,
            i0,
            i1,
            j0,
            j1,
            k0,
            k1,
            ti,
            tj,
            tk,
        )

        effective_drag_scale = self.axial_drag_scale
        if ENABLE_MACH_DEPENDENT_DRAG:
            effective_drag_scale *= self.mach_drag_scale.interpolate_scale(mach)
        axial_force_mag = axial_force * effective_drag_scale * density_ratio
        normal_force_mag = normal_force * density_ratio

        sin_z = math.sin(state.zenith)
        cos_z = math.cos(state.zenith)
        axial_force_x = -axial_force_mag * cos_z
        axial_force_y = -axial_force_mag * sin_z
        normal_force_x = -normal_force_mag * sin_z
        normal_force_y = normal_force_mag * cos_z
        angular = (normal_force_mag * (-0.2 * CP_CG_M)) / MOMENT_INERTIA

        if not lift_state:
            normal_force_x = -normal_force_x
            normal_force_y = -normal_force_y
            angular = -angular

        inv_mass = 1.0 / DRY_MASS
        axial_linear_x = axial_force_x * inv_mass
        linear_x = gravity_x + (axial_force_x + normal_force_x) * inv_mass
        linear_y = (axial_force_y + normal_force_y) * inv_mass
        return linear_x, linear_y, angular, axial_linear_x

    def integrate_step(self, state: ApogeeState, method: str) -> ApogeeState:
        if method == "midpoint":
            k1 = self.evaluate(state)
            k2 = self.evaluate(apply_derivative(state, k1, 0.5 * PRED_DT))
            return apply_derivative(state, k2, PRED_DT)

        half_dt = 0.5 * PRED_DT
        sixth_dt = PRED_DT / 6.0
        k1 = self.evaluate(state)
        k2 = self.evaluate(apply_derivative(state, k1, half_dt))
        k3 = self.evaluate(apply_derivative(state, k2, half_dt))
        k4 = self.evaluate(apply_derivative(state, k3, PRED_DT))
        return ApogeeState(
            altitude_m=state.altitude_m + sixth_dt * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]),
            horizontal_m=state.horizontal_m + sixth_dt * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]),
            vertical_v=state.vertical_v + sixth_dt * (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2]),
            horizontal_v=state.horizontal_v + sixth_dt * (k1[3] + 2.0 * k2[3] + 2.0 * k3[3] + k4[3]),
            zenith=state.zenith + sixth_dt * (k1[4] + 2.0 * k2[4] + 2.0 * k3[4] + k4[4]),
            angular_v=state.angular_v + sixth_dt * (k1[5] + 2.0 * k2[5] + 2.0 * k3[5] + k4[5]),
            acs_deg=state.acs_deg,
        )

    def evaluate(self, state: ApogeeState) -> tuple[float, float, float, float, float, float]:
        linear_x, linear_y, angular, _ = self.compute_acceleration(state)
        return (
            state.vertical_v,
            state.horizontal_v,
            linear_x,
            linear_y,
            state.angular_v,
            angular,
        )

    def predict_apogee(self, initial_state: ApogeeState, method: str = "rk4") -> float:
        if initial_state.vertical_v <= 0.0:
            return initial_state.altitude_m
        state = copy_state(initial_state)
        steps = 0
        while state.vertical_v > 0.0 and steps < MAX_STEPS:
            previous_state = copy_state(state)
            state = self.integrate_step(state, method)
            steps += 1
            if previous_state.vertical_v > 0.0 and state.vertical_v <= 0.0:
                return refine_apogee_at_zero_crossing(previous_state, state)
        return state.altitude_m

    def estimate_time_to_apogee(self, initial_state: ApogeeState) -> float:
        if initial_state.vertical_v <= 0.0:
            return 0.0
        state = copy_state(initial_state)
        time_s = 0.0
        steps = 0
        while state.vertical_v > 0.0 and steps < MAX_STEPS:
            state = self.integrate_step(state, "midpoint")
            time_s += PRED_DT
            steps += 1
        return time_s

    def compute_vertical_acceleration(self, state: ApogeeState) -> tuple[float, float]:
        linear_x, _, _, axial_linear_x = self.compute_acceleration(state)
        return linear_x, axial_linear_x

    def adapt_mach_drag_scale(
        self,
        mach: float,
        residual_accel: float,
        model_axial_accel: float,
        dt_seconds: float,
        time_to_apogee_seconds: float,
    ) -> None:
        if abs(model_axial_accel) < ADAPTIVE_AXIAL_ACCEL_MIN_ABS_MPS2:
            return
        current_scale = self.mach_drag_scale.interpolate_scale(mach)
        target_scale = current_scale + (residual_accel / model_axial_accel)
        tau_seconds = self.compute_adaptive_tau(time_to_apogee_seconds)
        alpha = (1.0 - math.exp(-dt_seconds / tau_seconds)) if dt_seconds > 0.0 and tau_seconds > 0.0 else 0.0
        self.mach_drag_scale.adapt_scale(mach, target_scale, alpha)


def parse_float(value: str | None) -> float | None:
    if value is None:
        return None
    text = value.strip()
    if not text:
        return None
    lowered = text.lower()
    if lowered in {"nan", "inf", "-inf"}:
        return None
    return float(text)


def parse_bool(value: str | None) -> bool:
    return (value or "").strip().lower() in {"1", "true", "yes"}


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def decimate_rows(rows: list[dict[str, float | str | bool]], max_points: int) -> list[dict[str, float | str | bool]]:
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
    times: list[float],
    values: list[float],
    t_min: float,
    t_max: float,
    y_min: float,
    y_max: float,
    left: float,
    top: float,
    plot_w: float,
    plot_h: float,
) -> str:
    points: list[str] = []
    bottom = top + plot_h
    for time_s, value in zip(times, values):
        x = scale(time_s, t_min, t_max, left, left + plot_w)
        y = scale(value, y_min, y_max, bottom, top)
        points.append(f"{x:.2f},{y:.2f}")
    return " ".join(points)


def write_svg(output_rows: list[dict[str, float | str | bool]], actual_apogee_m: float, output_svg: Path) -> None:
    sampled = decimate_rows(output_rows, max_points=6000)
    times = [float(row["time_s"]) for row in sampled]
    altitude = [float(row["altitude_m"]) for row in sampled]
    logged = [float(row["logged_apogee_m"]) for row in sampled]
    predicted = [float(row["predicted_apogee_m"]) for row in sampled]
    actual = [actual_apogee_m for _ in sampled]

    width = 1700
    height = 950
    left = 110
    right = 40
    top = 70
    bottom_margin = 80
    plot_w = width - left - right
    plot_h = height - top - bottom_margin

    t_min = min(times)
    t_max = max(times)
    y_min = min(min(altitude), min(logged), min(predicted), actual_apogee_m)
    y_max = max(max(altitude), max(logged), max(predicted), actual_apogee_m)
    y_pad = max(1.0, (y_max - y_min) * 0.05)
    y_min -= y_pad
    y_max += y_pad

    altitude_points = polyline_points(times, altitude, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    logged_points = polyline_points(times, logged, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    predicted_points = polyline_points(times, predicted, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)
    actual_points = polyline_points(times, actual, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h)

    grid_lines: list[str] = []
    for i in range(6):
        x = left + (plot_w * i / 5.0)
        grid_lines.append(
            f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{top + plot_h}" stroke="#d8d8d8" stroke-width="1"/>'
        )
    for i in range(6):
        y = top + (plot_h * i / 5.0)
        grid_lines.append(
            f'<line x1="{left}" y1="{y:.2f}" x2="{left + plot_w}" y2="{y:.2f}" stroke="#d8d8d8" stroke-width="1"/>'
        )

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="white"/>
  <text x="{width / 2:.0f}" y="36" text-anchor="middle" font-family="monospace" font-size="24">Current Firmware Apogee Predictor Replay</text>
  <text x="{width / 2:.0f}" y="{height - 20}" text-anchor="middle" font-family="monospace" font-size="18">Time (s)</text>
  <text x="30" y="{height / 2:.0f}" text-anchor="middle" font-family="monospace" font-size="18" transform="rotate(-90 30 {height / 2:.0f})">Meters</text>
  {''.join(grid_lines)}
  <rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="none" stroke="black" stroke-width="2"/>
  <polyline fill="none" stroke="#1f77b4" stroke-width="1.5" points="{altitude_points}"/>
  <polyline fill="none" stroke="#9467bd" stroke-width="1.5" points="{logged_points}"/>
  <polyline fill="none" stroke="#2ca02c" stroke-width="1.6" points="{predicted_points}"/>
  <polyline fill="none" stroke="#111111" stroke-width="1.2" stroke-dasharray="6,4" points="{actual_points}"/>
  <line x1="{left + 20}" y1="{top + 20}" x2="{left + 90}" y2="{top + 20}" stroke="#1f77b4" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 26}" font-family="monospace" font-size="16">Altitude</text>
  <line x1="{left + 20}" y1="{top + 48}" x2="{left + 90}" y2="{top + 48}" stroke="#9467bd" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 54}" font-family="monospace" font-size="16">Logged Predictor</text>
  <line x1="{left + 20}" y1="{top + 76}" x2="{left + 90}" y2="{top + 76}" stroke="#2ca02c" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 82}" font-family="monospace" font-size="16">Current Model Replay</text>
  <line x1="{left + 20}" y1="{top + 104}" x2="{left + 90}" y2="{top + 104}" stroke="#111111" stroke-width="2" stroke-dasharray="6,4"/>
  <text x="{left + 100}" y="{top + 110}" font-family="monospace" font-size="16">Actual Apogee</text>
</svg>
"""
    output_svg.parent.mkdir(parents=True, exist_ok=True)
    output_svg.write_text(svg)


def predictor_seed_has_fresh_sample(dt_seconds: float) -> bool:
    return math.isfinite(dt_seconds) and dt_seconds > 1.0e-4 and dt_seconds <= 0.25


def sanitize_predictor_zenith_radians(zenith_radians: float) -> float:
    if not math.isfinite(zenith_radians):
        return 0.0
    return clamp(
        zenith_radians,
        -math.radians(PREDICTOR_MAX_SEED_ZENITH_DEG),
        math.radians(PREDICTOR_MAX_SEED_ZENITH_DEG),
    )


def clamp_predictor_angular_rate(angular_rate_rad_per_sec: float) -> float:
    return clamp(
        angular_rate_rad_per_sec,
        -PREDICTOR_MAX_SEED_ANGULAR_RATE_RAD_PER_SEC,
        PREDICTOR_MAX_SEED_ANGULAR_RATE_RAD_PER_SEC,
    )


def compute_predictor_angular_rate(current_zenith: float, previous_zenith: float, dt_seconds: float) -> float:
    if not predictor_seed_has_fresh_sample(dt_seconds):
        return 0.0
    if not math.isfinite(current_zenith) or not math.isfinite(previous_zenith):
        return 0.0
    return (current_zenith - previous_zenith) / dt_seconds


def predictor_horizontal_speed_cap(vertical_velocity_mps: float, zenith_radians: float) -> float:
    effective_zenith = min(abs(zenith_radians), math.radians(PREDICTOR_MAX_SEED_ZENITH_DEG))
    tilt_cap = abs(vertical_velocity_mps) * math.tan(effective_zenith) + PREDICTOR_HORIZONTAL_SPEED_MARGIN_MPS
    return clamp(
        tilt_cap,
        PREDICTOR_MIN_HORIZONTAL_SPEED_CAP_MPS,
        PREDICTOR_MAX_HORIZONTAL_SPEED_MPS,
    )


def predictor_geometric_horizontal_speed(vertical_velocity_mps: float, zenith_radians: float) -> float:
    if not math.isfinite(vertical_velocity_mps) or not math.isfinite(zenith_radians):
        return 0.0
    vertical_speed = abs(vertical_velocity_mps)
    if vertical_speed <= 0.0:
        return 0.0
    speed_along_axis = vertical_speed / clamp(abs(math.cos(zenith_radians)), 0.1, 1.0)
    horizontal_speed_sq = speed_along_axis * speed_along_axis - vertical_speed * vertical_speed
    return math.sqrt(horizontal_speed_sq) if horizontal_speed_sq > 0.0 else 0.0


def resolve_predictor_horizontal_speed(
    tracked_horizontal_speed_mps: float,
    vertical_velocity_mps: float,
    zenith_radians: float,
) -> float:
    cap = predictor_horizontal_speed_cap(vertical_velocity_mps, zenith_radians)
    geometric = predictor_geometric_horizontal_speed(vertical_velocity_mps, zenith_radians)
    return clamp(max(tracked_horizontal_speed_mps, geometric), 0.0, cap)


def update_predictor_horizontal_speed(
    tracker: PredictorHorizontalVelocityTracker,
    accel_x_mps2: float,
    accel_y_mps2: float,
    dt_seconds: float,
    allow_integration: bool,
    vertical_velocity_mps: float,
    zenith_radians: float,
) -> float:
    if dt_seconds <= 0.0:
        return math.hypot(tracker.vx, tracker.vy)
    clamped_dt = min(dt_seconds, 0.25)
    ax = clamp(accel_x_mps2, -PREDICTOR_HORIZONTAL_ACCEL_LIMIT_MPS2, PREDICTOR_HORIZONTAL_ACCEL_LIMIT_MPS2)
    ay = clamp(accel_y_mps2, -PREDICTOR_HORIZONTAL_ACCEL_LIMIT_MPS2, PREDICTOR_HORIZONTAL_ACCEL_LIMIT_MPS2)
    decay = math.exp(-clamped_dt / PREDICTOR_HORIZONTAL_DECAY_TAU_SECONDS)

    if allow_integration and vertical_velocity_mps > 0.0:
        tracker.vx = (tracker.vx + ax * clamped_dt) * decay
        tracker.vy = (tracker.vy + ay * clamped_dt) * decay
    else:
        tracker.vx *= decay
        tracker.vy *= decay

    speed_sq = tracker.vx * tracker.vx + tracker.vy * tracker.vy
    cap = predictor_horizontal_speed_cap(vertical_velocity_mps, zenith_radians)
    if speed_sq > (cap * cap) and speed_sq > 1.0e-18:
        speed = math.sqrt(speed_sq)
        scale = cap / speed
        tracker.vx *= scale
        tracker.vy *= scale
        return cap
    return math.sqrt(speed_sq)


def apply_derivative(state: ApogeeState, derivative: tuple[float, float, float, float, float, float], dt: float) -> ApogeeState:
    return ApogeeState(
        altitude_m=state.altitude_m + derivative[0] * dt,
        horizontal_m=state.horizontal_m + derivative[1] * dt,
        vertical_v=state.vertical_v + derivative[2] * dt,
        horizontal_v=state.horizontal_v + derivative[3] * dt,
        zenith=state.zenith + derivative[4] * dt,
        angular_v=state.angular_v + derivative[5] * dt,
        acs_deg=state.acs_deg,
    )


def copy_state(state: ApogeeState) -> ApogeeState:
    return ApogeeState(**state.__dict__)


def refine_apogee_at_zero_crossing(ascending_state: ApogeeState, descending_state: ApogeeState) -> float:
    previous_vz = ascending_state.vertical_v
    current_vz = descending_state.vertical_v
    vz_delta = previous_vz - current_vz
    if not math.isfinite(previous_vz) or not math.isfinite(current_vz) or abs(vz_delta) < 1.0e-9:
        return max(ascending_state.altitude_m, descending_state.altitude_m)
    alpha = clamp(previous_vz / vz_delta, 0.0, 1.0)
    refined = ascending_state.altitude_m + alpha * (descending_state.altitude_m - ascending_state.altitude_m)
    return refined if math.isfinite(refined) else max(ascending_state.altitude_m, descending_state.altitude_m)


def load_replay_rows(path: Path) -> list[ReplayRow]:
    rows: list[ReplayRow] = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            if not parse_bool(raw.get("has_filtered_state")):
                continue
            status = (raw.get("flight_status") or "").strip().lower()
            time_s = parse_float(raw.get("state_time"))
            altitude_m = parse_float(raw.get("state_position_z"))
            vertical_velocity_mps = parse_float(raw.get("state_velocity_z"))
            measured_accel_z_mps2 = parse_float(raw.get("state_acceleration_z"))
            inertial_ax_mps2 = parse_float(raw.get("state_inertial_acceleration_x"))
            inertial_ay_mps2 = parse_float(raw.get("state_inertial_acceleration_y"))
            zenith_rad = parse_float(raw.get("state_zenith"))
            logged_apogee_m = parse_float(raw.get("state_apogee_estimate"))
            if None in (
                time_s,
                altitude_m,
                vertical_velocity_mps,
                measured_accel_z_mps2,
                inertial_ax_mps2,
                inertial_ay_mps2,
                zenith_rad,
                logged_apogee_m,
            ):
                continue
            if status not in {"burn", "coast"} or vertical_velocity_mps <= 0.0:
                continue
            rows.append(
                ReplayRow(
                    time_s=time_s,
                    status=status,
                    altitude_m=altitude_m,
                    vertical_velocity_mps=vertical_velocity_mps,
                    measured_accel_z_mps2=measured_accel_z_mps2,
                    inertial_ax_mps2=inertial_ax_mps2,
                    inertial_ay_mps2=inertial_ay_mps2,
                    zenith_rad=zenith_rad,
                    has_quaternion=parse_bool(raw.get("sensor_has_quaternion")),
                    logged_apogee_m=logged_apogee_m,
                )
            )
    return rows


def compute_actual_apogee_m(path: Path) -> float:
    maximum = -math.inf
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            altitude_m = parse_float(raw.get("state_position_z"))
            if altitude_m is not None:
                maximum = max(maximum, altitude_m)
    return maximum if maximum > -math.inf else math.nan


def run(
    input_csv: Path,
    force_table_csv: Path,
    output_csv: Path,
    output_svg: Path,
    stride: int,
    max_rows: int | None,
) -> None:
    rows = load_replay_rows(input_csv)
    if stride > 1:
        rows = rows[::stride]
    if max_rows is not None:
        rows = rows[:max_rows]
    force_table = load_force_table(force_table_csv)
    environment = EnvironmentModel()
    predictor = ApogeePredictor(environment, force_table)
    tracker = PredictorHorizontalVelocityTracker()

    output_csv.parent.mkdir(parents=True, exist_ok=True)
    actual_apogee_m = compute_actual_apogee_m(input_csv)

    previous_time_s: float | None = None
    previous_zenith_rad: float | None = None

    output_rows: list[dict[str, float | str | bool]] = []
    for row in rows:
        dt_seconds = 0.0 if previous_time_s is None else (row.time_s - previous_time_s)
        fresh_seed_sample = predictor_seed_has_fresh_sample(dt_seconds)
        quaternion_valid = row.has_quaternion
        seed_zenith = sanitize_predictor_zenith_radians(row.zenith_rad) if quaternion_valid else 0.0
        can_use_horizontal_seed = quaternion_valid and fresh_seed_sample
        if not can_use_horizontal_seed:
            tracker = PredictorHorizontalVelocityTracker()
        tracked_horizontal_velocity = (
            update_predictor_horizontal_speed(
                tracker,
                row.inertial_ax_mps2,
                row.inertial_ay_mps2,
                dt_seconds,
                True,
                row.vertical_velocity_mps,
                seed_zenith,
            )
            if can_use_horizontal_seed
            else 0.0
        )
        predictor_horizontal_velocity = (
            resolve_predictor_horizontal_speed(tracked_horizontal_velocity, row.vertical_velocity_mps, seed_zenith)
            if can_use_horizontal_seed
            else 0.0
        )
        predictor_angular_rate = (
            clamp_predictor_angular_rate(
                compute_predictor_angular_rate(
                    row.zenith_rad,
                    previous_zenith_rad if previous_zenith_rad is not None else row.zenith_rad,
                    dt_seconds,
                )
            )
            if can_use_horizontal_seed
            else 0.0
        )

        state = ApogeeState(
            altitude_m=row.altitude_m,
            horizontal_m=0.0,
            vertical_v=row.vertical_velocity_mps,
            horizontal_v=predictor_horizontal_velocity,
            zenith=seed_zenith,
            angular_v=predictor_angular_rate,
            acs_deg=0.0,
        )
        time_to_apogee_s = predictor.estimate_time_to_apogee(state) if state.vertical_v > 0.0 else 0.0

        predicted_vertical_accel, axial_model_accel = predictor.compute_vertical_acceleration(state)
        residual = clamp(
            row.measured_accel_z_mps2 - predicted_vertical_accel,
            -ADAPTIVE_AXIAL_DRAG_RESIDUAL_CLAMP_MPS2,
            ADAPTIVE_AXIAL_DRAG_RESIDUAL_CLAMP_MPS2,
        )

        mach = 0.0
        temp_k = environment.temperature_kelvin(state.altitude_m)
        if temp_k > 0.0:
            speed_of_sound = math.sqrt(GAMMA * R_GAS * temp_k)
            total_speed = math.hypot(state.vertical_v, state.horizontal_v)
            mach = total_speed / speed_of_sound if speed_of_sound > 0.0 else 0.0

        if (
            row.status == "coast"
            and state.vertical_v > 0.0
            and quaternion_valid
            and fresh_seed_sample
            and math.isfinite(row.measured_accel_z_mps2)
            and math.isfinite(state.zenith)
            and ENABLE_MACH_DEPENDENT_DRAG
        ):
            predictor.adapt_mach_drag_scale(
                mach,
                residual,
                axial_model_accel,
                dt_seconds,
                max(0.0, time_to_apogee_s),
            )
            predictor.axial_drag_scale = 1.0
        elif (
            row.status == "coast"
            and state.vertical_v > 0.0
            and quaternion_valid
            and fresh_seed_sample
            and math.isfinite(row.measured_accel_z_mps2)
            and math.isfinite(state.zenith)
            and abs(axial_model_accel) >= ADAPTIVE_AXIAL_ACCEL_MIN_ABS_MPS2
        ):
            target_scale = clamp(
                predictor.axial_drag_scale + (residual / axial_model_accel),
                ADAPTIVE_AXIAL_DRAG_SCALE_MIN,
                ADAPTIVE_AXIAL_DRAG_SCALE_MAX,
            )
            alpha = (
                1.0 - math.exp(-dt_seconds / ADAPTIVE_AXIAL_DRAG_TAU_SECONDS)
                if dt_seconds > 0.0 and ADAPTIVE_AXIAL_DRAG_TAU_SECONDS > 0.0
                else 0.0
            )
            predictor.axial_drag_scale = clamp(
                predictor.axial_drag_scale + alpha * (target_scale - predictor.axial_drag_scale),
                ADAPTIVE_AXIAL_DRAG_SCALE_MIN,
                ADAPTIVE_AXIAL_DRAG_SCALE_MAX,
            )

        predicted_apogee_m = predictor.predict_apogee(state, "rk4") if state.vertical_v > 0.0 else row.altitude_m
        output_rows.append(
            {
                "time_s": row.time_s,
                "status": row.status,
                "has_quaternion": quaternion_valid,
                "fresh_seed_sample": fresh_seed_sample,
                "can_use_horizontal_seed": can_use_horizontal_seed,
                "altitude_m": row.altitude_m,
                "vertical_velocity_mps": row.vertical_velocity_mps,
                "measured_accel_z_mps2": row.measured_accel_z_mps2,
                "predicted_accel_z_mps2": predicted_vertical_accel,
                "axial_model_accel_mps2": axial_model_accel,
                "residual_accel_mps2": residual,
                "logged_apogee_m": row.logged_apogee_m,
                "predicted_apogee_m": predicted_apogee_m,
                "prediction_error_m": predicted_apogee_m - actual_apogee_m,
                "time_to_apogee_s": time_to_apogee_s,
                "seed_zenith_deg": math.degrees(seed_zenith),
                "predictor_horizontal_velocity_mps": predictor_horizontal_velocity,
                "predictor_angular_rate_rad_s": predictor_angular_rate,
                "mach": mach,
                "legacy_axial_drag_scale": predictor.axial_drag_scale,
                "mach_drag_scale_interpolated": predictor.mach_drag_scale.interpolate_scale(mach),
                "mach_drag_scale_bin_0": predictor.mach_drag_scale.scales[0],
                "mach_drag_scale_bin_1": predictor.mach_drag_scale.scales[1],
                "mach_drag_scale_bin_2": predictor.mach_drag_scale.scales[2],
                "mach_drag_scale_bin_3": predictor.mach_drag_scale.scales[3],
            }
        )

        previous_time_s = row.time_s
        previous_zenith_rad = row.zenith_rad

    with output_csv.open("w", newline="") as handle:
        fieldnames = list(output_rows[0].keys()) if output_rows else []
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(output_rows)
    if output_rows:
        write_svg(output_rows, actual_apogee_m, output_svg)

    coast_rows = [row for row in output_rows if row["status"] == "coast"]
    if coast_rows:
        mean_abs_error = sum(abs(float(row["prediction_error_m"])) for row in coast_rows) / len(coast_rows)
        print(f"rows={len(output_rows)} coast_rows={len(coast_rows)} actual_apogee_m={actual_apogee_m:.3f}")
        print(f"coast_mean_abs_error_m={mean_abs_error:.3f}")
        print(
            "final_mach_scales="
            + ",".join(f"{value:.4f}" for value in predictor.mach_drag_scale.scales)
        )
    else:
        print(f"rows={len(output_rows)} actual_apogee_m={actual_apogee_m:.3f}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Replay the current firmware apogee predictor model against tools/replay/data/output.csv."
    )
    parser.add_argument("--input", type=Path, default=INPUT_CSV, help="Replay CSV to read.")
    parser.add_argument("--cfd", type=Path, default=CFD_CSV, help="CFD force table CSV.")
    parser.add_argument("--output", type=Path, default=OUTPUT_CSV, help="Output CSV for replayed predictions.")
    parser.add_argument("--svg", type=Path, default=OUTPUT_SVG, help="SVG plot output path.")
    parser.add_argument("--stride", type=int, default=1, help="Replay every Nth active predictor row.")
    parser.add_argument("--max-rows", type=int, default=None, help="Optional cap on processed active rows.")
    args = parser.parse_args()

    run(args.input, args.cfd, args.output, args.svg, max(1, args.stride), args.max_rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
