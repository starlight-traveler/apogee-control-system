from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple


G = 9.8067
GAMMA = 1.4
R_GAS = 287.05
METERS_TO_FEET = 3.28083989501
MPH_TO_MS = 0.44704

GROUND_TEMP_F = 32.0
WIND_SPEED_MPH = 8.0
WIND_DIR_DEG = 63.0
LAUNCH_DIR_DEG = 63.0
ROUGHNESS_M = 0.075
GRADIENT_HEIGHT_M = 300.0
MEASUREMENT_HEIGHT_M = 10.0

CP_CG_M = 0.4389
MOMENT_INERTIA = 8.28
DRY_MASS = 18.09975

PRED_DT = 0.1
MAX_STEPS = 256


@dataclass
class ReplayRow:
    time_s: float
    status: str
    altitude_m: float
    vertical_velocity_mps: float
    inertial_ax_mps2: float
    inertial_ay_mps2: float
    zenith_rad: float
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
class PredictionTraceRow:
    time_s: float
    status: str
    altitude_m: float
    vertical_velocity_mps: float
    logged_apogee_m: float
    legacy_apogee_m: float
    safer_apogee_m: float
    legacy_horizontal_speed_mps: float
    safer_horizontal_speed_mps: float
    zenith_deg: float


@dataclass
class SaferSeedConfig:
    accel_limit_mps2: float = 12.0
    horizontal_decay_tau_s: float = 1.75
    max_seed_zenith_deg: float = 20.0
    max_seed_rate_rad_s: float = 1.5
    max_horizontal_speed_mps: float = 65.0
    min_horizontal_cap_mps: float = 6.0
    horizontal_margin_mps: float = 3.0


class ForceTable:
    def __init__(
        self,
        acs_angles: Sequence[float],
        atk_angles: Sequence[float],
        mach_numbers: Sequence[float],
        axial_forces: Sequence[Sequence[Sequence[float]]],
        normal_forces: Sequence[Sequence[Sequence[float]]],
    ) -> None:
        self.acs_angles = list(acs_angles)
        self.atk_angles = list(atk_angles)
        self.mach_numbers = list(mach_numbers)
        self.axial_forces = axial_forces
        self.normal_forces = normal_forces


class HorizontalVelocityTracker:
    def __init__(self, config: SaferSeedConfig | None = None) -> None:
        self.config = config or SaferSeedConfig()
        self.vx = 0.0
        self.vy = 0.0
        self.last_time_s: Optional[float] = None

    def update(self, row: ReplayRow) -> float:
        if self.last_time_s is None:
            self.last_time_s = row.time_s
            return 0.0

        dt = row.time_s - self.last_time_s
        self.last_time_s = row.time_s
        if dt <= 0.0:
            return math.hypot(self.vx, self.vy)
        if dt > 0.25:
            dt = 0.25

        decay = math.exp(-dt / max(self.config.horizontal_decay_tau_s, 1.0e-3))
        ax = clamp(row.inertial_ax_mps2, -self.config.accel_limit_mps2, self.config.accel_limit_mps2)
        ay = clamp(row.inertial_ay_mps2, -self.config.accel_limit_mps2, self.config.accel_limit_mps2)

        if row.status in ("ground", "descent") or row.vertical_velocity_mps <= 0.0:
            self.vx *= decay
            self.vy *= decay
        else:
            self.vx = (self.vx + ax * dt) * decay
            self.vy = (self.vy + ay * dt) * decay

        cap = horizontal_speed_cap(row, self.config)
        mag = math.hypot(self.vx, self.vy)
        if mag > cap and mag > 1.0e-6:
            scale = cap / mag
            self.vx *= scale
            self.vy *= scale
            mag = cap
        return mag


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def temperature_k(alt_m: float) -> float:
    alt_ft = alt_m * METERS_TO_FEET
    temp_f = GROUND_TEMP_F - 0.00356 * alt_ft
    return (temp_f - 32.0) / 1.8 + 273.15


def gradient_wind() -> Tuple[float, float, float]:
    wind_speed_ms = WIND_SPEED_MPH * MPH_TO_MS
    wind_dir = math.radians(WIND_DIR_DEG)
    launch_dir = math.radians(LAUNCH_DIR_DEG)
    wind_vec_x = wind_speed_ms * math.cos(wind_dir)
    wind_vec_y = wind_speed_ms * math.sin(wind_dir)
    launch_unit_x = math.cos(launch_dir)
    launch_unit_y = math.sin(launch_dir)
    wind_downrange = wind_vec_x * launch_unit_x + wind_vec_y * launch_unit_y
    numerator = math.log(GRADIENT_HEIGHT_M / ROUGHNESS_M)
    denominator = math.log(MEASUREMENT_HEIGHT_M / ROUGHNESS_M)
    gradient_speed = wind_downrange * numerator / denominator if denominator != 0.0 else 0.0
    return (0.0, gradient_speed, 0.0)


def interp_axis(grid: Sequence[float], value: float) -> Tuple[int, int, float]:
    if value <= grid[0]:
        low, high = 0, 1
    elif value >= grid[-1]:
        low, high = len(grid) - 2, len(grid) - 1
    else:
        lo, hi = 0, len(grid) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if grid[mid] <= value:
                lo = mid
            else:
                hi = mid
        low, high = lo, lo + 1
    denom = grid[high] - grid[low]
    t = clamp((value - grid[low]) / denom, 0.0, 1.0) if denom != 0.0 else 0.0
    return low, high, t


def trilinear(values, i0, i1, j0, j1, k0, k1, ti, tj, tk) -> float:
    v000 = values[i0][j0][k0]
    v100 = values[i1][j0][k0]
    v010 = values[i0][j1][k0]
    v110 = values[i1][j1][k0]
    v001 = values[i0][j0][k1]
    v101 = values[i1][j0][k1]
    v011 = values[i0][j1][k1]
    v111 = values[i1][j1][k1]
    v00 = v000 + (v100 - v000) * ti
    v10 = v010 + (v110 - v010) * ti
    v01 = v001 + (v101 - v001) * ti
    v11 = v011 + (v111 - v011) * ti
    v0 = v00 + (v10 - v00) * tj
    v1 = v01 + (v11 - v01) * tj
    return v0 + (v1 - v0) * tk


def load_force_table(path: Path) -> ForceTable:
    acs_set, atk_set, mach_set = set(), set(), set()
    rows = []
    with path.open(newline="") as handle:
        reader = csv.reader(handle)
        next(reader, None)
        for row in reader:
            if len(row) < 5:
                continue
            acs, atk, mach, axial, normal = map(float, row[:5])
            acs_set.add(acs)
            atk_set.add(atk)
            mach_set.add(mach)
            rows.append((acs, atk, mach, axial, normal))

    acs_list = sorted(acs_set)
    atk_list = sorted(atk_set)
    mach_list = sorted(mach_set)
    axial = [[[math.nan for _ in mach_list] for _ in atk_list] for _ in acs_list]
    normal = [[[math.nan for _ in mach_list] for _ in atk_list] for _ in acs_list]
    idx_acs = {v: i for i, v in enumerate(acs_list)}
    idx_atk = {v: i for i, v in enumerate(atk_list)}
    idx_mach = {v: i for i, v in enumerate(mach_list)}
    for acs, atk, mach, axial_f, normal_f in rows:
        axial[idx_acs[acs]][idx_atk[atk]][idx_mach[mach]] = axial_f
        normal[idx_acs[acs]][idx_atk[atk]][idx_mach[mach]] = normal_f
    return ForceTable(acs_list, atk_list, mach_list, axial, normal)


def compute_accel(state: ApogeeState, wind: Tuple[float, float, float], table: ForceTable) -> Tuple[float, float, float]:
    rel_x = state.vertical_v - wind[0]
    rel_y = state.horizontal_v - wind[1]
    temp_k = temperature_k(state.altitude_m)
    speed_of_sound = math.sqrt(GAMMA * R_GAS * temp_k) if temp_k > 0.0 else 0.0
    mach = math.hypot(rel_x, rel_y) / speed_of_sound if speed_of_sound > 0.0 else 0.0

    gravity_x = -G
    gravity_y = 0.0
    linear_x = gravity_x
    linear_y = gravity_y
    angular = 0.0

    if mach >= 0.025:
        velocity_angle = math.atan2(rel_y, rel_x)
        signed_atk = wrap_to_pi(state.zenith - velocity_angle)
        lift_state = signed_atk >= 0.0
        atk_deg = math.degrees(abs(signed_atk))

        i0, i1, ti = interp_axis(table.acs_angles, state.acs_deg)
        j0, j1, tj = interp_axis(table.atk_angles, atk_deg)
        k0, k1, tk = interp_axis(table.mach_numbers, mach)

        axial_force = trilinear(table.axial_forces, i0, i1, j0, j1, k0, k1, ti, tj, tk)
        normal_force = trilinear(table.normal_forces, i0, i1, j0, j1, k0, k1, ti, tj, tk)

        sin_z = math.sin(state.zenith)
        cos_z = math.cos(state.zenith)
        axial_x = -axial_force * cos_z
        axial_y = -axial_force * sin_z
        normal_x = -normal_force * sin_z
        normal_y = normal_force * cos_z
        angular = (-normal_force * CP_CG_M * 0.2) / MOMENT_INERTIA

        if not lift_state:
            normal_x = -normal_x
            normal_y = -normal_y
            angular = -angular

        inv_mass = 1.0 / DRY_MASS
        linear_x = gravity_x + (axial_x + normal_x) * inv_mass
        linear_y = gravity_y + (axial_y + normal_y) * inv_mass

    return linear_x, linear_y, angular


def predict_apogee(state: ApogeeState, table: ForceTable) -> float:
    if state.vertical_v <= 0.0:
        return state.altitude_m
    wind = gradient_wind()
    current = ApogeeState(**state.__dict__)
    steps = 0
    while current.vertical_v > 0.0 and steps < MAX_STEPS:
        k1 = evaluate_derivative(current, wind, table)
        k2 = evaluate_derivative(apply_step(current, k1, 0.5 * PRED_DT), wind, table)
        k3 = evaluate_derivative(apply_step(current, k2, 0.5 * PRED_DT), wind, table)
        k4 = evaluate_derivative(apply_step(current, k3, PRED_DT), wind, table)
        integrate_rk4(current, k1, k2, k3, k4)
        steps += 1
    return current.altitude_m


def evaluate_derivative(state: ApogeeState, wind, table) -> Tuple[float, float, float, float, float, float]:
    ax, ay, ang = compute_accel(state, wind, table)
    return (state.vertical_v, state.horizontal_v, ax, ay, state.angular_v, ang)


def apply_step(state: ApogeeState, deriv, dt: float) -> ApogeeState:
    return ApogeeState(
        altitude_m=state.altitude_m + deriv[0] * dt,
        horizontal_m=state.horizontal_m + deriv[1] * dt,
        vertical_v=state.vertical_v + deriv[2] * dt,
        horizontal_v=state.horizontal_v + deriv[3] * dt,
        zenith=state.zenith + deriv[4] * dt,
        angular_v=state.angular_v + deriv[5] * dt,
        acs_deg=state.acs_deg,
    )


def integrate_rk4(state: ApogeeState, k1, k2, k3, k4) -> None:
    sixth = PRED_DT / 6.0
    state.altitude_m += sixth * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0])
    state.horizontal_m += sixth * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1])
    state.vertical_v += sixth * (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2])
    state.horizontal_v += sixth * (k1[3] + 2.0 * k2[3] + 2.0 * k3[3] + k4[3])
    state.zenith += sixth * (k1[4] + 2.0 * k2[4] + 2.0 * k3[4] + k4[4])
    state.angular_v += sixth * (k1[5] + 2.0 * k2[5] + 2.0 * k3[5] + k4[5])


def legacy_horizontal_speed(vertical_velocity_mps: float, zenith_rad: float) -> float:
    cos_zenith = math.cos(zenith_rad)
    clamped_cos = clamp(cos_zenith, 0.1, 1.0)
    speed_along_axis = vertical_velocity_mps / clamped_cos
    horizontal_sq = max(0.0, speed_along_axis * speed_along_axis - vertical_velocity_mps * vertical_velocity_mps)
    return math.sqrt(horizontal_sq)


def horizontal_speed_cap(row: ReplayRow, config: SaferSeedConfig) -> float:
    effective_zenith = min(abs(row.zenith_rad), math.radians(config.max_seed_zenith_deg))
    tilt_cap = abs(row.vertical_velocity_mps) * math.tan(effective_zenith) + config.horizontal_margin_mps
    return clamp(tilt_cap, config.min_horizontal_cap_mps, config.max_horizontal_speed_mps)


def build_seed_legacy(row: ReplayRow, angular_rate_rad_s: float) -> Tuple[ApogeeState, float]:
    horizontal_v = legacy_horizontal_speed(row.vertical_velocity_mps, row.zenith_rad)
    return (
        ApogeeState(
            altitude_m=row.altitude_m,
            horizontal_m=0.0,
            vertical_v=row.vertical_velocity_mps,
            horizontal_v=horizontal_v,
            zenith=row.zenith_rad,
            angular_v=angular_rate_rad_s,
            acs_deg=0.0,
        ),
        horizontal_v,
    )


def build_seed_safer(
    row: ReplayRow,
    angular_rate_rad_s: float,
    horizontal_v_mps: float,
    config: SaferSeedConfig,
) -> ApogeeState:
    capped_horizontal_v = min(horizontal_speed_cap(row, config), config.max_horizontal_speed_mps)
    return ApogeeState(
        altitude_m=row.altitude_m,
        horizontal_m=0.0,
        vertical_v=row.vertical_velocity_mps,
        horizontal_v=clamp(horizontal_v_mps, 0.0, capped_horizontal_v),
        zenith=clamp(
            row.zenith_rad,
            -math.radians(config.max_seed_zenith_deg),
            math.radians(config.max_seed_zenith_deg),
        ),
        angular_v=clamp(angular_rate_rad_s, -config.max_seed_rate_rad_s, config.max_seed_rate_rad_s),
        acs_deg=0.0,
    )


def parse_replay_rows(csv_path: Path) -> List[ReplayRow]:
    rows: List[ReplayRow] = []
    with csv_path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            if raw.get("has_filtered_state") != "True":
                continue
            rows.append(
                ReplayRow(
                    time_s=float(raw["state_time"]),
                    status=raw["flight_status"],
                    altitude_m=float(raw["state_position_z"]),
                    vertical_velocity_mps=float(raw["state_velocity_z"]),
                    inertial_ax_mps2=float(raw["state_inertial_acceleration_x"]),
                    inertial_ay_mps2=float(raw["state_inertial_acceleration_y"]),
                    zenith_rad=float(raw["state_zenith"]),
                    logged_apogee_m=float(raw["state_apogee_estimate"]),
                )
            )
    return rows


def simulate_predictions(
    rows: Sequence[ReplayRow],
    table: ForceTable,
    config: SaferSeedConfig | None = None,
    sample_stride: int = 25,
) -> List[PredictionTraceRow]:
    cfg = config or SaferSeedConfig()
    tracker = HorizontalVelocityTracker(cfg)
    trace: List[PredictionTraceRow] = []
    last_time: Optional[float] = None
    last_zenith: Optional[float] = None

    for index, row in enumerate(rows):
        safer_horizontal_v = tracker.update(row)
        if row.status not in ("burn", "coast"):
            last_time = row.time_s
            last_zenith = row.zenith_rad
            continue
        if sample_stride > 1 and (index % sample_stride) != 0:
            last_time = row.time_s
            last_zenith = row.zenith_rad
            continue
        if row.vertical_velocity_mps <= 0.0:
            last_time = row.time_s
            last_zenith = row.zenith_rad
            continue

        angular_rate = 0.0
        if last_time is not None and last_zenith is not None:
            dt = row.time_s - last_time
            if dt > 1.0e-4:
                angular_rate = (row.zenith_rad - last_zenith) / dt

        legacy_seed, legacy_horizontal_v = build_seed_legacy(row, angular_rate)
        safer_seed = build_seed_safer(row, angular_rate, safer_horizontal_v, cfg)
        trace.append(
            PredictionTraceRow(
                time_s=row.time_s,
                status=row.status,
                altitude_m=row.altitude_m,
                vertical_velocity_mps=row.vertical_velocity_mps,
                logged_apogee_m=row.logged_apogee_m,
                legacy_apogee_m=predict_apogee(legacy_seed, table),
                safer_apogee_m=predict_apogee(safer_seed, table),
                legacy_horizontal_speed_mps=legacy_horizontal_v,
                safer_horizontal_speed_mps=safer_horizontal_v,
                zenith_deg=math.degrees(row.zenith_rad),
            )
        )

        last_time = row.time_s
        last_zenith = row.zenith_rad

    return trace


def compute_actual_apogee(rows: Iterable[ReplayRow]) -> float:
    return max(row.altitude_m for row in rows)
