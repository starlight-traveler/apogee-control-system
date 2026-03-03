#!/usr/bin/env python3

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple


ROOT = Path(__file__).resolve().parents[1]
INPUT_CSV = ROOT / "standalone-c" / "fullscale_2.csv"
CFD_CSV = ROOT / "lib" / "cfd.csv"
OUTPUT_CSV = ROOT / "standalone-c" / "fullscale_2_option1_prediction.csv"


# Constants (match src/constants.h and src/settings.h)
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
class ApogeeState:
    altitude_m: float
    horizontal_m: float
    vertical_v: float
    horizontal_v: float
    zenith: float
    angular_v: float
    acs_deg: float = 0.0


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


def interp_axis(grid: List[float], value: float) -> Tuple[int, int, float]:
    if value <= grid[0]:
        low = 0
        high = 1
    elif value >= grid[-1]:
        low = len(grid) - 2
        high = len(grid) - 1
    else:
        high = 1
        low = 0
        # binary search
        lo = 0
        hi = len(grid) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if grid[mid] <= value:
                lo = mid
            else:
                hi = mid
        low = lo
        high = low + 1
    denom = grid[high] - grid[low]
    t = (value - grid[low]) / denom if denom != 0.0 else 0.0
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


def load_cfd_table(path: Path):
    acs_set, atk_set, mach_set = set(), set(), set()
    rows = []
    with path.open(newline="") as f:
        r = csv.reader(f)
        next(r, None)
        for row in r:
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
        i = idx_acs[acs]
        j = idx_atk[atk]
        k = idx_mach[mach]
        axial[i][j][k] = axial_f
        normal[i][j][k] = normal_f
    return acs_list, atk_list, mach_list, axial, normal


def compute_accel(state: ApogeeState, wind: Tuple[float, float, float], table) -> Tuple[float, float, float]:
    acs_list, atk_list, mach_list, axial, normal = table
    vel_x = state.vertical_v
    vel_y = state.horizontal_v
    rel_x = vel_x - wind[0]
    rel_y = vel_y - wind[1]
    temp_k = temperature_k(state.altitude_m)
    speed_of_sound = math.sqrt(GAMMA * R_GAS * temp_k) if temp_k > 0.0 else 0.0
    mach = math.sqrt(rel_x * rel_x + rel_y * rel_y) / speed_of_sound if speed_of_sound > 0.0 else 0.0

    gravity_x = -G
    gravity_y = 0.0
    linear_x = gravity_x
    linear_y = gravity_y
    angular = 0.0

    if mach >= 0.025:
        velocity_angle = math.atan2(rel_y, rel_x)
        signed_atk = wrap_to_pi(state.zenith - velocity_angle)
        lift_state = signed_atk >= 0.0
        atk_angle = abs(signed_atk)
        atk_deg = math.degrees(atk_angle)
        acs_deg = state.acs_deg

        i0, i1, ti = interp_axis(acs_list, acs_deg)
        j0, j1, tj = interp_axis(atk_list, atk_deg)
        k0, k1, tk = interp_axis(mach_list, mach)

        axial_force = trilinear(axial, i0, i1, j0, j1, k0, k1, ti, tj, tk)
        normal_force = trilinear(normal, i0, i1, j0, j1, k0, k1, ti, tj, tk)

        sin_z = math.sin(state.zenith)
        cos_z = math.cos(state.zenith)
        axial_x = -axial_force * cos_z
        axial_y = -axial_force * sin_z
        normal_x = -normal_force * sin_z
        normal_y = normal_force * cos_z
        aero_moment = -normal_force * CP_CG_M
        aero_moment *= 0.2
        angular = aero_moment / MOMENT_INERTIA

        if not lift_state:
            normal_x = -normal_x
            normal_y = -normal_y
            angular = -angular

        inv_mass = 1.0 / DRY_MASS
        aero_x = (axial_x + normal_x) * inv_mass
        aero_y = (axial_y + normal_y) * inv_mass
        linear_x = gravity_x + aero_x
        linear_y = gravity_y + aero_y

    return linear_x, linear_y, angular


def predict_apogee(state: ApogeeState, wind, table) -> float:
    if state.vertical_v <= 0.0:
        return state.altitude_m
    s = ApogeeState(**state.__dict__)
    steps = 0
    while s.vertical_v > 0.0 and steps < MAX_STEPS:
        def eval_deriv(st: ApogeeState):
            ax, ay, ang = compute_accel(st, wind, table)
            return (st.vertical_v, st.horizontal_v, ax, ay, st.angular_v, ang)

        k1 = eval_deriv(s)
        def apply(st, k, dt):
            return ApogeeState(
                altitude_m=st.altitude_m + k[0] * dt,
                horizontal_m=st.horizontal_m + k[1] * dt,
                vertical_v=st.vertical_v + k[2] * dt,
                horizontal_v=st.horizontal_v + k[3] * dt,
                zenith=st.zenith + k[4] * dt,
                angular_v=st.angular_v + k[5] * dt,
                acs_deg=st.acs_deg,
            )

        half = 0.5 * PRED_DT
        k2 = eval_deriv(apply(s, k1, half))
        k3 = eval_deriv(apply(s, k2, half))
        k4 = eval_deriv(apply(s, k3, PRED_DT))

        s.altitude_m += (PRED_DT / 6.0) * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0])
        s.horizontal_m += (PRED_DT / 6.0) * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1])
        s.vertical_v += (PRED_DT / 6.0) * (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2])
        s.horizontal_v += (PRED_DT / 6.0) * (k1[3] + 2.0 * k2[3] + 2.0 * k3[3] + k4[3])
        s.zenith += (PRED_DT / 6.0) * (k1[4] + 2.0 * k2[4] + 2.0 * k3[4] + k4[4])
        s.angular_v += (PRED_DT / 6.0) * (k1[5] + 2.0 * k2[5] + 2.0 * k3[5] + k4[5])
        steps += 1
    return s.altitude_m


def main() -> None:
    table = load_cfd_table(CFD_CSV)
    wind = gradient_wind()
    last_apg = 0.0
    last_zenith = None
    last_time = None
    last_predict_time = None

    with INPUT_CSV.open(newline="") as fin, OUTPUT_CSV.open("w", newline="") as fout:
        r = csv.DictReader(fin)
        w = csv.writer(fout)
        w.writerow(["time_s", "altitude_m", "velocity_mps", "apogee_prediction_m", "status"])
        for row in r:
            if row.get("has_filtered_state") != "True":
                continue
            try:
                t = float(row["state_time"])
                z = float(row["state_position_z"])
                vz = float(row["state_velocity_z"])
                zenith = float(row["state_zenith"])
            except (TypeError, ValueError):
                continue
            status = row.get("flight_status", "unknown")

            if last_zenith is None or last_time is None:
                omega = 0.0
            else:
                dt = t - last_time
                omega = (zenith - last_zenith) / dt if dt > 0.0 else 0.0

            should_predict = status in {"burn", "coast"} and vz > 0.0
            if should_predict:
                if last_predict_time is None or (t - last_predict_time) >= 0.1:
                    cos_zenith = math.cos(zenith)
                    clamped_cos = max(0.1, min(1.0, cos_zenith))
                    speed_along_axis = vz / clamped_cos
                    speed_sq = speed_along_axis * speed_along_axis
                    horizontal_sq = speed_sq - (vz * vz)
                    vh = math.sqrt(horizontal_sq) if horizontal_sq > 0.0 else 0.0
                    state = ApogeeState(
                        altitude_m=z,
                        horizontal_m=0.0,
                        vertical_v=vz,
                        horizontal_v=vh,
                        zenith=zenith,
                        angular_v=omega,
                        acs_deg=0.0,
                    )
                    last_apg = predict_apogee(state, wind, table)
                    last_predict_time = t
            w.writerow([t, z, vz, last_apg, status])

            last_zenith = zenith
            last_time = t

    print(f"Wrote {OUTPUT_CSV}")


if __name__ == "__main__":
    main()
