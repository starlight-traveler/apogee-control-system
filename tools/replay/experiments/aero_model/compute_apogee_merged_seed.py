#!/usr/bin/env python3

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path


from replaylib.paths import SCRIPTS_DIR
SCRIPT_DIR = SCRIPTS_DIR
REPLAY_DIR = SCRIPT_DIR.parent
ROOT = REPLAY_DIR.parents[1]
DATA_DIR = REPLAY_DIR / "data"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from python.apogee_predictor_sim import ApogeeState
from python.apogee_predictor_sim import SaferSeedConfig
from python.apogee_predictor_sim import compute_actual_apogee
from python.apogee_predictor_sim import load_force_table
from python.apogee_predictor_sim import parse_replay_rows
from python.apogee_predictor_sim import predict_apogee


INPUT_CSV = DATA_DIR / "output.csv"
CFD_CSV = ROOT / "lib" / "cfd.csv"
OUTPUT_CSV = DATA_DIR / "output_merged_seed_prediction.csv"


class HorizontalVelocityTracker:
    def __init__(self) -> None:
        self.vx = 0.0
        self.vy = 0.0

    def reset(self) -> None:
        self.vx = 0.0
        self.vy = 0.0


def predictor_seed_has_fresh_sample(dt_seconds: float) -> bool:
    return math.isfinite(dt_seconds) and dt_seconds > 1.0e-4 and dt_seconds <= 0.25


def clamp_predictor_zenith_radians(zenith_radians: float, config: SaferSeedConfig) -> float:
    max_zenith_radians = math.radians(config.max_seed_zenith_deg)
    return max(-max_zenith_radians, min(max_zenith_radians, zenith_radians))


def sanitize_predictor_zenith_radians(zenith_radians: float, config: SaferSeedConfig) -> float:
    if not math.isfinite(zenith_radians):
        return 0.0
    return clamp_predictor_zenith_radians(zenith_radians, config)


def clamp_predictor_angular_rate(angular_rate_rad_s: float, config: SaferSeedConfig) -> float:
    return max(-config.max_seed_rate_rad_s, min(config.max_seed_rate_rad_s, angular_rate_rad_s))


def compute_predictor_angular_rate(
    current_zenith_radians: float,
    previous_zenith_radians: float,
    dt_seconds: float,
) -> float:
    if (
        not predictor_seed_has_fresh_sample(dt_seconds)
        or not math.isfinite(current_zenith_radians)
        or not math.isfinite(previous_zenith_radians)
    ):
        return 0.0
    return (current_zenith_radians - previous_zenith_radians) / dt_seconds


def predictor_horizontal_speed_cap(
    vertical_velocity_mps: float,
    zenith_radians: float,
    config: SaferSeedConfig,
) -> float:
    effective_zenith = min(abs(zenith_radians), math.radians(config.max_seed_zenith_deg))
    tilt_cap = abs(vertical_velocity_mps) * math.tan(effective_zenith) + config.horizontal_margin_mps
    return max(config.min_horizontal_cap_mps, min(config.max_horizontal_speed_mps, tilt_cap))


def predictor_geometric_horizontal_speed(vertical_velocity_mps: float, zenith_radians: float) -> float:
    if not math.isfinite(vertical_velocity_mps) or not math.isfinite(zenith_radians):
        return 0.0
    vertical_speed = abs(vertical_velocity_mps)
    if vertical_speed <= 0.0:
        return 0.0

    clamped_cos_zenith = max(0.1, min(1.0, abs(math.cos(zenith_radians))))
    speed_along_axis = vertical_speed / clamped_cos_zenith
    horizontal_speed_squared = speed_along_axis * speed_along_axis - vertical_speed * vertical_speed
    return math.sqrt(horizontal_speed_squared) if horizontal_speed_squared > 0.0 else 0.0


def update_predictor_horizontal_speed(
    tracker: HorizontalVelocityTracker,
    accel_x_mps2: float,
    accel_y_mps2: float,
    dt_seconds: float,
    allow_integration: bool,
    vertical_velocity_mps: float,
    zenith_radians: float,
    config: SaferSeedConfig,
) -> float:
    if dt_seconds <= 0.0:
        return math.hypot(tracker.vx, tracker.vy)

    clamped_dt = min(dt_seconds, 0.25)
    ax = max(-config.accel_limit_mps2, min(config.accel_limit_mps2, accel_x_mps2))
    ay = max(-config.accel_limit_mps2, min(config.accel_limit_mps2, accel_y_mps2))
    decay = math.exp(-clamped_dt / config.horizontal_decay_tau_s) if config.horizontal_decay_tau_s > 0.0 else 0.0

    if allow_integration and vertical_velocity_mps > 0.0:
        tracker.vx = (tracker.vx + ax * clamped_dt) * decay
        tracker.vy = (tracker.vy + ay * clamped_dt) * decay
    else:
        tracker.vx *= decay
        tracker.vy *= decay

    speed_squared = tracker.vx * tracker.vx + tracker.vy * tracker.vy
    cap = predictor_horizontal_speed_cap(vertical_velocity_mps, zenith_radians, config)
    cap_squared = cap * cap
    if speed_squared > cap_squared and speed_squared > 1.0e-18:
        speed = math.sqrt(speed_squared)
        scale = cap / speed
        tracker.vx *= scale
        tracker.vy *= scale
        return cap
    return math.sqrt(speed_squared)


def resolve_predictor_horizontal_speed(
    tracked_horizontal_speed_mps: float,
    vertical_velocity_mps: float,
    zenith_radians: float,
    config: SaferSeedConfig,
) -> float:
    cap = predictor_horizontal_speed_cap(vertical_velocity_mps, zenith_radians, config)
    geometric_speed = predictor_geometric_horizontal_speed(vertical_velocity_mps, zenith_radians)
    return max(0.0, min(cap, max(tracked_horizontal_speed_mps, geometric_speed)))


def main() -> None:
    config = SaferSeedConfig()
    rows = parse_replay_rows(INPUT_CSV)
    force_table = load_force_table(CFD_CSV)
    actual_apogee_m = compute_actual_apogee(rows)

    tracker = HorizontalVelocityTracker()
    last_prediction_m = 0.0
    last_time_s = 0.0
    last_zenith_radians = 0.0
    initialized = False
    output_rows: list[dict[str, float | str]] = []

    for row in rows:
        dt_seconds = 0.0 if not initialized else (row.time_s - last_time_s)
        seed_zenith = sanitize_predictor_zenith_radians(row.zenith_rad, config)
        fresh_seed_sample = predictor_seed_has_fresh_sample(dt_seconds)
        can_use_horizontal_seed = math.isfinite(row.zenith_rad) and fresh_seed_sample
        should_predict = row.status in ("burn", "coast") and row.vertical_velocity_mps > 0.0

        tracked_horizontal_speed_mps = 0.0
        merged_horizontal_speed_mps = 0.0
        angular_rate_rad_s = 0.0

        if should_predict:
            if not can_use_horizontal_seed:
                tracker.reset()
            tracked_horizontal_speed_mps = (
                update_predictor_horizontal_speed(
                    tracker,
                    row.inertial_ax_mps2,
                    row.inertial_ay_mps2,
                    dt_seconds,
                    True,
                    row.vertical_velocity_mps,
                    seed_zenith,
                    config,
                )
                if can_use_horizontal_seed
                else 0.0
            )
            merged_horizontal_speed_mps = (
                resolve_predictor_horizontal_speed(
                    tracked_horizontal_speed_mps,
                    row.vertical_velocity_mps,
                    seed_zenith,
                    config,
                )
                if can_use_horizontal_seed
                else 0.0
            )
            angular_rate_rad_s = (
                clamp_predictor_angular_rate(
                    compute_predictor_angular_rate(row.zenith_rad, last_zenith_radians, dt_seconds),
                    config,
                )
                if can_use_horizontal_seed
                else 0.0
            )

            predictor_state = ApogeeState(
                altitude_m=row.altitude_m,
                horizontal_m=0.0,
                vertical_v=row.vertical_velocity_mps,
                horizontal_v=merged_horizontal_speed_mps,
                zenith=seed_zenith,
                angular_v=angular_rate_rad_s,
                acs_deg=0.0,
            )
            last_prediction_m = predict_apogee(predictor_state, force_table)
        else:
            tracker.reset()

        output_rows.append(
            {
                "time_s": row.time_s,
                "altitude_m": row.altitude_m,
                "velocity_mps": row.vertical_velocity_mps,
                "apogee_prediction_m": last_prediction_m,
                "tracked_horizontal_speed_mps": tracked_horizontal_speed_mps,
                "merged_horizontal_speed_mps": merged_horizontal_speed_mps,
                "zenith_deg": math.degrees(seed_zenith),
                "angular_rate_rad_s": angular_rate_rad_s,
                "status": row.status,
            }
        )

        last_time_s = row.time_s
        last_zenith_radians = row.zenith_rad
        initialized = True

    with OUTPUT_CSV.open("w", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "time_s",
                "altitude_m",
                "velocity_mps",
                "apogee_prediction_m",
                "tracked_horizontal_speed_mps",
                "merged_horizontal_speed_mps",
                "zenith_deg",
                "angular_rate_rad_s",
                "status",
            ],
        )
        writer.writeheader()
        writer.writerows(output_rows)

    print(f"Wrote {OUTPUT_CSV}")
    print(f"Actual apogee: {actual_apogee_m:.2f} m")
    for threshold_m in (100.0, 50.0, 25.0, 10.0, 5.0):
        hit = next(
            (row for row in output_rows if abs(float(row["apogee_prediction_m"]) - actual_apogee_m) <= threshold_m),
            None,
        )
        if hit is None:
            print(f"First within {threshold_m:.0f} m: none")
            continue
        print(
            f"First within {threshold_m:.0f} m: "
            f"t={float(hit['time_s']):.3f} s, "
            f"alt={float(hit['altitude_m']):.1f} m, "
            f"pred={float(hit['apogee_prediction_m']):.1f} m, "
            f"vh={float(hit['merged_horizontal_speed_mps']):.1f} m/s, "
            f"status={hit['status']}"
        )


if __name__ == "__main__":
    main()
