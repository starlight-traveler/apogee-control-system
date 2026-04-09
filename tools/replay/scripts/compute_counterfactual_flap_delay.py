#!/usr/bin/env python3

from __future__ import annotations

import argparse
import bisect
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
REPLAY_DIR = SCRIPT_DIR.parent
ROOT = REPLAY_DIR.parents[1]
DATA_DIR = REPLAY_DIR / "data"
PLOTS_DIR = REPLAY_DIR / "plots"

if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import compute_apogee_current_model as current_model


DEFAULT_INPUT_CSV = ROOT / "fullscale_3.csv"
DEFAULT_CFD_CSV = ROOT / "lib" / "cfd.csv"
DEFAULT_OUTPUT_CSV = DATA_DIR / "fullscale_3_counterfactual_flap_delay.csv"
DEFAULT_OUTPUT_SVG = PLOTS_DIR / "fullscale_3_counterfactual_flap_delay.svg"
DEFAULT_CORRECTED_SENSOR_CSV = DATA_DIR / "fullscale_3_bno_from_icm_sensor_only.csv"


@dataclass
class ReplayRow:
    time_s: float
    altitude_m: float
    vertical_velocity_mps: float
    apogee_estimate_m: float
    auto_cmd_deg: float
    status: str
    has_filtered_state: bool
    predictor_horizontal_speed_mps: float
    predictor_zenith_rad: float
    predictor_angular_rate_rad_s: float
    state_zenith_rad: float


@dataclass
class SimPoint:
    time_s: float
    altitude_m: float
    vertical_velocity_mps: float
    command_deg: float


class LoggedCommandSchedule:
    def __init__(self, rows: list[ReplayRow], branch_index: int, hold_seconds: float, mode: str) -> None:
        self.branch_time_s = rows[branch_index].time_s
        self.hold_seconds = hold_seconds
        self.mode = mode
        self.times = [row.time_s for row in rows[branch_index:]]
        self.commands = [row.auto_cmd_deg for row in rows[branch_index:]]

    def command_at(self, absolute_time_s: float) -> float:
        if self.mode == "stay_closed":
            return 0.0
        if absolute_time_s < (self.branch_time_s + self.hold_seconds):
            return 0.0

        source_time_s = absolute_time_s - self.hold_seconds
        idx = bisect.bisect_right(self.times, source_time_s) - 1
        if idx < 0:
            return 0.0
        if idx >= len(self.commands):
            return self.commands[-1]
        return self.commands[idx]


def _iter_csv_rows(path: Path):
    with path.open(newline="") as handle:
        reader = csv.DictReader(line for line in handle if not line.startswith("#"))
        for raw in reader:
            yield raw


def write_corrected_sensor_only_csv(input_csv: Path, output_csv: Path) -> None:
    required = [
        "sensor_timestamp",
        "sensor_altitude_feet",
        "sensor_accel_icm_x",
        "sensor_accel_icm_y",
        "sensor_accel_icm_z",
        "sensor_gyro_x",
        "sensor_gyro_y",
        "sensor_gyro_z",
        "sensor_quat_w",
        "sensor_quat_x",
        "sensor_quat_y",
        "sensor_quat_z",
        "sensor_has_quaternion",
    ]

    with input_csv.open(newline="") as handle:
        rows = list(csv.reader(line for line in handle if not line.startswith("#")))
    if not rows:
        raise SystemExit(f"Input CSV is empty: {input_csv}")

    header = rows[0]
    index = {name: i for i, name in enumerate(header)}
    missing = [name for name in required if name not in index]
    if missing:
        raise SystemExit(f"Missing required sensor columns for corrected CSV: {', '.join(missing)}")

    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "timestamp",
                "altitude_feet",
                "accel_bno_x",
                "accel_bno_y",
                "accel_bno_z",
                "accel_icm_x",
                "accel_icm_y",
                "accel_icm_z",
                "gyro_x",
                "gyro_y",
                "gyro_z",
                "quat_w",
                "quat_x",
                "quat_y",
                "quat_z",
                "has_quaternion",
            ]
        )
        for row in rows[1:]:
            writer.writerow(
                [
                    row[index["sensor_timestamp"]],
                    row[index["sensor_altitude_feet"]],
                    row[index["sensor_accel_icm_x"]],
                    row[index["sensor_accel_icm_y"]],
                    row[index["sensor_accel_icm_z"]],
                    row[index["sensor_accel_icm_x"]],
                    row[index["sensor_accel_icm_y"]],
                    row[index["sensor_accel_icm_z"]],
                    row[index["sensor_gyro_x"]],
                    row[index["sensor_gyro_y"]],
                    row[index["sensor_gyro_z"]],
                    row[index["sensor_quat_w"]],
                    row[index["sensor_quat_x"]],
                    row[index["sensor_quat_y"]],
                    row[index["sensor_quat_z"]],
                    row[index["sensor_has_quaternion"]],
                ]
            )


def _get_float(raw: dict[str, str], *names: str) -> float | None:
    for name in names:
        value = current_model.parse_float(raw.get(name))
        if value is not None:
            return value
    return None


def _get_bool(raw: dict[str, str], *names: str) -> bool:
    for name in names:
        if name in raw:
            return current_model.parse_bool(raw.get(name))
    return False


def load_replay_rows(path: Path) -> list[ReplayRow]:
    rows: list[ReplayRow] = []
    for raw in _iter_csv_rows(path):
        time_s = _get_float(raw, "state_time", "time_s", "sensor_timestamp", "timestamp")
        altitude_m = _get_float(raw, "state_position_z", "altitude_m", "altitude_meters")
        vertical_velocity_mps = _get_float(raw, "state_velocity_z", "velocity_mps", "vertical_velocity")
        apogee_estimate_m = _get_float(raw, "state_apogee_estimate", "apogee_prediction_m", "apogee_estimate")
        auto_cmd_deg = _get_float(raw, "sensor_auto_cmd_deg", "auto_cmd_deg", "command_deg")
        predictor_horizontal_speed_mps = _get_float(
            raw,
            "sensor_predictor_seed_horizontal_speed_mps",
            "predictor_horizontal_speed_mps",
        )
        predictor_zenith_rad = _get_float(
            raw,
            "sensor_predictor_seed_clamped_zenith_rad",
            "seed_zenith_rad",
            "state_zenith",
            "zenith_rad",
        )
        predictor_angular_rate_rad_s = _get_float(
            raw,
            "sensor_predictor_seed_clamped_angular_rate_rad_per_sec",
            "predictor_angular_rate_rad_s",
        )
        state_zenith_rad = _get_float(raw, "state_zenith", "zenith_rad")

        if None in (time_s, altitude_m, vertical_velocity_mps, apogee_estimate_m):
            continue

        rows.append(
            ReplayRow(
                time_s=time_s,
                altitude_m=altitude_m,
                vertical_velocity_mps=vertical_velocity_mps,
                apogee_estimate_m=apogee_estimate_m,
                auto_cmd_deg=0.0 if auto_cmd_deg is None else auto_cmd_deg,
                status=(raw.get("flight_status") or raw.get("status") or "").strip().lower(),
                has_filtered_state=_get_bool(raw, "has_filtered_state"),
                predictor_horizontal_speed_mps=0.0
                if predictor_horizontal_speed_mps is None
                else predictor_horizontal_speed_mps,
                predictor_zenith_rad=0.0 if predictor_zenith_rad is None else predictor_zenith_rad,
                predictor_angular_rate_rad_s=0.0
                if predictor_angular_rate_rad_s is None
                else predictor_angular_rate_rad_s,
                state_zenith_rad=0.0 if state_zenith_rad is None else state_zenith_rad,
            )
        )

    if not rows:
        raise SystemExit(f"No usable replay rows found in {path}")
    return rows


def find_branch_index(rows: list[ReplayRow], branch_time_s: float | None, command_threshold_deg: float) -> int:
    if branch_time_s is not None:
        for idx, row in enumerate(rows):
            if row.has_filtered_state and row.time_s >= branch_time_s:
                return idx
        raise SystemExit(f"No filtered replay row found at or after t={branch_time_s:.3f}s")

    for idx, row in enumerate(rows):
        if not row.has_filtered_state:
            continue
        if row.auto_cmd_deg > command_threshold_deg and row.status in {"burn", "coast", "overshoot"}:
            return idx
    raise SystemExit("Could not find an initial flap command in the replay data.")


def choose_seed_index(rows: list[ReplayRow], branch_index: int) -> int:
    for idx in range(branch_index - 1, -1, -1):
        row = rows[idx]
        if row.has_filtered_state and row.vertical_velocity_mps > 0.0:
            return idx
    raise SystemExit("Could not find a valid pre-branch seed row.")


def compute_actual_apogee(rows: list[ReplayRow]) -> tuple[float, float]:
    best_row = max((row for row in rows if row.has_filtered_state), key=lambda row: row.altitude_m)
    return best_row.altitude_m, best_row.time_s


def build_seed_state(seed_row: ReplayRow) -> current_model.ApogeeState:
    seed_zenith = seed_row.predictor_zenith_rad if math.isfinite(seed_row.predictor_zenith_rad) else 0.0
    if abs(seed_zenith) <= 1.0e-9 and math.isfinite(seed_row.state_zenith_rad):
        seed_zenith = seed_row.state_zenith_rad

    seed_angular_rate = (
        seed_row.predictor_angular_rate_rad_s if math.isfinite(seed_row.predictor_angular_rate_rad_s) else 0.0
    )
    seed_horizontal_speed = (
        seed_row.predictor_horizontal_speed_mps if math.isfinite(seed_row.predictor_horizontal_speed_mps) else 0.0
    )

    return current_model.ApogeeState(
        altitude_m=seed_row.altitude_m,
        horizontal_m=0.0,
        vertical_v=seed_row.vertical_velocity_mps,
        horizontal_v=seed_horizontal_speed,
        zenith=seed_zenith,
        angular_v=seed_angular_rate,
        acs_deg=0.0,
    )


def simulate_counterfactual(
    predictor: current_model.ApogeePredictor,
    initial_state: current_model.ApogeeState,
    seed_time_s: float,
    schedule: LoggedCommandSchedule,
) -> tuple[list[SimPoint], float, float]:
    state = current_model.copy_state(initial_state)
    time_s = seed_time_s
    points = [
        SimPoint(
            time_s=time_s,
            altitude_m=state.altitude_m,
            vertical_velocity_mps=state.vertical_v,
            command_deg=schedule.command_at(time_s),
        )
    ]

    apogee_m = state.altitude_m
    apogee_time_s = time_s
    steps = 0
    while state.vertical_v > 0.0 and steps < current_model.MAX_STEPS:
        state.acs_deg = schedule.command_at(time_s)
        previous_state = current_model.copy_state(state)
        previous_time_s = time_s
        state = predictor.integrate_step(state, "rk4")
        time_s += current_model.PRED_DT
        steps += 1
        points.append(
            SimPoint(
                time_s=time_s,
                altitude_m=state.altitude_m,
                vertical_velocity_mps=state.vertical_v,
                command_deg=schedule.command_at(time_s),
            )
        )
        if previous_state.vertical_v > 0.0 and state.vertical_v <= 0.0:
            apogee_m = current_model.refine_apogee_at_zero_crossing(previous_state, state)
            vz_delta = previous_state.vertical_v - state.vertical_v
            if abs(vz_delta) >= 1.0e-9:
                alpha = current_model.clamp(previous_state.vertical_v / vz_delta, 0.0, 1.0)
                apogee_time_s = previous_time_s + alpha * current_model.PRED_DT
            else:
                apogee_time_s = time_s
            break

    return points, apogee_m, apogee_time_s


def _interp_logged_value(times: list[float], values: list[float], query_time_s: float) -> float:
    idx = bisect.bisect_left(times, query_time_s)
    if idx <= 0:
        return values[0]
    if idx >= len(times):
        return values[-1]
    t0 = times[idx - 1]
    t1 = times[idx]
    v0 = values[idx - 1]
    v1 = values[idx]
    if t1 <= t0:
        return v0
    alpha = (query_time_s - t0) / (t1 - t0)
    return v0 + alpha * (v1 - v0)


def write_output_csv(
    output_csv: Path,
    sim_points: list[SimPoint],
    logged_rows: list[ReplayRow],
) -> None:
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    logged_times = [row.time_s for row in logged_rows if row.has_filtered_state]
    logged_altitudes = [row.altitude_m for row in logged_rows if row.has_filtered_state]
    logged_apogees = [row.apogee_estimate_m for row in logged_rows if row.has_filtered_state]

    with output_csv.open("w", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "time_s",
                "counterfactual_altitude_m",
                "counterfactual_vertical_velocity_mps",
                "counterfactual_command_deg",
                "logged_altitude_m",
                "logged_apogee_estimate_m",
            ],
        )
        writer.writeheader()
        for point in sim_points:
            writer.writerow(
                {
                    "time_s": point.time_s,
                    "counterfactual_altitude_m": point.altitude_m,
                    "counterfactual_vertical_velocity_mps": point.vertical_velocity_mps,
                    "counterfactual_command_deg": point.command_deg,
                    "logged_altitude_m": _interp_logged_value(logged_times, logged_altitudes, point.time_s),
                    "logged_apogee_estimate_m": _interp_logged_value(logged_times, logged_apogees, point.time_s),
                }
            )


def write_svg(
    output_svg: Path,
    logged_rows: list[ReplayRow],
    sim_points: list[SimPoint],
    actual_apogee_m: float,
    counterfactual_apogee_m: float,
    branch_time_s: float,
    hold_end_time_s: float,
    mode: str,
) -> None:
    output_svg.parent.mkdir(parents=True, exist_ok=True)

    logged_plot_rows = [row for row in logged_rows if row.has_filtered_state and row.time_s >= (branch_time_s - 0.5)]
    logged_series = [
        {
            "time_s": row.time_s,
            "logged_altitude_m": row.altitude_m,
            "logged_apogee_m": row.apogee_estimate_m,
        }
        for row in logged_plot_rows
    ]
    sim_series = [
        {
            "time_s": point.time_s,
            "counterfactual_altitude_m": point.altitude_m,
            "command_deg": point.command_deg,
        }
        for point in sim_points
    ]

    logged_sampled = current_model.decimate_rows(logged_series, 6000)
    sim_sampled = current_model.decimate_rows(sim_series, 6000)

    logged_times = [float(row["time_s"]) for row in logged_sampled]
    logged_altitudes = [float(row["logged_altitude_m"]) for row in logged_sampled]
    logged_apogees = [float(row["logged_apogee_m"]) for row in logged_sampled]
    sim_times = [float(row["time_s"]) for row in sim_sampled]
    sim_altitudes = [float(row["counterfactual_altitude_m"]) for row in sim_sampled]

    width = 1700
    height = 950
    left = 110
    right = 40
    top = 70
    bottom_margin = 80
    plot_w = width - left - right
    plot_h = height - top - bottom_margin

    t_min = min(logged_times[0], sim_times[0])
    t_max = max(logged_times[-1], sim_times[-1])
    y_min = min(min(logged_altitudes), min(logged_apogees), min(sim_altitudes), actual_apogee_m, counterfactual_apogee_m)
    y_max = max(max(logged_altitudes), max(logged_apogees), max(sim_altitudes), actual_apogee_m, counterfactual_apogee_m)
    y_pad = max(20.0, (y_max - y_min) * 0.05)
    y_min -= y_pad
    y_max += y_pad

    logged_altitude_points = current_model.polyline_points(
        logged_times, logged_altitudes, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h
    )
    logged_apogee_points = current_model.polyline_points(
        logged_times, logged_apogees, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h
    )
    sim_altitude_points = current_model.polyline_points(
        sim_times, sim_altitudes, t_min, t_max, y_min, y_max, left, top, plot_w, plot_h
    )

    counterfactual_line_points = current_model.polyline_points(
        [t_min, t_max],
        [counterfactual_apogee_m, counterfactual_apogee_m],
        t_min,
        t_max,
        y_min,
        y_max,
        left,
        top,
        plot_w,
        plot_h,
    )
    actual_line_points = current_model.polyline_points(
        [t_min, t_max],
        [actual_apogee_m, actual_apogee_m],
        t_min,
        t_max,
        y_min,
        y_max,
        left,
        top,
        plot_w,
        plot_h,
    )

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

    branch_x = current_model.scale(branch_time_s, t_min, t_max, left, left + plot_w)
    hold_end_x = current_model.scale(hold_end_time_s, t_min, t_max, left, left + plot_w)

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="white"/>
  <text x="{width / 2:.0f}" y="36" text-anchor="middle" font-family="monospace" font-size="24">Counterfactual Flap Delay Comparison</text>
  <text x="{width / 2:.0f}" y="{height - 20}" text-anchor="middle" font-family="monospace" font-size="18">Time (s)</text>
  <text x="30" y="{height / 2:.0f}" text-anchor="middle" font-family="monospace" font-size="18" transform="rotate(-90 30 {height / 2:.0f})">Meters</text>
  {''.join(grid_lines)}
  <rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="none" stroke="black" stroke-width="2"/>
  <polyline fill="none" stroke="#1f77b4" stroke-width="1.5" points="{logged_altitude_points}"/>
  <polyline fill="none" stroke="#9467bd" stroke-width="1.5" points="{logged_apogee_points}"/>
  <polyline fill="none" stroke="#d95f02" stroke-width="2.0" points="{sim_altitude_points}"/>
  <polyline fill="none" stroke="#2ca02c" stroke-width="1.5" stroke-dasharray="7,5" points="{counterfactual_line_points}"/>
  <polyline fill="none" stroke="#111111" stroke-width="1.2" stroke-dasharray="4,4" points="{actual_line_points}"/>
  <line x1="{branch_x:.2f}" y1="{top}" x2="{branch_x:.2f}" y2="{top + plot_h}" stroke="#c0392b" stroke-width="1.5" stroke-dasharray="5,4"/>
  <line x1="{hold_end_x:.2f}" y1="{top}" x2="{hold_end_x:.2f}" y2="{top + plot_h}" stroke="#16a085" stroke-width="1.5" stroke-dasharray="5,4"/>
  <line x1="{left + 20}" y1="{top + 20}" x2="{left + 90}" y2="{top + 20}" stroke="#1f77b4" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 26}" font-family="monospace" font-size="16">Logged Altitude</text>
  <line x1="{left + 20}" y1="{top + 48}" x2="{left + 90}" y2="{top + 48}" stroke="#9467bd" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 54}" font-family="monospace" font-size="16">Logged Apogee Estimate</text>
  <line x1="{left + 20}" y1="{top + 76}" x2="{left + 90}" y2="{top + 76}" stroke="#d95f02" stroke-width="3"/>
  <text x="{left + 100}" y="{top + 82}" font-family="monospace" font-size="16">Counterfactual Altitude</text>
  <line x1="{left + 20}" y1="{top + 104}" x2="{left + 90}" y2="{top + 104}" stroke="#2ca02c" stroke-width="2" stroke-dasharray="7,5"/>
  <text x="{left + 100}" y="{top + 110}" font-family="monospace" font-size="16">Counterfactual Final Apogee</text>
  <line x1="{left + 20}" y1="{top + 132}" x2="{left + 90}" y2="{top + 132}" stroke="#111111" stroke-width="2" stroke-dasharray="4,4"/>
  <text x="{left + 100}" y="{top + 138}" font-family="monospace" font-size="16">Logged Actual Apogee</text>
  <text x="{left + 20}" y="{top + 176}" font-family="monospace" font-size="15">Mode: {mode}</text>
  <text x="{left + 20}" y="{top + 198}" font-family="monospace" font-size="15">Branch: {branch_time_s:.3f}s</text>
  <text x="{left + 20}" y="{top + 220}" font-family="monospace" font-size="15">Hold End: {hold_end_time_s:.3f}s</text>
  <text x="{left + 20}" y="{top + 242}" font-family="monospace" font-size="15">Counterfactual Apogee: {counterfactual_apogee_m:.1f}m</text>
  <text x="{left + 20}" y="{top + 264}" font-family="monospace" font-size="15">Logged Actual Apogee: {actual_apogee_m:.1f}m</text>
  <text x="{branch_x + 8:.2f}" y="{top + 18}" font-family="monospace" font-size="13" fill="#c0392b">First flap command</text>
  <text x="{hold_end_x + 8:.2f}" y="{top + 36}" font-family="monospace" font-size="13" fill="#16a085">5s hold end</text>
</svg>
"""
    output_svg.write_text(svg)


def run(
    input_csv: Path,
    cfd_csv: Path,
    output_csv: Path,
    output_svg: Path,
    corrected_sensor_csv: Path | None,
    hold_seconds: float,
    mode: str,
    branch_time_s: float | None,
    command_threshold_deg: float,
) -> None:
    rows = load_replay_rows(input_csv)
    branch_index = find_branch_index(rows, branch_time_s, command_threshold_deg)
    seed_index = choose_seed_index(rows, branch_index)
    seed_row = rows[seed_index]
    branch_row = rows[branch_index]
    actual_apogee_m, actual_apogee_time_s = compute_actual_apogee(rows)

    force_table = current_model.load_force_table(cfd_csv)
    predictor = current_model.ApogeePredictor(current_model.EnvironmentModel(), force_table)
    schedule = LoggedCommandSchedule(rows, branch_index, hold_seconds, mode)
    initial_state = build_seed_state(seed_row)
    sim_points, counterfactual_apogee_m, counterfactual_apogee_time_s = simulate_counterfactual(
        predictor,
        initial_state,
        seed_row.time_s,
        schedule,
    )

    write_output_csv(output_csv, sim_points, rows)
    write_svg(
        output_svg,
        rows,
        sim_points,
        actual_apogee_m,
        counterfactual_apogee_m,
        branch_row.time_s,
        branch_row.time_s + hold_seconds,
        mode,
    )

    if corrected_sensor_csv is not None:
        write_corrected_sensor_only_csv(input_csv, corrected_sensor_csv)

    print(f"seed_time_s={seed_row.time_s:.6f}")
    print(f"branch_time_s={branch_row.time_s:.6f}")
    print(f"hold_seconds={hold_seconds:.3f}")
    print(f"mode={mode}")
    print(f"logged_actual_apogee_m={actual_apogee_m:.3f}")
    print(f"logged_actual_apogee_time_s={actual_apogee_time_s:.3f}")
    print(f"counterfactual_apogee_m={counterfactual_apogee_m:.3f}")
    print(f"counterfactual_apogee_time_s={counterfactual_apogee_time_s:.3f}")
    print(f"delta_vs_logged_actual_m={counterfactual_apogee_m - actual_apogee_m:.3f}")
    print(f"wrote_csv={output_csv}")
    print(f"wrote_svg={output_svg}")
    if corrected_sensor_csv is not None:
        print(f"wrote_corrected_sensor_csv={corrected_sensor_csv}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Counterfactual apogee branch: hold flaps closed for N seconds, then follow a delayed logged schedule."
    )
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT_CSV, help="Replay CSV to read.")
    parser.add_argument("--cfd", type=Path, default=DEFAULT_CFD_CSV, help="CFD force table CSV.")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT_CSV, help="Output CSV path.")
    parser.add_argument("--svg", type=Path, default=DEFAULT_OUTPUT_SVG, help="Output SVG path.")
    parser.add_argument(
        "--corrected-sensor-csv",
        type=Path,
        default=DEFAULT_CORRECTED_SENSOR_CSV,
        help="Write a sensor-only CSV with BNO accel overwritten from ICM accel.",
    )
    parser.add_argument("--hold-seconds", type=float, default=5.0, help="How long to hold flaps at 0 deg.")
    parser.add_argument(
        "--mode",
        choices=["delay_logged_schedule", "stay_closed"],
        default="delay_logged_schedule",
        help="Counterfactual command policy after branch.",
    )
    parser.add_argument(
        "--branch-time",
        type=float,
        default=None,
        help="Optional override for branch start time in seconds.",
    )
    parser.add_argument(
        "--command-threshold",
        type=float,
        default=0.5,
        help="Minimum logged command that counts as flap deployment.",
    )
    args = parser.parse_args()

    run(
        args.input,
        args.cfd,
        args.output,
        args.svg,
        args.corrected_sensor_csv,
        args.hold_seconds,
        args.mode,
        args.branch_time,
        args.command_threshold,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
