#!/usr/bin/env python3
"""
Diagnose body Z-axis alignment from quaternion.

This script extracts the body Z-axis vector from the logged quaternions and
compares it against:
  - Ground: accelerometer reading (should show gravity direction)
  - Flight: normalized velocity vector (rocket flies nose-first)

This helps determine if the issue is:
  - Mount rotation matrix (wrong axis mapping)
  - Quaternion convention (body-to-world vs world-to-body)
  - AHRS tracking failure during dynamics
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path


from replaylib.paths import SCRIPTS_DIR
SCRIPT_DIR = SCRIPTS_DIR
REPLAY_DIR = SCRIPT_DIR.parent
ROOT = REPLAY_DIR.parents[1]
PLOTS_DIR = REPLAY_DIR / "plots"

DEFAULT_INPUT_CSV = ROOT / "fullscale_3.csv"
DEFAULT_OUTPUT_SVG = PLOTS_DIR / "fullscale_3_body_axis_diagnostic.svg"


@dataclass
class Sample:
    time_s: float
    status: str
    # LSM quaternion
    lsm_qw: float
    lsm_qx: float
    lsm_qy: float
    lsm_qz: float
    # LSM accelerometer (body frame)
    lsm_ax: float
    lsm_ay: float
    lsm_az: float
    # ICM quaternion
    icm_qw: float
    icm_qx: float
    icm_qy: float
    icm_qz: float
    # State velocity
    vx: float
    vy: float
    vz: float
    # Altitude
    altitude_m: float


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


def normalize(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Normalize a 3D vector."""
    mag = math.sqrt(x*x + y*y + z*z)
    if mag < 1e-9:
        return (0.0, 0.0, 0.0)
    return (x/mag, y/mag, z/mag)


def dot3(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    """Dot product of two 3D vectors."""
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]


def body_z_axis_convention1(qw: float, qx: float, qy: float, qz: float) -> tuple[float, float, float]:
    """
    Extract body Z-axis in world frame assuming body-to-world quaternion.
    R * [0,0,1] = [2(xz+wy), 2(yz-wx), 1-2(x²+y²)]
    """
    zx = 2.0 * (qx*qz + qw*qy)
    zy = 2.0 * (qy*qz - qw*qx)
    zz = 1.0 - 2.0 * (qx*qx + qy*qy)
    return normalize(zx, zy, zz)


def body_z_axis_convention2(qw: float, qx: float, qy: float, qz: float) -> tuple[float, float, float]:
    """
    Extract body Z-axis in world frame assuming world-to-body quaternion.
    R^T * [0,0,1] = [2(xz-wy), 2(yz+wx), 1-2(x²+y²)]
    """
    zx = 2.0 * (qx*qz - qw*qy)
    zy = 2.0 * (qy*qz + qw*qx)
    zz = 1.0 - 2.0 * (qx*qx + qy*qy)
    return normalize(zx, zy, zz)


def load_samples(path: Path) -> list[Sample]:
    samples: list[Sample] = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(line for line in handle if not line.startswith("#"))
        for row in reader:
            status = (row.get("flight_status") or "").strip().lower()
            time_s = parse_float(row.get("sensor_timestamp"))

            # LSM quaternion
            lsm_qw = parse_float(row.get("sensor_lsm_quat_w"))
            lsm_qx = parse_float(row.get("sensor_lsm_quat_x"))
            lsm_qy = parse_float(row.get("sensor_lsm_quat_y"))
            lsm_qz = parse_float(row.get("sensor_lsm_quat_z"))

            # LSM accelerometer
            lsm_ax = parse_float(row.get("sensor_accel_lsm_x"))
            lsm_ay = parse_float(row.get("sensor_accel_lsm_y"))
            lsm_az = parse_float(row.get("sensor_accel_lsm_z"))

            # ICM quaternion
            icm_qw = parse_float(row.get("sensor_icm_quat_w"))
            icm_qx = parse_float(row.get("sensor_icm_quat_x"))
            icm_qy = parse_float(row.get("sensor_icm_quat_y"))
            icm_qz = parse_float(row.get("sensor_icm_quat_z"))

            # State velocity
            vx = parse_float(row.get("state_velocity_x"))
            vy = parse_float(row.get("state_velocity_y"))
            vz = parse_float(row.get("state_velocity_z"))

            # Altitude
            altitude_m = parse_float(row.get("state_position_z"))

            if None in (time_s, lsm_qw, lsm_qx, lsm_qy, lsm_qz,
                        lsm_ax, lsm_ay, lsm_az,
                        icm_qw, icm_qx, icm_qy, icm_qz,
                        vx, vy, vz, altitude_m):
                continue

            samples.append(Sample(
                time_s=time_s,
                status=status,
                lsm_qw=lsm_qw, lsm_qx=lsm_qx, lsm_qy=lsm_qy, lsm_qz=lsm_qz,
                lsm_ax=lsm_ax, lsm_ay=lsm_ay, lsm_az=lsm_az,
                icm_qw=icm_qw, icm_qx=icm_qx, icm_qy=icm_qy, icm_qz=icm_qz,
                vx=vx, vy=vy, vz=vz,
                altitude_m=altitude_m,
            ))
    return samples


def analyze_ground_phase(samples: list[Sample]) -> dict:
    """Analyze body Z-axis alignment with gravity during ground phase."""
    ground_samples = [s for s in samples if s.status == "ground"]
    if not ground_samples:
        return {}

    # Use first 100 samples for stable ground measurement
    ground_samples = ground_samples[:min(100, len(ground_samples))]

    # World up direction
    world_up = (0.0, 0.0, 1.0)
    world_down = (0.0, 0.0, -1.0)

    results = {
        "lsm_conv1_dot_up": [],
        "lsm_conv1_dot_down": [],
        "lsm_conv2_dot_up": [],
        "lsm_conv2_dot_down": [],
        "icm_conv1_dot_up": [],
        "icm_conv2_dot_up": [],
        "accel_direction": [],  # normalized accelerometer reading
    }

    for s in ground_samples:
        # LSM body Z-axis both conventions
        lsm_z1 = body_z_axis_convention1(s.lsm_qw, s.lsm_qx, s.lsm_qy, s.lsm_qz)
        lsm_z2 = body_z_axis_convention2(s.lsm_qw, s.lsm_qx, s.lsm_qy, s.lsm_qz)

        results["lsm_conv1_dot_up"].append(dot3(lsm_z1, world_up))
        results["lsm_conv1_dot_down"].append(dot3(lsm_z1, world_down))
        results["lsm_conv2_dot_up"].append(dot3(lsm_z2, world_up))
        results["lsm_conv2_dot_down"].append(dot3(lsm_z2, world_down))

        # ICM body Z-axis
        icm_z1 = body_z_axis_convention1(s.icm_qw, s.icm_qx, s.icm_qy, s.icm_qz)
        icm_z2 = body_z_axis_convention2(s.icm_qw, s.icm_qx, s.icm_qy, s.icm_qz)
        results["icm_conv1_dot_up"].append(dot3(icm_z1, world_up))
        results["icm_conv2_dot_up"].append(dot3(icm_z2, world_up))

        # Accelerometer direction (normalized)
        accel_norm = normalize(s.lsm_ax, s.lsm_ay, s.lsm_az)
        results["accel_direction"].append(accel_norm)

    # Compute means
    summary = {}
    for key in ["lsm_conv1_dot_up", "lsm_conv1_dot_down", "lsm_conv2_dot_up",
                "lsm_conv2_dot_down", "icm_conv1_dot_up", "icm_conv2_dot_up"]:
        values = results[key]
        summary[f"{key}_mean"] = sum(values) / len(values)
        summary[f"{key}_angle_deg"] = math.degrees(math.acos(max(-1, min(1, summary[f"{key}_mean"]))))

    # Mean accelerometer direction
    ax_mean = sum(a[0] for a in results["accel_direction"]) / len(results["accel_direction"])
    ay_mean = sum(a[1] for a in results["accel_direction"]) / len(results["accel_direction"])
    az_mean = sum(a[2] for a in results["accel_direction"]) / len(results["accel_direction"])
    summary["accel_mean_direction"] = normalize(ax_mean, ay_mean, az_mean)
    summary["accel_mean_magnitude"] = math.sqrt(
        sum(s.lsm_ax**2 + s.lsm_ay**2 + s.lsm_az**2 for s in ground_samples) / len(ground_samples)
    )

    return summary


def analyze_flight_phase(samples: list[Sample]) -> dict:
    """Analyze body Z-axis alignment with velocity during flight."""
    # Use burn and coast phases
    flight_samples = [s for s in samples if s.status in ("burn", "coast")]
    if not flight_samples:
        return {}

    results = {
        "time": [],
        "status": [],
        "lsm_z1_dot_vel": [],  # body Z (conv1) dot velocity
        "lsm_z2_dot_vel": [],  # body Z (conv2) dot velocity
        "icm_z1_dot_vel": [],
        "icm_z2_dot_vel": [],
        "velocity_magnitude": [],
        "altitude": [],
    }

    for s in flight_samples:
        vel_norm = normalize(s.vx, s.vy, s.vz)
        vel_mag = math.sqrt(s.vx**2 + s.vy**2 + s.vz**2)

        if vel_mag < 10.0:  # Skip low velocity samples
            continue

        # LSM body Z-axis
        lsm_z1 = body_z_axis_convention1(s.lsm_qw, s.lsm_qx, s.lsm_qy, s.lsm_qz)
        lsm_z2 = body_z_axis_convention2(s.lsm_qw, s.lsm_qx, s.lsm_qy, s.lsm_qz)

        # ICM body Z-axis
        icm_z1 = body_z_axis_convention1(s.icm_qw, s.icm_qx, s.icm_qy, s.icm_qz)
        icm_z2 = body_z_axis_convention2(s.icm_qw, s.icm_qx, s.icm_qy, s.icm_qz)

        results["time"].append(s.time_s)
        results["status"].append(s.status)
        results["lsm_z1_dot_vel"].append(dot3(lsm_z1, vel_norm))
        results["lsm_z2_dot_vel"].append(dot3(lsm_z2, vel_norm))
        results["icm_z1_dot_vel"].append(dot3(icm_z1, vel_norm))
        results["icm_z2_dot_vel"].append(dot3(icm_z2, vel_norm))
        results["velocity_magnitude"].append(vel_mag)
        results["altitude"].append(s.altitude_m)

    return results


def scale(value: float, src_lo: float, src_hi: float, dst_lo: float, dst_hi: float) -> float:
    if src_hi <= src_lo:
        return 0.5 * (dst_lo + dst_hi)
    ratio = (value - src_lo) / (src_hi - src_lo)
    return dst_lo + ratio * (dst_hi - dst_lo)


def polyline_points(xs: list[float], ys: list[float],
                    x_min: float, x_max: float, y_min: float, y_max: float,
                    left: float, top: float, width: float, height: float) -> str:
    points: list[str] = []
    for x, y in zip(xs, ys):
        if not math.isfinite(y):
            continue
        px = scale(x, x_min, x_max, left, left + width)
        py = scale(y, y_min, y_max, top + height, top)
        points.append(f"{px:.2f},{py:.2f}")
    return " ".join(points)


def write_svg(samples: list[Sample], ground_analysis: dict, flight_analysis: dict, output_path: Path) -> None:
    width = 1500.0
    height = 1200.0
    left = 95.0
    plot_width = 1360.0
    panel_height = 220.0
    gap = 80.0
    top1 = 120.0
    top2 = top1 + panel_height + gap
    top3 = top2 + panel_height + gap

    elements = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{int(width)}" height="{int(height)}" viewBox="0 0 {int(width)} {int(height)}">',
        '<rect width="100%" height="100%" fill="#fffaf3"/>',
        '<style>text{font-family:Helvetica,Arial,sans-serif}</style>',
        '<text x="95" y="42" font-size="30" font-weight="700" fill="#1f2937">Body Z-Axis Diagnostic: Quaternion vs Velocity/Gravity</text>',
        '<text x="95" y="72" font-size="15" fill="#374151">'
        'Compares body Z-axis from quaternion against velocity (flight) and gravity (ground). '
        'Dot product of 1.0 means perfect alignment, -1.0 means opposite.</text>',
    ]

    # Ground phase summary box
    if ground_analysis:
        box_y = top1 - 10
        elements.append(f'<rect x="{left}" y="{box_y}" width="700" height="90" rx="8" fill="#f0fdf4" stroke="#86efac"/>')
        elements.append(f'<text x="{left + 15}" y="{box_y + 25}" font-size="16" font-weight="700" fill="#166534">Ground Phase Analysis (body Z vs world up)</text>')

        lsm1_angle = ground_analysis.get("lsm_conv1_dot_up_angle_deg", 0)
        lsm2_angle = ground_analysis.get("lsm_conv2_dot_up_angle_deg", 0)
        icm1_angle = ground_analysis.get("icm_conv1_dot_up_angle_deg", 0)
        icm2_angle = ground_analysis.get("icm_conv2_dot_up_angle_deg", 0)

        elements.append(f'<text x="{left + 15}" y="{box_y + 50}" font-size="14" fill="#1f2937">'
                       f'LSM Conv1 (body-to-world): {lsm1_angle:.1f} deg from up | '
                       f'Conv2 (world-to-body): {lsm2_angle:.1f} deg from up</text>')
        elements.append(f'<text x="{left + 15}" y="{box_y + 72}" font-size="14" fill="#1f2937">'
                       f'ICM Conv1 (body-to-world): {icm1_angle:.1f} deg from up | '
                       f'Conv2 (world-to-body): {icm2_angle:.1f} deg from up</text>')

        # Interpretation
        best_lsm = "Conv1" if lsm1_angle < lsm2_angle else "Conv2"
        best_icm = "Conv1" if icm1_angle < icm2_angle else "Conv2"
        elements.append(f'<text x="{left + 15}" y="{box_y + 94}" font-size="13" fill="#059669">'
                       f'Best LSM: {best_lsm} ({min(lsm1_angle, lsm2_angle):.1f} deg) | '
                       f'Best ICM: {best_icm} ({min(icm1_angle, icm2_angle):.1f} deg)</text>')

    # Flight phase plots
    if flight_analysis and flight_analysis.get("time"):
        times = flight_analysis["time"]
        x_min, x_max = min(times), max(times)

        # Panel 1: LSM body Z dot velocity
        elements.append(f'<text x="{left}" y="{top1 + 100}" font-size="18" font-weight="700" fill="#1f2937">'
                       f'LSM Body Z-Axis dot Velocity (1.0 = nose aligned with velocity)</text>')

        # Background
        elements.append(f'<rect x="{left}" y="{top1 + 110}" width="{plot_width}" height="{panel_height}" '
                       'fill="#ffffff" stroke="#5b534a" stroke-width="1.5"/>')

        # Grid lines
        for i in range(5):
            gy = top1 + 110 + i * panel_height / 4.0
            y_val = 1.0 - i * 0.5
            elements.append(f'<line x1="{left}" y1="{gy}" x2="{left + plot_width}" y2="{gy}" stroke="#e5dccd" stroke-width="1"/>')
            elements.append(f'<text x="{left - 12}" y="{gy + 4}" text-anchor="end" font-size="12" fill="#1f2937">{y_val:.1f}</text>')

        # Data
        lsm_z1_pts = polyline_points(times, flight_analysis["lsm_z1_dot_vel"],
                                      x_min, x_max, -1.0, 1.0, left, top1 + 110, plot_width, panel_height)
        lsm_z2_pts = polyline_points(times, flight_analysis["lsm_z2_dot_vel"],
                                      x_min, x_max, -1.0, 1.0, left, top1 + 110, plot_width, panel_height)

        if lsm_z1_pts:
            elements.append(f'<polyline fill="none" stroke="#2563eb" stroke-width="2" points="{lsm_z1_pts}"/>')
        if lsm_z2_pts:
            elements.append(f'<polyline fill="none" stroke="#dc2626" stroke-width="2" stroke-dasharray="6 4" points="{lsm_z2_pts}"/>')

        # Legend
        elements.append(f'<line x1="{left + plot_width - 300}" y1="{top1 + 130}" x2="{left + plot_width - 270}" y2="{top1 + 130}" stroke="#2563eb" stroke-width="3"/>')
        elements.append(f'<text x="{left + plot_width - 260}" y="{top1 + 135}" font-size="13" fill="#1f2937">Conv1 (body-to-world)</text>')
        elements.append(f'<line x1="{left + plot_width - 300}" y1="{top1 + 155}" x2="{left + plot_width - 270}" y2="{top1 + 155}" stroke="#dc2626" stroke-width="3" stroke-dasharray="6 4"/>')
        elements.append(f'<text x="{left + plot_width - 260}" y="{top1 + 160}" font-size="13" fill="#1f2937">Conv2 (world-to-body)</text>')

        # Panel 2: ICM body Z dot velocity
        top2_adj = top1 + 110 + panel_height + 60
        elements.append(f'<text x="{left}" y="{top2_adj - 10}" font-size="18" font-weight="700" fill="#1f2937">'
                       f'ICM Body Z-Axis dot Velocity</text>')

        elements.append(f'<rect x="{left}" y="{top2_adj}" width="{plot_width}" height="{panel_height}" '
                       'fill="#ffffff" stroke="#5b534a" stroke-width="1.5"/>')

        for i in range(5):
            gy = top2_adj + i * panel_height / 4.0
            y_val = 1.0 - i * 0.5
            elements.append(f'<line x1="{left}" y1="{gy}" x2="{left + plot_width}" y2="{gy}" stroke="#e5dccd" stroke-width="1"/>')
            elements.append(f'<text x="{left - 12}" y="{gy + 4}" text-anchor="end" font-size="12" fill="#1f2937">{y_val:.1f}</text>')

        icm_z1_pts = polyline_points(times, flight_analysis["icm_z1_dot_vel"],
                                      x_min, x_max, -1.0, 1.0, left, top2_adj, plot_width, panel_height)
        icm_z2_pts = polyline_points(times, flight_analysis["icm_z2_dot_vel"],
                                      x_min, x_max, -1.0, 1.0, left, top2_adj, plot_width, panel_height)

        if icm_z1_pts:
            elements.append(f'<polyline fill="none" stroke="#2563eb" stroke-width="2" points="{icm_z1_pts}"/>')
        if icm_z2_pts:
            elements.append(f'<polyline fill="none" stroke="#dc2626" stroke-width="2" stroke-dasharray="6 4" points="{icm_z2_pts}"/>')

        elements.append(f'<line x1="{left + plot_width - 300}" y1="{top2_adj + 20}" x2="{left + plot_width - 270}" y2="{top2_adj + 20}" stroke="#2563eb" stroke-width="3"/>')
        elements.append(f'<text x="{left + plot_width - 260}" y="{top2_adj + 25}" font-size="13" fill="#1f2937">Conv1 (body-to-world)</text>')
        elements.append(f'<line x1="{left + plot_width - 300}" y1="{top2_adj + 45}" x2="{left + plot_width - 270}" y2="{top2_adj + 45}" stroke="#dc2626" stroke-width="3" stroke-dasharray="6 4"/>')
        elements.append(f'<text x="{left + plot_width - 260}" y="{top2_adj + 50}" font-size="13" fill="#1f2937">Conv2 (world-to-body)</text>')

        # Panel 3: Velocity magnitude context
        top3_adj = top2_adj + panel_height + 60
        elements.append(f'<text x="{left}" y="{top3_adj - 10}" font-size="18" font-weight="700" fill="#1f2937">'
                       f'Velocity Magnitude [m/s]</text>')

        elements.append(f'<rect x="{left}" y="{top3_adj}" width="{plot_width}" height="{panel_height}" '
                       'fill="#ffffff" stroke="#5b534a" stroke-width="1.5"/>')

        vel_mags = flight_analysis["velocity_magnitude"]
        v_max = max(vel_mags) if vel_mags else 1.0

        for i in range(5):
            gy = top3_adj + i * panel_height / 4.0
            y_val = v_max * (1.0 - i / 4.0)
            elements.append(f'<line x1="{left}" y1="{gy}" x2="{left + plot_width}" y2="{gy}" stroke="#e5dccd" stroke-width="1"/>')
            elements.append(f'<text x="{left - 12}" y="{gy + 4}" text-anchor="end" font-size="12" fill="#1f2937">{y_val:.0f}</text>')

        vel_pts = polyline_points(times, vel_mags, x_min, x_max, 0, v_max, left, top3_adj, plot_width, panel_height)
        if vel_pts:
            elements.append(f'<polyline fill="none" stroke="#059669" stroke-width="2" points="{vel_pts}"/>')

        # X-axis label
        elements.append(f'<text x="{left + plot_width / 2}" y="{top3_adj + panel_height + 35}" '
                       'text-anchor="middle" font-size="14" fill="#1f2937">Time [s]</text>')

        # Summary statistics
        lsm_z1_mean = sum(flight_analysis["lsm_z1_dot_vel"]) / len(flight_analysis["lsm_z1_dot_vel"])
        lsm_z2_mean = sum(flight_analysis["lsm_z2_dot_vel"]) / len(flight_analysis["lsm_z2_dot_vel"])
        icm_z1_mean = sum(flight_analysis["icm_z1_dot_vel"]) / len(flight_analysis["icm_z1_dot_vel"])
        icm_z2_mean = sum(flight_analysis["icm_z2_dot_vel"]) / len(flight_analysis["icm_z2_dot_vel"])

        summary_y = top3_adj + panel_height + 60
        elements.append(f'<rect x="{left}" y="{summary_y}" width="700" height="80" rx="8" fill="#fef3c7" stroke="#fcd34d"/>')
        elements.append(f'<text x="{left + 15}" y="{summary_y + 25}" font-size="16" font-weight="700" fill="#92400e">Flight Phase Summary (mean dot product with velocity)</text>')
        elements.append(f'<text x="{left + 15}" y="{summary_y + 50}" font-size="14" fill="#1f2937">'
                       f'LSM Conv1: {lsm_z1_mean:.3f} | Conv2: {lsm_z2_mean:.3f} | '
                       f'ICM Conv1: {icm_z1_mean:.3f} | Conv2: {icm_z2_mean:.3f}</text>')

        # Interpretation
        best_lsm_flight = "Conv1" if abs(lsm_z1_mean) > abs(lsm_z2_mean) else "Conv2"
        best_icm_flight = "Conv1" if abs(icm_z1_mean) > abs(icm_z2_mean) else "Conv2"
        elements.append(f'<text x="{left + 15}" y="{summary_y + 72}" font-size="13" fill="#b45309">'
                       f'Higher magnitude = better alignment. LSM best: {best_lsm_flight}, ICM best: {best_icm_flight}</text>')

    elements.append("</svg>")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(elements))


def main() -> None:
    parser = argparse.ArgumentParser(description="Diagnose body Z-axis alignment from quaternion.")
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT_CSV, help="Input decoded replay CSV.")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT_SVG, help="Output SVG path.")
    args = parser.parse_args()

    print(f"Loading samples from {args.input}...")
    samples = load_samples(args.input)
    if not samples:
        raise SystemExit("No valid samples found.")

    print(f"Loaded {len(samples)} samples")

    print("\n=== Ground Phase Analysis ===")
    ground_analysis = analyze_ground_phase(samples)
    if ground_analysis:
        print(f"LSM Conv1 (body-to-world) angle from up: {ground_analysis.get('lsm_conv1_dot_up_angle_deg', 0):.2f} deg")
        print(f"LSM Conv2 (world-to-body) angle from up: {ground_analysis.get('lsm_conv2_dot_up_angle_deg', 0):.2f} deg")
        print(f"ICM Conv1 (body-to-world) angle from up: {ground_analysis.get('icm_conv1_dot_up_angle_deg', 0):.2f} deg")
        print(f"ICM Conv2 (world-to-body) angle from up: {ground_analysis.get('icm_conv2_dot_up_angle_deg', 0):.2f} deg")
        print(f"Mean accel direction (body frame): {ground_analysis.get('accel_mean_direction', (0,0,0))}")
        print(f"Mean accel magnitude: {ground_analysis.get('accel_mean_magnitude', 0):.2f} m/s^2")

    print("\n=== Flight Phase Analysis ===")
    flight_analysis = analyze_flight_phase(samples)
    if flight_analysis and flight_analysis.get("time"):
        n = len(flight_analysis["time"])
        print(f"Analyzed {n} flight samples")
        lsm_z1_mean = sum(flight_analysis["lsm_z1_dot_vel"]) / n
        lsm_z2_mean = sum(flight_analysis["lsm_z2_dot_vel"]) / n
        icm_z1_mean = sum(flight_analysis["icm_z1_dot_vel"]) / n
        icm_z2_mean = sum(flight_analysis["icm_z2_dot_vel"]) / n
        print(f"LSM Conv1 mean dot velocity: {lsm_z1_mean:.3f}")
        print(f"LSM Conv2 mean dot velocity: {lsm_z2_mean:.3f}")
        print(f"ICM Conv1 mean dot velocity: {icm_z1_mean:.3f}")
        print(f"ICM Conv2 mean dot velocity: {icm_z2_mean:.3f}")

    print(f"\nWriting SVG to {args.output}...")
    write_svg(samples, ground_analysis, flight_analysis, args.output)
    print("Done!")


if __name__ == "__main__":
    main()
