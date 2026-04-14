#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import math
import statistics
from collections import Counter
from pathlib import Path


SOURCE_LABELS = {
    0: "None",
    1: "BNO",
    2: "ICM",
    3: "LSM",
    4: "Blended",
}


def normalize(q: list[float]) -> list[float]:
    norm = math.sqrt(sum(v * v for v in q))
    if not math.isfinite(norm) or norm < 1.0e-12:
        return [1.0, 0.0, 0.0, 0.0]
    return [v / norm for v in q]


def quat_dot(a: list[float], b: list[float]) -> float:
    a = normalize(a)
    b = normalize(b)
    return sum(x * y for x, y in zip(a, b))


def quat_angle_deg(a: list[float], b: list[float]) -> float:
    dot = abs(quat_dot(a, b))
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def quat_tilt_deg(q: list[float]) -> float:
    _, x, y, _ = normalize(q)
    gravity_z = 1.0 - 2.0 * (x * x + y * y)
    gravity_z = max(-1.0, min(1.0, gravity_z))
    return math.degrees(math.acos(gravity_z))


def quat_to_euler_deg(q: list[float]) -> tuple[float, float, float]:
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
    return tuple(math.degrees(v) for v in (yaw, pitch, roll))


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


def tilt_only(q: list[float]) -> list[float]:
    _, pitch_deg, roll_deg = quat_to_euler_deg(q)
    return quat_from_pitch_roll_deg(pitch_deg, roll_deg)


def buggy_tilt_only(q: list[float]) -> list[float]:
    _, pitch_deg, roll_deg = quat_to_euler_deg(q)
    pitch_rad = math.radians(pitch_deg)
    roll_rad = math.radians(roll_deg)
    return quat_from_pitch_roll_deg(pitch_rad, roll_rad)


def slerp(a: list[float], b: list[float], t: float) -> list[float]:
    a = normalize(a)
    b = normalize(b)
    dot = quat_dot(a, b)
    if dot < 0.0:
        b = [-v for v in b]
        dot = -dot
    if dot > 0.9995:
        return normalize([(1.0 - t) * x + t * y for x, y in zip(a, b)])
    theta_0 = math.acos(max(-1.0, min(1.0, dot)))
    sin_theta_0 = math.sin(theta_0)
    scale_a = math.sin((1.0 - t) * theta_0) / sin_theta_0
    scale_b = math.sin(t * theta_0) / sin_theta_0
    return [scale_a * x + scale_b * y for x, y in zip(a, b)]


def load_csv(path: Path) -> dict[str, list]:
    rows = list(csv.DictReader(path.open(newline="", encoding="utf-8")))
    time_s = [float(row["sensor_timestamp"]) for row in rows]
    source = [int(row["sensor_main_quaternion_source"]) for row in rows]
    main = [[float(row[f"sensor_quat_{axis}"]) for axis in "wxyz"] for row in rows]
    icm = [[float(row[f"sensor_icm_quat_{axis}"]) for axis in "wxyz"] for row in rows]
    lsm_raw = [[float(row[f"sensor_lsm_quat_{axis}"]) for axis in "wxyz"] for row in rows]
    state_zenith_deg = [float(row["state_zenith_deg"]) for row in rows]
    return {
        "rows": rows,
        "time_s": time_s,
        "source": source,
        "main": main,
        "icm": icm,
        "lsm_raw": lsm_raw,
        "state_zenith_deg": state_zenith_deg,
    }


def make_polyline_points(
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
    points: list[str] = []
    x_span = max(1.0e-9, x_max - x_min)
    y_span = max(1.0e-9, y_max - y_min)
    for x, y in zip(xs, ys):
        if not math.isfinite(y):
            continue
        px = left + width * (x - x_min) / x_span
        py = top + height * (1.0 - (y - y_min) / y_span)
        points.append(f"{px:.2f},{py:.2f}")
    return " ".join(points)


def svg_line(x1: float, y1: float, x2: float, y2: float, color: str, width: float, dash: str | None = None, alpha: float = 1.0) -> str:
    dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
    return (
        f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
        f'stroke="{color}" stroke-width="{width:.2f}" stroke-opacity="{alpha:.3f}"{dash_attr} />'
    )


def svg_text(x: float, y: float, text: str, size: int = 12, anchor: str = "start", color: str = "#2b221b", weight: str = "normal") -> str:
    safe = (
        text.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
    )
    return (
        f'<text x="{x:.2f}" y="{y:.2f}" font-size="{size}" text-anchor="{anchor}" '
        f'fill="{color}" font-family="Menlo, Consolas, monospace" font-weight="{weight}">{safe}</text>'
    )


def write_svg(
    output: Path,
    time_s: list[float],
    first_blended_t: float | None,
    w_series: list[tuple[str, str, list[float], float, float]],
    tilt_series: list[tuple[str, str, list[float], float, float]],
    error_series: list[tuple[str, str, list[float], float, float]],
    footer_lines: list[str],
) -> None:
    width = 1440
    height = 1040
    margin_left = 88
    margin_right = 32
    margin_top = 72
    panel_gap = 24
    footer_height = 130
    panel_height = (height - margin_top - footer_height - panel_gap * 2 - 36) / 3.0
    panel_width = width - margin_left - margin_right
    x_min = min(time_s)
    x_max = max(time_s)

    def panel_bounds(index: int) -> tuple[float, float, float, float]:
        top = margin_top + index * (panel_height + panel_gap)
        return margin_left, top, panel_width, panel_height

    def y_limits(series: list[tuple[str, str, list[float], float, float]], default_top: float | None = None) -> tuple[float, float]:
        values = [y for _, _, ys, _, _ in series for y in ys if math.isfinite(y)]
        y_min = min(values) if values else 0.0
        y_max = max(values) if values else 1.0
        if default_top is not None:
            y_max = max(y_max, default_top)
        if abs(y_max - y_min) < 1.0e-9:
            y_max = y_min + 1.0
        pad = 0.06 * (y_max - y_min)
        return y_min - pad, y_max + pad

    w_min, w_max = y_limits(w_series, default_top=1.0)
    tilt_min, tilt_max = 0.0, max(max(y for _, _, ys, _, _ in tilt_series for y in ys if math.isfinite(y)), 1.0) * 1.08
    error_min, error_max = 0.0, max(max(y for _, _, ys, _, _ in error_series for y in ys if math.isfinite(y)), 1.0) * 1.08

    bg = "#fffdfa"
    panel_bg = "#faf7f2"
    grid = "#ddd3c7"
    edge = "#c9c0b3"
    text = "#2b221b"
    accent = "#6f4e37"

    parts: list[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        f'<rect x="0" y="0" width="{width}" height="{height}" fill="{bg}" />',
        svg_text(margin_left, 34, "Quaternion Path Diagnosis: raw rails, tilt rails, reconstructed blend, and logged main", size=22, color=text, weight="bold"),
        svg_text(margin_left, 56, "ICM raw is not logged in this file; the logged ICM rail is already tilt-only. LSM raw is logged before tilt normalization.", size=12, color="#5b4a3c"),
    ]

    legend_items = [
        ("ICM logged rail", "#1f77b4"),
        ("LSM raw rail", "#d95f02"),
        ("LSM tilt-only rail", "#2ca02c"),
        ("Reconstructed pre-tilt blend", "#8c564b"),
        ("Logged main quaternion", "#111111"),
        ("Selected-path vs logged-main error", "#b2182b"),
    ]
    lx = margin_left
    ly = 84
    for label, color in legend_items:
        parts.append(svg_line(lx, ly - 5, lx + 22, ly - 5, color, 3.0))
        parts.append(svg_text(lx + 28, ly, label, size=12, color=text))
        lx += 220
        if lx > width - 280:
            lx = margin_left
            ly += 18

    def draw_panel(index: int, title: str, y_label: str, series: list[tuple[str, str, list[float], float, float]], y_min: float, y_max: float) -> None:
        left, top, p_width, p_height = panel_bounds(index)
        parts.append(f'<rect x="{left:.2f}" y="{top:.2f}" width="{p_width:.2f}" height="{p_height:.2f}" fill="{panel_bg}" stroke="{edge}" stroke-width="1.0" rx="8" ry="8" />')
        parts.append(svg_text(left + 12, top + 22, title, size=15, color=text, weight="bold"))
        parts.append(svg_text(left - 54, top + p_height * 0.5, y_label, size=12, anchor="middle", color="#5b4a3c"))

        for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
            y_val = y_min + frac * (y_max - y_min)
            py = top + p_height * (1.0 - frac)
            parts.append(svg_line(left, py, left + p_width, py, grid, 1.0))
            parts.append(svg_text(left - 8, py + 4, f"{y_val:.2f}", size=10, anchor="end", color="#6c5d50"))

        for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
            x_val = x_min + frac * (x_max - x_min)
            px = left + p_width * frac
            parts.append(svg_line(px, top, px, top + p_height, grid, 1.0))
            if index == 2:
                parts.append(svg_text(px, top + p_height + 18, f"{x_val:.1f}", size=10, anchor="middle", color="#6c5d50"))

        if first_blended_t is not None:
            x_span = max(1.0e-9, x_max - x_min)
            px = left + p_width * (first_blended_t - x_min) / x_span
            parts.append(svg_line(px, top, px, top + p_height, accent, 1.6, dash="6,6", alpha=0.85))
            if index == 0:
                parts.append(svg_text(px + 8, top + 38, "first blended sample", size=11, color=accent))

        for _, color, ys, width_px, alpha in series:
            points = make_polyline_points(time_s, ys, x_min, x_max, y_min, y_max, left, top, p_width, p_height)
            parts.append(
                f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="{width_px:.2f}" '
                f'stroke-opacity="{alpha:.3f}" stroke-linejoin="round" stroke-linecap="round" />'
            )

    draw_panel(0, "Quaternion w", "w", w_series, w_min, w_max)
    draw_panel(1, "Tilt Magnitude", "deg", tilt_series, tilt_min, tilt_max)
    draw_panel(2, "Selected-Path Reconstruction Error", "deg", error_series, error_min, error_max)

    footer_top = height - footer_height + 8
    parts.append(svg_text(margin_left, footer_top, "Summary", size=15, color=text, weight="bold"))
    for idx, line in enumerate(footer_lines):
        parts.append(svg_text(margin_left, footer_top + 22 + idx * 16, line, size=11, color="#4f4338"))

    parts.append("</svg>")
    output.write_text("\n".join(parts), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "csv_path",
        nargs="?",
        default=Path(__file__).with_name("fullscale_4.csv"),
        type=Path,
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).with_name("fullscale_4_quaternion_path_diagnosis.svg"),
    )
    args = parser.parse_args()

    data = load_csv(args.csv_path)
    time_s = data["time_s"]
    source = data["source"]
    main_q = data["main"]
    icm_q = data["icm"]
    lsm_raw_q = data["lsm_raw"]

    lsm_tilt_q = [tilt_only(q) for q in lsm_raw_q]
    lsm_buggy_tilt_q = [buggy_tilt_only(q) for q in lsm_raw_q]
    pre_blend_q = [slerp(icm, lsm_tilt, 0.5) for icm, lsm_tilt in zip(icm_q, lsm_tilt_q)]
    pre_blend_buggy_q = [slerp(icm, lsm_buggy, 0.5) for icm, lsm_buggy in zip(icm_q, lsm_buggy_tilt_q)]

    expected_selected_q: list[list[float]] = []
    buggy_expected_main_q: list[list[float]] = []
    for src, icm, lsm_tilt, pre_blend in zip(source, icm_q, lsm_tilt_q, pre_blend_q):
        if src == 4:
            expected_selected_q.append(tilt_only(pre_blend))
        elif src == 2:
            expected_selected_q.append(tilt_only(icm))
        elif src == 3:
            expected_selected_q.append(tilt_only(lsm_tilt))
        else:
            expected_selected_q.append([float("nan")] * 4)
    for src, icm, lsm_raw, pre_blend_buggy in zip(source, icm_q, lsm_raw_q, pre_blend_buggy_q):
        if src == 4:
            buggy_expected_main_q.append(buggy_tilt_only(pre_blend_buggy))
        elif src == 2:
            buggy_expected_main_q.append(buggy_tilt_only(icm))
        elif src == 3:
            buggy_expected_main_q.append(buggy_tilt_only(buggy_tilt_only(lsm_raw)))
        else:
            buggy_expected_main_q.append([float("nan")] * 4)

    icm_tilt = [quat_tilt_deg(q) for q in icm_q]
    lsm_raw_tilt = [quat_tilt_deg(q) for q in lsm_raw_q]
    lsm_tilt = [quat_tilt_deg(q) for q in lsm_tilt_q]
    lsm_buggy_tilt = [quat_tilt_deg(q) for q in lsm_buggy_tilt_q]
    pre_blend_tilt = [quat_tilt_deg(q) for q in pre_blend_q]
    pre_blend_buggy_tilt = [quat_tilt_deg(q) for q in pre_blend_buggy_q]
    main_tilt = [quat_tilt_deg(q) for q in main_q]
    expected_selected_tilt = [
        quat_tilt_deg(q) if math.isfinite(q[0]) else float("nan")
        for q in expected_selected_q
    ]
    buggy_expected_main_tilt = [
        quat_tilt_deg(q) if math.isfinite(q[0]) else float("nan")
        for q in buggy_expected_main_q
    ]
    selected_error_deg = [
        quat_angle_deg(expected, actual) if math.isfinite(expected[0]) else float("nan")
        for expected, actual in zip(expected_selected_q, main_q)
    ]
    buggy_model_error_deg = [
        quat_angle_deg(expected, actual) if math.isfinite(expected[0]) else float("nan")
        for expected, actual in zip(buggy_expected_main_q, main_q)
    ]

    blended_mask = [src == 4 for src in source]
    icm_mask = [src == 2 for src in source]
    lsm_mask = [src == 3 for src in source]
    blended_errors = [err for err, mask in zip(selected_error_deg, blended_mask) if mask and math.isfinite(err)]
    blended_buggy_errors = [err for err, mask in zip(buggy_model_error_deg, blended_mask) if mask and math.isfinite(err)]
    icm_errors = [err for err, mask in zip(selected_error_deg, icm_mask) if mask and math.isfinite(err)]
    icm_buggy_errors = [err for err, mask in zip(buggy_model_error_deg, icm_mask) if mask and math.isfinite(err)]
    lsm_errors = [err for err, mask in zip(selected_error_deg, lsm_mask) if mask and math.isfinite(err)]
    lsm_buggy_errors = [err for err, mask in zip(buggy_model_error_deg, lsm_mask) if mask and math.isfinite(err)]
    blended_expected_tilt = [v for v, mask in zip(expected_selected_tilt, blended_mask) if mask and math.isfinite(v)]
    blended_main_tilt = [v for v, mask in zip(main_tilt, blended_mask) if mask and math.isfinite(v)]
    blended_buggy_tilt = [v for v, mask in zip(buggy_expected_main_tilt, blended_mask) if mask and math.isfinite(v)]

    first_blended_idx = next((idx for idx, src in enumerate(source) if src == 4), None)
    first_blended_t = time_s[first_blended_idx] if first_blended_idx is not None else None
    source_counts = Counter(source)

    w_series = [
        ("ICM logged rail", "#1f77b4", [q[0] for q in icm_q], 1.7, 0.95),
        ("LSM raw rail", "#d95f02", [q[0] for q in lsm_raw_q], 1.2, 0.72),
        ("LSM tilt-only rail", "#2ca02c", [q[0] for q in lsm_tilt_q], 1.5, 0.9),
        (
            "Reconstructed pre-tilt blend",
            "#8c564b",
            [q[0] if mask else float("nan") for q, mask in zip(pre_blend_q, blended_mask)],
            1.6,
            0.95,
        ),
        ("Logged main quaternion", "#111111", [q[0] for q in main_q], 1.7, 0.95),
    ]

    tilt_series = [
        ("ICM logged rail", "#1f77b4", icm_tilt, 1.7, 0.95),
        ("LSM raw rail", "#d95f02", lsm_raw_tilt, 1.2, 0.65),
        ("LSM tilt-only rail", "#2ca02c", lsm_tilt, 1.5, 0.9),
        (
            "Reconstructed pre-tilt blend",
            "#8c564b",
            [v if mask else float("nan") for v, mask in zip(pre_blend_tilt, blended_mask)],
            1.6,
            0.95,
        ),
        (
            "Buggy LSM tilt",
            "#66a61e",
            [v if mask else float("nan") for v, mask in zip(lsm_buggy_tilt, blended_mask)],
            1.0,
            0.55,
        ),
        (
            "Buggy pre-tilt blend",
            "#7570b3",
            [v if mask else float("nan") for v, mask in zip(pre_blend_buggy_tilt, blended_mask)],
            1.2,
            0.8,
        ),
        ("Logged main quaternion", "#111111", main_tilt, 1.8, 0.95),
    ]

    error_series = [
        ("Expected selected-path tilt", "#8c564b", expected_selected_tilt, 1.5, 0.95),
        ("Buggy radians-as-degrees model", "#7570b3", buggy_expected_main_tilt, 1.5, 0.95),
        ("Logged main tilt", "#111111", main_tilt, 1.8, 0.95),
        ("Selected-path vs logged-main error", "#b2182b", selected_error_deg, 1.5, 0.95),
        ("Buggy-model vs logged-main error", "#1b9e77", buggy_model_error_deg, 1.3, 0.9),
    ]

    footer_lines = [
        f"Rows: {len(time_s):,}",
        "Source counts: " + ", ".join(f"{SOURCE_LABELS.get(key, str(key))}={source_counts[key]}" for key in sorted(source_counts)),
        f"Blended expected tilt mean: {statistics.mean(blended_expected_tilt):.2f} deg",
        f"Blended buggy-model tilt mean: {statistics.mean(blended_buggy_tilt):.3f} deg",
        f"Blended logged-main tilt mean: {statistics.mean(blended_main_tilt):.3f} deg",
        f"Blended mean quaternion error: {statistics.mean(blended_errors):.2f} deg",
        f"Blended buggy-model mean error: {statistics.mean(blended_buggy_errors):.4f} deg",
        f"ICM-only mean error: correct={statistics.mean(icm_errors):.2f} deg, buggy={statistics.mean(icm_buggy_errors):.6f} deg",
    ]
    write_svg(args.output, time_s, first_blended_t, w_series, tilt_series, error_series, footer_lines)

    print(f"wrote {args.output}")
    print(f"rows: {len(time_s)}")
    print("source counts:", ", ".join(f"{SOURCE_LABELS.get(key, str(key))}={source_counts[key]}" for key in sorted(source_counts)))
    print(f"blended expected tilt mean: {statistics.mean(blended_expected_tilt):.6f} deg")
    print(f"blended buggy-model tilt mean: {statistics.mean(blended_buggy_tilt):.6f} deg")
    print(f"blended logged-main tilt mean: {statistics.mean(blended_main_tilt):.6f} deg")
    print(f"blended mean quaternion error: {statistics.mean(blended_errors):.6f} deg")
    print(f"blended buggy-model mean error: {statistics.mean(blended_buggy_errors):.6f} deg")
    print(f"blended median quaternion error: {statistics.median(blended_errors):.6f} deg")
    print(f"blended max quaternion error: {max(blended_errors):.6f} deg")
    print(f"ICM-only mean quaternion error: {statistics.mean(icm_errors):.6f} deg")
    print(f"ICM-only buggy-model mean error: {statistics.mean(icm_buggy_errors):.9f} deg")
    if lsm_errors:
        print(f"LSM-only mean quaternion error: {statistics.mean(lsm_errors):.6f} deg")
    if lsm_buggy_errors:
        print(f"LSM-only buggy-model mean error: {statistics.mean(lsm_buggy_errors):.6f} deg")
    if first_blended_idx is not None:
        print("first blended sample:")
        print(f"  t = {time_s[first_blended_idx]:.6f} s")
        print(f"  icm tilt = {icm_tilt[first_blended_idx]:.6f} deg")
        print(f"  lsm raw tilt = {lsm_raw_tilt[first_blended_idx]:.6f} deg")
        print(f"  lsm tilt-only = {lsm_tilt[first_blended_idx]:.6f} deg")
        print(f"  buggy lsm tilt-only = {lsm_buggy_tilt[first_blended_idx]:.6f} deg")
        print(f"  reconstructed pre-tilt blend = {pre_blend_tilt[first_blended_idx]:.6f} deg")
        print(f"  buggy reconstructed pre-tilt blend = {pre_blend_buggy_tilt[first_blended_idx]:.6f} deg")
        print(f"  logged main tilt = {main_tilt[first_blended_idx]:.6f} deg")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
