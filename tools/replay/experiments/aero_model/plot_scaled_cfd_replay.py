#!/usr/bin/env python3
"""
Create a replay-only scaled CFD table under /tmp, rerun acs_replay against it,
and compare the resulting apogee trace against the logged flight.

This avoids leaving any permanent geometry-specific cfd.csv artifact in the
repository. The script supports three ways to choose the scale:

1. Direct force scaling with --force-scale.
2. Geometry scaling from a reference diameter ratio, using area scaling.
3. Target fitting with --fit-target-apogee-ft, which solves for a replay-only
   force scale that makes the peak predicted apogee hit the requested target.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import subprocess
import tempfile
from pathlib import Path

from plot_no_deweight_apogee_replay import DEFAULT_INPUT_CSV
from plot_no_deweight_apogee_replay import LoggedSample
from plot_no_deweight_apogee_replay import ReplaySample
from plot_no_deweight_apogee_replay import draw_panel
from plot_no_deweight_apogee_replay import find_replay_binary
from plot_no_deweight_apogee_replay import load_logged_samples
from plot_no_deweight_apogee_replay import write_replay_input_csv


from replaylib.paths import SCRIPTS_DIR
SCRIPT_DIR = SCRIPTS_DIR
REPLAY_DIR = SCRIPT_DIR.parent
ROOT = REPLAY_DIR.parents[1]

DEFAULT_BASE_CFD = ROOT / "lib" / "cfd.csv"
DEFAULT_OUTPUT_SVG = SCRIPT_DIR / "fullscale4_scaled_cfd_replay.svg"

FEET_TO_METERS = 0.3048
METERS_TO_FEET = 1.0 / FEET_TO_METERS


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


def sanitize_filename(text: str) -> str:
    out: list[str] = []
    for ch in text:
        if ch.isalnum():
            out.append(ch)
        elif ch in ("-", "_"):
            out.append(ch)
        else:
            out.append("_")
    return "".join(out).strip("_") or "output"


def run_hosted_replay(
    replay_binary: Path, replay_input_csv: Path, cfd_path: Path | None = None
) -> list[ReplaySample]:
    command = [str(replay_binary), str(replay_input_csv), "--ignore-logged-state"]
    if cfd_path is not None:
        command.extend(["--cfd-path", str(cfd_path)])

    result = subprocess.run(
        command,
        cwd=str(ROOT),
        text=True,
        capture_output=True,
        check=False,
    )
    if result.returncode != 0:
        raise SystemExit(result.stderr.strip() or result.stdout.strip() or "acs_replay failed")

    lines = result.stdout.splitlines()
    try:
        header_index = next(index for index, line in enumerate(lines) if line.startswith("time_s,"))
    except StopIteration as exc:
        raise SystemExit("acs_replay did not emit replay CSV output") from exc

    headers = lines[header_index].split(",")
    if headers[:5] != ["time_s", "altitude_m", "velocity_mps", "apogee_prediction_m", "status"]:
        raise SystemExit("Unexpected acs_replay output header")

    samples: list[ReplaySample] = []
    for line in lines[header_index + 1 :]:
        if line.startswith("Samples processed:"):
            break
        parts = line.split(",")
        if len(parts) != len(headers):
            continue
        try:
            time_s = float(parts[0])
            altitude_ft = float(parts[1]) * METERS_TO_FEET
            velocity_fps = float(parts[2]) * METERS_TO_FEET
            apogee_ft = float(parts[3]) * METERS_TO_FEET
        except ValueError:
            continue
        samples.append(
            ReplaySample(
                time_s=time_s,
                altitude_ft=altitude_ft,
                velocity_fps=velocity_fps,
                apogee_ft=apogee_ft,
                status=parts[4].strip().lower(),
            )
        )

    if not samples:
        raise SystemExit("acs_replay emitted no usable sample rows")
    return samples


def baro_peak_ft(logged_samples: list[LoggedSample]) -> float:
    return max(
        sample.baro_agl_ft
        for sample in logged_samples
        if sample.baro_agl_ft is not None and math.isfinite(sample.baro_agl_ft)
    )


def state_final_apogee_ft(logged_samples: list[LoggedSample]) -> float:
    return next(
        sample.state_apogee_ft
        for sample in reversed(logged_samples)
        if sample.state_apogee_ft is not None and math.isfinite(sample.state_apogee_ft)
    )


def replay_peak_apogee_ft(samples: list[ReplaySample]) -> float:
    return max(sample.apogee_ft for sample in samples if math.isfinite(sample.apogee_ft))


def replay_final_apogee_ft(samples: list[ReplaySample]) -> float:
    return next(sample.apogee_ft for sample in reversed(samples) if math.isfinite(sample.apogee_ft))


def write_scaled_cfd_csv(
    base_cfd_csv: Path, uniform_force_scale: float, axial_multiplier: float, normal_multiplier: float
) -> Path:
    fd, temp_path = tempfile.mkstemp(
        suffix=".csv",
        prefix=f"{sanitize_filename(base_cfd_csv.stem)}_scaled_",
        dir="/tmp",
    )
    os.close(fd)
    output_path = Path(temp_path)

    axial_scale = uniform_force_scale * axial_multiplier
    normal_scale = uniform_force_scale * normal_multiplier

    with base_cfd_csv.open(newline="") as handle_in, output_path.open("w", newline="") as handle_out:
        reader = csv.DictReader(line for line in handle_in if line.strip())
        if reader.fieldnames is None:
            raise SystemExit(f"Missing CFD CSV header in {base_cfd_csv}")

        writer = csv.DictWriter(handle_out, fieldnames=reader.fieldnames)
        writer.writeheader()
        for raw in reader:
            row = dict(raw)
            axial_force = parse_float(row.get("axial force"))
            normal_force = parse_float(row.get("normal force"))
            if axial_force is not None:
                row["axial force"] = f"{axial_force * axial_scale:.12g}"
            if normal_force is not None:
                row["normal force"] = f"{normal_force * normal_scale:.12g}"
            writer.writerow({header: row.get(header, "") for header in reader.fieldnames})

    return output_path


class ReplayEvaluator:
    def __init__(
        self,
        replay_binary: Path,
        replay_input_csv: Path,
        base_cfd_csv: Path,
        axial_multiplier: float,
        normal_multiplier: float,
        keep_temp_cfd: bool,
    ) -> None:
        self._replay_binary = replay_binary
        self._replay_input_csv = replay_input_csv
        self._base_cfd_csv = base_cfd_csv
        self._axial_multiplier = axial_multiplier
        self._normal_multiplier = normal_multiplier
        self._keep_temp_cfd = keep_temp_cfd
        self._sample_cache: dict[float, list[ReplaySample]] = {}
        self._temp_paths: list[Path] = []

    def evaluate(self, uniform_force_scale: float) -> list[ReplaySample]:
        key = round(uniform_force_scale, 12)
        cached = self._sample_cache.get(key)
        if cached is not None:
            return cached

        temp_cfd = write_scaled_cfd_csv(
            self._base_cfd_csv,
            uniform_force_scale=uniform_force_scale,
            axial_multiplier=self._axial_multiplier,
            normal_multiplier=self._normal_multiplier,
        )
        self._temp_paths.append(temp_cfd)
        try:
            samples = run_hosted_replay(
                self._replay_binary,
                self._replay_input_csv,
                cfd_path=temp_cfd,
            )
        finally:
            if not self._keep_temp_cfd:
                try:
                    temp_cfd.unlink()
                except OSError:
                    pass
        self._sample_cache[key] = samples
        return samples

    def cleanup(self) -> None:
        if self._keep_temp_cfd:
            return
        for path in self._temp_paths:
            try:
                path.unlink()
            except OSError:
                pass
        self._temp_paths.clear()


def solve_force_scale_for_target(
    evaluator: ReplayEvaluator,
    target_apogee_ft: float,
    initial_scale: float = 1.0,
    min_scale: float = 0.05,
    max_scale: float = 4.0,
    max_iterations: int = 24,
) -> tuple[float, list[ReplaySample], bool]:
    def objective(scale: float) -> float:
        return replay_peak_apogee_ft(evaluator.evaluate(scale)) - target_apogee_ft

    mid_scale = initial_scale
    mid_value = objective(mid_scale)
    if abs(mid_value) <= 0.25:
        return mid_scale, evaluator.evaluate(mid_scale), True

    lower_scale = mid_scale
    upper_scale = mid_scale
    lower_value = mid_value
    upper_value = mid_value

    if mid_value < 0.0:
        while lower_scale > min_scale and lower_value < 0.0:
            upper_scale = lower_scale
            upper_value = lower_value
            lower_scale = max(min_scale, lower_scale * 0.5)
            lower_value = objective(lower_scale)
            if lower_scale == min_scale:
                break
    else:
        while upper_scale < max_scale and upper_value > 0.0:
            lower_scale = upper_scale
            lower_value = upper_value
            upper_scale = min(max_scale, upper_scale * 2.0)
            upper_value = objective(upper_scale)
            if upper_scale == max_scale:
                break

    bracketed = lower_value >= 0.0 and upper_value <= 0.0
    if not bracketed:
        candidates = [
            (lower_scale, abs(lower_value)),
            (mid_scale, abs(mid_value)),
            (upper_scale, abs(upper_value)),
        ]
        best_scale = min(candidates, key=lambda item: item[1])[0]
        return best_scale, evaluator.evaluate(best_scale), False

    for _ in range(max_iterations):
        mid_scale = 0.5 * (lower_scale + upper_scale)
        mid_value = objective(mid_scale)
        if abs(mid_value) <= 0.25:
            return mid_scale, evaluator.evaluate(mid_scale), True
        if mid_value > 0.0:
            lower_scale = mid_scale
            lower_value = mid_value
        else:
            upper_scale = mid_scale
            upper_value = mid_value

    best_scale = 0.5 * (lower_scale + upper_scale)
    return best_scale, evaluator.evaluate(best_scale), True


def compute_force_scale_from_geometry(full_diameter: float, subscale_diameter: float) -> float:
    if full_diameter <= 0.0 or subscale_diameter <= 0.0:
        raise SystemExit("Diameter inputs must be positive.")
    diameter_ratio = subscale_diameter / full_diameter
    return diameter_ratio * diameter_ratio


def scale(value: float, src_lo: float, src_hi: float, dst_lo: float, dst_hi: float) -> float:
    if src_hi <= src_lo:
        return 0.5 * (dst_lo + dst_hi)
    ratio = (value - src_lo) / (src_hi - src_lo)
    return dst_lo + ratio * (dst_hi - dst_lo)


def write_svg(
    output_path: Path,
    logged_samples: list[LoggedSample],
    base_replay_samples: list[ReplaySample],
    scaled_replay_samples: list[ReplaySample],
    zoom_start: float,
    zoom_end: float,
    scale_label: str,
    base_scale: float,
    axial_scale: float,
    normal_scale: float,
) -> None:
    width = 1500
    height = 1120
    margin_left = 100.0
    panel_width = 1320.0
    panel_height = 250.0

    logged_times = [sample.time_s for sample in logged_samples]
    x_overview_min = logged_times[0]
    x_overview_max = logged_times[-1]

    logged_baro_points = [(sample.time_s, sample.baro_agl_ft) for sample in logged_samples]
    logged_state_alt_points = [(sample.time_s, sample.state_agl_ft) for sample in logged_samples]
    logged_state_apogee_points = [(sample.time_s, sample.state_apogee_ft) for sample in logged_samples]

    base_alt_points = [(sample.time_s, sample.altitude_ft) for sample in base_replay_samples]
    base_apogee_points = [(sample.time_s, sample.apogee_ft) for sample in base_replay_samples]
    scaled_alt_points = [(sample.time_s, sample.altitude_ft) for sample in scaled_replay_samples]
    scaled_apogee_points = [(sample.time_s, sample.apogee_ft) for sample in scaled_replay_samples]

    max_alt_ft = max(
        value
        for _, value in logged_baro_points
        + logged_state_alt_points
        + base_alt_points
        + scaled_alt_points
        if value is not None and math.isfinite(value)
    )
    max_apogee_ft = max(
        value
        for _, value in logged_state_apogee_points + base_apogee_points + scaled_apogee_points
        if value is not None and math.isfinite(value)
    )
    y_alt_max = math.ceil((max_alt_ft + 100.0) / 100.0) * 100.0
    y_apogee_max = math.ceil((max_apogee_ft + 100.0) / 100.0) * 100.0

    baro_peak = baro_peak_ft(logged_samples)
    logged_final = state_final_apogee_ft(logged_samples)
    base_peak = replay_peak_apogee_ft(base_replay_samples)
    scaled_peak = replay_peak_apogee_ft(scaled_replay_samples)
    base_final = replay_final_apogee_ft(base_replay_samples)
    scaled_final = replay_final_apogee_ft(scaled_replay_samples)

    subtitle_lines = [
        scale_label,
        f"Uniform force scale: {base_scale:.5f} | Axial scale: {axial_scale:.5f} | Normal scale: {normal_scale:.5f}",
        f"Logged final state apogee: {logged_final:.1f} ft | Baro peak: {baro_peak:.1f} ft",
        f"Base replay peak/final: {base_peak:.1f} / {base_final:.1f} ft",
        f"Scaled replay peak/final: {scaled_peak:.1f} / {scaled_final:.1f} ft",
    ]

    elements = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        f'<text x="{width / 2:.2f}" y="42" text-anchor="middle" font-family="monospace" '
        'font-size="28" font-weight="700" fill="#111827">Replay-Only Scaled CFD vs Logged Apogee</text>',
    ]

    for index, line in enumerate(subtitle_lines):
        elements.append(
            f'<text x="{width / 2:.2f}" y="{74 + index * 18:.2f}" text-anchor="middle" '
            'font-family="monospace" font-size="15" fill="#374151">'
            f"{line}</text>"
        )

    elements.append(
        draw_panel(
            title="Altitude AGL Overview (ft)",
            x_min=x_overview_min,
            x_max=x_overview_max,
            y_min=0.0,
            y_max=y_alt_max,
            left=margin_left,
            top=180.0,
            width=panel_width,
            height=panel_height,
            phase_source=logged_samples,
            series=[
                ("Baro AGL", logged_baro_points, "#111827", ""),
                ("Logged State Alt", logged_state_alt_points, "#2563eb", ""),
                ("Replay Alt (Base CFD)", base_alt_points, "#b45309", "8 6"),
                ("Replay Alt (Scaled CFD)", scaled_alt_points, "#dc2626", ""),
            ],
            reference_lines=[],
        )
    )
    elements.append(
        draw_panel(
            title="Altitude AGL Flight Window (ft)",
            x_min=zoom_start,
            x_max=zoom_end,
            y_min=0.0,
            y_max=y_alt_max,
            left=margin_left,
            top=510.0,
            width=panel_width,
            height=panel_height,
            phase_source=logged_samples,
            series=[
                ("Baro AGL", logged_baro_points, "#111827", ""),
                ("Logged State Alt", logged_state_alt_points, "#2563eb", ""),
                ("Replay Alt (Base CFD)", base_alt_points, "#b45309", "8 6"),
                ("Replay Alt (Scaled CFD)", scaled_alt_points, "#dc2626", ""),
            ],
            reference_lines=[],
        )
    )
    elements.append(
        draw_panel(
            title="Apogee Prediction Flight Window (ft)",
            x_min=zoom_start,
            x_max=zoom_end,
            y_min=0.0,
            y_max=y_apogee_max,
            left=margin_left,
            top=840.0,
            width=panel_width,
            height=240.0,
            phase_source=logged_samples,
            series=[
                ("Logged State Apogee", logged_state_apogee_points, "#1d4ed8", ""),
                ("Replay Apogee (Base CFD)", base_apogee_points, "#b45309", "8 6"),
                ("Replay Apogee (Scaled CFD)", scaled_apogee_points, "#dc2626", ""),
            ],
            reference_lines=[("Baro AGL Peak", baro_peak, "#059669")],
        )
    )
    elements.append("</svg>")

    output_path.write_text("\n".join(elements))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Scale lib/cfd.csv into a temporary replay-only table and rerun hosted replay."
    )
    parser.add_argument("input_csv", nargs="?", type=Path, default=DEFAULT_INPUT_CSV)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT_SVG)
    parser.add_argument("--replay-binary", type=Path, default=None)
    parser.add_argument("--base-cfd", type=Path, default=DEFAULT_BASE_CFD)
    parser.add_argument("--force-scale", type=float, default=None)
    parser.add_argument("--full-diameter", type=float, default=None)
    parser.add_argument("--subscale-diameter", type=float, default=None)
    parser.add_argument("--fit-target-apogee-ft", type=float, default=None)
    parser.add_argument("--axial-multiplier", type=float, default=1.0)
    parser.add_argument("--normal-multiplier", type=float, default=1.0)
    parser.add_argument("--zoom-start", type=float, default=178.5)
    parser.add_argument("--zoom-end", type=float, default=191.7)
    parser.add_argument("--keep-temp-cfd", action="store_true")
    args = parser.parse_args()

    if not args.base_cfd.is_file():
        raise SystemExit(f"Base CFD CSV not found: {args.base_cfd}")

    replay_binary = find_replay_binary(args.replay_binary)
    logged_samples = load_logged_samples(args.input_csv)
    temp_input_csv = write_replay_input_csv(args.input_csv)

    evaluator = ReplayEvaluator(
        replay_binary=replay_binary,
        replay_input_csv=temp_input_csv,
        base_cfd_csv=args.base_cfd,
        axial_multiplier=args.axial_multiplier,
        normal_multiplier=args.normal_multiplier,
        keep_temp_cfd=args.keep_temp_cfd,
    )

    try:
        base_replay_samples = evaluator.evaluate(1.0)

        mode_label = ""
        bracketed = True
        if args.fit_target_apogee_ft is not None:
            uniform_force_scale, scaled_replay_samples, bracketed = solve_force_scale_for_target(
                evaluator,
                target_apogee_ft=args.fit_target_apogee_ft,
            )
            mode_label = f"Fit mode: scaled to target peak apogee {args.fit_target_apogee_ft:.1f} ft"
        elif args.force_scale is not None:
            uniform_force_scale = args.force_scale
            scaled_replay_samples = evaluator.evaluate(uniform_force_scale)
            mode_label = f"Direct mode: user-specified uniform force scale {uniform_force_scale:.5f}"
        elif args.full_diameter is not None or args.subscale_diameter is not None:
            if args.full_diameter is None or args.subscale_diameter is None:
                raise SystemExit("Provide both --full-diameter and --subscale-diameter together.")
            uniform_force_scale = compute_force_scale_from_geometry(
                full_diameter=args.full_diameter,
                subscale_diameter=args.subscale_diameter,
            )
            scaled_replay_samples = evaluator.evaluate(uniform_force_scale)
            mode_label = (
                f"Geometry mode: area scale from diameter ratio {args.subscale_diameter:.5g} / "
                f"{args.full_diameter:.5g}"
            )
        else:
            target_apogee_ft = baro_peak_ft(logged_samples)
            uniform_force_scale, scaled_replay_samples, bracketed = solve_force_scale_for_target(
                evaluator,
                target_apogee_ft=target_apogee_ft,
            )
            mode_label = f"Fit mode: scaled to logged baro peak {target_apogee_ft:.1f} ft"

        axial_scale = uniform_force_scale * args.axial_multiplier
        normal_scale = uniform_force_scale * args.normal_multiplier

        write_svg(
            args.output,
            logged_samples,
            base_replay_samples,
            scaled_replay_samples,
            args.zoom_start,
            args.zoom_end,
            scale_label=mode_label,
            base_scale=uniform_force_scale,
            axial_scale=axial_scale,
            normal_scale=normal_scale,
        )

        print(f"Input CSV: {args.input_csv}")
        print(f"Replay binary: {replay_binary}")
        print(f"Base CFD: {args.base_cfd}")
        print(f"Output SVG: {args.output}")
        print(f"Mode: {mode_label}")
        if args.fit_target_apogee_ft is not None or (
            args.force_scale is None and args.full_diameter is None and args.subscale_diameter is None
        ):
            print(f"Fit bracketed cleanly: {'yes' if bracketed else 'no'}")
        print(f"Uniform force scale: {uniform_force_scale:.8f}")
        print(f"Axial force scale: {axial_scale:.8f}")
        print(f"Normal force scale: {normal_scale:.8f}")
        print(f"Logged final state apogee: {state_final_apogee_ft(logged_samples):.2f} ft")
        print(f"Baro AGL peak: {baro_peak_ft(logged_samples):.2f} ft")
        print(f"Base replay peak apogee: {replay_peak_apogee_ft(base_replay_samples):.2f} ft")
        print(f"Base replay final apogee: {replay_final_apogee_ft(base_replay_samples):.2f} ft")
        print(f"Scaled replay peak apogee: {replay_peak_apogee_ft(scaled_replay_samples):.2f} ft")
        print(f"Scaled replay final apogee: {replay_final_apogee_ft(scaled_replay_samples):.2f} ft")
    finally:
        evaluator.cleanup()
        try:
            temp_input_csv.unlink()
        except OSError:
            pass


if __name__ == "__main__":
    main()
