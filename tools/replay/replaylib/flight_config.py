"""Named replay flight configurations."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from .paths import DATA_DIR, PLOTS_DIR


@dataclass(frozen=True)
class FlightConfig:
    name: str
    input_csv: Path
    default_plot: Path
    description: str


FLIGHTS: dict[str, FlightConfig] = {
    "fullscale_2": FlightConfig(
        name="fullscale_2",
        input_csv=DATA_DIR / "fullscale_2_replay.csv",
        default_plot=PLOTS_DIR / "fullscale2_replay_detailed_comparison.svg",
        description="Historical fullscale 2 replay data.",
    ),
    "fullscale_3": FlightConfig(
        name="fullscale_3",
        input_csv=DATA_DIR / "fullscale_3_historical.csv",
        default_plot=PLOTS_DIR / "fullscale_3_causal_reconstruction.svg",
        description="Historical fullscale 3 reconstruction data.",
    ),
    "fullscale_4": FlightConfig(
        name="fullscale_4",
        input_csv=DATA_DIR / "fullscale_4.csv",
        default_plot=PLOTS_DIR / "fullscale_4_validation.svg",
        description="Historical fullscale 4 replay data.",
    ),
    "sens065": FlightConfig(
        name="sens065",
        input_csv=DATA_DIR / "SENS065.csv",
        default_plot=PLOTS_DIR / "SENS065_framefix_predictor_actual_clean.svg",
        description="Huntsville SENS065 replay data.",
    ),
}


def get_flight(name: str) -> FlightConfig:
    try:
        return FLIGHTS[name]
    except KeyError as exc:
        known = ", ".join(sorted(FLIGHTS))
        raise KeyError(f"Unknown flight {name!r}. Known flights: {known}") from exc

