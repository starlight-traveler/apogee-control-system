"""Reusable apogee-analysis helpers."""

from __future__ import annotations

from collections.abc import Iterable

from .csv_io import Row, parse_float
from .plotting import feet_to_meters, meters_to_feet


def peak_value(rows: Iterable[Row], *columns: str) -> float:
    peak = float("-inf")
    for row in rows:
        for column in columns:
            value = parse_float(row.get(column))
            if value == value:
                peak = max(peak, value)
                break
    return peak


def peak_altitude_m(rows: Iterable[Row]) -> float:
    peak_m = peak_value(rows, "altitude_m", "state_agl_m", "baro_agl_m")
    if peak_m != float("-inf"):
        return peak_m

    peak_ft = peak_value(rows, "altitude_ft", "altitude_feet", "state_agl_ft", "baro_agl_ft")
    if peak_ft != float("-inf"):
        return feet_to_meters(peak_ft)

    return float("nan")


__all__ = ["feet_to_meters", "meters_to_feet", "peak_altitude_m", "peak_value"]

