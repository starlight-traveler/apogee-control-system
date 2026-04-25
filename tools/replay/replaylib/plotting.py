"""Shared plotting and SVG helpers."""

from __future__ import annotations

import re


def sanitize_filename(name: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", name.strip())
    return cleaned.strip("._") or "plot"


def feet_to_meters(value_ft: float) -> float:
    return value_ft * 0.3048


def meters_to_feet(value_m: float) -> float:
    return value_m / 0.3048

