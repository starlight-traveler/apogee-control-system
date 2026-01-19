#!/usr/bin/env python3

"""Live tail viewer for CSV telemetry files."""

from __future__ import annotations

import argparse
import csv
import pathlib
import time
from collections import deque
import curses


def _read_tail(path: pathlib.Path, limit: int) -> tuple[list[str], list[dict[str, str]]]:
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fieldnames = reader.fieldnames or []
        rows: deque[dict[str, str]] = deque(maxlen=limit)
        for row in reader:
            rows.append(row)
    return fieldnames, list(rows)


def _calc_widths(total_width: int, column_count: int) -> list[int]:
    if column_count <= 0:
        return []
    spacing = 3 * (column_count - 1)
    available = max(1, total_width - spacing)
    base = max(4, available // column_count)
    widths = [base] * column_count
    widths[-1] = max(4, available - base * (column_count - 1))
    return widths


def _format_row(values: list[str], widths: list[int]) -> str:
    chunks = []
    for value, width in zip(values, widths):
        text = value if value is not None else ""
        text = str(text)
        if len(text) > width:
            text = text[: max(1, width - 1)] + "~"
        chunks.append(text.ljust(width))
    return " | ".join(chunks)


def _run_tui(stdscr: "curses._CursesWindow", path: pathlib.Path, limit: int, interval: float) -> None:
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)

    while True:
        height, width = stdscr.getmaxyx()
        fieldnames, rows = _read_tail(path, limit)
        stdscr.erase()

        if not fieldnames:
            stdscr.addstr(0, 0, "CSV has no header row.", curses.A_BOLD)
            stdscr.refresh()
            time.sleep(interval)
            continue

        widths = _calc_widths(width, len(fieldnames))
        header = _format_row(fieldnames, widths)
        stdscr.addstr(0, 0, header[:width], curses.A_REVERSE)

        body_height = max(0, height - 2)
        for idx in range(min(body_height, len(rows))):
            row = rows[-min(body_height, len(rows)) + idx]
            values = [row.get(col, "") for col in fieldnames]
            line = _format_row(values, widths)
            stdscr.addstr(1 + idx, 0, line[:width])

        footer = (
            f"Tail: {path}  showing last {limit} rows  "
            "q quit"
        )
        stdscr.addstr(height - 1, 0, footer[:width], curses.A_DIM)
        stdscr.refresh()

        key = stdscr.getch()
        if key in (ord("q"), ord("Q")):
            break
        time.sleep(interval)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", type=pathlib.Path, help="CSV file to tail")
    parser.add_argument("--rows", type=int, default=20, help="Number of rows to show")
    parser.add_argument(
        "--interval",
        type=float,
        default=0.5,
        help="Refresh interval in seconds",
    )
    args = parser.parse_args()
    curses.wrapper(_run_tui, args.csv_path, args.rows, args.interval)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
