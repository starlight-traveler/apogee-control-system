#!/usr/bin/env python3

"""TUI viewer for telemetry CSV files."""

from __future__ import annotations

import argparse
import csv
import pathlib
import curses


DEFAULT_COLUMNS = [
    "state_time",
    "sensor_altitude_feet",
    "state_velocity_z",
    "state_apogee_estimate",
    "flight_status",
]


def _read_csv(path: pathlib.Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
        fieldnames = reader.fieldnames or []
    return fieldnames, rows


def _pick_columns(fieldnames: list[str], requested: str | None) -> list[str]:
    if requested:
        return [name.strip() for name in requested.split(",") if name.strip()]

    columns = [name for name in DEFAULT_COLUMNS if name in fieldnames]
    if columns:
        return columns

    return fieldnames[: min(5, len(fieldnames))]


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


def _run_tui(stdscr: "curses._CursesWindow", rows: list[dict[str, str]], columns: list[str],
             title: str) -> None:
    curses.curs_set(0)
    stdscr.nodelay(False)
    stdscr.keypad(True)

    top = 0
    total = len(rows)

    while True:
        stdscr.erase()
        height, width = stdscr.getmaxyx()
        widths = _calc_widths(width, len(columns))
        header = _format_row(columns, widths)
        stdscr.addstr(0, 0, header[:width], curses.A_REVERSE)

        body_height = max(0, height - 2)
        for idx in range(body_height):
            row_index = top + idx
            if row_index >= total:
                break
            row = rows[row_index]
            values = [row.get(col, "") for col in columns]
            line = _format_row(values, widths)
            stdscr.addstr(1 + idx, 0, line[:width])

        if total == 0:
            footer = f"{title}  empty file  q quit"
        else:
            footer = (
                f"{title}  {top + 1}-{min(top + body_height, total)}/{total}  "
                "Up/Down PgUp/PgDn Home/End q quit"
            )
        stdscr.addstr(height - 1, 0, footer[:width], curses.A_DIM)
        stdscr.refresh()

        key = stdscr.getch()
        if key in (ord("q"), ord("Q")):
            break
        if key in (curses.KEY_DOWN, ord("j")):
            if top + 1 < total:
                top += 1
        elif key in (curses.KEY_UP, ord("k")):
            top = max(0, top - 1)
        elif key == curses.KEY_NPAGE:
            top = min(max(0, total - 1), top + body_height)
        elif key == curses.KEY_PPAGE:
            top = max(0, top - body_height)
        elif key == curses.KEY_HOME:
            top = 0
        elif key == curses.KEY_END:
            top = max(0, total - body_height)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", type=pathlib.Path, help="Telemetry CSV to view")
    parser.add_argument(
        "--columns",
        help="Comma-separated list of columns to show",
        default=None,
    )
    args = parser.parse_args()

    fieldnames, rows = _read_csv(args.csv_path)
    if not fieldnames:
        raise SystemExit("CSV has no header row.")

    columns = _pick_columns(fieldnames, args.columns)
    title = f"Telemetry: {args.csv_path}"
    curses.wrapper(_run_tui, rows, columns, title)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
