#!/usr/bin/env python3

"""Unified TUI dashboard for debugging and testing tools."""

from __future__ import annotations

import argparse
import csv
import curses
import itertools
import pathlib
import json
import os
import time
from typing import Iterable

import decode_sensor_log


ROOT_FALLBACK = pathlib.Path(".")
SETTINGS_PATH = pathlib.Path(__file__).resolve().parent / ".tui_settings.json"

DEFAULT_COLUMNS = [
    "state_time",
    "sensor_altitude_feet",
    "state_velocity_z",
    "state_apogee_estimate",
    "flight_status",
]

DEFAULT_EVENT_COLUMNS = [
    "timestamp",
    "flight_status",
    "event_type",
    "altitude_meters",
    "vertical_velocity",
    "apogee_estimate",
]

G_ACCEL = 9.80665

class CSVNavigator:
    def __init__(self, path: pathlib.Path) -> None:
        self.path = path
        self.scan_file = path.open("r", newline="", encoding="utf-8", errors="replace")
        self.read_file = path.open("r", newline="", encoding="utf-8", errors="replace")
        header_line = self.scan_file.readline()
        self.read_file.readline()
        if not header_line:
            self.header: list[str] = []
        else:
            self.header = next(csv.reader([header_line]))
        self.offsets: list[int] = []
        self.eof = False

    def close(self) -> None:
        self.scan_file.close()
        self.read_file.close()

    def ensure_index(self, index: int) -> None:
        while not self.eof and len(self.offsets) <= index:
            self._read_next_offset()

    def _read_next_offset(self) -> None:
        if self.eof:
            return
        pos = self.scan_file.tell()
        line = self.scan_file.readline()
        if line == "":
            self.eof = True
            return
        self.offsets.append(pos)

    def row_count(self) -> int | None:
        if self.eof:
            return len(self.offsets)
        return None

    def get_values(self, index: int, columns: list[str]) -> list[str]:
        self.ensure_index(index)
        if index >= len(self.offsets):
            return []
        self.read_file.seek(self.offsets[index])
        line = self.read_file.readline()
        values = next(csv.reader([line]), [])
        row_map = dict(zip(self.header, values))
        return [row_map.get(col, "") for col in columns]


def _pick_columns(fieldnames: list[str]) -> list[str]:
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


def _format_row(values: Iterable[str], widths: list[int]) -> str:
    chunks = []
    for value, width in zip(values, widths):
        text = value if value is not None else ""
        text = str(text)
        if len(text) > width:
            text = text[: max(1, width - 1)] + "~"
        chunks.append(text.ljust(width))
    return " | ".join(chunks)


def _select_file(
    stdscr: "curses._CursesWindow",
    title: str,
    start_dir: pathlib.Path,
    extensions: tuple[str, ...],
) -> pathlib.Path | None:
    stdscr.nodelay(False)
    stdscr.keypad(True)
    curses.curs_set(0)

    current_dir = start_dir if start_dir.exists() else ROOT_FALLBACK
    index = 0
    top = 0

    while True:
        stdscr.erase()
        height, width = stdscr.getmaxyx()
        _draw_header(stdscr, title, str(current_dir))

        entries = _list_directory(current_dir, extensions)
        body_height = max(0, height - 4)
        end = min(len(entries), top + body_height)
        for row, path in enumerate(entries[top:end], start=0):
            label = _format_entry_label(path, current_dir)
            attr = curses.A_REVERSE | curses.color_pair(2) if top + row == index else curses.color_pair(1)
            _safe_addstr(stdscr, 2 + row, 2, label[: width - 4], attr)

        footer = "Enter open/select  Backspace up  e enter path  q back"
        _safe_addstr(stdscr, height - 1, 0, footer[:width], curses.A_DIM | curses.color_pair(3))
        stdscr.refresh()

        key = stdscr.getch()
        if key in (ord("q"), ord("Q")):
            return None
        if key in (curses.KEY_DOWN, ord("j")):
            if index + 1 < len(entries):
                index += 1
            if index >= top + body_height:
                top += 1
        elif key in (curses.KEY_UP, ord("k")):
            if index > 0:
                index -= 1
            if index < top:
                top = max(0, top - 1)
        elif key == curses.KEY_NPAGE:
            index = min(len(entries) - 1, index + body_height)
            top = min(max(0, len(entries) - body_height), top + body_height)
        elif key == curses.KEY_PPAGE:
            index = max(0, index - body_height)
            top = max(0, top - body_height)
        elif key in (curses.KEY_ENTER, 10, 13):
            if entries:
                selection = entries[index]
                if selection.is_dir():
                    current_dir = selection
                    index = 0
                    top = 0
                else:
                    return selection
        elif key in (curses.KEY_BACKSPACE, 127, 8):
            parent = current_dir.parent
            if parent != current_dir:
                current_dir = parent
                index = 0
                top = 0
        elif key in (ord("e"), ord("E")):
            return _prompt_path(stdscr, "Enter path: ")


def _select_directory(
    stdscr: "curses._CursesWindow",
    title: str,
    start_dir: pathlib.Path,
) -> pathlib.Path | None:
    stdscr.nodelay(False)
    stdscr.keypad(True)
    curses.curs_set(0)

    current_dir = start_dir if start_dir.exists() else ROOT_FALLBACK
    index = 0
    top = 0

    while True:
        stdscr.erase()
        height, width = stdscr.getmaxyx()
        _draw_header(stdscr, title, str(current_dir))

        entries = [p for p in _list_directory(current_dir, ()) if p.is_dir()]
        body_height = max(0, height - 4)
        end = min(len(entries), top + body_height)
        for row, path in enumerate(entries[top:end], start=0):
            label = _format_entry_label(path, current_dir)
            attr = curses.A_REVERSE | curses.color_pair(2) if top + row == index else curses.color_pair(1)
            _safe_addstr(stdscr, 2 + row, 2, label[: width - 4], attr)

        footer = "Enter open/select  Backspace up  s select here  q back"
        _safe_addstr(stdscr, height - 1, 0, footer[:width], curses.A_DIM | curses.color_pair(3))
        stdscr.refresh()

        key = stdscr.getch()
        if key in (ord("q"), ord("Q")):
            return None
        if key in (curses.KEY_DOWN, ord("j")):
            if index + 1 < len(entries):
                index += 1
            if index >= top + body_height:
                top += 1
        elif key in (curses.KEY_UP, ord("k")):
            if index > 0:
                index -= 1
            if index < top:
                top = max(0, top - 1)
        elif key == curses.KEY_NPAGE:
            index = min(len(entries) - 1, index + body_height)
            top = min(max(0, len(entries) - body_height), top + body_height)
        elif key == curses.KEY_PPAGE:
            index = max(0, index - body_height)
            top = max(0, top - body_height)
        elif key in (curses.KEY_ENTER, 10, 13):
            if entries:
                selection = entries[index]
                current_dir = selection
                index = 0
                top = 0
        elif key in (curses.KEY_BACKSPACE, 127, 8):
            parent = current_dir.parent
            if parent != current_dir:
                current_dir = parent
                index = 0
                top = 0
        elif key in (ord("s"), ord("S")):
            return current_dir


def _prompt_path(stdscr: "curses._CursesWindow", prompt: str) -> pathlib.Path | None:
    stdscr.erase()
    height, width = stdscr.getmaxyx()
    header = f"{prompt}".strip()
    _safe_addstr(stdscr, 0, 0, header[:width], curses.A_BOLD | curses.color_pair(4))
    _safe_addstr(
        stdscr,
        1,
        0,
        "=" * max(0, width - 1),
        curses.A_DIM | curses.color_pair(3),
    )
    curses.echo()
    curses.curs_set(1)
    stdscr.refresh()
    value = stdscr.getstr(2, 0).decode("utf-8").strip()
    curses.noecho()
    curses.curs_set(0)
    if not value:
        return None
    return pathlib.Path(value)


def _prompt_float(
    stdscr: "curses._CursesWindow",
    prompt: str,
    default: float | None,
) -> float | None:
    stdscr.erase()
    height, width = stdscr.getmaxyx()
    header = prompt.strip()
    _safe_addstr(stdscr, 0, 0, header[:width], curses.A_BOLD | curses.color_pair(4))
    _safe_addstr(
        stdscr,
        1,
        0,
        "=" * max(0, width - 1),
        curses.A_DIM | curses.color_pair(3),
    )
    if default is not None:
        hint = f"(Enter to keep {default})"
        _safe_addstr(stdscr, 2, 0, hint[:width], curses.A_DIM | curses.color_pair(3))
    curses.echo()
    curses.curs_set(1)
    stdscr.refresh()
    value = stdscr.getstr(3, 0).decode("utf-8").strip()
    curses.noecho()
    curses.curs_set(0)
    if not value:
        return default
    try:
        return float(value)
    except ValueError:
        return default


def _telemetry_viewer(stdscr: "curses._CursesWindow", csv_path: pathlib.Path) -> None:
    navigator = CSVNavigator(csv_path)
    if not navigator.header:
        _status_screen(stdscr, "CSV has no header row.")
        navigator.close()
        return
    columns = _pick_columns(navigator.header)

    stdscr.nodelay(False)
    stdscr.keypad(True)
    curses.curs_set(0)

    top = 0

    while True:
        stdscr.erase()
        height, width = stdscr.getmaxyx()
        widths = _calc_widths(width, len(columns))
        header = _format_row(columns, widths)
        _safe_addstr(stdscr, 0, 0, header[:width], curses.A_REVERSE | curses.color_pair(2))

        body_height = max(0, height - 2)
        if body_height > 0:
            navigator.ensure_index(top + body_height - 1)
        for idx in range(body_height):
            row_index = top + idx
            navigator.ensure_index(row_index)
            if row_index >= len(navigator.offsets):
                break
            values = navigator.get_values(row_index, columns)
            line = _format_row(values, widths)
            _safe_addstr(stdscr, 1 + idx, 0, line[:width], curses.color_pair(1))

        shown_end = min(top + body_height, len(navigator.offsets))
        total_known = navigator.row_count()
        total_label = str(total_known) if total_known is not None else "?"
        footer = (
            f"Telemetry: {csv_path}  {top + 1}-{shown_end}/{total_label}  "
            "Up/Down PgUp/PgDn Home/End q back"
        )
        _safe_addstr(
            stdscr, height - 1, 0, footer[:width], curses.A_DIM | curses.color_pair(3)
        )
        stdscr.refresh()

        key = stdscr.getch()
        if key in (ord("q"), ord("Q")):
            navigator.close()
            break
        if key in (curses.KEY_DOWN, ord("j")):
            if not navigator.eof or top + 1 < len(navigator.offsets):
                top += 1
        elif key in (curses.KEY_UP, ord("k")):
            top = max(0, top - 1)
        elif key == curses.KEY_NPAGE:
            top = max(0, top + body_height)
        elif key == curses.KEY_PPAGE:
            top = max(0, top - body_height)
        elif key == curses.KEY_HOME:
            top = 0
        elif key == curses.KEY_END:
            if navigator.row_count() is not None:
                top = max(0, navigator.row_count() - body_height)
        if navigator.eof and navigator.row_count() is not None:
            top = min(top, max(0, navigator.row_count() - max(1, body_height)))


def _tail_viewer(stdscr: "curses._CursesWindow", csv_path: pathlib.Path) -> None:
    stdscr.nodelay(True)
    stdscr.keypad(True)
    curses.curs_set(0)

    spinner = itertools.cycle("-\\|/")
    rows_limit = 20
    interval = 0.5

    while True:
        height, width = stdscr.getmaxyx()
        fieldnames, rows = _read_tail_fast(csv_path, rows_limit)
        stdscr.erase()

        if not fieldnames:
            stdscr.addstr(0, 0, "CSV has no header row.", curses.A_BOLD | curses.color_pair(4))
            stdscr.refresh()
            time.sleep(interval)
            continue

        widths = _calc_widths(width, len(fieldnames))
        header = _format_row(fieldnames, widths)
        _safe_addstr(stdscr, 0, 0, header[:width], curses.A_REVERSE | curses.color_pair(2))

        body_height = max(0, height - 2)
        visible = rows[-min(body_height, len(rows)) :]
        for idx, row in enumerate(visible):
            values = [row.get(col, "") for col in fieldnames]
            line = _format_row(values, widths)
            _safe_addstr(stdscr, 1 + idx, 0, line[:width], curses.color_pair(1))

        spin = next(spinner)
        footer = (
            f"Tail: {csv_path}  last {rows_limit} rows  {spin} live  "
            "q back"
        )
        _safe_addstr(
            stdscr, height - 1, 0, footer[:width], curses.A_DIM | curses.color_pair(3)
        )
        stdscr.refresh()

        key = stdscr.getch()
        if key in (ord("q"), ord("Q")):
            break
        time.sleep(interval)


def _read_tail_fast(path: pathlib.Path, limit: int) -> tuple[list[str], list[dict[str, str]]]:
    try:
        with path.open("r", newline="", encoding="utf-8", errors="replace") as handle:
            header_line = handle.readline()
    except OSError:
        return [], []

    if not header_line:
        return [], []
    fieldnames = next(csv.reader([header_line]))

    lines = _tail_lines(path, limit + 1)
    decoded = [line.decode("utf-8", errors="replace") for line in lines]
    rows: list[dict[str, str]] = []
    for line in decoded:
        if not line.strip() or line.strip() == header_line.strip():
            continue
        values = next(csv.reader([line]), [])
        rows.append(dict(zip(fieldnames, values)))
    return fieldnames, rows[-limit:]


def _tail_lines(path: pathlib.Path, limit: int) -> list[bytes]:
    block_size = 4096
    data = b""
    lines: list[bytes] = []
    with path.open("rb") as handle:
        handle.seek(0, os.SEEK_END)
        end = handle.tell()
        while end > 0 and len(lines) <= limit:
            read_size = min(block_size, end)
            handle.seek(end - read_size)
            data = handle.read(read_size) + data
            lines = data.splitlines()
            end -= read_size
    return lines[-limit:]


def _event_viewer(stdscr: "curses._CursesWindow", events_path: pathlib.Path) -> None:
    with events_path.open("r", encoding="utf-8") as handle:
        data = handle.read()

    rows = _parse_events(data)
    columns = DEFAULT_EVENT_COLUMNS

    stdscr.nodelay(False)
    stdscr.keypad(True)
    curses.curs_set(0)

    top = 0
    total = len(rows)

    while True:
        stdscr.erase()
        height, width = stdscr.getmaxyx()
        widths = _calc_widths(width, len(columns))
        header = _format_row(columns, widths)
        _safe_addstr(stdscr, 0, 0, header[:width], curses.A_REVERSE | curses.color_pair(2))

        body_height = max(0, height - 2)
        for idx in range(body_height):
            row_index = top + idx
            if row_index >= total:
                break
            row = rows[row_index]
            values = [row.get(col, "") for col in columns]
            line = _format_row([str(v) for v in values], widths)
            _safe_addstr(stdscr, 1 + idx, 0, line[:width], curses.color_pair(1))

        footer = (
            f"Events: {events_path}  {top + 1}-{min(top + body_height, total)}/{total}  "
            "Up/Down PgUp/PgDn Home/End q back"
        )
        _safe_addstr(
            stdscr, height - 1, 0, footer[:width], curses.A_DIM | curses.color_pair(3)
        )
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


def _parse_events(data: str) -> list[dict[str, object]]:
    import json

    parsed = json.loads(data)
    if isinstance(parsed, list):
        return [row for row in parsed if isinstance(row, dict)]
    raise ValueError("Event file must contain a JSON list.")


def _status_screen(stdscr: "curses._CursesWindow", message: str) -> None:
    stdscr.erase()
    height, width = stdscr.getmaxyx()
    _safe_addstr(
        stdscr,
        0,
        0,
        "Status",
        curses.A_BOLD | curses.color_pair(4),
    )
    _safe_addstr(
        stdscr,
        1,
        0,
        "=" * max(0, width - 1),
        curses.A_DIM | curses.color_pair(3),
    )
    _safe_addstr(
        stdscr, height // 2, 0, message[:width], curses.A_BOLD | curses.color_pair(4)
    )
    _safe_addstr(
        stdscr,
        height - 1,
        0,
        "Press any key to continue"[:width],
        curses.A_DIM | curses.color_pair(3),
    )
    stdscr.refresh()
    stdscr.getch()


def _decode_log(stdscr: "curses._CursesWindow", bin_path: pathlib.Path) -> None:
    if not bin_path.exists():
        _status_screen(stdscr, f"File not found: {bin_path}")
        return

    csv_path = bin_path.with_suffix(".csv")
    stdscr.erase()
    _safe_addstr(
        stdscr,
        0,
        0,
        "Decoding log, please wait...",
        curses.A_BOLD | curses.color_pair(4),
    )
    stdscr.refresh()

    last_update = 0.0
    total_bytes = bin_path.stat().st_size

    def _progress(bytes_read: int | None, total: int | None) -> None:
        nonlocal last_update
        now = time.time()
        if now - last_update < 0.1 and bytes_read not in (None, total):
            return
        last_update = now
        _draw_progress(stdscr, "Decoding log...", bytes_read, total)

    try:
        decode_sensor_log.decode_with_progress(
            bin_path, csv_path, progress_callback=_progress
        )
    except Exception as exc:  # noqa: BLE001
        _status_screen(stdscr, f"Decode failed: {exc}")
        return

    _status_screen(stdscr, f"Decoded to {csv_path}")


def _menu(stdscr: "curses._CursesWindow") -> None:
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)
    _init_colors()

    options = [
        "View telemetry CSV",
        "Tail telemetry CSV",
        "View event JSON",
        "Decode binary log",
        "Rocketry analysis",
        "Settings",
        "Quit",
    ]
    index = 0
    spinner = itertools.cycle(".oO@O")
    frame = 0
    settings = _load_settings()
    data_root = _get_data_root(settings)
    if data_root is None:
        data_root = _configure_data_root(stdscr, settings)
        settings["data_root"] = str(data_root)
        _save_settings(settings)

    while True:
        stdscr.erase()
        height, width = stdscr.getmaxyx()
        title = f"Rocketry Data {next(spinner)}"
        _draw_menu_banner(stdscr, title, frame)
        menu_top = 6
        _safe_addstr(
            stdscr,
            menu_top - 3,
            0,
            "=" * max(0, width - 1),
            curses.A_DIM | curses.color_pair(3),
        )
        _safe_addstr(
            stdscr,
            menu_top - 2,
            0,
            f"Data root: {data_root}"[:width],
            curses.A_DIM | curses.color_pair(3),
        )
        _safe_addstr(
            stdscr,
            menu_top - 1,
            0,
            "=" * max(0, width - 1),
            curses.A_DIM | curses.color_pair(3),
        )

        for row, label in enumerate(options, start=0):
            attr = curses.A_REVERSE | curses.color_pair(2) if row == index else curses.color_pair(1)
            _safe_addstr(stdscr, menu_top + row, 4, label[: width - 6], attr)

        _safe_addstr(
            stdscr,
            height - 2,
            0,
            "=" * max(0, width - 1),
            curses.A_DIM | curses.color_pair(3),
        )
        footer = "Enter select  q quit"
        _safe_addstr(
            stdscr, height - 1, 0, footer[:width], curses.A_DIM | curses.color_pair(3)
        )
        stdscr.refresh()

        key = stdscr.getch()
        if key in (ord("q"), ord("Q")):
            break
        if key in (curses.KEY_DOWN, ord("j")):
            index = (index + 1) % len(options)
        elif key in (curses.KEY_UP, ord("k")):
            index = (index - 1) % len(options)
        elif key in (curses.KEY_ENTER, 10, 13):
            selection = options[index]
            if selection == "View telemetry CSV":
                _run_csv_viewer(stdscr, data_root)
            elif selection == "Tail telemetry CSV":
                _run_tail_viewer(stdscr, data_root)
            elif selection == "View event JSON":
                _run_event_viewer(stdscr, data_root)
            elif selection == "Decode binary log":
                _run_decoder(stdscr, data_root)
            elif selection == "Rocketry analysis":
                _run_analysis(stdscr, data_root, settings)
            elif selection == "Settings":
                data_root = _settings_menu(stdscr, settings, data_root)
            elif selection == "Quit":
                break
        frame += 1
        time.sleep(0.08)


def _run_csv_viewer(stdscr: "curses._CursesWindow", data_root: pathlib.Path) -> None:
    path = _select_file(stdscr, "Select telemetry CSV", data_root, (".csv",))
    if path is None:
        return
    _telemetry_viewer(stdscr, path)


def _run_tail_viewer(stdscr: "curses._CursesWindow", data_root: pathlib.Path) -> None:
    path = _select_file(stdscr, "Select CSV to tail", data_root, (".csv",))
    if path is None:
        return
    _tail_viewer(stdscr, path)


def _run_event_viewer(stdscr: "curses._CursesWindow", data_root: pathlib.Path) -> None:
    path = _select_file(stdscr, "Select event JSON", data_root, (".json",))
    if path is None:
        return
    _event_viewer(stdscr, path)


def _run_decoder(stdscr: "curses._CursesWindow", data_root: pathlib.Path) -> None:
    path = _select_file(
        stdscr,
        "Select binary log (.BIN)",
        data_root,
        (".bin", ".BIN"),
    )
    if path is None:
        return
    _decode_log(stdscr, path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    curses.wrapper(_menu)
    return 0


def _init_colors() -> None:
    if not curses.has_colors():
        return
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_WHITE, -1)
    curses.init_pair(2, curses.COLOR_BLACK, curses.COLOR_CYAN)
    curses.init_pair(3, curses.COLOR_YELLOW, -1)
    curses.init_pair(4, curses.COLOR_BLUE, -1)
    curses.init_pair(5, curses.COLOR_YELLOW, -1)


def _draw_header(
    stdscr: "curses._CursesWindow", title: str, subtitle: str
) -> None:
    height, width = stdscr.getmaxyx()
    _safe_addstr(stdscr, 0, 0, title[:width], curses.A_BOLD | curses.color_pair(4))
    _safe_addstr(stdscr, 1, 0, subtitle[:width], curses.A_DIM | curses.color_pair(3))


def _list_directory(
    directory: pathlib.Path, extensions: tuple[str, ...]
) -> list[pathlib.Path]:
    entries: list[pathlib.Path] = []
    try:
        for entry in directory.iterdir():
            if entry.is_dir():
                entries.append(entry)
            elif entry.suffix.lower() in extensions:
                entries.append(entry)
    except OSError:
        return []
    return sorted(entries, key=lambda p: (not p.is_dir(), p.name.lower()))


def _format_entry_label(path: pathlib.Path, base: pathlib.Path) -> str:
    name = path.name
    if path.is_dir():
        return f"[DIR] {name}/"
    try:
        relative = path.relative_to(base)
    except ValueError:
        relative = path
    return f"      {relative}"


def _safe_addstr(
    stdscr: "curses._CursesWindow", y: int, x: int, text: str, attr: int
) -> None:
    if y < 0:
        return
    height, width = stdscr.getmaxyx()
    if y >= height or x >= width or width <= 0:
        return
    try:
        stdscr.addstr(y, x, text[: max(0, width - x)], attr)
    except curses.error:
        return


def _load_settings() -> dict:
    if SETTINGS_PATH.exists():
        try:
            return json.loads(SETTINGS_PATH.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, OSError):
            return {}
    return {}


def _save_settings(settings: dict) -> None:
    try:
        SETTINGS_PATH.write_text(json.dumps(settings, indent=2), encoding="utf-8")
    except OSError:
        return


def _get_data_root(settings: dict) -> pathlib.Path | None:
    value = settings.get("data_root")
    if not value:
        return None
    path = pathlib.Path(value)
    return path if path.exists() else None


def _configure_data_root(stdscr: "curses._CursesWindow", settings: dict) -> pathlib.Path:
    start_dir = pathlib.Path.home()
    chosen = _select_directory(stdscr, "Select data root directory", start_dir)
    if chosen is None:
        chosen = pathlib.Path.cwd()
    settings["data_root"] = str(chosen)
    _save_settings(settings)
    return chosen


def _settings_menu(
    stdscr: "curses._CursesWindow",
    settings: dict,
    data_root: pathlib.Path,
) -> pathlib.Path:
    options = [
        "Set data root",
        "Set analysis constants",
        "Clear data root",
        "Back",
    ]
    index = 0
    while True:
        stdscr.erase()
        height, width = stdscr.getmaxyx()
        _draw_header(stdscr, "Settings", f"Current data root: {data_root}")
        for row, label in enumerate(options, start=0):
            attr = curses.A_REVERSE | curses.color_pair(2) if row == index else curses.color_pair(1)
            _safe_addstr(stdscr, 3 + row, 4, label[: width - 6], attr)
        footer = "Enter select  q back"
        _safe_addstr(stdscr, height - 1, 0, footer[:width], curses.A_DIM | curses.color_pair(3))
        stdscr.refresh()

        key = stdscr.getch()
        if key in (ord("q"), ord("Q")):
            return data_root
        if key in (curses.KEY_DOWN, ord("j")):
            index = (index + 1) % len(options)
        elif key in (curses.KEY_UP, ord("k")):
            index = (index - 1) % len(options)
        elif key in (curses.KEY_ENTER, 10, 13):
            selection = options[index]
            if selection == "Set data root":
                chosen = _select_directory(stdscr, "Select data root directory", data_root)
                if chosen is not None:
                    data_root = chosen
                    settings["data_root"] = str(data_root)
                    _save_settings(settings)
            elif selection == "Set analysis constants":
                _configure_analysis_constants(stdscr, settings)
            elif selection == "Clear data root":
                settings.pop("data_root", None)
                _save_settings(settings)
                data_root = pathlib.Path.cwd()
            elif selection == "Back":
                return data_root


def _draw_menu_banner(stdscr: "curses._CursesWindow", title: str, frame: int) -> None:
    height, width = stdscr.getmaxyx()
    banner = [
        "▗▄▄▖  ▗▄▖  ▗▄▄▖▗▖ ▗▖▗▄▄▄▖▗▄▄▄▖▗▄▄▖▗▖  ▗▖    ▗▄▄▄  ▗▄▖▗▄▄▄▖▗▄▖ ",
        "▐▌ ▐▌▐▌ ▐▌▐▌   ▐▌▗▞▘▐▌     █  ▐▌ ▐▌▝▚▞▘     ▐▌  █▐▌ ▐▌ █ ▐▌ ▐▌",
        "▐▛▀▚▖▐▌ ▐▌▐▌   ▐▛▚▖ ▐▛▀▀▘  █  ▐▛▀▚▖ ▐▌      ▐▌  █▐▛▀▜▌ █ ▐▛▀▜▌",
        "▐▌ ▐▌▝▚▄▞▘▝▚▄▄▖▐▌ ▐▌▐▙▄▄▖  █  ▐▌ ▐▌ ▐▌      ▐▙▄▄▀▐▌ ▐▌ █ ▐▌ ▐▌",
    ]
    start_row = 1
    for i, line in enumerate(banner):
        color = curses.color_pair(4) if (i + frame) % 2 else curses.color_pair(5)
        _safe_addstr(stdscr, start_row + i, 0, line[:width], curses.A_BOLD | color)
    pulse = "<>" if (frame // 4) % 2 == 0 else "><"
    label = f"{title} {pulse}"
    _safe_addstr(stdscr, len(banner), 0, label[:width], curses.A_DIM | curses.color_pair(3))


def _configure_analysis_constants(stdscr: "curses._CursesWindow", settings: dict) -> None:
    mass_kg = _prompt_float(
        stdscr,
        "Enter vehicle mass (kg)",
        _parse_setting_float(settings.get("mass_kg")),
    )
    ref_area = _prompt_float(
        stdscr,
        "Enter reference area (m^2)",
        _parse_setting_float(settings.get("ref_area_m2")),
    )
    air_density = _prompt_float(
        stdscr,
        "Enter air density (kg/m^3)",
        _parse_setting_float(settings.get("air_density_kg_m3", 1.225)),
    )
    if mass_kg is not None:
        settings["mass_kg"] = mass_kg
    if ref_area is not None:
        settings["ref_area_m2"] = ref_area
    if air_density is not None:
        settings["air_density_kg_m3"] = air_density
    _save_settings(settings)


def _parse_setting_float(value) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _run_analysis(
    stdscr: "curses._CursesWindow",
    data_root: pathlib.Path,
    settings: dict,
) -> None:
    path = _select_file(stdscr, "Select telemetry CSV for analysis", data_root, (".csv",))
    if path is None:
        return
    metrics, notes = _compute_metrics(path, settings)
    _analysis_screen(stdscr, metrics, notes)


def _analysis_screen(
    stdscr: "curses._CursesWindow",
    metrics: list[str],
    notes: list[str],
) -> None:
    stdscr.nodelay(False)
    stdscr.keypad(True)
    curses.curs_set(0)

    lines = metrics + ["", "Assumptions"] + notes
    top = 0

    while True:
        stdscr.erase()
        height, width = stdscr.getmaxyx()
        _draw_header(stdscr, "Rocketry Analysis", "=====" * 8)

        body_height = max(0, height - 4)
        end = min(len(lines), top + body_height)
        for idx, line in enumerate(lines[top:end]):
            _safe_addstr(stdscr, 2 + idx, 0, line[:width], curses.color_pair(1))

        footer = "Up/Down PgUp/PgDn Home/End q back"
        _safe_addstr(stdscr, height - 1, 0, footer[:width], curses.A_DIM | curses.color_pair(3))
        stdscr.refresh()

        key = stdscr.getch()
        if key in (ord("q"), ord("Q")):
            break
        if key in (curses.KEY_DOWN, ord("j")):
            if top + 1 < len(lines):
                top += 1
        elif key in (curses.KEY_UP, ord("k")):
            top = max(0, top - 1)
        elif key == curses.KEY_NPAGE:
            top = min(max(0, len(lines) - 1), top + body_height)
        elif key == curses.KEY_PPAGE:
            top = max(0, top - body_height)
        elif key == curses.KEY_HOME:
            top = 0
        elif key == curses.KEY_END:
            top = max(0, len(lines) - body_height)


def _compute_metrics(
    csv_path: pathlib.Path,
    settings: dict,
) -> tuple[list[str], list[str]]:
    mass_kg = _parse_setting_float(settings.get("mass_kg"))
    ref_area = _parse_setting_float(settings.get("ref_area_m2"))
    air_density = _parse_setting_float(settings.get("air_density_kg_m3", 1.225)) or 1.225

    max_alt = None
    max_alt_time = None
    max_vel = None
    max_accel = None
    max_accel_net = None
    max_apogee_est = None
    max_tw = None
    avg_tw_sum = 0.0
    avg_tw_count = 0

    start_time = None
    end_time = None
    last_time = None
    last_status = None
    status_durations: dict[str, float] = {}

    burn_start = None
    burn_end = None
    coast_start = None
    coast_end = None
    descent_start = None

    drag_samples = []
    cd_samples = []

    with csv_path.open("r", newline="", encoding="utf-8", errors="replace") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            time_val = _parse_float(row.get("state_time")) or _parse_float(
                row.get("sensor_timestamp")
            )
            if time_val is None:
                continue
            if start_time is None:
                start_time = time_val
            end_time = time_val

            status = (row.get("flight_status") or "").strip()
            if last_time is not None and last_status is not None:
                dt = time_val - last_time
                if dt >= 0:
                    status_durations[last_status] = status_durations.get(last_status, 0.0) + dt
            if status and status != last_status:
                if status == "burn" and burn_start is None:
                    burn_start = time_val
                if last_status == "burn" and burn_end is None:
                    burn_end = time_val
                if status == "coast" and coast_start is None:
                    coast_start = time_val
                if last_status == "coast" and coast_end is None:
                    coast_end = time_val
                if status == "descent" and descent_start is None:
                    descent_start = time_val
            last_status = status or last_status
            last_time = time_val

            alt_m = _parse_float(row.get("state_position_z"))
            if alt_m is None:
                alt_ft = _parse_float(row.get("sensor_altitude_feet"))
                if alt_ft is not None:
                    alt_m = alt_ft * 0.3048

            vel_z = _parse_float(row.get("state_velocity_z"))
            accel_z = _parse_float(row.get("state_acceleration_z"))
            inertial_accel_z = _parse_float(row.get("state_inertial_acceleration_z"))
            accel_net = inertial_accel_z if inertial_accel_z is not None else accel_z

            if alt_m is not None and (max_alt is None or alt_m > max_alt):
                max_alt = alt_m
                max_alt_time = time_val
            if vel_z is not None:
                if max_vel is None or abs(vel_z) > abs(max_vel):
                    max_vel = vel_z
            if accel_z is not None:
                if max_accel is None or abs(accel_z) > abs(max_accel):
                    max_accel = accel_z
            if accel_net is not None:
                if max_accel_net is None or abs(accel_net) > abs(max_accel_net):
                    max_accel_net = accel_net

            apogee_est = _parse_float(row.get("state_apogee_estimate"))
            if apogee_est is not None:
                if max_apogee_est is None or apogee_est > max_apogee_est:
                    max_apogee_est = apogee_est

            if mass_kg is not None and accel_net is not None and status == "burn":
                tw = (accel_net / G_ACCEL) + 1.0
                if max_tw is None or tw > max_tw:
                    max_tw = tw
                avg_tw_sum += tw
                avg_tw_count += 1

            if mass_kg is not None and status == "coast" and accel_net is not None and vel_z:
                drag = mass_kg * (-(accel_net) - G_ACCEL)
                drag_samples.append(drag)
                if ref_area is not None and air_density is not None and vel_z != 0.0:
                    cd = (2.0 * drag) / (air_density * (vel_z ** 2) * ref_area)
                    cd_samples.append(cd)

    metrics = ["Rocketry Analysis", "=====" * 8]
    if start_time is not None and end_time is not None:
        metrics.append(f"Flight time: {end_time - start_time:.2f} s")
    if max_alt is not None:
        metrics.append(f"Apogee (max altitude): {max_alt:.2f} m at t={max_alt_time:.2f}s")
    if max_apogee_est is not None:
        metrics.append(f"Max apogee estimate: {max_apogee_est:.2f} m")
    if max_vel is not None:
        metrics.append(f"Max |velocity_z|: {max_vel:.2f} m/s")
    if max_accel is not None:
        metrics.append(f"Max |accel_z|: {max_accel:.2f} m/s^2")
    if max_accel_net is not None:
        metrics.append(f"Max |net accel_z|: {max_accel_net:.2f} m/s^2")

    if burn_start is not None and burn_end is not None:
        metrics.append(f"Burn duration: {burn_end - burn_start:.2f} s")
    if coast_start is not None and coast_end is not None:
        metrics.append(f"Coast duration: {coast_end - coast_start:.2f} s")
    if descent_start is not None and end_time is not None:
        metrics.append(f"Descent duration: {end_time - descent_start:.2f} s")

    if status_durations:
        metrics.append("Phase durations:")
        metrics.append("=====" * 4)
        for key, val in status_durations.items():
            if key:
                metrics.append(f"  {key}: {val:.2f} s")

    if mass_kg is not None:
        metrics.append(f"Mass used: {mass_kg:.2f} kg")
    if ref_area is not None:
        metrics.append(f"Reference area used: {ref_area:.4f} m^2")

    if max_tw is not None and avg_tw_count:
        metrics.append(f"Peak T/W (est): {max_tw:.2f}")
        metrics.append(f"Avg T/W (est): {avg_tw_sum / avg_tw_count:.2f}")

    if drag_samples:
        avg_drag = sum(drag_samples) / len(drag_samples)
        metrics.append(f"Avg coast drag (est): {avg_drag:.2f} N")
    if cd_samples:
        avg_cd = sum(cd_samples) / len(cd_samples)
        metrics.append(f"Avg Cd (est): {avg_cd:.3f}")

    notes = [
        "state_position_z assumed meters; sensor_altitude_feet used when missing.",
        "state_inertial_acceleration_z used as net accel when present.",
        "T/W estimate assumes accel is gravity-corrected (net).",
        "Cd estimate uses constant air density and coast samples only.",
    ]
    if mass_kg is None:
        notes.append("Set mass_kg in Settings for thrust/drag estimates.")
    if ref_area is None:
        notes.append("Set ref_area_m2 in Settings for Cd estimates.")

    return metrics, notes


def _parse_float(value: str | None) -> float | None:
    if value is None:
        return None
    value = value.strip()
    if not value:
        return None
    try:
        return float(value)
    except ValueError:
        return None


def _draw_progress(
    stdscr: "curses._CursesWindow",
    label: str,
    bytes_read: int | None,
    total: int | None,
) -> None:
    height, width = stdscr.getmaxyx()
    _safe_addstr(stdscr, 1, 0, label[:width], curses.A_BOLD | curses.color_pair(4))

    if total is None or bytes_read is None or total <= 0:
        line = "Progress: unknown"
        _safe_addstr(stdscr, 2, 0, line[:width], curses.color_pair(3))
        stdscr.refresh()
        return

    ratio = min(max(bytes_read / total, 0.0), 1.0)
    percent = int(ratio * 100)
    bar_width = max(10, width - 18)
    filled = int(bar_width * ratio)
    bar = "[" + "=" * filled + " " * (bar_width - filled) + "]"
    detail = f"{_format_bytes(bytes_read)}/{_format_bytes(total)}"
    line = f"{bar} {percent:3d}% {detail}"
    _safe_addstr(stdscr, 2, 0, line[:width], curses.color_pair(3))
    stdscr.refresh()


def _format_bytes(value: int) -> str:
    units = ["B", "KB", "MB", "GB", "TB"]
    size = float(value)
    for unit in units:
        if size < 1024.0 or unit == units[-1]:
            return f"{size:.1f}{unit}"
        size /= 1024.0


if __name__ == "__main__":
    raise SystemExit(main())
