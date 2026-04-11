#!/usr/bin/env python3

"""Decode binary sensor log files produced by ``data_logger.cpp``."""

from __future__ import annotations

import argparse
import csv
import json
import pathlib
import struct
import sys
from contextlib import ExitStack
from typing import BinaryIO, Iterable, Iterator, Tuple


_RECORD_HEADER_STRUCT = struct.Struct("<BBBB")
_LOG_PREAMBLE_STRUCT = struct.Struct("<8sHH40s12s")
_LOG_MAGIC = b"ACSNDRT1"
_TELEMETRY_STRUCT = struct.Struct("<58f16B")
_EVENT_PAYLOAD_STRUCT = struct.Struct("<6f")

_FLIGHT_STATUS_NAMES = {
    0: "ground",
    1: "burn",
    2: "coast",
    3: "overshoot",
    4: "descent",
}

_EVENT_TYPE_NAMES = {
    0: "stage_change",
    1: "flap_actuated",
    2: "flap_settling_timer_fired",
}

_TELEMETRY_FLOAT_FIELD_NAMES = [
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
    "sensor_icm_quat_w",
    "sensor_icm_quat_x",
    "sensor_icm_quat_y",
    "sensor_icm_quat_z",
    "sensor_accel_lsm_x",
    "sensor_accel_lsm_y",
    "sensor_accel_lsm_z",
    "sensor_gyro_lsm_x",
    "sensor_gyro_lsm_y",
    "sensor_gyro_lsm_z",
    "sensor_lsm_quat_w",
    "sensor_lsm_quat_x",
    "sensor_lsm_quat_y",
    "sensor_lsm_quat_z",
    "sensor_flap_command_deg",
    "sensor_flap_effective_deg",
    "state_altitude_agl_feet",
    "state_vertical_velocity_fps",
    "state_zenith_deg",
    "state_apogee_estimate_feet",
    "sensor_accel_bno_x",
    "sensor_accel_bno_y",
    "sensor_accel_bno_z",
    "sensor_gyro_bno_x",
    "sensor_gyro_bno_y",
    "sensor_gyro_bno_z",
    "sensor_bno_quat_w",
    "sensor_bno_quat_x",
    "sensor_bno_quat_y",
    "sensor_bno_quat_z",
    "sensor_bno_yaw_deg",
    "sensor_bno_pitch_deg",
    "sensor_bno_roll_deg",
    "sensor_accel_wt901_x",
    "sensor_accel_wt901_y",
    "sensor_accel_wt901_z",
    "sensor_wt901_yaw_deg",
    "sensor_wt901_pitch_deg",
    "sensor_wt901_roll_deg",
    "sensor_gyro_wt901_x",
    "sensor_gyro_wt901_y",
    "sensor_gyro_wt901_z",
    "sensor_wt901_quat_w",
    "sensor_wt901_quat_x",
    "sensor_wt901_quat_y",
    "sensor_wt901_quat_z",
]

_TELEMETRY_U8_FIELD_NAMES = [
    "sensor_main_quaternion_source",
]

_TELEMETRY_BOOL_FIELD_NAMES = [
    "sensor_has_quaternion",
    "sensor_has_icm_quaternion",
    "sensor_has_lsm_quaternion",
    "sensor_icm_accel_saturated",
    "sensor_icm_gyro_saturated",
    "sensor_actuation_is_settling",
    "sensor_has_bno_quaternion",
    "sensor_has_bno_ypr",
    "sensor_has_wt901_accel",
    "sensor_has_wt901_ypr",
    "sensor_has_wt901_gyro",
    "sensor_has_wt901_quaternion",
]

_TELEMETRY_FIELD_NAMES = [
    "flight_status",
    "flight_status_raw",
    "has_filtered_state",
    *_TELEMETRY_FLOAT_FIELD_NAMES,
    *_TELEMETRY_U8_FIELD_NAMES,
    *_TELEMETRY_BOOL_FIELD_NAMES,
]


def _flight_status_name(value: int) -> str:
    return _FLIGHT_STATUS_NAMES.get(value, f"unknown_{value}")


def _event_type_name(value: int) -> str:
    return _EVENT_TYPE_NAMES.get(value, f"unknown_{value}")


def _iter_records(stream: BinaryIO) -> Iterator[Tuple[str, dict]]:
    offset = 0

    preamble = stream.read(_LOG_PREAMBLE_STRUCT.size)
    if len(preamble) == _LOG_PREAMBLE_STRUCT.size:
        magic, _format_version, _schema_version, _firmware_hash, _reserved = _LOG_PREAMBLE_STRUCT.unpack(preamble)
        if magic == _LOG_MAGIC:
            offset += _LOG_PREAMBLE_STRUCT.size
        else:
            stream.seek(0)
    else:
        stream.seek(0)

    while True:
        record_offset = offset
        header_data = stream.read(_RECORD_HEADER_STRUCT.size)
        if not header_data:
            break
        if len(header_data) != _RECORD_HEADER_STRUCT.size:
            raise ValueError(
                f"Encountered truncated record header at byte offset {record_offset}."
            )

        record_type, subtype, flags, _reserved = _RECORD_HEADER_STRUCT.unpack(header_data)
        offset += _RECORD_HEADER_STRUCT.size

        if record_type == 0:
            payload = stream.read(_TELEMETRY_STRUCT.size)
            if len(payload) != _TELEMETRY_STRUCT.size:
                raise ValueError(
                    f"Encountered truncated telemetry record at byte offset {record_offset}."
                )
            offset += _TELEMETRY_STRUCT.size
            yield (
                "telemetry",
                {
                    "status": subtype,
                    "has_filtered_state": bool(flags & 0x01),
                    "values": _TELEMETRY_STRUCT.unpack(payload),
                },
            )
        elif record_type == 1:
            payload = stream.read(_EVENT_PAYLOAD_STRUCT.size)
            if len(payload) != _EVENT_PAYLOAD_STRUCT.size:
                raise ValueError(
                    f"Encountered truncated event record at byte offset {record_offset}."
                )

            (
                timestamp,
                altitude_agl_feet,
                vertical_velocity_fps,
                apogee_estimate_feet,
                flap_command_deg,
                flap_effective_deg,
            ) = _EVENT_PAYLOAD_STRUCT.unpack(payload)
            offset += _EVENT_PAYLOAD_STRUCT.size

            yield (
                "event",
                {
                    "event_type": subtype,
                    "flight_status": flags,
                    "timestamp": timestamp,
                    "altitude_agl_feet": altitude_agl_feet,
                    "vertical_velocity_fps": vertical_velocity_fps,
                    "apogee_estimate_feet": apogee_estimate_feet,
                    "flap_command_deg": flap_command_deg,
                    "flap_effective_deg": flap_effective_deg,
                },
            )
        else:
            raise ValueError(
                f"Unknown record type {record_type} at byte offset {record_offset}."
            )


def _build_telemetry_row(data: dict) -> dict:
    row: dict[str, object] = {
        "flight_status": _flight_status_name(data["status"]),
        "flight_status_raw": data["status"],
        "has_filtered_state": data["has_filtered_state"],
    }

    values = data["values"]
    float_count = len(_TELEMETRY_FLOAT_FIELD_NAMES)
    u8_count = len(_TELEMETRY_U8_FIELD_NAMES)
    bool_count = len(_TELEMETRY_BOOL_FIELD_NAMES)

    float_values = values[:float_count]
    u8_values = values[float_count : float_count + u8_count]
    bool_values = values[float_count + u8_count : float_count + u8_count + bool_count]

    for name, value in zip(_TELEMETRY_FLOAT_FIELD_NAMES, float_values):
        row[name] = value
    for name, value in zip(_TELEMETRY_U8_FIELD_NAMES, u8_values):
        row[name] = value
    for name, value in zip(_TELEMETRY_BOOL_FIELD_NAMES, bool_values):
        row[name] = bool(value)

    return row


def _build_event_object(data: dict) -> dict:
    return {
        "event_type": _event_type_name(data["event_type"]),
        "event_type_raw": data["event_type"],
        "flight_status": _flight_status_name(data["flight_status"]),
        "flight_status_raw": data["flight_status"],
        "timestamp": data["timestamp"],
        "altitude_agl_feet": data["altitude_agl_feet"],
        "vertical_velocity_fps": data["vertical_velocity_fps"],
        "apogee_estimate_feet": data["apogee_estimate_feet"],
        "flap_command_deg": data["flap_command_deg"],
        "flap_effective_deg": data["flap_effective_deg"],
    }


class _ProgressStream:
    def __init__(self, raw: BinaryIO) -> None:
        self.raw = raw
        self.bytes_read = 0

    def read(self, size: int = -1) -> bytes:
        data = self.raw.read(size)
        self.bytes_read += len(data)
        return data

    def __getattr__(self, name: str):
        return getattr(self.raw, name)


def _decode_stream(
    source: BinaryIO,
    telemetry_destination: BinaryIO,
    events_destination: BinaryIO | None,
    progress_callback=None,
    total_bytes: int | None = None,
) -> None:
    writer = csv.DictWriter(telemetry_destination, fieldnames=_TELEMETRY_FIELD_NAMES)
    writer.writeheader()

    first_event = True

    if progress_callback is not None:
        progress_callback(0, total_bytes)

    for kind, data in _iter_records(source):
        if kind == "telemetry":
            writer.writerow(_build_telemetry_row(data))
        elif kind == "event" and events_destination is not None:
            event_object = _build_event_object(data)
            if first_event:
                events_destination.write("[\n")
            else:
                events_destination.write(",\n")
            json.dump(event_object, events_destination)
            first_event = False

        if progress_callback is not None:
            bytes_read = getattr(source, "bytes_read", None)
            progress_callback(bytes_read, total_bytes)

    if events_destination is not None:
        if first_event:
            events_destination.write("[]")
        else:
            events_destination.write("\n]\n")

    if progress_callback is not None:
        progress_callback(total_bytes, total_bytes)


def decode_with_progress(
    input_path: pathlib.Path,
    telemetry_output: pathlib.Path,
    events_output: pathlib.Path | None = None,
    progress_callback=None,
) -> None:
    if events_output is None:
        events_output = input_path.with_name(input_path.stem + "_events.json")

    total_bytes = input_path.stat().st_size if input_path.exists() else None
    with ExitStack() as stack:
        raw = stack.enter_context(input_path.open("rb"))
        source = _ProgressStream(raw)
        telemetry_dest = stack.enter_context(
            telemetry_output.open("w", newline="", encoding="utf-8")
        )
        events_dest = None
        if events_output is not None:
            events_dest = stack.enter_context(events_output.open("w", encoding="utf-8"))
        _decode_stream(
            source,
            telemetry_dest,
            events_dest,
            progress_callback=progress_callback,
            total_bytes=total_bytes,
        )


def _run(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=pathlib.Path, help="Binary log file to decode")
    parser.add_argument(
        "-o",
        "--telemetry-output",
        type=pathlib.Path,
        help="Optional CSV file to write telemetry records. Defaults to stdout.",
    )
    parser.add_argument(
        "--events-output",
        type=pathlib.Path,
        help="Path to write event records as JSON. Defaults to <input_stem>_events.json.",
        default=None,
    )
    parser.add_argument(
        "--no-events-output",
        action="store_true",
        help="Disable writing event records to a JSON file.",
    )
    args = parser.parse_args(argv)

    if args.no_events_output and args.events_output is not None:
        parser.error("--events-output and --no-events-output are mutually exclusive.")

    if args.no_events_output:
        events_path = None
    elif args.events_output is not None:
        events_path = args.events_output
    else:
        events_path = args.input.with_name(args.input.stem + "_events.json")

    with ExitStack() as stack:
        source = stack.enter_context(args.input.open("rb"))

        if args.telemetry_output is None:
            telemetry_destination = sys.stdout
        else:
            telemetry_destination = stack.enter_context(
                args.telemetry_output.open("w", newline="", encoding="utf-8")
            )

        events_destination = None
        if events_path is not None:
            events_destination = stack.enter_context(events_path.open("w", encoding="utf-8"))

        _decode_stream(source, telemetry_destination, events_destination)

    return 0


if __name__ == "__main__":
    raise SystemExit(_run())
