# apogee-control-system

Apogee Control System firmware plus Python-based analysis and tuning tools.

## Repository map

- `src/` - PlatformIO firmware sources (Teensy flight computer).
- `include/`, `lib/`, `test/` - PlatformIO headers, libraries, and tests.
- `python/` - Python analysis scripts and models.
- `tools/` - Native decoders, replay utilities, and offline analysis helpers.
- `telemetry/` - Live ground-station clients and telemetry UI code.
- `data/` - Sample logs, events, and subscale datasets.
- `docs/` - Project flow and architecture notes.

## Firmware flow (PlatformIO)

1. `src/main.cpp` boots the firmware and wires up the flight computer loop.
2. `src/flight_computer.*` manages sensor reads, state estimation, and staging logic.
3. `src/kalman_filters.h`, `src/apogee_model.h`, `src/environment_model.h` provide the
   embedded equivalents of the Python models.
4. `src/data_logger.*` writes telemetry/event records for offline decoding.
5. `src/network_telemetry.*` streams fixed-rate UDP telemetry over Wi-Fi for live ground display.

## Live network telemetry (Teensy + AirLift)

- Firmware packet schema: `include/telemetry_packet.h` (`telemetry::PacketV1`).
- Wi-Fi and UDP configuration: `src/settings.h` under `settings::network`.
- AirLift pins are configurable via:
  - `settings::network::kAirliftSsPin`
  - `settings::network::kAirliftAckPin`
  - `settings::network::kAirliftResetPin`
  - `settings::network::kAirliftGpio0Pin`
- Stream rate is capped by `settings::network::kTelemetryIntervalMs` to limit loop overhead.
- Telemetry can be subscriber-gated via heartbeat (`kRequireSubscriberHeartbeat`), so send work is skipped when the ground client disconnects.

Local ImGui receiver app: `telemetry/imgui/`.

## Python analysis flow

- `python/flight.py` replays logs and runs the estimator loop in Python.
- `python/filter.py`, `python/apogee.py`, `python/convert.py`, `python/environment.py`,
  `python/constants.py` supply the models used by `python/flight.py`.
- `python/apogee_lib.py` contains shared helpers for apogee prediction.
- `python/kalman_filter.py` is a vendored Kalman filter implementation.

> Note: `python/flight.py` and `python/filter.py` import `math_lib.py`, which is
> not present in this repo. Add it if you want to run those scripts end-to-end.

## Tools and data

- Decode a binary sensor log:
  `python tools/decode_sensor_log.py data/subscale_3/SENS010.BIN -o data/subscale_3/sens010.csv`
- Example event output is in `data/events/sample_events.json`.
- TUI helpers for debugging:
  - `python tools/tui_dashboard.py`
  - `python tools/tui_telemetry_viewer.py data/subscale_3/sens010.csv`
  - `python tools/tui_tail_csv.py data/subscale_3/sens010.csv --rows 15`
  - `python tools/tui_event_viewer.py data/events/sample_events.json`

## More detail

See `docs/flow.md` for a deeper walkthrough of the firmware and analysis pipeline.
