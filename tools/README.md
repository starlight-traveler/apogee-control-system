# ACS NDRT Rocketry Tools

## gui_dashboard.py (Recommended)

Local web GUI that consolidates decode + CSV/events browsing + smart plotting.

```bash
python tools/gui_dashboard.py
```

Safari HTTPS-Only users:

```bash
python tools/gui_dashboard.py --https
```

For fast BIN decoding, build the native decoder once:

```bash
cmake -S tools -B tools/build
cmake --build tools/build -j
```

This creates `tools/build/bin/acs_ndrt_rocketry_fast_decode` (and legacy `acs_fast_decode`). The GUI and `decode_sensor_log.py`
will automatically use it when present.

For very large logs with long idle time, use smart trimming:

```bash
tools/build/bin/acs_ndrt_rocketry_fast_decode input.BIN -o output.csv --smart-parser \
  --smart-pre-seconds 3 --smart-post-seconds 10 \
  --smart-min-alt-ft 25 --smart-min-vel-ftps 20 --smart-min-cmd-deg 0.5
```

Features:
- scans for `.BIN`, `.csv`, and `_events.json` under a root directory
- one-click BIN decode via `decode_sensor_log.py`
- preset graph views (flight overview, actuation/optimizer, baro quality, dynamics)
- event overlay markers on plots
- point decimation + in-memory caching for quick reloads

Optional CMake helper target:

```bash
cmake --build tools/build --target run_gui
```

## Native C++ GUI

For a native C++ GUI (Dear ImGui/OpenGL), use `local-imgui-telemetry` via the
same tools CMake workspace:

```bash
cmake -S tools -B tools/build
cmake --build tools/build -j
cmake --build tools/build --target run_native_gui
```

Windows notes:
- `acs_ndrt_rocketry_decoder` builds on Windows from the same CMake flow.
- `teensy_imgui_telemetry` is currently POSIX-socket based and is auto-disabled on Windows.

For offline CSV/BIN analysis (range graphing, signal picker, event markers):

```bash
cmake --build tools/build --target run_native_decoder_gui
tools/build/bin/acs_ndrt_rocketry_decoder path/to/SENS010.BIN
```

## tui_dashboard.py

Unified TUI dashboard for choosing CSV, JSON, or binary log inputs.

```bash
python tools/tui_dashboard.py
```

On first launch the dashboard asks for a data root directory and stores it
in `tools/.tui_settings.json`. You can change it later from the Settings
menu.

The dashboard also includes a Rocketry Analysis panel that summarizes
flight metrics (apogee, max velocity, burn/coast/descent timing) and can
estimate thrust-to-weight and drag if you provide mass and reference area
in Settings.

## decode_sensor_log.py

Decode binary telemetry logs written by the flight computer into CSV + JSON.

```bash
python tools/decode_sensor_log.py data/subscale_3/SENS010.BIN \
  -o data/subscale_3/sens010.csv
```

## tui_telemetry_viewer.py

Scrollable TUI for telemetry CSV files (arrow keys, PgUp/PgDn, q to quit).

```bash
python tools/tui_telemetry_viewer.py data/subscale_3/sens010.csv
```

Pick specific columns:

```bash
python tools/tui_telemetry_viewer.py data/subscale_3/sens010.csv \
  --columns state_time,state_apogee_estimate,flight_status
```

## tui_tail_csv.py

Live tail view of the latest CSV rows.

```bash
python tools/tui_tail_csv.py data/subscale_3/sens010.csv --rows 15 --interval 0.5
```

## tui_event_viewer.py

Scrollable TUI for event JSON files produced by `decode_sensor_log.py`.

```bash
python tools/tui_event_viewer.py data/events/sample_events.json
```
