# Hosted Replay Driver

This directory contains a small hosted application that reuses the
Teensy flight computer logic to replay telemetry from CSV files. It makes
it possible to tune Kalman filter parameters and validate the apogee
prediction algorithm without loading firmware onto the rocket avionics.

## Layout

- `scripts/`: Python replay analysis, plotting, and tuning utilities
- `replaylib/`: reusable Python helpers shared by the stable replay commands
- `experiments/`: archived flight-specific and model-specific investigations
- `data/`: local CSV, event, spreadsheet, and replay artifacts; ignored by default
- `plots/`: curated SVG comparison plots suitable for review
- `include/`, `main.cpp`, `arduino_stubs.cpp`: hosted replay source
- `build/`: CMake output

## Command Layers

Use the layers this way:

| Layer | Path | Use |
| ----- | ---- | --- |
| Supported commands | `scripts/` | Normal decode, replay, plotting, and predictor comparisons. |
| Shared helpers | `replaylib/` | New reusable Python code used by supported commands. |
| Archived scripts | `experiments/` | Old flight-specific investigations retained for reproducibility. |
| Native replay source | `main.cpp`, `include/`, `arduino_stubs.cpp` | Hosted C++ executable that reuses firmware logic. |

New tooling should usually add helper code in `replaylib/` and expose it through a small script in `scripts/`.

## Building

```bash
cd tools/replay
cmake -S . -B build
cmake --build build -j
```

The build produces `build/bin/acs_replay`. A legacy `Makefile` is still
present for ad-hoc local builds, but CMake is now the primary entry point.

## Input format

The tool expects a header row and at least the following columns:

- `timestamp` (seconds)
- `altitude_ft` **or** `altitude_m`

If both the ICM and BNO accelerometer columns are present they are used
exactly like the embedded firmware. If only one sensor exists, its
values are reused for the missing sensor so the algorithm can still run.

Recognised column names are listed when you run the program with
`--help`. You can also override any mapping with
`--field <field>=<header-name>`, which is useful when your CSV uses
custom labels, e.g.

```bash
./build/bin/acs_replay data.csv \
    --field timestamp=Time_s \
    --field altitude_feet=Alt_pad_ft \
    --field accel_icm_z=AX_wf
```

## Running

```bash
./build/bin/acs_replay flight_log.csv [options]
```

Important flags:

| Flag | Description |
| ---- | ----------- |
| `--sigma-accel-xy`, `--sigma-accel-z`, `--sigma-altimeter` | Measurement sigmas for the Kalman filters |
| `--process-xy`, `--process-z` | Process noise terms |
| `--apogee-target` | Target apogee altitude in meters |
| `--include-raw-altimeter` | Append the raw altimeter measurement (meters) to each CSV line |
| `--include-raw <fields>` | Append any combination of raw sensor channels (e.g. `accel_icm`, `gyro_z`) |
| `--graph <fields>` | Render ASCII graphs of the requested signals after processing |
| `--field <field>=<header>` | Override a single column mapping |
| `--quiet` | Suppress per-sample CSV output (summary only) |

Raw output and graph field names are listed in `--help`. You can supply
comma-separated values (for example, `--include-raw accel_icm,gyro` or
`--graph altitude_m,accel_icm_z`). Group names such as `accel_icm` and
`gyro` automatically expand to their respective axes.

While running, the program emits a filtered telemetry CSV to stdout
(`time_s, altitude_m, velocity_mps, apogee_prediction_m, status`) and a
summary that includes burn/apogee detection times and the most recent
apogee estimate.

## Python utilities

Run the stable Python replay commands from the repo root, for example:

```bash
python3 tools/replay/scripts/decode_log.py tools/replay/scripts/SENS065.BIN -o tools/replay/data/SENS065.csv
python3 tools/replay/scripts/plot_flight.py --flight fullscale_4 --view validation
python3 tools/replay/scripts/compare_predictors.py --mode current-model
```

Archived investigation scripts live under `tools/replay/experiments/`. They are kept for reproducibility, but the supported command surface is `tools/replay/scripts/`.
