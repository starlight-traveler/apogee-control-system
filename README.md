# Apogee Control System

Flight software and analysis tools for a Teensy 4.1 active apogee controller.

This repository contains:

- embedded firmware for state estimation, apogee prediction, logging, telemetry, and flap actuation
- calibration targets for the sensor and actuator bench workflow
- hosted replay and decode tools for post-flight analysis
- Python utilities used to compare estimator and predictor behavior against recorded flights

This is flight-critical software. Treat every change as requiring bench review, simulation/replay, and hardware validation before use in a vehicle.

## Quick Start

Install [PlatformIO](https://platformio.org/) and CMake, then build the main firmware:

```bash
pio run -e flight
```

Useful local checks:

```bash
pio run -e debug
cmake -S tools -B tools/build -DACS_TOOLS_ENABLE_NATIVE_CPP_GUI=OFF
cmake --build tools/build --target acs_replay -j
```

Decode a binary telemetry log:

```bash
python3 tools/replay/scripts/decode_log.py path/to/SENS010.BIN -o output.csv
```

Run the hosted replay tool:

```bash
tools/build/bin/acs_replay output.csv
```

Common replay-analysis commands:

```bash
python3 tools/replay/scripts/plot_flight.py --list
python3 tools/replay/scripts/plot_flight.py --flight fullscale_4 --view validation
python3 tools/replay/scripts/compare_predictors.py --list
python3 tools/replay/scripts/compare_predictors.py --mode current-model
```

Generate source API documentation:

```bash
bash tools/scripts/generate_doxygen.sh
```

The generated API site lands at `docs/api/html/index.html`. The GitHub Pages workflow generates that directory before publishing `docs/`, so the public static docs include both the hand-written pages and the Doxygen source reference. Set the repository Pages source to GitHub Actions.

## Repository Layout

| Path | Purpose |
| ---- | ------- |
| `src/` | Flight firmware, sensor drivers, estimator, predictor, logger, telemetry, and actuation code. |
| `include/` | Shared headers and packet definitions used across firmware and host tools. |
| `lib/` | Vendored or locally patched embedded libraries plus flight data tables such as `cfd.csv`. |
| `calibration/` | Standalone PlatformIO calibration and bench-check targets. |
| `calibration/ahrs_testing/` | AHRS comparison target and Fusion sources used by that target. |
| `python/` | Python-side estimator, environment, and apogee predictor experiments. |
| `tools/` | Native decode/replay CMake workspace. |
| `tools/replay/replaylib/` | Shared Python helpers for replay commands. |
| `tools/replay/scripts/` | Stable replay, plotting, and log conversion commands. |
| `tools/replay/experiments/` | Archived flight-specific and model-specific investigations. |
| `tools/replay/data/` | Local replay data staging area. Raw logs and generated CSVs are ignored by default. |
| `tools/replay/plots/` | Curated SVG validation and diagnostic plots. |
| `telemetry/imgui/` | Native telemetry/decode UI source. |
| `docs/` | Architecture, flow, and development notes. |
| `.github/` | Issue and pull request templates. |
| `test/` | Placeholder for host-side automated checks. |

## Tooling Map

Use `tools/` when you need to decode, replay, or explain flight data without flashing hardware.

| Tool Area | Start Here | Purpose |
| --------- | ---------- | ------- |
| Native build workspace | [`tools/README.md`](tools/README.md) | Build hosted replay, native decode, and optional GUI targets. |
| Utility scripts | [`tools/scripts/README.md`](tools/scripts/README.md) | Generate Doxygen docs and run repository maintenance helpers. |
| Hosted replay | [`tools/replay/README.md`](tools/replay/README.md) | Run flight-computer logic against CSV logs. |
| Stable Python commands | [`tools/replay/scripts/README.md`](tools/replay/scripts/README.md) | Decode logs, run replay, plot flights, compare predictors. |
| Python helper library | [`tools/replay/replaylib/README.md`](tools/replay/replaylib/README.md) | Shared code for paths, CSV IO, dispatch, and replay execution. |
| Archived investigations | [`tools/replay/experiments/README.md`](tools/replay/experiments/README.md) | Flight-specific and model-specific analysis scripts kept for reproducibility. |
| Replay data | [`tools/replay/data/README.md`](tools/replay/data/README.md) | Local staging area for raw logs and generated CSVs. |
| Replay plots | [`tools/replay/plots/README.md`](tools/replay/plots/README.md) | Curated SVG plots suitable for review. |

## Firmware Overview

The flight loop is roughly:

1. Read IMU and barometer data.
2. Normalize sensor data into the flight convention.
3. Select usable accel/gyro rails.
4. Estimate attitude / zenith and vertical state.
5. Predict apogee from the current coast state.
6. Command flap angle when the controller is active.
7. Log telemetry and discrete flight events.

The current flight-state progression is:

```text
Ground -> Burn -> Coast -> Overshoot -> Descent
```

Current sensor architecture:

- fast rails: `ICM20948`, `LSM9DS1`
- comparison rails: `BNO055`, `WT901`, optional `Ellipse20`
- primary barometric altitude: `BMP585`

The estimator is intentionally centered on the vertical channel. It is not a full navigation solution.

## Where To Start Reading

Start here when changing flight behavior:

1. [`src/main.cpp`](src/main.cpp)
2. [`src/flight_computer.cpp`](src/flight_computer.cpp)
3. [`src/apogee_model.h`](src/apogee_model.h)
4. [`src/environment_model.h`](src/environment_model.h)
5. [`src/data_logger.cpp`](src/data_logger.cpp)
6. [`src/settings.h`](src/settings.h)

Supporting docs:

- [`docs/index.html`](docs/index.html): polished multi-page documentation site entry point
- [`docs/math.html`](docs/math.html): readable predictor and estimator math walkthrough
- [`docs/code.html`](docs/code.html): code highlights and source walkthrough
- [`docs/api/html/index.html`](docs/api/html/index.html): generated API docs after running Doxygen locally or in GitHub Pages
- [`docs/Doxyfile`](docs/Doxyfile): generated API docs configuration
- [`src/README.md`](src/README.md): pad settings and runtime configuration
- [`include/README.md`](include/README.md): shared header policy
- [`lib/README.md`](lib/README.md): vendored library and table policy
- [`calibration/README.md`](calibration/README.md): calibration targets
- [`tools/README.md`](tools/README.md): decode and replay tooling
- [`tools/replay/data/README.md`](tools/replay/data/README.md): local data policy
- [`tools/replay/plots/README.md`](tools/replay/plots/README.md): curated plot policy
- [`docs/README.md`](docs/README.md): durable design notes
- [`docs/flow.md`](docs/flow.md): firmware/tooling flow
- [`docs/apogee-predictor.md`](docs/apogee-predictor.md): predictor model notes
- [`CONTRIBUTING.md`](CONTRIBUTING.md): contributor workflow and review expectations

## Common PlatformIO Targets

| Target | Command | Purpose |
| ------ | ------- | ------- |
| `flight` | `pio run -e flight` | Optimized flight firmware. |
| `debug` | `pio run -e debug` | Debug firmware build. |
| `safe_math` | `pio run -e safe_math` | Flight-like build without unsafe math flags. |
| `icm_calibration` | `pio run -e icm_calibration -t upload` | ICM20948 calibration target. |
| `lsm9ds1_calibration` | `pio run -e lsm9ds1_calibration -t upload` | LSM9DS1 calibration target. |
| `pulse20_calibration` | `pio run -e pulse20_calibration -t upload` | Ellipse/Pulse20 calibration target. |
| `orientation_check` | `pio run -e orientation_check -t upload` | Sensor orientation bench check. |
| `ahrs_testing` | `pio run -e ahrs_testing -t upload` | AHRS comparison target. |
| `motor_actuation_sequence` | `pio run -e motor_actuation_sequence -t upload` | Bench actuator sequence. |

## Data Policy

Do not commit raw flight logs, generated CSVs, local build trees, or ad-hoc plots unless they are intentionally curated documentation artifacts. The `.gitignore` is set up for the normal local outputs from PlatformIO, CMake, replay, and Python scripts.

Before pushing, check:

```bash
git status --short
git check-ignore -v path/to/local/artifact
```

Generated replay data should normally stay under `tools/replay/data/`; reviewable plots should live under `tools/replay/plots/` with a script in `tools/replay/scripts/` that explains how they were made.

## License

See [`LICENSE`](LICENSE).
