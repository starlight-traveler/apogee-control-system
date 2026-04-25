# Program Flow

This document describes how the firmware and Python analysis pieces fit
together, and where to look when you need to change behavior.

## Firmware pipeline (PlatformIO)

1. Sensor drivers (`src/bmp585_sensor.*`, `src/bno085_sensor.*`)
   provide raw measurements to the flight computer.
2. `src/flight_computer.*` owns the main control loop:
   - pulls raw sensor samples
   - normalizes frames and applies orientation handling
   - updates filters and apogee prediction
   - emits flight status transitions
3. `src/kalman_filters.h` and `src/apogee_model.h` mirror the Python logic
   in embedded-friendly form.
4. `src/data_logger.*` serializes telemetry and event records into the
   binary log format for post-flight analysis.

## Python analysis pipeline

1. `python/flight.py` reads a CSV log and runs the estimator loop in Python.
2. `python/filter.py` performs the Kalman filter updates and sensor fusion.
3. `python/apogee.py` and `python/apogee_lib.py` compute apogee predictions.
4. `python/convert.py` and `python/environment.py` handle frame and
   atmospheric conversions.

If you update the Python models, check the corresponding C++ files
(`src/kalman_filters.h`, `src/apogee_model.h`, `src/environment_model.h`)
so the embedded logic stays aligned.

## Offline tools

- `tools/replay/scripts/decode_log.py` decodes the binary log
  produced by `src/data_logger.*` into CSV + JSON events.
- `tools/replay/scripts/plot_flight.py` and
  `tools/replay/scripts/compare_predictors.py` are the stable replay-analysis
  entry points. Flight-specific experiments are archived under
  `tools/replay/experiments/`.
- `tools/decode/native/fast_decode_sensor_log.cpp` is the native fast
  decoder built through the top-level tools CMake workspace.
- Local datasets and generated replay artifacts should stay out of git unless
  they are intentionally curated documentation fixtures.

## Hosted replay tool

`tools/replay/` hosts a desktop executable that reuses the embedded logic
and can graph filtered telemetry. It vendors Matplot++ in place, so keep
that subtree intact.
