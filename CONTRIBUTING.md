# Contributing

This project is active flight software. Optimize for traceability over cleverness: small changes, clear validation, and explicit assumptions.

## Local Setup

Required tools:

- PlatformIO for firmware builds
- CMake 3.16 or newer for hosted tools
- Python 3 for replay and analysis scripts

Useful checks before opening a pull request:

```bash
pio run -e flight
pio run -e debug
cmake -S tools -B tools/build -DACS_TOOLS_ENABLE_NATIVE_CPP_GUI=OFF
cmake --build tools/build --target acs_replay -j
```

If your change touches calibration targets, also build the affected PlatformIO environment from `platformio.ini`.

## Pull Request Expectations

Every PR should state:

- what changed
- what flight behavior, bench workflow, or analysis workflow is affected
- what commands or hardware checks were run
- whether the change requires reflashing, recalibration, or replay against recorded logs

For flight-control changes, include the replay dataset or flight log used for validation when it can be shared. Do not commit raw logs or generated CSV/plot dumps unless they are intentionally curated artifacts.

## Code Organization

- Put flight behavior in `src/`.
- Put standalone bench targets in `calibration/`.
- Put hosted/native tools under `tools/`.
- Put replay and plotting scripts under `tools/replay/scripts/`.
- Keep Python model experiments in `python/`.
- Keep explanatory material in `docs/` and link it from the root README.

## Safety Review

Changes to estimator logic, apogee prediction, flap actuation, rail selection, units, coordinate frames, sensor mounting, log encoding, or runtime settings should be reviewed as flight-critical. Bench success alone is not enough; replay and hardware checks are expected before launch use.
