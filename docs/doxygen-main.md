# Apogee Control System API

This generated reference covers the flight firmware, shared headers, calibration helpers, replay entry points, and telemetry UI source that are useful when navigating the code.

Start with these source areas:

- `src/flight_computer.cpp` for estimator, phase, and predictor seed behavior.
- `src/apogee_model.h` for the coast physics model, CFD interpolation, RK integration, and adaptive drag scale.
- `src/main.cpp` for sensor orchestration, actuation optimization, runtime settings, and logging.
- `src/settings.h` for tunable constants and compile-time configuration.
- `include/telemetry_packet.h` for host/firmware telemetry packets.

The hand-written documentation site lives in `docs/index.html`. The generated HTML entry point is `docs/api/html/index.html`.
