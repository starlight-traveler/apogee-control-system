# Standalone Flight Computer Driver

This directory contains a small hosted application that reuses the
Teensy flight computer logic to replay telemetry from CSV files. It makes
it possible to tune Kalman filter parameters and validate the apogee
prediction algorithm without loading firmware onto the rocket avionics.

## Building

```bash
cd standalone-c
make
```

The build produces the `flight_computer_standalone` executable next to
the Makefile. Set `CXX` if you prefer a different compiler, and extend
`CXXFLAGS`/`LDFLAGS` as needed.

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
./flight_computer_standalone data.csv \
    --field timestamp=Time_s \
    --field altitude_feet=Alt_pad_ft \
    --field accel_icm_z=AX_wf
```

## Running

```bash
./flight_computer_standalone flight_log.csv [options]
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
