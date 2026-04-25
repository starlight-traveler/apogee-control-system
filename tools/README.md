# ACS Tools

Native and Python tooling for decoding flight logs, replaying recorded flights, and validating predictor behavior.

## Layout

| Path | Purpose |
| ---- | ------- |
| `CMakeLists.txt` | Top-level native tooling build. |
| `decode/` | Binary log decoders, including the native fast decoder. |
| `replay/` | Hosted replay executable, Python replay commands, reusable helpers, experiments, data, and plots. |
| `build/` | Local CMake output. Ignored by Git. |

Use the top-level CMake workspace from the repository root:

```bash
cmake -S tools -B tools/build -DACS_TOOLS_ENABLE_NATIVE_CPP_GUI=OFF
cmake --build tools/build -j
```

The common native outputs are placed under `tools/build/bin/`.

## Native Targets

| Target | Purpose |
| ------ | ------- |
| `acs_ndrt_rocketry_fast_decode` | Fast binary log decoder. |
| `acs_fast_decode` | Legacy alias for the fast decoder when provided by the build. |
| `acs_replay` | Hosted replay executable that reuses flight logic against CSV input. |
| `acs_ndrt_rocketry_decoder` | Native CSV/BIN analysis UI when GUI dependencies are enabled. |
| `teensy_imgui_telemetry` | Native telemetry UI from `telemetry/imgui` when GUI dependencies are enabled. |

Build replay without GUI dependencies:

```bash
cmake -S tools -B tools/build -DACS_TOOLS_ENABLE_NATIVE_CPP_GUI=OFF
cmake --build tools/build --target acs_replay -j
```

Build the native decode utility:

```bash
cmake -S tools -B tools/build -DACS_TOOLS_BUILD_DECODER=ON
cmake --build tools/build --target acs_ndrt_rocketry_fast_decode -j
```

## Decode Logs

Python decoder:

```bash
python3 tools/replay/scripts/decode_log.py path/to/SENS010.BIN -o output.csv
```

Native decoder after building tools:

```bash
tools/build/bin/acs_ndrt_rocketry_fast_decode path/to/SENS010.BIN -o output.csv
```

For long logs with idle time, the native decoder supports smart trimming:

```bash
tools/build/bin/acs_ndrt_rocketry_fast_decode input.BIN -o output.csv --smart-parser \
  --smart-pre-seconds 3 --smart-post-seconds 10 \
  --smart-min-alt-ft 25 --smart-min-vel-ftps 20 --smart-min-cmd-deg 0.5
```

## Hosted Replay

```bash
tools/build/bin/acs_replay output.csv
```

Replay implementation and usage details live in [`tools/replay/README.md`](replay/README.md).

## Python Replay Scripts

Run Python replay helpers from the repo root:

```bash
python3 tools/replay/scripts/plot_flight.py --flight fullscale_4 --view validation
python3 tools/replay/scripts/compare_predictors.py --mode current-model
```

Stable command documentation lives in [`replay/scripts/README.md`](replay/scripts/README.md). Archived investigation scripts live in [`replay/experiments/`](replay/experiments/).

Generated CSVs, raw logs, build trees, and ad-hoc plots should stay local unless they are intentionally curated documentation artifacts.
