# Replay Library

Reusable Python helpers for replay commands.

This package is intentionally small. It exists so stable command scripts in `tools/replay/scripts/` do not each rediscover repository paths, CSV parsing, replay-binary lookup, or archived-script dispatch.

## Modules

| Module | Purpose |
| ------ | ------- |
| `paths.py` | Repository, replay, data, plots, scripts, and experiments paths. Also finds the `acs_replay` binary. |
| `csv_io.py` | Minimal CSV row loading/writing and float parsing helpers. |
| `replay_runner.py` | Builds and runs hosted `acs_replay` commands. |
| `plotting.py` | Small plotting-related helpers such as unit conversion and filename cleanup. |
| `flight_config.py` | Named flight input/output defaults. |
| `frame_adapter.py` | Pointers to archived frame-conversion tools. |
| `apogee_analysis.py` | Shared apogee/altitude helpers. |
| `cli.py` | Dispatcher support for stable wrappers that call archived scripts. |

## Adding New Tooling

Prefer this pattern:

1. Put reusable logic here.
2. Add a small entry point under `tools/replay/scripts/`.
3. Put flight-specific or one-off investigations under `tools/replay/experiments/`.

Run from the repository root:

```bash
python3 tools/replay/scripts/plot_flight.py --list
python3 tools/replay/scripts/compare_predictors.py --list
```

