# Replay Scripts

Stable command-line entry points for decoding logs, running hosted replay, and generating curated analysis outputs.

## Supported Commands

| Script | Purpose |
| ------ | ------- |
| `decode_log.py` | Decode binary sensor logs into CSV/event outputs. |
| `run_replay.py` | Run the hosted `acs_replay` binary with a stable wrapper. |
| `plot_flight.py` | Dispatch curated flight plots by `--flight` and `--view`. |
| `compare_predictors.py` | Run predictor/model comparison modes. |
| `calibrate_attitude.py` | Align ACS attitude to a recovery-module reference log. |
| `adapt_legacy_frame.py` | Adapt historical logs into the current replay frame convention. |

## Common Commands

```bash
python3 tools/replay/scripts/decode_log.py tools/replay/scripts/SENS065.BIN -o tools/replay/data/SENS065.csv
python3 tools/replay/scripts/run_replay.py tools/replay/data/SENS065.csv --quiet
python3 tools/replay/scripts/plot_flight.py --flight fullscale_4 --view validation
python3 tools/replay/scripts/plot_flight.py --list
python3 tools/replay/scripts/compare_predictors.py --list
```

## How Dispatch Works

The scripts in this directory are the supported interface. Some of them dispatch to archived implementations under `tools/replay/experiments/`:

- `plot_flight.py` maps `--flight` and `--view` to the matching archived plot script.
- `compare_predictors.py` maps `--mode` to an archived predictor/model experiment.
- `calibrate_attitude.py` runs the fullscale 3 attitude calibration script.
- `adapt_legacy_frame.py` runs the fullscale 4 frame adapter or state-zenith recalculator.

Any extra arguments are passed through to the archived implementation. For example:

```bash
python3 tools/replay/scripts/plot_flight.py --flight fullscale_4 --view validation --clean
python3 tools/replay/scripts/adapt_legacy_frame.py --preset best-coast tools/replay/data/fullscale_4.csv
```

Old one-off analysis scripts live under `tools/replay/experiments/`. Keep new reusable behavior in `tools/replay/replaylib/` and expose it through one of the stable commands above.

Raw logs and generated outputs should stay local. Commit scripts and curated plot outputs, not bulk data products.
