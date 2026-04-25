# Replay Experiments

Archived flight investigations and model experiments live here. These scripts are intentionally kept out of `tools/replay/scripts/` so the public command surface stays small.

Use the stable wrappers in `tools/replay/scripts/` for normal work:

```bash
python3 tools/replay/scripts/plot_flight.py --flight fullscale_4 --view validation
python3 tools/replay/scripts/compare_predictors.py --mode current-model
python3 tools/replay/scripts/adapt_legacy_frame.py --preset best-coast tools/replay/data/fullscale_4.csv
```

The archived scripts are retained because they capture specific investigations. They may still be runnable, but new reusable behavior should go into `tools/replay/replaylib/` and be exposed through a stable command.
