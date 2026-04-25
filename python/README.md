# Python Analysis

This directory contains offline estimator and apogee-predictor experiments. These scripts are useful for model exploration and comparison against replay data, but the flight firmware implementation lives in `src/`.

Typical use from the repository root:

```bash
env PYTHONPATH=. python3 python/apogee_predictor_sim.py
```

Keep large inputs and generated outputs out of this directory. Raw logs, decoded CSVs, and replay products belong under `tools/replay/data/` and are ignored by default.
