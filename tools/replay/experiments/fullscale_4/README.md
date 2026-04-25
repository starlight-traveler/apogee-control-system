# Fullscale 4 Experiments

Historical fullscale 4 replay, frame-adapter, quaternion, and actuation analyses.

Preferred entry points:

```bash
python3 tools/replay/scripts/plot_flight.py --flight fullscale_4 --view validation
python3 tools/replay/scripts/plot_flight.py --flight fullscale_4 --view overshoot-actuation
python3 tools/replay/scripts/adapt_legacy_frame.py --preset best-coast tools/replay/data/fullscale_4.csv
```
