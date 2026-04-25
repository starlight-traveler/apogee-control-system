# Replay Plots

Curated SVG plots used to explain estimator, predictor, and flight-analysis behavior.

Use this directory for plots that are useful to someone reviewing the project later. Scratch plots, giant generated batches, and one-off debugging output should stay local or move to `archive/` if they are still worth keeping around.

Recommended naming:

- `fullscale_<n>_<topic>.svg` for flight-specific validation.
- `<system>_<diagnostic>.svg` for general model or estimator checks.

The source command and input data assumptions should be discoverable from `tools/replay/scripts/`, `tools/replay/experiments/`, or the relevant document in `docs/`.
