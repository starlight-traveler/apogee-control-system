# Documentation

Project-level architecture and safety notes live here. Keep this directory focused on durable explanations rather than one-off flight analysis output.

## Index

- [`index.html`](index.html): polished overview and landing page for the apogee control system docs.
- [`flight-loop.html`](flight-loop.html): interactive phase and firmware-loop walkthrough.
- [`predictor.html`](predictor.html): interactive apogee predictor explanation and simplified intuition tool.
- [`math.html`](math.html): readable equations for coast energy, aero force scaling, RK4 integration, and source mapping.
- [`code.html`](code.html): implementation highlights and source walkthrough for the predictor, estimator, IMU, and actuation paths.
- [`sensors.html`](sensors.html): sensor rail, frame, and freshness guide.
- [`replay.html`](replay.html): replay, plotting, and post-flight analysis workflow.
- [`flow.md`](flow.md): firmware and tooling data flow.
- [`apogee-predictor.md`](apogee-predictor.md): apogee predictor model, assumptions, and known limitations.
- [`Doxyfile`](Doxyfile): Doxygen configuration for generated source API docs.
- [`doxygen-main.md`](doxygen-main.md): generated API docs landing page content.

When adding a new design note, prefer a short Markdown document here and link it from the root README if it becomes part of the normal development workflow.

## Generated API Docs

Install Doxygen, then run:

```bash
bash tools/scripts/generate_doxygen.sh
```

The generated entry point is `docs/api/html/index.html`. The `docs/api/` directory is ignored by git; commit the comments and Doxygen config, not the generated HTML.
