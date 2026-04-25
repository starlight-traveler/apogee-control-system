# Apogee Predictor Notes

The onboard predictor answers one question during coast: given the current state and a candidate flap command, where will the vehicle top out?

The implementation lives primarily in [`src/apogee_model.h`](../src/apogee_model.h), with atmosphere support in [`src/environment_model.h`](../src/environment_model.h) and CFD table lookup in [`src/cfd_table.*`](../src/cfd_table.h).

## State

The predictor propagates a compact two-dimensional coast state:

```text
h         altitude
x         downrange distance
v_z       vertical velocity
v_h       horizontal speed in the predictor plane
theta_z   zenith angle
omega     zenith angular rate
```

During one rollout, flap angle is treated as a fixed input.

## Model

Each step:

1. Computes air-relative velocity from the vehicle state and configured wind model.
2. Converts speed to Mach using the local atmosphere.
3. Computes angle of attack in the predictor plane.
4. Looks up axial and normal aerodynamic forces from `lib/cfd.csv`.
5. Scales force by density ratio and the learned drag scale.
6. Resolves aerodynamic force into vertical and horizontal predictor axes.
7. Advances the state with the configured numerical integrator.

The force table is stored as aerodynamic force, not pure `Cd` / `Cn` coefficients. That matters when comparing vehicles of different size.

## Integration And Stop Condition

The main prediction path uses RK4. Cheaper candidate sweeps can use midpoint integration.

The rollout stops when vertical velocity crosses zero or when the integration step limit is hit. If the final step brackets apogee, the returned altitude and time are interpolated inside that final bracket rather than using the last sample directly.

## Adaptive Drag

During coast, the firmware compares measured vertical acceleration against modeled acceleration and updates a lightweight drag scale. Current code supports both a legacy single-scale path and Mach-binned learning.

Keep the C++ predictor and the Python analysis code aligned when changing model behavior:

- C++ flight path: [`src/apogee_model.h`](../src/apogee_model.h)
- C++ environment: [`src/environment_model.h`](../src/environment_model.h)
- Python predictor experiments: [`python/apogee.py`](../python/apogee.py), [`python/apogee_lib.py`](../python/apogee_lib.py)
- Replay scripts: [`tools/replay/scripts/`](../tools/replay/scripts/)
