# Apogee Control System

Flight software and analysis tools for a Teensy-based active apogee controller.

The repo has two main pieces:

- embedded firmware that estimates state in flight, predicts apogee, and drives flap actuation
- offline tools for decoding logs, replaying flights, checking sensors, and plotting results

## What The Firmware Does

At a high level, the flight loop is:

1. Read IMU and barometer data.
2. Put sensor data into a common body-frame convention.
3. Pick usable accel/gyro rails.
4. Build the current attitude / zenith estimate.
5. Run the vertical estimator.
6. Step the apogee predictor.
7. Command flap angle if the controller is active.
8. Log telemetry and discrete events.

The current flight-state progression is:

`Ground -> Burn -> Coast -> Overshoot -> Descent`

The estimator is intentionally centered on the vertical channel. It is not meant
to be a full navigation solution.

## Sensors And Rails

Current architecture:

- fast rails: `ICM20948`, `LSM9DS1`
- comparison rails: `BNO055`, `WT901`, optional `Ellipse20`
- primary barometric altitude: `BMP585`

The fast rails feed the main estimator. The comparison rails are still logged so
they can be checked later in replay or used for future rail selection work.

## Predictor Notes

The onboard predictor is trying to answer one question: given the current state,
where is the rocket going to top out?

Internally it rolls forward a compact coast state:

$$
\mathbf{x} =
\begin{bmatrix}
h \\
x \\
v_z \\
v_h \\
\theta_z \\
\omega
\end{bmatrix},
\qquad
u = \delta_{acs}
$$

where:

- `h` is altitude
- `x` is downrange distance
- `v_z` is vertical velocity
- `v_h` is horizontal speed in the predictor plane
- `\theta_z` is zenith
- `\omega` is zenith angular rate
- `\delta_{acs}` is the flap command held fixed during one rollout

The vertical part is roughly:

$$
\dot{h} = v_z
$$

$$
\dot{v}_z = a_z
$$

$$
a_z \approx -g - \frac{D_z}{m}
$$

The drag model comes from the CFD force table in `lib/cfd.csv`, with an
adaptive scale updated during coast from the difference between measured and
predicted acceleration.

The force table in this repo is stored as aerodynamic force, not pure `Cd` /
`Cn` coefficients. That matters when comparing vehicles of different size.

Dynamic pressure still follows the usual form:

$$
q = \frac{1}{2}\rho V^2
$$

and drag can still be thought of as:

$$
D = q C_D A
$$

## Repo Layout

### Firmware

- [`src/`](src/)
  Main firmware, sensor drivers, estimator, predictor, logging, actuation.
- [`include/`](include/)
  Shared headers and packet definitions.
- [`lib/`](lib/)
  Third-party libraries and vendor code.
- [`test/`](test/)
  Focused tests and checks.

### Analysis / replay

- [`python/`](python/)
  Python-side predictor and analysis code.
- [`tools/`](tools/)
  Decoders, replay tools, dashboards, and utilities.
- [`tools/replay/`](tools/replay/)
  Hosted replay application and related scripts.

### Calibration / sensor work

- [`calibration/`](calibration/)
  Calibration files and notes.
- [`ahrs_testing/`](ahrs_testing/)
  AHRS comparison and sensor experiments.

### Docs

- [`docs/flow.md`](docs/flow.md)
  Short overview of how firmware and tooling fit together.

## Where To Start Reading

If you are trying to understand the code, this is the order that makes the most
sense:

1. [`src/main.cpp`](src/main.cpp)
2. [`src/flight_computer.cpp`](src/flight_computer.cpp)
3. [`src/apogee_model.h`](src/apogee_model.h)
4. [`src/environment_model.h`](src/environment_model.h)
5. [`src/data_logger.cpp`](src/data_logger.cpp)
6. [`tools/replay/`](tools/replay/)

## Quick Start

### Build the flight firmware

```bash
pio run -e flight
```

### Decode a binary log

```bash
python tools/replay/scripts/decode_sensor_log.py path/to/SENS010.BIN -o output.csv
```

### Build the replay tools

```bash
cmake -S tools -B tools/build -DACS_TOOLS_ENABLE_NATIVE_CPP_GUI=OFF
cmake --build tools/build --target acs_replay -j
```

### Launch the native telemetry UI

```bash
cmake --build tools/build --target run_native_gui
```

## Current Simplifications

Some parts of the flight code are intentionally simple:

- horizontal position is not treated as a fully observable state
- zenith is the main attitude quantity for control
- drag adaptation is lightweight enough to stay in the hot loop
- comparison rails are logged even when they are not allowed to drive the main estimator

That is a deliberate trade. The target here is a controller that is useful on
real hardware, not a full-blown inertial navigation stack.

## Math Notes

These are the pieces that help when reading the estimator and predictor code.

### Apogee predictor walkthrough

This is the part that matters most for apogee control.

The predictor in [`src/apogee_model.h`](src/apogee_model.h) does not just use
`h` and `v_z`. It propagates a compact 2D state with attitude:

$$
\mathbf{x} =
\begin{bmatrix}
h \\
x \\
v_z \\
v_h \\
\theta_z \\
\omega
\end{bmatrix}
$$

and during one candidate rollout it treats flap angle as a fixed input:

$$
u = \delta_{acs}
$$

The state derivative used by the predictor is:

$$
\dot{h} = v_z
$$

$$
\dot{x} = v_h
$$

$$
\dot{\theta}_z = \omega
$$

The remaining pieces come from aerodynamic force lookup plus gravity.

### Relative airspeed, Mach, and angle of attack

The predictor first forms air-relative velocity using the environment model:

$$
\mathbf{v}_{rel} =
\begin{bmatrix}
v_z - w_x \\
v_h - w_y \\
-w_z
\end{bmatrix}
$$

$$
V = \left\lVert \mathbf{v}_{rel} \right\rVert
$$

The local speed of sound is:

$$
a = \sqrt{\gamma R T(h)}
$$

so Mach is:

$$
M = \frac{V}{a}
$$

The predictor then computes the relative-velocity direction in the predictor
plane:

$$
\gamma_v = \operatorname{atan2}(v_{rel,h}, v_{rel,z})
$$

and signed angle of attack:

$$
\alpha = \operatorname{wrap}_{[-\pi,\pi]}\left(\theta_z - \gamma_v\right)
$$

The code uses `|\alpha|` for the CFD lookup and keeps the sign separately to
set the direction of the normal force.

### CFD lookup and force scaling

The force table is indexed by:

$$
(\delta_{acs}, |\alpha|, M)
$$

and trilinearly interpolated from `lib/cfd.csv`:

$$
F_{axial}^{tab} = \mathcal{T}_{axial}(\delta_{acs}, |\alpha|, M)
$$

$$
F_{normal}^{tab} = \mathcal{T}_{normal}(\delta_{acs}, |\alpha|, M)
$$

Those are stored as absolute forces, not as pure `C_D` / `C_N` coefficients.

Atmospheric density is applied as a ratio to the reference density:

$$
\sigma_\rho(h) = \frac{\rho(h)}{\rho_0}
$$

The effective axial force also carries the learned drag scale:

$$
F_{axial} = \lambda(M)\,\sigma_\rho(h)\,F_{axial}^{tab}
$$

$$
F_{normal} = \sigma_\rho(h)\,F_{normal}^{tab}
$$

If Mach-dependent adaptation is disabled, `\lambda(M)` reduces to one scalar
drag scale.

### Force resolution into predictor axes

With zenith `\theta_z` and signed angle-of-attack direction
`s = \operatorname{sign}(\alpha)`, the code resolves forces into the predictor
axes as:

$$
F_{ax,z} = -F_{axial}\cos\theta_z
$$

$$
F_{ax,h} = -F_{axial}\sin\theta_z
$$

$$
F_{n,z} = -s\,F_{normal}\sin\theta_z
$$

$$
F_{n,h} = s\,F_{normal}\cos\theta_z
$$

The vertical and horizontal accelerations are then:

$$
\dot{v}_z = -g + \frac{F_{ax,z} + F_{n,z}}{m}
$$

$$
\dot{v}_h = \frac{F_{ax,h} + F_{n,h}}{m}
$$

The angular model is intentionally simple:

$$
\dot{\omega} = \frac{s\,F_{normal}\,\ell_{eff}}{I}
$$

where the current implementation uses an effective moment arm derived from the
configured CP-CG offset, not a full rigid-body aero model.

### Numerical rollout

Each prediction step evaluates the state derivative above and advances the
state with either midpoint integration or RK4.

Midpoint mode:

$$
\mathbf{k}_1 = f(\mathbf{x}_k)
$$

$$
\mathbf{k}_2 = f\left(\mathbf{x}_k + \frac{\Delta t}{2}\mathbf{k}_1\right)
$$

$$
\mathbf{x}_{k+1} = \mathbf{x}_k + \Delta t\,\mathbf{k}_2
$$

RK4 mode:

$$
\mathbf{k}_1 = f(\mathbf{x}_k)
$$

$$
\mathbf{k}_2 = f\left(\mathbf{x}_k + \frac{\Delta t}{2}\mathbf{k}_1\right)
$$

$$
\mathbf{k}_3 = f\left(\mathbf{x}_k + \frac{\Delta t}{2}\mathbf{k}_2\right)
$$

$$
\mathbf{k}_4 = f\left(\mathbf{x}_k + \Delta t\,\mathbf{k}_3\right)
$$

$$
\mathbf{x}_{k+1} =
\mathbf{x}_k +
\frac{\Delta t}{6}\left(
\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4
\right)
$$

The firmware uses RK4 for the main prediction path and midpoint for cheaper
candidate sweeps during flap-angle evaluation.

### Apogee detection inside the rollout

The rollout stops when vertical velocity crosses zero or when the integration
step limit is hit.

If the last step brackets apogee, the code refines both apogee altitude and
time by interpolating inside the final bracket:

$$
\eta = \frac{v_{z,k}}{v_{z,k} - v_{z,k+1}}
$$

$$
h_{apogee} = h_k + \eta (h_{k+1} - h_k)
$$

$$
t_{apogee} = t_k + \eta \Delta t
$$

That is why the returned apogee is not just the last altitude sample from a
descending state.

### Adaptive drag update

During coast, the firmware also compares the local measured vertical
acceleration against the model:

$$
r_a = a_{meas} - a_{pred}
$$

For the legacy single-scale path, the target drag scale is:

$$
\lambda^\star =
\operatorname{clip}\left(
\lambda + \frac{r_a}{a_{drag,axial}^{pred}}
\right)
$$

and the applied update is a first-order blend:

$$
\lambda_{k+1} = \lambda_k + \alpha(\lambda^\star - \lambda_k)
$$

When Mach-dependent learning is enabled, the same idea is applied to the drag
scale bins around the current Mach number instead of one global scalar.

### Vertical state

The vertical filter is basically a kinematics model with IMU acceleration as
input and barometric altitude as the main measurement:

$$
\mathbf{x}_k =
\begin{bmatrix}
z_k \\
v_{z,k}
\end{bmatrix}
$$

$$
\mathbf{x}_{k+1} =
\begin{bmatrix}
1 & \Delta t \\
0 & 1
\end{bmatrix}
\mathbf{x}_k
+
\begin{bmatrix}
\tfrac{1}{2}\Delta t^2 \\
\Delta t
\end{bmatrix}
a_{z,k}
+
\mathbf{w}_k
$$

$$
y_k =
\begin{bmatrix}
1 & 0
\end{bmatrix}
\mathbf{x}_k + v_k
$$

### Body-frame to inertial acceleration

The accelerometer is rotated into the simplified inertial frame using zenith:

$$
\mathbf{a}_I = R_y\left(\theta_z - \frac{\pi}{2}\right)\mathbf{a}_B - \mathbf{g}
$$

where

$$
\mathbf{g} =
\begin{bmatrix}
0 \\
0 \\
g
\end{bmatrix}
$$

### Mach and atmosphere

When Mach-dependent drag adaptation is enabled:

$$
M = \frac{V}{a}
$$

$$
a = \sqrt{\gamma R T}
$$

### Quaternion propagation

The attitude side is easiest to read if you keep the quaternion propagation
equation in mind:

$$
\dot{\mathbf{q}} = \frac{1}{2}\,\Omega(\boldsymbol{\omega})\,\mathbf{q}
$$

### Barometric altitude

Pressure altitude enters through the usual atmosphere relation:

$$
h \approx 44330 \left(1 - \left(\frac{p}{p_0}\right)^{0.1903}\right)
$$

### Rail agreement

Rail consistency is better thought of as the angle between gravity vectors than
as a comparison of Euler angles:

$$
\Delta \theta = \cos^{-1}\left(\hat{\mathbf{g}}_1 \cdot \hat{\mathbf{g}}_2\right)
$$

## References

These are the references that line up best with the code and the type of work
this repo is doing:

- [James S. Barrowman, *The Practical Calculation of the Aerodynamic Characteristics of Slender Finned Vehicles*](https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20010047838.pdf)
- [Sampo Niskanen, *OpenRocket Technical Documentation*](https://openrocket.info/documentation.html)
- [Brown and Hwang, *Introduction to Random Signals and Applied Kalman Filtering*](https://www.mathworks.com/academia/books/introduction-to-random-signals-and-applied-kalman-filtering-with-matlab-exercises-brown.html)
- [Titterton and Weston, *Strapdown Inertial Navigation Technology*](https://shop.theiet.org/strapdn-inertial-navig-t-2ed)
- [Sebastian Madgwick, *An efficient orientation filter for inertial and inertial/magnetic sensor arrays*](https://x-io.co.uk/downloads/madgwick_internal_report.pdf)
- [Robert F. Stengel, *Flight Dynamics*](https://collaborate.princeton.edu/en/publications/flight-dynamics/)
- [*U.S. Standard Atmosphere, 1976*](https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19770009539.pdf)

## Notes

- runtime settings can be updated only while the vehicle is still in `Ground`
- replay and decode tools are part of the normal debugging workflow, not side utilities
- the binary log is meant to preserve enough information to reconstruct estimator behavior after flight
