# Apogee Control System

Embedded flight software and analysis tooling for active-apogee model rocketry.

This repository is built around a simple idea: treat a high-power rocket as a
real control and estimation problem, not just a logging problem. The firmware
runs on a Teensy-based flight computer, estimates vehicle state in real time,
predicts apogee during ascent, and drives flap actuation to shape the final
altitude. Around that core loop, the repo includes binary logging, replay,
decoding, visualization, and sensor-validation tooling.

## Why This Repo Exists

Passive recovery is easy to describe and hard to tune. Active apogee control is
the opposite: it forces the hardware, estimator, and aerodynamic model to agree
well enough to make a useful decision in a few hundred milliseconds.

This project is therefore organized less like a generic Arduino firmware dump
and more like a small flight-dynamics lab:

- the embedded code runs the estimator, predictor, and actuation logic
- the replay tools let you inspect the exact same signals after the flight
- the sensor stack is logged aggressively enough to audit which rails were
  trustworthy and which ones were just along for comparison

## The Core Model

At the highest level, the onboard predictor is trying to answer a narrow
question:

$$
\text{Given } h,\ v_z,\ \theta_z,\ \omega,\ \rho(h),\ \text{and drag state, what is } \hat{h}_{apogee}?
$$

The embedded estimator uses IMU data and barometric altitude to maintain a
vertical state estimate, then projects the vehicle forward through a simplified
flight model. In continuous form, the important pieces look like:

$$
\dot{h} = v_z
$$

$$
\dot{v}_z = a_z
$$

$$
a_z \approx -g - \frac{D_z(v,\rho,C_DA)}{m}
$$

with drag scaling adapted in coast using the residual between measured and
predicted acceleration:

$$
\lambda_{k+1} = \mathrm{clip}\left(\lambda_k + \alpha \frac{a_{meas} - a_{pred}}{a_{drag,pred}}\right)
$$

The implementation is intentionally pragmatic rather than fully general:

- the vertical channel is treated as the observable channel
- the predictor seeds limited horizontal motion from attitude and inertial
  acceleration
- zenith, not full 3D navigation, is the control-critical attitude quantity
- the goal is robust apogee prediction on embedded hardware, not a full
  navigation solution

## Attitude and Sensor Philosophy

The firmware uses multiple rails because no single IMU is trusted blindly.

In the current architecture:

- fast rails: `ICM20948` and `LSM9DS1`
- comparison rails: `BNO055`, `WT901`, and optional `Ellipse20`
- barometric altitude: `BMP585`

The estimator consumes fresh fast-rail data and barometric updates. Comparison
rails are still logged because they are valuable for post-flight validation,
orientation debugging, and future sensor selection.

Conceptually, each rail is reduced to two things:

- a body-frame acceleration vector
- a tilt / gravity estimate

Rail health is then judged by freshness, continuity, and agreement with the
current reference gravity vector instead of trusting any one vendor filter.

## What Is Actually Interesting Here

From a model-rocketry perspective, the project is not just "sensor fusion plus
CSV output." The interesting parts are:

- multi-rail IMU validation on a real flight controller
- apogee prediction under intentionally simplified embedded dynamics
- adaptive drag correction during coast
- flight-state transitions that must survive pad vibration, burnout, and noisy
  barometric data
- binary logging designed for replay, not just for human-readable console text

In other words, this is a repo about closing the loop between:

1. sensor behavior in the airframe
2. onboard state estimation
3. aerodynamic prediction
4. flap commands
5. post-flight evidence

## Flight Computer Pipeline

The embedded hot path lives in [`src/main.cpp`](/Users/ryanpaillet/Documents/rocketry/acs/apogee-control-system-teensy/src/main.cpp) and [`src/flight_computer.cpp`](/Users/ryanpaillet/Documents/rocketry/acs/apogee-control-system-teensy/src/flight_computer.cpp).

The control flow is:

1. Read fresh IMU and barometer data.
2. Normalize sensor frames into the rocket body frame.
3. Select the usable fast rail for raw accel/gyro input.
4. Build a main quaternion / zenith estimate from the available fast rails.
5. Run the vertical Kalman filter and inertial acceleration update.
6. Update flight-state transitions: `Ground -> Burn -> Coast -> Overshoot -> Descent`.
7. Seed the apogee predictor and estimate the remaining peak altitude.
8. Compute flap demand and log both telemetry and discrete flight events.

The binary log is not an afterthought. It is part of the architecture:

- telemetry records capture the estimator inputs and outputs
- event records capture state transitions and actuation events
- the offline tools decode and replay those records so the exact onboard logic
  can be studied later

## Repository Guide

### Embedded firmware

- [`src/`](/Users/ryanpaillet/Documents/rocketry/acs/apogee-control-system-teensy/src)
  Firmware, sensor drivers, estimator, predictor, logging, and actuation logic.
- [`include/`](/Users/ryanpaillet/Documents/rocketry/acs/apogee-control-system-teensy/include)
  Shared headers and packet definitions.
- [`lib/`](/Users/ryanpaillet/Documents/rocketry/acs/apogee-control-system-teensy/lib)
  Bundled third-party libraries and vendor code.
- [`test/`](/Users/ryanpaillet/Documents/rocketry/acs/apogee-control-system-teensy/test)
  Focused tests, including orientation checks.

### Analysis and replay

- [`python/`](/Users/ryanpaillet/Documents/rocketry/acs/apogee-control-system-teensy/python)
  Python-side estimator and apogee analysis models.
- [`tools/`](/Users/ryanpaillet/Documents/rocketry/acs/apogee-control-system-teensy/tools)
  Decoders, replay executables, dashboards, and TUI helpers.
- [`tools/replay/`](/Users/ryanpaillet/Documents/rocketry/acs/apogee-control-system-teensy/tools/replay)
  Hosted replay application and plotting scripts.

### Calibration and sensor work

- [`calibration/`](/Users/ryanpaillet/Documents/rocketry/acs/apogee-control-system-teensy/calibration)
  Calibration notes and support files.
- [`ahrs_testing/`](/Users/ryanpaillet/Documents/rocketry/acs/apogee-control-system-teensy/ahrs_testing)
  AHRS experiments and comparison harnesses.

### Documentation

- [`docs/flow.md`](/Users/ryanpaillet/Documents/rocketry/acs/apogee-control-system-teensy/docs/flow.md)
  Short walkthrough of how firmware and analysis pieces fit together.

## Quick Start

### Build the flight firmware

```bash
pio run -e flight
```

### Decode a binary flight log

```bash
python tools/replay/scripts/decode_sensor_log.py path/to/SENS010.BIN -o output.csv
```

### Build the native replay tools

```bash
cmake -S tools -B tools/build -DACS_TOOLS_ENABLE_NATIVE_CPP_GUI=OFF
cmake --build tools/build --target acs_replay -j
```

### Launch the native telemetry UI

```bash
cmake --build tools/build --target run_native_gui
```

## Current Embedded Simplifications

This repo is ambitious, but the flight code is deliberately conservative in a
few places:

- horizontal position is not estimated as a full observable state
- zenith is treated as the primary flight-axis attitude quantity
- drag adaptation is lightweight enough to live in the hot loop
- comparison rails are logged even when they are not allowed to drive the main
  estimator

Those simplifications are intentional. The objective is not to produce the most
elegant academic estimator on paper; it is to produce a flight controller that
can survive real sensor behavior and still generate defensible apogee decisions.

## Mathematical Notes

These notes are not a full derivation of the firmware. They are the shortest
set of equations that make the estimator and predictor easier to read.

### 1. Vertical state model

The onboard filter is fundamentally a vertical kinematics estimator with IMU
acceleration as input and barometric altitude as the main measurement:

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

That is the core reason the vertical channel is the most important observable
state in this repository.

### 2. Body-frame to inertial acceleration

The flight computer does not integrate the raw accelerometer directly. It first
rotates body-frame acceleration into the simplified inertial frame using zenith:

$$
\mathbf{a}_I = R_y\left(\theta_z - \frac{\pi}{2}\right)\mathbf{a}_B - \mathbf{g}
$$

with

$$
\mathbf{g} =
\begin{bmatrix}
0 \\
0 \\
g
\end{bmatrix}
$$

This is why attitude quality matters even when the predictor is mostly vertical.

### 3. Dynamic pressure and drag

The apogee predictor is built around standard aerodynamic scaling:

$$
q = \frac{1}{2}\rho V^2
$$

$$
D = q C_D A
$$

$$
a_D = \frac{D}{m}
$$

The embedded code does not assume drag is perfectly known. Instead it adapts an
effective drag scale in coast using the measured acceleration residual.

### 4. Mach number and atmospheric dependence

When Mach-dependent drag adaptation is enabled, the predictor needs both total
speed and local speed of sound:

$$
M = \frac{V}{a}
$$

$$
a = \sqrt{\gamma R T}
$$

This connects the atmosphere model directly to aerodynamic prediction instead of
treating density alone as sufficient.

### 5. Quaternion kinematics

The attitude side of the system is easiest to read if you keep the quaternion
propagation equation in mind:

$$
\dot{\mathbf{q}} = \frac{1}{2}\,\Omega(\boldsymbol{\omega})\,\mathbf{q}
$$

In practice, the firmware uses gyro propagation together with fast-rail
correction and rail-health checks instead of a full navigation-grade inertial
solution.

### 6. Barometric altitude

Pressure altitude enters the estimator through the standard atmosphere relation:

$$
h \approx 44330 \left(1 - \left(\frac{p}{p_0}\right)^{0.1903}\right)
$$

That is what turns the pressure sensor into the primary bounded vertical
measurement.

### 7. Rail agreement as an angle between gravity vectors

A useful way to think about rail consistency is not by raw Euler angles, but by
the angle between unit gravity vectors:

$$
\Delta \theta = \cos^{-1}\left(\hat{\mathbf{g}}_1 \cdot \hat{\mathbf{g}}_2\right)
$$

That one equation explains much of the health logic in the multi-IMU pipeline:
if two rails disagree on the direction of gravity, they disagree on the only
attitude quantity the apogee predictor really needs.

## A Useful Way To Read This Repository

If you are approaching this as a controls or rocketry project, read it in this
order:

1. [`src/main.cpp`](/Users/ryanpaillet/Documents/rocketry/acs/apogee-control-system-teensy/src/main.cpp)
   for the real flight loop.
2. [`src/flight_computer.cpp`](/Users/ryanpaillet/Documents/rocketry/acs/apogee-control-system-teensy/src/flight_computer.cpp)
   for estimator, transitions, and predictor seeding.
3. [`src/apogee_model.h`](/Users/ryanpaillet/Documents/rocketry/acs/apogee-control-system-teensy/src/apogee_model.h)
   and [`src/environment_model.h`](/Users/ryanpaillet/Documents/rocketry/acs/apogee-control-system-teensy/src/environment_model.h)
   for the embedded physics model.
4. [`src/data_logger.cpp`](/Users/ryanpaillet/Documents/rocketry/acs/apogee-control-system-teensy/src/data_logger.cpp)
   to see what evidence is preserved after the flight.
5. [`tools/replay/`](/Users/ryanpaillet/Documents/rocketry/acs/apogee-control-system-teensy/tools/replay)
   to understand how the flight is reconstructed offline.

## Recommended Reading

If you want the mathematics behind the repository, these are the most useful
places to start. They map reasonably well onto the code, even when the embedded
implementation is intentionally simplified.

### Model rocket aerodynamics and stability

- [James S. Barrowman, *The Practical Calculation of the Aerodynamic Characteristics of Slender Finned Vehicles*](https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20010047838.pdf)
  The classic starting point for center of pressure, normal force derivatives,
  fin-body stability, and why slender-rocket assumptions work as well as they
  do for much of amateur and high-power rocketry.
- [Sampo Niskanen, *OpenRocket Technical Documentation*](https://openrocket.info/documentation.html)
  Probably the best single rocketry-specific reference for practical simulation
  assumptions, drag estimation, stability margins, and flight phases in a model
  rocket context.

### State estimation, Kalman filtering, and inertial sensing

- [Brown and Hwang, *Introduction to Random Signals and Applied Kalman Filtering*](https://www.mathworks.com/academia/books/introduction-to-random-signals-and-applied-kalman-filtering-with-matlab-exercises-brown.html)
  A good bridge between textbook Kalman filtering and the kind of estimator
  compromises embedded systems actually make.
- [Titterton and Weston, *Strapdown Inertial Navigation Technology*](https://shop.theiet.org/strapdn-inertial-navig-t-2ed)
  The right reference if you want to think more rigorously about gyro
  integration, accelerometer interpretation, error growth, and what IMUs can
  and cannot tell you in free flight.
- [Sebastian Madgwick, *An efficient orientation filter for inertial and inertial/magnetic sensor arrays*](https://x-io.co.uk/downloads/madgwick_internal_report.pdf)
  Useful for quaternion kinematics, low-cost IMU fusion, and the practical side
  of attitude estimation when computational budget matters.

### Flight dynamics, atmosphere, and predictor assumptions

- [Robert F. Stengel, *Flight Dynamics*](https://collaborate.princeton.edu/en/publications/flight-dynamics/)
  A strong reference for turning body motion, forces, and attitude into a
  coherent systems view rather than a pile of disconnected formulas.
- [*U.S. Standard Atmosphere, 1976*](https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19770009539.pdf)
  The baseline atmosphere model behind density, pressure, and temperature
  relationships used in many simulation and predictor pipelines.

These references are not meant to imply that the firmware is a direct textbook
implementation. The code is much more pragmatic than that. But if you want to
understand why the estimator, zenith handling, and apogee predictor are
structured the way they are, these are the right places to build intuition.

## Bottom Line

This is a repository about active model-rocketry guidance in the regime where
hardware, software, and aerodynamics all have to agree enough to matter.

It is not just firmware.
It is not just replay.
It is an attempt to make a flight computer scientifically inspectable.
