#pragma once

#include "flight_computer.h"

/**
 * @brief health, alignment, and last-sample diagnostics for the ICM-20948 rail.
 *
 * The ICM path can be a strong gyro/accel source, but it is still only useful
 * when its sample is fresh and its mount/alignment transform is ready. These
 * diagnostics keep those conditions visible in telemetry and replay instead of
 * hiding them behind a single "initialized" flag.
 */
struct Icm20948Diagnostics {
    bool initialized = false;
    // Availability of fused products from the driver/fusion path.
    bool hasQuaternion = false;
    bool hasBootstrapYpr = false;
    bool alignmentReady = false;
    // Fresh/cache flags distinguish a real new sample from reused last data.
    bool lastAcquireFresh = false;
    bool lastAcquireUsedCache = false;
    // Interrupt flags show whether the rail is running from data-ready timing.
    bool interruptConfigured = false;
    bool lastAcquireUsedInterrupt = false;
    uint32_t lastSampleMicros = 0;
    // YPR is diagnostic only; quaternion/body vectors are used by control code.
    float yprDeg[3] = {0.0f, 0.0f, 0.0f};
    float bootstrapYprDeg[3] = {0.0f, 0.0f, 0.0f};
    // Pre-mount vectors are raw-or-close-to-raw views for frame debugging.
    float accelPreMountG[3] = {0.0f, 0.0f, 0.0f};
    // Body-frame vectors are what the flight computer should compare/blend.
    float accelBodyMps2[3] = {0.0f, 0.0f, 0.0f};
    float magPreAxis[3] = {0.0f, 0.0f, 0.0f};
    float magBody[3] = {0.0f, 0.0f, 0.0f};
};

/// Initializes the optional ICM-20948 IMU path.
bool Icm20948SensorBegin();
/// Returns true once the ICM-20948 path is initialized.
bool Icm20948SensorIsInitialized();
/// Updates the ICM fusion policy with the latest known flight phase.
void Icm20948SensorSetFlightStatus(FlightStatus status);
/// Sets the burnout timestamp for the short post-burn correction burst.
void Icm20948SensorSetBurnoutTimestamp(float burnoutTimeSeconds);
/// Sets the current timestamp for burnout correction window calculation.
void Icm20948SensorSetCurrentTimestamp(float currentTimeSeconds);
/// Applies a [0, 1] external trust factor derived from cross-checking against other IMUs.
void Icm20948SensorSetCrossCheckTrust(float trust);
/// Acquires one fresh ICM-20948 sample and updates fused orientation outputs.
bool Icm20948SensorAcquire(SensorData &out);
/// Returns the current alignment and last-acquire diagnostics for the ICM path.
Icm20948Diagnostics Icm20948SensorGetDiagnostics();
