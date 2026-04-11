#pragma once

#include "flight_computer.h"

struct Icm20948Diagnostics {
    bool initialized = false;
    bool hasQuaternion = false;
    bool hasBootstrapYpr = false;
    bool alignmentReady = false;
    bool lastAcquireFresh = false;
    bool lastAcquireUsedCache = false;
    bool interruptConfigured = false;
    bool lastAcquireUsedInterrupt = false;
    uint32_t lastSampleMicros = 0;
    float yprDeg[3] = {0.0f, 0.0f, 0.0f};
    float bootstrapYprDeg[3] = {0.0f, 0.0f, 0.0f};
    float accelPreMountG[3] = {0.0f, 0.0f, 0.0f};
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
/// Sets the burnout timestamp for burnout correction burst feature.
void Icm20948SensorSetBurnoutTimestamp(float burnoutTimeSeconds);
/// Sets the current timestamp for burnout correction window calculation.
void Icm20948SensorSetCurrentTimestamp(float currentTimeSeconds);
/// Applies a [0, 1] external trust factor derived from cross-checking against other IMUs.
void Icm20948SensorSetCrossCheckTrust(float trust);
/// Acquires one ICM-20948 sample and updates the fused orientation outputs.
bool Icm20948SensorAcquire(SensorData &out);
/// Returns the current alignment and last-acquire diagnostics for the ICM path.
Icm20948Diagnostics Icm20948SensorGetDiagnostics();
