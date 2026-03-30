#pragma once

#include "flight_computer.h"

struct Icm20948Diagnostics {
    bool initialized = false;
    bool hasQuaternion = false;
    bool alignmentReady = false;
    bool lastAcquireFresh = false;
    bool lastAcquireUsedCache = false;
    bool interruptConfigured = false;
    bool lastAcquireUsedInterrupt = false;
    uint32_t lastSampleMicros = 0;
};

/// Initializes the optional ICM-20948 IMU path.
bool Icm20948SensorBegin();
/// Returns true once the ICM-20948 path is initialized.
bool Icm20948SensorIsInitialized();
/// Updates the ICM fusion policy with the latest known flight phase.
void Icm20948SensorSetFlightStatus(FlightStatus status);
/// Applies a [0, 1] external trust factor derived from cross-checking against other IMUs.
void Icm20948SensorSetCrossCheckTrust(float trust);
/// Acquires one ICM-20948 sample and updates the fused orientation outputs.
bool Icm20948SensorAcquire(SensorData &out);
/// Returns the current alignment and last-acquire diagnostics for the ICM path.
Icm20948Diagnostics Icm20948SensorGetDiagnostics();
