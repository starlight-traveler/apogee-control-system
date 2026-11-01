#pragma once

#include "flight_computer.h"

/**
 * @brief health, trust, and last-sample diagnostics for the lsm9ds1 rail.
 *
 * The LSM rail carries its own accel/gyro/mag fusion path. During boost the
 * accel and magnetometer can be bad attitude references, so the trust fields
 * expose how much the fusion path believed each correction source instead of
 * making replay guess from the final quaternion alone.
 */
struct Lsm9ds1Diagnostics {
    bool initialized = false;
    // Product availability flags from the sensor/fusion path.
    bool hasAccel = false;
    bool hasGyro = false;
    bool hasMag = false;
    bool hasQuaternion = false;
    bool hasBootstrapYpr = false;
    bool alignmentReady = false;
    // Fresh/cache flags protect against accidentally treating old IMU data as live.
    bool lastAcquireFresh = false;
    bool lastAcquireUsedCache = false;
    bool interruptConfigured = false;
    bool lastAcquireUsedInterrupt = false;
    bool fifoEnabled = false;
    uint32_t lastSampleMicros = 0;
    // Ground alignment count shows whether the initial gravity/mag references had enough samples.
    uint16_t groundAlignmentSampleCount = 0;
    // Trust and magnitude values explain why accel/mag corrections were accepted or rejected.
    float lastAccelTrust = 0.0f;
    float lastMagTrust = 0.0f;
    float lastAccelMagnitudeG = 0.0f;
    float lastMagMagnitude = 0.0f;
    float magReferenceNorm = 0.0f;
    // YPR is for display; quaternion/body vectors are the control-facing products.
    float yprDeg[3] = {0.0f, 0.0f, 0.0f};
    float bootstrapYprDeg[3] = {0.0f, 0.0f, 0.0f};
    float accelBodyMps2[3] = {0.0f, 0.0f, 0.0f};
    float magBody[3] = {0.0f, 0.0f, 0.0f};
};

/// @brief initializes the lsm9ds1 accel/gyro/mag rail.
bool Lsm9ds1SensorBegin();
/// @brief returns true once the lsm9ds1 path has initialized.
bool Lsm9ds1SensorIsInitialized();
/// @brief gives the rail the current flight phase so trust rules can change.
void Lsm9ds1SensorSetFlightStatus(FlightStatus status);
/// @brief sets burnout time for short post-burn attitude correction behavior.
void Lsm9ds1SensorSetBurnoutTimestamp(float burnoutTimeSeconds);
/// @brief sets current time for phase/timing-dependent correction windows.
void Lsm9ds1SensorSetCurrentTimestamp(float currentTimeSeconds);
/// @brief applies external cross-check trust from other imu rails.
void Lsm9ds1SensorSetCrossCheckTrust(float trust);
/// @brief acquires one fresh lsm9ds1 sample and publishes it into `SensorData`.
bool Lsm9ds1SensorAcquire(SensorData &out);
/// @brief returns the latest lsm9ds1 diagnostics.
Lsm9ds1Diagnostics Lsm9ds1SensorGetDiagnostics();
