#pragma once

#include "flight_computer.h"

/**
 * @brief BNO-family health summary shared by both BNO055 and BNO085 paths.
 *
 * These fields answer two different questions: whether each sensor product is
 * currently available, and whether the most recent acquisition actually
 * delivered fresh data. Keeping those separate matters because a device can be
 * initialized and still miss an update on a given loop.
 */
struct BnoDiagnostics {
    bool transportReady = false;
    // Product availability flags reported by the selected BNO implementation.
    bool hasAccel = false;
    bool hasGyro = false;
    bool hasMag = false;
    bool hasQuaternion = false;
    bool hasBootstrapYpr = false;
    // True only when the latest acquire call produced a usable new sample.
    bool lastAcquireFresh = false;
    // Human-readable orientation diagnostics; control uses quaternions instead.
    float yprDeg[3] = {0.0f, 0.0f, 0.0f};
    float bootstrapYprDeg[3] = {0.0f, 0.0f, 0.0f};
    // Body-frame vectors after each driver has applied its sensor mount mapping.
    float accelBodyMps2[3] = {0.0f, 0.0f, 0.0f};
    float magBody[3] = {0.0f, 0.0f, 0.0f};
};

/**
 * @brief Common BNO-family sample shape consumed by the flight computer.
 *
 * The per-product timestamps let the selector decide whether accel, gyro, and
 * attitude are fresh independently. A quaternion can be valid while the accel
 * vector is stale, so downstream code should check the matching `has*` flag
 * before mixing the channels.
 */
struct BnoSample {
    bool hasAccel = false;
    bool hasGyro = false;
    bool hasQuaternion = false;
    // Aggregate timestamp plus per-channel timestamps in microseconds.
    uint32_t sampleMicros = 0;
    uint32_t accelMicros = 0;
    uint32_t gyroMicros = 0;
    uint32_t quaternionMicros = 0;
    // All vectors are reported in the rocket body frame expected by the estimator.
    float accel[3] = {0.0f, 0.0f, 0.0f};
    float gyro[3] = {0.0f, 0.0f, 0.0f};
    float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
};

/// Supported BNO-family device names as user-facing strings.
const char *BnoSensorModelName();
/// Supported BNO transport name as a user-facing string.
const char *BnoSensorTransportName();
/// Initializes whichever BNO family device is selected in settings.
bool BnoSensorBegin();
/// Acquires the latest BNO-family IMU sample into `SensorData` when fresh.
bool BnoSensorAcquire(SensorData &out);
/// Returns true once the selected BNO-family device is initialized.
bool BnoSensorIsInitialized();
/// Returns the latest generic diagnostics for the selected BNO-family device.
BnoDiagnostics BnoSensorGetDiagnostics();
/// Returns the latest generic sample for the selected BNO-family device.
BnoSample BnoSensorGetSample();
