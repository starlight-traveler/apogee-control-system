#pragma once

#include "flight_computer.h"

/**
 * @brief BNO085 health summary before conversion into the generic BNO wrapper.
 *
 * The BNO085 reports accel, gyro, magnetometer, and rotation-vector products on
 * separate report streams. These flags keep report freshness visible so the
 * selector can avoid combining a new attitude with stale supporting vectors.
 */
struct Bno085Diagnostics {
    bool transportReady = false;
    // Product availability and last-acquire freshness for the selected transport.
    bool hasAccel = false;
    bool hasGyro = false;
    bool hasMag = false;
    bool hasQuaternion = false;
    bool hasBootstrapYpr = false;
    bool lastAcquireFresh = false;
    // Display/debug orientation in degrees; flight math uses the quaternion.
    float yprDeg[3] = {0.0f, 0.0f, 0.0f};
    float bootstrapYprDeg[3] = {0.0f, 0.0f, 0.0f};
    // Body-frame vectors after applying the BNO085 mount transform.
    float accelBodyMps2[3] = {0.0f, 0.0f, 0.0f};
    float magBody[3] = {0.0f, 0.0f, 0.0f};
};

/**
 * @brief Latest BNO085 report products in the flight-computer frame.
 *
 * Each channel carries its own timestamp because SH-2 reports can arrive at
 * different rates. Consumers should trust a channel only when the matching flag
 * is true and the timestamp is fresh enough for the decision being made.
 */
struct Bno085Sample {
    bool hasAccel = false;
    bool hasGyro = false;
    bool hasQuaternion = false;
    // Aggregate timestamp plus per-report timestamps in microseconds.
    uint32_t sampleMicros = 0;
    uint32_t accelMicros = 0;
    uint32_t gyroMicros = 0;
    uint32_t quaternionMicros = 0;
    // Body-frame units: accel m/s^2, gyro rad/s, quaternion wxyz.
    float accel[3] = {0.0f, 0.0f, 0.0f};
    float gyro[3] = {0.0f, 0.0f, 0.0f};
    float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
};

/// Initializes the BNO085 orientation/IMU path.
bool Bno085SensorBegin();
/// Acquires the latest fresh BNO085 IMU sample into `SensorData`.
bool Bno085SensorAcquire(SensorData &out);
/// Returns true once the BNO085 path is initialized and healthy enough to serve data.
bool Bno085SensorIsInitialized();
/// Returns the current transport and per-report health of the BNO085 path.
Bno085Diagnostics Bno085SensorGetDiagnostics();
/// Returns the latest cached BNO085 sample in body-frame coordinates.
Bno085Sample Bno085SensorGetSample();
