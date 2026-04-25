#pragma once

#include <Arduino.h>

/**
 * @brief health and latest-sample diagnostics for the wt901 comparison rail.
 *
 * The WT901 is mainly a comparison/reference IMU. Its outputs are kept in the
 * same body-frame convention as the rest of the system so logs can catch frame
 * sign errors, even if the rail is not selected for control.
 */
struct Wt901Diagnostics {
    // Driver startup state. This can be true even if the latest data is stale.
    bool initialized = false;
    // Product availability and freshness for the latest decoded serial frame.
    bool hasAccel = false;
    bool hasGyro = false;
    bool hasYpr = false;
    bool hasQuaternion = false;
    // True when the last acquire consumed a new UART frame.
    bool lastAcquireFresh = false;
    // True when the last acquire returned recent cached data for diagnostics.
    bool lastAcquireUsedCache = false;
    // Timestamp of the latest decoded WT901 register update.
    uint32_t lastSampleMicros = 0;
    // Body-frame channels used for replay plots and cross-rail comparisons.
    float accelBodyMps2[3] = {0.0f, 0.0f, 0.0f};
    float gyroBodyRadPerSec[3] = {0.0f, 0.0f, 0.0f};
    float yprDeg[3] = {0.0f, 0.0f, 0.0f};
    float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
};

/// @brief initializes the wt901 serial imu path.
bool Wt901SensorBegin();
/// @brief acquires and caches the latest wt901 sample.
bool Wt901SensorAcquire();
/// @brief returns true once the wt901 path is initialized.
bool Wt901SensorIsInitialized();
/// @brief returns the latest wt901 diagnostics.
Wt901Diagnostics Wt901SensorGetDiagnostics();
