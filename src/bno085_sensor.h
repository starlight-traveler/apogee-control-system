#pragma once

#include "flight_computer.h"

struct Bno085Diagnostics {
    bool transportReady = false;
    bool hasAccel = false;
    bool hasGyro = false;
    bool hasMag = false;
    bool hasQuaternion = false;
    bool hasBootstrapYpr = false;
    bool lastAcquireFresh = false;
    float yprDeg[3] = {0.0f, 0.0f, 0.0f};
    float bootstrapYprDeg[3] = {0.0f, 0.0f, 0.0f};
    float accelBodyMps2[3] = {0.0f, 0.0f, 0.0f};
    float magBody[3] = {0.0f, 0.0f, 0.0f};
};

struct Bno085Sample {
    bool hasAccel = false;
    bool hasGyro = false;
    bool hasQuaternion = false;
    uint32_t sampleMicros = 0;
    float accel[3] = {0.0f, 0.0f, 0.0f};
    float gyro[3] = {0.0f, 0.0f, 0.0f};
    float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
};

/// Initializes the BNO085 orientation/IMU path.
bool Bno085SensorBegin();
/// Acquires the latest BNO085 IMU sample into `SensorData`.
bool Bno085SensorAcquire(SensorData &out);
/// Returns true once the BNO085 path is initialized and healthy enough to serve data.
bool Bno085SensorIsInitialized();
/// Returns the current transport and per-report health of the BNO085 path.
Bno085Diagnostics Bno085SensorGetDiagnostics();
/// Returns the latest cached BNO085 sample in body-frame coordinates.
Bno085Sample Bno085SensorGetSample();
