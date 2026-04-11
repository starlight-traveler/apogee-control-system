#pragma once

#include "flight_computer.h"

struct BnoDiagnostics {
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

struct BnoSample {
    bool hasAccel = false;
    bool hasGyro = false;
    bool hasQuaternion = false;
    uint32_t sampleMicros = 0;
    uint32_t accelMicros = 0;
    uint32_t gyroMicros = 0;
    uint32_t quaternionMicros = 0;
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
/// Acquires the latest BNO-family IMU sample into `SensorData`.
bool BnoSensorAcquire(SensorData &out);
/// Returns true once the selected BNO-family device is initialized.
bool BnoSensorIsInitialized();
/// Returns the latest generic diagnostics for the selected BNO-family device.
BnoDiagnostics BnoSensorGetDiagnostics();
/// Returns the latest generic sample for the selected BNO-family device.
BnoSample BnoSensorGetSample();
