#pragma once

#include <Arduino.h>

struct Wt901Diagnostics {
    bool initialized = false;
    bool hasAccel = false;
    bool hasGyro = false;
    bool hasYpr = false;
    bool hasQuaternion = false;
    bool lastAcquireFresh = false;
    bool lastAcquireUsedCache = false;
    uint32_t lastSampleMicros = 0;
    float accelBodyMps2[3] = {0.0f, 0.0f, 0.0f};
    float gyroBodyRadPerSec[3] = {0.0f, 0.0f, 0.0f};
    float yprDeg[3] = {0.0f, 0.0f, 0.0f};
    float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
};

bool Wt901SensorBegin();
bool Wt901SensorAcquire();
bool Wt901SensorIsInitialized();
Wt901Diagnostics Wt901SensorGetDiagnostics();
