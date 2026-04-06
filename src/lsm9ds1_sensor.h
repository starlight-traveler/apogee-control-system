#pragma once

#include "flight_computer.h"

struct Lsm9ds1Diagnostics {
    bool initialized = false;
    bool hasAccel = false;
    bool hasGyro = false;
    bool hasMag = false;
    bool hasQuaternion = false;
    bool hasBootstrapYpr = false;
    bool alignmentReady = false;
    bool lastAcquireFresh = false;
    bool lastAcquireUsedCache = false;
    bool interruptConfigured = false;
    bool lastAcquireUsedInterrupt = false;
    bool fifoEnabled = false;
    uint32_t lastSampleMicros = 0;
    uint16_t groundAlignmentSampleCount = 0;
    float lastAccelTrust = 0.0f;
    float lastMagTrust = 0.0f;
    float lastAccelMagnitudeG = 0.0f;
    float lastMagMagnitude = 0.0f;
    float magReferenceNorm = 0.0f;
    float yprDeg[3] = {0.0f, 0.0f, 0.0f};
    float bootstrapYprDeg[3] = {0.0f, 0.0f, 0.0f};
    float accelBodyMps2[3] = {0.0f, 0.0f, 0.0f};
    float magBody[3] = {0.0f, 0.0f, 0.0f};
};

bool Lsm9ds1SensorBegin();
bool Lsm9ds1SensorIsInitialized();
void Lsm9ds1SensorSetFlightStatus(FlightStatus status);
void Lsm9ds1SensorSetBurnoutTimestamp(float burnoutTimeSeconds);
void Lsm9ds1SensorSetCurrentTimestamp(float currentTimeSeconds);
void Lsm9ds1SensorSetCrossCheckTrust(float trust);
bool Lsm9ds1SensorAcquire(SensorData &out);
Lsm9ds1Diagnostics Lsm9ds1SensorGetDiagnostics();
