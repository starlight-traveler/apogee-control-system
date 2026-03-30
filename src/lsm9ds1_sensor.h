#pragma once

#include "flight_computer.h"

struct Lsm9ds1Diagnostics {
    bool initialized = false;
    bool hasAccel = false;
    bool hasGyro = false;
    bool hasQuaternion = false;
    bool alignmentReady = false;
    bool lastAcquireFresh = false;
    bool lastAcquireUsedCache = false;
    bool interruptConfigured = false;
    bool lastAcquireUsedInterrupt = false;
    bool fifoEnabled = false;
    uint32_t lastSampleMicros = 0;
};

bool Lsm9ds1SensorBegin();
bool Lsm9ds1SensorIsInitialized();
void Lsm9ds1SensorSetFlightStatus(FlightStatus status);
void Lsm9ds1SensorSetCrossCheckTrust(float trust);
bool Lsm9ds1SensorAcquire(SensorData &out);
Lsm9ds1Diagnostics Lsm9ds1SensorGetDiagnostics();
