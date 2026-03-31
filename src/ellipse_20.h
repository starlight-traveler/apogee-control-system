#pragma once

#include "flight_computer.h"

struct Ellipse20Diagnostics {
    bool initialized = false;
    bool hasImu = false;
    bool hasMag = false;
    bool hasQuaternion = false;
    bool alignmentReady = false;
    bool hasYpr = false;
    bool lastAcquireFresh = false;
    bool lastAcquireUsedCache = false;
    uint32_t lastSampleMicros = 0;
};

bool Ellipse20SensorBegin();
void Ellipse20SensorSetFlightStatus(FlightStatus status);
void Ellipse20SensorSetCrossCheckTrust(float trust);
bool Ellipse20SensorAcquire(SensorData &out);
bool Ellipse20SensorIsInitialized();
Ellipse20Diagnostics Ellipse20SensorGetDiagnostics();
