#pragma once

#include <Arduino.h>

struct BarometerDiagnostics {
    bool initialized = false;
    bool hasSample = false;
    float altitudeFeet = 0.0f;
    float pressureHpa = 0.0f;
    float temperatureC = 0.0f;
    uint32_t lastReadDurationUs = 0;
    uint32_t averageReadDurationUs = 0;
    uint32_t averageUpdatePeriodUs = 0;
    uint32_t lastUpdateMicros = 0;
};
