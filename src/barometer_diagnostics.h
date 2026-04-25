#pragma once

#include <Arduino.h>

/**
 * @brief shared latest-sample and timing diagnostics for a barometer.
 *
 * this is what the rest of the firmware reads when it wants to know whether a
 * barometer is alive, what it last measured, and how fast it is updating.
 */
struct BarometerDiagnostics {
    bool initialized = false;
    // True after at least one pressure sample has been accepted.
    bool hasSample = false;
    // Latest accepted physical values, not necessarily a fresh sample this loop.
    float altitudeFeet = 0.0f;
    float pressureHpa = 0.0f;
    float temperatureC = 0.0f;
    // Timing fields are used to spot slow reads and update-rate dropouts.
    uint32_t lastReadDurationUs = 0;
    uint32_t averageReadDurationUs = 0;
    uint32_t averageUpdatePeriodUs = 0;
    // Timestamp of the latest accepted sample in micros().
    uint32_t lastUpdateMicros = 0;
};
