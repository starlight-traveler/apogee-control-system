#include "ms5611_sensor.h"

#include <Arduino.h>
#include <SPI.h>
#include <cmath>

#include "MS5611_SPI.h"
#include "serial_logging.h"
#include "settings.h"

namespace {

MS5611_SPI g_pressureSensor(settings::sensors::ms5611::kChipSelectPin, &SPI);

constexpr float kMaxValidAltitudeFeet = settings::sensors::ms5611::kMaxValidAltitudeFeet;
constexpr uint32_t kMinReadSpacingUs = settings::sensors::ms5611::kMinReadSpacingUs;
constexpr osr_t kOversampling = OSR_ULTRA_LOW;

bool g_initialized = false;
bool g_hasSample = false;
float g_seaLevelPressureHpa = settings::sensors::ms5611::kSeaLevelPressureHpa;
float g_seaLevelPressureInv = 1.0f / settings::sensors::ms5611::kSeaLevelPressureHpa;

float g_lastAltitudeFeet = 0.0f;
float g_lastPressureHpa = 0.0f;
float g_lastTemperatureC = 0.0f;
uint32_t g_lastReadMicros = 0;
uint32_t g_lastUpdateMicros = 0;
uint32_t g_lastReadDurationUs = 0;
uint32_t g_averageReadDurationUs = 0;
uint32_t g_averageUpdatePeriodUs = 0;

/// Updates an exponentially weighted timing average.
void UpdateAverage(uint32_t sample, uint32_t &average) {
    if (sample == 0) {
        return;
    }
    if (average == 0) {
        average = sample;
        return;
    }
    average = (average * 7u + sample) / 8u;
}

/// Converts pressure to altitude in feet using the MS5611 sea-level reference.
float ComputeAltitudeFeet(float pressureHpa) {
    const float ratio = pressureHpa * g_seaLevelPressureInv;
    if (!(ratio > 0.0f)) {
        return 0.0f;
    }
    const float altitudeMeters = 44330.0f * (1.0f - std::pow(ratio, 0.190294957f));
    return altitudeMeters * 3.28083989501312f;
}

}  // namespace

/// Initializes the MS5611 over SPI.
bool Ms5611SensorBegin() {
    if (g_initialized) {
        return true;
    }

    SPI.begin();
    if (!g_pressureSensor.begin()) {
        LOG_PRINTLN("Error: MS5611 not connected over SPI.");
        return false;
    }
    g_pressureSensor.setOversampling(kOversampling);

    LOG_PRINT("MS5611 connected over SPI (CS=");
    LOG_PRINT(settings::sensors::ms5611::kChipSelectPin);
    LOG_PRINT(") OSR=");
    LOG_PRINTLN(static_cast<uint8_t>(g_pressureSensor.getOversampling()));

    g_hasSample = false;
    g_lastReadMicros = 0;
    g_lastUpdateMicros = 0;
    g_lastReadDurationUs = 0;
    g_averageReadDurationUs = 0;
    g_averageUpdatePeriodUs = 0;

    g_initialized = true;
    return true;
}

/// Acquires one paced MS5611 sample and updates cached diagnostics.
bool Ms5611SensorAcquire() {
    if (!g_initialized) {
        return false;
    }

    const uint32_t nowMicros = micros();
    if (g_lastReadMicros != 0 && static_cast<uint32_t>(nowMicros - g_lastReadMicros) < kMinReadSpacingUs) {
        return false;
    }
    g_lastReadMicros = nowMicros;

    const uint32_t startMicros = micros();
    const int result = g_pressureSensor.read();
    const uint32_t stopMicros = micros();
    g_lastReadDurationUs = stopMicros - startMicros;
    UpdateAverage(g_lastReadDurationUs, g_averageReadDurationUs);

    if (result != 0) {
        return false;
    }

    const float pressureHpa = g_pressureSensor.getPressure();
    const float temperatureC = g_pressureSensor.getTemperature();
    if (!(pressureHpa > 0.0f) || !isfinite(pressureHpa)) {
        return false;
    }

    const float altitudeFeet = ComputeAltitudeFeet(pressureHpa);
    if (!isfinite(altitudeFeet) || std::fabs(altitudeFeet) > kMaxValidAltitudeFeet) {
        return false;
    }

    const uint32_t updateMicros = micros();
    if (g_lastUpdateMicros != 0) {
        UpdateAverage(updateMicros - g_lastUpdateMicros, g_averageUpdatePeriodUs);
    }
    g_lastUpdateMicros = updateMicros;
    g_lastAltitudeFeet = altitudeFeet;
    g_lastPressureHpa = pressureHpa;
    g_lastTemperatureC = temperatureC;
    g_hasSample = true;
    return true;
}

/// Returns cached MS5611 diagnostics for comparison against the BMP585 path.
BarometerDiagnostics Ms5611SensorGetDiagnostics() {
    BarometerDiagnostics diagnostics;
    diagnostics.initialized = g_initialized;
    diagnostics.hasSample = g_hasSample;
    diagnostics.altitudeFeet = g_lastAltitudeFeet;
    diagnostics.pressureHpa = g_lastPressureHpa;
    diagnostics.temperatureC = g_lastTemperatureC;
    diagnostics.lastReadDurationUs = g_lastReadDurationUs;
    diagnostics.averageReadDurationUs = g_averageReadDurationUs;
    diagnostics.averageUpdatePeriodUs = g_averageUpdatePeriodUs;
    diagnostics.lastUpdateMicros = g_lastUpdateMicros;
    return diagnostics;
}

void Ms5611SensorSetSeaLevelPressureHpa(float pressureHpa) {
    if (!(pressureHpa > 0.0f) || !isfinite(pressureHpa)) {
        return;
    }
    g_seaLevelPressureHpa = pressureHpa;
    g_seaLevelPressureInv = 1.0f / pressureHpa;
    if (g_lastPressureHpa > 0.0f) {
        g_lastAltitudeFeet = ComputeAltitudeFeet(g_lastPressureHpa);
    }
}
