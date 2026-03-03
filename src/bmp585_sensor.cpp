#include "bmp585_sensor.h"

#include <Arduino.h>
#include <SPI.h>
#include <algorithm>
#include <cmath>

#ifndef ARM_MATH_CM7
#define ARM_MATH_CM7
#endif
#include <arm_math.h>

#include "Adafruit_BMP5xx.h"
#include "serial_logging.h"
#include "settings.h"

namespace {

Adafruit_BMP5xx g_pressureSensor;

constexpr uint8_t kChipSelectPin = 35;
constexpr uint32_t kSampleIntervalUs = 20000UL;

constexpr float kSeaLevelPressureHpa = settings::sensors::bmp585::kSeaLevelPressureHpa;
constexpr float kSeaLevelPressureInv = 1.0f / kSeaLevelPressureHpa;
constexpr float kMaxAltitudeRateFeetPerSecond = settings::sensors::bmp585::kMaxAltitudeRateFeetPerSecond;
constexpr float kMinSpikeJumpFeet = settings::sensors::bmp585::kMinSpikeJumpFeet;
constexpr float kMaxValidAltitudeFeet = settings::sensors::bmp585::kMaxValidAltitudeFeet;

bool g_initialized = false;
bool g_hasSample = false;

float g_lastAltitudeFeet = 0.0f;
float g_lastPressureHpa = 0.0f;
float g_lastTemperatureC = 0.0f;
float g_lastTimestamp = 0.0f;
uint32_t g_lastReadMicros = 0;
uint32_t g_lastUpdateMicros = 0;
uint32_t g_lastReadDurationUs = 0;
uint32_t g_averageReadDurationUs = 0;
uint32_t g_averageUpdatePeriodUs = 0;

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

bool ConfigureSensor() {
    return g_pressureSensor.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X) &&
           g_pressureSensor.setPressureOversampling(BMP5XX_OVERSAMPLING_16X) &&
           g_pressureSensor.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3) &&
           g_pressureSensor.setOutputDataRate(BMP5XX_ODR_50_HZ) &&
           g_pressureSensor.setPowerMode(BMP5XX_POWERMODE_NORMAL);
}

float ComputeAltitudeFeet(float pressureHpa) {
    const float32_t ratio = std::max(pressureHpa * kSeaLevelPressureInv, 1.0e-6f);
    const float32_t powTerm = static_cast<float32_t>(std::pow(static_cast<double>(ratio), 0.190294957));

    float32_t buffer[1] = {powTerm};
    arm_offset_f32(buffer, -1.0f, buffer, 1);
    arm_negate_f32(buffer, buffer, 1);

    float32_t altitudeMeters[1];
    arm_scale_f32(buffer, 44330.0f, altitudeMeters, 1);

    float32_t altitudeFeet[1];
    arm_scale_f32(altitudeMeters, 3.28083989501312f, altitudeFeet, 1);

    return altitudeFeet[0];
}

void UpdateCachedSample(float pressureHpa, float temperatureC) {
    const uint32_t nowMicros = micros();
    if (g_lastUpdateMicros != 0) {
        UpdateAverage(nowMicros - g_lastUpdateMicros, g_averageUpdatePeriodUs);
    }
    g_lastUpdateMicros = nowMicros;
    g_lastPressureHpa = pressureHpa;
    g_lastTemperatureC = temperatureC;
    g_lastAltitudeFeet = ComputeAltitudeFeet(pressureHpa);
    g_lastTimestamp = static_cast<float>(nowMicros) * 1.0e-6f;
    g_hasSample = true;
}

bool IsAltitudeSpike(float candidateAltitudeFeet, float timestampSeconds) {
    if (!isfinite(candidateAltitudeFeet)) {
        return true;
    }
    if (std::fabs(candidateAltitudeFeet) > kMaxValidAltitudeFeet) {
        return true;
    }
    if (!g_hasSample) {
        return false;
    }

    float dt = timestampSeconds - g_lastTimestamp;
    if (dt < 0.0f) {
        dt = 0.0f;
    }
    const float allowedJumpFeet = std::max(kMinSpikeJumpFeet, kMaxAltitudeRateFeetPerSecond * dt);
    const float jumpFeet = std::fabs(candidateAltitudeFeet - g_lastAltitudeFeet);
    return jumpFeet > allowedJumpFeet;
}

}  // namespace

bool Bmp585SensorBegin() {
    if (g_initialized) {
        return true;
    }

    SPI.begin();

    while (!g_pressureSensor.begin(kChipSelectPin, &SPI)) {
        LOG_PRINTLN("Error: BMP585 not connected over SPI, check wiring and CS pin.");
        delay(1000);
    }

    LOG_PRINTLN("BMP585 connected!");

    if (!ConfigureSensor()) {
        LOG_PRINTLN("Error: BMP585 configuration failed.");
        return false;
    }

    g_hasSample = false;
    g_lastReadMicros = 0;
    g_lastUpdateMicros = 0;
    g_lastReadDurationUs = 0;
    g_averageReadDurationUs = 0;
    g_averageUpdatePeriodUs = 0;

    g_initialized = true;
    return true;
}

bool Bmp585SensorAcquire(SensorData &out) {
    if (g_hasSample) {
        out.altitudeFeet = g_lastAltitudeFeet;
        if (out.timestamp == 0.0f && g_lastTimestamp > 0.0f) {
            out.timestamp = g_lastTimestamp;
        }
    }

    if (!g_initialized) {
        return false;
    }

    const uint32_t nowMicros = micros();
    if (g_lastReadMicros != 0 && static_cast<uint32_t>(nowMicros - g_lastReadMicros) < kSampleIntervalUs) {
        return false;
    }
    g_lastReadMicros = nowMicros;

    const uint32_t startMicros = micros();
    if (!g_pressureSensor.performReading()) {
        return false;
    }
    g_lastReadDurationUs = micros() - startMicros;
    UpdateAverage(g_lastReadDurationUs, g_averageReadDurationUs);

    const float pressureHpa = g_pressureSensor.pressure;
    if (!(pressureHpa > 0.0f) || !isfinite(pressureHpa)) {
        return false;
    }
    const float timestampSeconds = static_cast<float>(micros()) * 1.0e-6f;
    const float altitudeFeet = ComputeAltitudeFeet(pressureHpa);
    if (IsAltitudeSpike(altitudeFeet, timestampSeconds)) {
        out.altitudeFeet = g_lastAltitudeFeet;
        if (out.timestamp == 0.0f && g_lastTimestamp > 0.0f) {
            out.timestamp = g_lastTimestamp;
        }
        return true;
    }

    UpdateCachedSample(pressureHpa, g_pressureSensor.temperature);

    out.altitudeFeet = g_lastAltitudeFeet;
    if (out.timestamp == 0.0f) {
        out.timestamp = g_lastTimestamp;
    }

    return true;
}

BarometerDiagnostics Bmp585SensorGetDiagnostics() {
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
