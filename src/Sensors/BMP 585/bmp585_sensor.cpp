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

/*
 * Barometer design rule:
 *
 * A pressure altitude sample is either fresh or it is not. Cached altitude is
 * useful for diagnostics/log continuity, but phase detection and actuation safety
 * should only respond to loops where the driver actually read pressure hardware.
 */

Adafruit_BMP5xx g_pressureSensor;

// The BMP585 is the primary barometer path. It is paced explicitly because the
// estimator treats a returned sample as fresh pressure altitude.
constexpr uint8_t kChipSelectPin = 35;
constexpr uint32_t kSampleIntervalUs = 10000UL;
constexpr float kMaxAltitudeRateFeetPerSecond = settings::sensors::bmp585::kMaxAltitudeRateFeetPerSecond;
constexpr float kMinSpikeJumpFeet = settings::sensors::bmp585::kMinSpikeJumpFeet;
constexpr float kMaxValidAltitudeFeet = settings::sensors::bmp585::kMaxValidAltitudeFeet;
bool g_initialized = false;
bool g_hasSample = false;
float g_seaLevelPressureHpa = settings::sensors::bmp585::kSeaLevelPressureHpa;
float g_seaLevelPressureInv = 1.0f / settings::sensors::bmp585::kSeaLevelPressureHpa;

float g_lastAltitudeFeet = 0.0f;
float g_lastPressureHpa = 0.0f;
float g_lastTemperatureC = 0.0f;
float g_lastTimestamp = 0.0f;
uint32_t g_lastReadMicros = 0;
uint32_t g_lastUpdateMicros = 0;
uint32_t g_lastReadDurationUs = 0;
uint32_t g_averageReadDurationUs = 0;
uint32_t g_averageUpdatePeriodUs = 0;

/// Updates an exponentially weighted average used for sensor timing diagnostics.
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

/// Applies the BMP585 oversampling/filter/output-rate configuration.
bool ConfigureSensor() {
    return g_pressureSensor.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X) &&
           g_pressureSensor.setPressureOversampling(BMP5XX_OVERSAMPLING_8X) &&
           g_pressureSensor.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3) &&
           g_pressureSensor.setOutputDataRate(BMP5XX_ODR_100_2_HZ) &&
           g_pressureSensor.setPowerMode(BMP5XX_POWERMODE_NORMAL);
}

/// Converts pressure to altitude in feet using the configured sea-level reference.
float ComputeAltitudeFeet(float pressureHpa) {
    // Barometric altitude from pressure ratio. The exponent is the standard
    // atmosphere pressure-to-height relationship for the troposphere.
    const float32_t ratio = std::max(pressureHpa * g_seaLevelPressureInv, 1.0e-6f);
    const float32_t powTerm = static_cast<float32_t>(std::pow(static_cast<double>(ratio), 0.190294957));

    // The CMSIS calls below are simple arithmetic in vector form. They are kept
    // here because this path runs often and the Teensy has optimized ARM math.
    float32_t buffer[1] = {powTerm};
    arm_offset_f32(buffer, -1.0f, buffer, 1);
    arm_negate_f32(buffer, buffer, 1);

    float32_t altitudeMeters[1];
    arm_scale_f32(buffer, 44330.0f, altitudeMeters, 1);

    float32_t altitudeFeet[1];
    arm_scale_f32(altitudeMeters, 3.28083989501312f, altitudeFeet, 1);

    return altitudeFeet[0];
}

/// Caches the latest accepted sample and updates timing diagnostics.
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

/// Rejects implausible single-sample altitude jumps before they reach the estimator.
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
    // Allow bigger jumps as time since the last accepted sample grows, but
    // reject single pressure glitches that would imply impossible vertical speed.
    return jumpFeet > allowedJumpFeet;
}

}  // namespace

/// Initializes and configures the BMP585.
bool Bmp585SensorBegin() {
    if (g_initialized) {
        return true;
    }

    SPI.begin();

    if (!g_pressureSensor.begin(kChipSelectPin, &SPI)) {
        LOG_PRINTLN("Error: BMP585 not connected over SPI, check wiring and CS pin.");
        return false;
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

/// Returns the latest BMP585 sample, subject to read pacing and spike rejection.
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
        // Return false rather than reusing the cache so callers can tell this
        // loop did not have a fresh barometer measurement.
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
        // Keep the previous cached altitude available for logging, but do not
        // mark this acquisition fresh.
        out.altitudeFeet = g_lastAltitudeFeet;
        if (out.timestamp == 0.0f && g_lastTimestamp > 0.0f) {
            out.timestamp = g_lastTimestamp;
        }
        return false;
    }

    UpdateCachedSample(pressureHpa, g_pressureSensor.temperature);

    out.altitudeFeet = g_lastAltitudeFeet;
    if (out.timestamp == 0.0f) {
        out.timestamp = g_lastTimestamp;
    }

    return true;
}

/// Returns true once the BMP585 transport/configuration completed successfully.
bool Bmp585SensorIsInitialized() {
    return g_initialized;
}

/// Returns cached BMP585 diagnostics for logging and field debugging.
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

void Bmp585SensorSetSeaLevelPressureHpa(float pressureHpa) {
    /*
     * Changing sea-level pressure changes the altitude frame but not the measured
     * pressure. Reprojecting the cached pressure keeps telemetry consistent with
     * the currently active runtime setting.
     */
    if (!(pressureHpa > 0.0f) || !isfinite(pressureHpa)) {
        return;
    }
    g_seaLevelPressureHpa = pressureHpa;
    g_seaLevelPressureInv = 1.0f / pressureHpa;
    if (g_lastPressureHpa > 0.0f) {
        // Recompute cached altitude so telemetry does not keep the old pressure
        // reference after runtime settings change.
        g_lastAltitudeFeet = ComputeAltitudeFeet(g_lastPressureHpa);
    }
}
