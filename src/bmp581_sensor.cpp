#include "bmp581_sensor.h"

#include <Arduino.h>
#include <SPI.h>
#include <algorithm>
#include <cmath>

#ifndef ARM_MATH_CM7
#define ARM_MATH_CM7
#endif
#include <arm_math.h>

#include "SparkFun_BMP581_Arduino_Library.h"

namespace {

BMP581 g_pressureSensor;

constexpr uint8_t kChipSelectPin = 10;
constexpr uint8_t kInterruptPin = 9;
constexpr uint32_t kSpiClockHz = 1000000UL;

constexpr float kSeaLevelPressureHpa = 1012.19f;
constexpr float kSeaLevelPressureInv = 1.0f / kSeaLevelPressureHpa;

volatile bool g_interruptFlag = false;
bool g_initialized = false;
bool g_hasSample = false;

float g_lastAltitudeFeet = 0.0f;
float g_lastPressureHpa = 0.0f;
float g_lastTemperatureC = 0.0f;
float g_lastTimestamp = 0.0f;

void InterruptHandler() {
    g_interruptFlag = true;
}

bool ConfigureSensor() {
    if (g_pressureSensor.setODRFrequency(BMP5_ODR_140_HZ) != BMP5_OK) {
        return false;
    }

    BMP581_InterruptConfig config{};
    config.enable = BMP5_INTR_ENABLE;
    config.drive = BMP5_INTR_PUSH_PULL;
    config.polarity = BMP5_ACTIVE_HIGH;
    config.mode = BMP5_PULSED;
    config.sources.drdy_en = BMP5_ENABLE;
    config.sources.fifo_full_en = BMP5_DISABLE;
    config.sources.fifo_thres_en = BMP5_DISABLE;
    config.sources.oor_press_en = BMP5_DISABLE;

    if (g_pressureSensor.setInterruptConfig(&config) != BMP5_OK) {
        return false;
    }

    bmp5_sensor_data throwaway{};
    if (g_pressureSensor.getSensorData(&throwaway) != BMP5_OK) {
        return false;
    }

    return true;
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
    g_lastPressureHpa = pressureHpa;
    g_lastTemperatureC = temperatureC;
    g_lastAltitudeFeet = ComputeAltitudeFeet(pressureHpa);
    g_lastTimestamp = static_cast<float>(micros()) * 1.0e-6f;
    g_hasSample = true;
}

}  // namespace

bool Bmp581SensorBegin() {
    if (g_initialized) {
        return true;
    }

    SPI.begin();

    if (g_pressureSensor.beginSPI(kChipSelectPin, kSpiClockHz) != BMP5_OK) {
        return false;
    }

    if (!ConfigureSensor()) {
        return false;
    }

    pinMode(kInterruptPin, INPUT);
    g_interruptFlag = false;
    g_hasSample = false;
    attachInterrupt(digitalPinToInterrupt(kInterruptPin), InterruptHandler, RISING);

    g_initialized = true;
    return true;
}

bool Bmp581SensorAcquire(SensorData &out) {
    if (g_hasSample) {
        out.altitudeFeet = g_lastAltitudeFeet;
        if (out.timestamp == 0.0f && g_lastTimestamp > 0.0f) {
            out.timestamp = g_lastTimestamp;
        }
    }

    if (!g_initialized) {
        return false;
    }

    bool shouldRead = false;
    noInterrupts();
    if (g_interruptFlag) {
        g_interruptFlag = false;
        shouldRead = true;
    }
    interrupts();

    if (!shouldRead) {
        return false;
    }

    uint8_t status = 0;
    if (g_pressureSensor.getInterruptStatus(&status) != BMP5_OK) {
        return false;
    }

    if ((status & BMP5_INT_ASSERTED_DRDY) == 0) {
        return false;
    }

    bmp5_sensor_data sample{};
    if (g_pressureSensor.getSensorData(&sample) != BMP5_OK) {
        return false;
    }

    const float pressureHpa = sample.pressure * 0.01f;
    UpdateCachedSample(pressureHpa, sample.temperature);

    out.altitudeFeet = g_lastAltitudeFeet;
    if (out.timestamp == 0.0f) {
        out.timestamp = g_lastTimestamp;
    }

    return true;
}
