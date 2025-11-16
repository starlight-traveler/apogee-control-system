#include "bno085_sensor.h"

#include <Arduino.h>
#include <SPI.h>
#include "SparkFun_BNO080_Arduino_Library.h"

namespace {

BNO080 g_bno;
constexpr uint8_t kImuCSPin = 36;
constexpr uint8_t kImuWakPin = 32;
constexpr uint8_t kImuIntPin = 33;
constexpr uint8_t kImuRstPin = 35;

volatile bool g_intFlag = false;
bool g_initialized = false;

void ImuIntISR() {
    g_intFlag = true;
}

void ConfigureReports() {
    constexpr uint16_t intervalMs = 5;
    g_bno.enableAccelerometer(intervalMs);
    g_bno.enableGyro(intervalMs);
    g_bno.enableRotationVector(intervalMs);
}

}

bool Bno085SensorBegin() {
    if (g_initialized) {
        return true;
    }

    pinMode(kImuIntPin, INPUT_PULLUP);

    if (!g_bno.beginSPI(kImuCSPin, kImuWakPin, kImuIntPin, kImuRstPin, 30000000UL, SPI1)) {
        return false;
    }

    ConfigureReports();

    attachInterrupt(digitalPinToInterrupt(kImuIntPin), ImuIntISR, FALLING);

    g_initialized = true;
    return true;
}

bool Bno085SensorAcquire(SensorData &out) {
    if (!g_initialized) {
        return false;
    }

    bool shouldRead = false;
    noInterrupts();
    if (g_intFlag) {
        g_intFlag = false;
        shouldRead = true;
    }
    interrupts();

    if (!shouldRead && !g_bno.dataAvailable()) {
        return false;
    }

    bool hasSample = false;
    while (g_bno.dataAvailable()) {
        hasSample = true;

        out.timestamp = static_cast<float>(micros()) * 1.0e-6f;
        out.accelBNO[0] = g_bno.getAccelX();
        out.accelBNO[1] = g_bno.getAccelY();
        out.accelBNO[2] = g_bno.getAccelZ();
        out.accelICM[0] = g_bno.getAccelX();
        out.accelICM[1] = g_bno.getAccelY();
        out.accelICM[2] = g_bno.getAccelZ();
        out.gyro[0] = g_bno.getGyroX();
        out.gyro[1] = g_bno.getGyroY();
        out.gyro[2] = g_bno.getGyroZ();
        out.quaternion[0] = g_bno.getQuatReal();
        out.quaternion[1] = g_bno.getQuatI();
        out.quaternion[2] = g_bno.getQuatJ();
        out.quaternion[3] = g_bno.getQuatK();
        out.hasQuaternion = true;
    }

    return hasSample;
}

