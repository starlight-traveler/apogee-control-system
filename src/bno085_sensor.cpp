#include "bno085_sensor.h"

#include <Arduino.h>
#include <SPI.h>

#include "SparkFun_BNO080_Arduino_Library.h"
#include "bno085_orientation.h"

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

        const float rawAccelX = g_bno.getAccelX();
        const float rawAccelY = g_bno.getAccelY();
        const float rawAccelZ = g_bno.getAccelZ();
        float accelX;
        float accelY;
        float accelZ;
        bno085_orientation::TransformVector(rawAccelX, rawAccelY, rawAccelZ, accelX, accelY, accelZ);
        out.accelBNO[0] = accelX;
        out.accelBNO[1] = accelY;
        out.accelBNO[2] = accelZ;
        out.accelICM[0] = accelX;
        out.accelICM[1] = accelY;
        out.accelICM[2] = accelZ;

        const float rawGyroX = g_bno.getGyroX();
        const float rawGyroY = g_bno.getGyroY();
        const float rawGyroZ = g_bno.getGyroZ();
        float gyroX;
        float gyroY;
        float gyroZ;
        bno085_orientation::TransformVector(rawGyroX, rawGyroY, rawGyroZ, gyroX, gyroY, gyroZ);
        out.gyro[0] = gyroX;
        out.gyro[1] = gyroY;
        out.gyro[2] = gyroZ;

        const float rawQuat[4] = {
            g_bno.getQuatReal(),
            g_bno.getQuatI(),
            g_bno.getQuatJ(),
            g_bno.getQuatK(),
        };
        float adjustedQuat[4];
        bno085_orientation::AdjustQuaternion(rawQuat[0], rawQuat[1], rawQuat[2], rawQuat[3], adjustedQuat);
        out.quaternion[0] = adjustedQuat[0];
        out.quaternion[1] = adjustedQuat[1];
        out.quaternion[2] = adjustedQuat[2];
        out.quaternion[3] = adjustedQuat[3];
        out.hasQuaternion = true;
    }

    return hasSample;
}

