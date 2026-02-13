#include "bno055_sensor.h"

#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "bno055_orientation.h"

namespace {

constexpr uint8_t kBnoI2cAddress = 0x28;
constexpr uint32_t kSampleIntervalUs = 10000;

Adafruit_BNO055 g_bno(55, kBnoI2cAddress);
bool g_initialized = false;
uint32_t g_lastSampleUs = 0;

}

bool Bno055SensorBegin() {
    if (g_initialized) {
        return true;
    }

    Wire.begin();
    Wire.setClock(400000);
    if (!g_bno.begin()) {
        return false;
    }

    delay(10);
    g_bno.setExtCrystalUse(true);

    g_lastSampleUs = 0;
    g_initialized = true;
    return true;
}

bool Bno055SensorAcquire(SensorData &out) {
    if (!g_initialized) {
        return false;
    }

    const uint32_t nowUs = micros();
    if (g_lastSampleUs != 0 && (nowUs - g_lastSampleUs) < kSampleIntervalUs) {
        return false;
    }
    g_lastSampleUs = nowUs;

    out.timestamp = static_cast<float>(nowUs) * 1.0e-6f;

    const imu::Vector<3> rawAccel = g_bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    float accelX;
    float accelY;
    float accelZ;
    bno055_orientation::TransformVector(rawAccel.x(), rawAccel.y(), rawAccel.z(), accelX, accelY, accelZ);
    out.accelBNO[0] = accelX;
    out.accelBNO[1] = accelY;
    out.accelBNO[2] = accelZ;
    out.accelICM[0] = accelX;
    out.accelICM[1] = accelY;
    out.accelICM[2] = accelZ;

    const imu::Vector<3> rawGyro = g_bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    float gyroX;
    float gyroY;
    float gyroZ;
    bno055_orientation::TransformVector(rawGyro.x(), rawGyro.y(), rawGyro.z(), gyroX, gyroY, gyroZ);
    out.gyro[0] = gyroX;
    out.gyro[1] = gyroY;
    out.gyro[2] = gyroZ;

    const imu::Quaternion rawQuat = g_bno.getQuat();
    float adjustedQuat[4];
    bno055_orientation::AdjustQuaternion(rawQuat.w(), rawQuat.x(), rawQuat.y(), rawQuat.z(), adjustedQuat);
    out.quaternion[0] = adjustedQuat[0];
    out.quaternion[1] = adjustedQuat[1];
    out.quaternion[2] = adjustedQuat[2];
    out.quaternion[3] = adjustedQuat[3];
    out.hasQuaternion = true;

    return true;
}
