#include "bno085_sensor.h"

#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "bno085_orientation.h"
#include "serial_logging.h"
#include "settings.h"

namespace {

constexpr uint32_t kSampleIntervalUs = settings::sensors::bno085::kSampleIntervalUs;
constexpr uint8_t kBnoI2cAddress = settings::sensors::bno085::kI2cAddress;
constexpr int8_t kBnoResetPin = settings::sensors::bno085::kResetPin;
constexpr uint8_t kInitializationAttempts = settings::sensors::bno085::kInitializationAttempts;
constexpr uint32_t kRetryDelayMs = settings::sensors::bno085::kRetryDelayMs;
constexpr uint32_t kDataTimeoutUs = settings::sensors::bno085::kDataTimeoutUs;

Adafruit_BNO055 g_bno(55, kBnoI2cAddress, &Wire);
bool g_initialized = false;
uint32_t g_lastSampleUs = 0;
uint32_t g_lastHealthyEventUs = 0;

float g_lastAccel[3] = {0.0f, 0.0f, 0.0f};
float g_lastGyro[3] = {0.0f, 0.0f, 0.0f};
float g_lastQuat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
bool g_haveAccel = false;
bool g_haveGyro = false;
bool g_haveQuat = false;

void ResetCachedState() {
    g_lastSampleUs = 0;
    g_lastHealthyEventUs = 0;
    g_haveAccel = false;
    g_haveGyro = false;
    g_haveQuat = false;
}

bool StartSensorTransport() {
    Wire.begin();
    if (!g_bno.begin()) {
        return false;
    }
    if (kBnoResetPin >= 0) {
        pinMode(kBnoResetPin, OUTPUT);
        digitalWrite(kBnoResetPin, HIGH);
    }
    g_bno.setExtCrystalUse(false);
    delay(10);
    return true;
}

bool RecoverSensor(const char *reason) {
    g_initialized = false;
    ResetCachedState();

    if (reason != nullptr) {
        LOG_PRINT("BNO085 recovery: ");
        LOG_PRINTLN(reason);
    }

    for (uint8_t attempt = 0; attempt < kInitializationAttempts; ++attempt) {
        if (StartSensorTransport()) {
            g_initialized = true;
            LOG_PRINT("BNO055 online after attempt ");
            LOG_PRINTLN(static_cast<unsigned>(attempt + 1));
            return true;
        }

        LOG_PRINT("BNO055 init retry ");
        LOG_PRINT(static_cast<unsigned>(attempt + 1));
        LOG_PRINT("/");
        LOG_PRINTLN(static_cast<unsigned>(kInitializationAttempts));
        delay(kRetryDelayMs * (attempt + 1));
    }
    return false;
}

void PopulateOutput(SensorData &out, uint32_t nowUs) {
    out.timestamp = static_cast<float>(nowUs) * 1.0e-6f;
    out.accelBNO[0] = g_haveAccel ? g_lastAccel[0] : 0.0f;
    out.accelBNO[1] = g_haveAccel ? g_lastAccel[1] : 0.0f;
    out.accelBNO[2] = g_haveAccel ? g_lastAccel[2] : 0.0f;
    out.gyro[0] = g_haveGyro ? g_lastGyro[0] : 0.0f;
    out.gyro[1] = g_haveGyro ? g_lastGyro[1] : 0.0f;
    out.gyro[2] = g_haveGyro ? g_lastGyro[2] : 0.0f;
    out.quaternion[0] = g_lastQuat[0];
    out.quaternion[1] = g_lastQuat[1];
    out.quaternion[2] = g_lastQuat[2];
    out.quaternion[3] = g_lastQuat[3];
    out.hasQuaternion = g_haveQuat;
}

}  // namespace

bool Bno085SensorBegin() {
    if (g_initialized) {
        return true;
    }

    return RecoverSensor("startup");
}

bool Bno085SensorAcquire(SensorData &out) {
    if (!g_initialized) {
        return false;
    }

    const uint32_t nowUs = micros();
    if (g_lastHealthyEventUs != 0 && (nowUs - g_lastHealthyEventUs) > kDataTimeoutUs) {
        return RecoverSensor("data timeout");
    }
    if (g_lastSampleUs != 0 && (nowUs - g_lastSampleUs) < kSampleIntervalUs) {
        if (!g_haveQuat) {
            return false;
        }
        PopulateOutput(out, nowUs);
        return true;
    }
    g_lastSampleUs = nowUs;

    imu::Vector<3> accel = g_bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro = g_bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Quaternion quat = g_bno.getQuat();

    if (quat.w() == 0.0f && quat.x() == 0.0f && quat.y() == 0.0f && quat.z() == 0.0f) {
        if (!g_haveQuat) {
            return false;
        }
        PopulateOutput(out, nowUs);
        return true;
    }

    bno085_orientation::TransformVector(accel.x(), accel.y(), accel.z(),
                                        g_lastAccel[0], g_lastAccel[1], g_lastAccel[2]);
    bno085_orientation::TransformVector(gyro.x(), gyro.y(), gyro.z(),
                                        g_lastGyro[0], g_lastGyro[1], g_lastGyro[2]);
    bno085_orientation::AdjustQuaternion(quat.w(), quat.x(), quat.y(), quat.z(), g_lastQuat);
    g_haveAccel = true;
    g_haveGyro = true;
    g_haveQuat = true;
    g_lastHealthyEventUs = nowUs;

    PopulateOutput(out, nowUs);
    return true;
}
