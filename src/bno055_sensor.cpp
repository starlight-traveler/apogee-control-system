#include "bno055_sensor.h"

#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "bno085_orientation.h"
#include "math_utils.h"
#include "serial_logging.h"
#include "settings.h"

namespace {

constexpr uint32_t kSampleIntervalUs = settings::sensors::bno055::kSampleIntervalUs;
constexpr uint8_t kBnoI2cAddress = settings::sensors::bno055::kI2cAddress;
constexpr int8_t kBnoResetPin = settings::sensors::bno055::kResetPin;
constexpr uint32_t kDataTimeoutUs = settings::sensors::bno055::kDataTimeoutUs;
constexpr uint8_t kQuaternionInvalidDropThreshold = 2;

Adafruit_BNO055 g_bno(55, kBnoI2cAddress, &Wire1);
bool g_initialized = false;
uint32_t g_lastSampleUs = 0;
uint32_t g_lastHealthyEventUs = 0;
bool g_lastAcquireFresh = false;

float g_lastAccel[3] = {0.0f, 0.0f, 0.0f};
float g_lastGyro[3] = {0.0f, 0.0f, 0.0f};
float g_lastQuat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
bool g_haveAccel = false;
bool g_haveGyro = false;
bool g_haveQuat = false;
uint8_t g_invalidQuaternionStreak = 0;

void ResetCachedState() {
    g_lastSampleUs = 0;
    g_lastHealthyEventUs = 0;
    g_haveAccel = false;
    g_haveGyro = false;
    g_haveQuat = false;
    g_lastAcquireFresh = false;
    g_invalidQuaternionStreak = 0;
}

bool StartSensorTransport() {
    if (settings::sensors::bno::kTransport != settings::sensors::bno::Transport::I2c) {
        LOG_PRINTLN("BNO055 only supports I2C in this firmware build");
        return false;
    }
    Wire1.begin();
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
        LOG_PRINT("BNO055 recovery: ");
        LOG_PRINTLN(reason);
    }

    if (StartSensorTransport()) {
        g_initialized = true;
        LOG_PRINTLN("BNO055 online via I2C");
        return true;
    }
    LOG_PRINTLN("BNO055 init failed; caller will retry");
    return false;
}

void PopulateOutput(SensorData &out, uint32_t nowUs) {
    float sanitizedQuat[4] = {g_lastQuat[0], g_lastQuat[1], g_lastQuat[2], g_lastQuat[3]};
    const bool quaternionValid =
        g_haveQuat && math_utils::SanitizeQuaternionArray(sanitizedQuat);
    out.accelBNO[0] = g_haveAccel ? g_lastAccel[0] : 0.0f;
    out.accelBNO[1] = g_haveAccel ? g_lastAccel[1] : 0.0f;
    out.accelBNO[2] = g_haveAccel ? g_lastAccel[2] : 0.0f;
    out.gyroBNO[0] = g_haveGyro ? g_lastGyro[0] : 0.0f;
    out.gyroBNO[1] = g_haveGyro ? g_lastGyro[1] : 0.0f;
    out.gyroBNO[2] = g_haveGyro ? g_lastGyro[2] : 0.0f;
    out.quaternionBNO[0] = quaternionValid ? sanitizedQuat[0] : 1.0f;
    out.quaternionBNO[1] = quaternionValid ? sanitizedQuat[1] : 0.0f;
    out.quaternionBNO[2] = quaternionValid ? sanitizedQuat[2] : 0.0f;
    out.quaternionBNO[3] = quaternionValid ? sanitizedQuat[3] : 0.0f;
    out.hasBnoQuaternion = quaternionValid;
}

}  // namespace

bool Bno055SensorBegin() {
    if (g_initialized) {
        return true;
    }
    return RecoverSensor("startup");
}

bool Bno055SensorAcquire(SensorData &out) {
    if (!g_initialized) {
        return false;
    }

    const uint32_t nowUs = micros();
    g_lastAcquireFresh = false;
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
    float adjustedQuat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    bno085_orientation::AdjustQuaternion(quat.w(), quat.x(), quat.y(), quat.z(), adjustedQuat);
    if (math_utils::SanitizeQuaternionArray(adjustedQuat)) {
        for (int i = 0; i < 4; ++i) {
            g_lastQuat[i] = adjustedQuat[i];
        }
        g_invalidQuaternionStreak = 0;
        g_haveQuat = true;
    } else {
        if (g_haveQuat && g_invalidQuaternionStreak < 0xff) {
            ++g_invalidQuaternionStreak;
        }
        if (!g_haveQuat || g_invalidQuaternionStreak >= kQuaternionInvalidDropThreshold) {
            g_haveQuat = false;
        }
    }
    g_haveAccel = true;
    g_haveGyro = true;
    g_lastHealthyEventUs = nowUs;
    g_lastAcquireFresh = true;

    PopulateOutput(out, nowUs);
    return true;
}

bool Bno055SensorIsInitialized() {
    return g_initialized;
}

BnoDiagnostics Bno055SensorGetDiagnostics() {
    BnoDiagnostics diagnostics;
    const uint32_t nowUs = micros();
    float sanitizedQuat[4] = {g_lastQuat[0], g_lastQuat[1], g_lastQuat[2], g_lastQuat[3]};
    diagnostics.transportReady = g_initialized;
    diagnostics.hasAccel = g_haveAccel && g_lastHealthyEventUs != 0 && (nowUs - g_lastHealthyEventUs) <= kDataTimeoutUs;
    diagnostics.hasGyro = diagnostics.hasAccel && g_haveGyro;
    diagnostics.hasQuaternion =
        diagnostics.hasAccel && g_haveQuat && math_utils::SanitizeQuaternionArray(sanitizedQuat);
    diagnostics.lastAcquireFresh = g_lastAcquireFresh;
    diagnostics.accelBodyMps2[0] = g_lastAccel[0];
    diagnostics.accelBodyMps2[1] = g_lastAccel[1];
    diagnostics.accelBodyMps2[2] = g_lastAccel[2];
    if (diagnostics.hasQuaternion) {
        float yaw = 0.0f;
        float pitch = 0.0f;
        float roll = 0.0f;
        const math_utils::Quaternion quat =
            math_utils::MakeQuaternion(sanitizedQuat[0], sanitizedQuat[1], sanitizedQuat[2], sanitizedQuat[3]);
        math_utils::QuaternionToEuler(quat, yaw, pitch, roll);
        diagnostics.yprDeg[0] = yaw * 57.295779513082320876f;
        diagnostics.yprDeg[1] = pitch * 57.295779513082320876f;
        diagnostics.yprDeg[2] = roll * 57.295779513082320876f;
    }
    return diagnostics;
}

BnoSample Bno055SensorGetSample() {
    BnoSample sample;
    const uint32_t nowUs = micros();
    const bool fresh = g_lastHealthyEventUs != 0 && (nowUs - g_lastHealthyEventUs) <= kDataTimeoutUs;
    float sanitizedQuat[4] = {g_lastQuat[0], g_lastQuat[1], g_lastQuat[2], g_lastQuat[3]};
    sample.hasAccel = fresh && g_haveAccel;
    sample.hasGyro = fresh && g_haveGyro;
    sample.hasQuaternion = fresh && g_haveQuat && math_utils::SanitizeQuaternionArray(sanitizedQuat);
    sample.accelMicros = sample.hasAccel ? g_lastHealthyEventUs : 0;
    sample.gyroMicros = sample.hasGyro ? g_lastHealthyEventUs : 0;
    sample.quaternionMicros = sample.hasQuaternion ? g_lastHealthyEventUs : 0;
    sample.sampleMicros = g_lastHealthyEventUs;
    if (sample.hasAccel) {
        for (int i = 0; i < 3; ++i) {
            sample.accel[i] = g_lastAccel[i];
        }
    }
    if (sample.hasGyro) {
        for (int i = 0; i < 3; ++i) {
            sample.gyro[i] = g_lastGyro[i];
        }
    }
    if (sample.hasQuaternion) {
        for (int i = 0; i < 4; ++i) {
            sample.quaternion[i] = sanitizedQuat[i];
        }
    }
    return sample;
}
