#include "bno085_sensor.h"

#include <algorithm>

#include <Arduino.h>
#include <Wire.h>

#include <SparkFun_BNO08x_Arduino_Library.h>

#include "bno085_orientation.h"
#include "math_utils.h"
#include "serial_logging.h"
#include "settings.h"

namespace {

constexpr bool kBnoEnabled = settings::sensors::bno::kEnabled;
constexpr uint8_t kBnoI2cAddress = settings::sensors::bno085::kI2cAddress;
constexpr uint32_t kBnoI2cClockHz = settings::sensors::bno085::kI2cClockHz;
constexpr int8_t kBnoInterruptPin = settings::sensors::bno085::kInterruptPin;
constexpr int8_t kBnoResetPin = settings::sensors::bno085::kResetPin;
constexpr uint16_t kReportIntervalMs = 5;
constexpr uint8_t kQuaternionInvalidDropThreshold = 2;
constexpr uint32_t kFreshSignalMaxAgeUs = settings::sensors::icm20948::crosscheck::kBnoSampleMaxAgeUs;

BNO08x g_bno;

volatile bool g_intFlag = false;
bool g_initialized = false;
bool g_haveAccel = false;
bool g_haveGyro = false;
bool g_haveQuaternion = false;
bool g_haveLinearAccel = false;
bool g_haveGravity = false;
bool g_haveMagnetometer = false;
bool g_lastAcquireFresh = false;
uint32_t g_lastSampleMicros = 0;
uint32_t g_lastAccelMicros = 0;
uint32_t g_lastGyroMicros = 0;
uint32_t g_lastQuatMicros = 0;
uint32_t g_lastMagnetometerMicros = 0;

float g_lastAccel[3] = {0.0f, 0.0f, 0.0f};
float g_lastGyro[3] = {0.0f, 0.0f, 0.0f};
float g_lastQuat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float g_lastLinearAccel[3] = {0.0f, 0.0f, 0.0f};
float g_lastGravity[3] = {0.0f, 0.0f, 0.0f};
float g_lastMagnetometer[3] = {0.0f, 0.0f, 0.0f};
uint8_t g_invalidQuaternionStreak = 0;

void ImuIntISR() {
    g_intFlag = true;
}

void ResetCachedState() {
    g_haveAccel = false;
    g_haveGyro = false;
    g_haveQuaternion = false;
    g_haveLinearAccel = false;
    g_haveGravity = false;
    g_haveMagnetometer = false;
    g_lastAccel[0] = 0.0f;
    g_lastAccel[1] = 0.0f;
    g_lastAccel[2] = 0.0f;
    g_lastGyro[0] = 0.0f;
    g_lastGyro[1] = 0.0f;
    g_lastGyro[2] = 0.0f;
    g_lastQuat[0] = 1.0f;
    g_lastQuat[1] = 0.0f;
    g_lastQuat[2] = 0.0f;
    g_lastQuat[3] = 0.0f;
    g_invalidQuaternionStreak = 0;
    g_lastLinearAccel[0] = 0.0f;
    g_lastLinearAccel[1] = 0.0f;
    g_lastLinearAccel[2] = 0.0f;
    g_lastGravity[0] = 0.0f;
    g_lastGravity[1] = 0.0f;
    g_lastGravity[2] = 0.0f;
    g_lastMagnetometer[0] = 0.0f;
    g_lastMagnetometer[1] = 0.0f;
    g_lastMagnetometer[2] = 0.0f;
    g_lastSampleMicros = 0;
    g_lastAccelMicros = 0;
    g_lastGyroMicros = 0;
    g_lastQuatMicros = 0;
    g_lastMagnetometerMicros = 0;
}

bool SignalFresh(uint32_t nowUs, uint32_t lastUpdateUs) {
    return lastUpdateUs > 0 && (nowUs - lastUpdateUs) <= kFreshSignalMaxAgeUs;
}

bool InterruptAsserted() {
    if (kBnoInterruptPin < 0) {
        return false;
    }
    return digitalRead(kBnoInterruptPin) == LOW;
}

void TransformIntoBodyFrame(float rawX, float rawY, float rawZ, float &x, float &y, float &z) {
    bno085_orientation::TransformVector(rawX, rawY, rawZ, x, y, z);
}

bool Normalize3(float &x, float &y, float &z) {
    const float magSq = x * x + y * y + z * z;
    if (magSq <= 1.0e-18f) {
        return false;
    }
    const float inv = math_utils::FastInvSqrt(magSq);
    x *= inv;
    y *= inv;
    z *= inv;
    return true;
}

void Cross3(float ax, float ay, float az, float bx, float by, float bz, float out[3]) {
    out[0] = ay * bz - az * by;
    out[1] = az * bx - ax * bz;
    out[2] = ax * by - ay * bx;
}

bool QuaternionFromEarthBasisInBody(const float northBody[3],
                                    const float eastBody[3],
                                    const float upBody[3],
                                    float quaternion[4]) {
    const float r00 = northBody[0];
    const float r01 = eastBody[0];
    const float r02 = upBody[0];
    const float r10 = northBody[1];
    const float r11 = eastBody[1];
    const float r12 = upBody[1];
    const float r20 = northBody[2];
    const float r21 = eastBody[2];
    const float r22 = upBody[2];

    const float trace = r00 + r11 + r22;
    if (trace > 0.0f) {
        const float s = 2.0f * sqrtf(trace + 1.0f);
        if (s <= 1.0e-9f) {
            return false;
        }
        quaternion[0] = 0.25f * s;
        quaternion[1] = (r21 - r12) / s;
        quaternion[2] = (r02 - r20) / s;
        quaternion[3] = (r10 - r01) / s;
    } else if (r00 > r11 && r00 > r22) {
        const float s = 2.0f * sqrtf(1.0f + r00 - r11 - r22);
        if (s <= 1.0e-9f) {
            return false;
        }
        quaternion[0] = (r21 - r12) / s;
        quaternion[1] = 0.25f * s;
        quaternion[2] = (r01 + r10) / s;
        quaternion[3] = (r02 + r20) / s;
    } else if (r11 > r22) {
        const float s = 2.0f * sqrtf(1.0f + r11 - r00 - r22);
        if (s <= 1.0e-9f) {
            return false;
        }
        quaternion[0] = (r02 - r20) / s;
        quaternion[1] = (r01 + r10) / s;
        quaternion[2] = 0.25f * s;
        quaternion[3] = (r12 + r21) / s;
    } else {
        const float s = 2.0f * sqrtf(1.0f + r22 - r00 - r11);
        if (s <= 1.0e-9f) {
            return false;
        }
        quaternion[0] = (r10 - r01) / s;
        quaternion[1] = (r02 + r20) / s;
        quaternion[2] = (r12 + r21) / s;
        quaternion[3] = 0.25f * s;
    }

    const float norm = sqrtf(quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] +
                             quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);
    if (norm <= 1.0e-9f) {
        return false;
    }
    const float invNorm = 1.0f / norm;
    quaternion[0] *= invNorm;
    quaternion[1] *= invNorm;
    quaternion[2] *= invNorm;
    quaternion[3] *= invNorm;
    return true;
}

bool QuaternionFromAccelMag(const float accelBody[3], const float magBody[3], float quaternion[4]) {
    float upBody[3] = {accelBody[0], accelBody[1], accelBody[2]};
    float magneticBody[3] = {magBody[0], magBody[1], magBody[2]};
    if (!Normalize3(upBody[0], upBody[1], upBody[2]) ||
        !Normalize3(magneticBody[0], magneticBody[1], magneticBody[2])) {
        return false;
    }

    float eastBody[3] = {0.0f, 0.0f, 0.0f};
    Cross3(upBody[0], upBody[1], upBody[2], magneticBody[0], magneticBody[1], magneticBody[2], eastBody);
    if (!Normalize3(eastBody[0], eastBody[1], eastBody[2])) {
        return false;
    }

    float northBody[3] = {0.0f, 0.0f, 0.0f};
    Cross3(eastBody[0], eastBody[1], eastBody[2], upBody[0], upBody[1], upBody[2], northBody);
    if (!Normalize3(northBody[0], northBody[1], northBody[2])) {
        return false;
    }

    return QuaternionFromEarthBasisInBody(northBody, eastBody, upBody, quaternion);
}

void UpdateQuaternion(float real, float i, float j, float k) {
    float adjusted[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    bno085_orientation::AdjustQuaternion(real, i, j, k, adjusted);
    if (math_utils::ValidateQuaternionArray(adjusted)) {
        for (int axis = 0; axis < 4; ++axis) {
            g_lastQuat[axis] = adjusted[axis];
        }
        g_invalidQuaternionStreak = 0;
        g_haveQuaternion = true;
        g_lastQuatMicros = micros();
        return;
    }

    if (g_haveQuaternion && g_invalidQuaternionStreak < 0xff) {
        ++g_invalidQuaternionStreak;
    }
    if (!g_haveQuaternion || g_invalidQuaternionStreak >= kQuaternionInvalidDropThreshold) {
        g_haveQuaternion = false;
    }
}

bool ConfigureReports() {
    bool ok = true;

    ok = g_bno.enableAccelerometer(kReportIntervalMs) && ok;
    if (!ok) {
        LOG_PRINTLN("BNO085: enableAccelerometer failed");
    }
    if (!g_bno.enableLinearAccelerometer(kReportIntervalMs)) {
        LOG_PRINTLN("BNO085: enableLinearAccelerometer failed");
        ok = false;
    }
    if (!g_bno.enableGravity(kReportIntervalMs)) {
        LOG_PRINTLN("BNO085: enableGravity failed");
        ok = false;
    }
    if (!g_bno.enableGyro(kReportIntervalMs)) {
        LOG_PRINTLN("BNO085: enableGyro failed");
        ok = false;
    }
    if (!g_bno.enableUncalibratedGyro(kReportIntervalMs)) {
        LOG_PRINTLN("BNO085: enableUncalibratedGyro failed");
        ok = false;
    }
    if (!g_bno.enableMagnetometer(kReportIntervalMs)) {
        LOG_PRINTLN("BNO085: enableMagnetometer failed");
        ok = false;
    }
    if (!g_bno.enableRotationVector(kReportIntervalMs)) {
        LOG_PRINTLN("BNO085: enableRotationVector failed");
        ok = false;
    }
    if (!g_bno.enableGameRotationVector(kReportIntervalMs)) {
        LOG_PRINTLN("BNO085: enableGameRotationVector failed");
        ok = false;
    }
    if (!g_bno.enableGeomagneticRotationVector(kReportIntervalMs)) {
        LOG_PRINTLN("BNO085: enableGeomagneticRotationVector failed");
        ok = false;
    }
    if (!g_bno.enableGyroIntegratedRotationVector(kReportIntervalMs)) {
        LOG_PRINTLN("BNO085: enableGyroIntegratedRotationVector failed");
        ok = false;
    }

    return ok;
}

void PublishCachedState(SensorData &out, uint32_t nowUs) {
    const bool accelFresh = g_haveAccel && SignalFresh(nowUs, g_lastAccelMicros);
    const bool gyroFresh = g_haveGyro && SignalFresh(nowUs, g_lastGyroMicros);
    const bool quaternionValid =
        g_haveQuaternion && SignalFresh(nowUs, g_lastQuatMicros) && math_utils::ValidateQuaternionArray(g_lastQuat);
    if (accelFresh) {
        out.accelBNO[0] = g_lastAccel[0];
        out.accelBNO[1] = g_lastAccel[1];
        out.accelBNO[2] = g_lastAccel[2];
    } else {
        out.accelBNO[0] = 0.0f;
        out.accelBNO[1] = 0.0f;
        out.accelBNO[2] = 0.0f;
    }
    if (gyroFresh) {
        out.gyroBNO[0] = g_lastGyro[0];
        out.gyroBNO[1] = g_lastGyro[1];
        out.gyroBNO[2] = g_lastGyro[2];
    } else {
        out.gyroBNO[0] = 0.0f;
        out.gyroBNO[1] = 0.0f;
        out.gyroBNO[2] = 0.0f;
    }
    if (quaternionValid) {
        out.quaternionBNO[0] = g_lastQuat[0];
        out.quaternionBNO[1] = g_lastQuat[1];
        out.quaternionBNO[2] = g_lastQuat[2];
        out.quaternionBNO[3] = g_lastQuat[3];
        out.hasBnoQuaternion = true;
    } else {
        out.quaternionBNO[0] = 1.0f;
        out.quaternionBNO[1] = 0.0f;
        out.quaternionBNO[2] = 0.0f;
        out.quaternionBNO[3] = 0.0f;
        out.hasBnoQuaternion = false;
    }
}

bool ConsumeSensorEvent() {
    const uint8_t reportId = g_bno.getSensorEventID();
    const uint32_t nowUs = micros();
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    switch (reportId) {
        case SENSOR_REPORTID_ACCELEROMETER:
            TransformIntoBodyFrame(g_bno.getAccelX(), g_bno.getAccelY(), g_bno.getAccelZ(), x, y, z);
            g_lastAccel[0] = x;
            g_lastAccel[1] = y;
            g_lastAccel[2] = z;
            g_haveAccel = true;
            g_lastAccelMicros = nowUs;
            return true;
        case SENSOR_REPORTID_LINEAR_ACCELERATION:
            TransformIntoBodyFrame(g_bno.getLinAccelX(), g_bno.getLinAccelY(), g_bno.getLinAccelZ(), x, y, z);
            g_lastLinearAccel[0] = x;
            g_lastLinearAccel[1] = y;
            g_lastLinearAccel[2] = z;
            g_haveLinearAccel = true;
            return true;
        case SENSOR_REPORTID_GRAVITY:
            TransformIntoBodyFrame(g_bno.getGravityX(), g_bno.getGravityY(), g_bno.getGravityZ(), x, y, z);
            g_lastGravity[0] = x;
            g_lastGravity[1] = y;
            g_lastGravity[2] = z;
            g_haveGravity = true;
            return true;
        case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
            TransformIntoBodyFrame(g_bno.getGyroX(), g_bno.getGyroY(), g_bno.getGyroZ(), x, y, z);
            g_lastGyro[0] = x;
            g_lastGyro[1] = y;
            g_lastGyro[2] = z;
            g_haveGyro = true;
            g_lastGyroMicros = nowUs;
            return true;
        case SENSOR_REPORTID_UNCALIBRATED_GYRO:
            if (!g_haveGyro) {
                TransformIntoBodyFrame(g_bno.getUncalibratedGyroX(),
                                       g_bno.getUncalibratedGyroY(),
                                       g_bno.getUncalibratedGyroZ(),
                                       x,
                                       y,
                                       z);
                g_lastGyro[0] = x;
                g_lastGyro[1] = y;
                g_lastGyro[2] = z;
                g_haveGyro = true;
                g_lastGyroMicros = nowUs;
            }
            return true;
        case SENSOR_REPORTID_MAGNETIC_FIELD:
            TransformIntoBodyFrame(g_bno.getMagX(), g_bno.getMagY(), g_bno.getMagZ(), x, y, z);
            g_lastMagnetometer[0] = x;
            g_lastMagnetometer[1] = y;
            g_lastMagnetometer[2] = z;
            g_haveMagnetometer = true;
            g_lastMagnetometerMicros = nowUs;
            return true;
        case SENSOR_REPORTID_ROTATION_VECTOR:
            UpdateQuaternion(g_bno.getQuatReal(), g_bno.getQuatI(), g_bno.getQuatJ(), g_bno.getQuatK());
            return true;
        case SENSOR_REPORTID_GAME_ROTATION_VECTOR:
            if (!g_haveQuaternion) {
                UpdateQuaternion(g_bno.getGameQuatReal(),
                                 g_bno.getGameQuatI(),
                                 g_bno.getGameQuatJ(),
                                 g_bno.getGameQuatK());
            }
            return true;
        case SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR:
            if (!g_haveQuaternion) {
                UpdateQuaternion(g_bno.getGyroIntegratedRVReal(),
                                 g_bno.getGyroIntegratedRVI(),
                                 g_bno.getGyroIntegratedRVJ(),
                                 g_bno.getGyroIntegratedRVK());
            }
            return true;
        default:
            return false;
    }
}

}  // namespace

bool Bno085SensorBegin() {
    if (!kBnoEnabled) {
        return false;
    }
    if (g_initialized) {
        return true;
    }

    ResetCachedState();
    g_intFlag = false;

    if (kBnoInterruptPin >= 0) {
        pinMode(kBnoInterruptPin, INPUT_PULLUP);
    }

    Wire1.begin();
    Wire1.setClock(kBnoI2cClockHz);
    if (!g_bno.begin(kBnoI2cAddress, Wire1)) {
    // if (!g_bno.begin(kBnoI2cAddress, Wire1, kBnoInterruptPin, kBnoResetPin)) {
        LOG_PRINTLN("BNO085: begin(I2C) failed");
        return false;
    }

    ConfigureReports();
    if (kBnoInterruptPin >= 0) {
        attachInterrupt(digitalPinToInterrupt(kBnoInterruptPin), ImuIntISR, FALLING);
    }

    g_initialized = true;
    LOG_PRINTLN("BNO085: I2C startup complete");
    return true;
}

bool Bno085SensorAcquire(SensorData &out) {
    if (!kBnoEnabled || !g_initialized) {
        g_lastAcquireFresh = false;
        return false;
    }

    if (g_bno.wasReset()) {
        LOG_PRINTLN("BNO085: sensor reset detected");
        ResetCachedState();
        ConfigureReports();
    }

    bool shouldRead = false;
    noInterrupts();
    if (g_intFlag) {
        g_intFlag = false;
        shouldRead = true;
    }
    interrupts();

    if (!shouldRead && kBnoInterruptPin >= 0 && !InterruptAsserted()) {
        g_lastAcquireFresh = false;
        return false;
    }

    bool consumedAny = false;
    while (g_bno.getSensorEvent()) {
        consumedAny = ConsumeSensorEvent() || consumedAny;
        if (kBnoInterruptPin >= 0 && !InterruptAsserted()) {
            break;
        }
    }

    const uint32_t nowUs = micros();
    const bool accelFresh = g_haveAccel && SignalFresh(nowUs, g_lastAccelMicros);
    const bool gyroFresh = g_haveGyro && SignalFresh(nowUs, g_lastGyroMicros);
    const bool quatFresh =
        g_haveQuaternion && SignalFresh(nowUs, g_lastQuatMicros) && math_utils::ValidateQuaternionArray(g_lastQuat);
    if (!consumedAny || !(accelFresh || gyroFresh || quatFresh)) {
        g_lastAcquireFresh = false;
        return false;
    }
    PublishCachedState(out, nowUs);
    g_lastAcquireFresh = true;
    g_lastSampleMicros = std::max(g_lastAccelMicros, std::max(g_lastGyroMicros, g_lastQuatMicros));
    return true;
}

bool Bno085SensorIsInitialized() {
    return kBnoEnabled && g_initialized;
}

Bno085Diagnostics Bno085SensorGetDiagnostics() {
    const uint32_t nowUs = micros();
    Bno085Diagnostics diagnostics;
    diagnostics.transportReady = kBnoEnabled && g_initialized;
    diagnostics.hasAccel = kBnoEnabled && g_haveAccel && SignalFresh(nowUs, g_lastAccelMicros);
    diagnostics.hasGyro = kBnoEnabled && g_haveGyro;
    diagnostics.hasMag =
        kBnoEnabled && g_haveMagnetometer && SignalFresh(nowUs, g_lastMagnetometerMicros);
    diagnostics.hasQuaternion =
        kBnoEnabled && g_haveQuaternion && SignalFresh(nowUs, g_lastQuatMicros) &&
        math_utils::ValidateQuaternionArray(g_lastQuat);
    diagnostics.lastAcquireFresh = kBnoEnabled && g_lastAcquireFresh;
    diagnostics.accelBodyMps2[0] = g_lastAccel[0];
    diagnostics.accelBodyMps2[1] = g_lastAccel[1];
    diagnostics.accelBodyMps2[2] = g_lastAccel[2];
    diagnostics.magBody[0] = g_lastMagnetometer[0];
    diagnostics.magBody[1] = g_lastMagnetometer[1];
    diagnostics.magBody[2] = g_lastMagnetometer[2];
    if (diagnostics.hasAccel && diagnostics.hasMag) {
        float bootstrapQuat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        if (QuaternionFromAccelMag(g_lastAccel, g_lastMagnetometer, bootstrapQuat)) {
            float yaw = 0.0f;
            float pitch = 0.0f;
            float roll = 0.0f;
            const math_utils::Quaternion quat =
                math_utils::MakeQuaternion(bootstrapQuat[0], bootstrapQuat[1], bootstrapQuat[2], bootstrapQuat[3]);
            math_utils::QuaternionToEuler(quat, yaw, pitch, roll);
            diagnostics.hasBootstrapYpr = true;
            diagnostics.bootstrapYprDeg[0] = yaw * 57.295779513082320876f;
            diagnostics.bootstrapYprDeg[1] = pitch * 57.295779513082320876f;
            diagnostics.bootstrapYprDeg[2] = roll * 57.295779513082320876f;
        }
    }
    if (diagnostics.hasQuaternion) {
        float yaw = 0.0f;
        float pitch = 0.0f;
        float roll = 0.0f;
        const math_utils::Quaternion quat =
            math_utils::MakeQuaternion(g_lastQuat[0], g_lastQuat[1], g_lastQuat[2], g_lastQuat[3]);
        math_utils::QuaternionToEuler(quat, yaw, pitch, roll);
        diagnostics.yprDeg[0] = yaw * 57.295779513082320876f;
        diagnostics.yprDeg[1] = pitch * 57.295779513082320876f;
        diagnostics.yprDeg[2] = roll * 57.295779513082320876f;
    }
    return diagnostics;
}

Bno085Sample Bno085SensorGetSample() {
    Bno085Sample sample;
    const uint32_t nowUs = micros();
    sample.hasAccel = kBnoEnabled && g_haveAccel && SignalFresh(nowUs, g_lastAccelMicros);
    sample.hasGyro = kBnoEnabled && g_haveGyro && SignalFresh(nowUs, g_lastGyroMicros);
    sample.hasQuaternion =
        kBnoEnabled && g_haveQuaternion && SignalFresh(nowUs, g_lastQuatMicros) &&
        math_utils::ValidateQuaternionArray(g_lastQuat);
    sample.accelMicros = sample.hasAccel ? g_lastAccelMicros : 0;
    sample.gyroMicros = sample.hasGyro ? g_lastGyroMicros : 0;
    sample.quaternionMicros = sample.hasQuaternion ? g_lastQuatMicros : 0;
    sample.sampleMicros =
        std::max(sample.accelMicros, std::max(sample.gyroMicros, sample.quaternionMicros));
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
            sample.quaternion[i] = g_lastQuat[i];
        }
    }
    return sample;
}
