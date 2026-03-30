#include "bno085_sensor.h"

#include <Arduino.h>
#include <Wire.h>

#include <SparkFun_BNO08x_Arduino_Library.h>

#include "bno085_orientation.h"
#include "serial_logging.h"
#include "settings.h"

namespace {

constexpr bool kBnoEnabled = settings::sensors::bno::kEnabled;
constexpr uint8_t kBnoI2cAddress = settings::sensors::bno085::kI2cAddress;
constexpr uint32_t kBnoI2cClockHz = settings::sensors::bno085::kI2cClockHz;
constexpr int8_t kBnoInterruptPin = settings::sensors::bno085::kInterruptPin;
constexpr int8_t kBnoResetPin = settings::sensors::bno085::kResetPin;
constexpr uint16_t kReportIntervalMs = 5;

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

float g_lastAccel[3] = {0.0f, 0.0f, 0.0f};
float g_lastGyro[3] = {0.0f, 0.0f, 0.0f};
float g_lastQuat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float g_lastLinearAccel[3] = {0.0f, 0.0f, 0.0f};
float g_lastGravity[3] = {0.0f, 0.0f, 0.0f};
float g_lastMagnetometer[3] = {0.0f, 0.0f, 0.0f};

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

void UpdateQuaternion(float real, float i, float j, float k) {
    bno085_orientation::AdjustQuaternion(real, i, j, k, g_lastQuat);
    g_haveQuaternion = true;
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
    out.timestamp = static_cast<float>(nowUs) * 1.0e-6f;
    if (g_haveAccel) {
        out.accelBNO[0] = g_lastAccel[0];
        out.accelBNO[1] = g_lastAccel[1];
        out.accelBNO[2] = g_lastAccel[2];
        out.accelICM[0] = g_lastAccel[0];
        out.accelICM[1] = g_lastAccel[1];
        out.accelICM[2] = g_lastAccel[2];
    }
    if (g_haveGyro) {
        out.gyroBNO[0] = g_lastGyro[0];
        out.gyroBNO[1] = g_lastGyro[1];
        out.gyroBNO[2] = g_lastGyro[2];
        out.gyro[0] = g_lastGyro[0];
        out.gyro[1] = g_lastGyro[1];
        out.gyro[2] = g_lastGyro[2];
    }
    if (g_haveQuaternion) {
        out.quaternionBNO[0] = g_lastQuat[0];
        out.quaternionBNO[1] = g_lastQuat[1];
        out.quaternionBNO[2] = g_lastQuat[2];
        out.quaternionBNO[3] = g_lastQuat[3];
        out.quaternion[0] = g_lastQuat[0];
        out.quaternion[1] = g_lastQuat[1];
        out.quaternion[2] = g_lastQuat[2];
        out.quaternion[3] = g_lastQuat[3];
        out.hasBnoQuaternion = true;
        out.hasQuaternion = true;
    }
}

bool ConsumeSensorEvent() {
    const uint8_t reportId = g_bno.getSensorEventID();
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
            }
            return true;
        case SENSOR_REPORTID_MAGNETIC_FIELD:
            TransformIntoBodyFrame(g_bno.getMagX(), g_bno.getMagY(), g_bno.getMagZ(), x, y, z);
            g_lastMagnetometer[0] = x;
            g_lastMagnetometer[1] = y;
            g_lastMagnetometer[2] = z;
            g_haveMagnetometer = true;
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

    Wire.begin();
    Wire.setClock(kBnoI2cClockHz);

    if (!g_bno.begin(kBnoI2cAddress, Wire, kBnoInterruptPin, kBnoResetPin)) {
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

    if (!consumedAny || !(g_haveAccel || g_haveGyro || g_haveQuaternion)) {
        g_lastAcquireFresh = false;
        return false;
    }

    const uint32_t nowUs = micros();
    PublishCachedState(out, nowUs);
    g_lastAcquireFresh = true;
    g_lastSampleMicros = nowUs;
    return true;
}

bool Bno085SensorIsInitialized() {
    return kBnoEnabled && g_initialized;
}

Bno085Diagnostics Bno085SensorGetDiagnostics() {
    Bno085Diagnostics diagnostics;
    diagnostics.transportReady = kBnoEnabled && g_initialized;
    diagnostics.hasAccel = kBnoEnabled && g_haveAccel;
    diagnostics.hasGyro = kBnoEnabled && g_haveGyro;
    diagnostics.hasQuaternion = kBnoEnabled && g_haveQuaternion;
    diagnostics.lastAcquireFresh = kBnoEnabled && g_lastAcquireFresh;
    return diagnostics;
}

Bno085Sample Bno085SensorGetSample() {
    Bno085Sample sample;
    sample.hasAccel = kBnoEnabled && g_haveAccel;
    sample.hasGyro = kBnoEnabled && g_haveGyro;
    sample.hasQuaternion = kBnoEnabled && g_haveQuaternion;
    sample.sampleMicros = kBnoEnabled ? g_lastSampleMicros : 0;
    for (int i = 0; i < 3; ++i) {
        sample.accel[i] = g_lastAccel[i];
        sample.gyro[i] = g_lastGyro[i];
    }
    for (int i = 0; i < 4; ++i) {
        sample.quaternion[i] = g_lastQuat[i];
    }
    return sample;
}
