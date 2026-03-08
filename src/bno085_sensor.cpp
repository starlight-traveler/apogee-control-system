#include "bno085_sensor.h"

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <Adafruit_BNO08x.h>
#include <sh2.h>

#include "bno085_orientation.h"
#include "serial_logging.h"
#include "settings.h"

namespace {

constexpr uint32_t kSampleIntervalUs = settings::sensors::bno085::kSampleIntervalUs;
constexpr uint8_t kBnoI2cAddress = settings::sensors::bno085::kI2cAddress;
constexpr uint8_t kBnoChipSelectPin = settings::sensors::bno085::kChipSelectPin;
constexpr int8_t kBnoInterruptPin = settings::sensors::bno085::kInterruptPin;
constexpr int8_t kBnoResetPin = settings::sensors::bno085::kResetPin;
constexpr uint32_t kDataTimeoutUs = settings::sensors::bno085::kDataTimeoutUs;

Adafruit_BNO08x g_bno(kBnoResetPin);
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

bool EnableReports() {
    return g_bno.enableReport(SH2_ACCELEROMETER, kSampleIntervalUs) &&
           g_bno.enableReport(SH2_GYROSCOPE_CALIBRATED, kSampleIntervalUs) &&
           g_bno.enableReport(SH2_ROTATION_VECTOR, kSampleIntervalUs);
}

bool StartSensorTransport() {
    switch (settings::sensors::bno::kTransport) {
        case settings::sensors::bno::Transport::I2c:
            Wire.begin();
            if (!g_bno.begin_I2C(kBnoI2cAddress, &Wire)) {
                return false;
            }
            break;
        case settings::sensors::bno::Transport::Spi:
            SPI.begin();
            if (!g_bno.begin_SPI(kBnoChipSelectPin, kBnoInterruptPin, &SPI)) {
                return false;
            }
            break;
    }
    delay(10);
    return EnableReports();
}

bool RecoverSensor(const char *reason) {
    g_initialized = false;
    ResetCachedState();

    if (reason != nullptr) {
        LOG_PRINT("BNO085 recovery: ");
        LOG_PRINTLN(reason);
    }

    if (StartSensorTransport()) {
        g_initialized = true;
        LOG_PRINT("BNO085 online via ");
        LOG_PRINTLN(settings::sensors::bno::kTransport == settings::sensors::bno::Transport::I2c ? "I2C" : "SPI");
        return true;
    }
    LOG_PRINTLN("BNO085 init failed; caller will retry");
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

void ConsumeEvent(const sh2_SensorValue_t &sensorValue, uint32_t nowUs) {
    switch (sensorValue.sensorId) {
        case SH2_ACCELEROMETER:
            bno085_orientation::TransformVector(sensorValue.un.accelerometer.x,
                                                sensorValue.un.accelerometer.y,
                                                sensorValue.un.accelerometer.z,
                                                g_lastAccel[0],
                                                g_lastAccel[1],
                                                g_lastAccel[2]);
            g_haveAccel = true;
            g_lastHealthyEventUs = nowUs;
            break;
        case SH2_GYROSCOPE_CALIBRATED:
            bno085_orientation::TransformVector(sensorValue.un.gyroscope.x,
                                                sensorValue.un.gyroscope.y,
                                                sensorValue.un.gyroscope.z,
                                                g_lastGyro[0],
                                                g_lastGyro[1],
                                                g_lastGyro[2]);
            g_haveGyro = true;
            g_lastHealthyEventUs = nowUs;
            break;
        case SH2_ROTATION_VECTOR:
            bno085_orientation::AdjustQuaternion(sensorValue.un.rotationVector.real,
                                                 sensorValue.un.rotationVector.i,
                                                 sensorValue.un.rotationVector.j,
                                                 sensorValue.un.rotationVector.k,
                                                 g_lastQuat);
            g_haveQuat = true;
            g_lastHealthyEventUs = nowUs;
            break;
        default:
            break;
    }
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

    sh2_SensorValue_t sensorValue;
    bool consumedAny = false;
    while (g_bno.getSensorEvent(&sensorValue)) {
        ConsumeEvent(sensorValue, nowUs);
        consumedAny = true;
    }

    if (!consumedAny && !g_haveQuat) {
        return false;
    }

    PopulateOutput(out, nowUs);
    return g_haveQuat;
}

bool Bno085SensorIsInitialized() {
    return g_initialized;
}
