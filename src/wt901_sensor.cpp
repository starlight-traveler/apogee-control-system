#include "wt901_sensor.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <math.h>

extern "C" {
#include "wt901_reg.h"
#include "wt901_sdk.h"
}

#ifdef q0
#undef q0
#endif
#ifdef q1
#undef q1
#endif
#ifdef q2
#undef q2
#endif
#ifdef q3
#undef q3
#endif

#include "math_utils.h"
#include "serial_logging.h"
#include "settings.h"

namespace {

constexpr uint8_t kSerialPortIndex = settings::sensors::wt901::kSerialPortIndex;
constexpr int8_t kRxPin = settings::sensors::wt901::kRxPin;
constexpr int8_t kTxPin = settings::sensors::wt901::kTxPin;
constexpr uint32_t kPreferredBaudRate = settings::sensors::wt901::kBaudRate;
constexpr uint32_t kPollIntervalUs = settings::sensors::wt901::kPollIntervalUs;
constexpr uint32_t kSampleMaxAgeUs = settings::sensors::wt901::kSampleMaxAgeUs;
constexpr float kAccelRangeG = settings::sensors::wt901::kAccelRangeG;
constexpr float kGyroRangeDps = settings::sensors::wt901::kGyroRangeDps;
constexpr float kGToMps2 = 9.80665f;
constexpr float kDegToRad = 0.01745329251994329577f;
constexpr uint8_t kSensorAddress = 0x50;
constexpr uint8_t kAutoScanRetries = 2;
constexpr uint32_t kAutoScanBauds[] = {115200u, 9600u, 230400u, 57600u, 38400u, 19200u, 4800u};
constexpr int32_t kConfiguredBaudIndex = WIT_BAUD_230400;
constexpr int32_t kConfiguredBandwidth = BANDWIDTH_256HZ;
constexpr int32_t kConfiguredOutputRate = RRATE_200HZ;
constexpr uint16_t kConfiguredContentMask = RSW_TIME | RSW_ACC | RSW_GYRO | RSW_ANGLE | RSW_MAG | RSW_Q;
constexpr uint32_t kQuat0Reg = 0x51u;
constexpr uint32_t kQuat1Reg = 0x52u;
constexpr uint32_t kQuat2Reg = 0x53u;
constexpr uint32_t kQuat3Reg = 0x54u;
constexpr uint32_t kReadRequestRegisterCount = (kQuat3Reg - AX) + 1;
constexpr uint16_t kCommandSwitchDelayMs = 10;

constexpr uint8_t kAccelUpdate = 0x01;
constexpr uint8_t kGyroUpdate = 0x02;
constexpr uint8_t kAngleUpdate = 0x04;
constexpr uint8_t kQuaternionUpdate = 0x20;
constexpr uint8_t kReadUpdate = 0x80;
constexpr uint8_t kFreshUpdateMask = kAccelUpdate | kGyroUpdate | kAngleUpdate | kQuaternionUpdate | kReadUpdate;

HardwareSerial *g_serial = nullptr;
bool g_initialized = false;
bool g_hasAccel = false;
bool g_hasGyro = false;
bool g_hasYpr = false;
bool g_hasQuaternion = false;
bool g_lastAcquireFresh = false;
bool g_lastAcquireUsedCache = false;
uint32_t g_lastPollUs = 0;
uint32_t g_lastSampleUs = 0;
volatile uint8_t g_dataUpdateMask = 0;
float g_lastAccelBodyMps2[3] = {0.0f, 0.0f, 0.0f};
float g_lastGyroBodyRadPerSec[3] = {0.0f, 0.0f, 0.0f};
float g_lastYprDeg[3] = {0.0f, 0.0f, 0.0f};
float g_lastQuaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};

HardwareSerial *ResolveSerialPort(uint8_t portIndex) {
    switch (portIndex) {
        case 1:
            return &Serial1;
        case 2:
            return &Serial2;
        case 3:
            return &Serial3;
        case 4:
            return &Serial4;
        case 5:
            return &Serial5;
        case 6:
            return &Serial6;
        case 7:
            return &Serial7;
        case 8:
            return &Serial8;
        default:
            return nullptr;
    }
}

void Apply3x3(const float matrix[3][3], const float in[3], float out[3]) {
    out[0] = matrix[0][0] * in[0] + matrix[0][1] * in[1] + matrix[0][2] * in[2];
    out[1] = matrix[1][0] * in[0] + matrix[1][1] * in[1] + matrix[1][2] * in[2];
    out[2] = matrix[2][0] * in[0] + matrix[2][1] * in[1] + matrix[2][2] * in[2];
}

void ConfigureSerial(uint32_t baudRate) {
    if (g_serial == nullptr) {
        return;
    }
    switch (kSerialPortIndex) {
        case 1:
            if (kRxPin >= 0) Serial1.setRX(kRxPin);
            if (kTxPin >= 0) Serial1.setTX(kTxPin);
            break;
        case 2:
            if (kRxPin >= 0) Serial2.setRX(kRxPin);
            if (kTxPin >= 0) Serial2.setTX(kTxPin);
            break;
        case 3:
            if (kRxPin >= 0) Serial3.setRX(kRxPin);
            if (kTxPin >= 0) Serial3.setTX(kTxPin);
            break;
        case 4:
            if (kRxPin >= 0) Serial4.setRX(kRxPin);
            if (kTxPin >= 0) Serial4.setTX(kTxPin);
            break;
        case 5:
            if (kRxPin >= 0) Serial5.setRX(kRxPin);
            if (kTxPin >= 0) Serial5.setTX(kTxPin);
            break;
        case 6:
            if (kRxPin >= 0) Serial6.setRX(kRxPin);
            if (kTxPin >= 0) Serial6.setTX(kTxPin);
            break;
        case 7:
            if (kRxPin >= 0) Serial7.setRX(kRxPin);
            if (kTxPin >= 0) Serial7.setTX(kTxPin);
            break;
        case 8:
            if (kRxPin >= 0) Serial8.setRX(kRxPin);
            if (kTxPin >= 0) Serial8.setTX(kTxPin);
            break;
        default:
            break;
    }
    g_serial->begin(baudRate);
    delay(20);
    while (g_serial->available() > 0) {
        g_serial->read();
    }
}

void SensorUartSend(uint8_t *data, uint32_t size) {
    if (g_serial == nullptr) {
        return;
    }
    g_serial->write(data, size);
    g_serial->flush();
}

void DelayMs(uint16_t delayMs) {
    delay(delayMs);
}

void SensorDataUpdate(uint32_t reg, uint32_t regCount) {
    for (uint32_t i = 0; i < regCount; ++i, ++reg) {
        switch (reg) {
            case AZ:
                g_dataUpdateMask |= kAccelUpdate;
                break;
            case GZ:
                g_dataUpdateMask |= kGyroUpdate;
                break;
            case Yaw:
                g_dataUpdateMask |= kAngleUpdate;
                break;
            case kQuat3Reg:
                g_dataUpdateMask |= kQuaternionUpdate;
                break;
            default:
                g_dataUpdateMask |= kReadUpdate;
                break;
        }
    }
}

void DrainSerialInput() {
    if (g_serial == nullptr) {
        return;
    }
    while (g_serial->available() > 0) {
        WitSerialDataIn(static_cast<uint8_t>(g_serial->read()));
    }
}

bool ProbeBaud(uint32_t baudRate) {
    ConfigureSerial(baudRate);
    g_dataUpdateMask = 0;
    for (uint8_t retry = 0; retry < kAutoScanRetries; ++retry) {
        WitReadReg(AX, 3);
        delay(200);
        DrainSerialInput();
        if (g_dataUpdateMask != 0) {
            LOG_PRINT("WT901 detected at ");
            LOG_PRINT(baudRate);
            LOG_PRINTLN(" baud");
            g_dataUpdateMask = 0;
            return true;
        }
    }
    return false;
}

bool AutoScanSensor() {
    if (ProbeBaud(kPreferredBaudRate)) {
        return true;
    }
    for (uint32_t baudRate : kAutoScanBauds) {
        if (baudRate == kPreferredBaudRate) {
            continue;
        }
        if (ProbeBaud(baudRate)) {
            return true;
        }
    }
    LOG_PRINTLN("WT901 unavailable");
    return false;
}

bool SetSensorBaud(int32_t baudIndex, uint32_t baudRate) {
    if (WitSetUartBaud(baudIndex) != WIT_HAL_OK) {
        return false;
    }
    delay(kCommandSwitchDelayMs);
    ConfigureSerial(baudRate);
    delay(kCommandSwitchDelayMs);
    DrainSerialInput();
    return true;
}

bool ConfigureSensorForMaxData() {
    bool ok = true;
    if (!SetSensorBaud(kConfiguredBaudIndex, kPreferredBaudRate)) {
        LOG_PRINTLN("WT901: failed to set UART baud");
        ok = false;
    }
    if (WitSetBandwidth(kConfiguredBandwidth) != WIT_HAL_OK) {
        LOG_PRINTLN("WT901: failed to set bandwidth");
        ok = false;
    }
    if (WitSetOutputRate(kConfiguredOutputRate) != WIT_HAL_OK) {
        LOG_PRINTLN("WT901: failed to set output rate");
        ok = false;
    }
    if (WitSetContent(kConfiguredContentMask) != WIT_HAL_OK) {
        LOG_PRINTLN("WT901: failed to set output content");
        ok = false;
    }
    DrainSerialInput();
    g_dataUpdateMask = 0;
    if (ok) {
        LOG_PRINTLN("WT901 configured: 230400 baud, 256 Hz bandwidth, 200 Hz stream, time+acc+gyro+angle+mag+quat");
    }
    return ok;
}

void UpdateComputedOutputs(uint32_t nowUs) {
    const float accelSensorG[3] = {
        static_cast<float>(sReg[AX]) / 32768.0f * kAccelRangeG,
        static_cast<float>(sReg[AY]) / 32768.0f * kAccelRangeG,
        static_cast<float>(sReg[AZ]) / 32768.0f * kAccelRangeG,
    };
    float accelBodyG[3] = {0.0f, 0.0f, 0.0f};
    Apply3x3(settings::sensors::wt901::kMountRotation, accelSensorG, accelBodyG);
    for (int i = 0; i < 3; ++i) {
        g_lastAccelBodyMps2[i] = accelBodyG[i] * kGToMps2;
    }
    g_hasAccel = true;

    const float gyroSensorDps[3] = {
        static_cast<float>(sReg[GX]) / 32768.0f * kGyroRangeDps,
        static_cast<float>(sReg[GY]) / 32768.0f * kGyroRangeDps,
        static_cast<float>(sReg[GZ]) / 32768.0f * kGyroRangeDps,
    };
    float gyroBodyDps[3] = {0.0f, 0.0f, 0.0f};
    Apply3x3(settings::sensors::wt901::kMountRotation, gyroSensorDps, gyroBodyDps);
    for (int i = 0; i < 3; ++i) {
        g_lastGyroBodyRadPerSec[i] = gyroBodyDps[i] * kDegToRad;
    }
    g_hasGyro = true;

    g_lastYprDeg[0] = static_cast<float>(sReg[Yaw]) / 32768.0f * 180.0f;
    g_lastYprDeg[1] = static_cast<float>(sReg[Pitch]) / 32768.0f * 180.0f;
    g_lastYprDeg[2] = static_cast<float>(sReg[Roll]) / 32768.0f * 180.0f;
    g_hasYpr = true;

    g_lastQuaternion[0] = static_cast<float>(sReg[kQuat0Reg]) / 32768.0f;
    g_lastQuaternion[1] = static_cast<float>(sReg[kQuat1Reg]) / 32768.0f;
    g_lastQuaternion[2] = static_cast<float>(sReg[kQuat2Reg]) / 32768.0f;
    g_lastQuaternion[3] = static_cast<float>(sReg[kQuat3Reg]) / 32768.0f;
    const math_utils::Quaternion normalized = math_utils::Normalize(
        math_utils::MakeQuaternion(g_lastQuaternion[0], g_lastQuaternion[1], g_lastQuaternion[2], g_lastQuaternion[3]));
    g_lastQuaternion[0] = normalized.w;
    g_lastQuaternion[1] = normalized.x;
    g_lastQuaternion[2] = normalized.y;
    g_lastQuaternion[3] = normalized.z;
    g_hasQuaternion = true;

    g_lastSampleUs = nowUs;
}

bool DataIsFresh(uint32_t nowUs) {
    return g_lastSampleUs != 0 && (nowUs - g_lastSampleUs) <= kSampleMaxAgeUs;
}

}  // namespace

bool Wt901SensorBegin() {
    if (g_initialized) {
        return true;
    }

    g_serial = ResolveSerialPort(kSerialPortIndex);
    if (g_serial == nullptr) {
        LOG_PRINTLN("WT901: invalid serial port");
        return false;
    }

    WitInit(WIT_PROTOCOL_NORMAL, kSensorAddress);
    WitSerialWriteRegister(SensorUartSend);
    WitRegisterCallBack(SensorDataUpdate);
    WitDelayMsRegister(DelayMs);

    if (!AutoScanSensor()) {
        return false;
    }
    ConfigureSensorForMaxData();

    g_initialized = true;
    g_hasAccel = false;
    g_hasGyro = false;
    g_hasYpr = false;
    g_hasQuaternion = false;
    g_lastAcquireFresh = false;
    g_lastAcquireUsedCache = false;
    g_lastPollUs = 0;
    g_lastSampleUs = 0;
    g_dataUpdateMask = 0;
    return true;
}

bool Wt901SensorAcquire() {
    g_lastAcquireFresh = false;
    g_lastAcquireUsedCache = false;
    if (!g_initialized || g_serial == nullptr) {
        return false;
    }

    const uint32_t nowUs = micros();
    DrainSerialInput();

    if ((g_dataUpdateMask & kFreshUpdateMask) == 0 &&
        (g_lastPollUs == 0 || (nowUs - g_lastPollUs) >= kPollIntervalUs)) {
        WitReadReg(AX, kReadRequestRegisterCount);
        g_lastPollUs = nowUs;
        delayMicroseconds(500);
        DrainSerialInput();
    }

    const uint8_t updateMask = g_dataUpdateMask;
    if ((updateMask & kFreshUpdateMask) != 0) {
        g_dataUpdateMask = 0;
        UpdateComputedOutputs(nowUs);
        g_lastAcquireFresh = true;
        return true;
    }

    if (DataIsFresh(nowUs)) {
        g_lastAcquireUsedCache = true;
        return true;
    }
    return false;
}

bool Wt901SensorIsInitialized() {
    return g_initialized;
}

Wt901Diagnostics Wt901SensorGetDiagnostics() {
    Wt901Diagnostics diagnostics;
    diagnostics.initialized = g_initialized;
    diagnostics.lastAcquireFresh = g_lastAcquireFresh;
    diagnostics.lastAcquireUsedCache = g_lastAcquireUsedCache;
    diagnostics.lastSampleMicros = g_lastSampleUs;
    const bool fresh = g_initialized && DataIsFresh(micros());
    diagnostics.hasAccel = fresh && g_hasAccel;
    diagnostics.hasGyro = fresh && g_hasGyro;
    diagnostics.hasYpr = fresh && g_hasYpr;
    diagnostics.hasQuaternion = fresh && g_hasQuaternion;
    for (int i = 0; i < 3; ++i) {
        diagnostics.accelBodyMps2[i] = g_lastAccelBodyMps2[i];
        diagnostics.gyroBodyRadPerSec[i] = g_lastGyroBodyRadPerSec[i];
        diagnostics.yprDeg[i] = g_lastYprDeg[i];
    }
    for (int i = 0; i < 4; ++i) {
        diagnostics.quaternion[i] = g_lastQuaternion[i];
    }
    return diagnostics;
}
