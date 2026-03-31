#include <Arduino.h>
#include <HardwareSerial.h>

extern "C" {
#include <interfaces/sbgInterface.h>
#include <sbgCommon.h>
#include <sbgECom.h>
#include <sbgEComLib.h>
}

#include "calibration_matrix.h"
#include "settings.h"

#ifndef ACS_BUILD_PULSE20_CALIBRATION
#error "Use the pulse20_calibration PlatformIO environment to build this target."
#endif

namespace {

constexpr uint8_t kSerialPortIndex = settings::sensors::ellipse20::kSerialPortIndex;
constexpr int8_t kRxPin = settings::sensors::ellipse20::kRxPin;
constexpr int8_t kTxPin = settings::sensors::ellipse20::kTxPin;
constexpr uint32_t kPulseBaudRate = settings::sensors::ellipse20::kBaudRate;
constexpr SbgEComOutputMode kImuOutputMode =
    static_cast<SbgEComOutputMode>(settings::sensors::ellipse20::kImuOutputMode);
constexpr SbgEComOutputMode kMagOutputMode =
    static_cast<SbgEComOutputMode>(settings::sensors::ellipse20::kMagOutputMode);
constexpr uint8_t kHandleBudgetPerLoop = 12;
constexpr uint32_t kConsoleBaud = 115200;
constexpr uint32_t kStreamIntervalMs = 100;
constexpr uint32_t kGyroCalibrationSamples = 1000;
constexpr size_t kMaxGyroTemperaturePoints = 8;
constexpr uint16_t kMaxMagSamples = 2048;
constexpr float kGToMps2 = 9.80665f;
constexpr float kRadToDeg = 57.295779513082320876f;

struct SerialInterfaceContext {
    HardwareSerial *serial = nullptr;
    uint32_t baudRate = 0;
};

SerialInterfaceContext g_serialContext;
SbgInterface g_interface;
SbgEComHandle g_comHandle;
bool g_deviceInitialized = false;

bool g_streamEnabled = true;
uint32_t g_lastStreamMs = 0;
bool g_haveImu = false;
bool g_haveMag = false;
uint32_t g_lastImuHostMicros = 0;
uint32_t g_lastMagHostMicros = 0;

float g_lastAccel[3] = {0.0f, 0.0f, 0.0f};
float g_lastGyro[3] = {0.0f, 0.0f, 0.0f};
float g_lastMag[3] = {0.0f, 0.0f, 0.0f};
float g_lastTemperatureC = 25.0f;

struct VectorStats {
    double sum[3] = {0.0, 0.0, 0.0};
    double sumSquares[3] = {0.0, 0.0, 0.0};
    double temperatureSum = 0.0;
    uint32_t count = 0;

    void Reset() {
        sum[0] = sum[1] = sum[2] = 0.0;
        sumSquares[0] = sumSquares[1] = sumSquares[2] = 0.0;
        temperatureSum = 0.0;
        count = 0;
    }

    void Add(float x, float y, float z, float temperatureC) {
        sum[0] += x;
        sum[1] += y;
        sum[2] += z;
        sumSquares[0] += static_cast<double>(x) * static_cast<double>(x);
        sumSquares[1] += static_cast<double>(y) * static_cast<double>(y);
        sumSquares[2] += static_cast<double>(z) * static_cast<double>(z);
        temperatureSum += temperatureC;
        ++count;
    }

    float Mean(size_t axis) const {
        return count > 0 ? static_cast<float>(sum[axis] / static_cast<double>(count)) : 0.0f;
    }

    float StdDev(size_t axis) const {
        if (count < 2) {
            return 0.0f;
        }
        const double mean = sum[axis] / static_cast<double>(count);
        const double variance = (sumSquares[axis] / static_cast<double>(count)) - (mean * mean);
        return variance > 0.0 ? static_cast<float>(sqrt(variance)) : 0.0f;
    }

    float MeanTemperature() const {
        return count > 0 ? static_cast<float>(temperatureSum / static_cast<double>(count)) : 25.0f;
    }
};

enum AccelFaceIndex {
    kFacePosX = 0,
    kFaceNegX,
    kFacePosY,
    kFaceNegY,
    kFacePosZ,
    kFaceNegZ,
    kFaceCount
};

struct AccelFaceCapture {
    const char *name = "";
    bool valid = false;
    float mean[3] = {0.0f, 0.0f, 0.0f};
    float temperatureC = 0.0f;
};

AccelFaceCapture g_accelFaces[kFaceCount] = {
    {" +X up", false, {0.0f, 0.0f, 0.0f}, 0.0f},
    {" -X up", false, {0.0f, 0.0f, 0.0f}, 0.0f},
    {" +Y up", false, {0.0f, 0.0f, 0.0f}, 0.0f},
    {" -Y up", false, {0.0f, 0.0f, 0.0f}, 0.0f},
    {" +Z up", false, {0.0f, 0.0f, 0.0f}, 0.0f},
    {" -Z up", false, {0.0f, 0.0f, 0.0f}, 0.0f},
};

bool g_gyroCalibrationActive = false;
VectorStats g_gyroCalibrationStats;

struct GyroTemperaturePoint {
    float temperatureC = 0.0f;
    float mean[3] = {0.0f, 0.0f, 0.0f};
    bool valid = false;
};

GyroTemperaturePoint g_gyroTemperaturePoints[kMaxGyroTemperaturePoints];
size_t g_gyroTemperaturePointCount = 0;

bool g_magCaptureActive = false;
bool g_magCaptureReady = false;
float g_magMin[3] = {0.0f, 0.0f, 0.0f};
float g_magMax[3] = {0.0f, 0.0f, 0.0f};
float g_magLast[3] = {0.0f, 0.0f, 0.0f};
uint32_t g_magSampleCount = 0;
float g_magSamples[kMaxMagSamples][3] = {};
uint16_t g_magStoredSamples = 0;

HardwareSerial *ResolveSerialPort(uint8_t portIndex) {
    switch (portIndex) {
        case 1: return &Serial1;
        case 2: return &Serial2;
        case 3: return &Serial3;
        case 4: return &Serial4;
        case 5: return &Serial5;
        case 6: return &Serial6;
        case 7: return &Serial7;
        case 8: return &Serial8;
        default: return nullptr;
    }
}

void ConfigureSerial(uint32_t baudRate) {
    if (g_serialContext.serial == nullptr) {
        return;
    }
    if (kRxPin >= 0 || kTxPin >= 0) {
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
    }
    g_serialContext.serial->begin(baudRate);
    g_serialContext.baudRate = baudRate;
}

void FlushInput() {
    if (g_serialContext.serial == nullptr) {
        return;
    }
    while (g_serialContext.serial->available() > 0) {
        g_serialContext.serial->read();
    }
}

SbgErrorCode SerialDestroy(SbgInterface *pInterface) {
    SBG_UNUSED_PARAMETER(pInterface);
    return SBG_NO_ERROR;
}

SbgErrorCode SerialWrite(SbgInterface *pInterface, const void *pBuffer, size_t bytesToWrite) {
    SBG_UNUSED_PARAMETER(pInterface);
    if (g_serialContext.serial == nullptr || bytesToWrite == 0) {
        return SBG_NO_ERROR;
    }
    const size_t written = g_serialContext.serial->write(static_cast<const uint8_t *>(pBuffer), bytesToWrite);
    return written == bytesToWrite ? SBG_NO_ERROR : SBG_WRITE_ERROR;
}

SbgErrorCode SerialRead(SbgInterface *pInterface, void *pBuffer, size_t *pReadBytes, size_t bytesToRead) {
    SBG_UNUSED_PARAMETER(pInterface);
    if (pReadBytes == nullptr) {
        return SBG_NULL_POINTER;
    }
    *pReadBytes = 0;
    if (g_serialContext.serial == nullptr || pBuffer == nullptr || bytesToRead == 0) {
        return SBG_NO_ERROR;
    }
    uint8_t *dst = static_cast<uint8_t *>(pBuffer);
    while ((*pReadBytes < bytesToRead) && (g_serialContext.serial->available() > 0)) {
        const int value = g_serialContext.serial->read();
        if (value < 0) {
            break;
        }
        dst[*pReadBytes] = static_cast<uint8_t>(value);
        ++(*pReadBytes);
    }
    return SBG_NO_ERROR;
}

SbgErrorCode SerialFlush(SbgInterface *pInterface, uint32_t flags) {
    SBG_UNUSED_PARAMETER(pInterface);
    if (g_serialContext.serial == nullptr) {
        return SBG_NO_ERROR;
    }
    if ((flags & SBG_IF_FLUSH_INPUT) != 0u) {
        FlushInput();
    }
    if ((flags & SBG_IF_FLUSH_OUTPUT) != 0u) {
        g_serialContext.serial->flush();
    }
    return SBG_NO_ERROR;
}

SbgErrorCode SerialSetSpeed(SbgInterface *pInterface, uint32_t speed) {
    SBG_UNUSED_PARAMETER(pInterface);
    ConfigureSerial(speed);
    return SBG_NO_ERROR;
}

uint32_t SerialGetSpeed(const SbgInterface *pInterface) {
    SBG_UNUSED_PARAMETER(pInterface);
    return g_serialContext.baudRate;
}

uint32_t SerialGetDelay(const SbgInterface *pInterface, size_t numBytes) {
    SBG_UNUSED_PARAMETER(pInterface);
    if (g_serialContext.baudRate == 0u) {
        return 0u;
    }
    return static_cast<uint32_t>((numBytes * 10000000ull) / g_serialContext.baudRate);
}

bool SetupInterface() {
    g_serialContext.serial = ResolveSerialPort(kSerialPortIndex);
    if (g_serialContext.serial == nullptr) {
        Serial.println("Pulse20: invalid serial port index");
        return false;
    }

    ConfigureSerial(kPulseBaudRate);
    FlushInput();

    sbgInterfaceZeroInit(&g_interface);
    g_interface.type = SBG_IF_TYPE_SERIAL;
    g_interface.handle = &g_serialContext;
    g_interface.pDestroyFunc = &SerialDestroy;
    g_interface.pWriteFunc = &SerialWrite;
    g_interface.pReadFunc = &SerialRead;
    g_interface.pFlushFunc = &SerialFlush;
    g_interface.pSetSpeedFunc = &SerialSetSpeed;
    g_interface.pGetSpeedFunc = &SerialGetSpeed;
    g_interface.pDelayFunc = &SerialGetDelay;
    sbgInterfaceNameSet(&g_interface, "Pulse20Cal");
    return true;
}

SbgErrorCode OnLogReceived(SbgEComHandle *pHandle,
                           SbgEComClass msgClass,
                           SbgEComMsgId msg,
                           const SbgEComLogUnion *pLogData,
                           void *pUserArg) {
    SBG_UNUSED_PARAMETER(pHandle);
    SBG_UNUSED_PARAMETER(pUserArg);

    if (msgClass != SBG_ECOM_CLASS_LOG_ECOM_0 || pLogData == nullptr) {
        return SBG_NO_ERROR;
    }

    const uint32_t nowUs = micros();
    switch (msg) {
        case SBG_ECOM_LOG_IMU_DATA:
            for (int i = 0; i < 3; ++i) {
                g_lastAccel[i] = pLogData->imuData.accelerometers[i];
                g_lastGyro[i] = pLogData->imuData.gyroscopes[i];
            }
            g_lastTemperatureC = pLogData->imuData.temperature;
            g_lastImuHostMicros = nowUs;
            g_haveImu = true;
            break;

        case SBG_ECOM_LOG_MAG:
            for (int i = 0; i < 3; ++i) {
                g_lastMag[i] = pLogData->magData.magnetometers[i];
                g_magLast[i] = g_lastMag[i];
            }
            g_lastMagHostMicros = nowUs;
            g_haveMag = true;
            break;

        default:
            break;
    }

    return SBG_NO_ERROR;
}

bool ConfigureOutputs() {
    const struct {
        SbgEComMsgId msgId;
        SbgEComOutputMode mode;
    } configs[] = {
        {SBG_ECOM_LOG_IMU_DATA, kImuOutputMode},
        {SBG_ECOM_LOG_MAG, kMagOutputMode},
    };

    bool ok = true;
    for (const auto &config : configs) {
        const SbgErrorCode errorCode =
            sbgEComCmdOutputSetConf(&g_comHandle,
                                    SBG_ECOM_OUTPUT_PORT_A,
                                    SBG_ECOM_CLASS_LOG_ECOM_0,
                                    config.msgId,
                                    config.mode);
        if (errorCode != SBG_NO_ERROR) {
            ok = false;
        }
    }
    return ok;
}

bool ConfigureDevice() {
    if (!SetupInterface()) {
        return false;
    }
    if (sbgEComInit(&g_comHandle, &g_interface) != SBG_NO_ERROR) {
        Serial.println("Pulse20: sbgECom init failed");
        return false;
    }
    sbgEComSetCmdTrialsAndTimeOut(&g_comHandle, 2u, 150u);
    sbgEComSetReceiveLogCallback(&g_comHandle, &OnLogReceived, nullptr);
    if (!ConfigureOutputs()) {
        Serial.println("Pulse20: output configuration warning");
    }
    g_deviceInitialized = true;
    return true;
}

void PollDevice() {
    if (!g_deviceInitialized) {
        return;
    }
    for (uint8_t i = 0; i < kHandleBudgetPerLoop; ++i) {
        const SbgErrorCode errorCode = sbgEComHandleOneLog(&g_comHandle);
        if (errorCode == SBG_NOT_READY) {
            break;
        }
        if (errorCode != SBG_NO_ERROR) {
            break;
        }
    }
}

void Print3x3(const float matrix[3][3]) {
    for (int row = 0; row < 3; ++row) {
        Serial.print("  {");
        Serial.print(matrix[row][0], 5);
        Serial.print("f, ");
        Serial.print(matrix[row][1], 5);
        Serial.print("f, ");
        Serial.print(matrix[row][2], 5);
        Serial.println("f},");
    }
}

void PrintSettingsInsertionGuide() {
    Serial.println("Paste target:");
    Serial.println("  file: src/settings.h");
    Serial.println("  section: namespace settings::sensors::ellipse20");
    Serial.println("Replace these constants with the values printed below:");
    Serial.println("  kGyroReferenceTemperatureC");
    Serial.println("  kGyroOffset[3]");
    Serial.println("  kAccelBias[3]");
    Serial.println("  kAccelAinv[3][3]");
    Serial.println("  kMountRotation[3][3]");
    Serial.println("  kMagBias[3]");
    Serial.println("  kMagAinv[3][3]");
    Serial.println("  kGyroTempBiasSlopeRadPerSecPerC[3]");
    Serial.println();
}

void PrintWorkflow() {
    Serial.println();
    Serial.println("Recommended workflow:");
    Serial.println("  1. Let the Pulse20 thermally settle on the bench.");
    Serial.println("  2. Run 'g' with the vehicle perfectly still.");
    Serial.println("  3. Put each accel axis up and capture x X y Y z Z.");
    Serial.println("  4. Run 'm', sweep all orientations slowly, then run 'm' again.");
    Serial.println("  5. Run 'p' for the paste-ready settings block.");
    Serial.println("Optional:");
    Serial.println("  Repeat 'g' at 2+ different temperatures to fit gyro temp slopes.");
    Serial.println("Useful detail commands:");
    Serial.println("  d = detailed capture dump");
    Serial.println("  w = print this workflow again");
    Serial.println();
}

void PrintHelp() {
    Serial.println();
    Serial.println("Pulse20 advanced calibration");
    Serial.println("Commands:");
    Serial.println("  h  : help");
    Serial.println("  w  : recommended workflow");
    Serial.println("  s  : toggle live stream");
    Serial.println("  c  : print one current sample");
    Serial.println("  g  : start gyro bias capture (repeat at different temps for slope fit)");
    Serial.println("  x  : capture accel face +X up");
    Serial.println("  X  : capture accel face -X up");
    Serial.println("  y  : capture accel face +Y up");
    Serial.println("  Y  : capture accel face -Y up");
    Serial.println("  z  : capture accel face +Z up");
    Serial.println("  Z  : capture accel face -Z up");
    Serial.println("  m  : toggle magnetometer sweep capture");
    Serial.println("  p  : print recommended settings block");
    Serial.println("  d  : print detailed capture dump and quality report");
    Serial.println("  r  : reset all captured calibration data");
    Serial.println();
    PrintWorkflow();
}

void ResetMagCapture() {
    g_magCaptureReady = false;
    g_magSampleCount = 0;
    for (int i = 0; i < 3; ++i) {
        g_magMin[i] = 0.0f;
        g_magMax[i] = 0.0f;
        g_magLast[i] = 0.0f;
    }
    g_magStoredSamples = 0;
}

void ResetCalibrationState() {
    g_gyroCalibrationActive = false;
    g_gyroCalibrationStats.Reset();
    g_gyroTemperaturePointCount = 0;
    for (size_t i = 0; i < kMaxGyroTemperaturePoints; ++i) {
        g_gyroTemperaturePoints[i].valid = false;
        g_gyroTemperaturePoints[i].temperatureC = 0.0f;
        g_gyroTemperaturePoints[i].mean[0] = 0.0f;
        g_gyroTemperaturePoints[i].mean[1] = 0.0f;
        g_gyroTemperaturePoints[i].mean[2] = 0.0f;
    }
    for (int i = 0; i < kFaceCount; ++i) {
        g_accelFaces[i].valid = false;
        g_accelFaces[i].mean[0] = 0.0f;
        g_accelFaces[i].mean[1] = 0.0f;
        g_accelFaces[i].mean[2] = 0.0f;
        g_accelFaces[i].temperatureC = 0.0f;
    }
    g_magCaptureActive = false;
    ResetMagCapture();
}

void PrintCurrentSample() {
    if (!g_haveImu) {
        Serial.println("No Pulse20 IMU sample yet.");
        return;
    }

    const float accelNormG = sqrtf((g_lastAccel[0] / kGToMps2) * (g_lastAccel[0] / kGToMps2) +
                                   (g_lastAccel[1] / kGToMps2) * (g_lastAccel[1] / kGToMps2) +
                                   (g_lastAccel[2] / kGToMps2) * (g_lastAccel[2] / kGToMps2));
    const float gyroNormRadPerSec =
        sqrtf(g_lastGyro[0] * g_lastGyro[0] + g_lastGyro[1] * g_lastGyro[1] + g_lastGyro[2] * g_lastGyro[2]);
    const float magNorm =
        sqrtf(g_magLast[0] * g_magLast[0] + g_magLast[1] * g_magLast[1] + g_magLast[2] * g_magLast[2]);

    Serial.print("acc_mps2=[");
    Serial.print(g_lastAccel[0], 4);
    Serial.print(", ");
    Serial.print(g_lastAccel[1], 4);
    Serial.print(", ");
    Serial.print(g_lastAccel[2], 4);
    Serial.print("] acc_g=[");
    Serial.print(g_lastAccel[0] / kGToMps2, 4);
    Serial.print(", ");
    Serial.print(g_lastAccel[1] / kGToMps2, 4);
    Serial.print(", ");
    Serial.print(g_lastAccel[2] / kGToMps2, 4);
    Serial.print("] gyr_radps=[");
    Serial.print(g_lastGyro[0], 5);
    Serial.print(", ");
    Serial.print(g_lastGyro[1], 5);
    Serial.print(", ");
    Serial.print(g_lastGyro[2], 5);
    Serial.print("] gyr_dps=[");
    Serial.print(g_lastGyro[0] * kRadToDeg, 4);
    Serial.print(", ");
    Serial.print(g_lastGyro[1] * kRadToDeg, 4);
    Serial.print(", ");
    Serial.print(g_lastGyro[2] * kRadToDeg, 4);
    Serial.print("] mag_raw=[");
    Serial.print(g_magLast[0], 4);
    Serial.print(", ");
    Serial.print(g_magLast[1], 4);
    Serial.print(", ");
    Serial.print(g_magLast[2], 4);
    Serial.print("] |acc|g=");
    Serial.print(accelNormG, 4);
    Serial.print(" |gyro|radps=");
    Serial.print(gyroNormRadPerSec, 4);
    Serial.print(" |mag|raw=");
    Serial.print(magNorm, 4);
    Serial.print(" temp_c=");
    Serial.println(g_lastTemperatureC, 2);
}

void CaptureAccelFace(AccelFaceIndex face) {
    if (!g_haveImu) {
        Serial.println("Cannot capture accel face before the first IMU sample.");
        return;
    }
    g_accelFaces[face].valid = true;
    g_accelFaces[face].mean[0] = g_lastAccel[0];
    g_accelFaces[face].mean[1] = g_lastAccel[1];
    g_accelFaces[face].mean[2] = g_lastAccel[2];
    g_accelFaces[face].temperatureC = g_lastTemperatureC;

    Serial.print("Captured accel face");
    Serial.print(g_accelFaces[face].name);
    Serial.print(" m/s^2=[");
    Serial.print(g_accelFaces[face].mean[0], 4);
    Serial.print(", ");
    Serial.print(g_accelFaces[face].mean[1], 4);
    Serial.print(", ");
    Serial.print(g_accelFaces[face].mean[2], 4);
    Serial.println("]");
}

void StartGyroCalibration() {
    if (!g_haveImu) {
        Serial.println("Cannot start gyro calibration before the first IMU sample.");
        return;
    }
    g_gyroCalibrationActive = true;
    g_gyroCalibrationStats.Reset();
    Serial.print("Starting gyro calibration for ");
    Serial.print(kGyroCalibrationSamples);
    Serial.println(" samples. Keep the vehicle still.");
}

void StoreGyroTemperaturePoint() {
    if (g_gyroCalibrationStats.count == 0) {
        return;
    }
    size_t slot = g_gyroTemperaturePointCount;
    if (slot >= kMaxGyroTemperaturePoints) {
        slot = kMaxGyroTemperaturePoints - 1;
        for (size_t i = 1; i < kMaxGyroTemperaturePoints; ++i) {
            g_gyroTemperaturePoints[i - 1] = g_gyroTemperaturePoints[i];
        }
    } else {
        ++g_gyroTemperaturePointCount;
    }

    g_gyroTemperaturePoints[slot].valid = true;
    g_gyroTemperaturePoints[slot].temperatureC = g_gyroCalibrationStats.MeanTemperature();
    g_gyroTemperaturePoints[slot].mean[0] = g_gyroCalibrationStats.Mean(0);
    g_gyroTemperaturePoints[slot].mean[1] = g_gyroCalibrationStats.Mean(1);
    g_gyroTemperaturePoints[slot].mean[2] = g_gyroCalibrationStats.Mean(2);
}

bool FitGyroTemperatureCompensation(float &referenceTemperatureC,
                                    float gyroOffset[3],
                                    float gyroTempSlopeRadPerSecPerC[3]) {
    if (g_gyroTemperaturePointCount == 0) {
        return false;
    }

    double tempSum = 0.0;
    for (size_t i = 0; i < g_gyroTemperaturePointCount; ++i) {
        tempSum += static_cast<double>(g_gyroTemperaturePoints[i].temperatureC);
    }
    referenceTemperatureC = static_cast<float>(tempSum / static_cast<double>(g_gyroTemperaturePointCount));

    for (int axis = 0; axis < 3; ++axis) {
        double biasSum = 0.0;
        double xx = 0.0;
        double xy = 0.0;
        for (size_t i = 0; i < g_gyroTemperaturePointCount; ++i) {
            const double centeredTemp = static_cast<double>(g_gyroTemperaturePoints[i].temperatureC) -
                                        static_cast<double>(referenceTemperatureC);
            const double bias = static_cast<double>(g_gyroTemperaturePoints[i].mean[axis]);
            biasSum += bias;
            xx += centeredTemp * centeredTemp;
            xy += centeredTemp * bias;
        }
        gyroOffset[axis] = static_cast<float>(biasSum / static_cast<double>(g_gyroTemperaturePointCount));
        gyroTempSlopeRadPerSecPerC[axis] = (xx > 1.0e-9) ? static_cast<float>(xy / xx) : 0.0f;
    }
    return true;
}

void UpdateGyroCalibration() {
    if (!g_gyroCalibrationActive || !g_haveImu) {
        return;
    }
    g_gyroCalibrationStats.Add(g_lastGyro[0], g_lastGyro[1], g_lastGyro[2], g_lastTemperatureC);
    if (g_gyroCalibrationStats.count < kGyroCalibrationSamples) {
        return;
    }

    g_gyroCalibrationActive = false;
    StoreGyroTemperaturePoint();
    Serial.println("Gyro calibration capture complete.");
    Serial.print("Gyro mean rad/s offsets = {");
    Serial.print(g_gyroCalibrationStats.Mean(0), 6);
    Serial.print("f, ");
    Serial.print(g_gyroCalibrationStats.Mean(1), 6);
    Serial.print("f, ");
    Serial.print(g_gyroCalibrationStats.Mean(2), 6);
    Serial.println("f};");
    Serial.print("Gyro stddev rad/s = {");
    Serial.print(g_gyroCalibrationStats.StdDev(0), 6);
    Serial.print(", ");
    Serial.print(g_gyroCalibrationStats.StdDev(1), 6);
    Serial.print(", ");
    Serial.print(g_gyroCalibrationStats.StdDev(2), 6);
    Serial.println("}");
    Serial.print("Mean calibration temperature C = ");
    Serial.println(g_gyroCalibrationStats.MeanTemperature(), 2);
    Serial.print("Stored gyro temp-fit points = ");
    Serial.println(static_cast<unsigned long>(g_gyroTemperaturePointCount));
}

void ToggleMagCapture() {
    if (!g_haveMag) {
        Serial.println("Cannot start mag capture before the first MAG sample.");
        return;
    }
    g_magCaptureActive = !g_magCaptureActive;
    if (g_magCaptureActive) {
        ResetMagCapture();
        Serial.println("Mag capture started. Rotate through as many orientations as possible, then press m again.");
        return;
    }
    Serial.print("Mag capture stopped after ");
    Serial.print(g_magSampleCount);
    Serial.println(" samples.");
}

void UpdateMagCapture() {
    if (!g_haveMag) {
        return;
    }
    if (!g_magCaptureActive) {
        return;
    }

    if (!g_magCaptureReady) {
        g_magMin[0] = g_magMax[0] = g_magLast[0];
        g_magMin[1] = g_magMax[1] = g_magLast[1];
        g_magMin[2] = g_magMax[2] = g_magLast[2];
        g_magCaptureReady = true;
    } else {
        g_magMin[0] = min(g_magMin[0], g_magLast[0]);
        g_magMin[1] = min(g_magMin[1], g_magLast[1]);
        g_magMin[2] = min(g_magMin[2], g_magLast[2]);
        g_magMax[0] = max(g_magMax[0], g_magLast[0]);
        g_magMax[1] = max(g_magMax[1], g_magLast[1]);
        g_magMax[2] = max(g_magMax[2], g_magLast[2]);
    }
    ++g_magSampleCount;
    if (g_magStoredSamples < kMaxMagSamples) {
        g_magSamples[g_magStoredSamples][0] = g_magLast[0];
        g_magSamples[g_magStoredSamples][1] = g_magLast[1];
        g_magSamples[g_magStoredSamples][2] = g_magLast[2];
        ++g_magStoredSamples;
    }
}

bool ComputeAccelCalibration(float bias[3], float ainv[3][3], float mountRotation[3][3]) {
    if (!(g_accelFaces[kFacePosX].valid && g_accelFaces[kFaceNegX].valid && g_accelFaces[kFacePosY].valid &&
          g_accelFaces[kFaceNegY].valid && g_accelFaces[kFacePosZ].valid && g_accelFaces[kFaceNegZ].valid)) {
        return false;
    }

    float totalCorrection[3][3];
    if (!calibration_matrix::BuildAccelTotalCorrection(g_accelFaces[kFacePosX].mean,
                                                       g_accelFaces[kFaceNegX].mean,
                                                       g_accelFaces[kFacePosY].mean,
                                                       g_accelFaces[kFaceNegY].mean,
                                                       g_accelFaces[kFacePosZ].mean,
                                                       g_accelFaces[kFaceNegZ].mean,
                                                       kGToMps2,
                                                       bias,
                                                       totalCorrection)) {
        return false;
    }

    return calibration_matrix::PolarDecomposeRight(totalCorrection, mountRotation, ainv);
}

bool ComputeMagCalibration(float bias[3], float ainv[3][3]) {
    if (!g_magCaptureReady || g_magStoredSamples < 32) {
        return false;
    }

    const float radiusX = 0.5f * (g_magMax[0] - g_magMin[0]);
    const float radiusY = 0.5f * (g_magMax[1] - g_magMin[1]);
    const float radiusZ = 0.5f * (g_magMax[2] - g_magMin[2]);
    if (!(radiusX > 1.0e-6f && radiusY > 1.0e-6f && radiusZ > 1.0e-6f)) {
        return false;
    }

    bias[0] = 0.5f * (g_magMax[0] + g_magMin[0]);
    bias[1] = 0.5f * (g_magMax[1] + g_magMin[1]);
    bias[2] = 0.5f * (g_magMax[2] + g_magMin[2]);

    return calibration_matrix::ComputeMagSoftIronFromSamples(g_magSamples, g_magStoredSamples, bias, ainv);
}

void PrintGyroTemperatureTable() {
    Serial.println("Gyro temperature points:");
    if (g_gyroTemperaturePointCount == 0) {
        Serial.println("  none");
        return;
    }
    for (size_t i = 0; i < g_gyroTemperaturePointCount; ++i) {
        Serial.print("  [");
        Serial.print(static_cast<unsigned long>(i));
        Serial.print("] temp_c=");
        Serial.print(g_gyroTemperaturePoints[i].temperatureC, 2);
        Serial.print(" mean_radps={");
        Serial.print(g_gyroTemperaturePoints[i].mean[0], 6);
        Serial.print(", ");
        Serial.print(g_gyroTemperaturePoints[i].mean[1], 6);
        Serial.print(", ");
        Serial.print(g_gyroTemperaturePoints[i].mean[2], 6);
        Serial.println("}");
    }
}

void PrintAccelFaceTable() {
    Serial.println("Accel face captures:");
    for (int i = 0; i < kFaceCount; ++i) {
        Serial.print("  ");
        Serial.print(g_accelFaces[i].name);
        if (!g_accelFaces[i].valid) {
            Serial.println(": missing");
            continue;
        }
        Serial.print(": m/s^2={");
        Serial.print(g_accelFaces[i].mean[0], 4);
        Serial.print(", ");
        Serial.print(g_accelFaces[i].mean[1], 4);
        Serial.print(", ");
        Serial.print(g_accelFaces[i].mean[2], 4);
        Serial.print("} temp_c=");
        Serial.println(g_accelFaces[i].temperatureC, 2);
    }
}

void PrintMagCaptureTable() {
    Serial.println("Mag capture:");
    if (!g_magCaptureReady) {
        Serial.println("  none");
        return;
    }
    Serial.print("  samples=");
    Serial.println(g_magSampleCount);
    Serial.print("  min={");
    Serial.print(g_magMin[0], 4);
    Serial.print(", ");
    Serial.print(g_magMin[1], 4);
    Serial.print(", ");
    Serial.print(g_magMin[2], 4);
    Serial.println("}");
    Serial.print("  max={");
    Serial.print(g_magMax[0], 4);
    Serial.print(", ");
    Serial.print(g_magMax[1], 4);
    Serial.print(", ");
    Serial.print(g_magMax[2], 4);
    Serial.println("}");
    Serial.print("  last={");
    Serial.print(g_magLast[0], 4);
    Serial.print(", ");
    Serial.print(g_magLast[1], 4);
    Serial.print(", ");
    Serial.print(g_magLast[2], 4);
    Serial.println("}");
}

void PrintQualityAssessment() {
    float accelBias[3] = {0.0f, 0.0f, 0.0f};
    float accelAinv[3][3];
    float mountRotation[3][3];
    float magBias[3] = {0.0f, 0.0f, 0.0f};
    float magAinv[3][3];
    calibration_matrix::SetIdentity3(accelAinv);
    calibration_matrix::SetIdentity3(mountRotation);
    calibration_matrix::SetIdentity3(magAinv);
    const bool haveAccel = ComputeAccelCalibration(accelBias, accelAinv, mountRotation);
    const bool haveMag = ComputeMagCalibration(magBias, magAinv);

    Serial.println("Quality report:");
    if (g_gyroCalibrationStats.count > 0) {
        Serial.print("  gyro: OK, ");
        Serial.print(g_gyroCalibrationStats.count);
        Serial.print(" samples, stddev rad/s={");
        Serial.print(g_gyroCalibrationStats.StdDev(0), 6);
        Serial.print(", ");
        Serial.print(g_gyroCalibrationStats.StdDev(1), 6);
        Serial.print(", ");
        Serial.print(g_gyroCalibrationStats.StdDev(2), 6);
        Serial.println("}");
    } else {
        Serial.println("  gyro: missing");
    }
    if (g_gyroTemperaturePointCount >= 2) {
        Serial.print("  gyro temp fit: OK, points=");
        Serial.println(static_cast<unsigned long>(g_gyroTemperaturePointCount));
    } else {
        Serial.print("  gyro temp fit: weak, points=");
        Serial.println(static_cast<unsigned long>(g_gyroTemperaturePointCount));
    }
    Serial.print("  accel: ");
    Serial.println(haveAccel ? "OK" : "incomplete, need x X y Y z Z");
    Serial.print("  mag: ");
    if (haveMag) {
        Serial.print("OK, samples=");
        Serial.println(g_magSampleCount);
    } else if (g_magCaptureReady) {
        Serial.print("weak sweep, samples=");
        Serial.println(g_magSampleCount);
    } else {
        Serial.println("missing, run m sweep");
    }
    Serial.println();
}

void PrintDetailedDump() {
    Serial.println();
    Serial.println("==== Pulse20 detailed calibration dump ====");
    PrintSettingsInsertionGuide();
    PrintQualityAssessment();
    PrintGyroTemperatureTable();
    PrintAccelFaceTable();
    PrintMagCaptureTable();
    Serial.println("==== End detailed dump ====");
    Serial.println();
}

void PrintRecommendedSettings() {
    float accelBias[3] = {0.0f, 0.0f, 0.0f};
    float accelAinv[3][3];
    float mountRotation[3][3];
    float magBias[3] = {0.0f, 0.0f, 0.0f};
    float magAinv[3][3];
    float gyroReferenceTemperatureC = g_gyroCalibrationStats.MeanTemperature();
    float gyroOffset[3] = {
        g_gyroCalibrationStats.Mean(0),
        g_gyroCalibrationStats.Mean(1),
        g_gyroCalibrationStats.Mean(2),
    };
    float gyroTempSlopeRadPerSecPerC[3] = {0.0f, 0.0f, 0.0f};
    calibration_matrix::SetIdentity3(accelAinv);
    calibration_matrix::SetIdentity3(mountRotation);
    calibration_matrix::SetIdentity3(magAinv);
    const bool haveAccel = ComputeAccelCalibration(accelBias, accelAinv, mountRotation);
    const bool haveMag = ComputeMagCalibration(magBias, magAinv);
    const bool haveGyroFit =
        FitGyroTemperatureCompensation(gyroReferenceTemperatureC, gyroOffset, gyroTempSlopeRadPerSecPerC);

    Serial.println();
    Serial.println("==== BEGIN paste into src/settings.h / settings::sensors::ellipse20 ====");
    PrintSettingsInsertionGuide();
    Serial.print("constexpr float kGyroReferenceTemperatureC = ");
    Serial.print(gyroReferenceTemperatureC, 2);
    Serial.println("f;");
    Serial.print("constexpr float kGyroOffset[3] = {");
    Serial.print(gyroOffset[0], 6);
    Serial.print("f, ");
    Serial.print(gyroOffset[1], 6);
    Serial.print("f, ");
    Serial.print(gyroOffset[2], 6);
    Serial.println("f};");

    if (haveAccel) {
        Serial.print("constexpr float kAccelBias[3] = {");
        Serial.print(accelBias[0], 5);
        Serial.print("f, ");
        Serial.print(accelBias[1], 5);
        Serial.print("f, ");
        Serial.print(accelBias[2], 5);
        Serial.println("f};");
        Serial.println("constexpr float kAccelAinv[3][3] = {");
        Print3x3(accelAinv);
        Serial.println("};");
        Serial.println("constexpr float kMountRotation[3][3] = {");
        Print3x3(mountRotation);
        Serial.println("};");
    } else {
        Serial.println("// Accel calibration incomplete. Capture +X/-X/+Y/-Y/+Z/-Z first.");
    }

    if (haveMag) {
        Serial.print("constexpr float kMagBias[3] = {");
        Serial.print(magBias[0], 5);
        Serial.print("f, ");
        Serial.print(magBias[1], 5);
        Serial.print("f, ");
        Serial.print(magBias[2], 5);
        Serial.println("f};");
        Serial.println("constexpr float kMagAinv[3][3] = {");
        Print3x3(magAinv);
        Serial.println("};");
    } else {
        Serial.println("// Mag calibration incomplete. Run a full sweep capture with m.");
    }

    if (haveGyroFit && g_gyroTemperaturePointCount >= 2) {
        Serial.print("constexpr float kGyroTempBiasSlopeRadPerSecPerC[3] = {");
        Serial.print(gyroTempSlopeRadPerSecPerC[0], 8);
        Serial.print("f, ");
        Serial.print(gyroTempSlopeRadPerSecPerC[1], 8);
        Serial.print("f, ");
        Serial.print(gyroTempSlopeRadPerSecPerC[2], 8);
        Serial.println("f};");
    } else {
        Serial.println("// Capture gyro bias at two or more temperatures with repeated g runs to fit temp slope.");
        Serial.println("constexpr float kGyroTempBiasSlopeRadPerSecPerC[3] = {0.0f, 0.0f, 0.0f};");
    }
    Serial.println("==== END paste block ====");
    Serial.println();
}

void PrintCaptureSummary() {
    Serial.println();
    Serial.println("Calibration summary:");
    if (g_gyroCalibrationStats.count > 0) {
        Serial.print("  gyro samples: ");
        Serial.print(g_gyroCalibrationStats.count);
        Serial.print(" at ");
        Serial.print(g_gyroCalibrationStats.MeanTemperature(), 2);
        Serial.println(" C");
        Serial.print("  gyro temp-fit points: ");
        Serial.println(static_cast<unsigned long>(g_gyroTemperaturePointCount));
    } else {
        Serial.println("  gyro samples: none");
    }

    for (int i = 0; i < kFaceCount; ++i) {
        Serial.print("  accel");
        Serial.print(g_accelFaces[i].name);
        Serial.print(": ");
        Serial.println(g_accelFaces[i].valid ? "captured" : "missing");
    }

    Serial.print("  mag sweep: ");
    if (g_magCaptureReady) {
        Serial.print(g_magSampleCount);
        Serial.println(" samples");
    } else {
        Serial.println("none");
    }
    Serial.println();
}

void HandleCommand(char command) {
    switch (command) {
        case 'h':
        case '?':
            PrintHelp();
            break;
        case 'w':
            PrintWorkflow();
            break;
        case 's':
            g_streamEnabled = !g_streamEnabled;
            Serial.print("Live stream ");
            Serial.println(g_streamEnabled ? "enabled" : "disabled");
            break;
        case 'c':
            PrintCurrentSample();
            break;
        case 'g':
            StartGyroCalibration();
            break;
        case 'x':
            CaptureAccelFace(kFacePosX);
            break;
        case 'X':
            CaptureAccelFace(kFaceNegX);
            break;
        case 'y':
            CaptureAccelFace(kFacePosY);
            break;
        case 'Y':
            CaptureAccelFace(kFaceNegY);
            break;
        case 'z':
            CaptureAccelFace(kFacePosZ);
            break;
        case 'Z':
            CaptureAccelFace(kFaceNegZ);
            break;
        case 'm':
            ToggleMagCapture();
            break;
        case 'p':
            PrintCaptureSummary();
            PrintRecommendedSettings();
            break;
        case 'd':
            PrintDetailedDump();
            PrintCaptureSummary();
            PrintRecommendedSettings();
            break;
        case 'r':
            ResetCalibrationState();
            Serial.println("Calibration captures reset.");
            break;
        case '\n':
        case '\r':
            break;
        default:
            Serial.print("Unknown command: ");
            Serial.println(command);
            PrintHelp();
            break;
    }
}

}  // namespace

void setup() {
    Serial.begin(kConsoleBaud);
    while (!Serial) {
        delay(10);
    }

    Serial.println();
    Serial.println("Pulse20 calibration target");
    Serial.print("Pulse serial port index: ");
    Serial.print(kSerialPortIndex);
    Serial.print(" baud: ");
    Serial.println(kPulseBaudRate);

    if (!ConfigureDevice()) {
        Serial.println("Pulse20 init failed. Check serial port, baud rate, and wiring.");
        while (true) {
            delay(1000);
        }
    }

    ResetCalibrationState();
    PrintHelp();
}

void loop() {
    while (Serial.available() > 0) {
        HandleCommand(static_cast<char>(Serial.read()));
    }

    PollDevice();
    UpdateGyroCalibration();
    UpdateMagCapture();

    const uint32_t nowMs = millis();
    if (!g_streamEnabled || (nowMs - g_lastStreamMs) < kStreamIntervalMs) {
        delay(1);
        return;
    }
    g_lastStreamMs = nowMs;

    PrintCurrentSample();
    if (g_magCaptureActive) {
        Serial.print("mag_capture samples=");
        Serial.print(g_magSampleCount);
        Serial.print(" min=[");
        Serial.print(g_magMin[0], 4);
        Serial.print(", ");
        Serial.print(g_magMin[1], 4);
        Serial.print(", ");
        Serial.print(g_magMin[2], 4);
        Serial.print("] max=[");
        Serial.print(g_magMax[0], 4);
        Serial.print(", ");
        Serial.print(g_magMax[1], 4);
        Serial.print(", ");
        Serial.print(g_magMax[2], 4);
        Serial.println("]");
    }
    if (g_gyroCalibrationActive) {
        Serial.print("gyro_capture progress=");
        Serial.print(g_gyroCalibrationStats.count);
        Serial.print("/");
        Serial.println(kGyroCalibrationSamples);
    }
}
