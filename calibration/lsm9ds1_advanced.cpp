#include <Arduino.h>
#include <SPI.h>

#include <SparkFunLSM9DS1.h>

#include "calibration_matrix.h"
#include "settings.h"

#ifndef ACS_BUILD_LSM9DS1_CALIBRATION
#error "Use the lsm9ds1_calibration PlatformIO environment to build this target."
#endif

namespace {

constexpr uint8_t kAccelGyroChipSelectPin = settings::sensors::lsm9ds1::kAccelGyroChipSelectPin;
constexpr uint8_t kMagChipSelectPin = settings::sensors::lsm9ds1::kMagChipSelectPin;
constexpr uint8_t kAccelRangeG = settings::sensors::lsm9ds1::kCalibrationAccelRangeG;
constexpr uint16_t kGyroRangeDps = settings::sensors::lsm9ds1::kCalibrationGyroRangeDps;
constexpr uint8_t kMagRangeGauss = settings::sensors::lsm9ds1::kCalibrationMagRangeGauss;
constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kStreamIntervalMs = 100;
constexpr uint32_t kGyroCalibrationSamples = 1000;
constexpr size_t kMaxGyroTemperaturePoints = 8;
constexpr uint16_t kMaxMagSamples = 2048;

LSM9DS1 g_lsm;
bool g_streamEnabled = true;
uint32_t g_lastStreamMs = 0;
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

float AccelLsbPerGForRange(uint8_t rangeG) {
    switch (rangeG) {
        case 2:
            return 16384.0f;
        case 4:
            return 8192.0f;
        case 8:
            return 4096.0f;
        case 16:
            return 2048.0f;
        default:
            return 16384.0f;
    }
}

float GyroLsbPerDpsForRange(uint16_t rangeDps) {
    switch (rangeDps) {
        case 245:
            return 114.285714f;
        case 500:
            return 57.142857f;
        case 2000:
            return 14.285714f;
        default:
            return 114.285714f;
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
    Serial.println("  section: namespace settings::sensors::lsm9ds1");
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
    Serial.println("  1. Let the board thermally settle on the bench.");
    Serial.println("  2. Run 'g' with the board perfectly still.");
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
    Serial.println("LSM9DS1 advanced calibration");
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

bool ConfigureSensor() {
    g_lsm.settings.device.commInterface = IMU_MODE_SPI;
    g_lsm.settings.device.agAddress = kAccelGyroChipSelectPin;
    g_lsm.settings.device.mAddress = kMagChipSelectPin;
    g_lsm.settings.accel.scale = kAccelRangeG;
    g_lsm.settings.gyro.scale = kGyroRangeDps;
    g_lsm.settings.mag.scale = kMagRangeGauss;
    g_lsm.settings.gyro.sampleRate = settings::sensors::lsm9ds1::kGyroSampleRateSetting;
    g_lsm.settings.accel.sampleRate = settings::sensors::lsm9ds1::kAccelSampleRateSetting;
    g_lsm.settings.mag.sampleRate = settings::sensors::lsm9ds1::kMagSampleRateSetting;
    SPI.begin();
    return g_lsm.beginSPI(kAccelGyroChipSelectPin, kMagChipSelectPin) != 0;
}

void UpdateTemperature() {
    if (g_lsm.tempAvailable()) {
        g_lsm.readTemp();
        g_lastTemperatureC = static_cast<float>(g_lsm.temperature);
    }
}

void UpdateSensors() {
    if (g_lsm.accelAvailable()) {
        g_lsm.readAccel();
    }
    if (g_lsm.gyroAvailable()) {
        g_lsm.readGyro();
    }
    if (g_lsm.magAvailable()) {
        g_lsm.readMag();
    }
    UpdateTemperature();
}

void PrintCurrentSample() {
    const float ax = static_cast<float>(g_lsm.ax);
    const float ay = static_cast<float>(g_lsm.ay);
    const float az = static_cast<float>(g_lsm.az);
    const float gx = static_cast<float>(g_lsm.gx);
    const float gy = static_cast<float>(g_lsm.gy);
    const float gz = static_cast<float>(g_lsm.gz);
    const float mx = static_cast<float>(g_lsm.mx);
    const float my = static_cast<float>(g_lsm.my);
    const float mz = static_cast<float>(g_lsm.mz);
    const float accelScale = 1.0f / AccelLsbPerGForRange(kAccelRangeG);
    const float gyroScale = 1.0f / GyroLsbPerDpsForRange(kGyroRangeDps);
    const float accelNorm = sqrtf((ax * accelScale) * (ax * accelScale) +
                                  (ay * accelScale) * (ay * accelScale) +
                                  (az * accelScale) * (az * accelScale));
    const float gyroNorm = sqrtf((gx * gyroScale) * (gx * gyroScale) +
                                 (gy * gyroScale) * (gy * gyroScale) +
                                 (gz * gyroScale) * (gz * gyroScale));
    const float magNorm = sqrtf(mx * mx + my * my + mz * mz);

    Serial.print("acc_raw=[");
    Serial.print(ax, 2);
    Serial.print(", ");
    Serial.print(ay, 2);
    Serial.print(", ");
    Serial.print(az, 2);
    Serial.print("] acc_g=[");
    Serial.print(ax * accelScale, 4);
    Serial.print(", ");
    Serial.print(ay * accelScale, 4);
    Serial.print(", ");
    Serial.print(az * accelScale, 4);
    Serial.print("] gyr_raw=[");
    Serial.print(gx, 2);
    Serial.print(", ");
    Serial.print(gy, 2);
    Serial.print(", ");
    Serial.print(gz, 2);
    Serial.print("] gyr_dps=[");
    Serial.print(gx * gyroScale, 4);
    Serial.print(", ");
    Serial.print(gy * gyroScale, 4);
    Serial.print(", ");
    Serial.print(gz * gyroScale, 4);
    Serial.print("] mag_raw=[");
    Serial.print(mx, 2);
    Serial.print(", ");
    Serial.print(my, 2);
    Serial.print(", ");
    Serial.print(mz, 2);
    Serial.print("] |acc|g=");
    Serial.print(accelNorm, 4);
    Serial.print(" |gyro|dps=");
    Serial.print(gyroNorm, 4);
    Serial.print(" |mag|raw=");
    Serial.print(magNorm, 2);
    Serial.print(" temp_c=");
    Serial.println(g_lastTemperatureC, 2);
}

void CaptureAccelFace(AccelFaceIndex face) {
    g_accelFaces[face].valid = true;
    g_accelFaces[face].mean[0] = static_cast<float>(g_lsm.ax);
    g_accelFaces[face].mean[1] = static_cast<float>(g_lsm.ay);
    g_accelFaces[face].mean[2] = static_cast<float>(g_lsm.az);
    g_accelFaces[face].temperatureC = g_lastTemperatureC;

    Serial.print("Captured accel face");
    Serial.print(g_accelFaces[face].name);
    Serial.print(" raw=[");
    Serial.print(g_accelFaces[face].mean[0], 2);
    Serial.print(", ");
    Serial.print(g_accelFaces[face].mean[1], 2);
    Serial.print(", ");
    Serial.print(g_accelFaces[face].mean[2], 2);
    Serial.println("]");
}

void StartGyroCalibration() {
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
                                    float gyroOffsetRaw[3],
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
        gyroOffsetRaw[axis] = static_cast<float>(biasSum / static_cast<double>(g_gyroTemperaturePointCount));
        const float slopeCountsPerC = (xx > 1.0e-9) ? static_cast<float>(xy / xx) : 0.0f;
        gyroTempSlopeRadPerSecPerC[axis] = slopeCountsPerC *
                                           ((1.0f / GyroLsbPerDpsForRange(kGyroRangeDps)) *
                                            (3.14159265358979323846f / 180.0f));
    }
    return true;
}

void UpdateGyroCalibration() {
    if (!g_gyroCalibrationActive) {
        return;
    }
    g_gyroCalibrationStats.Add(static_cast<float>(g_lsm.gx),
                               static_cast<float>(g_lsm.gy),
                               static_cast<float>(g_lsm.gz),
                               g_lastTemperatureC);
    if (g_gyroCalibrationStats.count < kGyroCalibrationSamples) {
        return;
    }

    g_gyroCalibrationActive = false;
    StoreGyroTemperaturePoint();
    Serial.println("Gyro calibration capture complete.");
    Serial.print("Gyro mean raw offsets = {");
    Serial.print(g_gyroCalibrationStats.Mean(0), 2);
    Serial.print("f, ");
    Serial.print(g_gyroCalibrationStats.Mean(1), 2);
    Serial.print("f, ");
    Serial.print(g_gyroCalibrationStats.Mean(2), 2);
    Serial.println("f};");
    Serial.print("Gyro stddev raw = {");
    Serial.print(g_gyroCalibrationStats.StdDev(0), 2);
    Serial.print(", ");
    Serial.print(g_gyroCalibrationStats.StdDev(1), 2);
    Serial.print(", ");
    Serial.print(g_gyroCalibrationStats.StdDev(2), 2);
    Serial.println("}");
    Serial.print("Mean calibration temperature C = ");
    Serial.println(g_gyroCalibrationStats.MeanTemperature(), 2);
    Serial.print("Stored gyro temp-fit points = ");
    Serial.println(static_cast<unsigned long>(g_gyroTemperaturePointCount));
}

void ToggleMagCapture() {
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
    g_magLast[0] = static_cast<float>(g_lsm.mx);
    g_magLast[1] = static_cast<float>(g_lsm.my);
    g_magLast[2] = static_cast<float>(g_lsm.mz);
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

    const float target = AccelLsbPerGForRange(kAccelRangeG);
    float totalCorrection[3][3];
    if (!calibration_matrix::BuildAccelTotalCorrection(g_accelFaces[kFacePosX].mean,
                                                       g_accelFaces[kFaceNegX].mean,
                                                       g_accelFaces[kFacePosY].mean,
                                                       g_accelFaces[kFaceNegY].mean,
                                                       g_accelFaces[kFacePosZ].mean,
                                                       g_accelFaces[kFaceNegZ].mean,
                                                       target,
                                                       bias,
                                                       totalCorrection)) {
        return false;
    }

    float axisTransform[3][3];
    float axisTransformT[3][3];
    float preAxisCorrection[3][3];
    calibration_matrix::BuildAxisTransform(settings::sensors::lsm9ds1::kAxisMap,
                                           settings::sensors::lsm9ds1::kAxisSign,
                                           axisTransform);
    calibration_matrix::Transpose3x3(axisTransform, axisTransformT);
    calibration_matrix::Multiply3x3(totalCorrection, axisTransformT, preAxisCorrection);
    return calibration_matrix::PolarDecomposeRight(preAxisCorrection, mountRotation, ainv);
}

bool ComputeMagCalibration(float bias[3], float ainv[3][3]) {
    if (!g_magCaptureReady || g_magStoredSamples < 32) {
        return false;
    }

    const float radiusX = 0.5f * (g_magMax[0] - g_magMin[0]);
    const float radiusY = 0.5f * (g_magMax[1] - g_magMin[1]);
    const float radiusZ = 0.5f * (g_magMax[2] - g_magMin[2]);
    if (!(radiusX > 1.0f && radiusY > 1.0f && radiusZ > 1.0f)) {
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
        Serial.print(" mean_raw={");
        Serial.print(g_gyroTemperaturePoints[i].mean[0], 2);
        Serial.print(", ");
        Serial.print(g_gyroTemperaturePoints[i].mean[1], 2);
        Serial.print(", ");
        Serial.print(g_gyroTemperaturePoints[i].mean[2], 2);
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
        Serial.print(": raw={");
        Serial.print(g_accelFaces[i].mean[0], 2);
        Serial.print(", ");
        Serial.print(g_accelFaces[i].mean[1], 2);
        Serial.print(", ");
        Serial.print(g_accelFaces[i].mean[2], 2);
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
    Serial.print(g_magMin[0], 2);
    Serial.print(", ");
    Serial.print(g_magMin[1], 2);
    Serial.print(", ");
    Serial.print(g_magMin[2], 2);
    Serial.println("}");
    Serial.print("  max={");
    Serial.print(g_magMax[0], 2);
    Serial.print(", ");
    Serial.print(g_magMax[1], 2);
    Serial.print(", ");
    Serial.print(g_magMax[2], 2);
    Serial.println("}");
    Serial.print("  last={");
    Serial.print(g_magLast[0], 2);
    Serial.print(", ");
    Serial.print(g_magLast[1], 2);
    Serial.print(", ");
    Serial.print(g_magLast[2], 2);
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
        Serial.print(" samples, stddev raw={");
        Serial.print(g_gyroCalibrationStats.StdDev(0), 2);
        Serial.print(", ");
        Serial.print(g_gyroCalibrationStats.StdDev(1), 2);
        Serial.print(", ");
        Serial.print(g_gyroCalibrationStats.StdDev(2), 2);
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
    Serial.println("==== LSM detailed calibration dump ====");
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
    float gyroOffsetRaw[3] = {
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
    const bool haveGyroFit = FitGyroTemperatureCompensation(
        gyroReferenceTemperatureC, gyroOffsetRaw, gyroTempSlopeRadPerSecPerC);

    Serial.println();
    Serial.println("==== BEGIN paste into src/settings.h / settings::sensors::lsm9ds1 ====");
    PrintSettingsInsertionGuide();
    Serial.print("constexpr uint8_t kCalibrationAccelRangeG = ");
    Serial.print(kAccelRangeG);
    Serial.println(";");
    Serial.print("constexpr uint16_t kCalibrationGyroRangeDps = ");
    Serial.print(kGyroRangeDps);
    Serial.println(";");
    Serial.print("constexpr uint8_t kCalibrationMagRangeGauss = ");
    Serial.print(kMagRangeGauss);
    Serial.println(";");
    Serial.print("constexpr float kGyroReferenceTemperatureC = ");
    Serial.print(gyroReferenceTemperatureC, 2);
    Serial.println("f;");
    Serial.print("constexpr float kGyroOffset[3] = {");
    Serial.print(gyroOffsetRaw[0], 2);
    Serial.print("f, ");
    Serial.print(gyroOffsetRaw[1], 2);
    Serial.print("f, ");
    Serial.print(gyroOffsetRaw[2], 2);
    Serial.println("f};");

    if (haveAccel) {
        Serial.print("constexpr float kAccelBias[3] = {");
        Serial.print(accelBias[0], 2);
        Serial.print("f, ");
        Serial.print(accelBias[1], 2);
        Serial.print("f, ");
        Serial.print(accelBias[2], 2);
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
        Serial.print(magBias[0], 2);
        Serial.print("f, ");
        Serial.print(magBias[1], 2);
        Serial.print("f, ");
        Serial.print(magBias[2], 2);
        Serial.println("f};");
        Serial.println("constexpr float kMagAinv[3][3] = {");
        Print3x3(magAinv);
        Serial.println("};");
    } else {
        Serial.println("// Mag calibration incomplete. Run a full sweep capture with m.");
    }

    Serial.print("// Current axis map/sign = {");
    Serial.print(settings::sensors::lsm9ds1::kAxisMap[0]);
    Serial.print(", ");
    Serial.print(settings::sensors::lsm9ds1::kAxisMap[1]);
    Serial.print(", ");
    Serial.print(settings::sensors::lsm9ds1::kAxisMap[2]);
    Serial.print("} / {");
    Serial.print(settings::sensors::lsm9ds1::kAxisSign[0]);
    Serial.print(", ");
    Serial.print(settings::sensors::lsm9ds1::kAxisSign[1]);
    Serial.print(", ");
    Serial.print(settings::sensors::lsm9ds1::kAxisSign[2]);
    Serial.println("}");
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
    Serial.begin(kSerialBaud);
    while (!Serial) {
        delay(10);
    }

    Serial.println();
    Serial.println("LSM9DS1 calibration target");
    Serial.print("AG CS pin: ");
    Serial.print(kAccelGyroChipSelectPin);
    Serial.print(" M CS pin: ");
    Serial.println(kMagChipSelectPin);

    if (!ConfigureSensor()) {
        Serial.println("LSM9DS1 init failed. Check wiring and SPI bus.");
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

    if (!(g_lsm.accelAvailable() || g_lsm.gyroAvailable() || g_lsm.magAvailable() || g_lsm.tempAvailable())) {
        delay(1);
        return;
    }

    UpdateSensors();
    UpdateGyroCalibration();
    UpdateMagCapture();

    const uint32_t nowMs = millis();
    if (!g_streamEnabled || (nowMs - g_lastStreamMs) < kStreamIntervalMs) {
        return;
    }
    g_lastStreamMs = nowMs;

    PrintCurrentSample();
    if (g_magCaptureActive) {
        Serial.print("mag_capture samples=");
        Serial.print(g_magSampleCount);
        Serial.print(" min=[");
        Serial.print(g_magMin[0], 2);
        Serial.print(", ");
        Serial.print(g_magMin[1], 2);
        Serial.print(", ");
        Serial.print(g_magMin[2], 2);
        Serial.print("] max=[");
        Serial.print(g_magMax[0], 2);
        Serial.print(", ");
        Serial.print(g_magMax[1], 2);
        Serial.print(", ");
        Serial.print(g_magMax[2], 2);
        Serial.println("]");
    }
    if (g_gyroCalibrationActive) {
        Serial.print("gyro_capture progress=");
        Serial.print(g_gyroCalibrationStats.count);
        Serial.print("/");
        Serial.println(kGyroCalibrationSamples);
    }
}
