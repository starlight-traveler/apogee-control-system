#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <Adafruit_BNO055.h>
#include <ICM_20948.h>
#include <SparkFunLSM9DS1.h>

#include "ellipse_20.h"
#include "settings.h"

#ifndef ACS_BUILD_ORIENTATION_CHECK
#error "Use the orientation_check PlatformIO environment to build this target."
#endif

namespace {

constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kStreamIntervalMs = 300;
constexpr uint32_t kCaptureDurationMs = 600;
constexpr float kBnoMetersPerSecondSquaredPerG = 9.80665f;

Adafruit_BNO055 g_bno(55, settings::sensors::bno055::kI2cAddress, &Wire);
ICM_20948_SPI g_icm;
LSM9DS1 g_lsm;

bool g_bnoOnline = false;
bool g_icmOnline = false;
bool g_lsmOnline = false;
bool g_pulseOnline = false;
bool g_streamEnabled = true;
uint32_t g_lastStreamMs = 0;

struct LiveRailSample {
    bool valid = false;
    uint32_t samples = 0;
    float rawG[3] = {0.0f, 0.0f, 0.0f};
    float bodyG[3] = {0.0f, 0.0f, 0.0f};
};

struct CaptureStats {
    double rawSum[3] = {0.0, 0.0, 0.0};
    double bodySum[3] = {0.0, 0.0, 0.0};
    uint32_t count = 0;

    void Add(const float raw[3], const float body[3]) {
        for (int i = 0; i < 3; ++i) {
            rawSum[i] += raw[i];
            bodySum[i] += body[i];
        }
        ++count;
    }

    bool Ready() const { return count > 0; }

    void MeanRaw(float out[3]) const {
        if (count == 0) {
            out[0] = out[1] = out[2] = 0.0f;
            return;
        }
        const double invCount = 1.0 / static_cast<double>(count);
        for (int i = 0; i < 3; ++i) {
            out[i] = static_cast<float>(rawSum[i] * invCount);
        }
    }

    void MeanBody(float out[3]) const {
        if (count == 0) {
            out[0] = out[1] = out[2] = 0.0f;
            return;
        }
        const double invCount = 1.0 / static_cast<double>(count);
        for (int i = 0; i < 3; ++i) {
            out[i] = static_cast<float>(bodySum[i] * invCount);
        }
    }
};

struct RailCapture {
    const char *name = "";
    bool online = false;
    CaptureStats stats;
};

struct CaptureBundle {
    RailCapture bno{"BNO", false, {}};
    RailCapture icm{"ICM", false, {}};
    RailCapture lsm{"LSM", false, {}};
    RailCapture pulse{"PULSE", false, {}};
};

struct PoseExpectation {
    char command = 'c';
    const char *label = "";
    float expectedUnit[3] = {0.0f, 0.0f, 0.0f};
};

LiveRailSample g_liveBno;
LiveRailSample g_liveIcm;
LiveRailSample g_liveLsm;
LiveRailSample g_livePulse;

constexpr PoseExpectation kPoseExpectations[] = {
    {'x', "body +X up", {1.0f, 0.0f, 0.0f}},
    {'X', "body -X up", {-1.0f, 0.0f, 0.0f}},
    {'y', "body +Y up", {0.0f, 1.0f, 0.0f}},
    {'Y', "body -Y up", {0.0f, -1.0f, 0.0f}},
    {'z', "body +Z up", {0.0f, 0.0f, 1.0f}},
    {'Z', "body -Z up", {0.0f, 0.0f, -1.0f}},
};

float ClampUnit(float value) {
    if (value > 1.0f) {
        return 1.0f;
    }
    if (value < -1.0f) {
        return -1.0f;
    }
    return value;
}

void Apply3x3(const float matrix[3][3], const float in[3], float out[3]) {
    const float x = in[0];
    const float y = in[1];
    const float z = in[2];
    out[0] = matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z;
    out[1] = matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z;
    out[2] = matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z;
}

void BuildAxisTransform(const uint8_t axisMap[3], const int8_t axisSign[3], float matrix[3][3]) {
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            matrix[row][col] = 0.0f;
        }
        matrix[row][axisMap[row]] = static_cast<float>(axisSign[row]);
    }
}

void Multiply3x3(const float a[3][3], const float b[3][3], float out[3][3]) {
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            out[row][col] = a[row][0] * b[0][col] +
                            a[row][1] * b[1][col] +
                            a[row][2] * b[2][col];
        }
    }
}

float Determinant3x3(const float matrix[3][3]) {
    return matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) -
           matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
           matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
}

float Magnitude3(const float vector[3]) {
    return sqrtf(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
}

bool Normalize3(float vector[3]) {
    const float magnitude = Magnitude3(vector);
    if (!(magnitude > 1.0e-6f)) {
        return false;
    }
    vector[0] /= magnitude;
    vector[1] /= magnitude;
    vector[2] /= magnitude;
    return true;
}

float AngleDegBetweenUnits(const float a[3], const float b[3]) {
    const float dot = ClampUnit(a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
    return acosf(dot) * 57.295779513082320876f;
}

const char *DominantAxisLabel(const float unit[3]) {
    const float ax = fabsf(unit[0]);
    const float ay = fabsf(unit[1]);
    const float az = fabsf(unit[2]);
    if (ax >= ay && ax >= az) {
        return unit[0] >= 0.0f ? "+X" : "-X";
    }
    if (ay >= ax && ay >= az) {
        return unit[1] >= 0.0f ? "+Y" : "-Y";
    }
    return unit[2] >= 0.0f ? "+Z" : "-Z";
}

float RescaleCalibrationCounts(float calibrationCounts, float activeLsbPerUnit, float calibrationLsbPerUnit) {
    if (!(calibrationLsbPerUnit > 0.0f)) {
        return calibrationCounts;
    }
    return calibrationCounts * (activeLsbPerUnit / calibrationLsbPerUnit);
}

float IcmAccelLsbPerG(uint8_t rangeG) {
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

float LsmAccelLsbPerG(uint8_t rangeG) {
    switch (rangeG) {
        case 2:
            return 16384.0f;
        case 4:
            return 8192.0f;
        case 8:
            return 4096.0f;
        case 16:
            return 1366.12024f;
        default:
            return 16384.0f;
    }
}

void PrintVector3(const float vector[3], uint8_t digits) {
    Serial.print("(");
    Serial.print(vector[0], digits);
    Serial.print(", ");
    Serial.print(vector[1], digits);
    Serial.print(", ");
    Serial.print(vector[2], digits);
    Serial.print(")");
}

void PrintMatrixStatus(const char *name, const float matrix[3][3]) {
    const float determinant = Determinant3x3(matrix);
    Serial.print(name);
    Serial.print(": det=");
    Serial.print(determinant, 3);
    Serial.print(" -> ");
    Serial.println(determinant >= 0.0f ? "rotation" : "reflection");
}

void PrintMappingSummary() {
    float lsmAxisTransform[3][3];
    float lsmCombined[3][3];
    BuildAxisTransform(settings::sensors::lsm9ds1::kAxisMap,
                       settings::sensors::lsm9ds1::kAxisSign,
                       lsmAxisTransform);
    Multiply3x3(settings::sensors::lsm9ds1::kMountRotation, lsmAxisTransform, lsmCombined);

    Serial.println();
    Serial.println("Configured accel frame determinants:");
    PrintMatrixStatus("  BNO", settings::sensors::bno055::kMountRotation);
    PrintMatrixStatus("  ICM", settings::sensors::icm20948::kMountRotation);
    PrintMatrixStatus("  LSM", lsmCombined);
    PrintMatrixStatus("  Pulse", settings::sensors::ellipse20::kMountRotation);
    Serial.println("A determinant of -1 means the accel frame is mirrored.");
    Serial.println("That can still make body accel match, but it is not a proper quaternion frame.");
}

void PrintHelp() {
    Serial.println();
    Serial.println("Orientation frame check");
    Serial.println("Commands:");
    Serial.println("  h = help");
    Serial.println("  s = toggle live stream");
    Serial.println("  c = capture a 0.6 s averaged snapshot");
    Serial.println("  x/X/y/Y/z/Z = capture and compare against that body-up pose");
    Serial.println();
    Serial.println("This validates accel-frame agreement across rails.");
    Serial.println("It does not, by itself, prove the frame is right-handed for quaternion use.");
    PrintMappingSummary();
    Serial.println();
}

bool BeginBno() {
    if (settings::sensors::bno::kModel != settings::sensors::bno::Model::Bno055) {
        Serial.println("BNO check skipped: active BNO model is not BNO055.");
        return false;
    }
    if (settings::sensors::bno::kTransport != settings::sensors::bno::Transport::I2c) {
        Serial.println("BNO check skipped: BNO055 test expects I2C transport.");
        return false;
    }
    Wire.begin();
    if (!g_bno.begin()) {
        Serial.println("BNO init failed.");
        return false;
    }
    if (settings::sensors::bno055::kResetPin >= 0) {
        pinMode(settings::sensors::bno055::kResetPin, OUTPUT);
        digitalWrite(settings::sensors::bno055::kResetPin, HIGH);
    }
    g_bno.setExtCrystalUse(true);
    delay(10);
    Serial.println("BNO online.");
    return true;
}

bool BeginIcm() {
    SPI1.begin();
    g_icm.begin(settings::sensors::icm20948::kChipSelectPin, SPI1);
    if (g_icm.status != ICM_20948_Stat_Ok) {
        Serial.print("ICM init failed: ");
        Serial.println(g_icm.statusString());
        return false;
    }
    if (g_icm.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
                            ICM_20948_Sample_Mode_Continuous) != ICM_20948_Stat_Ok) {
        Serial.println("ICM sample-mode config failed.");
        return false;
    }
    ICM_20948_fss_t fullScale = {};
    fullScale.a = settings::sensors::icm20948::kAccelRangeG == 2
                      ? gpm2
                      : settings::sensors::icm20948::kAccelRangeG == 4 ? gpm4
                                                                        : settings::sensors::icm20948::kAccelRangeG == 8 ? gpm8
                                                                                                                         : gpm16;
    fullScale.g = settings::sensors::icm20948::kGyroRangeDps == 250
                      ? dps250
                      : settings::sensors::icm20948::kGyroRangeDps == 500   ? dps500
                        : settings::sensors::icm20948::kGyroRangeDps == 1000 ? dps1000
                                                                             : dps2000;
    if (g_icm.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fullScale) != ICM_20948_Stat_Ok) {
        Serial.println("ICM full-scale config failed.");
        return false;
    }
    ICM_20948_dlpcfg_t filterConfig = {};
    filterConfig.a = settings::sensors::icm20948::kAccelDlpFilterSetting;
    filterConfig.g = settings::sensors::icm20948::kGyroDlpFilterSetting;
    if (g_icm.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), filterConfig) != ICM_20948_Stat_Ok) {
        Serial.println("ICM DLPF config failed.");
        return false;
    }
    if (g_icm.enableDLPF((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
                         settings::sensors::icm20948::kEnableDlpFilter) != ICM_20948_Stat_Ok) {
        Serial.println("ICM DLPF enable failed.");
        return false;
    }
    ICM_20948_smplrt_t sampleRate = {};
    sampleRate.a = settings::sensors::icm20948::kAccelSampleRateDivider;
    sampleRate.g = settings::sensors::icm20948::kGyroSampleRateDivider;
    if (g_icm.setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), sampleRate) != ICM_20948_Stat_Ok) {
        Serial.println("ICM sample-rate config failed.");
        return false;
    }
    Serial.println("ICM online.");
    return true;
}

bool BeginLsm() {
    g_lsm.settings.device.commInterface = IMU_MODE_SPI;
    g_lsm.settings.device.agAddress = settings::sensors::lsm9ds1::kAccelGyroChipSelectPin;
    g_lsm.settings.device.mAddress = settings::sensors::lsm9ds1::kMagChipSelectPin;
    g_lsm.settings.accel.scale = settings::sensors::lsm9ds1::kAccelRangeG;
    g_lsm.settings.gyro.scale = settings::sensors::lsm9ds1::kGyroRangeDps;
    g_lsm.settings.mag.scale = settings::sensors::lsm9ds1::kMagRangeGauss;
    g_lsm.settings.gyro.sampleRate = settings::sensors::lsm9ds1::kGyroSampleRateSetting;
    g_lsm.settings.accel.sampleRate = settings::sensors::lsm9ds1::kAccelSampleRateSetting;
    g_lsm.settings.mag.sampleRate = settings::sensors::lsm9ds1::kMagSampleRateSetting;
    g_lsm.settings.gyro.bandwidth = settings::sensors::lsm9ds1::kGyroBandwidthSetting;
    g_lsm.settings.accel.bandwidth = settings::sensors::lsm9ds1::kAccelBandwidthSetting;
    g_lsm.settings.accel.highResEnable = settings::sensors::lsm9ds1::kAccelHighResolutionEnable;
    g_lsm.settings.accel.highResBandwidth = settings::sensors::lsm9ds1::kAccelHighResolutionBandwidthSetting;
    g_lsm.settings.mag.tempCompensationEnable = settings::sensors::lsm9ds1::kMagTemperatureCompensationEnable;
    g_lsm.settings.mag.XYPerformance = settings::sensors::lsm9ds1::kMagXyPerformanceSetting;
    g_lsm.settings.mag.ZPerformance = settings::sensors::lsm9ds1::kMagZPerformanceSetting;
    g_lsm.settings.mag.lowPowerEnable = settings::sensors::lsm9ds1::kMagLowPowerEnable;
    g_lsm.settings.mag.operatingMode = settings::sensors::lsm9ds1::kMagOperatingModeSetting;
    SPI1.begin();
    if (g_lsm.beginSPI(settings::sensors::lsm9ds1::kAccelGyroChipSelectPin,
                       settings::sensors::lsm9ds1::kMagChipSelectPin,
                       SPI1) == 0) {
        Serial.println("LSM init failed.");
        return false;
    }
    Serial.println("LSM online.");
    return true;
}

bool BeginPulse() {
    Ellipse20SensorSetCrossCheckTrust(1.0f);
    Ellipse20SensorSetFlightStatus(FlightStatus::Ground);
    if (!Ellipse20SensorBegin()) {
        Serial.println("Pulse init failed.");
        return false;
    }
    Serial.println("Pulse online.");
    return true;
}

bool ReadBnoAccel(float rawG[3], float bodyG[3]) {
    imu::Vector<3> accel = g_bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    rawG[0] = accel.x() / kBnoMetersPerSecondSquaredPerG;
    rawG[1] = accel.y() / kBnoMetersPerSecondSquaredPerG;
    rawG[2] = accel.z() / kBnoMetersPerSecondSquaredPerG;
    if (!isfinite(rawG[0]) || !isfinite(rawG[1]) || !isfinite(rawG[2])) {
        return false;
    }
    Apply3x3(settings::sensors::bno055::kMountRotation, rawG, bodyG);
    return true;
}

bool ReadIcmAccel(float rawG[3], float bodyG[3]) {
    if (!g_icm.dataReady()) {
        return false;
    }
    g_icm.getAGMT();
    const float activeLsbPerG = IcmAccelLsbPerG(settings::sensors::icm20948::kAccelRangeG);
    const float calibrationLsbPerG = IcmAccelLsbPerG(settings::sensors::icm20948::kCalibrationAccelRangeG);
    float correctedCounts[3] = {
        static_cast<float>(g_icm.agmt.acc.axes.x) -
            RescaleCalibrationCounts(settings::sensors::icm20948::kAccelBias[0], activeLsbPerG, calibrationLsbPerG),
        static_cast<float>(g_icm.agmt.acc.axes.y) -
            RescaleCalibrationCounts(settings::sensors::icm20948::kAccelBias[1], activeLsbPerG, calibrationLsbPerG),
        static_cast<float>(g_icm.agmt.acc.axes.z) -
            RescaleCalibrationCounts(settings::sensors::icm20948::kAccelBias[2], activeLsbPerG, calibrationLsbPerG),
    };
    float calibratedCounts[3] = {0.0f, 0.0f, 0.0f};
    Apply3x3(settings::sensors::icm20948::kAccelAinv, correctedCounts, calibratedCounts);
    rawG[0] = calibratedCounts[0] / activeLsbPerG;
    rawG[1] = calibratedCounts[1] / activeLsbPerG;
    rawG[2] = calibratedCounts[2] / activeLsbPerG;
    Apply3x3(settings::sensors::icm20948::kMountRotation, rawG, bodyG);
    return true;
}

bool ReadLsmAccel(float rawG[3], float bodyG[3]) {
    if (!g_lsm.accelAvailable()) {
        return false;
    }
    g_lsm.readAccel();
    const float activeLsbPerG = LsmAccelLsbPerG(settings::sensors::lsm9ds1::kAccelRangeG);
    const float calibrationLsbPerG = LsmAccelLsbPerG(settings::sensors::lsm9ds1::kCalibrationAccelRangeG);
    float correctedCounts[3] = {
        static_cast<float>(g_lsm.ax) -
            RescaleCalibrationCounts(settings::sensors::lsm9ds1::kAccelBias[0], activeLsbPerG, calibrationLsbPerG),
        static_cast<float>(g_lsm.ay) -
            RescaleCalibrationCounts(settings::sensors::lsm9ds1::kAccelBias[1], activeLsbPerG, calibrationLsbPerG),
        static_cast<float>(g_lsm.az) -
            RescaleCalibrationCounts(settings::sensors::lsm9ds1::kAccelBias[2], activeLsbPerG, calibrationLsbPerG),
    };
    Apply3x3(settings::sensors::lsm9ds1::kAccelAinv, correctedCounts, correctedCounts);
    float axisCorrectedCounts[3] = {
        correctedCounts[settings::sensors::lsm9ds1::kAxisMap[0]] * settings::sensors::lsm9ds1::kAxisSign[0],
        correctedCounts[settings::sensors::lsm9ds1::kAxisMap[1]] * settings::sensors::lsm9ds1::kAxisSign[1],
        correctedCounts[settings::sensors::lsm9ds1::kAxisMap[2]] * settings::sensors::lsm9ds1::kAxisSign[2],
    };
    rawG[0] = axisCorrectedCounts[0] / activeLsbPerG;
    rawG[1] = axisCorrectedCounts[1] / activeLsbPerG;
    rawG[2] = axisCorrectedCounts[2] / activeLsbPerG;
    Apply3x3(settings::sensors::lsm9ds1::kMountRotation, rawG, bodyG);
    return true;
}

bool ReadPulseAccel(float rawG[3], float bodyG[3]) {
    SensorData sample;
    const bool acquired = Ellipse20SensorAcquire(sample);
    const Ellipse20Diagnostics diagnostics = Ellipse20SensorGetDiagnostics();
    if (!acquired || !diagnostics.hasImu) {
        return false;
    }
    for (int i = 0; i < 3; ++i) {
        bodyG[i] = sample.accelPulse[i] / kBnoMetersPerSecondSquaredPerG;
        // The Ellipse driver already applies calibration and mount rotation.
        rawG[i] = bodyG[i];
    }
    return true;
}

void UpdateLiveRail(LiveRailSample &live, const float rawG[3], const float bodyG[3]) {
    live.valid = true;
    ++live.samples;
    for (int i = 0; i < 3; ++i) {
        live.rawG[i] = rawG[i];
        live.bodyG[i] = bodyG[i];
    }
}

void PollSensors(CaptureBundle *capture) {
    float rawG[3] = {0.0f, 0.0f, 0.0f};
    float bodyG[3] = {0.0f, 0.0f, 0.0f};

    if (g_bnoOnline && ReadBnoAccel(rawG, bodyG)) {
        UpdateLiveRail(g_liveBno, rawG, bodyG);
        if (capture != nullptr) {
            capture->bno.online = true;
            capture->bno.stats.Add(rawG, bodyG);
        }
    }

    if (g_icmOnline && ReadIcmAccel(rawG, bodyG)) {
        UpdateLiveRail(g_liveIcm, rawG, bodyG);
        if (capture != nullptr) {
            capture->icm.online = true;
            capture->icm.stats.Add(rawG, bodyG);
        }
    }

    if (g_lsmOnline && ReadLsmAccel(rawG, bodyG)) {
        UpdateLiveRail(g_liveLsm, rawG, bodyG);
        if (capture != nullptr) {
            capture->lsm.online = true;
            capture->lsm.stats.Add(rawG, bodyG);
        }
    }

    if (g_pulseOnline && ReadPulseAccel(rawG, bodyG)) {
        UpdateLiveRail(g_livePulse, rawG, bodyG);
        if (capture != nullptr) {
            capture->pulse.online = true;
            capture->pulse.stats.Add(rawG, bodyG);
        }
    }
}

void PrintRailLine(const char *name, const LiveRailSample &live, bool online) {
    Serial.print(name);
    Serial.print(": ");
    if (!online) {
        Serial.println("offline");
        return;
    }
    if (!live.valid) {
        Serial.println("waiting");
        return;
    }
    float unit[3] = {live.bodyG[0], live.bodyG[1], live.bodyG[2]};
    Normalize3(unit);
    Serial.print("body_g=");
    PrintVector3(live.bodyG, 3);
    Serial.print(" unit=");
    PrintVector3(unit, 3);
    Serial.print(" dom=");
    Serial.print(DominantAxisLabel(unit));
    Serial.print(" |g|=");
    Serial.print(Magnitude3(live.bodyG), 3);
    Serial.print(" samples=");
    Serial.println(live.samples);
}

void PrintPairwiseCaptureAngles(const RailCapture &a, const RailCapture &b) {
    if (!(a.stats.Ready() && b.stats.Ready())) {
        return;
    }
    float aMean[3] = {0.0f, 0.0f, 0.0f};
    float bMean[3] = {0.0f, 0.0f, 0.0f};
    a.stats.MeanBody(aMean);
    b.stats.MeanBody(bMean);
    if (!Normalize3(aMean) || !Normalize3(bMean)) {
        return;
    }
    Serial.print("  ");
    Serial.print(a.name);
    Serial.print(" vs ");
    Serial.print(b.name);
    Serial.print(": ");
    Serial.print(AngleDegBetweenUnits(aMean, bMean), 2);
    Serial.println(" deg");
}

void PrintCaptureReport(const RailCapture &rail, const PoseExpectation *pose) {
    Serial.print(rail.name);
    Serial.print(": ");
    if (!rail.online) {
        Serial.println("offline");
        return;
    }
    if (!rail.stats.Ready()) {
        Serial.println("no samples");
        return;
    }

    float meanRaw[3] = {0.0f, 0.0f, 0.0f};
    float meanBody[3] = {0.0f, 0.0f, 0.0f};
    rail.stats.MeanRaw(meanRaw);
    rail.stats.MeanBody(meanBody);
    float unit[3] = {meanBody[0], meanBody[1], meanBody[2]};
    Normalize3(unit);

    Serial.print("n=");
    Serial.print(rail.stats.count);
    Serial.print(" raw_g=");
    PrintVector3(meanRaw, 3);
    Serial.print(" body_g=");
    PrintVector3(meanBody, 3);
    Serial.print(" unit=");
    PrintVector3(unit, 3);
    Serial.print(" dom=");
    Serial.print(DominantAxisLabel(unit));
    Serial.print(" |g|=");
    Serial.print(Magnitude3(meanBody), 3);
    if (pose != nullptr) {
        Serial.print(" err=");
        Serial.print(AngleDegBetweenUnits(unit, pose->expectedUnit), 2);
        Serial.print(" deg");
    }
    Serial.println();
}

const PoseExpectation *FindPoseExpectation(char command) {
    for (const PoseExpectation &pose : kPoseExpectations) {
        if (pose.command == command) {
            return &pose;
        }
    }
    return nullptr;
}

void RunCapture(const PoseExpectation *pose) {
    CaptureBundle capture;
    capture.bno.online = g_bnoOnline;
    capture.icm.online = g_icmOnline;
    capture.lsm.online = g_lsmOnline;
    capture.pulse.online = g_pulseOnline;

    Serial.println();
    if (pose != nullptr) {
        Serial.print("Capture: hold ");
        Serial.print(pose->label);
        Serial.println(" still.");
    } else {
        Serial.println("Capture: hold current pose still.");
    }
    const uint32_t startMs = millis();
    while ((millis() - startMs) < kCaptureDurationMs) {
        PollSensors(&capture);
        delay(5);
    }

    PrintCaptureReport(capture.bno, pose);
    PrintCaptureReport(capture.icm, pose);
    PrintCaptureReport(capture.lsm, pose);
    PrintCaptureReport(capture.pulse, pose);
    Serial.println("Pairwise body-vector angles:");
    PrintPairwiseCaptureAngles(capture.bno, capture.icm);
    PrintPairwiseCaptureAngles(capture.bno, capture.lsm);
    PrintPairwiseCaptureAngles(capture.bno, capture.pulse);
    PrintPairwiseCaptureAngles(capture.icm, capture.lsm);
    PrintPairwiseCaptureAngles(capture.icm, capture.pulse);
    PrintPairwiseCaptureAngles(capture.lsm, capture.pulse);
    Serial.println();
}

void HandleCommand(char command) {
    const PoseExpectation *pose = FindPoseExpectation(command);
    if (pose != nullptr) {
        RunCapture(pose);
        return;
    }

    switch (command) {
        case 'h':
        case '?':
            PrintHelp();
            break;
        case 's':
            g_streamEnabled = !g_streamEnabled;
            Serial.print("Live stream ");
            Serial.println(g_streamEnabled ? "enabled" : "disabled");
            break;
        case 'c':
            RunCapture(nullptr);
            break;
        default:
            break;
    }
}

}  // namespace

void setup() {
    Serial.begin(kSerialBaud);
    delay(1500);
    Serial.println();
    Serial.println("Starting orientation frame check...");

    g_bnoOnline = BeginBno();
    g_icmOnline = BeginIcm();
    g_lsmOnline = BeginLsm();
    g_pulseOnline = BeginPulse();

    PrintHelp();
}

void loop() {
    while (Serial.available() > 0) {
        HandleCommand(static_cast<char>(Serial.read()));
    }

    PollSensors(nullptr);

    const uint32_t nowMs = millis();
    if (!g_streamEnabled || (nowMs - g_lastStreamMs) < kStreamIntervalMs) {
        delay(2);
        return;
    }
    g_lastStreamMs = nowMs;

    PrintRailLine("BNO", g_liveBno, g_bnoOnline);
    PrintRailLine("ICM", g_liveIcm, g_icmOnline);
    PrintRailLine("LSM", g_liveLsm, g_lsmOnline);
    PrintRailLine("PULSE", g_livePulse, g_pulseOnline);
    Serial.println();
}
