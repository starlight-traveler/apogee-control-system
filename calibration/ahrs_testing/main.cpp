#include <Arduino.h>
#include <SPI.h>

#include <Fusion.h>
#include <ICM_20948.h>
#include <SparkFunLSM9DS1.h>

#include "settings.h"

#ifndef ACS_BUILD_AHRS_TESTING
#error "Use the ahrs_testing PlatformIO environment to build this target."
#endif

namespace {

constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kPrintIntervalMs = 50;
constexpr float kRadiansToDegrees = 57.295779513082320876f;

struct FusionRailOutput {
    bool valid = false;
    uint32_t timestampUs = 0;
    float accelG[3] = {0.0f, 0.0f, 0.0f};
    float rawMag[3] = {0.0f, 0.0f, 0.0f};
    float magBody[3] = {0.0f, 0.0f, 0.0f};
    float rollDeg = 0.0f;
    float pitchDeg = 0.0f;
    float yawDeg = 0.0f;
};

LSM9DS1 g_lsm;
ICM_20948_SPI g_icm;

FusionAhrs g_lsmAhrs;
FusionAhrs g_icmAhrs;
FusionBias g_lsmBias;
FusionBias g_icmBias;

FusionRailOutput g_lsmOutput;
FusionRailOutput g_icmOutput;

bool g_lsmHaveAccel = false;
bool g_lsmHaveGyro = false;
bool g_lsmHaveMag = false;
uint32_t g_lsmLastSampleUs = 0;
uint32_t g_icmLastSampleUs = 0;
float g_lsmTemperatureC = settings::sensors::lsm9ds1::kGyroReferenceTemperatureC;
uint32_t g_lastPrintMs = 0;

float LsmAccelLsbPerGForRange(uint8_t rangeG) {
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

float LsmGyroLsbPerDpsForRange(uint16_t rangeDps) {
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

float LsmMagGaussPerLsbForRange(uint8_t rangeGauss) {
    switch (rangeGauss) {
        case 4:
            return 0.00014f;
        case 8:
            return 0.00029f;
        case 12:
            return 0.00043f;
        case 16:
            return 0.00058f;
        default:
            return 0.00014f;
    }
}

float IcmAccelLsbPerGForRange(uint8_t rangeG) {
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

float IcmGyroLsbPerDpsForRange(uint16_t rangeDps) {
    switch (rangeDps) {
        case 250:
            return 131.0f;
        case 500:
            return 65.5f;
        case 1000:
            return 32.8f;
        case 2000:
            return 16.4f;
        default:
            return 131.0f;
    }
}

float RescaleCalibrationCounts(float calibrationCounts, float activeLsbPerUnit, float calibrationLsbPerUnit) {
    if (!(calibrationLsbPerUnit > 0.0f)) {
        return calibrationCounts;
    }
    return calibrationCounts * (activeLsbPerUnit / calibrationLsbPerUnit);
}

FusionVector MakeVector(float x, float y, float z) {
    FusionVector result = FUSION_VECTOR_ZERO;
    result.axis.x = x;
    result.axis.y = y;
    result.axis.z = z;
    return result;
}

void Apply3x3(const float matrix[3][3], const float in[3], float out[3]) {
    out[0] = matrix[0][0] * in[0] + matrix[0][1] * in[1] + matrix[0][2] * in[2];
    out[1] = matrix[1][0] * in[0] + matrix[1][1] * in[1] + matrix[1][2] * in[2];
    out[2] = matrix[2][0] * in[0] + matrix[2][1] * in[1] + matrix[2][2] * in[2];
}

void ApplyLsmAxisTransform(float vector[3]) {
    float remapped[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 3; ++i) {
        const uint8_t source = settings::sensors::lsm9ds1::kAxisMap[i];
        remapped[i] = static_cast<float>(settings::sensors::lsm9ds1::kAxisSign[i]) * vector[source];
    }
    vector[0] = remapped[0];
    vector[1] = remapped[1];
    vector[2] = remapped[2];
}

void ApplyIcmMagAxisTransform(float vector[3]) {
    float remapped[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 3; ++i) {
        const uint8_t source = settings::sensors::icm20948::kMagAxisMap[i];
        remapped[i] = static_cast<float>(settings::sensors::icm20948::kMagAxisSign[i]) * vector[source];
    }
    vector[0] = remapped[0];
    vector[1] = remapped[1];
    vector[2] = remapped[2];
}

void ApplyLsmMountRotation(float vector[3]) {
    float rotated[3] = {0.0f, 0.0f, 0.0f};
    Apply3x3(settings::sensors::lsm9ds1::kMountRotation, vector, rotated);
    vector[0] = rotated[0];
    vector[1] = rotated[1];
    vector[2] = rotated[2];
}

void ApplyIcmMountRotation(float vector[3]) {
    float rotated[3] = {0.0f, 0.0f, 0.0f};
    Apply3x3(settings::sensors::icm20948::kMountRotation, vector, rotated);
    vector[0] = rotated[0];
    vector[1] = rotated[1];
    vector[2] = rotated[2];
}

float Wrap360(float degrees) {
    while (degrees < 0.0f) {
        degrees += 360.0f;
    }
    while (degrees >= 360.0f) {
        degrees -= 360.0f;
    }
    return degrees;
}

float WrapSigned180(float degrees) {
    while (degrees <= -180.0f) {
        degrees += 360.0f;
    }
    while (degrees > 180.0f) {
        degrees -= 360.0f;
    }
    return degrees;
}

void ConfigureFusion(FusionAhrs &ahrs, FusionBias &bias, float sampleRateHz, float gyroRangeDps) {
    FusionAhrsInitialise(&ahrs);
    FusionBiasInitialise(&bias);

    FusionAhrsSettings ahrsSettings;
    ahrsSettings.convention = FusionConventionNwu;
    ahrsSettings.gain = 0.5f;
    ahrsSettings.gyroscopeRange = gyroRangeDps;
    ahrsSettings.accelerationRejection = 10.0f;
    ahrsSettings.magneticRejection = 10.0f;
    ahrsSettings.recoveryTriggerPeriod = static_cast<unsigned int>(sampleRateHz * 5.0f);
    FusionAhrsSetSettings(&ahrs, &ahrsSettings);

    FusionBiasSettings biasSettings;
    biasSettings.sampleRate = sampleRateHz;
    biasSettings.stationaryThreshold = 3.0f;
    biasSettings.stationaryPeriod = 3.0f;
    FusionBiasSetSettings(&bias, &biasSettings);
}

bool ConfigureLsmSensor() {
    g_lsm.settings.device.commInterface = IMU_MODE_SPI;
    g_lsm.settings.device.agAddress = settings::sensors::lsm9ds1::kAccelGyroChipSelectPin;
    g_lsm.settings.device.mAddress = settings::sensors::lsm9ds1::kMagChipSelectPin;
    g_lsm.settings.gyro.scale = settings::sensors::lsm9ds1::kGyroRangeDps;
    g_lsm.settings.accel.scale = settings::sensors::lsm9ds1::kAccelRangeG;
    g_lsm.settings.mag.scale = settings::sensors::lsm9ds1::kMagRangeGauss;
    g_lsm.settings.gyro.sampleRate = settings::sensors::lsm9ds1::kGyroSampleRateSetting;
    g_lsm.settings.accel.sampleRate = settings::sensors::lsm9ds1::kAccelSampleRateSetting;
    g_lsm.settings.mag.sampleRate = settings::sensors::lsm9ds1::kMagSampleRateSetting;
    g_lsm.settings.gyro.bandwidth = settings::sensors::lsm9ds1::kGyroBandwidthSetting;
    g_lsm.settings.accel.bandwidth = settings::sensors::lsm9ds1::kAccelBandwidthSetting;
    g_lsm.settings.accel.highResEnable = settings::sensors::lsm9ds1::kAccelHighResolutionEnable;
    g_lsm.settings.accel.highResBandwidth =
        settings::sensors::lsm9ds1::kAccelHighResolutionBandwidthSetting;
    g_lsm.settings.mag.tempCompensationEnable =
        settings::sensors::lsm9ds1::kMagTemperatureCompensationEnable;
    g_lsm.settings.mag.XYPerformance = settings::sensors::lsm9ds1::kMagXyPerformanceSetting;
    g_lsm.settings.mag.ZPerformance = settings::sensors::lsm9ds1::kMagZPerformanceSetting;
    g_lsm.settings.mag.lowPowerEnable = settings::sensors::lsm9ds1::kMagLowPowerEnable;
    g_lsm.settings.mag.operatingMode = settings::sensors::lsm9ds1::kMagOperatingModeSetting;
    return g_lsm.beginSPI(settings::sensors::lsm9ds1::kAccelGyroChipSelectPin,
                          settings::sensors::lsm9ds1::kMagChipSelectPin,
                          SPI1) != 0;
}

ICM_20948_ACCEL_CONFIG_FS_SEL_e IcmAccelFullScaleEnum(uint8_t rangeG) {
    switch (rangeG) {
        case 2:
            return gpm2;
        case 4:
            return gpm4;
        case 8:
            return gpm8;
        case 16:
            return gpm16;
        default:
            return gpm2;
    }
}

ICM_20948_GYRO_CONFIG_1_FS_SEL_e IcmGyroFullScaleEnum(uint16_t rangeDps) {
    switch (rangeDps) {
        case 250:
            return dps250;
        case 500:
            return dps500;
        case 1000:
            return dps1000;
        case 2000:
            return dps2000;
        default:
            return dps250;
    }
}

bool ConfigureIcmSensor() {
    g_icm.begin(settings::sensors::icm20948::kChipSelectPin, SPI1);
    if (g_icm.status != ICM_20948_Stat_Ok) {
        return false;
    }

    if (g_icm.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
                            ICM_20948_Sample_Mode_Continuous) != ICM_20948_Stat_Ok) {
        return false;
    }

    ICM_20948_fss_t fullScale = {};
    fullScale.a = IcmAccelFullScaleEnum(settings::sensors::icm20948::kAccelRangeG);
    fullScale.g = IcmGyroFullScaleEnum(settings::sensors::icm20948::kGyroRangeDps);
    if (g_icm.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fullScale) != ICM_20948_Stat_Ok) {
        return false;
    }

    ICM_20948_dlpcfg_t filterConfig = {};
    filterConfig.a = settings::sensors::icm20948::kAccelDlpFilterSetting;
    filterConfig.g = settings::sensors::icm20948::kGyroDlpFilterSetting;
    if (g_icm.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), filterConfig) != ICM_20948_Stat_Ok) {
        return false;
    }
    if (g_icm.enableDLPF((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
                         settings::sensors::icm20948::kEnableDlpFilter) != ICM_20948_Stat_Ok) {
        return false;
    }

    ICM_20948_smplrt_t sampleRate = {};
    sampleRate.a = settings::sensors::icm20948::kAccelSampleRateDivider;
    sampleRate.g = settings::sensors::icm20948::kGyroSampleRateDivider;
    if (g_icm.setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), sampleRate) != ICM_20948_Stat_Ok) {
        return false;
    }

    return true;
}

void UpdateFusionOutput(FusionAhrs &ahrs,
                        FusionBias &bias,
                        const FusionVector gyroDps,
                        const FusionVector accelG,
                        const float rawMag[3],
                        const FusionVector magBody,
                        float dtSeconds,
                        float declinationDeg,
                        bool invertRollDisplay,
                        FusionRailOutput &out) {
    const FusionVector correctedGyroDps = FusionBiasUpdate(&bias, gyroDps);
    FusionAhrsUpdate(&ahrs, correctedGyroDps, accelG, magBody, dtSeconds);

    const FusionQuaternion quaternion = FusionAhrsGetQuaternion(&ahrs);
    const FusionEuler euler = FusionQuaternionToEuler(quaternion);

    out.valid = true;
    out.accelG[0] = accelG.axis.x;
    out.accelG[1] = accelG.axis.y;
    out.accelG[2] = accelG.axis.z;
    out.rawMag[0] = rawMag[0];
    out.rawMag[1] = rawMag[1];
    out.rawMag[2] = rawMag[2];
    out.magBody[0] = magBody.axis.x;
    out.magBody[1] = magBody.axis.y;
    out.magBody[2] = magBody.axis.z;
    out.rollDeg = invertRollDisplay ? -euler.angle.roll : euler.angle.roll;
    out.pitchDeg = euler.angle.pitch;
    out.yawDeg = Wrap360(-(euler.angle.yaw + declinationDeg));
}

bool UpdateLsmRail() {
    const uint32_t nowUs = micros();
    if (g_lsmLastSampleUs != 0 &&
        (nowUs - g_lsmLastSampleUs) < settings::sensors::lsm9ds1::kSampleIntervalUs) {
        return false;
    }

    bool updated = false;
    if (g_lsm.gyroAvailable()) {
        g_lsm.readGyro();
        g_lsmHaveGyro = true;
        updated = true;
    }
    if (g_lsm.accelAvailable()) {
        g_lsm.readAccel();
        g_lsmHaveAccel = true;
        updated = true;
    }
    // Force a mag register read every cycle for diagnostics. The LSM mag path
    // was appearing stale in the CSV, and a direct read tells us whether the
    // device registers are actually changing.
    g_lsm.readMag();
    g_lsmHaveMag = true;
    updated = true;
    if (updated && g_lsm.tempAvailable()) {
        g_lsm.readTemp();
        g_lsmTemperatureC = static_cast<float>(g_lsm.temperature);
    }
    if (!(updated && g_lsmHaveAccel && g_lsmHaveGyro && g_lsmHaveMag)) {
        return false;
    }

    float dtSeconds = static_cast<float>(settings::sensors::lsm9ds1::kSampleIntervalUs) * 1.0e-6f;
    if (g_lsmLastSampleUs != 0) {
        dtSeconds = static_cast<float>(nowUs - g_lsmLastSampleUs) * 1.0e-6f;
        if (!(dtSeconds > 0.0f) || dtSeconds > 0.1f) {
            dtSeconds = static_cast<float>(settings::sensors::lsm9ds1::kSampleIntervalUs) * 1.0e-6f;
        }
    }
    g_lsmLastSampleUs = nowUs;

    const float activeAccelLsbPerG = LsmAccelLsbPerGForRange(settings::sensors::lsm9ds1::kAccelRangeG);
    const float activeGyroLsbPerDps = LsmGyroLsbPerDpsForRange(settings::sensors::lsm9ds1::kGyroRangeDps);
    const float activeMagGaussPerLsb = LsmMagGaussPerLsbForRange(settings::sensors::lsm9ds1::kMagRangeGauss);
    const float calibrationAccelLsbPerG =
        LsmAccelLsbPerGForRange(settings::sensors::lsm9ds1::kCalibrationAccelRangeG);
    const float calibrationGyroLsbPerDps =
        LsmGyroLsbPerDpsForRange(settings::sensors::lsm9ds1::kCalibrationGyroRangeDps);
    const float calibrationMagGaussPerLsb =
        LsmMagGaussPerLsbForRange(settings::sensors::lsm9ds1::kCalibrationMagRangeGauss);

    float gyroCounts[3] = {
        static_cast<float>(g_lsm.gx) -
            RescaleCalibrationCounts(settings::sensors::lsm9ds1::kGyroOffset[0],
                                     activeGyroLsbPerDps,
                                     calibrationGyroLsbPerDps),
        static_cast<float>(g_lsm.gy) -
            RescaleCalibrationCounts(settings::sensors::lsm9ds1::kGyroOffset[1],
                                     activeGyroLsbPerDps,
                                     calibrationGyroLsbPerDps),
        static_cast<float>(g_lsm.gz) -
            RescaleCalibrationCounts(settings::sensors::lsm9ds1::kGyroOffset[2],
                                     activeGyroLsbPerDps,
                                     calibrationGyroLsbPerDps),
    };
    ApplyLsmAxisTransform(gyroCounts);
    float gyroDps[3] = {
        gyroCounts[0] / activeGyroLsbPerDps,
        gyroCounts[1] / activeGyroLsbPerDps,
        gyroCounts[2] / activeGyroLsbPerDps,
    };
    const float lsmTemperatureDeltaC =
        g_lsmTemperatureC - settings::sensors::lsm9ds1::kGyroReferenceTemperatureC;
    gyroDps[0] -= settings::sensors::lsm9ds1::kGyroTempBiasSlopeRadPerSecPerC[0] * lsmTemperatureDeltaC *
                  kRadiansToDegrees;
    gyroDps[1] -= settings::sensors::lsm9ds1::kGyroTempBiasSlopeRadPerSecPerC[1] * lsmTemperatureDeltaC *
                  kRadiansToDegrees;
    gyroDps[2] -= settings::sensors::lsm9ds1::kGyroTempBiasSlopeRadPerSecPerC[2] * lsmTemperatureDeltaC *
                  kRadiansToDegrees;
    Apply3x3(settings::sensors::lsm9ds1::kGyroAinv, gyroDps, gyroDps);
    ApplyLsmMountRotation(gyroDps);

    float accelCounts[3] = {
        static_cast<float>(g_lsm.ax) -
            RescaleCalibrationCounts(settings::sensors::lsm9ds1::kAccelBias[0],
                                     activeAccelLsbPerG,
                                     calibrationAccelLsbPerG),
        static_cast<float>(g_lsm.ay) -
            RescaleCalibrationCounts(settings::sensors::lsm9ds1::kAccelBias[1],
                                     activeAccelLsbPerG,
                                     calibrationAccelLsbPerG),
        static_cast<float>(g_lsm.az) -
            RescaleCalibrationCounts(settings::sensors::lsm9ds1::kAccelBias[2],
                                     activeAccelLsbPerG,
                                     calibrationAccelLsbPerG),
    };
    Apply3x3(settings::sensors::lsm9ds1::kAccelAinv, accelCounts, accelCounts);
    ApplyLsmAxisTransform(accelCounts);
    ApplyLsmMountRotation(accelCounts);
    const FusionVector accelG = MakeVector(accelCounts[0] / activeAccelLsbPerG,
                                           accelCounts[1] / activeAccelLsbPerG,
                                           accelCounts[2] / activeAccelLsbPerG);

    const float magBiasX = RescaleCalibrationCounts(settings::sensors::lsm9ds1::kMagBias[0],
                                                    1.0f / activeMagGaussPerLsb,
                                                    1.0f / calibrationMagGaussPerLsb);
    const float magBiasY = RescaleCalibrationCounts(settings::sensors::lsm9ds1::kMagBias[1],
                                                    1.0f / activeMagGaussPerLsb,
                                                    1.0f / calibrationMagGaussPerLsb);
    const float magBiasZ = RescaleCalibrationCounts(settings::sensors::lsm9ds1::kMagBias[2],
                                                    1.0f / activeMagGaussPerLsb,
                                                    1.0f / calibrationMagGaussPerLsb);
    float magCounts[3] = {
        static_cast<float>(g_lsm.mx) - magBiasX,
        static_cast<float>(g_lsm.my) - magBiasY,
        static_cast<float>(g_lsm.mz) - magBiasZ,
    };
    const float lsmRawMag[3] = {
        static_cast<float>(g_lsm.mx),
        static_cast<float>(g_lsm.my),
        static_cast<float>(g_lsm.mz),
    };
    Apply3x3(settings::sensors::lsm9ds1::kMagAinv, magCounts, magCounts);
    ApplyLsmAxisTransform(magCounts);
    ApplyLsmMountRotation(magCounts);
    const FusionVector magBody = MakeVector(magCounts[0], magCounts[1], magCounts[2]);

    g_lsmOutput.timestampUs = nowUs;
    UpdateFusionOutput(g_lsmAhrs,
                       g_lsmBias,
                       MakeVector(gyroDps[0], gyroDps[1], gyroDps[2]),
                       accelG,
                       lsmRawMag,
                       magBody,
                       dtSeconds,
                       settings::sensors::lsm9ds1::kMagDeclinationDeg,
                       false,
                       g_lsmOutput);
    return true;
}

bool UpdateIcmRail() {
    const uint32_t nowUs = micros();
    if (g_icmLastSampleUs != 0 &&
        (nowUs - g_icmLastSampleUs) < settings::sensors::icm20948::kSampleIntervalUs) {
        return false;
    }
    if (!g_icm.dataReady()) {
        return false;
    }

    g_icm.getAGMT();

    float dtSeconds = static_cast<float>(settings::sensors::icm20948::kSampleIntervalUs) * 1.0e-6f;
    if (g_icmLastSampleUs != 0) {
        dtSeconds = static_cast<float>(nowUs - g_icmLastSampleUs) * 1.0e-6f;
        if (!(dtSeconds > 0.0f) || dtSeconds > 0.1f) {
            dtSeconds = static_cast<float>(settings::sensors::icm20948::kSampleIntervalUs) * 1.0e-6f;
        }
    }
    g_icmLastSampleUs = nowUs;

    const float activeAccelLsbPerG = IcmAccelLsbPerGForRange(settings::sensors::icm20948::kAccelRangeG);
    const float activeGyroLsbPerDps = IcmGyroLsbPerDpsForRange(settings::sensors::icm20948::kGyroRangeDps);
    const float calibrationAccelLsbPerG =
        IcmAccelLsbPerGForRange(settings::sensors::icm20948::kCalibrationAccelRangeG);
    const float calibrationGyroLsbPerDps =
        IcmGyroLsbPerDpsForRange(settings::sensors::icm20948::kCalibrationGyroRangeDps);

    const float icmTemperatureC = g_icm.temp();
    const float icmTemperatureDeltaC =
        icmTemperatureC - settings::sensors::icm20948::kGyroReferenceTemperatureC;

    float gyroCounts[3] = {
        static_cast<float>(g_icm.agmt.gyr.axes.x) -
            RescaleCalibrationCounts(settings::sensors::icm20948::kGyroOffset[0],
                                     activeGyroLsbPerDps,
                                     calibrationGyroLsbPerDps),
        static_cast<float>(g_icm.agmt.gyr.axes.y) -
            RescaleCalibrationCounts(settings::sensors::icm20948::kGyroOffset[1],
                                     activeGyroLsbPerDps,
                                     calibrationGyroLsbPerDps),
        static_cast<float>(g_icm.agmt.gyr.axes.z) -
            RescaleCalibrationCounts(settings::sensors::icm20948::kGyroOffset[2],
                                     activeGyroLsbPerDps,
                                     calibrationGyroLsbPerDps),
    };
    float gyroDps[3] = {
        gyroCounts[0] / activeGyroLsbPerDps,
        gyroCounts[1] / activeGyroLsbPerDps,
        gyroCounts[2] / activeGyroLsbPerDps,
    };
    gyroDps[0] -= settings::sensors::icm20948::kGyroTempBiasSlopeRadPerSecPerC[0] * icmTemperatureDeltaC *
                  kRadiansToDegrees;
    gyroDps[1] -= settings::sensors::icm20948::kGyroTempBiasSlopeRadPerSecPerC[1] * icmTemperatureDeltaC *
                  kRadiansToDegrees;
    gyroDps[2] -= settings::sensors::icm20948::kGyroTempBiasSlopeRadPerSecPerC[2] * icmTemperatureDeltaC *
                  kRadiansToDegrees;
    Apply3x3(settings::sensors::icm20948::kGyroAinv, gyroDps, gyroDps);
    ApplyIcmMountRotation(gyroDps);

    float accelCounts[3] = {
        static_cast<float>(g_icm.agmt.acc.axes.x) -
            RescaleCalibrationCounts(settings::sensors::icm20948::kAccelBias[0],
                                     activeAccelLsbPerG,
                                     calibrationAccelLsbPerG),
        static_cast<float>(g_icm.agmt.acc.axes.y) -
            RescaleCalibrationCounts(settings::sensors::icm20948::kAccelBias[1],
                                     activeAccelLsbPerG,
                                     calibrationAccelLsbPerG),
        static_cast<float>(g_icm.agmt.acc.axes.z) -
            RescaleCalibrationCounts(settings::sensors::icm20948::kAccelBias[2],
                                     activeAccelLsbPerG,
                                     calibrationAccelLsbPerG),
    };
    Apply3x3(settings::sensors::icm20948::kAccelAinv, accelCounts, accelCounts);
    float accelGArray[3] = {
        accelCounts[0] / activeAccelLsbPerG,
        accelCounts[1] / activeAccelLsbPerG,
        accelCounts[2] / activeAccelLsbPerG,
    };
    ApplyIcmMountRotation(accelGArray);
    const FusionVector accelG = MakeVector(accelGArray[0], accelGArray[1], accelGArray[2]);

    float magBodyArray[3] = {
        static_cast<float>(g_icm.agmt.mag.axes.x) - settings::sensors::icm20948::kMagBias[0],
        static_cast<float>(g_icm.agmt.mag.axes.y) - settings::sensors::icm20948::kMagBias[1],
        static_cast<float>(g_icm.agmt.mag.axes.z) - settings::sensors::icm20948::kMagBias[2],
    };
    const float icmRawMag[3] = {
        static_cast<float>(g_icm.agmt.mag.axes.x),
        static_cast<float>(g_icm.agmt.mag.axes.y),
        static_cast<float>(g_icm.agmt.mag.axes.z),
    };
    Apply3x3(settings::sensors::icm20948::kMagAinv, magBodyArray, magBodyArray);
    ApplyIcmMagAxisTransform(magBodyArray);
    ApplyIcmMountRotation(magBodyArray);
    const FusionVector magBody = MakeVector(magBodyArray[0], magBodyArray[1], magBodyArray[2]);

    g_icmOutput.timestampUs = nowUs;
    UpdateFusionOutput(g_icmAhrs,
                       g_icmBias,
                       MakeVector(gyroDps[0], gyroDps[1], gyroDps[2]),
                       accelG,
                       icmRawMag,
                       magBody,
                       dtSeconds,
                       settings::sensors::icm20948::kMagDeclinationDeg,
                       true,
                       g_icmOutput);
    return true;
}

void PrintHeader() {
    Serial.println(
        "lsm_ax_g,lsm_ay_g,lsm_az_g,lsm_raw_mx,lsm_raw_my,lsm_raw_mz,lsm_mx,lsm_my,lsm_mz,lsm_roll_deg,lsm_pitch_deg,lsm_yaw_deg,"
        "icm_ax_g,icm_ay_g,icm_az_g,icm_raw_mx,icm_raw_my,icm_raw_mz,icm_mx,icm_my,icm_mz,icm_roll_deg,icm_pitch_deg,icm_yaw_deg,"
        "delta_roll_deg,delta_pitch_deg,delta_yaw_deg");
}

void PrintComparison() {
    const float deltaRoll = WrapSigned180(g_lsmOutput.rollDeg - g_icmOutput.rollDeg);
    const float deltaPitch = WrapSigned180(g_lsmOutput.pitchDeg - g_icmOutput.pitchDeg);
    const float deltaYaw = WrapSigned180(g_lsmOutput.yawDeg - g_icmOutput.yawDeg);

    // Serial.print(g_lsmOutput.accelG[0], 6);
    // Serial.print(',');
    // Serial.print(g_lsmOutput.accelG[1], 6);
    // Serial.print(',');
    // Serial.print(g_lsmOutput.accelG[2], 6);
    // Serial.print(',');
    // Serial.print(g_lsmOutput.rawMag[0], 3);
    // Serial.print(',');
    // Serial.print(g_lsmOutput.rawMag[1], 3);
    // Serial.print(',');
    // Serial.print(g_lsmOutput.rawMag[2], 3);
    // Serial.print(',');
    // Serial.print(g_lsmOutput.magBody[0], 6);
    // Serial.print(',');
    // Serial.print(g_lsmOutput.magBody[1], 6);
    // Serial.print(',');
    // Serial.print(g_lsmOutput.magBody[2], 6);
    Serial.print(',');
    Serial.print(g_lsmOutput.rollDeg, 3);
    Serial.print(',');
    Serial.print(g_lsmOutput.pitchDeg, 3);
    Serial.print(',');
    Serial.print(g_lsmOutput.yawDeg, 3);
    // Serial.print(',');
    // Serial.print(g_icmOutput.accelG[0], 6);
    // Serial.print(',');
    // Serial.print(g_icmOutput.accelG[1], 6);
    // Serial.print(',');
    // Serial.print(g_icmOutput.accelG[2], 6);
    Serial.print(',');
    // Serial.print(g_icmOutput.rawMag[0], 3);
    // Serial.print(',');
    // Serial.print(g_icmOutput.rawMag[1], 3);
    // Serial.print(',');
    // Serial.print(g_icmOutput.rawMag[2], 3);
    // Serial.print(',');
    // Serial.print(g_icmOutput.magBody[0], 6);
    // Serial.print(',');
    // Serial.print(g_icmOutput.magBody[1], 6);
    // Serial.print(',');
    // Serial.print(g_icmOutput.magBody[2], 6);
    Serial.print(',');
    Serial.print(g_icmOutput.rollDeg, 3);
    Serial.print(',');
    Serial.print(g_icmOutput.pitchDeg, 3);
    Serial.print(',');
    Serial.print(g_icmOutput.yawDeg, 3);
    Serial.print(',');
    Serial.print(deltaRoll, 3);
    Serial.print(',');
    Serial.print(deltaPitch, 3);
    Serial.print(',');
    Serial.println(deltaYaw, 3);
}

}  // namespace

void setup() {
    Serial.begin(kSerialBaud);
    const uint32_t startMs = millis();
    while (!Serial && (millis() - startMs) < 3000) {
    }

    SPI1.begin();

    Serial.println("ahrs_testing: LSM9DS1 + ICM-20948 + Fusion");

    if (!ConfigureLsmSensor()) {
        Serial.println("Failed to initialize the LSM9DS1 over SPI1.");
        while (true) {
            delay(1000);
        }
    }
    if (!ConfigureIcmSensor()) {
        Serial.println("Failed to initialize the ICM-20948 over SPI1.");
        while (true) {
            delay(1000);
        }
    }

    ConfigureFusion(g_lsmAhrs,
                    g_lsmBias,
                    1000000.0f / static_cast<float>(settings::sensors::lsm9ds1::kSampleIntervalUs),
                    static_cast<float>(settings::sensors::lsm9ds1::kGyroRangeDps));
    ConfigureFusion(g_icmAhrs,
                    g_icmBias,
                    1000000.0f / static_cast<float>(settings::sensors::icm20948::kSampleIntervalUs),
                    static_cast<float>(settings::sensors::icm20948::kGyroRangeDps));

    PrintHeader();
}

void loop() {
    UpdateLsmRail();
    UpdateIcmRail();

    if (!(g_lsmOutput.valid && g_icmOutput.valid)) {
        return;
    }

    const uint32_t nowMs = millis();
    if ((nowMs - g_lastPrintMs) < kPrintIntervalMs) {
        return;
    }
    g_lastPrintMs = nowMs;
    PrintComparison();
}
