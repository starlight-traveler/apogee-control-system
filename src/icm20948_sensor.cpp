#include "icm20948_sensor.h"

#include <Arduino.h>
#include <SPI.h>

#include <ICM_20948.h>

#include "math_utils.h"
#include "serial_logging.h"
#include "settings.h"

namespace {

constexpr uint8_t kChipSelectPin = settings::sensors::icm20948::kChipSelectPin;
constexpr int8_t kInterruptPin = settings::sensors::icm20948::kInterruptPin;
constexpr uint32_t kSampleIntervalUs = settings::sensors::icm20948::kSampleIntervalUs;
constexpr uint16_t kAccelSampleRateDivider = settings::sensors::icm20948::kAccelSampleRateDivider;
constexpr uint8_t kGyroSampleRateDivider = settings::sensors::icm20948::kGyroSampleRateDivider;
constexpr float kGToMps2 = 9.80665f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadToDeg = 57.295779513082320876f;
constexpr uint8_t kQuaternionInvalidDropThreshold = 2;
constexpr uint8_t kDmpFastSampleRateDivider = 4;  // 225 Hz fast-DMP mode from SparkFun Example10.
constexpr float kDmpQuaternionScale = 1073741824.0f;  // 2^30
constexpr uint8_t kDmpMaxDrainFrames = 8;
constexpr uint8_t kMaxConsecutiveReadFailures = 3;
constexpr uint8_t kMaxConsecutiveDmpFailures = 2;

enum class DmpReadResult : uint8_t {
    NoData = 0,
    QuaternionRead,
    Failure,
};

ICM_20948_SPI g_icm;
volatile bool g_dataReadyInterrupt = false;
bool g_initialized = false;
bool g_hasCachedSample = false;
uint32_t g_lastSampleUs = 0;
uint32_t g_lastFilterUs = 0;

float g_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float g_gyroBias[3] = {0.0f, 0.0f, 0.0f};
float g_lastAccel[3] = {0.0f, 0.0f, 0.0f};
float g_lastAccelPreMountG[3] = {0.0f, 0.0f, 0.0f};
float g_lastMagPreAxis[3] = {0.0f, 0.0f, 0.0f};
float g_lastMagBody[3] = {0.0f, 0.0f, 0.0f};
float g_lastGyro[3] = {0.0f, 0.0f, 0.0f};
float g_lastIcmYprDeg[3] = {0.0f, 0.0f, 0.0f};
float g_lastBootstrapYprDeg[3] = {0.0f, 0.0f, 0.0f};
float g_lastTemperatureC = 0.0f;
float g_lastAhrsDt = 0.0f;
float g_lastAccelTrust = 0.0f;
float g_lastMagTrust = 0.0f;
bool g_lastAccelSaturated = false;
bool g_lastGyroSaturated = false;
bool g_lastRailConstrained = false;
bool g_hasMagReference = false;
float g_magReferenceNorm = 0.0f;
bool g_hasEarthMagReference = false;
float g_magReferenceEarth[3] = {0.0f, 1.0f, 0.0f};
FlightStatus g_flightStatus = FlightStatus::Ground;
FlightStatus g_lastAppliedFlightStatus = FlightStatus::Ground;
float g_activeAccelLsbPerG = 16384.0f;
float g_activeGyroLsbPerDps = 131.0f;
float g_activeGyroRadPerSecPerLsb = (250.0f / 32768.0f) * (kPi / 180.0f);
float g_calibrationAccelLsbPerG = 16384.0f;
float g_calibrationGyroLsbPerDps = 131.0f;
float g_accelSaturationCounts = settings::sensors::icm20948::kAccelSaturationFraction * 32767.0f;
float g_gyroSaturationCounts = settings::sensors::icm20948::kGyroSaturationFraction * 32767.0f;
bool g_hasQuaternionContinuityReference = false;
float g_lastContinuousQuaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
bool g_hasRailReferenceQuaternion = false;
float g_railReferenceQuaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
bool g_railConstraintActive = false;
uint32_t g_burnStartUs = 0;
float g_burnoutTimestampSeconds = 0.0f;
float g_currentTimestampSeconds = 0.0f;
bool g_groundAlignmentReady = false;
float g_groundAlignmentAccelSum[3] = {0.0f, 0.0f, 0.0f};
float g_groundAlignmentMagSum[3] = {0.0f, 0.0f, 0.0f};
uint16_t g_groundAlignmentSampleCount = 0;
bool g_lastAcquireFresh = false;
bool g_lastAcquireUsedCache = false;
bool g_lastAcquireUsedInterrupt = false;
bool g_interruptConfigured = false;
bool g_dmpQuaternionActive = false;
float g_crossCheckTrust = 1.0f;
uint8_t g_consecutiveReadFailures = 0;
uint8_t g_consecutiveDmpFailures = 0;

// Outlier detection state
bool g_hasPreviousGyro = false;
float g_previousGyro[3] = {0.0f, 0.0f, 0.0f};
bool g_hasPreviousAccel = false;
float g_previousAccel[3] = {0.0f, 0.0f, 0.0f};
uint32_t g_outlierGyroCount = 0;
uint32_t g_outlierAccelCount = 0;

// Gyro bias learning validation: track accelerometer magnitude for sanity check.
float g_lastAccelMagnitudeG = 1.0f;
bool g_lastQuaternionOutputValid = false;
bool g_lastBootstrapYprValid = false;
uint8_t g_invalidQuaternionStreak = 0;
float g_lastQuaternionOutput[4] = {1.0f, 0.0f, 0.0f, 0.0f};

void DataReadyISR() {
    g_dataReadyInterrupt = true;
}

bool Normalize3(float &x, float &y, float &z);
void Cross3(float ax, float ay, float az, float bx, float by, float bz, float out[3]);
bool QuaternionFromEarthBasisInBody(const float northBody[3],
                                    const float eastBody[3],
                                    const float upBody[3],
                                    float quaternion[4]);

void ApplyMountRotation(const float in[3], float out[3]) {
    for (int row = 0; row < 3; ++row) {
        out[row] = settings::sensors::icm20948::kMountRotation[row][0] * in[0] +
                   settings::sensors::icm20948::kMountRotation[row][1] * in[1] +
                   settings::sensors::icm20948::kMountRotation[row][2] * in[2];
    }
}

bool QuaternionFromUpVector(const float upIn[3], float quaternion[4]) {
    float upBody[3] = {upIn[0], upIn[1], upIn[2]};
    if (!Normalize3(upBody[0], upBody[1], upBody[2])) {
        return false;
    }

    float northSeed[3] = {1.0f, 0.0f, 0.0f};
    if (fabsf(upBody[0]) > 0.9f) {
        northSeed[0] = 0.0f;
        northSeed[1] = 1.0f;
    }

    float eastBody[3] = {0.0f, 0.0f, 0.0f};
    Cross3(upBody[0], upBody[1], upBody[2], northSeed[0], northSeed[1], northSeed[2], eastBody);
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

/// Clamps a scalar into [-1, 1] before inverse-trig use.
inline float ClampUnit(float value) {
    if (value < -1.0f) {
        return -1.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
}

/// Computes soft saturation trust that ramps smoothly from 1.0 to 0.0
/// as the value approaches the saturation threshold.
/// Returns 1.0 below warning threshold, 0.0 at or above saturation.
inline float ComputeSoftSaturationTrust(float absValue, float saturationThreshold) {
    if (!settings::ahrs::kEnableSoftSaturation) {
        // Binary saturation when feature is disabled.
        return (absValue >= saturationThreshold) ? 0.0f : 1.0f;
    }
    const float warningThreshold = settings::ahrs::kSoftSaturationWarningFraction * saturationThreshold;
    if (absValue <= warningThreshold) {
        return 1.0f;
    }
    if (absValue >= saturationThreshold) {
        return 0.0f;
    }
    // Linear ramp between warning and saturation.
    const float ratio = (saturationThreshold - absValue) / (saturationThreshold - warningThreshold);
    return (ratio < 0.0f) ? 0.0f : ((ratio > 1.0f) ? 1.0f : ratio);
}

/// Checks if a gyro rate of change is implausible (outlier detection).
/// Returns a trust factor between 0.0 (outlier) and 1.0 (normal).
inline float ComputeGyroOutlierTrust(const float current[3], const float previous[3], float dtSec) {
    if (!settings::ahrs::kEnableOutlierDetection || dtSec <= 0.0f) {
        return 1.0f;
    }
    const float maxRateChange = settings::ahrs::kGyroMaxRateChangeRadPerSecSq * dtSec;
    float maxDiff = 0.0f;
    for (int i = 0; i < 3; ++i) {
        const float diff = fabsf(current[i] - previous[i]);
        if (diff > maxDiff) {
            maxDiff = diff;
        }
    }
    if (maxDiff <= maxRateChange) {
        return 1.0f;
    }
    // Soft rejection: ramp from 1.0 at maxRateChange to 0.0 at 2x maxRateChange.
    const float overLimit = maxDiff - maxRateChange;
    const float trust = 1.0f - overLimit / maxRateChange;
    return (trust < 0.0f) ? 0.0f : ((trust > 1.0f) ? 1.0f : trust);
}

/// Checks if an accel rate of change is implausible (outlier detection).
/// Returns a trust factor between 0.0 (outlier) and 1.0 (normal).
inline float ComputeAccelOutlierTrust(const float current[3], const float previous[3], float dtSec) {
    if (!settings::ahrs::kEnableOutlierDetection || dtSec <= 0.0f) {
        return 1.0f;
    }
    const float maxRateChange = settings::ahrs::kAccelMaxRateChangeMps3 * dtSec;
    float maxDiff = 0.0f;
    for (int i = 0; i < 3; ++i) {
        const float diff = fabsf(current[i] - previous[i]);
        if (diff > maxDiff) {
            maxDiff = diff;
        }
    }
    if (maxDiff <= maxRateChange) {
        return 1.0f;
    }
    const float overLimit = maxDiff - maxRateChange;
    const float trust = 1.0f - overLimit / maxRateChange;
    return (trust < 0.0f) ? 0.0f : ((trust > 1.0f) ? 1.0f : trust);
}

/// Clamps a scalar into [0, 1].
inline float Clamp01(float value) {
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
}

/// Returns the ICM accelerometer sensitivity for the requested full-scale range.
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

/// Returns the ICM gyroscope sensitivity for the requested full-scale range.
float GyroLsbPerDpsForRange(uint16_t rangeDps) {
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

/// Returns the gyroscope conversion scale in rad/s per raw count.
float GyroRadPerSecPerLsbForRange(uint16_t rangeDps) {
    return (static_cast<float>(rangeDps) / 32768.0f) * (kPi / 180.0f);
}

/// Maps the configured accel range to the SparkFun library enum.
ICM_20948_ACCEL_CONFIG_FS_SEL_e AccelFullScaleEnum(uint8_t rangeG) {
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

/// Maps the configured gyro range to the SparkFun library enum.
ICM_20948_GYRO_CONFIG_1_FS_SEL_e GyroFullScaleEnum(uint16_t rangeDps) {
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

#if defined(ICM_20948_USE_DMP)
uint8_t DmpGyroLevel(uint16_t rangeDps) {
    switch (rangeDps) {
        case 250:
            return 0;
        case 500:
            return 1;
        case 1000:
            return 2;
        case 2000:
            return 3;
        default:
            return 3;
    }
}

// SparkFun documents the 4g pair only: ACC_SCALE=0x04000000 and
// ACC_SCALE2=0x00040000. The DMP keeps 1g at 2^25 internally, so when FSR
// increases ACC_SCALE must grow proportionally while ACC_SCALE2 shrinks by the
// same factor to keep exported raw units matched to the configured range.
uint32_t DmpAccelScaleValue(uint8_t rangeG) {
    switch (rangeG) {
        case 2:
            return 0x02000000u;
        case 4:
            return 0x04000000u;
        case 8:
            return 0x08000000u;
        case 16:
            return 0x10000000u;
        default:
            return 0x04000000u;
    }
}

uint32_t DmpAccelScale2Value(uint8_t rangeG) {
    switch (rangeG) {
        case 2:
            return 0x00080000u;
        case 4:
            return 0x00040000u;
        case 8:
            return 0x00020000u;
        case 16:
            return 0x00010000u;
        default:
            return 0x00040000u;
    }
}

uint32_t DmpGyroFullScaleValue(uint16_t rangeDps) {
    switch (rangeDps) {
        case 250:
            return 0x02000000u;
        case 500:
            return 0x04000000u;
        case 1000:
            return 0x08000000u;
        case 2000:
            return 0x10000000u;
        default:
            return 0x10000000u;
    }
}

void EncodeBigEndianU32(uint32_t value, unsigned char bytes[4]) {
    bytes[0] = static_cast<unsigned char>((value >> 24) & 0xffu);
    bytes[1] = static_cast<unsigned char>((value >> 16) & 0xffu);
    bytes[2] = static_cast<unsigned char>((value >> 8) & 0xffu);
    bytes[3] = static_cast<unsigned char>(value & 0xffu);
}
#endif

/// Returns the raw-count equivalent of a calibration term at the active range.
float RescaleCalibrationCounts(float calibrationCounts, float activeLsbPerUnit, float calibrationLsbPerUnit) {
    if (!(calibrationLsbPerUnit > 0.0f)) {
        return calibrationCounts;
    }
    return calibrationCounts * (activeLsbPerUnit / calibrationLsbPerUnit);
}

float QuaternionDot(const float a[4], const float b[4]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

void NegateQuaternion(float q[4]) {
    q[0] = -q[0];
    q[1] = -q[1];
    q[2] = -q[2];
    q[3] = -q[3];
}

void ApplyQuaternionContinuity() {
    if (!g_hasQuaternionContinuityReference) {
        g_lastContinuousQuaternion[0] = g_q[0];
        g_lastContinuousQuaternion[1] = g_q[1];
        g_lastContinuousQuaternion[2] = g_q[2];
        g_lastContinuousQuaternion[3] = g_q[3];
        g_hasQuaternionContinuityReference = true;
        return;
    }

    if (QuaternionDot(g_q, g_lastContinuousQuaternion) < 0.0f) {
        NegateQuaternion(g_q);
    }
    g_lastContinuousQuaternion[0] = g_q[0];
    g_lastContinuousQuaternion[1] = g_q[1];
    g_lastContinuousQuaternion[2] = g_q[2];
    g_lastContinuousQuaternion[3] = g_q[3];
}

void QuaternionConjugate(const float q[4], float out[4]) {
    out[0] = q[0];
    out[1] = -q[1];
    out[2] = -q[2];
    out[3] = -q[3];
}

void QuaternionMultiply(const float a[4], const float b[4], float out[4]) {
    out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

void ResetGroundAlignment() {
    g_groundAlignmentReady = false;
    g_groundAlignmentSampleCount = 0;
    g_groundAlignmentAccelSum[0] = 0.0f;
    g_groundAlignmentAccelSum[1] = 0.0f;
    g_groundAlignmentAccelSum[2] = 0.0f;
    g_groundAlignmentMagSum[0] = 0.0f;
    g_groundAlignmentMagSum[1] = 0.0f;
    g_groundAlignmentMagSum[2] = 0.0f;
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

bool QuaternionFromAccelMag(const float accelNorm[3], const float magNorm[3], float quaternion[4]) {
    float upBody[3] = {accelNorm[0], accelNorm[1], accelNorm[2]};
    float magneticBody[3] = {magNorm[0], magNorm[1], magNorm[2]};
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

    float aligned[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    if (!QuaternionFromEarthBasisInBody(northBody, eastBody, upBody, aligned)) {
        return false;
    }

    quaternion[0] = aligned[0];
    quaternion[1] = aligned[1];
    quaternion[2] = aligned[2];
    quaternion[3] = aligned[3];
    return true;
}

void UpdateBootstrapYprDiagnostics(const float accelNorm[3], const float magNorm[3]) {
    float bootstrapQuaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    if (!QuaternionFromAccelMag(accelNorm, magNorm, bootstrapQuaternion)) {
        g_lastBootstrapYprValid = false;
        return;
    }

    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    const math_utils::Quaternion quat = math_utils::MakeQuaternion(bootstrapQuaternion[0],
                                                                   bootstrapQuaternion[1],
                                                                   bootstrapQuaternion[2],
                                                                   bootstrapQuaternion[3]);
    math_utils::QuaternionToEuler(quat, yaw, pitch, roll);
    g_lastBootstrapYprDeg[0] = yaw * kRadToDeg;
    g_lastBootstrapYprDeg[1] = pitch * kRadToDeg;
    // Keep the bootstrap Euler diagnostics in the same body-frame convention as
    // the steady-state quaternion output.
    g_lastBootstrapYprDeg[2] = roll * kRadToDeg;
    g_lastBootstrapYprValid = true;
}

bool InitializeQuaternionFromAccelMag(const float accelNorm[3], const float magNorm[3]) {
    float aligned[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    if (!QuaternionFromAccelMag(accelNorm, magNorm, aligned)) {
        return false;
    }

    g_q[0] = aligned[0];
    g_q[1] = aligned[1];
    g_q[2] = aligned[2];
    g_q[3] = aligned[3];
    ApplyQuaternionContinuity();
    g_groundAlignmentReady = true;
    return true;
}

/// Normalizes a 3-vector in place.
/// Uses FastInvSqrt for better performance on ARM targets.
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

/// Returns the Euclidean norm of a 3-vector.
/// Uses FastSqrt for better performance on ARM targets.
float Magnitude3(float x, float y, float z) {
    return math_utils::FastSqrt(x * x + y * y + z * z);
}

/// Computes the cross product a x b.
void Cross3(float ax, float ay, float az, float bx, float by, float bz, float out[3]) {
    out[0] = ay * bz - az * by;
    out[1] = az * bx - ax * bz;
    out[2] = ax * by - ay * bx;
}

/// Limits a vector's norm without changing its direction.
void LimitVector(float vector[3], float maxNorm) {
    if (maxNorm <= 0.0f) {
        vector[0] = 0.0f;
        vector[1] = 0.0f;
        vector[2] = 0.0f;
        return;
    }
    const float norm = Magnitude3(vector[0], vector[1], vector[2]);
    if (norm <= maxNorm || norm <= 1.0e-9f) {
        return;
    }
    const float scale = maxNorm / norm;
    vector[0] *= scale;
    vector[1] *= scale;
    vector[2] *= scale;
}

/// Returns a triangular trust window centered between minValue and maxValue.
float WindowTrust(float value, float minValue, float maxValue) {
    if (!(value >= minValue) || !(value <= maxValue) || !(maxValue > minValue)) {
        return 0.0f;
    }
    const float center = 0.5f * (minValue + maxValue);
    const float halfWidth = 0.5f * (maxValue - minValue);
    if (halfWidth <= 0.0f) {
        return 0.0f;
    }
    return Clamp01(1.0f - fabsf(value - center) / halfWidth);
}

/// Returns a descending trust window that stays at 1 below start and 0 above end.
float DescendingTrust(float value, float fullTrustMax, float zeroTrustMin) {
    if (value <= fullTrustMax) {
        return 1.0f;
    }
    if (value >= zeroTrustMin || !(zeroTrustMin > fullTrustMax)) {
        return 0.0f;
    }
    return Clamp01((zeroTrustMin - value) / (zeroTrustMin - fullTrustMax));
}

/// Rotates a body-frame vector into the Earth frame.
void RotateBodyToEarth(const float q[4], const float body[3], float earth[3]) {
    const float w = q[0];
    const float x = q[1];
    const float y = q[2];
    const float z = q[3];

    const float r00 = 1.0f - 2.0f * (y * y + z * z);
    const float r01 = 2.0f * (x * y - w * z);
    const float r02 = 2.0f * (x * z + w * y);
    const float r10 = 2.0f * (x * y + w * z);
    const float r11 = 1.0f - 2.0f * (x * x + z * z);
    const float r12 = 2.0f * (y * z - w * x);
    const float r20 = 2.0f * (x * z - w * y);
    const float r21 = 2.0f * (y * z + w * x);
    const float r22 = 1.0f - 2.0f * (x * x + y * y);

    earth[0] = r00 * body[0] + r10 * body[1] + r20 * body[2];
    earth[1] = r01 * body[0] + r11 * body[1] + r21 * body[2];
    earth[2] = r02 * body[0] + r12 * body[1] + r22 * body[2];
}

/// Applies a 3x3 calibration matrix to a sensor vector.
void Apply3x3(const float matrix[3][3], const float in[3], float out[3]) {
    out[0] = matrix[0][0] * in[0] + matrix[0][1] * in[1] + matrix[0][2] * in[2];
    out[1] = matrix[1][0] * in[0] + matrix[1][1] * in[1] + matrix[1][2] * in[2];
    out[2] = matrix[2][0] * in[0] + matrix[2][1] * in[1] + matrix[2][2] * in[2];
}

void ApplyMountRotation(float vector[3]) {
    float rotated[3] = {0.0f, 0.0f, 0.0f};
    Apply3x3(settings::sensors::icm20948::kMountRotation, vector, rotated);
    vector[0] = rotated[0];
    vector[1] = rotated[1];
    vector[2] = rotated[2];
}

void ApplyGyroCalibration(float vector[3]) {
    float corrected[3] = {0.0f, 0.0f, 0.0f};
    Apply3x3(settings::sensors::icm20948::kGyroAinv, vector, corrected);
    vector[0] = corrected[0];
    vector[1] = corrected[1];
    vector[2] = corrected[2];
}

void ApplyMagAxisTransform(float vector[3]) {
    float remapped[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 3; ++i) {
        const uint8_t source = settings::sensors::icm20948::kMagAxisMap[i];
        remapped[i] = static_cast<float>(settings::sensors::icm20948::kMagAxisSign[i]) * vector[source];
    }
    vector[0] = remapped[0];
    vector[1] = remapped[1];
    vector[2] = remapped[2];
}

/// Applies stored calibration to gyro/accel/mag and reports both the
/// calibrated accel vector in g-units and normalized vectors for AHRS gating.
/// Now also computes soft saturation trust values.
void GetScaledImu(float gyroRadPerSec[3],
                  float accelCalG[3],
                  float accelNorm[3],
                  float magNorm[3],
                  float &accelMagnitudeG,
                  float &magMagnitude,
                  float &temperatureC,
                  bool &accelSaturated,
                  bool &gyroSaturated,
                  float &gyroSaturationTrust,
                  float &accelSaturationTrust) {
    const float rawGx = static_cast<float>(g_icm.agmt.gyr.axes.x);
    const float rawGy = static_cast<float>(g_icm.agmt.gyr.axes.y);
    const float rawGz = static_cast<float>(g_icm.agmt.gyr.axes.z);
    const float rawAx = static_cast<float>(g_icm.agmt.acc.axes.x);
    const float rawAy = static_cast<float>(g_icm.agmt.acc.axes.y);
    const float rawAz = static_cast<float>(g_icm.agmt.acc.axes.z);

    // Binary saturation check (legacy behavior).
    gyroSaturated = fabsf(rawGx) >= g_gyroSaturationCounts ||
                    fabsf(rawGy) >= g_gyroSaturationCounts ||
                    fabsf(rawGz) >= g_gyroSaturationCounts;
    accelSaturated = fabsf(rawAx) >= g_accelSaturationCounts ||
                     fabsf(rawAy) >= g_accelSaturationCounts ||
                     fabsf(rawAz) >= g_accelSaturationCounts;

    // Soft saturation trust: smooth ramp from 85% to 100% of threshold.
    const float maxAbsGyro = fmaxf(fmaxf(fabsf(rawGx), fabsf(rawGy)), fabsf(rawGz));
    const float maxAbsAccel = fmaxf(fmaxf(fabsf(rawAx), fabsf(rawAy)), fabsf(rawAz));
    gyroSaturationTrust = ComputeSoftSaturationTrust(maxAbsGyro, g_gyroSaturationCounts);
    accelSaturationTrust = ComputeSoftSaturationTrust(maxAbsAccel, g_accelSaturationCounts);

    const float gyroOffsetX = RescaleCalibrationCounts(settings::sensors::icm20948::kGyroOffset[0],
                                                       g_activeGyroLsbPerDps,
                                                       g_calibrationGyroLsbPerDps);
    const float gyroOffsetY = RescaleCalibrationCounts(settings::sensors::icm20948::kGyroOffset[1],
                                                       g_activeGyroLsbPerDps,
                                                       g_calibrationGyroLsbPerDps);
    const float gyroOffsetZ = RescaleCalibrationCounts(settings::sensors::icm20948::kGyroOffset[2],
                                                       g_activeGyroLsbPerDps,
                                                       g_calibrationGyroLsbPerDps);
    temperatureC = g_icm.temp();
    const float temperatureDeltaC = temperatureC - settings::sensors::icm20948::kGyroReferenceTemperatureC;
    gyroRadPerSec[0] = g_activeGyroRadPerSecPerLsb * (rawGx - gyroOffsetX) -
                       settings::sensors::icm20948::kGyroTempBiasSlopeRadPerSecPerC[0] * temperatureDeltaC;
    gyroRadPerSec[1] = g_activeGyroRadPerSecPerLsb * (rawGy - gyroOffsetY) -
                       settings::sensors::icm20948::kGyroTempBiasSlopeRadPerSecPerC[1] * temperatureDeltaC;
    gyroRadPerSec[2] = g_activeGyroRadPerSecPerLsb * (rawGz - gyroOffsetZ) -
                       settings::sensors::icm20948::kGyroTempBiasSlopeRadPerSecPerC[2] * temperatureDeltaC;
    ApplyGyroCalibration(gyroRadPerSec);
    ApplyMountRotation(gyroRadPerSec);

    float rawAccel[3] = {
        rawAx - RescaleCalibrationCounts(settings::sensors::icm20948::kAccelBias[0],
                                         g_activeAccelLsbPerG,
                                         g_calibrationAccelLsbPerG),
        rawAy - RescaleCalibrationCounts(settings::sensors::icm20948::kAccelBias[1],
                                         g_activeAccelLsbPerG,
                                         g_calibrationAccelLsbPerG),
        rawAz - RescaleCalibrationCounts(settings::sensors::icm20948::kAccelBias[2],
                                         g_activeAccelLsbPerG,
                                         g_calibrationAccelLsbPerG),
    };
    float accelCalCounts[3] = {0.0f, 0.0f, 0.0f};
    Apply3x3(settings::sensors::icm20948::kAccelAinv, rawAccel, accelCalCounts);
    accelCalG[0] = accelCalCounts[0] / g_activeAccelLsbPerG;
    accelCalG[1] = accelCalCounts[1] / g_activeAccelLsbPerG;
    accelCalG[2] = accelCalCounts[2] / g_activeAccelLsbPerG;
    g_lastAccelPreMountG[0] = accelCalG[0];
    g_lastAccelPreMountG[1] = accelCalG[1];
    g_lastAccelPreMountG[2] = accelCalG[2];
    ApplyMountRotation(accelCalG);
    accelNorm[0] = accelCalG[0];
    accelNorm[1] = accelCalG[1];
    accelNorm[2] = accelCalG[2];
    accelMagnitudeG = sqrtf(accelCalG[0] * accelCalG[0] + accelCalG[1] * accelCalG[1] + accelCalG[2] * accelCalG[2]);
    Normalize3(accelNorm[0], accelNorm[1], accelNorm[2]);

    float rawMag[3] = {
        static_cast<float>(g_icm.agmt.mag.axes.x) - settings::sensors::icm20948::kMagBias[0],
        static_cast<float>(g_icm.agmt.mag.axes.y) - settings::sensors::icm20948::kMagBias[1],
        static_cast<float>(g_icm.agmt.mag.axes.z) - settings::sensors::icm20948::kMagBias[2],
    };
    Apply3x3(settings::sensors::icm20948::kMagAinv, rawMag, magNorm);
    g_lastMagPreAxis[0] = magNorm[0];
    g_lastMagPreAxis[1] = magNorm[1];
    g_lastMagPreAxis[2] = magNorm[2];
    ApplyMagAxisTransform(magNorm);
    ApplyMountRotation(magNorm);
    g_lastMagBody[0] = magNorm[0];
    g_lastMagBody[1] = magNorm[1];
    g_lastMagBody[2] = magNorm[2];
    magMagnitude = sqrtf(magNorm[0] * magNorm[0] + magNorm[1] * magNorm[1] + magNorm[2] * magNorm[2]);
    Normalize3(magNorm[0], magNorm[1], magNorm[2]);
}

/// Computes trust in the accelerometer gravity proxy for the current phase.
/// During coast, implements burnout correction burst to quickly correct gyro drift.
float ComputeAccelTrust(float accelMagnitudeG, float gyroNorm) {
    float phaseTrust = 0.0f;
    float flightSuppression = 1.0f;
    switch (g_flightStatus) {
        case FlightStatus::Ground:
            phaseTrust = 1.0f;
            break;
        case FlightStatus::Descent:
            phaseTrust = 0.75f;
            break;
        case FlightStatus::Burn:
            // During burn, accelerometer reads thrust, not gravity - no trust
            return 0.0f;
        case FlightStatus::Coast:
        case FlightStatus::Overshoot:
            // Burnout correction burst: aggressive correction right after burnout
            if (settings::ahrs::kEnableBurnoutCorrectionBurst && g_burnoutTimestampSeconds > 0.0f) {
                const float timeSinceBurnout = g_currentTimestampSeconds - g_burnoutTimestampSeconds;
                if (timeSinceBurnout >= 0.0f && timeSinceBurnout < settings::ahrs::kBurnoutCorrectionWindowSeconds) {
                    // Within burnout correction window - high trust to quickly correct drift
                    phaseTrust = settings::ahrs::kBurnoutCorrectionAccelTrust;
                    break;
                }
            }
            // After burnout window, maintain moderate trust for ongoing correction
            phaseTrust = settings::ahrs::kCoastAccelTrust;
            flightSuppression =
                DescendingTrust(fabsf(accelMagnitudeG - 1.0f),
                                settings::ahrs::kFlightAccelDeviationFullTrustG,
                                settings::ahrs::kFlightAccelDeviationZeroTrustG) *
                DescendingTrust(gyroNorm,
                                settings::ahrs::kFlightAccelGyroFadeStartRadPerSec,
                                settings::ahrs::kFlightAccelGyroFadeEndRadPerSec);
            break;
    }

    const float magnitudeTrust = WindowTrust(accelMagnitudeG,
                                             settings::sensors::icm20948::kAccelCorrectionMinG,
                                             settings::sensors::icm20948::kAccelCorrectionMaxG);
    const float rateTrust = DescendingTrust(gyroNorm,
                                            settings::sensors::icm20948::kAccelCorrectionGyroFadeStartRadPerSec,
                                            settings::sensors::icm20948::kAccelCorrectionGyroFadeEndRadPerSec);
    return phaseTrust * magnitudeTrust * rateTrust * flightSuppression;
}

float MagTrustPhaseScale() {
    switch (g_flightStatus) {
        case FlightStatus::Ground:
            return settings::sensors::icm20948::kMagTrustGround;
        case FlightStatus::Burn:
            return settings::sensors::icm20948::kMagTrustBurn;
        case FlightStatus::Coast:
            return settings::sensors::icm20948::kMagTrustCoast;
        case FlightStatus::Overshoot:
            return settings::sensors::icm20948::kMagTrustOvershoot;
        case FlightStatus::Descent:
            return settings::sensors::icm20948::kMagTrustDescent;
    }
    return 0.0f;
}

/// Computes trust in the magnetometer for the current sample without updating the reference.
float ComputeMagTrust(float magMagnitude, float gyroNorm) {
    if (!(magMagnitude > 0.0f) || !isfinite(magMagnitude)) {
        return 0.0f;
    }

    const float phaseTrust = MagTrustPhaseScale();
    if (phaseTrust <= 0.0f) {
        return 0.0f;
    }

    if (!g_hasMagReference) {
        return phaseTrust * DescendingTrust(gyroNorm,
                                            settings::sensors::icm20948::kMagTrustGyroFadeStartRadPerSec,
                                            settings::sensors::icm20948::kMagTrustGyroFadeEndRadPerSec);
    }

    if (!(g_magReferenceNorm > 0.0f) || !isfinite(g_magReferenceNorm)) {
        return 0.0f;
    }

    const float relativeError = fabsf(magMagnitude - g_magReferenceNorm) / g_magReferenceNorm;
    if (relativeError >= settings::sensors::icm20948::kMagCorrectionMaxRelativeError) {
        return 0.0f;
    }

    const float magnitudeTrust =
        Clamp01(1.0f - relativeError / settings::sensors::icm20948::kMagCorrectionMaxRelativeError);
    const float rateTrust = DescendingTrust(gyroNorm,
                                            settings::sensors::icm20948::kMagTrustGyroFadeStartRadPerSec,
                                            settings::sensors::icm20948::kMagTrustGyroFadeEndRadPerSec);
    return phaseTrust * magnitudeTrust * rateTrust;
}

/// Updates the running magnetometer magnitude baseline from trusted samples.
void UpdateMagMagnitudeReference(float magMagnitude, float trust) {
    if (!(magMagnitude > 0.0f) || !isfinite(magMagnitude) || trust <= 0.0f) {
        return;
    }
    if (!g_hasMagReference) {
        g_magReferenceNorm = magMagnitude;
        g_hasMagReference = true;
        return;
    }

    const float blend = settings::sensors::icm20948::kMagReferenceBlend * trust;
    g_magReferenceNorm += blend * (magMagnitude - g_magReferenceNorm);
}

/// Updates the Earth-frame magnetic reference using low-dynamic samples.
void UpdateEarthMagReference(const float magBody[3], float accelTrust, float magTrust) {
    const float referenceTrust = accelTrust * magTrust;
    if (referenceTrust <= 0.0f) {
        return;
    }

    float earthMag[3] = {0.0f, 0.0f, 0.0f};
    RotateBodyToEarth(g_q, magBody, earthMag);
    if (!Normalize3(earthMag[0], earthMag[1], earthMag[2])) {
        return;
    }

    if (!g_hasEarthMagReference) {
        g_magReferenceEarth[0] = earthMag[0];
        g_magReferenceEarth[1] = earthMag[1];
        g_magReferenceEarth[2] = earthMag[2];
        g_hasEarthMagReference = true;
        return;
    }

    const float blend = settings::sensors::icm20948::kMagReferenceBlend * referenceTrust;
    g_magReferenceEarth[0] += blend * (earthMag[0] - g_magReferenceEarth[0]);
    g_magReferenceEarth[1] += blend * (earthMag[1] - g_magReferenceEarth[1]);
    g_magReferenceEarth[2] += blend * (earthMag[2] - g_magReferenceEarth[2]);
    Normalize3(g_magReferenceEarth[0], g_magReferenceEarth[1], g_magReferenceEarth[2]);
}

void UpdateGroundAlignment(const float accelNorm[3], float accelTrust, const float magNorm[3], float magTrust, float gyroNorm) {
    if (g_groundAlignmentReady || g_flightStatus != FlightStatus::Ground) {
        return;
    }
    if (accelTrust < settings::sensors::icm20948::kGroundAlignmentAccelTrustMin ||
        magTrust < settings::sensors::icm20948::kGroundAlignmentMagTrustMin ||
        gyroNorm > settings::sensors::icm20948::kStationaryGyroMaxRadPerSec) {
        g_groundAlignmentSampleCount = 0;
        g_groundAlignmentAccelSum[0] = 0.0f;
        g_groundAlignmentAccelSum[1] = 0.0f;
        g_groundAlignmentAccelSum[2] = 0.0f;
        g_groundAlignmentMagSum[0] = 0.0f;
        g_groundAlignmentMagSum[1] = 0.0f;
        g_groundAlignmentMagSum[2] = 0.0f;
        return;
    }

    for (int i = 0; i < 3; ++i) {
        g_groundAlignmentAccelSum[i] += accelNorm[i];
        g_groundAlignmentMagSum[i] += magNorm[i];
    }
    ++g_groundAlignmentSampleCount;

    if (g_groundAlignmentSampleCount < settings::sensors::icm20948::kGroundAlignmentMinSamples) {
        return;
    }

    float avgAccel[3] = {
        g_groundAlignmentAccelSum[0] / static_cast<float>(g_groundAlignmentSampleCount),
        g_groundAlignmentAccelSum[1] / static_cast<float>(g_groundAlignmentSampleCount),
        g_groundAlignmentAccelSum[2] / static_cast<float>(g_groundAlignmentSampleCount),
    };
    float avgMag[3] = {
        g_groundAlignmentMagSum[0] / static_cast<float>(g_groundAlignmentSampleCount),
        g_groundAlignmentMagSum[1] / static_cast<float>(g_groundAlignmentSampleCount),
        g_groundAlignmentMagSum[2] / static_cast<float>(g_groundAlignmentSampleCount),
    };

    if (InitializeQuaternionFromAccelMag(avgAccel, avgMag)) {
        UpdateEarthMagReference(avgMag, 1.0f, 1.0f);
    }
}

/// Returns the phase-specific proportional gain for accelerometer correction.
/// Returns aggressive gain during burnout correction window.
float AccelCorrectionGain() {
    switch (g_flightStatus) {
        case FlightStatus::Ground:
            return settings::sensors::icm20948::kAccelCorrectionGainGround;
        case FlightStatus::Descent:
            return settings::sensors::icm20948::kAccelCorrectionGainDescent;
        case FlightStatus::Burn:
            return 0.0f;
        case FlightStatus::Coast:
        case FlightStatus::Overshoot:
            // Burnout correction burst: aggressive gain right after burnout
            if (settings::ahrs::kEnableBurnoutCorrectionBurst && g_burnoutTimestampSeconds > 0.0f) {
                const float timeSinceBurnout = g_currentTimestampSeconds - g_burnoutTimestampSeconds;
                if (timeSinceBurnout >= 0.0f && timeSinceBurnout < settings::ahrs::kBurnoutCorrectionWindowSeconds) {
                    return settings::ahrs::kBurnoutCorrectionAccelGain;
                }
            }
            // After burnout window, use moderate correction gain
            return settings::ahrs::kCoastAccelCorrectionGain;
    }
    return 0.0f;
}

/// Returns the phase-specific proportional gain for magnetic correction.
float MagCorrectionGain() {
    switch (g_flightStatus) {
        case FlightStatus::Ground:
            return settings::sensors::icm20948::kMagCorrectionGainGround;
        case FlightStatus::Descent:
            return settings::sensors::icm20948::kMagCorrectionGainDescent;
        case FlightStatus::Burn:
        case FlightStatus::Coast:
        case FlightStatus::Overshoot:
            return settings::sensors::icm20948::kMagCorrectionGainFlight;
    }
    return 0.0f;
}

/// Returns true when residual gyro bias can be learned safely.
/// Validates accelerometer magnitude is ~1g to prevent learning corrupted bias.
bool ShouldLearnGyroBias(float accelTrust, float gyroNorm) {
    // Validate accelerometer magnitude is close to 1g (gravity only).
    // If magnitude is way off, accelerometer may be corrupted - skip bias learning.
    constexpr float kMinAccelMagnitudeG = 0.85f;  // Allow some tolerance
    constexpr float kMaxAccelMagnitudeG = 1.15f;
    if (g_lastAccelMagnitudeG < kMinAccelMagnitudeG || g_lastAccelMagnitudeG > kMaxAccelMagnitudeG) {
        return false;  // Accelerometer not reading ~1g, don't trust for bias learning
    }

    if (g_flightStatus == FlightStatus::Ground) {
        return accelTrust > 0.35f && gyroNorm <= settings::sensors::icm20948::kStationaryGyroMaxRadPerSec;
    }
    if (g_flightStatus == FlightStatus::Descent) {
        return accelTrust > 0.65f && gyroNorm <= 0.5f * settings::sensors::icm20948::kStationaryGyroMaxRadPerSec;
    }
    return false;
}

void LearnGyroBias(const float gyroRadPerSec[3], float accelTrust, float gyroNorm, float dt) {
    if (!(dt > 0.0f) || !ShouldLearnGyroBias(accelTrust, gyroNorm)) {
        return;
    }
    const float alpha = settings::sensors::icm20948::kGyroBiasLearningRate * dt;
    for (int i = 0; i < 3; ++i) {
        g_gyroBias[i] += alpha * (gyroRadPerSec[i] - g_gyroBias[i]);
        g_gyroBias[i] = fmaxf(-settings::sensors::icm20948::kGyroBiasMaxRadPerSec,
                              fminf(settings::sensors::icm20948::kGyroBiasMaxRadPerSec, g_gyroBias[i]));
    }
}

void UpdateRailConstraintState(uint32_t nowUs) {
    if (g_flightStatus != g_lastAppliedFlightStatus) {
        if (g_flightStatus == FlightStatus::Burn && g_lastAppliedFlightStatus == FlightStatus::Ground) {
            g_burnStartUs = nowUs;
            g_railConstraintActive = g_hasRailReferenceQuaternion;
        } else if (g_flightStatus == FlightStatus::Ground) {
            g_railConstraintActive = false;
            g_burnStartUs = 0;
        } else if (g_flightStatus != FlightStatus::Burn) {
            g_railConstraintActive = false;
        }
        g_lastAppliedFlightStatus = g_flightStatus;
    }

    if (g_railConstraintActive && g_flightStatus == FlightStatus::Burn) {
        const float elapsedSeconds = static_cast<float>(nowUs - g_burnStartUs) * 1.0e-6f;
        if (elapsedSeconds >= settings::sensors::icm20948::kRailConstraintDurationSeconds) {
            g_railConstraintActive = false;
        }
    }
}

void CaptureRailReferenceQuaternion(float accelTrust, float magTrust) {
    if (g_flightStatus != FlightStatus::Ground) {
        return;
    }
    if (accelTrust < 0.7f || magTrust < 0.2f) {
        return;
    }
    g_railReferenceQuaternion[0] = g_q[0];
    g_railReferenceQuaternion[1] = g_q[1];
    g_railReferenceQuaternion[2] = g_q[2];
    g_railReferenceQuaternion[3] = g_q[3];
    g_hasRailReferenceQuaternion = true;
}

/// Integrates the quaternion one step using adaptive sensor feedback.
/// Uses the same error computation algorithm as LSM for consistency.
void AdaptiveQuaternionUpdate(const float accelNorm[3],
                              float accelTrust,
                              const float gyroRadPerSec[3],
                              const float magNorm[3],
                              float magTrust,
                              float dt) {
    if (dt <= 0.0f) {
        return;
    }

    float feedback[3] = {0.0f, 0.0f, 0.0f};

    // Compute error corrections using LSM algorithm.
    {
        const float qw = g_q[0];
        const float qx = g_q[1];
        const float qy = g_q[2];
        const float qz = g_q[3];

        // Keep this convention aligned with QuaternionFromEarthBasisInBody()
        // and GravityVectorFromQuaternion().
        const float ux = 2.0f * (qx * qz + qw * qy);
        const float uy = 2.0f * (qy * qz - qw * qx);
        const float uz = qw * qw - qx * qx - qy * qy + qz * qz;

        // Compute predicted east direction in body frame from accel × mag.
        float hx = accelNorm[1] * magNorm[2] - accelNorm[2] * magNorm[1];
        float hy = accelNorm[2] * magNorm[0] - accelNorm[0] * magNorm[2];
        float hz = accelNorm[0] * magNorm[1] - accelNorm[1] * magNorm[0];
        float localMagTrust = magTrust;
        if (!Normalize3(hx, hy, hz)) {
            localMagTrust = 0.0f;
            hx = hy = hz = 0.0f;
        }

        // Expected east from the same body-from-earth rotation matrix.
        const float wx = 2.0f * (qx * qy - qw * qz);
        const float wy = qw * qw - qx * qx + qy * qy - qz * qz;
        const float wz = 2.0f * (qy * qz + qw * qx);

        // Compute errors using cross products.
        float accelError[3] = {
            accelNorm[1] * uz - accelNorm[2] * uy,
            accelNorm[2] * ux - accelNorm[0] * uz,
            accelNorm[0] * uy - accelNorm[1] * ux,
        };
        float magError[3] = {
            hy * wz - hz * wy,
            hz * wx - hx * wz,
            hx * wy - hy * wx,
        };

        // Apply trust-weighted gains.
        const float accelGain = accelTrust * AccelCorrectionGain();
        const float magGain = localMagTrust * MagCorrectionGain();
        for (int i = 0; i < 3; ++i) {
            accelError[i] *= accelGain;
            magError[i] *= magGain;
        }
        LimitVector(accelError, settings::sensors::icm20948::kAccelCorrectionMaxRateRadPerSec);
        LimitVector(magError, settings::sensors::icm20948::kMagCorrectionMaxRateRadPerSec);

        feedback[0] = accelError[0] + magError[0];
        feedback[1] = accelError[1] + magError[1];
        feedback[2] = accelError[2] + magError[2];
    }

    if (g_railConstraintActive && g_hasRailReferenceQuaternion) {
        float conjugate[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        float errorQuat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        QuaternionConjugate(g_q, conjugate);
        QuaternionMultiply(g_railReferenceQuaternion, conjugate, errorQuat);
        if (errorQuat[0] < 0.0f) {
            NegateQuaternion(errorQuat);
        }
        float railFeedback[3] = {
            2.0f * settings::sensors::icm20948::kRailConstraintGain * errorQuat[1],
            2.0f * settings::sensors::icm20948::kRailConstraintGain * errorQuat[2],
            2.0f * settings::sensors::icm20948::kRailConstraintGain * errorQuat[3],
        };
        LimitVector(railFeedback, settings::sensors::icm20948::kTotalCorrectionMaxRateRadPerSec);
        for (int i = 0; i < 3; ++i) {
            feedback[i] += railFeedback[i];
        }
    }

    LimitVector(feedback, settings::sensors::icm20948::kTotalCorrectionMaxRateRadPerSec);

    // Remove the learned gyro bias estimate from the measured rate. Adding it
    // makes the stationary bias learner drive the observer away from the
    // accel+mag bootstrap instead of damping residual drift.
    float correctedGyro[3] = {
        gyroRadPerSec[0] - g_gyroBias[0] + feedback[0],
        gyroRadPerSec[1] - g_gyroBias[1] + feedback[1],
        gyroRadPerSec[2] - g_gyroBias[2] + feedback[2],
    };

    // Keep the ICM rail on the same update convention as the working LSM path.
    // The separate exponential-map branch was walking away from the stable
    // accel+mag bootstrap while stationary.
    const float halfDt = 0.5f * dt;
    const float scaledGx = correctedGyro[0] * halfDt;
    const float scaledGy = correctedGyro[1] * halfDt;
    const float scaledGz = correctedGyro[2] * halfDt;

    const float q0 = g_q[0];
    const float q1 = g_q[1];
    const float q2 = g_q[2];
    const float q3 = g_q[3];

    float nq0 = q0 + (-q1 * scaledGx - q2 * scaledGy - q3 * scaledGz);
    float nq1 = q1 + (q0 * scaledGx + q2 * scaledGz - q3 * scaledGy);
    float nq2 = q2 + (q0 * scaledGy - q1 * scaledGz + q3 * scaledGx);
    float nq3 = q3 + (q0 * scaledGz + q1 * scaledGy - q2 * scaledGx);

    const float norm = sqrtf(nq0 * nq0 + nq1 * nq1 + nq2 * nq2 + nq3 * nq3);
    if (norm <= 1.0e-9f) {
        return;
    }
    const float inv = 1.0f / norm;
    g_q[0] = nq0 * inv;
    g_q[1] = nq1 * inv;
    g_q[2] = nq2 * inv;
    g_q[3] = nq3 * inv;
    if (!math_utils::ValidateQuaternionArray(g_q)) {
        g_hasQuaternionContinuityReference = false;
        return;
    }
    ApplyQuaternionContinuity();
}

/// Converts the current quaternion into yaw/pitch/roll degrees.
void QuaternionToYprDeg(float &yawDeg, float &pitchDeg, float &rollDeg) {
    const float q0 = g_q[0];
    const float q1 = g_q[1];
    const float q2 = g_q[2];
    const float q3 = g_q[3];

    float roll = atan2f((q0 * q1 + q2 * q3), 0.5f - (q1 * q1 + q2 * q2));
    float pitch = asinf(ClampUnit(2.0f * (q0 * q2 - q1 * q3)));
    float yaw = atan2f((q1 * q2 + q0 * q3), 0.5f - (q2 * q2 + q3 * q3));

    yaw *= kRadToDeg;
    pitch *= kRadToDeg;
    roll *= kRadToDeg;

    yaw = -(yaw + settings::sensors::icm20948::kMagDeclinationDeg);
    while (yaw < 0.0f) {
        yaw += 360.0f;
    }
    while (yaw >= 360.0f) {
        yaw -= 360.0f;
    }

    yawDeg = yaw;
    pitchDeg = pitch;
    // Use same roll sign convention as LSM (no negation).
    rollDeg = roll;
}

bool ExtractDmpQuaternion(const icm_20948_DMP_data_t &data, float quaternion[4]) {
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;
    bool hasQuaternion = false;

    if (settings::sensors::icm20948::kUseDmpQuat9 && (data.header & DMP_header_bitmap_Quat9) > 0) {
        q1 = static_cast<float>(data.Quat9.Data.Q1) / kDmpQuaternionScale;
        q2 = static_cast<float>(data.Quat9.Data.Q2) / kDmpQuaternionScale;
        q3 = static_cast<float>(data.Quat9.Data.Q3) / kDmpQuaternionScale;
        hasQuaternion = true;
    } else if ((data.header & DMP_header_bitmap_Quat6) > 0) {
        q1 = static_cast<float>(data.Quat6.Data.Q1) / kDmpQuaternionScale;
        q2 = static_cast<float>(data.Quat6.Data.Q2) / kDmpQuaternionScale;
        q3 = static_cast<float>(data.Quat6.Data.Q3) / kDmpQuaternionScale;
        hasQuaternion = true;
    } else if ((data.header & DMP_header_bitmap_Quat9) > 0) {
        q1 = static_cast<float>(data.Quat9.Data.Q1) / kDmpQuaternionScale;
        q2 = static_cast<float>(data.Quat9.Data.Q2) / kDmpQuaternionScale;
        q3 = static_cast<float>(data.Quat9.Data.Q3) / kDmpQuaternionScale;
        hasQuaternion = true;
    }

    if (!hasQuaternion) {
        return false;
    }

    const float q0Squared = 1.0f - (q1 * q1 + q2 * q2 + q3 * q3);
    if (q0Squared < -1.0e-3f) {
        return false;
    }
    const float q0 = sqrtf(q0Squared > 0.0f ? q0Squared : 0.0f);
    const math_utils::Quaternion sensorQuat =
        math_utils::Normalize(math_utils::MakeQuaternion(q0, q1, q2, q3));

    float upSensor[3] = {
        2.0f * (sensorQuat.x * sensorQuat.z + sensorQuat.w * sensorQuat.y),
        2.0f * (sensorQuat.y * sensorQuat.z - sensorQuat.w * sensorQuat.x),
        1.0f - 2.0f * (sensorQuat.x * sensorQuat.x + sensorQuat.y * sensorQuat.y),
    };
    float upBody[3] = {0.0f, 0.0f, 0.0f};
    ApplyMountRotation(upSensor, upBody);
    if (!Normalize3(upBody[0], upBody[1], upBody[2])) {
        return false;
    }

    return QuaternionFromUpVector(upBody, quaternion);
}

DmpReadResult TryReadDmpQuaternion(float quaternion[4]) {
    if (!g_dmpQuaternionActive) {
        return DmpReadResult::Failure;
    }

    bool foundQuaternion = false;
    for (uint8_t frame = 0; frame < kDmpMaxDrainFrames; ++frame) {
        icm_20948_DMP_data_t data = {};
        g_icm.readDMPdataFromFIFO(&data);
        const ICM_20948_Status_e status = g_icm.status;
        if (status == ICM_20948_Stat_FIFONoDataAvail || status == ICM_20948_Stat_NoData) {
            break;
        }
        if (status != ICM_20948_Stat_Ok && status != ICM_20948_Stat_FIFOMoreDataAvail) {
            return DmpReadResult::Failure;
        }
        if (ExtractDmpQuaternion(data, quaternion)) {
            foundQuaternion = true;
        }
        if (status != ICM_20948_Stat_FIFOMoreDataAvail) {
            break;
        }
    }
    return foundQuaternion ? DmpReadResult::QuaternionRead : DmpReadResult::NoData;
}

/// Publishes the last valid ICM sample so the estimator can hold state between
/// hardware updates instead of seeing a missing IMU sample.
bool PopulateFromCache(SensorData &out, uint32_t nowUs) {
    if (!g_hasCachedSample) {
        return false;
    }
    g_lastAcquireFresh = false;
    g_lastAcquireUsedCache = true;

    if (out.timestamp == 0.0f) {
        out.timestamp = static_cast<float>(nowUs) * 1.0e-6f;
    }

    out.accelICM[0] = g_lastAccel[0];
    out.accelICM[1] = g_lastAccel[1];
    out.accelICM[2] = g_lastAccel[2];
    out.gyro[0] = g_lastGyro[0];
    out.gyro[1] = g_lastGyro[1];
    out.gyro[2] = g_lastGyro[2];
    const bool quaternionValid = g_lastQuaternionOutputValid;
    out.icmQuaternion[0] = g_lastQuaternionOutput[0];
    out.icmQuaternion[1] = g_lastQuaternionOutput[1];
    out.icmQuaternion[2] = g_lastQuaternionOutput[2];
    out.icmQuaternion[3] = g_lastQuaternionOutput[3];
    out.icmYprDeg[0] = g_lastIcmYprDeg[0];
    out.icmYprDeg[1] = g_lastIcmYprDeg[1];
    out.icmYprDeg[2] = g_lastIcmYprDeg[2];
    out.icmTemperatureC = g_lastTemperatureC;
    out.icmAhrsDt = g_lastAhrsDt;
    out.icmAccelTrust = g_lastAccelTrust;
    out.icmMagTrust = g_lastMagTrust;
    out.icmGyroBias[0] = g_gyroBias[0];
    out.icmGyroBias[1] = g_gyroBias[1];
    out.icmGyroBias[2] = g_gyroBias[2];
    out.icmAccelSaturated = g_lastAccelSaturated;
    out.icmGyroSaturated = g_lastGyroSaturated;
    out.icmRailConstrained = g_railConstraintActive;
    out.hasIcmQuaternion = g_groundAlignmentReady && quaternionValid;
    out.hasIcmYpr = out.hasIcmQuaternion;
    return true;
}

void ConfigureSensorScales() {
    g_activeAccelLsbPerG = AccelLsbPerGForRange(settings::sensors::icm20948::kAccelRangeG);
    g_activeGyroLsbPerDps = GyroLsbPerDpsForRange(settings::sensors::icm20948::kGyroRangeDps);
    g_activeGyroRadPerSecPerLsb = GyroRadPerSecPerLsbForRange(settings::sensors::icm20948::kGyroRangeDps);
    g_calibrationAccelLsbPerG = AccelLsbPerGForRange(settings::sensors::icm20948::kCalibrationAccelRangeG);
    g_calibrationGyroLsbPerDps = GyroLsbPerDpsForRange(settings::sensors::icm20948::kCalibrationGyroRangeDps);
    g_accelSaturationCounts = settings::sensors::icm20948::kAccelSaturationFraction *
                              (g_activeAccelLsbPerG * settings::sensors::icm20948::kAccelRangeG);
    g_gyroSaturationCounts = settings::sensors::icm20948::kGyroSaturationFraction *
                             (g_activeGyroLsbPerDps * settings::sensors::icm20948::kGyroRangeDps);
}

bool ConfigureSampleMode() {
    if (g_icm.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous) !=
        ICM_20948_Stat_Ok) {
        LOG_PRINTLN("ICM-20948: setSampleMode failed");
        return false;
    }
    return true;
}

bool ConfigureFullScale() {
    ICM_20948_fss_t fullScale = {};
    fullScale.a = AccelFullScaleEnum(settings::sensors::icm20948::kAccelRangeG);
    fullScale.g = GyroFullScaleEnum(settings::sensors::icm20948::kGyroRangeDps);
    if (g_icm.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fullScale) != ICM_20948_Stat_Ok) {
        LOG_PRINTLN("ICM-20948: setFullScale failed");
        return false;
    }
    return true;
}

bool ConfigureLowPassFilter() {
    if (!settings::sensors::icm20948::kEnableDlpFilter) {
        return true;
    }
    ICM_20948_dlpcfg_t filterConfig = {};
    filterConfig.a = settings::sensors::icm20948::kAccelDlpFilterSetting;
    filterConfig.g = settings::sensors::icm20948::kGyroDlpFilterSetting;
    if (g_icm.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), filterConfig) != ICM_20948_Stat_Ok) {
        LOG_PRINTLN("ICM-20948: setDLPFcfg failed");
        return false;
    }
    if (g_icm.enableDLPF((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), true) != ICM_20948_Stat_Ok) {
        LOG_PRINTLN("ICM-20948: enableDLPF failed");
        return false;
    }
    return true;
}

bool ConfigureSampleRate() {
    ICM_20948_smplrt_t sampleRate = {};
    sampleRate.a = kAccelSampleRateDivider;
    sampleRate.g = kGyroSampleRateDivider;
    if (g_icm.setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), sampleRate) != ICM_20948_Stat_Ok) {
        LOG_PRINTLN("ICM-20948: setSampleRate failed");
        return false;
    }
    return true;
}

bool ConfigureDmpQuaternion() {
    g_dmpQuaternionActive = false;
    if (!settings::sensors::icm20948::kUseDmpQuaternion) {
        return true;
    }

    bool success = true;
    success &= (g_icm.initializeDMP() == ICM_20948_Stat_Ok);
    if (settings::sensors::icm20948::kUseDmpQuat9) {
        success &= (g_icm.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
        success &= (g_icm.setDMPODRrate(DMP_ODR_Reg_Quat9, settings::sensors::icm20948::kDmpQuatOdrInterval) ==
                    ICM_20948_Stat_Ok);
    } else {
        success &= (g_icm.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
        success &= (g_icm.setDMPODRrate(DMP_ODR_Reg_Quat6, settings::sensors::icm20948::kDmpQuatOdrInterval) ==
                    ICM_20948_Stat_Ok);
    }
    success &= (g_icm.enableFIFO() == ICM_20948_Stat_Ok);
    success &= (g_icm.enableDMP() == ICM_20948_Stat_Ok);
    success &= (g_icm.resetDMP() == ICM_20948_Stat_Ok);
    success &= (g_icm.resetFIFO() == ICM_20948_Stat_Ok);

    if (!success) {
        LOG_PRINTLN("ICM-20948: DMP quaternion setup failed; falling back to software fusion");
        return false;
    }

    LOG_PRINTLN(settings::sensors::icm20948::kUseDmpQuat9 ? "ICM-20948: using DMP Quat9"
                                                          : "ICM-20948: using DMP Quat6");
    g_dmpQuaternionActive = true;
    return true;
}

bool ConfigureInterrupt() {
    if (kInterruptPin < 0) {
        return true;
    }

    pinMode(kInterruptPin, INPUT_PULLUP);

    if (g_icm.cfgIntActiveLow(true) != ICM_20948_Stat_Ok) {
        LOG_PRINTLN("ICM-20948: cfgIntActiveLow failed");
        return false;
    }
    if (g_icm.cfgIntOpenDrain(false) != ICM_20948_Stat_Ok) {
        LOG_PRINTLN("ICM-20948: cfgIntOpenDrain failed");
        return false;
    }
    if (g_icm.cfgIntLatch(false) != ICM_20948_Stat_Ok) {
        LOG_PRINTLN("ICM-20948: cfgIntLatch failed");
        return false;
    }
    if (g_icm.cfgIntAnyReadToClear(true) != ICM_20948_Stat_Ok) {
        LOG_PRINTLN("ICM-20948: cfgIntAnyReadToClear failed");
        return false;
    }
    if (g_icm.clearInterrupts() != ICM_20948_Stat_Ok) {
        LOG_PRINTLN("ICM-20948: clearInterrupts failed");
        return false;
    }
    if (g_dmpQuaternionActive) {
        if (g_icm.intEnableDMP(true) != ICM_20948_Stat_Ok) {
            LOG_PRINTLN("ICM-20948: intEnableDMP failed");
            return false;
        }
    } else {
        if (g_icm.intEnableRawDataReady(true) != ICM_20948_Stat_Ok) {
            LOG_PRINTLN("ICM-20948: intEnableRawDataReady failed");
            return false;
        }
    }

    attachInterrupt(digitalPinToInterrupt(kInterruptPin), DataReadyISR, FALLING);
    g_interruptConfigured = true;
    return true;
}

void ResetAcquireFailureCounters() {
    g_consecutiveReadFailures = 0;
    g_consecutiveDmpFailures = 0;
}

bool ResetDmpFifoState() {
    bool success = true;
    if (g_icm.resetFIFO() != ICM_20948_Stat_Ok) {
        success = false;
    }
    if (g_dmpQuaternionActive && g_icm.resetDMP() != ICM_20948_Stat_Ok) {
        success = false;
    }
    return success;
}

bool ReconfigureSensorTransport() {
    g_dataReadyInterrupt = false;
    g_interruptConfigured = false;
    g_dmpQuaternionActive = false;
    SPI1.begin();
    g_icm.begin(kChipSelectPin, SPI1);
    if (g_icm.status != ICM_20948_Stat_Ok) {
        LOG_PRINTLN("ICM-20948: reinitialize begin failed");
        return false;
    }
    ConfigureSensorScales();
    if (!ConfigureSampleMode()) {
        return false;
    }
    if (!ConfigureFullScale()) {
        return false;
    }
    if (!ConfigureLowPassFilter()) {
        return false;
    }
    if (!ConfigureSampleRate()) {
        return false;
    }
    if (!ConfigureDmpQuaternion()) {
        g_dmpQuaternionActive = false;
    }
    if (!ConfigureInterrupt()) {
        LOG_PRINTLN("ICM-20948: interrupt setup failed; continuing in polling mode");
    }
    g_lastSampleUs = 0;
    return true;
}

bool HandleAcquireFailure(SensorData &out, uint32_t nowUs, const char *reason, bool dmpFailure) {
    if (g_consecutiveReadFailures < 0xff) {
        ++g_consecutiveReadFailures;
    }
    if (dmpFailure && g_consecutiveDmpFailures < 0xff) {
        ++g_consecutiveDmpFailures;
    }

    if (dmpFailure &&
        g_dmpQuaternionActive &&
        g_consecutiveDmpFailures >= kMaxConsecutiveDmpFailures) {
        if (ResetDmpFifoState()) {
            LOG_PRINT("ICM-20948: reset FIFO after ");
            LOG_PRINTLN(reason);
            g_consecutiveDmpFailures = 0;
        } else {
            LOG_PRINT("ICM-20948: FIFO reset failed after ");
            LOG_PRINTLN(reason);
        }
    }

    if (g_consecutiveReadFailures >= kMaxConsecutiveReadFailures) {
        LOG_PRINT("ICM-20948: reinitializing after ");
        LOG_PRINTLN(reason);
        if (ReconfigureSensorTransport()) {
            ResetAcquireFailureCounters();
        }
    }

    return PopulateFromCache(out, nowUs);
}

}  // namespace

#if defined(ICM_20948_USE_DMP)
ICM_20948_Status_e ICM_20948::initializeDMP(void) {
    if (_device._dmp_firmware_available != true) {
        debugPrint(F("ICM_20948::startupDMP: DMP is not available. Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
        return ICM_20948_Stat_DMPNotSupported;
    }

    const uint8_t accelRangeG = settings::sensors::icm20948::kAccelRangeG;
    const uint16_t gyroRangeDps = settings::sensors::icm20948::kGyroRangeDps;
    const ICM_20948_ACCEL_CONFIG_FS_SEL_e accelFullScale = AccelFullScaleEnum(accelRangeG);
    const ICM_20948_GYRO_CONFIG_1_FS_SEL_e gyroFullScale = GyroFullScaleEnum(gyroRangeDps);
    const uint8_t gyroLevel = DmpGyroLevel(gyroRangeDps);

    ICM_20948_Status_e result = ICM_20948_Stat_Ok;
    ICM_20948_Status_e worstResult = ICM_20948_Stat_Ok;

    result = i2cControllerConfigurePeripheral(0, MAG_AK09916_I2C_ADDR, AK09916_REG_RSV2, 10, true, true, false, true,
                                              true);
    if (result > worstResult) {
        worstResult = result;
    }
    result = i2cControllerConfigurePeripheral(1, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL2, 1, false, true, false, false,
                                              false, AK09916_mode_single);
    if (result > worstResult) {
        worstResult = result;
    }

    result = setBank(3);
    if (result > worstResult) {
        worstResult = result;
    }
    uint8_t mstODRconfig = 0x04;
    result = write(AGB3_REG_I2C_MST_ODR_CONFIG, &mstODRconfig, 1);
    if (result > worstResult) {
        worstResult = result;
    }

    result = setClockSource(ICM_20948_Clock_Auto);
    if (result > worstResult) {
        worstResult = result;
    }

    result = setBank(0);
    if (result > worstResult) {
        worstResult = result;
    }
    uint8_t pwrMgmt2 = 0x40;
    result = write(AGB0_REG_PWR_MGMT_2, &pwrMgmt2, 1);
    if (result > worstResult) {
        worstResult = result;
    }

    result = setSampleMode(ICM_20948_Internal_Mst, ICM_20948_Sample_Mode_Cycled);
    if (result > worstResult) {
        worstResult = result;
    }
    result = enableFIFO(false);
    if (result > worstResult) {
        worstResult = result;
    }
    result = enableDMP(false);
    if (result > worstResult) {
        worstResult = result;
    }

    ICM_20948_fss_t fullScaleSettings = {};
    fullScaleSettings.a = accelFullScale;
    fullScaleSettings.g = gyroFullScale;
    result = setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fullScaleSettings);
    if (result > worstResult) {
        worstResult = result;
    }
    result = enableDLPF(ICM_20948_Internal_Gyr, true);
    if (result > worstResult) {
        worstResult = result;
    }

    result = setBank(0);
    if (result > worstResult) {
        worstResult = result;
    }
    uint8_t zero = 0;
    result = write(AGB0_REG_FIFO_EN_1, &zero, 1);
    if (result > worstResult) {
        worstResult = result;
    }
    result = write(AGB0_REG_FIFO_EN_2, &zero, 1);
    if (result > worstResult) {
        worstResult = result;
    }
    result = intEnableRawDataReady(false);
    if (result > worstResult) {
        worstResult = result;
    }
    result = resetFIFO();
    if (result > worstResult) {
        worstResult = result;
    }

    ICM_20948_smplrt_t sampleRate = {};
    sampleRate.g = kDmpFastSampleRateDivider;
    sampleRate.a = kDmpFastSampleRateDivider;
    result = setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), sampleRate);
    if (result > worstResult) {
        worstResult = result;
    }

    result = setDMPstartAddress();
    if (result > worstResult) {
        worstResult = result;
    }
    result = loadDMPFirmware();
    if (result > worstResult) {
        worstResult = result;
    }
    result = setDMPstartAddress();
    if (result > worstResult) {
        worstResult = result;
    }

    result = setBank(0);
    if (result > worstResult) {
        worstResult = result;
    }
    uint8_t fix = 0x48;
    result = write(AGB0_REG_HW_FIX_DISABLE, &fix, 1);
    if (result > worstResult) {
        worstResult = result;
    }
    uint8_t fifoPrio = 0xE4;
    result = write(AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, &fifoPrio, 1);
    if (result > worstResult) {
        worstResult = result;
    }

    unsigned char accScale[4];
    EncodeBigEndianU32(DmpAccelScaleValue(accelRangeG), accScale);
    result = writeDMPmems(ACC_SCALE, 4, &accScale[0]);
    if (result > worstResult) {
        worstResult = result;
    }
    unsigned char accScale2[4];
    EncodeBigEndianU32(DmpAccelScale2Value(accelRangeG), accScale2);
    result = writeDMPmems(ACC_SCALE2, 4, &accScale2[0]);
    if (result > worstResult) {
        worstResult = result;
    }

    const unsigned char mountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
    const unsigned char mountMultiplierPlus[4] = {0x09, 0x99, 0x99, 0x99};
    const unsigned char mountMultiplierMinus[4] = {0xF6, 0x66, 0x66, 0x67};
    result = writeDMPmems(CPASS_MTX_00, 4, &mountMultiplierPlus[0]);
    if (result > worstResult) {
        worstResult = result;
    }
    result = writeDMPmems(CPASS_MTX_01, 4, &mountMultiplierZero[0]);
    if (result > worstResult) {
        worstResult = result;
    }
    result = writeDMPmems(CPASS_MTX_02, 4, &mountMultiplierZero[0]);
    if (result > worstResult) {
        worstResult = result;
    }
    result = writeDMPmems(CPASS_MTX_10, 4, &mountMultiplierZero[0]);
    if (result > worstResult) {
        worstResult = result;
    }
    result = writeDMPmems(CPASS_MTX_11, 4, &mountMultiplierMinus[0]);
    if (result > worstResult) {
        worstResult = result;
    }
    result = writeDMPmems(CPASS_MTX_12, 4, &mountMultiplierZero[0]);
    if (result > worstResult) {
        worstResult = result;
    }
    result = writeDMPmems(CPASS_MTX_20, 4, &mountMultiplierZero[0]);
    if (result > worstResult) {
        worstResult = result;
    }
    result = writeDMPmems(CPASS_MTX_21, 4, &mountMultiplierZero[0]);
    if (result > worstResult) {
        worstResult = result;
    }
    result = writeDMPmems(CPASS_MTX_22, 4, &mountMultiplierMinus[0]);
    if (result > worstResult) {
        worstResult = result;
    }

    const unsigned char b2sMountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
    const unsigned char b2sMountMultiplierPlus[4] = {0x40, 0x00, 0x00, 0x00};
    result = writeDMPmems(B2S_MTX_00, 4, &b2sMountMultiplierPlus[0]);
    if (result > worstResult) {
        worstResult = result;
    }
    result = writeDMPmems(B2S_MTX_01, 4, &b2sMountMultiplierZero[0]);
    if (result > worstResult) {
        worstResult = result;
    }
    result = writeDMPmems(B2S_MTX_02, 4, &b2sMountMultiplierZero[0]);
    if (result > worstResult) {
        worstResult = result;
    }
    result = writeDMPmems(B2S_MTX_10, 4, &b2sMountMultiplierZero[0]);
    if (result > worstResult) {
        worstResult = result;
    }
    result = writeDMPmems(B2S_MTX_11, 4, &b2sMountMultiplierPlus[0]);
    if (result > worstResult) {
        worstResult = result;
    }
    result = writeDMPmems(B2S_MTX_12, 4, &b2sMountMultiplierZero[0]);
    if (result > worstResult) {
        worstResult = result;
    }
    result = writeDMPmems(B2S_MTX_20, 4, &b2sMountMultiplierZero[0]);
    if (result > worstResult) {
        worstResult = result;
    }
    result = writeDMPmems(B2S_MTX_21, 4, &b2sMountMultiplierZero[0]);
    if (result > worstResult) {
        worstResult = result;
    }
    result = writeDMPmems(B2S_MTX_22, 4, &b2sMountMultiplierPlus[0]);
    if (result > worstResult) {
        worstResult = result;
    }

    result = setGyroSF(kDmpFastSampleRateDivider, gyroLevel);
    if (result > worstResult) {
        worstResult = result;
    }
    unsigned char gyroFullScaleValue[4];
    EncodeBigEndianU32(DmpGyroFullScaleValue(gyroRangeDps), gyroFullScaleValue);
    result = writeDMPmems(GYRO_FULLSCALE, 4, &gyroFullScaleValue[0]);
    if (result > worstResult) {
        worstResult = result;
    }

    const unsigned char accelOnlyGain[4] = {0x00, 0xE8, 0xBA, 0x2E};
    result = writeDMPmems(ACCEL_ONLY_GAIN, 4, &accelOnlyGain[0]);
    if (result > worstResult) {
        worstResult = result;
    }
    const unsigned char accelAlphaVar[4] = {0x3D, 0x27, 0xD2, 0x7D};
    result = writeDMPmems(ACCEL_ALPHA_VAR, 4, &accelAlphaVar[0]);
    if (result > worstResult) {
        worstResult = result;
    }
    const unsigned char accelAVar[4] = {0x02, 0xD8, 0x2D, 0x83};
    result = writeDMPmems(ACCEL_A_VAR, 4, &accelAVar[0]);
    if (result > worstResult) {
        worstResult = result;
    }

    const unsigned char accelCalRate[2] = {0x00, 0x00};
    result = writeDMPmems(ACCEL_CAL_RATE, 2, &accelCalRate[0]);
    if (result > worstResult) {
        worstResult = result;
    }
    const unsigned char compassRate[2] = {0x00, 0x45};
    result = writeDMPmems(CPASS_TIME_BUFFER, 2, &compassRate[0]);
    if (result > worstResult) {
        worstResult = result;
    }

    return worstResult;
}
#endif

/// Initializes the ICM-20948 over SPI.
bool Icm20948SensorBegin() {

    if (g_initialized) {
        return true;
    }

    if (!ReconfigureSensorTransport()) {
        return false;
    }

    g_lastSampleUs = 0;
    g_lastFilterUs = 0;
    g_hasCachedSample = false;
    g_hasMagReference = false;
    g_magReferenceNorm = 0.0f;
    g_hasEarthMagReference = false;
    g_magReferenceEarth[0] = 0.0f;
    g_magReferenceEarth[1] = 1.0f;
    g_magReferenceEarth[2] = 0.0f;
    g_q[0] = 1.0f;
    g_q[1] = 0.0f;
    g_q[2] = 0.0f;
    g_q[3] = 0.0f;
    g_lastQuaternionOutputValid = false;
    g_lastBootstrapYprValid = false;
    g_invalidQuaternionStreak = 0;
    g_lastQuaternionOutput[0] = 1.0f;
    g_lastQuaternionOutput[1] = 0.0f;
    g_lastQuaternionOutput[2] = 0.0f;
    g_lastQuaternionOutput[3] = 0.0f;
    g_hasQuaternionContinuityReference = false;
    g_lastContinuousQuaternion[0] = 1.0f;
    g_lastContinuousQuaternion[1] = 0.0f;
    g_lastContinuousQuaternion[2] = 0.0f;
    g_lastContinuousQuaternion[3] = 0.0f;
    g_hasRailReferenceQuaternion = false;
    g_railReferenceQuaternion[0] = 1.0f;
    g_railReferenceQuaternion[1] = 0.0f;
    g_railReferenceQuaternion[2] = 0.0f;
    g_railReferenceQuaternion[3] = 0.0f;
    g_railConstraintActive = false;
    g_burnStartUs = 0;
    ResetGroundAlignment();
    g_lastAppliedFlightStatus = FlightStatus::Ground;
    g_gyroBias[0] = 0.0f;
    g_gyroBias[1] = 0.0f;
    g_gyroBias[2] = 0.0f;
    g_lastAccel[0] = 0.0f;
    g_lastAccel[1] = 0.0f;
    g_lastAccel[2] = 0.0f;
    g_lastAccelPreMountG[0] = 0.0f;
    g_lastAccelPreMountG[1] = 0.0f;
    g_lastAccelPreMountG[2] = 0.0f;
    g_lastMagPreAxis[0] = 0.0f;
    g_lastMagPreAxis[1] = 0.0f;
    g_lastMagPreAxis[2] = 0.0f;
    g_lastMagBody[0] = 0.0f;
    g_lastMagBody[1] = 0.0f;
    g_lastMagBody[2] = 0.0f;
    g_lastGyro[0] = 0.0f;
    g_lastGyro[1] = 0.0f;
    g_lastGyro[2] = 0.0f;
    g_lastIcmYprDeg[0] = 0.0f;
    g_lastIcmYprDeg[1] = 0.0f;
    g_lastIcmYprDeg[2] = 0.0f;
    g_lastTemperatureC = 0.0f;
    g_lastAhrsDt = 0.0f;
    g_lastAccelTrust = 0.0f;
    g_lastMagTrust = 0.0f;
    g_lastAccelSaturated = false;
    g_lastGyroSaturated = false;
    g_lastRailConstrained = false;
    g_lastAcquireFresh = false;
    g_lastAcquireUsedCache = false;
    g_lastAcquireUsedInterrupt = false;
    g_crossCheckTrust = 1.0f;
    ResetAcquireFailureCounters();
    g_initialized = true;
    return true;
}

bool Icm20948SensorIsInitialized() {
    return g_initialized;
}

Icm20948Diagnostics Icm20948SensorGetDiagnostics() {
    Icm20948Diagnostics diagnostics;
    diagnostics.initialized = g_initialized;
    diagnostics.hasQuaternion = g_groundAlignmentReady && g_lastQuaternionOutputValid;
    diagnostics.alignmentReady = g_groundAlignmentReady;
    diagnostics.lastAcquireFresh = g_lastAcquireFresh;
    diagnostics.lastAcquireUsedCache = g_lastAcquireUsedCache;
    diagnostics.interruptConfigured = g_interruptConfigured;
    diagnostics.lastAcquireUsedInterrupt = g_lastAcquireUsedInterrupt;
    diagnostics.lastSampleMicros = g_lastSampleUs;
    diagnostics.hasBootstrapYpr = g_lastBootstrapYprValid;
    diagnostics.accelPreMountG[0] = g_lastAccelPreMountG[0];
    diagnostics.accelPreMountG[1] = g_lastAccelPreMountG[1];
    diagnostics.accelPreMountG[2] = g_lastAccelPreMountG[2];
    diagnostics.bootstrapYprDeg[0] = g_lastBootstrapYprDeg[0];
    diagnostics.bootstrapYprDeg[1] = g_lastBootstrapYprDeg[1];
    diagnostics.bootstrapYprDeg[2] = g_lastBootstrapYprDeg[2];
    diagnostics.accelBodyMps2[0] = g_lastAccel[0];
    diagnostics.accelBodyMps2[1] = g_lastAccel[1];
    diagnostics.accelBodyMps2[2] = g_lastAccel[2];
    diagnostics.magPreAxis[0] = g_lastMagPreAxis[0];
    diagnostics.magPreAxis[1] = g_lastMagPreAxis[1];
    diagnostics.magPreAxis[2] = g_lastMagPreAxis[2];
    diagnostics.magBody[0] = g_lastMagBody[0];
    diagnostics.magBody[1] = g_lastMagBody[1];
    diagnostics.magBody[2] = g_lastMagBody[2];
    if (diagnostics.hasQuaternion) {
        float yaw = 0.0f;
        float pitch = 0.0f;
        float roll = 0.0f;
        const math_utils::Quaternion quat = math_utils::MakeQuaternion(g_lastQuaternionOutput[0],
                                                                       g_lastQuaternionOutput[1],
                                                                       g_lastQuaternionOutput[2],
                                                                       g_lastQuaternionOutput[3]);
        math_utils::QuaternionToEuler(quat, yaw, pitch, roll);
        diagnostics.yprDeg[0] = yaw * 57.295779513082320876f;
        diagnostics.yprDeg[1] = pitch * 57.295779513082320876f;
        diagnostics.yprDeg[2] = roll * 57.295779513082320876f;
    }
    return diagnostics;
}

void Icm20948SensorSetFlightStatus(FlightStatus status) {
    g_flightStatus = status;
}

void Icm20948SensorSetBurnoutTimestamp(float burnoutTimeSeconds) {
    g_burnoutTimestampSeconds = burnoutTimeSeconds;
}

void Icm20948SensorSetCurrentTimestamp(float currentTimeSeconds) {
    g_currentTimestampSeconds = currentTimeSeconds;
}

void Icm20948SensorSetCrossCheckTrust(float trust) {
    g_crossCheckTrust = Clamp01(trust);
}

/// Acquires one ICM sample, updates the adaptive AHRS observer, and publishes calibrated outputs.
bool Icm20948SensorAcquire(SensorData &out) {
    if (!g_initialized) {
        return false;
    }
    g_lastAcquireFresh = false;
    g_lastAcquireUsedCache = false;
    g_lastAcquireUsedInterrupt = false;

    const uint32_t nowUs = micros();
    if (!g_dmpQuaternionActive) {
        UpdateRailConstraintState(nowUs);
    } else {
        g_railConstraintActive = false;
    }
    if (g_lastSampleUs != 0 && (nowUs - g_lastSampleUs) < kSampleIntervalUs) {
        return PopulateFromCache(out, nowUs);
    }

    bool interruptTriggered = false;
    noInterrupts();
    if (g_dataReadyInterrupt) {
        g_dataReadyInterrupt = false;
        interruptTriggered = true;
    }
    interrupts();

    float dmpQuaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    if (g_dmpQuaternionActive) {
        bool shouldRead = false;
        if (g_interruptConfigured) {
            shouldRead = interruptTriggered;
            if (!shouldRead && g_icm.dataReady()) {
                shouldRead = true;
            }
        } else {
            shouldRead = g_icm.dataReady();
        }

        if (!shouldRead) {
            return PopulateFromCache(out, nowUs);
        }

        const DmpReadResult dmpReadResult = TryReadDmpQuaternion(dmpQuaternion);
        if (dmpReadResult == DmpReadResult::NoData) {
            return PopulateFromCache(out, nowUs);
        }
        if (dmpReadResult == DmpReadResult::Failure) {
            return HandleAcquireFailure(out, nowUs, "DMP FIFO read failure", true);
        }
        g_lastSampleUs = nowUs;
        g_icm.getAGMT();
        if (g_icm.status != ICM_20948_Stat_Ok) {
            return HandleAcquireFailure(out, nowUs, "AGMT read failure (DMP)", true);
        }
        g_lastAcquireFresh = true;
        g_lastAcquireUsedInterrupt = interruptTriggered;
    } else {
        bool shouldRead = false;
        if (g_interruptConfigured) {
            shouldRead = interruptTriggered;
            if (!shouldRead && g_icm.dataReady()) {
                shouldRead = true;
            }
        } else {
            shouldRead = g_icm.dataReady();
        }

        if (!shouldRead) {
            return PopulateFromCache(out, nowUs);
        }

        g_lastSampleUs = nowUs;
        g_icm.getAGMT();
        if (g_icm.status != ICM_20948_Stat_Ok) {
            return HandleAcquireFailure(out, nowUs, "AGMT read failure", false);
        }
        g_lastAcquireFresh = true;
        g_lastAcquireUsedInterrupt = interruptTriggered;
    }
    ResetAcquireFailureCounters();

    if (out.timestamp == 0.0f) {
        out.timestamp = static_cast<float>(nowUs) * 1.0e-6f;
    }

    float gyroCal[3] = {0.0f, 0.0f, 0.0f};
    float accelCalG[3] = {0.0f, 0.0f, 0.0f};
    float accelCalNorm[3] = {0.0f, 0.0f, 0.0f};
    float magCalNorm[3] = {0.0f, 0.0f, 0.0f};
    float accelMagnitudeG = 0.0f;
    float magMagnitude = 0.0f;
    float temperatureC = 0.0f;
    bool accelSaturated = false;
    bool gyroSaturated = false;
    float gyroSaturationTrust = 1.0f;
    float accelSaturationTrust = 1.0f;
    GetScaledImu(
        gyroCal,
        accelCalG,
        accelCalNorm,
        magCalNorm,
        accelMagnitudeG,
        magMagnitude,
        temperatureC,
        accelSaturated,
        gyroSaturated,
        gyroSaturationTrust,
        accelSaturationTrust);

    const float accelX = accelCalG[0] * kGToMps2;
    const float accelY = accelCalG[1] * kGToMps2;
    const float accelZ = accelCalG[2] * kGToMps2;

    if (!accelSaturated) {
        out.accelICM[0] = accelX;
        out.accelICM[1] = accelY;
        out.accelICM[2] = accelZ;
        g_lastAccel[0] = accelX;
        g_lastAccel[1] = accelY;
        g_lastAccel[2] = accelZ;
    }
    if (!gyroSaturated) {
        out.gyro[0] = gyroCal[0];
        out.gyro[1] = gyroCal[1];
        out.gyro[2] = gyroCal[2];
        g_lastGyro[0] = gyroCal[0];
        g_lastGyro[1] = gyroCal[1];
        g_lastGyro[2] = gyroCal[2];
    }

    // Match the reference implementation's mag-axis reconciliation so replay
    // and offline analysis stay consistent with historical logs.
    const float gyroNorm = Magnitude3(gyroCal[0], gyroCal[1], gyroCal[2]);

    // Compute dt early so outlier detection can use it.
    float dt = settings::flight::kDefaultDtSeconds;
    if (g_lastFilterUs != 0) {
        dt = static_cast<float>(nowUs - g_lastFilterUs) * 1.0e-6f;
        if (dt <= 0.0f || dt > 0.2f) {
            dt = settings::flight::kDefaultDtSeconds;
        }
    }
    g_lastFilterUs = nowUs;

    // Outlier detection (Phase 3.1): check for implausible rate-of-change.
    float gyroOutlierTrust = 1.0f;
    float accelOutlierTrust = 1.0f;
    if (g_hasPreviousGyro) {
        gyroOutlierTrust = ComputeGyroOutlierTrust(gyroCal, g_previousGyro, dt);
        if (gyroOutlierTrust < 1.0f) {
            ++g_outlierGyroCount;
        }
    }
    if (g_hasPreviousAccel) {
        const float accelMps2[3] = {accelCalG[0] * kGToMps2, accelCalG[1] * kGToMps2, accelCalG[2] * kGToMps2};
        const float prevAccelMps2[3] = {g_previousAccel[0], g_previousAccel[1], g_previousAccel[2]};
        accelOutlierTrust = ComputeAccelOutlierTrust(accelMps2, prevAccelMps2, dt);
        if (accelOutlierTrust < 1.0f) {
            ++g_outlierAccelCount;
        }
    }
    // Store current values for next iteration.
    g_previousGyro[0] = gyroCal[0];
    g_previousGyro[1] = gyroCal[1];
    g_previousGyro[2] = gyroCal[2];
    g_hasPreviousGyro = true;
    g_previousAccel[0] = accelCalG[0] * kGToMps2;
    g_previousAccel[1] = accelCalG[1] * kGToMps2;
    g_previousAccel[2] = accelCalG[2] * kGToMps2;
    g_hasPreviousAccel = true;

    // Compute trust with soft saturation and outlier detection.
    // Soft saturation (Phase 1.2): multiply trust by saturation trust factor.
    const float effectiveAccelSaturationTrust = accelSaturated ? 0.0f : accelSaturationTrust;
    const float effectiveGyroSaturationTrust = gyroSaturated ? 0.0f : gyroSaturationTrust;
    const float localAccelTrust = ComputeAccelTrust(accelMagnitudeG, gyroNorm);
    const float localMagTrust = ComputeMagTrust(magMagnitude, gyroNorm);

    // Track accelerometer magnitude for gyro bias learning validation.
    g_lastAccelMagnitudeG = accelMagnitudeG;
    const float startupAccelTrust = localAccelTrust * effectiveAccelSaturationTrust * accelOutlierTrust;
    const float startupMagTrust = localMagTrust * effectiveGyroSaturationTrust * gyroOutlierTrust;
    const float accelTrust = startupAccelTrust * g_crossCheckTrust;
    const float magTrust = startupMagTrust * g_crossCheckTrust;
    if (g_dmpQuaternionActive) {
        for (int i = 0; i < 4; ++i) {
            g_q[i] = dmpQuaternion[i];
        }
        g_groundAlignmentReady = true;
        g_lastBootstrapYprValid = false;
        g_invalidQuaternionStreak = 0;
        ApplyQuaternionContinuity();
    } else {
        UpdateBootstrapYprDiagnostics(accelCalNorm, magCalNorm);
        UpdateMagMagnitudeReference(magMagnitude, magTrust);
        UpdateGroundAlignment(accelCalNorm, accelTrust, magCalNorm, magTrust, gyroNorm);

        if (g_groundAlignmentReady && !gyroSaturated) {
            // Re-enable pad-time magnetic correction now that the direct body-frame
            // mag diagnostics show the ICM field vector points in the same general
            // direction as the trusted LSM rail.
            AdaptiveQuaternionUpdate(accelCalNorm, accelTrust, gyroCal, magCalNorm, magTrust, dt);
            UpdateEarthMagReference(magCalNorm, accelTrust, magTrust);
            CaptureRailReferenceQuaternion(accelTrust, magTrust);
        }
        LearnGyroBias(gyroCal, accelTrust, gyroNorm, dt);
    }
    const bool quaternionValidNow = math_utils::ValidateQuaternionArray(g_q);
    if (quaternionValidNow) {
        g_invalidQuaternionStreak = 0;
        g_lastQuaternionOutputValid = true;
        for (int i = 0; i < 4; ++i) {
            g_lastQuaternionOutput[i] = g_q[i];
        }
        QuaternionToYprDeg(g_lastIcmYprDeg[0], g_lastIcmYprDeg[1], g_lastIcmYprDeg[2]);
    } else {
        g_hasQuaternionContinuityReference = false;
        if (g_lastQuaternionOutputValid && g_invalidQuaternionStreak < 0xff) {
            ++g_invalidQuaternionStreak;
        }
        if (!g_lastQuaternionOutputValid || g_invalidQuaternionStreak >= kQuaternionInvalidDropThreshold) {
            g_lastQuaternionOutputValid = false;
        }
    }
    out.icmQuaternion[0] = g_lastQuaternionOutput[0];
    out.icmQuaternion[1] = g_lastQuaternionOutput[1];
    out.icmQuaternion[2] = g_lastQuaternionOutput[2];
    out.icmQuaternion[3] = g_lastQuaternionOutput[3];
    out.hasIcmQuaternion = g_groundAlignmentReady && g_lastQuaternionOutputValid;

    out.icmYprDeg[0] = g_lastIcmYprDeg[0];
    out.icmYprDeg[1] = g_lastIcmYprDeg[1];
    out.icmYprDeg[2] = g_lastIcmYprDeg[2];
    out.hasIcmYpr = out.hasIcmQuaternion;
    out.icmTemperatureC = temperatureC;
    out.icmAhrsDt = dt;
    out.icmAccelTrust = accelTrust;
    out.icmMagTrust = magTrust;
    out.icmGyroBias[0] = g_gyroBias[0];
    out.icmGyroBias[1] = g_gyroBias[1];
    out.icmGyroBias[2] = g_gyroBias[2];
    out.icmAccelSaturated = accelSaturated;
    out.icmGyroSaturated = gyroSaturated;
    out.icmRailConstrained = g_railConstraintActive;
    g_lastTemperatureC = temperatureC;
    g_lastAhrsDt = dt;
    g_lastAccelTrust = accelTrust;
    g_lastMagTrust = magTrust;
    g_lastAccelSaturated = accelSaturated;
    g_lastGyroSaturated = gyroSaturated;
    g_lastRailConstrained = g_railConstraintActive;
    g_hasCachedSample = true;

    return true;
}
