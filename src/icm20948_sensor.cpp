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

ICM_20948_SPI g_icm;
volatile bool g_dataReadyInterrupt = false;
bool g_initialized = false;
bool g_hasCachedSample = false;
uint32_t g_lastSampleUs = 0;
uint32_t g_lastFilterUs = 0;

float g_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float g_gyroBias[3] = {0.0f, 0.0f, 0.0f};
float g_lastAccel[3] = {0.0f, 0.0f, 0.0f};
float g_lastGyro[3] = {0.0f, 0.0f, 0.0f};
float g_lastIcmYprDeg[3] = {0.0f, 0.0f, 0.0f};
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
bool g_groundAlignmentReady = false;
float g_groundAlignmentAccelSum[3] = {0.0f, 0.0f, 0.0f};
float g_groundAlignmentMagSum[3] = {0.0f, 0.0f, 0.0f};
uint16_t g_groundAlignmentSampleCount = 0;
bool g_lastAcquireFresh = false;
bool g_lastAcquireUsedCache = false;
bool g_lastAcquireUsedInterrupt = false;
bool g_interruptConfigured = false;
float g_crossCheckTrust = 1.0f;

// Outlier detection state (Phase 3.1).
bool g_hasPreviousGyro = false;
float g_previousGyro[3] = {0.0f, 0.0f, 0.0f};
bool g_hasPreviousAccel = false;
float g_previousAccel[3] = {0.0f, 0.0f, 0.0f};
uint32_t g_outlierGyroCount = 0;
uint32_t g_outlierAccelCount = 0;

void DataReadyISR() {
    g_dataReadyInterrupt = true;
}

bool Normalize3(float &x, float &y, float &z);
void Cross3(float ax, float ay, float az, float bx, float by, float bz, float out[3]);

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

bool InitializeQuaternionFromAccelMag(const float accelNorm[3], const float magNorm[3]) {
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

/// Rotates an Earth-frame vector into the body frame using the stored quaternion convention.
void RotateEarthToBody(const float q[4], const float earth[3], float body[3]) {
    const float w = q[0];
    const float x = q[1];
    const float y = q[2];
    const float z = q[3];

    const float r00 = 1.0f - 2.0f * (y * y + z * z);
    const float r01 = 2.0f * (x * y + w * z);
    const float r02 = 2.0f * (x * z - w * y);
    const float r10 = 2.0f * (x * y - w * z);
    const float r11 = 1.0f - 2.0f * (x * x + z * z);
    const float r12 = 2.0f * (y * z + w * x);
    const float r20 = 2.0f * (x * z + w * y);
    const float r21 = 2.0f * (y * z - w * x);
    const float r22 = 1.0f - 2.0f * (x * x + y * y);

    body[0] = r00 * earth[0] + r01 * earth[1] + r02 * earth[2];
    body[1] = r10 * earth[0] + r11 * earth[1] + r12 * earth[2];
    body[2] = r20 * earth[0] + r21 * earth[1] + r22 * earth[2];
}

/// Rotates a body-frame vector into the Earth frame.
void RotateBodyToEarth(const float q[4], const float body[3], float earth[3]) {
    const float w = q[0];
    const float x = q[1];
    const float y = q[2];
    const float z = q[3];

    const float r00 = 1.0f - 2.0f * (y * y + z * z);
    const float r01 = 2.0f * (x * y + w * z);
    const float r02 = 2.0f * (x * z - w * y);
    const float r10 = 2.0f * (x * y - w * z);
    const float r11 = 1.0f - 2.0f * (x * x + z * z);
    const float r12 = 2.0f * (y * z + w * x);
    const float r20 = 2.0f * (x * z + w * y);
    const float r21 = 2.0f * (y * z - w * x);
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
    ApplyMountRotation(magNorm);
    magMagnitude = sqrtf(magNorm[0] * magNorm[0] + magNorm[1] * magNorm[1] + magNorm[2] * magNorm[2]);
    Normalize3(magNorm[0], magNorm[1], magNorm[2]);
}

/// Computes trust in the accelerometer gravity proxy for the current phase.
float ComputeAccelTrust(float accelMagnitudeG, float gyroNorm) {
    float phaseTrust = 0.0f;
    switch (g_flightStatus) {
        case FlightStatus::Ground:
            phaseTrust = 1.0f;
            break;
        case FlightStatus::Descent:
            phaseTrust = 0.75f;
            break;
        case FlightStatus::Burn:
        case FlightStatus::Coast:
        case FlightStatus::Overshoot:
            return 0.0f;
    }

    const float magnitudeTrust = WindowTrust(accelMagnitudeG,
                                             settings::sensors::icm20948::kAccelCorrectionMinG,
                                             settings::sensors::icm20948::kAccelCorrectionMaxG);
    const float rateTrust = DescendingTrust(gyroNorm,
                                            settings::sensors::icm20948::kAccelCorrectionGyroFadeStartRadPerSec,
                                            settings::sensors::icm20948::kAccelCorrectionGyroFadeEndRadPerSec);
    return g_crossCheckTrust * phaseTrust * magnitudeTrust * rateTrust;
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
        return g_crossCheckTrust * phaseTrust * DescendingTrust(gyroNorm,
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
    return g_crossCheckTrust * phaseTrust * magnitudeTrust * rateTrust;
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

/// Computes the legacy accel+mag yaw correction used to bootstrap the Earth-field reference.
bool ComputeBootstrapMagError(const float accelNorm[3], const float magNorm[3], float error[3]) {
    const float ax = accelNorm[0];
    const float ay = accelNorm[1];
    const float az = accelNorm[2];
    const float mx = magNorm[0];
    const float my = magNorm[1];
    const float mz = magNorm[2];

    float hx = ay * mz - az * my;
    float hy = az * mx - ax * mz;
    float hz = ax * my - ay * mx;
    if (!Normalize3(hx, hy, hz)) {
        return false;
    }

    const float q0 = g_q[0];
    const float q1 = g_q[1];
    const float q2 = g_q[2];
    const float q3 = g_q[3];

    const float wx = 2.0f * (q1 * q2 + q0 * q3);
    const float wy = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;
    const float wz = 2.0f * (q2 * q3 - q0 * q1);

    error[0] = hy * wz - hz * wy;
    error[1] = hz * wx - hx * wz;
    error[2] = hx * wy - hy * wx;
    return true;
}

/// Returns the phase-specific proportional gain for accelerometer correction.
float AccelCorrectionGain() {
    switch (g_flightStatus) {
        case FlightStatus::Ground:
            return settings::sensors::icm20948::kAccelCorrectionGainGround;
        case FlightStatus::Descent:
            return settings::sensors::icm20948::kAccelCorrectionGainDescent;
        case FlightStatus::Burn:
        case FlightStatus::Coast:
        case FlightStatus::Overshoot:
            return 0.0f;
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
bool ShouldLearnGyroBias(float accelTrust, float gyroNorm) {
    if (g_flightStatus == FlightStatus::Ground) {
        return accelTrust > 0.35f && gyroNorm <= settings::sensors::icm20948::kStationaryGyroMaxRadPerSec;
    }
    if (g_flightStatus == FlightStatus::Descent) {
        return accelTrust > 0.65f && gyroNorm <= 0.5f * settings::sensors::icm20948::kStationaryGyroMaxRadPerSec;
    }
    return false;
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
void AdaptiveQuaternionUpdate(const float accelNorm[3],
                              float accelTrust,
                              const float gyroRadPerSec[3],
                              const float magNorm[3],
                              float magTrust,
                              float dt) {
    if (dt <= 0.0f) {
        return;
    }

    float learningError[3] = {0.0f, 0.0f, 0.0f};
    float feedback[3] = {0.0f, 0.0f, 0.0f};

    if (accelTrust > 0.0f) {
        constexpr float kEarthUp[3] = {0.0f, 0.0f, 1.0f};
        float predictedGravity[3] = {0.0f, 0.0f, 0.0f};
        RotateEarthToBody(g_q, kEarthUp, predictedGravity);

        float accelError[3] = {0.0f, 0.0f, 0.0f};
        float accelFeedback[3] = {0.0f, 0.0f, 0.0f};
        Cross3(accelNorm[0], accelNorm[1], accelNorm[2],
               predictedGravity[0], predictedGravity[1], predictedGravity[2],
               accelError);
        for (int i = 0; i < 3; ++i) {
            learningError[i] += accelTrust * accelError[i];
            accelFeedback[i] = accelTrust * AccelCorrectionGain() * accelError[i];
        }
        LimitVector(accelFeedback, settings::sensors::icm20948::kAccelCorrectionMaxRateRadPerSec);
        for (int i = 0; i < 3; ++i) {
            feedback[i] += accelFeedback[i];
        }
    }

    if (magTrust > 0.0f) {
        float magError[3] = {0.0f, 0.0f, 0.0f};
        float magFeedback[3] = {0.0f, 0.0f, 0.0f};
        bool haveMagError = false;
        if (accelTrust > 0.0f) {
            haveMagError = ComputeBootstrapMagError(accelNorm, magNorm, magError);
        } else if (g_hasEarthMagReference) {
            float predictedMag[3] = {0.0f, 0.0f, 0.0f};
            RotateEarthToBody(g_q, g_magReferenceEarth, predictedMag);
            Cross3(magNorm[0], magNorm[1], magNorm[2],
                   predictedMag[0], predictedMag[1], predictedMag[2],
                   magError);
            haveMagError = true;
        }

        if (haveMagError) {
            for (int i = 0; i < 3; ++i) {
                learningError[i] += 0.5f * magTrust * magError[i];
                magFeedback[i] = magTrust * MagCorrectionGain() * magError[i];
            }
            LimitVector(magFeedback, settings::sensors::icm20948::kMagCorrectionMaxRateRadPerSec);
            for (int i = 0; i < 3; ++i) {
                feedback[i] += magFeedback[i];
            }
        }
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

    const bool learnBias = ShouldLearnGyroBias(accelTrust, Magnitude3(gyroRadPerSec[0], gyroRadPerSec[1], gyroRadPerSec[2]));
    if (learnBias) {
        for (int i = 0; i < 3; ++i) {
            g_gyroBias[i] += settings::sensors::icm20948::kGyroBiasLearningRate * learningError[i] * dt;
            g_gyroBias[i] = fmaxf(-settings::sensors::icm20948::kGyroBiasMaxRadPerSec,
                                  fminf(settings::sensors::icm20948::kGyroBiasMaxRadPerSec, g_gyroBias[i]));
        }
    }

    LimitVector(feedback, settings::sensors::icm20948::kTotalCorrectionMaxRateRadPerSec);

    float correctedGyro[3] = {
        gyroRadPerSec[0] + g_gyroBias[0] + feedback[0],
        gyroRadPerSec[1] + g_gyroBias[1] + feedback[1],
        gyroRadPerSec[2] + g_gyroBias[2] + feedback[2],
    };

    if (settings::ahrs::kEnableExponentialMap) {
        // Exponential map integration using Rodrigues formula.
        // More accurate than first-order Euler: reduces O(dt^2) error per step.
        math_utils::Quaternion qCurrent = math_utils::MakeQuaternion(g_q[0], g_q[1], g_q[2], g_q[3]);
        math_utils::Quaternion qUpdated = math_utils::ExponentialMapUpdate(
            qCurrent, correctedGyro[0], correctedGyro[1], correctedGyro[2], dt);
        g_q[0] = qUpdated.w;
        g_q[1] = qUpdated.x;
        g_q[2] = qUpdated.y;
        g_q[3] = qUpdated.z;
    } else {
        // Legacy first-order Euler integration.
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
    rollDeg = roll;
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
    out.icmQuaternion[0] = g_q[0];
    out.icmQuaternion[1] = g_q[1];
    out.icmQuaternion[2] = g_q[2];
    out.icmQuaternion[3] = g_q[3];
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
    out.hasIcmQuaternion = g_groundAlignmentReady;
    out.hasIcmYpr = g_groundAlignmentReady;
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
    if (g_icm.intEnableRawDataReady(true) != ICM_20948_Stat_Ok) {
        LOG_PRINTLN("ICM-20948: intEnableRawDataReady failed");
        return false;
    }

    attachInterrupt(digitalPinToInterrupt(kInterruptPin), DataReadyISR, FALLING);
    g_interruptConfigured = true;
    return true;
}

}  // namespace

/// Initializes the ICM-20948 over SPI.
bool Icm20948SensorBegin() {

    if (g_initialized) {
        return true;
    }

    g_dataReadyInterrupt = false;
    g_interruptConfigured = false;
    SPI1.begin();
    g_icm.begin(kChipSelectPin, SPI1);
    if (g_icm.status != ICM_20948_Stat_Ok) {
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
    if (!ConfigureInterrupt()) {
        LOG_PRINTLN("ICM-20948: interrupt setup failed; continuing in polling mode");
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
    g_initialized = true;
    return true;
}

bool Icm20948SensorIsInitialized() {
    return g_initialized;
}

Icm20948Diagnostics Icm20948SensorGetDiagnostics() {
    Icm20948Diagnostics diagnostics;
    diagnostics.initialized = g_initialized;
    diagnostics.hasQuaternion = g_groundAlignmentReady;
    diagnostics.alignmentReady = g_groundAlignmentReady;
    diagnostics.lastAcquireFresh = g_lastAcquireFresh;
    diagnostics.lastAcquireUsedCache = g_lastAcquireUsedCache;
    diagnostics.interruptConfigured = g_interruptConfigured;
    diagnostics.lastAcquireUsedInterrupt = g_lastAcquireUsedInterrupt;
    diagnostics.lastSampleMicros = g_lastSampleUs;
    return diagnostics;
}

void Icm20948SensorSetFlightStatus(FlightStatus status) {
    g_flightStatus = status;
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
    UpdateRailConstraintState(nowUs);
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
    g_lastAcquireFresh = true;
    g_lastAcquireUsedInterrupt = interruptTriggered;

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
    magCalNorm[1] = -magCalNorm[1];
    magCalNorm[2] = -magCalNorm[2];
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
    const float baseAccelTrust = ComputeAccelTrust(accelMagnitudeG, gyroNorm);
    const float baseMagTrust = ComputeMagTrust(magMagnitude, gyroNorm);
    const float accelTrust = baseAccelTrust * effectiveAccelSaturationTrust * accelOutlierTrust;
    const float magTrust = baseMagTrust * effectiveGyroSaturationTrust * gyroOutlierTrust;
    UpdateMagMagnitudeReference(magMagnitude, magTrust);
    UpdateGroundAlignment(accelCalNorm, accelTrust, magCalNorm, magTrust, gyroNorm);

    if (g_groundAlignmentReady && !gyroSaturated) {
        AdaptiveQuaternionUpdate(accelCalNorm, accelTrust, gyroCal, magCalNorm, magTrust, dt);
        UpdateEarthMagReference(magCalNorm, accelTrust, magTrust);
        CaptureRailReferenceQuaternion(accelTrust, magTrust);
    }
    out.icmQuaternion[0] = g_q[0];
    out.icmQuaternion[1] = g_q[1];
    out.icmQuaternion[2] = g_q[2];
    out.icmQuaternion[3] = g_q[3];
    out.hasIcmQuaternion = g_groundAlignmentReady;

    QuaternionToYprDeg(out.icmYprDeg[0], out.icmYprDeg[1], out.icmYprDeg[2]);
    g_lastIcmYprDeg[0] = out.icmYprDeg[0];
    g_lastIcmYprDeg[1] = out.icmYprDeg[1];
    g_lastIcmYprDeg[2] = out.icmYprDeg[2];
    out.hasIcmYpr = g_groundAlignmentReady;
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
