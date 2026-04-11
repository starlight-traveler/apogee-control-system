#include "lsm9ds1_sensor.h"

#include <Arduino.h>
#include <SPI.h>

#include <SparkFunLSM9DS1.h>

#include "math_utils.h"
#include "serial_logging.h"
#include "settings.h"

namespace {

constexpr uint8_t kAccelGyroChipSelectPin = settings::sensors::lsm9ds1::kAccelGyroChipSelectPin;
constexpr uint8_t kMagChipSelectPin = settings::sensors::lsm9ds1::kMagChipSelectPin;
constexpr int8_t kInterruptPin = settings::sensors::lsm9ds1::kInterruptPin;
constexpr uint32_t kSampleIntervalUs = settings::sensors::lsm9ds1::kSampleIntervalUs;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kGToMps2 = 9.80665f;
constexpr float kRadToDeg = 57.295779513082320876f;
constexpr uint8_t kQuaternionInvalidDropThreshold = 2;

LSM9DS1 g_lsm;
volatile bool g_dataReadyInterrupt = false;
bool g_initialized = false;
bool g_hasCachedSample = false;
uint32_t g_lastSampleUs = 0;
uint32_t g_lastFilterUs = 0;
FlightStatus g_flightStatus = FlightStatus::Ground;
float g_burnoutTimestampSeconds = 0.0f;
float g_currentTimestampSeconds = 0.0f;

float g_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float g_lastQuaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
bool g_hasQuaternionContinuityReference = false;
float g_gyroBiasLearned[3] = {0.0f, 0.0f, 0.0f};
float g_lastAccel[3] = {0.0f, 0.0f, 0.0f};
float g_lastGyro[3] = {0.0f, 0.0f, 0.0f};
float g_lastQuaternionOut[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float g_lastYprDeg[3] = {0.0f, 0.0f, 0.0f};
float g_lastTemperatureC = 0.0f;
float g_lastAhrsDt = 0.0f;
float g_lastAccelTrust = 0.0f;
float g_lastMagTrust = 0.0f;
bool g_haveAccel = false;
bool g_haveGyro = false;
bool g_haveMag = false;
bool g_haveQuaternion = false;
bool g_hasMagReference = false;
float g_magReferenceNorm = 0.0f;
bool g_groundAlignmentReady = false;
float g_groundAlignmentAccelSum[3] = {0.0f, 0.0f, 0.0f};
float g_groundAlignmentMagSum[3] = {0.0f, 0.0f, 0.0f};
uint16_t g_groundAlignmentSampleCount = 0;
bool g_lastAcquireFresh = false;
bool g_lastAcquireUsedCache = false;
bool g_lastAcquireUsedInterrupt = false;
bool g_interruptConfigured = false;
bool g_fifoEnabled = false;
float g_crossCheckTrust = 1.0f;
uint8_t g_invalidQuaternionStreak = 0;

// Gyro bias learning validation: track accelerometer magnitude for sanity check.
float g_lastAccelMagnitudeG = 1.0f;
float g_lastMagMagnitude = 0.0f;
float g_lastMagBody[3] = {0.0f, 0.0f, 0.0f};

float g_activeAccelLsbPerG = 16384.0f;
float g_activeGyroLsbPerDps = 114.285714f;
float g_activeGyroRadPerSecPerLsb = 0.0f;
float g_calibrationAccelLsbPerG = 16384.0f;
float g_calibrationGyroLsbPerDps = 114.285714f;
float g_activeMagGaussPerLsb = 0.00043f;
float g_calibrationMagGaussPerLsb = 0.00014f;

void ApplyQuaternionContinuity();

void DataReadyISR() {
    g_dataReadyInterrupt = true;
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

inline float ClampUnit(float value) {
    if (value < -1.0f) {
        return -1.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
}

inline float Clamp01(float value) {
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
}

float AccelLsbPerGForRange(uint8_t rangeG) {
    switch (rangeG) {
        case 2:
            return 16384.0f;
        case 4:
            return 8192.0f;
        case 8:
            return 4096.0f;
        case 16:
            // The SparkFun LSM9DS1 library uses 0.732 mg/LSB at +/-16 g.
            return 1366.12024f;
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

float GyroRadPerSecPerLsbForRange(uint16_t rangeDps) {
    return (1.0f / GyroLsbPerDpsForRange(rangeDps)) * (kPi / 180.0f);
}

float MagGaussPerLsbForRange(uint8_t rangeGauss) {
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

void Apply3x3(const float matrix[3][3], const float in[3], float out[3]) {
    const float x = in[0];
    const float y = in[1];
    const float z = in[2];

    out[0] = matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z;
    out[1] = matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z;
    out[2] = matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z;
}

void ApplyMountRotation(float vector[3]) {
    float rotated[3] = {0.0f, 0.0f, 0.0f};
    Apply3x3(settings::sensors::lsm9ds1::kMountRotation, vector, rotated);
    vector[0] = rotated[0];
    vector[1] = rotated[1];
    vector[2] = rotated[2];
}

void ApplyGyroCalibration(float vector[3]) {
    float corrected[3] = {0.0f, 0.0f, 0.0f};
    Apply3x3(settings::sensors::lsm9ds1::kGyroAinv, vector, corrected);
    vector[0] = corrected[0];
    vector[1] = corrected[1];
    vector[2] = corrected[2];
}

void ApplyAxisTransform(float vector[3]) {
    float remapped[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 3; ++i) {
        const uint8_t source = settings::sensors::lsm9ds1::kAxisMap[i];
        remapped[i] = static_cast<float>(settings::sensors::lsm9ds1::kAxisSign[i]) * vector[source];
    }
    vector[0] = remapped[0];
    vector[1] = remapped[1];
    vector[2] = remapped[2];
}

bool Normalize3(float &x, float &y, float &z) {
    const float norm = sqrtf(x * x + y * y + z * z);
    if (norm <= 1.0e-9f) {
        return false;
    }
    const float inv = 1.0f / norm;
    x *= inv;
    y *= inv;
    z *= inv;
    return true;
}

float Magnitude3(float x, float y, float z) {
    return sqrtf(x * x + y * y + z * z);
}

void LimitVector(float vector[3], float maxNorm) {
    const float norm = Magnitude3(vector[0], vector[1], vector[2]);
    if (norm <= maxNorm || norm <= 1.0e-9f || maxNorm <= 0.0f) {
        return;
    }
    const float scale = maxNorm / norm;
    vector[0] *= scale;
    vector[1] *= scale;
    vector[2] *= scale;
}

float WindowTrust(float value, float minValue, float maxValue) {
    if (!(value >= minValue) || !(value <= maxValue) || !(maxValue > minValue)) {
        return 0.0f;
    }
    const float center = 0.5f * (minValue + maxValue);
    const float halfWidth = 0.5f * (maxValue - minValue);
    return Clamp01(1.0f - fabsf(value - center) / halfWidth);
}

float DescendingTrust(float value, float fullTrustMax, float zeroTrustMin) {
    if (value <= fullTrustMax) {
        return 1.0f;
    }
    if (value >= zeroTrustMin || !(zeroTrustMin > fullTrustMax)) {
        return 0.0f;
    }
    return Clamp01((zeroTrustMin - value) / (zeroTrustMin - fullTrustMax));
}

void NegateQuaternion(float q[4]) {
    q[0] = -q[0];
    q[1] = -q[1];
    q[2] = -q[2];
    q[3] = -q[3];
}

float QuaternionDot(const float a[4], const float b[4]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
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

    float eastBody[3] = {
        upBody[1] * magneticBody[2] - upBody[2] * magneticBody[1],
        upBody[2] * magneticBody[0] - upBody[0] * magneticBody[2],
        upBody[0] * magneticBody[1] - upBody[1] * magneticBody[0],
    };
    if (!Normalize3(eastBody[0], eastBody[1], eastBody[2])) {
        return false;
    }

    float northBody[3] = {
        eastBody[1] * upBody[2] - eastBody[2] * upBody[1],
        eastBody[2] * upBody[0] - eastBody[0] * upBody[2],
        eastBody[0] * upBody[1] - eastBody[1] * upBody[0],
    };
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

void ApplyQuaternionContinuity() {
    if (!g_hasQuaternionContinuityReference) {
        for (int i = 0; i < 4; ++i) {
            g_lastQuaternion[i] = g_q[i];
        }
        g_hasQuaternionContinuityReference = true;
        return;
    }
    if (QuaternionDot(g_q, g_lastQuaternion) < 0.0f) {
        NegateQuaternion(g_q);
    }
    for (int i = 0; i < 4; ++i) {
        g_lastQuaternion[i] = g_q[i];
    }
}

void QuaternionToEulerDeg(const float q[4], float yprDeg[3]) {
    float roll = atan2f((q[0] * q[1] + q[2] * q[3]), 0.5f - (q[1] * q[1] + q[2] * q[2]));
    float pitch = asinf(ClampUnit(2.0f * (q[0] * q[2] - q[1] * q[3])));
    float yaw = atan2f((q[1] * q[2] + q[0] * q[3]), 0.5f - (q[2] * q[2] + q[3] * q[3]));

    yaw *= kRadToDeg;
    pitch *= kRadToDeg;
    roll *= kRadToDeg;

    yaw = -(yaw + settings::sensors::lsm9ds1::kMagDeclinationDeg);
    if (yaw < 0.0f) {
        yaw += 360.0f;
    }
    if (yaw >= 360.0f) {
        yaw -= 360.0f;
    }

    yprDeg[0] = yaw;
    yprDeg[1] = pitch;
    yprDeg[2] = roll;
}

float RescaleCalibrationCounts(float calibrationCounts, float activeLsbPerUnit, float calibrationLsbPerUnit) {
    if (!(calibrationLsbPerUnit > 0.0f)) {
        return calibrationCounts;
    }
    return calibrationCounts * (activeLsbPerUnit / calibrationLsbPerUnit);
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
                                             settings::sensors::lsm9ds1::kAccelCorrectionMinG,
                                             settings::sensors::lsm9ds1::kAccelCorrectionMaxG);
    const float rateTrust = DescendingTrust(gyroNorm,
                                            settings::sensors::lsm9ds1::kAccelCorrectionGyroFadeStartRadPerSec,
                                            settings::sensors::lsm9ds1::kAccelCorrectionGyroFadeEndRadPerSec);
    return phaseTrust * magnitudeTrust * rateTrust * flightSuppression;
}

float MagTrustPhaseScale() {
    switch (g_flightStatus) {
        case FlightStatus::Ground:
            return settings::sensors::lsm9ds1::kMagTrustGround;
        case FlightStatus::Burn:
            return settings::sensors::lsm9ds1::kMagTrustBurn;
        case FlightStatus::Coast:
            return settings::sensors::lsm9ds1::kMagTrustCoast;
        case FlightStatus::Overshoot:
            return settings::sensors::lsm9ds1::kMagTrustOvershoot;
        case FlightStatus::Descent:
            return settings::sensors::lsm9ds1::kMagTrustDescent;
    }
    return 0.0f;
}

float ComputeMagTrust(float magMagnitude, float gyroNorm) {
    const float phaseTrust = MagTrustPhaseScale();
    if (!(phaseTrust > 0.0f) || !(magMagnitude > 0.0f)) {
        return 0.0f;
    }
    if (!g_hasMagReference || !(g_magReferenceNorm > 0.0f)) {
        return phaseTrust * DescendingTrust(gyroNorm,
                                            settings::sensors::lsm9ds1::kMagTrustGyroFadeStartRadPerSec,
                                            settings::sensors::lsm9ds1::kMagTrustGyroFadeEndRadPerSec);
    }
    const float relativeError = fabsf(magMagnitude - g_magReferenceNorm) / g_magReferenceNorm;
    if (relativeError >= settings::sensors::lsm9ds1::kMagCorrectionMaxRelativeError) {
        return 0.0f;
    }
    const float magnitudeTrust =
        Clamp01(1.0f - relativeError / settings::sensors::lsm9ds1::kMagCorrectionMaxRelativeError);
    const float rateTrust = DescendingTrust(gyroNorm,
                                            settings::sensors::lsm9ds1::kMagTrustGyroFadeStartRadPerSec,
                                            settings::sensors::lsm9ds1::kMagTrustGyroFadeEndRadPerSec);
    return phaseTrust * magnitudeTrust * rateTrust;
}

void UpdateMagReference(float magMagnitude, float trust) {
    if (!(magMagnitude > 0.0f) || trust <= 0.0f) {
        return;
    }
    if (!g_hasMagReference) {
        g_magReferenceNorm = magMagnitude;
        g_hasMagReference = true;
        return;
    }
    const float blend = settings::sensors::lsm9ds1::kMagReferenceBlend * trust;
    g_magReferenceNorm += blend * (magMagnitude - g_magReferenceNorm);
}

void UpdateGroundAlignment(const float accelNorm[3], float accelTrust, const float magNorm[3], float magTrust, float gyroNorm) {
    if (g_groundAlignmentReady || g_flightStatus != FlightStatus::Ground) {
        return;
    }
    if (accelTrust < settings::sensors::lsm9ds1::kGroundAlignmentAccelTrustMin ||
        magTrust < settings::sensors::lsm9ds1::kGroundAlignmentMagTrustMin ||
        gyroNorm > settings::sensors::lsm9ds1::kStationaryGyroMaxRadPerSec) {
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

    if (g_groundAlignmentSampleCount < settings::sensors::lsm9ds1::kGroundAlignmentMinSamples) {
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

    InitializeQuaternionFromAccelMag(avgAccel, avgMag);
}

/// Returns the phase-specific proportional gain for accelerometer correction.
/// Returns aggressive gain during burnout correction window.
float AccelCorrectionGain() {
    switch (g_flightStatus) {
        case FlightStatus::Ground:
            return settings::sensors::lsm9ds1::kAccelCorrectionGainGround;
        case FlightStatus::Descent:
            return settings::sensors::lsm9ds1::kAccelCorrectionGainDescent;
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

float MagCorrectionGain() {
    switch (g_flightStatus) {
        case FlightStatus::Ground:
            return settings::sensors::lsm9ds1::kMagCorrectionGainGround;
        case FlightStatus::Descent:
            return settings::sensors::lsm9ds1::kMagCorrectionGainDescent;
        case FlightStatus::Burn:
        case FlightStatus::Coast:
        case FlightStatus::Overshoot:
            return settings::sensors::lsm9ds1::kMagCorrectionGainFlight;
    }
    return 0.0f;
}

/// Returns true when residual gyro bias can be learned safely.
/// Validates accelerometer magnitude is ~1g to prevent learning corrupted bias.
bool ShouldLearnGyroBias(float accelTrust, float gyroNorm) {
    // Validate accelerometer magnitude is close to 1g (gravity only).
    // If magnitude is way off, accelerometer may be corrupted - skip bias learning.
    constexpr float kMinAccelMagnitudeG = 0.85f;
    constexpr float kMaxAccelMagnitudeG = 1.15f;
    if (g_lastAccelMagnitudeG < kMinAccelMagnitudeG || g_lastAccelMagnitudeG > kMaxAccelMagnitudeG) {
        return false;
    }

    if (g_flightStatus == FlightStatus::Ground) {
        return accelTrust > 0.35f && gyroNorm <= settings::sensors::lsm9ds1::kStationaryGyroMaxRadPerSec;
    }
    if (g_flightStatus == FlightStatus::Descent) {
        return accelTrust > 0.65f &&
               gyroNorm <= 0.5f * settings::sensors::lsm9ds1::kStationaryGyroMaxRadPerSec;
    }
    return false;
}

void LearnGyroBias(const float gyroRadPerSec[3], float accelTrust, float gyroNorm) {
    if (!ShouldLearnGyroBias(accelTrust, gyroNorm)) {
        return;
    }
    const float alpha = settings::sensors::lsm9ds1::kGyroBiasLearningRate * g_lastAhrsDt;
    for (int i = 0; i < 3; ++i) {
        g_gyroBiasLearned[i] += alpha * (gyroRadPerSec[i] - g_gyroBiasLearned[i]);
        if (g_gyroBiasLearned[i] > settings::sensors::lsm9ds1::kGyroBiasMaxRadPerSec) {
            g_gyroBiasLearned[i] = settings::sensors::lsm9ds1::kGyroBiasMaxRadPerSec;
        }
        if (g_gyroBiasLearned[i] < -settings::sensors::lsm9ds1::kGyroBiasMaxRadPerSec) {
            g_gyroBiasLearned[i] = -settings::sensors::lsm9ds1::kGyroBiasMaxRadPerSec;
        }
    }
}

float Dot3(const float a[3], const float b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void AdaptiveQuaternionUpdate(const float accelNorm[3],
                              float accelTrust,
                              const float gyroRadPerSec[3],
                              const float magNorm[3],
                              float magTrust,
                              float dt) {
    if (dt <= 0.0f) {
        return;
    }

    const float q1 = g_q[0];  // w
    const float q2 = g_q[1];  // x
    const float q3 = g_q[2];  // y
    const float q4 = g_q[3];  // z

    // Rotation matrix columns = Earth basis vectors expressed in body frame.
    float northBody[3] = {
        q1 * q1 + q2 * q2 - q3 * q3 - q4 * q4,
        2.0f * (q2 * q3 + q1 * q4),
        2.0f * (q2 * q4 - q1 * q3),
    };

    float upBody[3] = {
        2.0f * (q2 * q4 + q1 * q3),
        2.0f * (q3 * q4 - q1 * q2),
        q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4,
    };

    // Tilt correction: predicted up x measured up.
    float accelError[3] = {
        upBody[1] * accelNorm[2] - upBody[2] * accelNorm[1],
        upBody[2] * accelNorm[0] - upBody[0] * accelNorm[2],
        upBody[0] * accelNorm[1] - upBody[1] * accelNorm[0],
    };

    // Yaw-only mag correction so mag cannot tilt pitch/roll.
    float magError[3] = {0.0f, 0.0f, 0.0f};
    if (magTrust > 0.0f) {
        const float magUpDot = Dot3(magNorm, upBody);
        float magHoriz[3] = {
            magNorm[0] - magUpDot * upBody[0],
            magNorm[1] - magUpDot * upBody[1],
            magNorm[2] - magUpDot * upBody[2],
        };

        const float northUpDot = Dot3(northBody, upBody);
        float northHoriz[3] = {
            northBody[0] - northUpDot * upBody[0],
            northBody[1] - northUpDot * upBody[1],
            northBody[2] - northUpDot * upBody[2],
        };

        if (Normalize3(magHoriz[0], magHoriz[1], magHoriz[2]) &&
            Normalize3(northHoriz[0], northHoriz[1], northHoriz[2])) {
            const float yawError =
                upBody[0] * (northHoriz[1] * magHoriz[2] - northHoriz[2] * magHoriz[1]) +
                upBody[1] * (northHoriz[2] * magHoriz[0] - northHoriz[0] * magHoriz[2]) +
                upBody[2] * (northHoriz[0] * magHoriz[1] - northHoriz[1] * magHoriz[0]);

            magError[0] = upBody[0] * yawError;
            magError[1] = upBody[1] * yawError;
            magError[2] = upBody[2] * yawError;
        } else {
            magTrust = 0.0f;
        }
    }

    const float accelGain = accelTrust * AccelCorrectionGain();
    const float magGain = magTrust * MagCorrectionGain();

    for (int i = 0; i < 3; ++i) {
        accelError[i] *= accelGain;
        magError[i] *= magGain;
    }

    LimitVector(accelError, settings::sensors::lsm9ds1::kAccelCorrectionMaxRateRadPerSec);
    LimitVector(magError, settings::sensors::lsm9ds1::kMagCorrectionMaxRateRadPerSec);

    float correction[3] = {
        accelError[0] + magError[0],
        accelError[1] + magError[1],
        accelError[2] + magError[2],
    };
    LimitVector(correction, settings::sensors::lsm9ds1::kTotalCorrectionMaxRateRadPerSec);

    const float gx = (gyroRadPerSec[0] - g_gyroBiasLearned[0] + correction[0]) * (0.5f * dt);
    const float gy = (gyroRadPerSec[1] - g_gyroBiasLearned[1] + correction[1]) * (0.5f * dt);
    const float gz = (gyroRadPerSec[2] - g_gyroBiasLearned[2] + correction[2]) * (0.5f * dt);

    const float qa = g_q[0];
    const float qb = g_q[1];
    const float qc = g_q[2];
    const float qd = g_q[3];

    g_q[0] += (-qb * gx - qc * gy - qd * gz);
    g_q[1] += (qa * gx + qc * gz - qd * gy);
    g_q[2] += (qa * gy - qb * gz + qd * gx);
    g_q[3] += (qa * gz + qb * gy - qc * gx);

    const float norm = sqrtf(g_q[0] * g_q[0] + g_q[1] * g_q[1] +
                             g_q[2] * g_q[2] + g_q[3] * g_q[3]);
    if (norm <= 1.0e-9f) {
        g_hasQuaternionContinuityReference = false;
        return;
    }

    const float invNorm = 1.0f / norm;
    g_q[0] *= invNorm;
    g_q[1] *= invNorm;
    g_q[2] *= invNorm;
    g_q[3] *= invNorm;

    if (!math_utils::ValidateQuaternionArray(g_q)) {
        g_hasQuaternionContinuityReference = false;
        return;
    }

    ApplyQuaternionContinuity();
}

void PublishFromState(SensorData &out, uint32_t nowUs) {
    out.timestamp = static_cast<float>(nowUs) * 1.0e-6f;
    out.accelICM[0] = g_lastAccel[0];
    out.accelICM[1] = g_lastAccel[1];
    out.accelICM[2] = g_lastAccel[2];
    out.gyro[0] = g_lastGyro[0];
    out.gyro[1] = g_lastGyro[1];
    out.gyro[2] = g_lastGyro[2];
    out.icmQuaternion[0] = g_lastQuaternionOut[0];
    out.icmQuaternion[1] = g_lastQuaternionOut[1];
    out.icmQuaternion[2] = g_lastQuaternionOut[2];
    out.icmQuaternion[3] = g_lastQuaternionOut[3];
    out.icmYprDeg[0] = g_lastYprDeg[0];
    out.icmYprDeg[1] = g_lastYprDeg[1];
    out.icmYprDeg[2] = g_lastYprDeg[2];
    out.icmTemperatureC = g_lastTemperatureC;
    out.icmAhrsDt = g_lastAhrsDt;
    out.icmAccelTrust = g_lastAccelTrust;
    out.icmMagTrust = g_lastMagTrust;
    out.icmGyroBias[0] = g_gyroBiasLearned[0];
    out.icmGyroBias[1] = g_gyroBiasLearned[1];
    out.icmGyroBias[2] = g_gyroBiasLearned[2];
    out.hasIcmQuaternion = g_haveQuaternion;
    out.hasIcmYpr = g_haveQuaternion;
}

bool InterruptAsserted() {
    if (!(g_interruptConfigured && kInterruptPin >= 0)) {
        return false;
    }
    return digitalRead(kInterruptPin) == LOW;
}

void ApplyLibraryMagOffsets() {
    if (!settings::sensors::lsm9ds1::kUseLibraryMagOffsets) {
        return;
    }
    const int16_t x = static_cast<int16_t>(lroundf(
        RescaleCalibrationCounts(settings::sensors::lsm9ds1::kMagBias[0],
                                 1.0f / g_activeMagGaussPerLsb,
                                 1.0f / g_calibrationMagGaussPerLsb)));
    const int16_t y = static_cast<int16_t>(lroundf(
        RescaleCalibrationCounts(settings::sensors::lsm9ds1::kMagBias[1],
                                 1.0f / g_activeMagGaussPerLsb,
                                 1.0f / g_calibrationMagGaussPerLsb)));
    const int16_t z = static_cast<int16_t>(lroundf(
        RescaleCalibrationCounts(settings::sensors::lsm9ds1::kMagBias[2],
                                 1.0f / g_activeMagGaussPerLsb,
                                 1.0f / g_calibrationMagGaussPerLsb)));
    g_lsm.magOffset(X_AXIS, x);
    g_lsm.magOffset(Y_AXIS, y);
    g_lsm.magOffset(Z_AXIS, z);
}

bool ConfigureSensor() {
    g_lsm.settings.device.commInterface = IMU_MODE_SPI;
    g_lsm.settings.device.agAddress = kAccelGyroChipSelectPin;
    g_lsm.settings.device.mAddress = kMagChipSelectPin;
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
    SPI1.begin();
    if (g_lsm.beginSPI(kAccelGyroChipSelectPin, kMagChipSelectPin, SPI1) == 0) {
        return false;
    }

    if (kInterruptPin >= 0) {
        pinMode(kInterruptPin, INPUT_PULLUP);
        g_lsm.configInt(XG_INT1, INT_DRDY_XL | INT_DRDY_G, INT_ACTIVE_LOW, INT_PUSH_PULL);
        attachInterrupt(digitalPinToInterrupt(kInterruptPin), DataReadyISR, FALLING);
        g_interruptConfigured = true;
    }

    if (settings::sensors::lsm9ds1::kUseFifo) {
        g_lsm.enableFIFO(true);
        g_lsm.setFIFO(FIFO_CONT, settings::sensors::lsm9ds1::kFifoThresholdSamples);
        g_fifoEnabled = true;
    }
    return true;
}

void ConfigureSensorScales() {
    g_activeAccelLsbPerG = AccelLsbPerGForRange(settings::sensors::lsm9ds1::kAccelRangeG);
    g_activeGyroLsbPerDps = GyroLsbPerDpsForRange(settings::sensors::lsm9ds1::kGyroRangeDps);
    g_activeGyroRadPerSecPerLsb = GyroRadPerSecPerLsbForRange(settings::sensors::lsm9ds1::kGyroRangeDps);
    g_calibrationAccelLsbPerG = AccelLsbPerGForRange(settings::sensors::lsm9ds1::kCalibrationAccelRangeG);
    g_calibrationGyroLsbPerDps = GyroLsbPerDpsForRange(settings::sensors::lsm9ds1::kCalibrationGyroRangeDps);
    g_activeMagGaussPerLsb = MagGaussPerLsbForRange(settings::sensors::lsm9ds1::kMagRangeGauss);
    g_calibrationMagGaussPerLsb = MagGaussPerLsbForRange(settings::sensors::lsm9ds1::kCalibrationMagRangeGauss);
}

bool SensorDataAvailable() {
    return g_lsm.accelAvailable() || g_lsm.gyroAvailable() || g_lsm.magAvailable();
}

bool UpdateSensorCache() {
    bool updated = false;
    uint8_t fifoSamples = 0;
    if (g_fifoEnabled) {
        fifoSamples = g_lsm.getFIFOSamples();
    }

    const bool accelReady = fifoSamples > 0 || g_lsm.accelAvailable();
    const bool gyroReady = fifoSamples > 0 || g_lsm.gyroAvailable();
    uint8_t burstSamples = 1;
    if (fifoSamples > 0) {
        burstSamples = fifoSamples;
        if (burstSamples > settings::sensors::lsm9ds1::kFifoMaxBurstSamplesPerAcquire) {
            burstSamples = settings::sensors::lsm9ds1::kFifoMaxBurstSamplesPerAcquire;
        }
    }

    if (accelReady || gyroReady) {
        for (uint8_t i = 0; i < burstSamples; ++i) {
            if (gyroReady) {
                g_lsm.readGyro();
                g_haveGyro = true;
                updated = true;
            }
            if (accelReady) {
                g_lsm.readAccel();
                g_haveAccel = true;
                updated = true;
            }
        }
    }
    if (g_lsm.magAvailable()) {
        g_lsm.readMag();
        g_haveMag = true;
        updated = true;
    }
    if (updated && g_lsm.tempAvailable()) {
        g_lsm.readTemp();
    }
    return updated;
}

}  // namespace

bool Lsm9ds1SensorBegin() {
    if (g_initialized) {
        return true;
    }
    g_dataReadyInterrupt = false;
    g_interruptConfigured = false;
    g_fifoEnabled = false;
    if (!ConfigureSensor()) {
        LOG_PRINTLN("LSM9DS1: beginSPI failed");
        return false;
    }
    ConfigureSensorScales();
    ApplyLibraryMagOffsets();
    g_lastSampleUs = 0;
    g_lastFilterUs = 0;
    g_hasCachedSample = false;
    g_hasMagReference = false;
    g_magReferenceNorm = 0.0f;
    g_haveAccel = false;
    g_haveGyro = false;
    g_haveMag = false;
    g_haveQuaternion = false;
    ResetGroundAlignment();
    g_q[0] = 1.0f;
    g_q[1] = 0.0f;
    g_q[2] = 0.0f;
    g_q[3] = 0.0f;
    g_hasQuaternionContinuityReference = false;
    g_invalidQuaternionStreak = 0;
    g_lastAcquireFresh = false;
    g_lastAcquireUsedCache = false;
    g_lastAcquireUsedInterrupt = false;
    g_crossCheckTrust = 1.0f;
    g_initialized = true;
    return true;
}

bool Lsm9ds1SensorIsInitialized() {
    return g_initialized;
}

void Lsm9ds1SensorSetFlightStatus(FlightStatus status) {
    g_flightStatus = status;
}

void Lsm9ds1SensorSetBurnoutTimestamp(float burnoutTimeSeconds) {
    g_burnoutTimestampSeconds = burnoutTimeSeconds;
}

void Lsm9ds1SensorSetCurrentTimestamp(float currentTimeSeconds) {
    g_currentTimestampSeconds = currentTimeSeconds;
}

void Lsm9ds1SensorSetCrossCheckTrust(float trust) {
    g_crossCheckTrust = Clamp01(trust);
}

bool Lsm9ds1SensorAcquire(SensorData &out) {
    if (!g_initialized) {
        return false;
    }
    g_lastAcquireFresh = false;
    g_lastAcquireUsedCache = false;
    g_lastAcquireUsedInterrupt = false;

    const uint32_t nowUs = micros();
    if (g_lastSampleUs != 0 && (nowUs - g_lastSampleUs) < kSampleIntervalUs) {
        if (!g_hasCachedSample) {
            return false;
        }
        g_lastAcquireUsedCache = true;
        PublishFromState(out, nowUs);
        return true;
    }

    bool interruptTriggered = false;
    noInterrupts();
    if (g_dataReadyInterrupt) {
        g_dataReadyInterrupt = false;
        interruptTriggered = true;
    }
    interrupts();
    if (!interruptTriggered && InterruptAsserted()) {
        interruptTriggered = true;
    }

    bool shouldRead = false;
    if (g_interruptConfigured) {
        shouldRead = interruptTriggered;
        if (!shouldRead && g_fifoEnabled && g_lsm.getFIFOSamples() > 0) {
            shouldRead = true;
        }
        if (!shouldRead && g_lsm.magAvailable()) {
            shouldRead = true;
        }
    } else {
        shouldRead = SensorDataAvailable();
    }

    if (!shouldRead) {
        if (!g_hasCachedSample) {
            return false;
        }
        g_lastAcquireUsedCache = true;
        PublishFromState(out, nowUs);
        return true;
    }

    if (!UpdateSensorCache()) {
        if (!g_hasCachedSample) {
            return false;
        }
        g_lastAcquireUsedCache = true;
        PublishFromState(out, nowUs);
        return true;
    }

    g_lastSampleUs = nowUs;
    g_lastAcquireFresh = true;
    g_lastAcquireUsedInterrupt = interruptTriggered;

    const float rawGx = static_cast<float>(g_lsm.gx);
    const float rawGy = static_cast<float>(g_lsm.gy);
    const float rawGz = static_cast<float>(g_lsm.gz);
    const float rawAx = static_cast<float>(g_lsm.ax);
    const float rawAy = static_cast<float>(g_lsm.ay);
    const float rawAz = static_cast<float>(g_lsm.az);
    const float rawMx = static_cast<float>(g_lsm.mx);
    const float rawMy = static_cast<float>(g_lsm.my);
    const float rawMz = static_cast<float>(g_lsm.mz);

    g_lastTemperatureC = static_cast<float>(g_lsm.temperature);
    const float temperatureDeltaC =
        g_lastTemperatureC - settings::sensors::lsm9ds1::kGyroReferenceTemperatureC;

    float gyroRaw[3] = {
        rawGx - RescaleCalibrationCounts(settings::sensors::lsm9ds1::kGyroOffset[0],
                                         g_activeGyroLsbPerDps,
                                         g_calibrationGyroLsbPerDps),
        rawGy - RescaleCalibrationCounts(settings::sensors::lsm9ds1::kGyroOffset[1],
                                         g_activeGyroLsbPerDps,
                                         g_calibrationGyroLsbPerDps),
        rawGz - RescaleCalibrationCounts(settings::sensors::lsm9ds1::kGyroOffset[2],
                                         g_activeGyroLsbPerDps,
                                         g_calibrationGyroLsbPerDps),
    };
    ApplyAxisTransform(gyroRaw);
    float gyroRadPerSec[3] = {
        gyroRaw[0] * g_activeGyroRadPerSecPerLsb -
            settings::sensors::lsm9ds1::kGyroTempBiasSlopeRadPerSecPerC[0] * temperatureDeltaC,
        gyroRaw[1] * g_activeGyroRadPerSecPerLsb -
            settings::sensors::lsm9ds1::kGyroTempBiasSlopeRadPerSecPerC[1] * temperatureDeltaC,
        gyroRaw[2] * g_activeGyroRadPerSecPerLsb -
            settings::sensors::lsm9ds1::kGyroTempBiasSlopeRadPerSecPerC[2] * temperatureDeltaC,
    };
    ApplyGyroCalibration(gyroRadPerSec);
    ApplyMountRotation(gyroRadPerSec);

    float accelRaw[3] = {
        rawAx - RescaleCalibrationCounts(settings::sensors::lsm9ds1::kAccelBias[0],
                                         g_activeAccelLsbPerG,
                                         g_calibrationAccelLsbPerG),
        rawAy - RescaleCalibrationCounts(settings::sensors::lsm9ds1::kAccelBias[1],
                                         g_activeAccelLsbPerG,
                                         g_calibrationAccelLsbPerG),
        rawAz - RescaleCalibrationCounts(settings::sensors::lsm9ds1::kAccelBias[2],
                                         g_activeAccelLsbPerG,
                                         g_calibrationAccelLsbPerG),
    };
    Apply3x3(settings::sensors::lsm9ds1::kAccelAinv, accelRaw, accelRaw);
    ApplyAxisTransform(accelRaw);
    ApplyMountRotation(accelRaw);
    const float accelMagnitudeG = Magnitude3(accelRaw[0], accelRaw[1], accelRaw[2]) / g_activeAccelLsbPerG;
    float accelNorm[3] = {accelRaw[0], accelRaw[1], accelRaw[2]};
    Normalize3(accelNorm[0], accelNorm[1], accelNorm[2]);

    const float magBiasX = settings::sensors::lsm9ds1::kUseLibraryMagOffsets
                               ? 0.0f
                               : RescaleCalibrationCounts(settings::sensors::lsm9ds1::kMagBias[0],
                                                          1.0f / g_activeMagGaussPerLsb,
                                                          1.0f / g_calibrationMagGaussPerLsb);
    const float magBiasY = settings::sensors::lsm9ds1::kUseLibraryMagOffsets
                               ? 0.0f
                               : RescaleCalibrationCounts(settings::sensors::lsm9ds1::kMagBias[1],
                                                          1.0f / g_activeMagGaussPerLsb,
                                                          1.0f / g_calibrationMagGaussPerLsb);
    const float magBiasZ = settings::sensors::lsm9ds1::kUseLibraryMagOffsets
                               ? 0.0f
                               : RescaleCalibrationCounts(settings::sensors::lsm9ds1::kMagBias[2],
                                                          1.0f / g_activeMagGaussPerLsb,
                                                          1.0f / g_calibrationMagGaussPerLsb);
    float magRaw[3] = {
        rawMx - magBiasX,
        rawMy - magBiasY,
        rawMz - magBiasZ,
    };
    Apply3x3(settings::sensors::lsm9ds1::kMagAinv, magRaw, magRaw);
    ApplyAxisTransform(magRaw);
    ApplyMountRotation(magRaw);
    g_lastMagBody[0] = magRaw[0];
    g_lastMagBody[1] = magRaw[1];
    g_lastMagBody[2] = magRaw[2];
    const float magMagnitude = Magnitude3(magRaw[0], magRaw[1], magRaw[2]);
    float magNorm[3] = {magRaw[0], magRaw[1], magRaw[2]};
    Normalize3(magNorm[0], magNorm[1], magNorm[2]);

    float dt = 0.0f;
    if (g_lastFilterUs != 0) {
        dt = static_cast<float>(nowUs - g_lastFilterUs) * 1.0e-6f;
    }
    g_lastFilterUs = nowUs;
    if (!(dt > 0.0f) || dt > 0.1f) {
        dt = static_cast<float>(kSampleIntervalUs) * 1.0e-6f;
    }
    g_lastAhrsDt = dt;

    const float gyroNorm = Magnitude3(gyroRadPerSec[0], gyroRadPerSec[1], gyroRadPerSec[2]);
    const float localAccelTrust = ComputeAccelTrust(accelMagnitudeG, gyroNorm);
    const float localMagTrust = ComputeMagTrust(magMagnitude, gyroNorm);
    g_lastAccelTrust = localAccelTrust;
    g_lastMagTrust = localMagTrust;

    // Track accelerometer magnitude for gyro bias learning validation.
    g_lastAccelMagnitudeG = accelMagnitudeG;
    g_lastMagMagnitude = magMagnitude;
    UpdateGroundAlignment(accelNorm, localAccelTrust, magNorm, localMagTrust, gyroNorm);

    if (g_groundAlignmentReady) {
        AdaptiveQuaternionUpdate(accelNorm, localAccelTrust, gyroRadPerSec, magNorm, localMagTrust, dt);
    }
    LearnGyroBias(gyroRadPerSec, localAccelTrust, gyroNorm);
    UpdateMagReference(magMagnitude, localMagTrust);
    const bool quaternionValidNow = g_groundAlignmentReady && math_utils::ValidateQuaternionArray(g_q);
    if (quaternionValidNow) {
        g_invalidQuaternionStreak = 0;
        QuaternionToEulerDeg(g_q, g_lastYprDeg);
        for (int i = 0; i < 4; ++i) {
            g_lastQuaternionOut[i] = g_q[i];
        }
        g_haveQuaternion = true;
    } else {
        if (g_haveQuaternion && g_invalidQuaternionStreak < 0xff) {
            ++g_invalidQuaternionStreak;
        }
        if (!g_haveQuaternion || g_invalidQuaternionStreak >= kQuaternionInvalidDropThreshold) {
            g_haveQuaternion = false;
        }
    }

    g_lastAccel[0] = accelRaw[0] / g_activeAccelLsbPerG * kGToMps2;
    g_lastAccel[1] = accelRaw[1] / g_activeAccelLsbPerG * kGToMps2;
    g_lastAccel[2] = accelRaw[2] / g_activeAccelLsbPerG * kGToMps2;
    g_lastGyro[0] = gyroRadPerSec[0];
    g_lastGyro[1] = gyroRadPerSec[1];
    g_lastGyro[2] = gyroRadPerSec[2];
    g_hasCachedSample = true;

    PublishFromState(out, nowUs);
    return true;
}

Lsm9ds1Diagnostics Lsm9ds1SensorGetDiagnostics() {
    Lsm9ds1Diagnostics diagnostics;
    diagnostics.initialized = g_initialized;
    diagnostics.hasAccel = g_haveAccel;
    diagnostics.hasGyro = g_haveGyro;
    diagnostics.hasMag = g_haveMag;
    diagnostics.hasQuaternion = g_haveQuaternion;
    diagnostics.alignmentReady = g_groundAlignmentReady;
    diagnostics.lastAcquireFresh = g_lastAcquireFresh;
    diagnostics.lastAcquireUsedCache = g_lastAcquireUsedCache;
    diagnostics.interruptConfigured = g_interruptConfigured;
    diagnostics.lastAcquireUsedInterrupt = g_lastAcquireUsedInterrupt;
    diagnostics.fifoEnabled = g_fifoEnabled;
    diagnostics.lastSampleMicros = g_lastSampleUs;
    diagnostics.groundAlignmentSampleCount = g_groundAlignmentSampleCount;
    diagnostics.lastAccelTrust = g_lastAccelTrust;
    diagnostics.lastMagTrust = g_lastMagTrust;
    diagnostics.lastAccelMagnitudeG = g_lastAccelMagnitudeG;
    diagnostics.lastMagMagnitude = g_lastMagMagnitude;
    diagnostics.magReferenceNorm = g_magReferenceNorm;
    diagnostics.accelBodyMps2[0] = g_lastAccel[0];
    diagnostics.accelBodyMps2[1] = g_lastAccel[1];
    diagnostics.accelBodyMps2[2] = g_lastAccel[2];
    diagnostics.magBody[0] = g_lastMagBody[0];
    diagnostics.magBody[1] = g_lastMagBody[1];
    diagnostics.magBody[2] = g_lastMagBody[2];
    if (diagnostics.hasAccel && diagnostics.hasMag) {
        float accelNorm[3] = {g_lastAccel[0], g_lastAccel[1], g_lastAccel[2]};
        float magNorm[3] = {g_lastMagBody[0], g_lastMagBody[1], g_lastMagBody[2]};
        float bootstrapQuat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        if (QuaternionFromAccelMag(accelNorm, magNorm, bootstrapQuat)) {
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
        const math_utils::Quaternion quat = math_utils::MakeQuaternion(g_lastQuaternionOut[0],
                                                                       g_lastQuaternionOut[1],
                                                                       g_lastQuaternionOut[2],
                                                                       g_lastQuaternionOut[3]);
        math_utils::QuaternionToEuler(quat, yaw, pitch, roll);
        diagnostics.yprDeg[0] = yaw * 57.295779513082320876f;
        diagnostics.yprDeg[1] = pitch * 57.295779513082320876f;
        diagnostics.yprDeg[2] = roll * 57.295779513082320876f;
    }
    return diagnostics;
}
