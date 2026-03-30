#include "lsm9ds1_sensor.h"

#include <Arduino.h>
#include <SPI.h>

#include <SparkFunLSM9DS1.h>

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

LSM9DS1 g_lsm;
volatile bool g_dataReadyInterrupt = false;
bool g_initialized = false;
bool g_hasCachedSample = false;
uint32_t g_lastSampleUs = 0;
uint32_t g_lastFilterUs = 0;
FlightStatus g_flightStatus = FlightStatus::Ground;

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
    out[0] = matrix[0][0] * in[0] + matrix[0][1] * in[1] + matrix[0][2] * in[2];
    out[1] = matrix[1][0] * in[0] + matrix[1][1] * in[1] + matrix[1][2] * in[2];
    out[2] = matrix[2][0] * in[0] + matrix[2][1] * in[1] + matrix[2][2] * in[2];
}

void ApplyMountRotation(float vector[3]) {
    float rotated[3] = {0.0f, 0.0f, 0.0f};
    Apply3x3(settings::sensors::lsm9ds1::kMountRotation, vector, rotated);
    vector[0] = rotated[0];
    vector[1] = rotated[1];
    vector[2] = rotated[2];
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

bool InitializeQuaternionFromAccelMag(const float accelNorm[3], const float magNorm[3]) {
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
                                             settings::sensors::lsm9ds1::kAccelCorrectionMinG,
                                             settings::sensors::lsm9ds1::kAccelCorrectionMaxG);
    const float rateTrust = DescendingTrust(gyroNorm,
                                            settings::sensors::lsm9ds1::kAccelCorrectionGyroFadeStartRadPerSec,
                                            settings::sensors::lsm9ds1::kAccelCorrectionGyroFadeEndRadPerSec);
    return g_crossCheckTrust * phaseTrust * magnitudeTrust * rateTrust;
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
        return g_crossCheckTrust * phaseTrust * DescendingTrust(gyroNorm,
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
    return g_crossCheckTrust * phaseTrust * magnitudeTrust * rateTrust;
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

float AccelCorrectionGain() {
    switch (g_flightStatus) {
        case FlightStatus::Ground:
            return settings::sensors::lsm9ds1::kAccelCorrectionGainGround;
        case FlightStatus::Descent:
            return settings::sensors::lsm9ds1::kAccelCorrectionGainDescent;
        case FlightStatus::Burn:
        case FlightStatus::Coast:
        case FlightStatus::Overshoot:
            return 0.0f;
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

bool ShouldLearnGyroBias(float accelTrust, float gyroNorm) {
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

void AdaptiveQuaternionUpdate(const float accelNorm[3],
                              float accelTrust,
                              const float gyroRadPerSec[3],
                              const float magNorm[3],
                              float magTrust,
                              float dt) {
    if (dt <= 0.0f) {
        return;
    }

    const float q1 = g_q[0];
    const float q2 = g_q[1];
    const float q3 = g_q[2];
    const float q4 = g_q[3];

    const float ux = 2.0f * (q2 * q4 - q1 * q3);
    const float uy = 2.0f * (q1 * q2 + q3 * q4);
    const float uz = q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4;

    float hx = accelNorm[1] * magNorm[2] - accelNorm[2] * magNorm[1];
    float hy = accelNorm[2] * magNorm[0] - accelNorm[0] * magNorm[2];
    float hz = accelNorm[0] * magNorm[1] - accelNorm[1] * magNorm[0];
    if (!Normalize3(hx, hy, hz)) {
        magTrust = 0.0f;
        hx = hy = hz = 0.0f;
    }

    const float wx = 2.0f * (q2 * q3 + q1 * q4);
    const float wy = q1 * q1 - q2 * q2 + q3 * q3 - q4 * q4;
    const float wz = 2.0f * (q3 * q4 - q1 * q2);

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

    const float invNorm = 1.0f / sqrtf(g_q[0] * g_q[0] + g_q[1] * g_q[1] + g_q[2] * g_q[2] + g_q[3] * g_q[3]);
    g_q[0] *= invNorm;
    g_q[1] *= invNorm;
    g_q[2] *= invNorm;
    g_q[3] *= invNorm;
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
    out.hasIcmQuaternion = g_groundAlignmentReady;
    out.hasIcmYpr = g_groundAlignmentReady;
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
    SPI.begin();
    if (g_lsm.beginSPI(kAccelGyroChipSelectPin, kMagChipSelectPin) == 0) {
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
    const float accelTrust = ComputeAccelTrust(accelMagnitudeG, gyroNorm);
    const float magTrust = ComputeMagTrust(magMagnitude, gyroNorm);
    g_lastAccelTrust = accelTrust;
    g_lastMagTrust = magTrust;
    UpdateGroundAlignment(accelNorm, accelTrust, magNorm, magTrust, gyroNorm);

    if (g_groundAlignmentReady) {
        AdaptiveQuaternionUpdate(accelNorm, accelTrust, gyroRadPerSec, magNorm, magTrust, dt);
    }
    LearnGyroBias(gyroRadPerSec, accelTrust, gyroNorm);
    UpdateMagReference(magMagnitude, magTrust);
    QuaternionToEulerDeg(g_q, g_lastYprDeg);

    g_lastAccel[0] = accelRaw[0] / g_activeAccelLsbPerG * kGToMps2;
    g_lastAccel[1] = accelRaw[1] / g_activeAccelLsbPerG * kGToMps2;
    g_lastAccel[2] = accelRaw[2] / g_activeAccelLsbPerG * kGToMps2;
    g_lastGyro[0] = gyroRadPerSec[0];
    g_lastGyro[1] = gyroRadPerSec[1];
    g_lastGyro[2] = gyroRadPerSec[2];
    for (int i = 0; i < 4; ++i) {
        g_lastQuaternionOut[i] = g_q[i];
    }
    g_haveQuaternion = g_groundAlignmentReady;
    g_hasCachedSample = true;

    PublishFromState(out, nowUs);
    return true;
}

Lsm9ds1Diagnostics Lsm9ds1SensorGetDiagnostics() {
    Lsm9ds1Diagnostics diagnostics;
    diagnostics.initialized = g_initialized;
    diagnostics.hasAccel = g_haveAccel;
    diagnostics.hasGyro = g_haveGyro;
    diagnostics.hasQuaternion = g_haveQuaternion;
    diagnostics.alignmentReady = g_groundAlignmentReady;
    diagnostics.lastAcquireFresh = g_lastAcquireFresh;
    diagnostics.lastAcquireUsedCache = g_lastAcquireUsedCache;
    diagnostics.interruptConfigured = g_interruptConfigured;
    diagnostics.lastAcquireUsedInterrupt = g_lastAcquireUsedInterrupt;
    diagnostics.fifoEnabled = g_fifoEnabled;
    diagnostics.lastSampleMicros = g_lastSampleUs;
    return diagnostics;
}
