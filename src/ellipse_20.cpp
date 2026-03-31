#include "ellipse_20.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <math.h>

extern "C" {
#include <interfaces/sbgInterface.h>
#include <sbgCommon.h>
#include <sbgECom.h>
#include <sbgEComLib.h>
}

#include "math_utils.h"
#include "serial_logging.h"
#include "settings.h"

namespace {

constexpr bool kPulseEnabled = settings::sensors::ellipse20::kEnabled;
constexpr uint8_t kSerialPortIndex = settings::sensors::ellipse20::kSerialPortIndex;
constexpr int8_t kRxPin = settings::sensors::ellipse20::kRxPin;
constexpr int8_t kTxPin = settings::sensors::ellipse20::kTxPin;
constexpr uint32_t kBaudRate = settings::sensors::ellipse20::kBaudRate;
constexpr uint8_t kHandleBudgetPerAcquire = settings::sensors::ellipse20::kHandleBudgetPerAcquire;
constexpr uint32_t kSampleMaxAgeUs = settings::sensors::ellipse20::kSampleMaxAgeUs;
constexpr SbgEComOutputMode kImuOutputMode =
    static_cast<SbgEComOutputMode>(settings::sensors::ellipse20::kImuOutputMode);
constexpr SbgEComOutputMode kMagOutputMode =
    static_cast<SbgEComOutputMode>(settings::sensors::ellipse20::kMagOutputMode);
constexpr float kGToMps2 = 9.80665f;
constexpr float kRadToDeg = 57.295779513082320876f;
constexpr float kDefaultDtSeconds = 0.005f;

struct SerialInterfaceContext {
    HardwareSerial *serial = nullptr;
    uint32_t baudRate = 0;
};

SerialInterfaceContext g_serialContext;
SbgInterface g_interface;
SbgEComHandle g_comHandle;

bool g_initialized = false;
bool g_haveImu = false;
bool g_haveMag = false;
bool g_haveQuaternion = false;
bool g_haveYpr = false;
bool g_groundAlignmentReady = false;
bool g_hasCachedSample = false;
bool g_hasMagReference = false;
bool g_hasEarthMagReference = false;
bool g_hasQuaternionContinuityReference = false;
bool g_lastAcquireFresh = false;
bool g_lastAcquireUsedCache = false;

FlightStatus g_flightStatus = FlightStatus::Ground;
float g_crossCheckTrust = 1.0f;

uint32_t g_lastSampleMicros = 0;
uint32_t g_lastImuHostMicros = 0;
uint32_t g_lastMagHostMicros = 0;
uint32_t g_lastImuSensorTimestampUs = 0;
uint32_t g_lastProcessedImuSensorTimestampUs = 0;

float g_rawAccel[3] = {0.0f, 0.0f, 0.0f};
float g_rawGyro[3] = {0.0f, 0.0f, 0.0f};
float g_rawMag[3] = {0.0f, 0.0f, 0.0f};

float g_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float g_lastAccel[3] = {0.0f, 0.0f, 0.0f};
float g_lastGyro[3] = {0.0f, 0.0f, 0.0f};
float g_lastQuaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float g_lastYprDeg[3] = {0.0f, 0.0f, 0.0f};
float g_lastTemperatureC = 0.0f;
float g_lastAhrsDt = 0.0f;
float g_lastAccelTrust = 0.0f;
float g_lastMagTrust = 0.0f;
float g_gyroBiasLearned[3] = {0.0f, 0.0f, 0.0f};
float g_magReferenceNorm = 0.0f;
float g_magReferenceEarth[3] = {0.0f, 1.0f, 0.0f};
float g_lastContinuousQuaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};

float g_groundAlignmentAccelSum[3] = {0.0f, 0.0f, 0.0f};
float g_groundAlignmentMagSum[3] = {0.0f, 0.0f, 0.0f};
uint16_t g_groundAlignmentSampleCount = 0;

inline float Clamp01(float value) {
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
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

void ApplyMountRotation(float vector[3]) {
    float rotated[3] = {0.0f, 0.0f, 0.0f};
    Apply3x3(settings::sensors::ellipse20::kMountRotation, vector, rotated);
    vector[0] = rotated[0];
    vector[1] = rotated[1];
    vector[2] = rotated[2];
}

float Magnitude3(float x, float y, float z) {
    return sqrtf(x * x + y * y + z * z);
}

bool Normalize3(float &x, float &y, float &z) {
    const float norm = Magnitude3(x, y, z);
    if (norm <= 1.0e-9f) {
        return false;
    }
    const float invNorm = 1.0f / norm;
    x *= invNorm;
    y *= invNorm;
    z *= invNorm;
    return true;
}

void Cross3(float ax, float ay, float az, float bx, float by, float bz, float out[3]) {
    out[0] = ay * bz - az * by;
    out[1] = az * bx - ax * bz;
    out[2] = ax * by - ay * bx;
}

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

void ApplyQuaternionContinuity() {
    if (!g_hasQuaternionContinuityReference) {
        for (int i = 0; i < 4; ++i) {
            g_lastContinuousQuaternion[i] = g_q[i];
        }
        g_hasQuaternionContinuityReference = true;
        return;
    }
    if (QuaternionDot(g_q, g_lastContinuousQuaternion) < 0.0f) {
        NegateQuaternion(g_q);
    }
    for (int i = 0; i < 4; ++i) {
        g_lastContinuousQuaternion[i] = g_q[i];
    }
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

    const float norm = Magnitude3(quaternion[0], quaternion[1], quaternion[2]);
    const float fullNorm = sqrtf(norm * norm + quaternion[3] * quaternion[3]);
    if (fullNorm <= 1.0e-9f) {
        return false;
    }
    const float invNorm = 1.0f / fullNorm;
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

    if (!QuaternionFromEarthBasisInBody(northBody, eastBody, upBody, g_q)) {
        return false;
    }
    ApplyQuaternionContinuity();
    g_groundAlignmentReady = true;
    g_haveQuaternion = true;
    g_haveYpr = true;
    return true;
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

float AccelCorrectionGain() {
    switch (g_flightStatus) {
        case FlightStatus::Ground:
            return settings::sensors::ellipse20::kAccelCorrectionGainGround;
        case FlightStatus::Descent:
            return settings::sensors::ellipse20::kAccelCorrectionGainDescent;
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
            return settings::sensors::ellipse20::kMagCorrectionGainGround;
        case FlightStatus::Descent:
            return settings::sensors::ellipse20::kMagCorrectionGainDescent;
        case FlightStatus::Burn:
        case FlightStatus::Coast:
        case FlightStatus::Overshoot:
            return settings::sensors::ellipse20::kMagCorrectionGainFlight;
    }
    return 0.0f;
}

float MagTrustPhaseScale() {
    switch (g_flightStatus) {
        case FlightStatus::Ground:
            return settings::sensors::ellipse20::kMagTrustGround;
        case FlightStatus::Burn:
            return settings::sensors::ellipse20::kMagTrustBurn;
        case FlightStatus::Coast:
            return settings::sensors::ellipse20::kMagTrustCoast;
        case FlightStatus::Overshoot:
            return settings::sensors::ellipse20::kMagTrustOvershoot;
        case FlightStatus::Descent:
            return settings::sensors::ellipse20::kMagTrustDescent;
    }
    return 0.0f;
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
                                             settings::sensors::ellipse20::kAccelCorrectionMinG,
                                             settings::sensors::ellipse20::kAccelCorrectionMaxG);
    const float rateTrust = DescendingTrust(gyroNorm,
                                            settings::sensors::ellipse20::kAccelCorrectionGyroFadeStartRadPerSec,
                                            settings::sensors::ellipse20::kAccelCorrectionGyroFadeEndRadPerSec);
    return g_crossCheckTrust * phaseTrust * magnitudeTrust * rateTrust;
}

float ComputeMagTrust(float magMagnitude, float gyroNorm) {
    if (!(magMagnitude > 0.0f) || !isfinite(magMagnitude)) {
        return 0.0f;
    }

    const float phaseTrust = MagTrustPhaseScale();
    if (phaseTrust <= 0.0f) {
        return 0.0f;
    }

    const float rateTrust = DescendingTrust(gyroNorm,
                                            settings::sensors::ellipse20::kMagTrustGyroFadeStartRadPerSec,
                                            settings::sensors::ellipse20::kMagTrustGyroFadeEndRadPerSec);

    if (!g_hasMagReference) {
        return g_crossCheckTrust * phaseTrust * rateTrust;
    }

    const float relativeError = fabsf(magMagnitude - g_magReferenceNorm) / g_magReferenceNorm;
    if (relativeError >= settings::sensors::ellipse20::kMagCorrectionMaxRelativeError) {
        return 0.0f;
    }

    const float magnitudeTrust =
        Clamp01(1.0f - relativeError / settings::sensors::ellipse20::kMagCorrectionMaxRelativeError);
    return g_crossCheckTrust * phaseTrust * magnitudeTrust * rateTrust;
}

bool ShouldLearnGyroBias(float accelTrust, float gyroNorm) {
    if (g_flightStatus == FlightStatus::Ground) {
        return accelTrust > 0.35f && gyroNorm <= settings::sensors::ellipse20::kStationaryGyroMaxRadPerSec;
    }
    if (g_flightStatus == FlightStatus::Descent) {
        return accelTrust > 0.65f &&
               gyroNorm <= 0.5f * settings::sensors::ellipse20::kStationaryGyroMaxRadPerSec;
    }
    return false;
}

void UpdateMagMagnitudeReference(float magMagnitude, float trust) {
    if (!(magMagnitude > 0.0f) || !isfinite(magMagnitude) || trust <= 0.0f) {
        return;
    }
    if (!g_hasMagReference) {
        g_magReferenceNorm = magMagnitude;
        g_hasMagReference = true;
        return;
    }
    const float blend = settings::sensors::ellipse20::kMagReferenceBlend * trust;
    g_magReferenceNorm += blend * (magMagnitude - g_magReferenceNorm);
}

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

    const float blend = settings::sensors::ellipse20::kMagReferenceBlend * referenceTrust;
    g_magReferenceEarth[0] += blend * (earthMag[0] - g_magReferenceEarth[0]);
    g_magReferenceEarth[1] += blend * (earthMag[1] - g_magReferenceEarth[1]);
    g_magReferenceEarth[2] += blend * (earthMag[2] - g_magReferenceEarth[2]);
    Normalize3(g_magReferenceEarth[0], g_magReferenceEarth[1], g_magReferenceEarth[2]);
}

bool ComputeBootstrapMagError(const float accelNorm[3], const float magNorm[3], float error[3]) {
    float hx = accelNorm[1] * magNorm[2] - accelNorm[2] * magNorm[1];
    float hy = accelNorm[2] * magNorm[0] - accelNorm[0] * magNorm[2];
    float hz = accelNorm[0] * magNorm[1] - accelNorm[1] * magNorm[0];
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

void UpdateGroundAlignment(const float accelNorm[3],
                           float accelTrust,
                           const float magNorm[3],
                           float magTrust,
                           float gyroNorm) {
    if (g_groundAlignmentReady || g_flightStatus != FlightStatus::Ground) {
        return;
    }
    if (accelTrust < settings::sensors::ellipse20::kGroundAlignmentAccelTrustMin ||
        magTrust < settings::sensors::ellipse20::kGroundAlignmentMagTrustMin ||
        gyroNorm > settings::sensors::ellipse20::kStationaryGyroMaxRadPerSec) {
        ResetGroundAlignment();
        return;
    }

    for (int i = 0; i < 3; ++i) {
        g_groundAlignmentAccelSum[i] += accelNorm[i];
        g_groundAlignmentMagSum[i] += magNorm[i];
    }
    ++g_groundAlignmentSampleCount;
    if (g_groundAlignmentSampleCount < settings::sensors::ellipse20::kGroundAlignmentMinSamples) {
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

void QuaternionToYprDeg(const float q[4], float yprDeg[3]) {
    const float q0 = q[0];
    const float q1 = q[1];
    const float q2 = q[2];
    const float q3 = q[3];

    float roll = atan2f((q0 * q1 + q2 * q3), 0.5f - (q1 * q1 + q2 * q2));
    float pitch = asinf(ClampUnit(2.0f * (q0 * q2 - q1 * q3)));
    float yaw = atan2f((q1 * q2 + q0 * q3), 0.5f - (q2 * q2 + q3 * q3));

    yaw *= kRadToDeg;
    pitch *= kRadToDeg;
    roll *= kRadToDeg;

    yaw = -(yaw + settings::sensors::ellipse20::kMagDeclinationDeg);
    while (yaw < 0.0f) {
        yaw += 360.0f;
    }
    while (yaw >= 360.0f) {
        yaw -= 360.0f;
    }

    yprDeg[0] = yaw;
    yprDeg[1] = pitch;
    yprDeg[2] = roll;
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
        LimitVector(accelFeedback, settings::sensors::ellipse20::kAccelCorrectionMaxRateRadPerSec);
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
            LimitVector(magFeedback, settings::sensors::ellipse20::kMagCorrectionMaxRateRadPerSec);
            for (int i = 0; i < 3; ++i) {
                feedback[i] += magFeedback[i];
            }
        }
    }

    const float gyroNorm = Magnitude3(gyroRadPerSec[0], gyroRadPerSec[1], gyroRadPerSec[2]);
    if (ShouldLearnGyroBias(accelTrust, gyroNorm)) {
        for (int i = 0; i < 3; ++i) {
            g_gyroBiasLearned[i] += settings::sensors::ellipse20::kGyroBiasLearningRate * learningError[i] * dt;
            g_gyroBiasLearned[i] =
                fmaxf(-settings::sensors::ellipse20::kGyroBiasMaxRadPerSec,
                      fminf(settings::sensors::ellipse20::kGyroBiasMaxRadPerSec, g_gyroBiasLearned[i]));
        }
    }

    LimitVector(feedback, settings::sensors::ellipse20::kTotalCorrectionMaxRateRadPerSec);

    float correctedGyro[3] = {
        gyroRadPerSec[0] - g_gyroBiasLearned[0] + feedback[0],
        gyroRadPerSec[1] - g_gyroBiasLearned[1] + feedback[1],
        gyroRadPerSec[2] - g_gyroBiasLearned[2] + feedback[2],
    };

    if (settings::ahrs::kEnableExponentialMap) {
        const math_utils::Quaternion qCurrent = math_utils::MakeQuaternion(g_q[0], g_q[1], g_q[2], g_q[3]);
        const math_utils::Quaternion qUpdated = math_utils::ExponentialMapUpdate(
            qCurrent, correctedGyro[0], correctedGyro[1], correctedGyro[2], dt);
        g_q[0] = qUpdated.w;
        g_q[1] = qUpdated.x;
        g_q[2] = qUpdated.y;
        g_q[3] = qUpdated.z;
    } else {
        const float halfDt = 0.5f * dt;
        const float gx = correctedGyro[0] * halfDt;
        const float gy = correctedGyro[1] * halfDt;
        const float gz = correctedGyro[2] * halfDt;

        const float qa = g_q[0];
        const float qb = g_q[1];
        const float qc = g_q[2];
        const float qd = g_q[3];

        g_q[0] += (-qb * gx - qc * gy - qd * gz);
        g_q[1] += (qa * gx + qc * gz - qd * gy);
        g_q[2] += (qa * gy - qb * gz + qd * gx);
        g_q[3] += (qa * gz + qb * gy - qc * gx);

        const float invNorm = 1.0f / sqrtf(g_q[0] * g_q[0] + g_q[1] * g_q[1] + g_q[2] * g_q[2] +
                                           g_q[3] * g_q[3]);
        g_q[0] *= invNorm;
        g_q[1] *= invNorm;
        g_q[2] *= invNorm;
        g_q[3] *= invNorm;
    }
    ApplyQuaternionContinuity();
}

void ResetCachedState() {
    g_haveImu = false;
    g_haveMag = false;
    g_haveQuaternion = false;
    g_haveYpr = false;
    g_hasCachedSample = false;
    g_hasMagReference = false;
    g_hasEarthMagReference = false;
    g_hasQuaternionContinuityReference = false;
    g_lastAcquireFresh = false;
    g_lastAcquireUsedCache = false;
    g_lastSampleMicros = 0;
    g_lastImuHostMicros = 0;
    g_lastMagHostMicros = 0;
    g_lastImuSensorTimestampUs = 0;
    g_lastProcessedImuSensorTimestampUs = 0;
    g_lastTemperatureC = 0.0f;
    g_lastAhrsDt = 0.0f;
    g_lastAccelTrust = 0.0f;
    g_lastMagTrust = 0.0f;
    g_magReferenceNorm = 0.0f;
    g_magReferenceEarth[0] = 0.0f;
    g_magReferenceEarth[1] = 1.0f;
    g_magReferenceEarth[2] = 0.0f;
    g_q[0] = 1.0f;
    g_q[1] = 0.0f;
    g_q[2] = 0.0f;
    g_q[3] = 0.0f;
    g_lastQuaternion[0] = 1.0f;
    g_lastQuaternion[1] = 0.0f;
    g_lastQuaternion[2] = 0.0f;
    g_lastQuaternion[3] = 0.0f;
    for (int i = 0; i < 3; ++i) {
        g_rawAccel[i] = 0.0f;
        g_rawGyro[i] = 0.0f;
        g_rawMag[i] = 0.0f;
        g_lastAccel[i] = 0.0f;
        g_lastGyro[i] = 0.0f;
        g_lastYprDeg[i] = 0.0f;
        g_gyroBiasLearned[i] = 0.0f;
        g_lastContinuousQuaternion[i] = 0.0f;
    }
    g_lastContinuousQuaternion[0] = 1.0f;
    ResetGroundAlignment();
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

void PublishFromState(SensorData &out, uint32_t nowUs) {
    out.timestamp = static_cast<float>(nowUs) * 1.0e-6f;
    for (int i = 0; i < 3; ++i) {
        out.accelPulse[i] = g_lastAccel[i];
        out.gyroPulse[i] = g_lastGyro[i];
        out.pulseYprDeg[i] = g_lastYprDeg[i];
    }
    for (int i = 0; i < 4; ++i) {
        out.quaternionPulse[i] = g_lastQuaternion[i];
    }
    out.hasPulseQuaternion = g_haveQuaternion;
    out.hasPulseYpr = g_haveYpr;
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
        LOG_PRINTLN("Pulse20: invalid serial port index");
        return false;
    }

    ConfigureSerial(kBaudRate);
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
    sbgInterfaceNameSet(&g_interface, "Pulse20");
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
                g_rawAccel[i] = pLogData->imuData.accelerometers[i];
                g_rawGyro[i] = pLogData->imuData.gyroscopes[i];
            }
            g_lastTemperatureC = pLogData->imuData.temperature;
            g_lastImuSensorTimestampUs = pLogData->imuData.timeStamp;
            g_lastImuHostMicros = nowUs;
            g_haveImu = true;
            break;

        case SBG_ECOM_LOG_MAG:
            for (int i = 0; i < 3; ++i) {
                g_rawMag[i] = pLogData->magData.magnetometers[i];
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

}  // namespace

bool Ellipse20SensorBegin() {
    if (!kPulseEnabled) {
        return false;
    }
    if (g_initialized) {
        return true;
    }

    ResetCachedState();
    if (!SetupInterface()) {
        return false;
    }
    if (sbgEComInit(&g_comHandle, &g_interface) != SBG_NO_ERROR) {
        LOG_PRINTLN("Pulse20: sbgECom init failed");
        return false;
    }
    sbgEComSetCmdTrialsAndTimeOut(&g_comHandle, 2u, 150u);
    sbgEComSetReceiveLogCallback(&g_comHandle, &OnLogReceived, nullptr);
    if (!ConfigureOutputs()) {
        LOG_PRINTLN("Pulse20: output configuration warning");
    }
    g_initialized = true;
    LOG_PRINTLN("Pulse20: raw IMU rail startup complete");
    return true;
}

void Ellipse20SensorSetFlightStatus(FlightStatus status) {
    g_flightStatus = status;
}

void Ellipse20SensorSetCrossCheckTrust(float trust) {
    g_crossCheckTrust = Clamp01(trust);
}

bool Ellipse20SensorAcquire(SensorData &out) {
    if (!kPulseEnabled || !g_initialized) {
        g_lastAcquireFresh = false;
        g_lastAcquireUsedCache = false;
        return false;
    }

    bool handledFrame = false;
    for (uint8_t i = 0; i < kHandleBudgetPerAcquire; ++i) {
        const SbgErrorCode errorCode = sbgEComHandleOneLog(&g_comHandle);
        if (errorCode == SBG_NOT_READY) {
            break;
        }
        if (errorCode != SBG_NO_ERROR) {
            break;
        }
        handledFrame = true;
    }

    const uint32_t nowUs = micros();
    const bool imuRecent =
        g_haveImu && g_lastImuHostMicros != 0u && static_cast<uint32_t>(nowUs - g_lastImuHostMicros) <= kSampleMaxAgeUs;
    const bool magRecent =
        g_haveMag && g_lastMagHostMicros != 0u && static_cast<uint32_t>(nowUs - g_lastMagHostMicros) <= kSampleMaxAgeUs;
    const bool freshImu = imuRecent && g_lastImuSensorTimestampUs != g_lastProcessedImuSensorTimestampUs;

    if (!freshImu) {
        g_lastAcquireFresh = false;
        if (!g_hasCachedSample || !imuRecent) {
            g_lastAcquireUsedCache = false;
            return false;
        }
        g_lastAcquireUsedCache = true;
        PublishFromState(out, nowUs);
        return true;
    }

    float dt = kDefaultDtSeconds;
    if (g_lastProcessedImuSensorTimestampUs != 0u) {
        const uint32_t deltaUs = g_lastImuSensorTimestampUs - g_lastProcessedImuSensorTimestampUs;
        if (deltaUs > 0u && deltaUs <= 100000u) {
            dt = static_cast<float>(deltaUs) * 1.0e-6f;
        }
    }

    float gyroRadPerSec[3] = {
        g_rawGyro[0] - settings::sensors::ellipse20::kGyroOffset[0] -
            settings::sensors::ellipse20::kGyroTempBiasSlopeRadPerSecPerC[0] *
                (g_lastTemperatureC - settings::sensors::ellipse20::kGyroReferenceTemperatureC),
        g_rawGyro[1] - settings::sensors::ellipse20::kGyroOffset[1] -
            settings::sensors::ellipse20::kGyroTempBiasSlopeRadPerSecPerC[1] *
                (g_lastTemperatureC - settings::sensors::ellipse20::kGyroReferenceTemperatureC),
        g_rawGyro[2] - settings::sensors::ellipse20::kGyroOffset[2] -
            settings::sensors::ellipse20::kGyroTempBiasSlopeRadPerSecPerC[2] *
                (g_lastTemperatureC - settings::sensors::ellipse20::kGyroReferenceTemperatureC),
    };
    ApplyMountRotation(gyroRadPerSec);

    float accelBody[3] = {
        g_rawAccel[0] - settings::sensors::ellipse20::kAccelBias[0],
        g_rawAccel[1] - settings::sensors::ellipse20::kAccelBias[1],
        g_rawAccel[2] - settings::sensors::ellipse20::kAccelBias[2],
    };
    Apply3x3(settings::sensors::ellipse20::kAccelAinv, accelBody, accelBody);
    ApplyMountRotation(accelBody);
    const float accelMagnitudeG = Magnitude3(accelBody[0], accelBody[1], accelBody[2]) / kGToMps2;
    float accelNorm[3] = {accelBody[0], accelBody[1], accelBody[2]};
    Normalize3(accelNorm[0], accelNorm[1], accelNorm[2]);

    float magBody[3] = {
        g_rawMag[0] - settings::sensors::ellipse20::kMagBias[0],
        g_rawMag[1] - settings::sensors::ellipse20::kMagBias[1],
        g_rawMag[2] - settings::sensors::ellipse20::kMagBias[2],
    };
    Apply3x3(settings::sensors::ellipse20::kMagAinv, magBody, magBody);
    ApplyMountRotation(magBody);
    const float magMagnitude = Magnitude3(magBody[0], magBody[1], magBody[2]);
    float magNorm[3] = {magBody[0], magBody[1], magBody[2]};
    Normalize3(magNorm[0], magNorm[1], magNorm[2]);

    const float gyroNorm = Magnitude3(gyroRadPerSec[0], gyroRadPerSec[1], gyroRadPerSec[2]);
    const float accelTrust = ComputeAccelTrust(accelMagnitudeG, gyroNorm);
    const float magTrust = magRecent ? ComputeMagTrust(magMagnitude, gyroNorm) : 0.0f;

    g_lastAhrsDt = dt;
    g_lastAccelTrust = accelTrust;
    g_lastMagTrust = magTrust;

    if (magRecent) {
        UpdateGroundAlignment(accelNorm, accelTrust, magNorm, magTrust, gyroNorm);
    }

    if (g_groundAlignmentReady) {
        if (magRecent) {
            UpdateMagMagnitudeReference(magMagnitude, magTrust);
            UpdateEarthMagReference(magNorm, accelTrust, magTrust);
        }
        AdaptiveQuaternionUpdate(accelNorm, accelTrust, gyroRadPerSec, magNorm, magTrust, dt);
        QuaternionToYprDeg(g_q, g_lastYprDeg);
        for (int i = 0; i < 4; ++i) {
            g_lastQuaternion[i] = g_q[i];
        }
        g_haveQuaternion = true;
        g_haveYpr = true;
    }

    for (int i = 0; i < 3; ++i) {
        g_lastAccel[i] = accelBody[i];
        g_lastGyro[i] = gyroRadPerSec[i];
    }

    g_lastSampleMicros = nowUs;
    g_lastProcessedImuSensorTimestampUs = g_lastImuSensorTimestampUs;
    g_lastAcquireFresh = handledFrame;
    g_lastAcquireUsedCache = false;
    g_hasCachedSample = true;

    PublishFromState(out, nowUs);
    return true;
}

bool Ellipse20SensorIsInitialized() {
    return kPulseEnabled && g_initialized;
}

Ellipse20Diagnostics Ellipse20SensorGetDiagnostics() {
    Ellipse20Diagnostics diagnostics;
    diagnostics.initialized = kPulseEnabled && g_initialized;
    diagnostics.hasImu = kPulseEnabled && g_haveImu;
    diagnostics.hasMag = kPulseEnabled && g_haveMag;
    diagnostics.hasQuaternion = kPulseEnabled && g_haveQuaternion;
    diagnostics.alignmentReady = kPulseEnabled && g_groundAlignmentReady;
    diagnostics.hasYpr = kPulseEnabled && g_haveYpr;
    diagnostics.lastAcquireFresh = kPulseEnabled && g_lastAcquireFresh;
    diagnostics.lastAcquireUsedCache = kPulseEnabled && g_lastAcquireUsedCache;
    diagnostics.lastSampleMicros = kPulseEnabled ? g_lastSampleMicros : 0u;
    return diagnostics;
}
