#include "icm20948_sensor.h"

#include <Arduino.h>
#include <SPI.h>

#include <ICM_20948.h>

#include "settings.h"

namespace {

constexpr uint32_t kSampleIntervalUs = 10000;
constexpr float kAccelLsbPerG = 16384.0f;
constexpr float kGToMps2 = 9.80665f;
constexpr float kRadToDeg = 57.295779513082320876f;

ICM_20948_SPI g_icm;
bool g_initialized = false;
uint32_t g_lastSampleUs = 0;
uint32_t g_lastFilterUs = 0;

float g_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float g_integralError[3] = {0.0f, 0.0f, 0.0f};

inline float ClampUnit(float value) {
    if (value < -1.0f) {
        return -1.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
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

void Apply3x3(const float matrix[3][3], const float in[3], float out[3]) {
    out[0] = matrix[0][0] * in[0] + matrix[0][1] * in[1] + matrix[0][2] * in[2];
    out[1] = matrix[1][0] * in[0] + matrix[1][1] * in[1] + matrix[1][2] * in[2];
    out[2] = matrix[2][0] * in[0] + matrix[2][1] * in[1] + matrix[2][2] * in[2];
}

void GetScaledImu(float gyroRadPerSec[3], float accelNorm[3], float magNorm[3]) {
    gyroRadPerSec[0] = settings::sensors::icm20948::kGyroScaleRadPerSecPerLsb *
                       (static_cast<float>(g_icm.agmt.gyr.axes.x) - settings::sensors::icm20948::kGyroOffset[0]);
    gyroRadPerSec[1] = settings::sensors::icm20948::kGyroScaleRadPerSecPerLsb *
                       (static_cast<float>(g_icm.agmt.gyr.axes.y) - settings::sensors::icm20948::kGyroOffset[1]);
    gyroRadPerSec[2] = settings::sensors::icm20948::kGyroScaleRadPerSecPerLsb *
                       (static_cast<float>(g_icm.agmt.gyr.axes.z) - settings::sensors::icm20948::kGyroOffset[2]);

    float rawAccel[3] = {
        static_cast<float>(g_icm.agmt.acc.axes.x) - settings::sensors::icm20948::kAccelBias[0],
        static_cast<float>(g_icm.agmt.acc.axes.y) - settings::sensors::icm20948::kAccelBias[1],
        static_cast<float>(g_icm.agmt.acc.axes.z) - settings::sensors::icm20948::kAccelBias[2],
    };
    Apply3x3(settings::sensors::icm20948::kAccelAinv, rawAccel, accelNorm);
    Normalize3(accelNorm[0], accelNorm[1], accelNorm[2]);

    float rawMag[3] = {
        static_cast<float>(g_icm.agmt.mag.axes.x) - settings::sensors::icm20948::kMagBias[0],
        static_cast<float>(g_icm.agmt.mag.axes.y) - settings::sensors::icm20948::kMagBias[1],
        static_cast<float>(g_icm.agmt.mag.axes.z) - settings::sensors::icm20948::kMagBias[2],
    };
    Apply3x3(settings::sensors::icm20948::kMagAinv, rawMag, magNorm);
    Normalize3(magNorm[0], magNorm[1], magNorm[2]);
}

void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz,
                            float dt) {
    if (dt <= 0.0f) {
        return;
    }
    if (!Normalize3(ax, ay, az)) {
        return;
    }
    if (!Normalize3(mx, my, mz)) {
        return;
    }

    const float q1 = g_q[0];
    const float q2 = g_q[1];
    const float q3 = g_q[2];
    const float q4 = g_q[3];

    const float q1q1 = q1 * q1;
    const float q1q2 = q1 * q2;
    const float q1q3 = q1 * q3;
    const float q1q4 = q1 * q4;
    const float q2q2 = q2 * q2;
    const float q2q3 = q2 * q3;
    const float q2q4 = q2 * q4;
    const float q3q3 = q3 * q3;
    const float q3q4 = q3 * q4;
    const float q4q4 = q4 * q4;

    float hx = ay * mz - az * my;
    float hy = az * mx - ax * mz;
    float hz = ax * my - ay * mx;
    if (!Normalize3(hx, hy, hz)) {
        return;
    }

    const float ux = 2.0f * (q2q4 - q1q3);
    const float uy = 2.0f * (q1q2 + q3q4);
    const float uz = q1q1 - q2q2 - q3q3 + q4q4;

    const float wx = 2.0f * (q2q3 + q1q4);
    const float wy = q1q1 - q2q2 + q3q3 - q4q4;
    const float wz = 2.0f * (q3q4 - q1q2);

    const float ex = (ay * uz - az * uy) + (hy * wz - hz * wy);
    const float ey = (az * ux - ax * uz) + (hz * wx - hx * wz);
    const float ez = (ax * uy - ay * ux) + (hx * wy - hy * wx);

    if (settings::sensors::icm20948::kMahonyKi > 0.0f) {
        g_integralError[0] += ex * dt;
        g_integralError[1] += ey * dt;
        g_integralError[2] += ez * dt;
        gx += settings::sensors::icm20948::kMahonyKi * g_integralError[0];
        gy += settings::sensors::icm20948::kMahonyKi * g_integralError[1];
        gz += settings::sensors::icm20948::kMahonyKi * g_integralError[2];
    }

    gx += settings::sensors::icm20948::kMahonyKp * ex;
    gy += settings::sensors::icm20948::kMahonyKp * ey;
    gz += settings::sensors::icm20948::kMahonyKp * ez;

    const float halfDt = 0.5f * dt;
    const float scaledGx = gx * halfDt;
    const float scaledGy = gy * halfDt;
    const float scaledGz = gz * halfDt;

    float nq1 = q1 + (-q2 * scaledGx - q3 * scaledGy - q4 * scaledGz);
    float nq2 = q2 + (q1 * scaledGx + q3 * scaledGz - q4 * scaledGy);
    float nq3 = q3 + (q1 * scaledGy - q2 * scaledGz + q4 * scaledGx);
    float nq4 = q4 + (q1 * scaledGz + q2 * scaledGy - q3 * scaledGx);

    const float norm = sqrtf(nq1 * nq1 + nq2 * nq2 + nq3 * nq3 + nq4 * nq4);
    if (norm <= 1.0e-9f) {
        return;
    }
    const float inv = 1.0f / norm;
    g_q[0] = nq1 * inv;
    g_q[1] = nq2 * inv;
    g_q[2] = nq3 * inv;
    g_q[3] = nq4 * inv;
}

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

}  // namespace

bool Icm20948SensorBegin() {
    if (g_initialized) {
        return true;
    }

    SPI1.begin();
    g_icm.begin(25, SPI1);
    if (g_icm.status != ICM_20948_Stat_Ok) {
        return false;
    }

    g_lastSampleUs = 0;
    g_lastFilterUs = 0;
    g_q[0] = 1.0f;
    g_q[1] = 0.0f;
    g_q[2] = 0.0f;
    g_q[3] = 0.0f;
    g_integralError[0] = 0.0f;
    g_integralError[1] = 0.0f;
    g_integralError[2] = 0.0f;
    g_initialized = true;
    return true;
}

bool Icm20948SensorAcquire(SensorData &out) {
    if (!g_initialized) {
        return false;
    }

    const uint32_t nowUs = micros();
    if (g_lastSampleUs != 0 && (nowUs - g_lastSampleUs) < kSampleIntervalUs) {
        return false;
    }

    if (!g_icm.dataReady()) {
        return false;
    }

    g_lastSampleUs = nowUs;
    g_icm.getAGMT();

    if (out.timestamp == 0.0f) {
        out.timestamp = static_cast<float>(nowUs) * 1.0e-6f;
    }

    const float rawAx = static_cast<float>(g_icm.agmt.acc.axes.x);
    const float rawAy = static_cast<float>(g_icm.agmt.acc.axes.y);
    const float rawAz = static_cast<float>(g_icm.agmt.acc.axes.z);
    float gyroCal[3] = {0.0f, 0.0f, 0.0f};
    float accelCalNorm[3] = {0.0f, 0.0f, 0.0f};
    float magCalNorm[3] = {0.0f, 0.0f, 0.0f};
    GetScaledImu(gyroCal, accelCalNorm, magCalNorm);

    const float accelX = (rawAx / kAccelLsbPerG) * kGToMps2;
    const float accelY = (rawAy / kAccelLsbPerG) * kGToMps2;
    const float accelZ = (rawAz / kAccelLsbPerG) * kGToMps2;

    out.accelICM[0] = accelX;
    out.accelICM[1] = accelY;
    out.accelICM[2] = accelZ;
    out.gyro[0] = gyroCal[0];
    out.gyro[1] = gyroCal[1];
    out.gyro[2] = gyroCal[2];

    // Match reference implementation axis reconciliation.
    magCalNorm[1] = -magCalNorm[1];
    magCalNorm[2] = -magCalNorm[2];

    float dt = settings::flight::kDefaultDtSeconds;
    if (g_lastFilterUs != 0) {
        dt = static_cast<float>(nowUs - g_lastFilterUs) * 1.0e-6f;
        if (dt <= 0.0f || dt > 0.2f) {
            dt = settings::flight::kDefaultDtSeconds;
        }
    }
    g_lastFilterUs = nowUs;

    MahonyQuaternionUpdate(accelCalNorm[0],
                           accelCalNorm[1],
                           accelCalNorm[2],
                           gyroCal[0],
                           gyroCal[1],
                           gyroCal[2],
                           magCalNorm[0],
                           magCalNorm[1],
                           magCalNorm[2],
                           dt);
    out.icmQuaternion[0] = g_q[0];
    out.icmQuaternion[1] = g_q[1];
    out.icmQuaternion[2] = g_q[2];
    out.icmQuaternion[3] = g_q[3];
    out.hasIcmQuaternion = true;

    QuaternionToYprDeg(out.icmYprDeg[0], out.icmYprDeg[1], out.icmYprDeg[2]);
    out.hasIcmYpr = true;

    return true;
}
