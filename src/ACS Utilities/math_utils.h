#pragma once

#include <cmath>
#include <cstdint>

#if __has_include(<arm_math.h>)
#ifndef ARM_MATH_CM7
#define ARM_MATH_CM7
#endif
#include <arm_math.h>
#define MATHUTILS_HAVE_ARM_MATH 1
#else
#define MATHUTILS_HAVE_ARM_MATH 0
#endif

/// Minimal vector and quaternion helpers mirrored from the Python analysis tools.
///
/// These functions stay header-only because they are used heavily in estimator,
/// replay, and predictor paths where small math helpers should inline cleanly.
namespace math_utils {

/*
 * Quaternion convention in this codebase:
 *
 *   q = [w, x, y, z]
 *
 * Quaternions represent attitude without the singularity problems of Euler
 * angles. They still need care: they should stay unit length, q and -q mean the
 * same physical orientation, and interpolation should follow the shortest arc.
 * Most helpers below either preserve those properties or repair small drift.
 */

struct Vec3 {
    float x;
    float y;
    float z;
};

struct Vec3d {
    double x;
    double y;
    double z;
};

struct Quaternion {
    float w;
    float x;
    float y;
    float z;
};

struct Quaterniond {
    double w;
    double x;
    double y;
    double z;
};

inline Vec3 MakeVec3(float x, float y, float z) { return Vec3{x, y, z}; }
inline Vec3d MakeVec3d(double x, double y, double z) { return Vec3d{x, y, z}; }

inline void FastSinCos(float angleRadians, float &sineOut, float &cosineOut) {
#if MATHUTILS_HAVE_ARM_MATH
    // CMSIS expects degrees for arm_sin_cos_f32, while the rest of the code
    // stores attitude in radians.
    constexpr float kRadToDeg = 57.295779513082320876f;
    arm_sin_cos_f32(angleRadians * kRadToDeg, &sineOut, &cosineOut);
#else
    sineOut = sinf(angleRadians);
    cosineOut = cosf(angleRadians);
#endif
}

inline float FastAtan2(float y, float x) { return atan2f(y, x); }
inline double FastAtan2(double y, double x) { return atan2(y, x); }

inline float FastSqrt(float value) {
#if MATHUTILS_HAVE_ARM_MATH
    float result;
    arm_sqrt_f32(value, &result);
    return result;
#else
    return sqrtf(value);
#endif
}

inline double FastSqrt(double value) { return sqrt(value); }

inline Vec3 Add(const Vec3 &a, const Vec3 &b) {
    return Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Vec3d Add(const Vec3d &a, const Vec3d &b) {
    return Vec3d{a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Vec3 Subtract(const Vec3 &a, const Vec3 &b) {
    return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
}

inline Vec3d Subtract(const Vec3d &a, const Vec3d &b) {
    return Vec3d{a.x - b.x, a.y - b.y, a.z - b.z};
}

inline Vec3 Scale(const Vec3 &v, float s) {
    return Vec3{v.x * s, v.y * s, v.z * s};
}

inline Vec3d Scale(const Vec3d &v, double s) {
    return Vec3d{v.x * s, v.y * s, v.z * s};
}

inline float Dot(const Vec3 &a, const Vec3 &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline double Dot(const Vec3d &a, const Vec3d &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline float MagnitudeSquared(const Vec3 &v) {
    return Dot(v, v);
}

inline double MagnitudeSquared(const Vec3d &v) {
    return Dot(v, v);
}

inline float Magnitude(const Vec3 &v) {
    return FastSqrt(MagnitudeSquared(v));
}

inline double Magnitude(const Vec3d &v) {
    return FastSqrt(MagnitudeSquared(v));
}

inline float Magnitude2(float x, float y) {
    return FastSqrt(x * x + y * y);
}

inline double Magnitude2(double x, double y) {
    return FastSqrt(x * x + y * y);
}

inline Vec3 Normalize(const Vec3 &v) {
    const float mag = Magnitude(v);
    if (mag <= 0.0f) {
        return Vec3{0.0f, 0.0f, 0.0f};
    }
    // Unit vectors are used for directions such as gravity or magnetic heading.
    // Scaling removes sensor magnitude while preserving direction.
    const float inv = 1.0f / mag;
    return Scale(v, inv);
}

inline Vec3d Normalize(const Vec3d &v) {
    const double mag = Magnitude(v);
    if (mag <= 0.0) {
        return Vec3d{0.0, 0.0, 0.0};
    }
    const double inv = 1.0 / mag;
    return Scale(v, inv);
}

inline Quaternion MakeQuaternion(float w, float x, float y, float z) {
    return Quaternion{w, x, y, z};
}

inline Quaterniond MakeQuaternion(double w, double x, double y, double z) {
    return Quaterniond{w, x, y, z};
}

inline Quaternion Multiply(const Quaternion &a, const Quaternion &b) {
    // Quaternion multiplication composes rotations. Order matters: a*b means
    // apply b, then a in the usual active-rotation convention.
    return Quaternion{
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
}

inline Quaterniond Multiply(const Quaterniond &a, const Quaterniond &b) {
    return Quaterniond{
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
}

inline Quaternion Normalize(const Quaternion &q) {
    const float norm = FastSqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm <= 0.0f) {
        return Quaternion{1.0f, 0.0f, 0.0f, 0.0f};
    }
    // A rotation quaternion must be unit length. Normalizing after integration or
    // blending prevents scale drift from becoming a fake rotation.
    const float inv = 1.0f / norm;
    return Quaternion{q.w * inv, q.x * inv, q.y * inv, q.z * inv};
}

inline Quaterniond Normalize(const Quaterniond &q) {
    const double norm = FastSqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm <= 0.0) {
        return Quaterniond{1.0, 0.0, 0.0, 0.0};
    }
    const double inv = 1.0 / norm;
    return Quaterniond{q.w * inv, q.x * inv, q.y * inv, q.z * inv};
}

inline float Clamp(float value, float minValue, float maxValue) {
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

inline double Clamp(double value, double minValue, double maxValue) {
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

inline void QuaternionToEuler(const Quaternion &q, float &yaw, float &pitch, float &roll) {
    // Match python/convert.py (psi, theta, phi) convention exactly.
    //
    // Euler angles are only for diagnostics/logging here. The filters keep using
    // quaternions internally so they do not hit gimbal-lock style singularities.
    const float w = q.w;
    const float x = q.x;
    const float y = q.y;
    const float z = q.z;

    const float r11 = 2.0f * w * w - 1.0f + 2.0f * x * x;
    const float r21 = 2.0f * (x * y - w * z);
    const float r31 = 2.0f * (x * z + w * y);
    const float r32 = 2.0f * (y * z - w * x);
    const float r33 = 2.0f * w * w - 1.0f + 2.0f * z * z;

    // These are rotation-matrix terms expanded directly from the quaternion.
    roll = FastAtan2(r32, r33);

    const float r31Clamped = Clamp(r31, -1.0f, 1.0f);
    pitch = -asinf(r31Clamped);

    yaw = FastAtan2(r21, r11);
}

inline void QuaternionToEuler(const Quaterniond &q, double &yaw, double &pitch, double &roll) {
    // Match python/convert.py (psi, theta, phi) convention exactly.
    const double w = q.w;
    const double x = q.x;
    const double y = q.y;
    const double z = q.z;

    const double r11 = 2.0 * w * w - 1.0 + 2.0 * x * x;
    const double r21 = 2.0 * (x * y - w * z);
    const double r31 = 2.0 * (x * z + w * y);
    const double r32 = 2.0 * (y * z - w * x);
    const double r33 = 2.0 * w * w - 1.0 + 2.0 * z * z;

    roll = FastAtan2(r32, r33);

    const double r31Clamped = Clamp(r31, -1.0, 1.0);
    pitch = -asin(r31Clamped);

    yaw = FastAtan2(r21, r11);
}

inline float EulerToZenith(float pitch, float roll) {
    // Zenith is tilt away from vertical. It ignores yaw because yaw does not
    // change how much frontal area the rocket presents to the airflow.
    float sinPitch;
    float cosPitch;
    float sinRoll;
    float cosRoll;
    FastSinCos(pitch, sinPitch, cosPitch);
    FastSinCos(roll, sinRoll, cosRoll);
    (void)sinPitch;
    (void)sinRoll;
    // Folded zenith uses |cosZenith| to get tilt from vertical even if the IMU
    // mounting makes +Z point opposite the expected direction.
    const float value = Clamp(cosPitch * cosRoll, -1.0f, 1.0f);
    return acosf(fabsf(value));
}

inline double EulerToZenith(double pitch, double roll) {
    const double value = Clamp(cos(pitch) * cos(roll), -1.0, 1.0);
    // Folded zenith: use |cosZenith| to get tilt from vertical (0-90°)
    return acos(fabs(value));
}

/// Computes folded zenith angle directly from a quaternion.
///
/// `R[2][2]` is the vertical component of the body Z axis. Taking `abs` gives
/// tilt from vertical in [0, pi/2] even if the sensor is mounted nose-up or
/// tail-up relative to the body frame.
inline float QuaternionToZenith(const Quaternion &q) {
    // This is the quaternion version of pitch/roll zenith. It is cheaper and
    // avoids converting through Euler angles.
    const float cosZenith = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    return acosf(fabsf(Clamp(cosZenith, -1.0f, 1.0f)));
}
inline double QuaternionToZenith(const Quaterniond &q) {
    const double cosZenith = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    return acos(fabs(Clamp(cosZenith, -1.0, 1.0)));
}

inline float QuaternionToZenithArray(const float q[4]) {
    // q[0]=w, q[1]=x, q[2]=y, q[3]=z
    const float cosZenith = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
    return acosf(fabsf(Clamp(cosZenith, -1.0f, 1.0f)));
}

/// Returns true if a quaternion represents at least `minAngleRad` of rotation.
///
/// This is useful for spotting sensors stuck at identity after boot.
inline bool QuaternionHasRotation(const Quaternion &q, float minAngleRad = 0.01f) {
    // Rotation angle = 2 * acos(|w|), so |w| < cos(minAngle/2) means rotation > minAngle
    const float cosHalfMin = cosf(minAngleRad * 0.5f);
    return fabsf(q.w) < cosHalfMin;
}

inline bool QuaternionHasRotationArray(const float q[4], float minAngleRad = 0.01f) {
    const float cosHalfMin = cosf(minAngleRad * 0.5f);
    return fabsf(q[0]) < cosHalfMin;
}

/// Computes an approximate reciprocal square root for normalization paths.
inline float FastInvSqrt(float x) {
    if (x <= 0.0f) {
        return 0.0f;
    }
#if MATHUTILS_HAVE_ARM_MATH
    // ARM provides VRSQRTE for fast reciprocal sqrt estimation.
    // Use one Newton-Raphson iteration for better accuracy.
    float estimate;
    arm_sqrt_f32(x, &estimate);
    if (estimate <= 0.0f) {
        return 0.0f;
    }
    return 1.0f / estimate;
#else
    // Quake III fast inverse square root with one Newton-Raphson iteration.
    // The exact last bit is not important for trust gates or vector directions;
    // the speed is useful in hot normalization paths.
    union {
        float f;
        uint32_t i;
    } conv;
    conv.f = x;
    conv.i = 0x5f3759df - (conv.i >> 1);
    conv.f *= 1.5f - (0.5f * x * conv.f * conv.f);
    return conv.f;
#endif
}

inline double FastInvSqrt(double x) {
    if (x <= 0.0) {
        return 0.0;
    }
    return 1.0 / sqrt(x);
}

/// Validates and renormalizes a quaternion in-place.
///
/// A badly corrupted quaternion is reset to identity. A slightly non-unit one
/// is normalized so later rotation math stays bounded.
inline bool ValidateQuaternion(Quaternion &q) {
    const float normSq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    // NaN/Inf or a large norm error means the quaternion is unsafe to use.
    if (!std::isfinite(normSq) || normSq < 0.5f || normSq > 2.0f) {
        // Reset to identity rather than letting bad attitude contaminate filters.
        q.w = 1.0f;
        q.x = 0.0f;
        q.y = 0.0f;
        q.z = 0.0f;
        return false;
    }
    // Small drift is expected from numerical integration; renormalize it.
    if (normSq < 0.98f || normSq > 1.02f) {
        const float inv = FastInvSqrt(normSq);
        q.w *= inv;
        q.x *= inv;
        q.y *= inv;
        q.z *= inv;
    }
    return true;
}

inline bool ValidateQuaternion(Quaterniond &q) {
    const double normSq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    if (!std::isfinite(normSq) || normSq < 0.5 || normSq > 2.0) {
        q.w = 1.0;
        q.x = 0.0;
        q.y = 0.0;
        q.z = 0.0;
        return false;
    }
    if (normSq < 0.98 || normSq > 1.02) {
        const double inv = FastInvSqrt(normSq);
        q.w *= inv;
        q.x *= inv;
        q.y *= inv;
        q.z *= inv;
    }
    return true;
}

/// Returns true when the quaternion array is finite and close to unit length.
inline bool IsQuaternionArrayValid(const float *q) {
    if (q == nullptr) {
        return false;
    }
    const float normSq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    return std::isfinite(normSq) && normSq >= 0.5f && normSq <= 2.0f;
}

/// Validates and, when possible, renormalizes a quaternion array in-place.
/// Returns true if valid, false if corrupted (and resets to identity).
inline bool SanitizeQuaternionArray(float *q) {
    if (q == nullptr) {
        return false;
    }
    const float normSq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if (!IsQuaternionArrayValid(q)) {
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
        return false;
    }
    if (normSq < 0.98f || normSq > 1.02f) {
        const float inv = FastInvSqrt(normSq);
        q[0] *= inv;
        q[1] *= inv;
        q[2] *= inv;
        q[3] *= inv;
    }
    return true;
}

/// Backward-compatible alias for callers that expect in-place repair behavior.
inline bool ValidateQuaternionArray(float *q) {
    return SanitizeQuaternionArray(q);
}

// ---------------------------------------------------------------------------
// Quaternion Dot Product
// ---------------------------------------------------------------------------
inline float Dot(const Quaternion &a, const Quaternion &b) {
    return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

inline double Dot(const Quaterniond &a, const Quaterniond &b) {
    return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

/// Blends between two quaternions along the shortest arc on the unit sphere.
inline Quaternion Slerp(const Quaternion &a, const Quaternion &b, float t) {
    /*
     * Slerp blends attitudes by moving along the surface of the unit quaternion
     * sphere. Linear interpolation is fine for nearly equal quaternions, but for
     * larger differences it can cut through the sphere and imply the wrong angular
     * speed unless it is normalized.
     */
    // Clamp t to [0, 1].
    if (t <= 0.0f) {
        return a;
    }
    if (t >= 1.0f) {
        return b;
    }

    // Dot product gives the cosine of the half-angle between orientations.
    float cosHalfTheta = Dot(a, b);

    // q and -q represent the same orientation; flip sign to take the short path.
    Quaternion bAdjusted = b;
    if (cosHalfTheta < 0.0f) {
        bAdjusted.w = -b.w;
        bAdjusted.x = -b.x;
        bAdjusted.y = -b.y;
        bAdjusted.z = -b.z;
        cosHalfTheta = -cosHalfTheta;
    }

    // Very close quaternions can be blended linearly without visible error.
    if (cosHalfTheta > 0.9995f) {
        Quaternion result;
        result.w = a.w + t * (bAdjusted.w - a.w);
        result.x = a.x + t * (bAdjusted.x - a.x);
        result.y = a.y + t * (bAdjusted.y - a.y);
        result.z = a.z + t * (bAdjusted.z - a.z);
        return Normalize(result);
    }

    // Standard spherical interpolation weights each endpoint by sine distance.
    const float halfTheta = acosf(Clamp(cosHalfTheta, -1.0f, 1.0f));
    const float sinHalfTheta = sinf(halfTheta);
    if (sinHalfTheta < 1.0e-6f) {
        return a;  // Degenerate case.
    }

    const float ratioA = sinf((1.0f - t) * halfTheta) / sinHalfTheta;
    const float ratioB = sinf(t * halfTheta) / sinHalfTheta;

    Quaternion result;
    result.w = ratioA * a.w + ratioB * bAdjusted.w;
    result.x = ratioA * a.x + ratioB * bAdjusted.x;
    result.y = ratioA * a.y + ratioB * bAdjusted.y;
    result.z = ratioA * a.z + ratioB * bAdjusted.z;
    return Normalize(result);
}

inline Quaterniond Slerp(const Quaterniond &a, const Quaterniond &b, double t) {
    if (t <= 0.0) {
        return a;
    }
    if (t >= 1.0) {
        return b;
    }

    double cosHalfTheta = Dot(a, b);

    Quaterniond bAdjusted = b;
    if (cosHalfTheta < 0.0) {
        bAdjusted.w = -b.w;
        bAdjusted.x = -b.x;
        bAdjusted.y = -b.y;
        bAdjusted.z = -b.z;
        cosHalfTheta = -cosHalfTheta;
    }

    if (cosHalfTheta > 0.9995) {
        Quaterniond result;
        result.w = a.w + t * (bAdjusted.w - a.w);
        result.x = a.x + t * (bAdjusted.x - a.x);
        result.y = a.y + t * (bAdjusted.y - a.y);
        result.z = a.z + t * (bAdjusted.z - a.z);
        return Normalize(result);
    }

    const double halfTheta = acos(Clamp(cosHalfTheta, -1.0, 1.0));
    const double sinHalfTheta = sin(halfTheta);
    if (sinHalfTheta < 1.0e-9) {
        return a;
    }

    const double ratioA = sin((1.0 - t) * halfTheta) / sinHalfTheta;
    const double ratioB = sin(t * halfTheta) / sinHalfTheta;

    Quaterniond result;
    result.w = ratioA * a.w + ratioB * bAdjusted.w;
    result.x = ratioA * a.x + ratioB * bAdjusted.x;
    result.y = ratioA * a.y + ratioB * bAdjusted.y;
    result.z = ratioA * a.z + ratioB * bAdjusted.z;
    return Normalize(result);
}

/// Integrates gyro angular rate into a quaternion with an exponential map.
///
/// This applies the small rotation represented by omega*dt to the previous
/// attitude. Small angles use a Taylor approximation to avoid numerical trouble.
inline Quaternion ExponentialMapUpdate(const Quaternion &q, float wx, float wy, float wz, float dt) {
    /*
     * Gyro integration is "apply the rotation measured during this timestep."
     * The exponential map converts angular rate * dt into exactly that small
     * quaternion rotation. This is better behaved than directly integrating Euler
     * angles, especially when the rocket is tilted or rolling.
     */
    // Half-angle vector.
    const float hx = 0.5f * wx * dt;
    const float hy = 0.5f * wy * dt;
    const float hz = 0.5f * wz * dt;

    const float thetaSq = hx * hx + hy * hy + hz * hz;
    constexpr float kSmallAngleThresholdSq = 0.01f * 0.01f;

    float dqW, dqX, dqY, dqZ;
    if (thetaSq < kSmallAngleThresholdSq) {
        // Small angle: approximate sin(theta)/theta and cos(theta) directly.
        dqW = 1.0f - 0.5f * thetaSq;
        const float sincApprox = 1.0f - thetaSq / 6.0f;
        dqX = hx * sincApprox;
        dqY = hy * sincApprox;
        dqZ = hz * sincApprox;
    } else {
        // Larger angle: compute the exact unit rotation from the half-angle vector.
        const float theta = FastSqrt(thetaSq);
        float sinTheta, cosTheta;
        FastSinCos(theta, sinTheta, cosTheta);
        const float sincTheta = sinTheta / theta;
        dqW = cosTheta;
        dqX = hx * sincTheta;
        dqY = hy * sincTheta;
        dqZ = hz * sincTheta;
    }

    // Quaternion multiplication: dq * q.
    Quaternion result;
    result.w = dqW * q.w - dqX * q.x - dqY * q.y - dqZ * q.z;
    result.x = dqW * q.x + dqX * q.w + dqY * q.z - dqZ * q.y;
    result.y = dqW * q.y - dqX * q.z + dqY * q.w + dqZ * q.x;
    result.z = dqW * q.z + dqX * q.y - dqY * q.x + dqZ * q.w;

    // Normalize to maintain unit quaternion constraint.
    return Normalize(result);
}

inline Quaterniond ExponentialMapUpdate(const Quaterniond &q, double wx, double wy, double wz, double dt) {
    const double hx = 0.5 * wx * dt;
    const double hy = 0.5 * wy * dt;
    const double hz = 0.5 * wz * dt;

    const double thetaSq = hx * hx + hy * hy + hz * hz;
    constexpr double kSmallAngleThresholdSq = 0.01 * 0.01;

    double dqW, dqX, dqY, dqZ;
    if (thetaSq < kSmallAngleThresholdSq) {
        dqW = 1.0 - 0.5 * thetaSq;
        const double sincApprox = 1.0 - thetaSq / 6.0;
        dqX = hx * sincApprox;
        dqY = hy * sincApprox;
        dqZ = hz * sincApprox;
    } else {
        const double theta = FastSqrt(thetaSq);
        const double sinTheta = sin(theta);
        const double cosTheta = cos(theta);
        const double sincTheta = sinTheta / theta;
        dqW = cosTheta;
        dqX = hx * sincTheta;
        dqY = hy * sincTheta;
        dqZ = hz * sincTheta;
    }

    Quaterniond result;
    result.w = dqW * q.w - dqX * q.x - dqY * q.y - dqZ * q.z;
    result.x = dqW * q.x + dqX * q.w + dqY * q.z - dqZ * q.y;
    result.y = dqW * q.y - dqX * q.z + dqY * q.w + dqZ * q.x;
    result.z = dqW * q.z + dqX * q.y - dqY * q.x + dqZ * q.w;

    return Normalize(result);
}

/// Blends two attitude estimates with relative weights.
inline Quaternion WeightedQuaternionBlend(const Quaternion &q1, float w1,
                                          const Quaternion &q2, float w2) {
    // Convert weights into a Slerp fraction so the blend stays on the unit sphere.
    const float totalWeight = w1 + w2;
    if (totalWeight <= 0.0f) {
        return q1;
    }
    const float t = w2 / totalWeight;
    return Slerp(q1, q2, t);
}

}  // namespace math_utils
