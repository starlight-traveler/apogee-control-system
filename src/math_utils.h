#pragma once

#include <math.h>

#if __has_include(<arm_math.h>)
#ifndef ARM_MATH_CM7
#define ARM_MATH_CM7
#endif
#include <arm_math.h>
#define MATHUTILS_HAVE_ARM_MATH 1
#else
#define MATHUTILS_HAVE_ARM_MATH 0
#endif

// Minimal vector and quaternion helpers mirroring math_lib.py functionality.
namespace math_utils {

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

inline void FastSinCos(float angle, float &sineOut, float &cosineOut) {
#if MATHUTILS_HAVE_ARM_MATH
    arm_sin_cos_f32(angle, &sineOut, &cosineOut);
#else
    sineOut = sinf(angle);
    cosineOut = cosf(angle);
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
    const float w = q.w;
    const float x = q.x;
    const float y = q.y;
    const float z = q.z;

    const float r11 = 2.0f * w * w - 1.0f + 2.0f * x * x;
    const float r21 = 2.0f * (x * y - w * z);
    const float r31 = 2.0f * (x * z + w * y);
    const float r32 = 2.0f * (y * z - w * x);
    const float r33 = 2.0f * w * w - 1.0f + 2.0f * z * z;

    roll = FastAtan2(r32, r33);

    const float denom = 1.0f - r31 * r31;
    const float root = (denom >= 0.0f) ? FastSqrt(denom) : sqrtf(denom);
    if (root != 0.0f) {
        pitch = -atanf(r31 / root);
    } else {
        pitch = (r31 >= 0.0f ? -1.0f : 1.0f) * (3.14159265358979323846f * 0.5f);
    }

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

    const double denom = 1.0 - r31 * r31;
    const double root = (denom >= 0.0) ? FastSqrt(denom) : sqrt(denom);
    if (root != 0.0) {
        pitch = -atan(r31 / root);
    } else {
        pitch = (r31 >= 0.0 ? -1.0 : 1.0) * (3.14159265358979323846 * 0.5);
    }

    yaw = FastAtan2(r21, r11);
}

inline float EulerToZenith(float pitch, float roll) {
    float sinPitch;
    float cosPitch;
    float sinRoll;
    float cosRoll;
    FastSinCos(pitch, sinPitch, cosPitch);
    FastSinCos(roll, sinRoll, cosRoll);
    (void)sinPitch;
    (void)sinRoll;
    // Folded zenith: use |cosZenith| to get tilt from vertical (0-90°)
    // regardless of sensor mounting convention (whether +Z points to nose or tail)
    return acosf(fabsf(cosPitch * cosRoll));
}

inline double EulerToZenith(double pitch, double roll) {
    const double value = Clamp(cos(pitch) * cos(roll), -1.0, 1.0);
    // Folded zenith: use |cosZenith| to get tilt from vertical (0-90°)
    return acos(fabs(value));
}

// ---------------------------------------------------------------------------
// Direct Quaternion to Zenith (Folded)
// Computes zenith angle (tilt from vertical) directly from quaternion without
// intermediate Euler conversion. Uses |cosZenith| to return folded zenith
// in range [0, π/2] (0-90°), giving actual tilt from vertical regardless of
// sensor mounting convention (whether +Z points to nose or tail).
// zenith = acos(|R[2][2]|) where R[2][2] = 1 - 2*(x² + y²)
// ---------------------------------------------------------------------------
inline float QuaternionToZenith(const Quaternion &q) {
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

// ---------------------------------------------------------------------------
// Quaternion Non-Identity Check
// Returns true if quaternion represents a rotation of at least minAngleRad.
// Useful for detecting uninitialized or stuck-at-identity quaternions.
// For a quaternion q, the rotation angle is 2*acos(|w|).
// ---------------------------------------------------------------------------
inline bool QuaternionHasRotation(const Quaternion &q, float minAngleRad = 0.01f) {
    // Rotation angle = 2 * acos(|w|), so |w| < cos(minAngle/2) means rotation > minAngle
    const float cosHalfMin = cosf(minAngleRad * 0.5f);
    return fabsf(q.w) < cosHalfMin;
}

inline bool QuaternionHasRotationArray(const float q[4], float minAngleRad = 0.01f) {
    const float cosHalfMin = cosf(minAngleRad * 0.5f);
    return fabsf(q[0]) < cosHalfMin;
}

// ---------------------------------------------------------------------------
// Fast Inverse Square Root
// Uses ARM intrinsics when available, otherwise Quake-style approximation.
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// Quaternion Magnitude Validation
// Checks if quaternion norm is within acceptable bounds.
// Returns true if valid, false if corrupted (and resets to identity).
// ---------------------------------------------------------------------------
inline bool ValidateQuaternion(Quaternion &q) {
    const float normSq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    // Check for NaN/Inf or magnitude way out of bounds
    if (!std::isfinite(normSq) || normSq < 0.5f || normSq > 2.0f) {
        // Severely corrupted - reset to identity
        q.w = 1.0f;
        q.x = 0.0f;
        q.y = 0.0f;
        q.z = 0.0f;
        return false;
    }
    // Check if slightly out of unit bounds - renormalize
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

/// Validates a quaternion stored as float array [w, x, y, z].
/// Returns true if valid, false if corrupted (and resets to identity).
inline bool ValidateQuaternionArray(float* q) {
    const float normSq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if (!std::isfinite(normSq) || normSq < 0.5f || normSq > 2.0f) {
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

// ---------------------------------------------------------------------------
// Quaternion Dot Product
// ---------------------------------------------------------------------------
inline float Dot(const Quaternion &a, const Quaternion &b) {
    return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

inline double Dot(const Quaterniond &a, const Quaterniond &b) {
    return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

// ---------------------------------------------------------------------------
// Spherical Linear Interpolation (SLERP)
// Blends between two quaternions along the shortest arc on the unit sphere.
// ---------------------------------------------------------------------------
inline Quaternion Slerp(const Quaternion &a, const Quaternion &b, float t) {
    // Clamp t to [0, 1].
    if (t <= 0.0f) {
        return a;
    }
    if (t >= 1.0f) {
        return b;
    }

    // Compute cosine of angle between quaternions.
    float cosHalfTheta = Dot(a, b);

    // If negative dot, negate one quaternion to take the shorter path.
    Quaternion bAdjusted = b;
    if (cosHalfTheta < 0.0f) {
        bAdjusted.w = -b.w;
        bAdjusted.x = -b.x;
        bAdjusted.y = -b.y;
        bAdjusted.z = -b.z;
        cosHalfTheta = -cosHalfTheta;
    }

    // If quaternions are very close, use linear interpolation to avoid division by zero.
    if (cosHalfTheta > 0.9995f) {
        Quaternion result;
        result.w = a.w + t * (bAdjusted.w - a.w);
        result.x = a.x + t * (bAdjusted.x - a.x);
        result.y = a.y + t * (bAdjusted.y - a.y);
        result.z = a.z + t * (bAdjusted.z - a.z);
        return Normalize(result);
    }

    // Standard SLERP formula.
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
    return result;
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
    return result;
}

// ---------------------------------------------------------------------------
// Exponential Map Quaternion Update
// More accurate integration than first-order Euler: q_new = exp(0.5 * omega * dt) * q_old
// Uses Taylor series for small angles, Rodrigues formula for larger angles.
// ---------------------------------------------------------------------------
inline Quaternion ExponentialMapUpdate(const Quaternion &q, float wx, float wy, float wz, float dt) {
    // Half-angle vector.
    const float hx = 0.5f * wx * dt;
    const float hy = 0.5f * wy * dt;
    const float hz = 0.5f * wz * dt;

    const float thetaSq = hx * hx + hy * hy + hz * hz;
    constexpr float kSmallAngleThresholdSq = 0.01f * 0.01f;

    float dqW, dqX, dqY, dqZ;
    if (thetaSq < kSmallAngleThresholdSq) {
        // Small angle: use Taylor series expansion.
        // exp(theta) ≈ 1 + theta + theta^2/2 (for quaternion: cos(|h|) ≈ 1 - |h|^2/2, sin(|h|)/|h| ≈ 1 - |h|^2/6)
        dqW = 1.0f - 0.5f * thetaSq;
        const float sincApprox = 1.0f - thetaSq / 6.0f;
        dqX = hx * sincApprox;
        dqY = hy * sincApprox;
        dqZ = hz * sincApprox;
    } else {
        // Larger angle: use full Rodrigues formula.
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

// ---------------------------------------------------------------------------
// Weighted Quaternion Blend
// Blends multiple quaternions using weighted SLERP for multi-IMU fusion.
// ---------------------------------------------------------------------------
inline Quaternion WeightedQuaternionBlend(const Quaternion &q1, float w1,
                                          const Quaternion &q2, float w2) {
    const float totalWeight = w1 + w2;
    if (totalWeight <= 0.0f) {
        return q1;
    }
    const float t = w2 / totalWeight;
    return Slerp(q1, q2, t);
}

}  // namespace math_utils
