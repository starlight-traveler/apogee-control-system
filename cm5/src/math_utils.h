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

/// \file math_utils.h
/// \brief Minimal vector and quaternion helpers mirroring math_lib.py functionality.

/// \brief Collection of math helpers used across cm5.
namespace math_utils {

/// \brief Simple 3D vector.
struct Vec3 {
    float x;
    float y;
    float z;
};

/// \brief Quaternion in wxyz order.
struct Quaternion {
    float w;
    float x;
    float y;
    float z;
};

/// \brief Construct a Vec3.
/// \param x X component.
/// \param y Y component.
/// \param z Z component.
/// \return A new vector.
inline Vec3 MakeVec3(float x, float y, float z) { return Vec3{x, y, z}; }

/// \brief Compute sine and cosine for an angle.
/// \param angle Angle in radians.
/// \param sineOut Output sine.
/// \param cosineOut Output cosine.
inline void FastSinCos(float angle, float &sineOut, float &cosineOut) {
#if MATHUTILS_HAVE_ARM_MATH
    arm_sin_cos_f32(angle, &sineOut, &cosineOut);
#else
    sineOut = sinf(angle);
    cosineOut = cosf(angle);
#endif
}

/// \brief Fast atan2 wrapper with hardware optimization where available.
/// \param y Y component.
/// \param x X component.
/// \return atan2(y, x) in radians.
inline float FastAtan2(float y, float x) { return atan2f(y, x); }

/// \brief Fast square root wrapper with hardware optimization where available.
/// \param value Input value.
/// \return Square root of value.
inline float FastSqrt(float value) {
#if MATHUTILS_HAVE_ARM_MATH
    float result;
    arm_sqrt_f32(value, &result);
    return result;
#else
    return sqrtf(value);
#endif
}

/// \brief Add two vectors.
/// \param a Left operand.
/// \param b Right operand.
/// \return a + b.
inline Vec3 Add(const Vec3 &a, const Vec3 &b) {
    return Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}

/// \brief Subtract two vectors.
/// \param a Left operand.
/// \param b Right operand.
/// \return a - b.
inline Vec3 Subtract(const Vec3 &a, const Vec3 &b) {
    return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
}

/// \brief Scale a vector.
/// \param v Vector to scale.
/// \param s Scale factor.
/// \return v * s.
inline Vec3 Scale(const Vec3 &v, float s) {
    return Vec3{v.x * s, v.y * s, v.z * s};
}

/// \brief Dot product of two vectors.
/// \param a Left operand.
/// \param b Right operand.
/// \return a dot b.
inline float Dot(const Vec3 &a, const Vec3 &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// \brief Squared magnitude of a vector.
/// \param v Input vector.
/// \return |v|^2.
inline float MagnitudeSquared(const Vec3 &v) {
    return Dot(v, v);
}

/// \brief Magnitude of a vector.
/// \param v Input vector.
/// \return |v|.
inline float Magnitude(const Vec3 &v) {
    return FastSqrt(MagnitudeSquared(v));
}

/// \brief Magnitude of a 2D vector.
/// \param x X component.
/// \param y Y component.
/// \return sqrt(x^2 + y^2).
inline float Magnitude2(float x, float y) {
    return FastSqrt(x * x + y * y);
}

/// \brief Normalize a vector.
/// \param v Input vector.
/// \return Unit-length vector (or zero vector if magnitude is zero).
inline Vec3 Normalize(const Vec3 &v) {
    const float mag = Magnitude(v);
    if (mag <= 0.0f) {
        return Vec3{0.0f, 0.0f, 0.0f};
    }
    const float inv = 1.0f / mag;
    return Scale(v, inv);
}

/// \brief Construct a quaternion.
/// \param w Scalar component.
/// \param x X component.
/// \param y Y component.
/// \param z Z component.
/// \return A new quaternion.
inline Quaternion MakeQuaternion(float w, float x, float y, float z) {
    return Quaternion{w, x, y, z};
}

/// \brief Multiply two quaternions.
/// \param a Left operand.
/// \param b Right operand.
/// \return a * b.
inline Quaternion Multiply(const Quaternion &a, const Quaternion &b) {
    return Quaternion{
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
}

/// \brief Normalize a quaternion.
/// \param q Input quaternion.
/// \return Unit quaternion (or identity if norm is zero).
inline Quaternion Normalize(const Quaternion &q) {
    const float norm = FastSqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm <= 0.0f) {
        return Quaternion{1.0f, 0.0f, 0.0f, 0.0f};
    }
    const float inv = 1.0f / norm;
    return Quaternion{q.w * inv, q.x * inv, q.y * inv, q.z * inv};
}

/// \brief Clamp a value to [minValue, maxValue].
/// \param value Input value.
/// \param minValue Minimum allowed.
/// \param maxValue Maximum allowed.
/// \return Clamped value.
inline float Clamp(float value, float minValue, float maxValue) {
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

/// \brief Convert a quaternion to yaw/pitch/roll Euler angles.
/// \param q Input quaternion.
/// \param yaw Output yaw in radians.
/// \param pitch Output pitch in radians.
/// \param roll Output roll in radians.
inline void QuaternionToEuler(const Quaternion &q, float &yaw, float &pitch, float &roll) {
    const float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    const float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    roll = FastAtan2(sinr_cosp, cosr_cosp);

    const float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (fabsf(sinp) >= 1.0f) {
        pitch = (sinp > 0.0f ? 1.0f : -1.0f) * (3.14159265358979323846f * 0.5f);
    } else {
        pitch = asinf(sinp);
    }

    const float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    const float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    yaw = FastAtan2(siny_cosp, cosy_cosp);
}

/// \brief Convert pitch/roll into a zenith angle.
/// \param pitch Pitch in radians.
/// \param roll Roll in radians.
/// \return Zenith angle in radians.
inline float EulerToZenith(float pitch, float roll) {
    const float value = Clamp(cosf(pitch) * cosf(roll), -1.0f, 1.0f);
    return acosf(value);
}

}  // namespace math_utils
