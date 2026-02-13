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
    return acosf(cosPitch * cosRoll);
}

inline double EulerToZenith(double pitch, double roll) {
    const double value = Clamp(cos(pitch) * cos(roll), -1.0, 1.0);
    return acos(value);
}

}  // namespace math_utils
