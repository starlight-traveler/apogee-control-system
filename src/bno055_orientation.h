#pragma once

#include <math.h>

namespace bno055_orientation {

// Rotation matrix that maps vectors reported in the IMU's frame into the
// rocket body frame. The sensor is mounted with +Z pointing up toward the
// nosecone, and +X/+Y lying in the body XY plane (right-handed).
inline constexpr float kOrientationMatrix[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
};

// Quaternion representing the sensor-to-body rotation. With the current
// mounting, the frames are aligned.
inline constexpr float kQuaternionOffset[4] = {1.0f, 0.0f, 0.0f, 0.0f};

inline void TransformVector(float x, float y, float z, float &outX, float &outY, float &outZ) {
    outX = kOrientationMatrix[0][0] * x + kOrientationMatrix[0][1] * y + kOrientationMatrix[0][2] * z;
    outY = kOrientationMatrix[1][0] * x + kOrientationMatrix[1][1] * y + kOrientationMatrix[1][2] * z;
    outZ = kOrientationMatrix[2][0] * x + kOrientationMatrix[2][1] * y + kOrientationMatrix[2][2] * z;
}

inline void AdjustQuaternion(float w, float x, float y, float z, float *out) {
    const float ow = kQuaternionOffset[0];
    const float ox = kQuaternionOffset[1];
    const float oy = kQuaternionOffset[2];
    const float oz = kQuaternionOffset[3];

    // Premultiply: new_orientation = offset * raw_orientation.
    const float rw = ow * w - ox * x - oy * y - oz * z;
    const float rx = ow * x + ox * w + oy * z - oz * y;
    const float ry = ow * y - ox * z + oy * w + oz * x;
    const float rz = ow * z + ox * y - oy * x + oz * w;

    const float norm = sqrtf(rw * rw + rx * rx + ry * ry + rz * rz);
    if (norm > 0.0f) {
        const float invNorm = 1.0f / norm;
        out[0] = rw * invNorm;
        out[1] = rx * invNorm;
        out[2] = ry * invNorm;
        out[3] = rz * invNorm;
    } else {
        out[0] = rw;
        out[1] = rx;
        out[2] = ry;
        out[3] = rz;
    }
}

}  // namespace bno055_orientation
