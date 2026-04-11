#pragma once

#include <math.h>

namespace bno085_orientation {

// Sensor-to-body basis change for the current BNO mounting:
// body +X = sensor +Z
// body +Y = sensor -Y
// body +Z = sensor -X
inline void TransformBasisVector(float x, float y, float z, float &outX, float &outY, float &outZ) {
    outX = z;
    outY = -y;
    outZ = -x;
}

inline void TransformVector(float x, float y, float z, float &outX, float &outY, float &outZ) {
    TransformBasisVector(x, y, z, outX, outY, outZ);
}

inline void AdjustQuaternion(float w, float x, float y, float z, float *out) {
    const float r00 = 1.0f - 2.0f * (y * y + z * z);
    const float r01 = 2.0f * (x * y - w * z);
    const float r02 = 2.0f * (x * z + w * y);
    const float r10 = 2.0f * (x * y + w * z);
    const float r11 = 1.0f - 2.0f * (x * x + z * z);
    const float r12 = 2.0f * (y * z - w * x);
    const float r20 = 2.0f * (x * z - w * y);
    const float r21 = 2.0f * (y * z + w * x);
    const float r22 = 1.0f - 2.0f * (x * x + y * y);

    float col0[3] = {r00, r10, r20};
    float col1[3] = {r01, r11, r21};
    float col2[3] = {r02, r12, r22};
    TransformBasisVector(col0[0], col0[1], col0[2], col0[0], col0[1], col0[2]);
    TransformBasisVector(col1[0], col1[1], col1[2], col1[0], col1[1], col1[2]);
    TransformBasisVector(col2[0], col2[1], col2[2], col2[0], col2[1], col2[2]);

    float row0[3];
    float row1[3];
    float row2[3];
    TransformBasisVector(col0[0], col1[0], col2[0], row0[0], row0[1], row0[2]);
    TransformBasisVector(col0[1], col1[1], col2[1], row1[0], row1[1], row1[2]);
    TransformBasisVector(col0[2], col1[2], col2[2], row2[0], row2[1], row2[2]);

    const float trace = row0[0] + row1[1] + row2[2];
    if (trace > 0.0f) {
        const float s = 2.0f * sqrtf(trace + 1.0f);
        out[0] = 0.25f * s;
        out[1] = (row2[1] - row1[2]) / s;
        out[2] = (row0[2] - row2[0]) / s;
        out[3] = (row1[0] - row0[1]) / s;
    } else if (row0[0] > row1[1] && row0[0] > row2[2]) {
        const float s = 2.0f * sqrtf(1.0f + row0[0] - row1[1] - row2[2]);
        out[0] = (row2[1] - row1[2]) / s;
        out[1] = 0.25f * s;
        out[2] = (row0[1] + row1[0]) / s;
        out[3] = (row0[2] + row2[0]) / s;
    } else if (row1[1] > row2[2]) {
        const float s = 2.0f * sqrtf(1.0f + row1[1] - row0[0] - row2[2]);
        out[0] = (row0[2] - row2[0]) / s;
        out[1] = (row0[1] + row1[0]) / s;
        out[2] = 0.25f * s;
        out[3] = (row1[2] + row2[1]) / s;
    } else {
        const float s = 2.0f * sqrtf(1.0f + row2[2] - row0[0] - row1[1]);
        out[0] = (row1[0] - row0[1]) / s;
        out[1] = (row0[2] + row2[0]) / s;
        out[2] = (row1[2] + row2[1]) / s;
        out[3] = 0.25f * s;
    }

    const float norm = sqrtf(out[0] * out[0] + out[1] * out[1] + out[2] * out[2] + out[3] * out[3]);
    if (norm <= 1.0e-9f) {
        out[0] = 1.0f;
        out[1] = 0.0f;
        out[2] = 0.0f;
        out[3] = 0.0f;
        return;
    }
    const float invNorm = 1.0f / norm;
    out[0] *= invNorm;
    out[1] *= invNorm;
    out[2] *= invNorm;
    out[3] *= invNorm;
}

}  // namespace bno085_orientation
