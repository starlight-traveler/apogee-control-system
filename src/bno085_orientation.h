#pragma once

namespace bno085_orientation {

// The BNO frame is now the rocket body-frame reference. Leave vectors and
// quaternions unchanged unless the sidecar is physically remounted.
inline void TransformVector(float x, float y, float z, float &outX, float &outY, float &outZ) {
    outX = x;
    outY = y;
    outZ = z;
}

inline void AdjustQuaternion(float w, float x, float y, float z, float *out) {
    out[0] = w;
    out[1] = x;
    out[2] = y;
    out[3] = z;
}

}  // namespace bno085_orientation
