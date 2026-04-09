#pragma once

#include <math.h>
#include <stdint.h>

namespace calibration_matrix {

inline void SetIdentity3(float matrix[3][3]) {
    matrix[0][0] = 1.0f;
    matrix[0][1] = 0.0f;
    matrix[0][2] = 0.0f;
    matrix[1][0] = 0.0f;
    matrix[1][1] = 1.0f;
    matrix[1][2] = 0.0f;
    matrix[2][0] = 0.0f;
    matrix[2][1] = 0.0f;
    matrix[2][2] = 1.0f;
}

inline void Copy3(const float in[3], float out[3]) {
    out[0] = in[0];
    out[1] = in[1];
    out[2] = in[2];
}

inline void Copy3x3(const float in[3][3], float out[3][3]) {
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out[r][c] = in[r][c];
        }
    }
}

inline void Transpose3x3(const float in[3][3], float out[3][3]) {
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out[r][c] = in[c][r];
        }
    }
}

inline void Multiply3x3(const float a[3][3], const float b[3][3], float out[3][3]) {
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out[r][c] = a[r][0] * b[0][c] + a[r][1] * b[1][c] + a[r][2] * b[2][c];
        }
    }
}

inline void Apply3x3(const float matrix[3][3], const float in[3], float out[3]) {
    out[0] = matrix[0][0] * in[0] + matrix[0][1] * in[1] + matrix[0][2] * in[2];
    out[1] = matrix[1][0] * in[0] + matrix[1][1] * in[1] + matrix[1][2] * in[2];
    out[2] = matrix[2][0] * in[0] + matrix[2][1] * in[1] + matrix[2][2] * in[2];
}

inline float Dot3(const float a[3], const float b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline float Magnitude3(const float v[3]) {
    return sqrtf(Dot3(v, v));
}

inline bool Normalize3(float v[3]) {
    const float norm = Magnitude3(v);
    if (!(norm > 1.0e-9f)) {
        return false;
    }
    const float inv = 1.0f / norm;
    v[0] *= inv;
    v[1] *= inv;
    v[2] *= inv;
    return true;
}

inline void Cross3(const float a[3], const float b[3], float out[3]) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

inline float Determinant3x3(const float matrix[3][3]) {
    return matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) -
           matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
           matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
}

inline bool Invert3x3(const float in[3][3], float out[3][3]) {
    const float det = Determinant3x3(in);
    if (fabsf(det) <= 1.0e-9f) {
        return false;
    }
    const float invDet = 1.0f / det;
    out[0][0] = (in[1][1] * in[2][2] - in[1][2] * in[2][1]) * invDet;
    out[0][1] = (in[0][2] * in[2][1] - in[0][1] * in[2][2]) * invDet;
    out[0][2] = (in[0][1] * in[1][2] - in[0][2] * in[1][1]) * invDet;
    out[1][0] = (in[1][2] * in[2][0] - in[1][0] * in[2][2]) * invDet;
    out[1][1] = (in[0][0] * in[2][2] - in[0][2] * in[2][0]) * invDet;
    out[1][2] = (in[0][2] * in[1][0] - in[0][0] * in[1][2]) * invDet;
    out[2][0] = (in[1][0] * in[2][1] - in[1][1] * in[2][0]) * invDet;
    out[2][1] = (in[0][1] * in[2][0] - in[0][0] * in[2][1]) * invDet;
    out[2][2] = (in[0][0] * in[1][1] - in[0][1] * in[1][0]) * invDet;
    return true;
}

inline void BuildAxisTransform(const uint8_t axisMap[3], const int8_t axisSign[3], float out[3][3]) {
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out[r][c] = 0.0f;
        }
        out[r][axisMap[r]] = static_cast<float>(axisSign[r]);
    }
}

inline bool BuildAccelTotalCorrection(const float facePosX[3],
                                      const float faceNegX[3],
                                      const float facePosY[3],
                                      const float faceNegY[3],
                                      const float facePosZ[3],
                                      const float faceNegZ[3],
                                      float targetCounts,
                                      float bias[3],
                                      float correction[3][3]) {
    bias[0] = 0.5f * (facePosX[0] + faceNegX[0]);
    bias[1] = 0.5f * (facePosY[1] + faceNegY[1]);
    bias[2] = 0.5f * (facePosZ[2] + faceNegZ[2]);

    float columns[3][3] = {
        {facePosX[0] - bias[0], facePosY[0] - bias[0], facePosZ[0] - bias[0]},
        {facePosX[1] - bias[1], facePosY[1] - bias[1], facePosZ[1] - bias[1]},
        {facePosX[2] - bias[2], facePosY[2] - bias[2], facePosZ[2] - bias[2]},
    };

    float inverseColumns[3][3];
    if (!Invert3x3(columns, inverseColumns)) {
        SetIdentity3(correction);
        return false;
    }

    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            correction[r][c] = targetCounts * inverseColumns[r][c];
        }
    }
    return true;
}

inline bool JacobiEigenSymmetric3(const float in[3][3], float eigenVectors[3][3], float eigenValues[3]) {
    float a[3][3];
    Copy3x3(in, a);
    SetIdentity3(eigenVectors);

    for (int iter = 0; iter < 16; ++iter) {
        int p = 0;
        int q = 1;
        float maxOffDiag = fabsf(a[0][1]);
        if (fabsf(a[0][2]) > maxOffDiag) {
            p = 0;
            q = 2;
            maxOffDiag = fabsf(a[0][2]);
        }
        if (fabsf(a[1][2]) > maxOffDiag) {
            p = 1;
            q = 2;
            maxOffDiag = fabsf(a[1][2]);
        }
        if (maxOffDiag <= 1.0e-7f) {
            break;
        }

        const float app = a[p][p];
        const float aqq = a[q][q];
        const float apq = a[p][q];
        const float phi = 0.5f * atan2f(2.0f * apq, aqq - app);
        const float c = cosf(phi);
        const float s = sinf(phi);

        for (int r = 0; r < 3; ++r) {
            const float arp = a[r][p];
            const float arq = a[r][q];
            a[r][p] = c * arp - s * arq;
            a[r][q] = s * arp + c * arq;
        }
        for (int r = 0; r < 3; ++r) {
            const float apr = a[p][r];
            const float aqr = a[q][r];
            a[p][r] = c * apr - s * aqr;
            a[q][r] = s * apr + c * aqr;
        }
        a[p][q] = 0.0f;
        a[q][p] = 0.0f;

        for (int r = 0; r < 3; ++r) {
            const float vrp = eigenVectors[r][p];
            const float vrq = eigenVectors[r][q];
            eigenVectors[r][p] = c * vrp - s * vrq;
            eigenVectors[r][q] = s * vrp + c * vrq;
        }
    }

    eigenValues[0] = a[0][0];
    eigenValues[1] = a[1][1];
    eigenValues[2] = a[2][2];
    return eigenValues[0] > 1.0e-9f && eigenValues[1] > 1.0e-9f && eigenValues[2] > 1.0e-9f;
}

inline bool SymmetricMatrixPower(const float in[3][3], float exponent, float out[3][3]) {
    float eigenVectors[3][3];
    float eigenValues[3];
    if (!JacobiEigenSymmetric3(in, eigenVectors, eigenValues)) {
        return false;
    }

    float diagonal[3][3] = {};
    diagonal[0][0] = powf(eigenValues[0], exponent);
    diagonal[1][1] = powf(eigenValues[1], exponent);
    diagonal[2][2] = powf(eigenValues[2], exponent);

    float temp[3][3];
    float eigenVectorsT[3][3];
    Multiply3x3(eigenVectors, diagonal, temp);
    Transpose3x3(eigenVectors, eigenVectorsT);
    Multiply3x3(temp, eigenVectorsT, out);
    return true;
}

inline bool PolarDecomposeRight(const float in[3][3], float rotation[3][3], float symmetric[3][3]) {
    float inT[3][3];
    float gram[3][3];
    float invSqrtGram[3][3];
    Transpose3x3(in, inT);
    Multiply3x3(inT, in, gram);
    if (!SymmetricMatrixPower(gram, 0.5f, symmetric) ||
        !SymmetricMatrixPower(gram, -0.5f, invSqrtGram)) {
        return false;
    }
    Multiply3x3(in, invSqrtGram, rotation);
    return true;
}

inline void RotationMatrixToQuaternion(const float matrix[3][3], float quaternion[4]) {
    const float trace = matrix[0][0] + matrix[1][1] + matrix[2][2];
    if (trace > 0.0f) {
        const float s = sqrtf(trace + 1.0f) * 2.0f;
        quaternion[0] = 0.25f * s;
        quaternion[1] = (matrix[2][1] - matrix[1][2]) / s;
        quaternion[2] = (matrix[0][2] - matrix[2][0]) / s;
        quaternion[3] = (matrix[1][0] - matrix[0][1]) / s;
    } else if (matrix[0][0] > matrix[1][1] && matrix[0][0] > matrix[2][2]) {
        const float s = sqrtf(1.0f + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0f;
        quaternion[0] = (matrix[2][1] - matrix[1][2]) / s;
        quaternion[1] = 0.25f * s;
        quaternion[2] = (matrix[0][1] + matrix[1][0]) / s;
        quaternion[3] = (matrix[0][2] + matrix[2][0]) / s;
    } else if (matrix[1][1] > matrix[2][2]) {
        const float s = sqrtf(1.0f + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0f;
        quaternion[0] = (matrix[0][2] - matrix[2][0]) / s;
        quaternion[1] = (matrix[0][1] + matrix[1][0]) / s;
        quaternion[2] = 0.25f * s;
        quaternion[3] = (matrix[1][2] + matrix[2][1]) / s;
    } else {
        const float s = sqrtf(1.0f + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0f;
        quaternion[0] = (matrix[1][0] - matrix[0][1]) / s;
        quaternion[1] = (matrix[0][2] + matrix[2][0]) / s;
        quaternion[2] = (matrix[1][2] + matrix[2][1]) / s;
        quaternion[3] = 0.25f * s;
    }

    const float norm = sqrtf(quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] +
                             quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);
    if (norm <= 1.0e-9f) {
        quaternion[0] = 1.0f;
        quaternion[1] = 0.0f;
        quaternion[2] = 0.0f;
        quaternion[3] = 0.0f;
        return;
    }

    const float invNorm = 1.0f / norm;
    quaternion[0] *= invNorm;
    quaternion[1] *= invNorm;
    quaternion[2] *= invNorm;
    quaternion[3] *= invNorm;
}

inline bool ComputeMagSoftIronFromSamples(const float (*samples)[3],
                                          uint16_t sampleCount,
                                          const float bias[3],
                                          float correction[3][3]) {
    if (sampleCount < 32) {
        return false;
    }

    float covariance[3][3] = {};
    for (uint16_t i = 0; i < sampleCount; ++i) {
        const float centered[3] = {
            samples[i][0] - bias[0],
            samples[i][1] - bias[1],
            samples[i][2] - bias[2],
        };
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                covariance[r][c] += centered[r] * centered[c];
            }
        }
    }

    const float invCount = 1.0f / static_cast<float>(sampleCount);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            covariance[r][c] *= invCount;
        }
    }

    float invSqrtCov[3][3];
    if (!SymmetricMatrixPower(covariance, -0.5f, invSqrtCov)) {
        return false;
    }

    float meanNorm = 0.0f;
    for (uint16_t i = 0; i < sampleCount; ++i) {
        const float centered[3] = {
            samples[i][0] - bias[0],
            samples[i][1] - bias[1],
            samples[i][2] - bias[2],
        };
        float corrected[3] = {0.0f, 0.0f, 0.0f};
        Apply3x3(invSqrtCov, centered, corrected);
        meanNorm += Magnitude3(corrected);
    }
    meanNorm /= static_cast<float>(sampleCount);
    if (!(meanNorm > 1.0e-6f)) {
        return false;
    }

    const float scale = 1.0f / meanNorm;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            correction[r][c] = invSqrtCov[r][c] * scale;
        }
    }
    return true;
}

}  // namespace calibration_matrix
