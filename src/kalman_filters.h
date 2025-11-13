#pragma once

#include <math.h>
#include <string.h>

#include "constants.h"

// Implements the lightweight Kalman filters from filter.py without dynamic allocations.

class KalmanFilterAccel {
  public:
    KalmanFilterAccel() { Reset(); }

    void Configure(float measurementSigma) {
        measurementVariance_ = measurementSigma * measurementSigma;
        Reset();
    }

    void Reset() {
        memset(state_, 0, sizeof(state_));
        memset(covariance_, 0, sizeof(covariance_));
        covariance_[0][0] = 1000.0f;
        covariance_[1][1] = 1000.0f;
        covariance_[2][2] = 10.0f;
    }

    void Predict(float dt, float processSigma) {
        if (dt <= 0.0f) {
            dt = 0.03f;
        }
        const float dt2 = dt * dt;
        const float dt3 = dt2 * dt;
        const float dt4 = dt3 * dt;
        const float half_dt2 = 0.5f * dt2;

        const float x0 = state_[0];
        const float x1 = state_[1];
        const float x2 = state_[2];

        state_[0] = x0 + dt * x1 + half_dt2 * x2;
        state_[1] = x1 + dt * x2;
        state_[2] = x2;

        const float p00 = covariance_[0][0];
        const float p01 = covariance_[0][1];
        const float p02 = covariance_[0][2];
        const float p10 = covariance_[1][0];
        const float p11 = covariance_[1][1];
        const float p12 = covariance_[1][2];
        const float p20 = covariance_[2][0];
        const float p21 = covariance_[2][1];
        const float p22 = covariance_[2][2];

        const float fp00 = p00 + dt * p10 + half_dt2 * p20;
        const float fp01 = p01 + dt * p11 + half_dt2 * p21;
        const float fp02 = p02 + dt * p12 + half_dt2 * p22;
        const float fp10 = p10 + dt * p20;
        const float fp11 = p11 + dt * p21;
        const float fp12 = p12 + dt * p22;
        const float fp20 = p20;
        const float fp21 = p21;
        const float fp22 = p22;

        float newP00 = fp00;
        float newP01 = fp00 * dt + fp01;
        float newP02 = fp00 * half_dt2 + fp01 * dt + fp02;
        float newP11 = fp10 * dt + fp11;
        float newP12 = fp10 * half_dt2 + fp11 * dt + fp12;
        float newP22 = fp20 * half_dt2 + fp21 * dt + fp22;

        const float qVar = processSigma * processSigma;
        newP00 += qVar * (0.25f * dt4);
        newP01 += qVar * (0.5f * dt3);
        newP02 += qVar * (0.5f * dt2);
        newP11 += qVar * dt2;
        newP12 += qVar * dt;
        newP22 += qVar;

        covariance_[0][0] = newP00;
        covariance_[0][1] = newP01;
        covariance_[0][2] = newP02;
        covariance_[1][0] = newP01;
        covariance_[1][1] = newP11;
        covariance_[1][2] = newP12;
        covariance_[2][0] = newP02;
        covariance_[2][1] = newP12;
        covariance_[2][2] = newP22;
    }

    // Measurement update using Joseph form to mirror the FilterPy implementation.
    void Update(float accelMeasurement) {
        const float residual = accelMeasurement - state_[2];
        float innovation = covariance_[2][2] + measurementVariance_;
        if (innovation <= 0.0f) {
            innovation = measurementVariance_;
        }

        const float invInnovation = 1.0f / innovation;
        const float k0 = covariance_[0][2] * invInnovation;
        const float k1 = covariance_[1][2] * invInnovation;
        const float k2 = covariance_[2][2] * invInnovation;

        state_[0] += k0 * residual;
        state_[1] += k1 * residual;
        state_[2] += k2 * residual;

        const float p00 = covariance_[0][0];
        const float p01 = covariance_[0][1];
        const float p02 = covariance_[0][2];
        const float p10 = covariance_[1][0];
        const float p11 = covariance_[1][1];
        const float p12 = covariance_[1][2];
        const float p20 = covariance_[2][0];
        const float p21 = covariance_[2][1];
        const float p22 = covariance_[2][2];

        const float temp00 = p00 - k0 * p20;
        const float temp01 = p01 - k0 * p21;
        const float temp02 = p02 - k0 * p22;
        const float temp10 = p10 - k1 * p20;
        const float temp11 = p11 - k1 * p21;
        const float temp12 = p12 - k1 * p22;
        const float oneMinusK2 = 1.0f - k2;
        const float temp20 = oneMinusK2 * p20;
        const float temp21 = oneMinusK2 * p21;
        const float temp22 = oneMinusK2 * p22;

        float newP00 = temp00;
        float newP01 = temp01;
        float newP02 = -k0 * temp00 - k1 * temp01 + (1.0f - k2) * temp02;
        float newP10 = temp10;
        float newP11 = temp11;
        float newP12 = -k0 * temp10 - k1 * temp11 + (1.0f - k2) * temp12;
        float newP20 = temp20;
        float newP21 = temp21;
        float newP22 = -k0 * temp20 - k1 * temp21 + (1.0f - k2) * temp22;

        const float measVar = measurementVariance_;
        const float add00 = measVar * k0 * k0;
        const float add01 = measVar * k0 * k1;
        const float add02 = measVar * k0 * k2;
        const float add11 = measVar * k1 * k1;
        const float add12 = measVar * k1 * k2;
        const float add22 = measVar * k2 * k2;

        newP00 += add00;
        newP01 += add01;
        newP02 += add02;
        newP10 += add01;
        newP11 += add11;
        newP12 += add12;
        newP20 += add02;
        newP21 += add12;
        newP22 += add22;

        covariance_[0][0] = newP00;
        covariance_[0][1] = 0.5f * (newP01 + newP10);
        covariance_[0][2] = 0.5f * (newP02 + newP20);
        covariance_[1][0] = covariance_[0][1];
        covariance_[1][1] = newP11;
        covariance_[1][2] = 0.5f * (newP12 + newP21);
        covariance_[2][0] = covariance_[0][2];
        covariance_[2][1] = covariance_[1][2];
        covariance_[2][2] = newP22;
    }

    float Position() const { return state_[0]; }
    float Velocity() const { return state_[1]; }
    float Acceleration() const { return state_[2]; }

  private:
    float state_[3] = {0.0f, 0.0f, 0.0f};
    float covariance_[3][3] = {{0.0f}};
    float measurementVariance_ = 0.25f;
};

class KalmanFilterAccelAlt {
  public:
    KalmanFilterAccelAlt() { Reset(); }

    void Configure(float accelSigma, float altSigma) {
        accelVariance_ = accelSigma * accelSigma;
        altitudeVariance_ = altSigma * altSigma;
        Reset();
    }

    void Reset() {
        memset(state_, 0, sizeof(state_));
        memset(covariance_, 0, sizeof(covariance_));
        covariance_[0][0] = 1000.0f;
        covariance_[1][1] = 1000.0f;
        covariance_[2][2] = 10.0f;
    }

    void Predict(float dt, float processSigma) {
        if (dt <= 0.0f) {
            dt = 0.03f;
        }
        const float dt2 = dt * dt;
        const float dt3 = dt2 * dt;
        const float dt4 = dt3 * dt;

        const float F[3][3] = {
            {1.0f, dt, 0.5f * dt2},
            {0.0f, 1.0f, dt},
            {0.0f, 0.0f, 1.0f},
        };

        float newState[3];
        for (int i = 0; i < 3; ++i) {
            newState[i] = F[i][0] * state_[0] + F[i][1] * state_[1] + F[i][2] * state_[2];
        }
        memcpy(state_, newState, sizeof(state_));

        float temp[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                temp[i][j] = F[i][0] * covariance_[0][j] + F[i][1] * covariance_[1][j] + F[i][2] * covariance_[2][j];
            }
        }

        float updated[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                updated[i][j] = temp[i][0] * F[j][0] + temp[i][1] * F[j][1] + temp[i][2] * F[j][2];
            }
        }

        const float qVar = processSigma * processSigma;
        const float Q[3][3] = {
            {0.25f * dt4 * qVar, 0.5f * dt3 * qVar, 0.5f * dt2 * qVar},
            {0.5f * dt3 * qVar, dt2 * qVar, dt * qVar},
            {0.5f * dt2 * qVar, dt * qVar, qVar},
        };

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                covariance_[i][j] = updated[i][j] + Q[i][j];
            }
        }
    }

    // Joint acceleration/altimeter update using Joseph form for numerical stability.
    void Update(float accelMeasurement, float altitudeMeasurement) {
        const float residualAccel = accelMeasurement - state_[2];
        const float residualAlt = altitudeMeasurement - state_[0];

        const float p00 = covariance_[0][0];
        const float p01 = covariance_[0][1];
        const float p02 = covariance_[0][2];
        const float p10 = covariance_[1][0];
        const float p11 = covariance_[1][1];
        const float p12 = covariance_[1][2];
        const float p20 = covariance_[2][0];
        const float p21 = covariance_[2][1];
        const float p22 = covariance_[2][2];

        const float S00 = p22 + accelVariance_;
        const float S01 = p20;
        const float S10 = p02;
        const float S11 = p00 + altitudeVariance_;
        float det = S00 * S11 - S01 * S10;
        if (fabsf(det) < 1e-6f) {
            det = (det >= 0.0f ? 1e-6f : -1e-6f);
        }
        const float invDet = 1.0f / det;
        const float invS00 = S11 * invDet;
        const float invS01 = -S01 * invDet;
        const float invS10 = -S10 * invDet;
        const float invS11 = S00 * invDet;

        const float k00 = p02 * invS00 + p00 * invS10;
        const float k01 = p02 * invS01 + p00 * invS11;
        const float k10 = p12 * invS00 + p10 * invS10;
        const float k11 = p12 * invS01 + p10 * invS11;
        const float k20 = p22 * invS00 + p20 * invS10;
        const float k21 = p22 * invS01 + p20 * invS11;

        state_[0] += k00 * residualAccel + k01 * residualAlt;
        state_[1] += k10 * residualAccel + k11 * residualAlt;
        state_[2] += k20 * residualAccel + k21 * residualAlt;

        const float m00 = 1.0f - k01;
        const float m02 = -k00;
        const float m10 = -k11;
        const float m12 = -k10;
        const float m20 = -k21;
        const float m22 = 1.0f - k20;

        const float temp00 = m00 * p00 + m02 * p20;
        const float temp01 = m00 * p01 + m02 * p21;
        const float temp02 = m00 * p02 + m02 * p22;
        const float temp10 = m10 * p00 + p10 + m12 * p20;
        const float temp11 = m10 * p01 + p11 + m12 * p21;
        const float temp12 = m10 * p02 + p12 + m12 * p22;
        const float temp20 = m20 * p00 + m22 * p20;
        const float temp21 = m20 * p01 + m22 * p21;
        const float temp22 = m20 * p02 + m22 * p22;

        float newP00 = temp00 * m00 + temp01 * m10 + temp02 * m20;
        float newP01 = temp01;
        float newP02 = temp00 * m02 + temp01 * m12 + temp02 * m22;
        float newP10 = temp10 * m00 + temp11 * m10 + temp12 * m20;
        float newP11 = temp11;
        float newP12 = temp10 * m02 + temp11 * m12 + temp12 * m22;
        float newP20 = temp20 * m00 + temp21 * m10 + temp22 * m20;
        float newP21 = temp21;
        float newP22 = temp20 * m02 + temp21 * m12 + temp22 * m22;

        const float accelVar = accelVariance_;
        const float altVar = altitudeVariance_;
        const float add00 = accelVar * k00 * k00 + altVar * k01 * k01;
        const float add01 = accelVar * k00 * k10 + altVar * k01 * k11;
        const float add02 = accelVar * k00 * k20 + altVar * k01 * k21;
        const float add11 = accelVar * k10 * k10 + altVar * k11 * k11;
        const float add12 = accelVar * k10 * k20 + altVar * k11 * k21;
        const float add22 = accelVar * k20 * k20 + altVar * k21 * k21;

        newP00 += add00;
        newP01 += add01;
        newP02 += add02;
        newP10 += add01;
        newP11 += add11;
        newP12 += add12;
        newP20 += add02;
        newP21 += add12;
        newP22 += add22;

        covariance_[0][0] = newP00;
        covariance_[0][1] = 0.5f * (newP01 + newP10);
        covariance_[0][2] = 0.5f * (newP02 + newP20);
        covariance_[1][0] = covariance_[0][1];
        covariance_[1][1] = newP11;
        covariance_[1][2] = 0.5f * (newP12 + newP21);
        covariance_[2][0] = covariance_[0][2];
        covariance_[2][1] = covariance_[1][2];
        covariance_[2][2] = newP22;
    }

    float Position() const { return state_[0]; }
    float Velocity() const { return state_[1]; }
    float Acceleration() const { return state_[2]; }

  private:
    float state_[3] = {0.0f, 0.0f, 0.0f};
    float covariance_[3][3] = {{0.0f}};
    float accelVariance_ = 0.25f;
    float altitudeVariance_ = 0.25f;
};
