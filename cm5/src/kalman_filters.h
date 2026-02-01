#pragma once

#include <math.h>
#include <string.h>

#include "constants.h"

/// \file kalman_filters.h
/// \brief Lightweight Kalman filters from filter.py without dynamic allocations.

/// \brief 1D acceleration-only Kalman filter.
class KalmanFilterAccel {
  public:
    /// \brief Construct and reset the filter.
    KalmanFilterAccel() { Reset(); }

    /// \brief Configure measurement noise.
    /// \param measurementSigma Standard deviation of measurement noise.
    void Configure(float measurementSigma) {
        measurementVariance_ = measurementSigma * measurementSigma;
        Reset();
    }

    /// \brief Reset state and covariance to defaults.
    void Reset() {
        memset(state_, 0, sizeof(state_));
        memset(covariance_, 0, sizeof(covariance_));
        covariance_[0][0] = 1000.0f;
        covariance_[1][1] = 1000.0f;
        covariance_[2][2] = 10.0f;
    }

    /// \brief Predict state forward using process noise.
    /// \param dt Time step in seconds.
    /// \param processSigma Standard deviation of process noise.
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

    /// \brief Measurement update using Joseph form to mirror the FilterPy implementation.
    /// \param accelMeasurement Measured acceleration.
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

        const float a00 = 1.0f;
        const float a01 = 0.0f;
        const float a02 = -k0;
        const float a10 = 0.0f;
        const float a11 = 1.0f;
        const float a12 = -k1;
        const float a20 = 0.0f;
        const float a21 = 0.0f;
        const float a22 = 1.0f - k2;

        const float ap00 = a00 * p00 + a01 * p10 + a02 * p20;
        const float ap01 = a00 * p01 + a01 * p11 + a02 * p21;
        const float ap02 = a00 * p02 + a01 * p12 + a02 * p22;
        const float ap10 = a10 * p00 + a11 * p10 + a12 * p20;
        const float ap11 = a10 * p01 + a11 * p11 + a12 * p21;
        const float ap12 = a10 * p02 + a11 * p12 + a12 * p22;
        const float ap20 = a20 * p00 + a21 * p10 + a22 * p20;
        const float ap21 = a20 * p01 + a21 * p11 + a22 * p21;
        const float ap22 = a20 * p02 + a21 * p12 + a22 * p22;

        float newP00 = ap00 * a00 + ap01 * a01 + ap02 * a02;
        float newP01 = ap00 * a10 + ap01 * a11 + ap02 * a12;
        float newP02 = ap00 * a20 + ap01 * a21 + ap02 * a22;
        float newP10 = ap10 * a00 + ap11 * a01 + ap12 * a02;
        float newP11 = ap10 * a10 + ap11 * a11 + ap12 * a12;
        float newP12 = ap10 * a20 + ap11 * a21 + ap12 * a22;
        float newP20 = ap20 * a00 + ap21 * a01 + ap22 * a02;
        float newP21 = ap20 * a10 + ap21 * a11 + ap22 * a12;
        float newP22 = ap20 * a20 + ap21 * a21 + ap22 * a22;

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

    /// \brief Get estimated position.
    /// \return Position state.
    float Position() const { return state_[0]; }
    /// \brief Get estimated velocity.
    /// \return Velocity state.
    float Velocity() const { return state_[1]; }
    /// \brief Get estimated acceleration.
    /// \return Acceleration state.
    float Acceleration() const { return state_[2]; }

  private:
    /// \brief [position, velocity, acceleration] state.
    float state_[3] = {0.0f, 0.0f, 0.0f};
    /// \brief State covariance.
    float covariance_[3][3] = {{0.0f}};
    /// \brief Measurement noise variance.
    float measurementVariance_ = 0.25f;
};

/// \brief 1D Kalman filter fusing acceleration and altitude measurements.
class KalmanFilterAccelAlt {
  public:
    /// \brief Construct and reset the filter.
    KalmanFilterAccelAlt() { Reset(); }

    /// \brief Configure measurement noise for acceleration and altitude.
    /// \param accelSigma Standard deviation of acceleration measurement noise.
    /// \param altSigma Standard deviation of altitude measurement noise.
    void Configure(float accelSigma, float altSigma) {
        accelVariance_ = accelSigma * accelSigma;
        altitudeVariance_ = altSigma * altSigma;
        Reset();
    }

    /// \brief Reset state and covariance to defaults.
    void Reset() {
        memset(state_, 0, sizeof(state_));
        memset(covariance_, 0, sizeof(covariance_));
        covariance_[0][0] = 1000.0f;
        covariance_[1][1] = 1000.0f;
        covariance_[2][2] = 10.0f;
    }

    /// \brief Predict state forward using process noise.
    /// \param dt Time step in seconds.
    /// \param processSigma Standard deviation of process noise.
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

    /// \brief Joint acceleration/altimeter update using Joseph form for numerical stability.
    /// \param accelMeasurement Measured acceleration.
    /// \param altitudeMeasurement Measured altitude.
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

        const float a00 = 1.0f - k01;
        const float a01 = 0.0f;
        const float a02 = -k00;
        const float a10 = -k11;
        const float a11 = 1.0f;
        const float a12 = -k10;
        const float a20 = -k21;
        const float a21 = 0.0f;
        const float a22 = 1.0f - k20;

        const float ap00 = a00 * p00 + a01 * p10 + a02 * p20;
        const float ap01 = a00 * p01 + a01 * p11 + a02 * p21;
        const float ap02 = a00 * p02 + a01 * p12 + a02 * p22;
        const float ap10 = a10 * p00 + a11 * p10 + a12 * p20;
        const float ap11 = a10 * p01 + a11 * p11 + a12 * p21;
        const float ap12 = a10 * p02 + a11 * p12 + a12 * p22;
        const float ap20 = a20 * p00 + a21 * p10 + a22 * p20;
        const float ap21 = a20 * p01 + a21 * p11 + a22 * p21;
        const float ap22 = a20 * p02 + a21 * p12 + a22 * p22;

        float newP00 = ap00 * a00 + ap01 * a01 + ap02 * a02;
        float newP01 = ap00 * a10 + ap01 * a11 + ap02 * a12;
        float newP02 = ap00 * a20 + ap01 * a21 + ap02 * a22;
        float newP10 = ap10 * a00 + ap11 * a01 + ap12 * a02;
        float newP11 = ap10 * a10 + ap11 * a11 + ap12 * a12;
        float newP12 = ap10 * a20 + ap11 * a21 + ap12 * a22;
        float newP20 = ap20 * a00 + ap21 * a01 + ap22 * a02;
        float newP21 = ap20 * a10 + ap21 * a11 + ap22 * a12;
        float newP22 = ap20 * a20 + ap21 * a21 + ap22 * a22;

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

    /// \brief Get estimated position.
    /// \return Position state.
    float Position() const { return state_[0]; }
    /// \brief Get estimated velocity.
    /// \return Velocity state.
    float Velocity() const { return state_[1]; }
    /// \brief Get estimated acceleration.
    /// \return Acceleration state.
    float Acceleration() const { return state_[2]; }

  private:
    /// \brief [position, velocity, acceleration] state.
    float state_[3] = {0.0f, 0.0f, 0.0f};
    /// \brief State covariance.
    float covariance_[3][3] = {{0.0f}};
    /// \brief Acceleration measurement variance.
    float accelVariance_ = 0.25f;
    /// \brief Altitude measurement variance.
    float altitudeVariance_ = 0.25f;
};
