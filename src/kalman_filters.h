#pragma once

#include <cmath>
#include <string.h>

#include "constants.h"

// Implements the lightweight Kalman filters from filter.py without dynamic allocations.

class KalmanFilterAccel {
  public:
    KalmanFilterAccel() { Reset(); }

    void Configure(double measurementSigma) {
        measurementVariance_ = measurementSigma * measurementSigma;
        Reset();
    }

    void Reset() {
        memset(state_, 0, sizeof(state_));
        memset(covariance_, 0, sizeof(covariance_));
        covariance_[0][0] = 1000.0;
        covariance_[1][1] = 1000.0;
        covariance_[2][2] = 10.0;
    }

    void Predict(double dt, double processSigma) {
        if (dt <= 0.0) {
            dt = 0.03;
        }
        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double dt4 = dt3 * dt;
        const double half_dt2 = 0.5 * dt2;

        const double x0 = state_[0];
        const double x1 = state_[1];
        const double x2 = state_[2];

        state_[0] = x0 + dt * x1 + half_dt2 * x2;
        state_[1] = x1 + dt * x2;
        state_[2] = x2;

        const double p00 = covariance_[0][0];
        const double p01 = covariance_[0][1];
        const double p02 = covariance_[0][2];
        const double p10 = covariance_[1][0];
        const double p11 = covariance_[1][1];
        const double p12 = covariance_[1][2];
        const double p20 = covariance_[2][0];
        const double p21 = covariance_[2][1];
        const double p22 = covariance_[2][2];
        (void)p11;
        (void)p21;

        const double fp00 = p00 + dt * p10 + half_dt2 * p20;
        const double fp01 = p01 + dt * p11 + half_dt2 * p21;
        const double fp02 = p02 + dt * p12 + half_dt2 * p22;
        const double fp10 = p10 + dt * p20;
        const double fp11 = p11 + dt * p21;
        const double fp12 = p12 + dt * p22;
        const double fp20 = p20;
        const double fp21 = p21;
        const double fp22 = p22;

        double newP00 = fp00;
        double newP01 = fp00 * dt + fp01;
        double newP02 = fp00 * half_dt2 + fp01 * dt + fp02;
        double newP11 = fp10 * dt + fp11;
        double newP12 = fp10 * half_dt2 + fp11 * dt + fp12;
        double newP22 = fp20 * half_dt2 + fp21 * dt + fp22;

        const double qVar = processSigma * processSigma;
        newP00 += qVar * (0.25 * dt4);
        newP01 += qVar * (0.5 * dt3);
        newP02 += qVar * (0.5 * dt2);
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
    void Update(double accelMeasurement) {
        const double residual = accelMeasurement - state_[2];
        double innovation = covariance_[2][2] + measurementVariance_;
        if (innovation <= 0.0) {
            innovation = measurementVariance_;
        }

        const double invInnovation = 1.0 / innovation;
        const double k0 = covariance_[0][2] * invInnovation;
        const double k1 = covariance_[1][2] * invInnovation;
        const double k2 = covariance_[2][2] * invInnovation;

        state_[0] += k0 * residual;
        state_[1] += k1 * residual;
        state_[2] += k2 * residual;

        const double p00 = covariance_[0][0];
        const double p01 = covariance_[0][1];
        const double p02 = covariance_[0][2];
        const double p10 = covariance_[1][0];
        const double p11 = covariance_[1][1];
        const double p12 = covariance_[1][2];
        const double p20 = covariance_[2][0];
        const double p21 = covariance_[2][1];
        const double p22 = covariance_[2][2];

        const double temp00 = p00 - k0 * p20;
        const double temp01 = p01 - k0 * p21;
        const double temp02 = p02 - k0 * p22;
        const double temp10 = p10 - k1 * p20;
        const double temp11 = p11 - k1 * p21;
        const double temp12 = p12 - k1 * p22;
        const double oneMinusK2 = 1.0 - k2;
        const double temp20 = oneMinusK2 * p20;
        const double temp21 = oneMinusK2 * p21;
        const double temp22 = oneMinusK2 * p22;

        double newP00 = temp00;
        double newP01 = temp01;
        double newP02 = -k0 * temp00 - k1 * temp01 + (1.0 - k2) * temp02;
        double newP10 = temp10;
        double newP11 = temp11;
        double newP12 = -k0 * temp10 - k1 * temp11 + (1.0 - k2) * temp12;
        double newP20 = temp20;
        double newP21 = temp21;
        double newP22 = -k0 * temp20 - k1 * temp21 + (1.0 - k2) * temp22;

        const double measVar = measurementVariance_;
        const double add00 = measVar * k0 * k0;
        const double add01 = measVar * k0 * k1;
        const double add02 = measVar * k0 * k2;
        const double add11 = measVar * k1 * k1;
        const double add12 = measVar * k1 * k2;
        const double add22 = measVar * k2 * k2;

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
        covariance_[0][1] = 0.5 * (newP01 + newP10);
        covariance_[0][2] = 0.5 * (newP02 + newP20);
        covariance_[1][0] = covariance_[0][1];
        covariance_[1][1] = newP11;
        covariance_[1][2] = 0.5 * (newP12 + newP21);
        covariance_[2][0] = covariance_[0][2];
        covariance_[2][1] = covariance_[1][2];
        covariance_[2][2] = newP22;
    }

    double Position() const { return state_[0]; }
    double Velocity() const { return state_[1]; }
    double Acceleration() const { return state_[2]; }

  private:
    double state_[3] = {0.0, 0.0, 0.0};
    double covariance_[3][3] = {{0.0}};
    double measurementVariance_ = 0.25;
};

class KalmanFilterAccelAlt {
  public:
    KalmanFilterAccelAlt() { Reset(); }

    void Configure(double accelSigma, double altSigma) {
        accelVariance_ = accelSigma * accelSigma;
        altitudeVariance_ = altSigma * altSigma;
        Reset();
    }

    void Reset() {
        memset(state_, 0, sizeof(state_));
        memset(covariance_, 0, sizeof(covariance_));
        covariance_[0][0] = 1000.0;
        covariance_[1][1] = 1000.0;
        covariance_[2][2] = 10.0;
    }

    void Predict(double dt, double processSigma) {
        if (dt <= 0.0) {
            dt = 0.03;
        }
        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double dt4 = dt3 * dt;

        const double F[3][3] = {
            {1.0, dt, 0.5 * dt2},
            {0.0, 1.0, dt},
            {0.0, 0.0, 1.0},
        };

        double newState[3];
        for (int i = 0; i < 3; ++i) {
            newState[i] = F[i][0] * state_[0] + F[i][1] * state_[1] + F[i][2] * state_[2];
        }
        memcpy(state_, newState, sizeof(state_));

        double temp[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                temp[i][j] = F[i][0] * covariance_[0][j] + F[i][1] * covariance_[1][j] + F[i][2] * covariance_[2][j];
            }
        }

        double updated[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                updated[i][j] = temp[i][0] * F[j][0] + temp[i][1] * F[j][1] + temp[i][2] * F[j][2];
            }
        }

        const double qVar = processSigma * processSigma;
        const double Q[3][3] = {
            {0.25 * dt4 * qVar, 0.5 * dt3 * qVar, 0.5 * dt2 * qVar},
            {0.5 * dt3 * qVar, dt2 * qVar, dt * qVar},
            {0.5 * dt2 * qVar, dt * qVar, qVar},
        };

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                covariance_[i][j] = updated[i][j] + Q[i][j];
            }
        }
    }

    // Joint acceleration/altimeter update using Joseph form for numerical stability.
    void Update(double accelMeasurement, double altitudeMeasurement) {
        const double residualAccel = accelMeasurement - state_[2];
        const double residualAlt = altitudeMeasurement - state_[0];

        const double p00 = covariance_[0][0];
        const double p01 = covariance_[0][1];
        const double p02 = covariance_[0][2];
        const double p10 = covariance_[1][0];
        const double p11 = covariance_[1][1];
        const double p12 = covariance_[1][2];
        const double p20 = covariance_[2][0];
        const double p21 = covariance_[2][1];
        const double p22 = covariance_[2][2];

        const double S00 = p22 + accelVariance_;
        const double S01 = p20;
        const double S10 = p02;
        const double S11 = p00 + altitudeVariance_;
        double det = S00 * S11 - S01 * S10;
        if (std::fabs(det) < 1e-12) {
            det = (det >= 0.0 ? 1e-12 : -1e-12);
        }
        const double invDet = 1.0 / det;
        const double invS00 = S11 * invDet;
        const double invS01 = -S01 * invDet;
        const double invS10 = -S10 * invDet;
        const double invS11 = S00 * invDet;

        const double k00 = p02 * invS00 + p00 * invS10;
        const double k01 = p02 * invS01 + p00 * invS11;
        const double k10 = p12 * invS00 + p10 * invS10;
        const double k11 = p12 * invS01 + p10 * invS11;
        const double k20 = p22 * invS00 + p20 * invS10;
        const double k21 = p22 * invS01 + p20 * invS11;

        state_[0] += k00 * residualAccel + k01 * residualAlt;
        state_[1] += k10 * residualAccel + k11 * residualAlt;
        state_[2] += k20 * residualAccel + k21 * residualAlt;

        const double m00 = 1.0 - k01;
        const double m01 = 0.0;
        const double m02 = -k00;
        const double m10 = -k11;
        const double m11 = 1.0;
        const double m12 = -k10;
        const double m20 = -k21;
        const double m21 = 0.0;
        const double m22 = 1.0 - k20;

        double M[3][3] = {
            {m00, m01, m02},
            {m10, m11, m12},
            {m20, m21, m22},
        };

        double MP[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                MP[i][j] = M[i][0] * covariance_[0][j] + M[i][1] * covariance_[1][j] + M[i][2] * covariance_[2][j];
            }
        }

        double newP[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                newP[i][j] = MP[i][0] * M[j][0] + MP[i][1] * M[j][1] + MP[i][2] * M[j][2];
            }
        }

        const double accelVar = accelVariance_;
        const double altVar = altitudeVariance_;
        for (int i = 0; i < 3; ++i) {
            const double k0i = (i == 0 ? k00 : (i == 1 ? k10 : k20));
            const double k1i = (i == 0 ? k01 : (i == 1 ? k11 : k21));
            for (int j = 0; j < 3; ++j) {
                const double k0j = (j == 0 ? k00 : (j == 1 ? k10 : k20));
                const double k1j = (j == 0 ? k01 : (j == 1 ? k11 : k21));
                newP[i][j] += accelVar * k0i * k0j + altVar * k1i * k1j;
            }
        }

        covariance_[0][0] = newP[0][0];
        covariance_[0][1] = newP[0][1];
        covariance_[0][2] = newP[0][2];
        covariance_[1][0] = newP[1][0];
        covariance_[1][1] = newP[1][1];
        covariance_[1][2] = newP[1][2];
        covariance_[2][0] = newP[2][0];
        covariance_[2][1] = newP[2][1];
        covariance_[2][2] = newP[2][2];
    }

    double Position() const { return state_[0]; }
    double Velocity() const { return state_[1]; }
    double Acceleration() const { return state_[2]; }

  private:
    double state_[3] = {0.0, 0.0, 0.0};
    double covariance_[3][3] = {{0.0}};
    double accelVariance_ = 0.25;
    double altitudeVariance_ = 0.25;
};
