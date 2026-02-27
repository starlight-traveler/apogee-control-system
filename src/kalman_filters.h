#pragma once

#include <algorithm>
#include <cmath>
#include <string.h>

// Hand-unrolled 3-state Kalman filters.
// Uses double precision to preserve numeric fidelity with Python reference flow.

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
        const double h = 0.5 * dt2;

        const double x0 = state_[0];
        const double x1 = state_[1];
        const double x2 = state_[2];

        state_[0] = x0 + dt * x1 + h * x2;
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

        const double t00 = p00 + dt * p10 + h * p20;
        const double t01 = p01 + dt * p11 + h * p21;
        const double t02 = p02 + dt * p12 + h * p22;
        const double t10 = p10 + dt * p20;
        const double t11 = p11 + dt * p21;
        const double t12 = p12 + dt * p22;
        const double t20 = p20;
        const double t21 = p21;
        const double t22 = p22;

        double newP00 = t00 + dt * t01 + h * t02;
        double newP01 = t01 + dt * t02;
        double newP02 = t02;
        double newP10 = t10 + dt * t11 + h * t12;
        double newP11 = t11 + dt * t12;
        double newP12 = t12;
        double newP20 = t20 + dt * t21 + h * t22;
        double newP21 = t21 + dt * t22;
        double newP22 = t22;

        const double qVar = processSigma * processSigma;
        const double q00 = 0.25 * dt4 * qVar;
        const double q01 = 0.5 * dt3 * qVar;
        const double q02 = 0.5 * dt2 * qVar;
        const double q11 = dt2 * qVar;
        const double q12 = dt * qVar;

        newP00 += q00;
        newP01 += q01;
        newP02 += q02;
        newP10 += q01;
        newP11 += q11;
        newP12 += q12;
        newP20 += q02;
        newP21 += q12;
        newP22 += qVar;

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

    void Update(double accelMeasurement) {
        if (!std::isfinite(accelMeasurement)) {
            return;
        }
        constexpr double kMaxAccelResidual = 80.0;
        const double residual =
            std::clamp(accelMeasurement - state_[2], -kMaxAccelResidual, kMaxAccelResidual);
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

        // Joseph-form covariance update for H = [0 0 1], keeps P symmetric/PSD.
        const double m00 = 1.0;
        const double m01 = 0.0;
        const double m02 = -k0;
        const double m10 = 0.0;
        const double m11 = 1.0;
        const double m12 = -k1;
        const double m20 = 0.0;
        const double m21 = 0.0;
        const double m22 = 1.0 - k2;

        const double mp00 = m00 * p00 + m01 * p10 + m02 * p20;
        const double mp01 = m00 * p01 + m01 * p11 + m02 * p21;
        const double mp02 = m00 * p02 + m01 * p12 + m02 * p22;
        const double mp10 = m10 * p00 + m11 * p10 + m12 * p20;
        const double mp11 = m10 * p01 + m11 * p11 + m12 * p21;
        const double mp12 = m10 * p02 + m11 * p12 + m12 * p22;
        const double mp20 = m20 * p00 + m21 * p10 + m22 * p20;
        const double mp21 = m20 * p01 + m21 * p11 + m22 * p21;
        const double mp22 = m20 * p02 + m21 * p12 + m22 * p22;

        double newP00 = mp00 * m00 + mp01 * m01 + mp02 * m02;
        double newP01 = mp00 * m10 + mp01 * m11 + mp02 * m12;
        double newP02 = mp00 * m20 + mp01 * m21 + mp02 * m22;
        double newP10 = mp10 * m00 + mp11 * m01 + mp12 * m02;
        double newP11 = mp10 * m10 + mp11 * m11 + mp12 * m12;
        double newP12 = mp10 * m20 + mp11 * m21 + mp12 * m22;
        double newP20 = mp20 * m00 + mp21 * m01 + mp22 * m02;
        double newP21 = mp20 * m10 + mp21 * m11 + mp22 * m12;
        double newP22 = mp20 * m20 + mp21 * m21 + mp22 * m22;

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
        const double h = 0.5 * dt2;

        const double x0 = state_[0];
        const double x1 = state_[1];
        const double x2 = state_[2];
        state_[0] = x0 + dt * x1 + h * x2;
        state_[1] = x1 + dt * x2;

        const double p00 = covariance_[0][0];
        const double p01 = covariance_[0][1];
        const double p02 = covariance_[0][2];
        const double p10 = covariance_[1][0];
        const double p11 = covariance_[1][1];
        const double p12 = covariance_[1][2];
        const double p20 = covariance_[2][0];
        const double p21 = covariance_[2][1];
        const double p22 = covariance_[2][2];

        const double t00 = p00 + dt * p10 + h * p20;
        const double t01 = p01 + dt * p11 + h * p21;
        const double t02 = p02 + dt * p12 + h * p22;
        const double t10 = p10 + dt * p20;
        const double t11 = p11 + dt * p21;
        const double t12 = p12 + dt * p22;
        const double t20 = p20;
        const double t21 = p21;
        const double t22 = p22;

        double newP00 = t00 + dt * t01 + h * t02;
        double newP01 = t01 + dt * t02;
        double newP02 = t02;
        double newP10 = t10 + dt * t11 + h * t12;
        double newP11 = t11 + dt * t12;
        double newP12 = t12;
        double newP20 = t20 + dt * t21 + h * t22;
        double newP21 = t21 + dt * t22;
        double newP22 = t22;

        const double qVar = processSigma * processSigma;
        const double q00 = 0.25 * dt4 * qVar;
        const double q01 = 0.5 * dt3 * qVar;
        const double q02 = 0.5 * dt2 * qVar;
        const double q11 = dt2 * qVar;
        const double q12 = dt * qVar;

        newP00 += q00;
        newP01 += q01;
        newP02 += q02;
        newP10 += q01;
        newP11 += q11;
        newP12 += q12;
        newP20 += q02;
        newP21 += q12;
        newP22 += qVar;

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

    void Update(double accelMeasurement, double altitudeMeasurement) {
        if (!std::isfinite(accelMeasurement) || !std::isfinite(altitudeMeasurement)) {
            return;
        }
        // Limit single-sample innovation so outliers don't create sharp velocity spikes.
        constexpr double kMaxAccelResidual = 80.0;
        constexpr double kMaxAltitudeResidual = 60.0;
        const double residualAccel =
            std::clamp(accelMeasurement - state_[2], -kMaxAccelResidual, kMaxAccelResidual);
        const double residualAlt =
            std::clamp(altitudeMeasurement - state_[0], -kMaxAltitudeResidual, kMaxAltitudeResidual);

        const double p00 = covariance_[0][0];
        const double p01 = covariance_[0][1];
        const double p02 = covariance_[0][2];
        const double p10 = covariance_[1][0];
        const double p11 = covariance_[1][1];
        const double p12 = covariance_[1][2];
        const double p20 = covariance_[2][0];
        const double p21 = covariance_[2][1];
        const double p22 = covariance_[2][2];

        const double s00 = p22 + accelVariance_;
        const double s01 = p20;
        const double s10 = p02;
        const double s11 = p00 + altitudeVariance_;
        double det = s00 * s11 - s01 * s10;
        if (std::fabs(det) < 1.0e-12) {
            det = (det >= 0.0) ? 1.0e-12 : -1.0e-12;
        }
        const double invDet = 1.0 / det;
        const double invS00 = s11 * invDet;
        const double invS01 = -s01 * invDet;
        const double invS10 = -s10 * invDet;
        const double invS11 = s00 * invDet;

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

        const double mp00 = m00 * p00 + m01 * p10 + m02 * p20;
        const double mp01 = m00 * p01 + m01 * p11 + m02 * p21;
        const double mp02 = m00 * p02 + m01 * p12 + m02 * p22;
        const double mp10 = m10 * p00 + m11 * p10 + m12 * p20;
        const double mp11 = m10 * p01 + m11 * p11 + m12 * p21;
        const double mp12 = m10 * p02 + m11 * p12 + m12 * p22;
        const double mp20 = m20 * p00 + m21 * p10 + m22 * p20;
        const double mp21 = m20 * p01 + m21 * p11 + m22 * p21;
        const double mp22 = m20 * p02 + m21 * p12 + m22 * p22;

        double newP00 = mp00 * m00 + mp01 * m01 + mp02 * m02;
        double newP01 = mp00 * m10 + mp01 * m11 + mp02 * m12;
        double newP02 = mp00 * m20 + mp01 * m21 + mp02 * m22;
        double newP10 = mp10 * m00 + mp11 * m01 + mp12 * m02;
        double newP11 = mp10 * m10 + mp11 * m11 + mp12 * m12;
        double newP12 = mp10 * m20 + mp11 * m21 + mp12 * m22;
        double newP20 = mp20 * m00 + mp21 * m01 + mp22 * m02;
        double newP21 = mp20 * m10 + mp21 * m11 + mp22 * m12;
        double newP22 = mp20 * m20 + mp21 * m21 + mp22 * m22;

        const double accelVar = accelVariance_;
        const double altVar = altitudeVariance_;
        newP00 += accelVar * k00 * k00 + altVar * k01 * k01;
        newP01 += accelVar * k00 * k10 + altVar * k01 * k11;
        newP02 += accelVar * k00 * k20 + altVar * k01 * k21;
        newP10 += accelVar * k10 * k00 + altVar * k11 * k01;
        newP11 += accelVar * k10 * k10 + altVar * k11 * k11;
        newP12 += accelVar * k10 * k20 + altVar * k11 * k21;
        newP20 += accelVar * k20 * k00 + altVar * k21 * k01;
        newP21 += accelVar * k20 * k10 + altVar * k21 * k11;
        newP22 += accelVar * k20 * k20 + altVar * k21 * k21;

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
    double accelVariance_ = 0.25;
    double altitudeVariance_ = 0.25;
};
