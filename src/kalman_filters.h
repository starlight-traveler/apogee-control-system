#pragma once

#include <algorithm>
#include <cmath>
#include <string.h>

// Compact Kalman filters for translational state estimation.
// Uses double precision to preserve numeric fidelity with Python reference flow.

class KalmanFilterAccel {
  public:
    KalmanFilterAccel() { Reset(); }

    void Configure(double measurementSigma) {
        measurementVariance_ = measurementSigma * measurementSigma;
        Reset();
    }

    void SetMeasurementSigma(double measurementSigma) {
        if (!std::isfinite(measurementSigma) || measurementSigma <= 0.0) {
            return;
        }
        measurementVariance_ = measurementSigma * measurementSigma;
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
        constexpr double kMaxAccelResidual = 250.0;
        const double residual =
            std::clamp(accelMeasurement - state_[2], -kMaxAccelResidual, kMaxAccelResidual);
        // Floor innovation to prevent division by very small numbers near floating-point limits.
        constexpr double kMinInnovation = 1.0e-12;
        double innovation = covariance_[2][2] + measurementVariance_;
        if (innovation < kMinInnovation) {
            innovation = std::max(kMinInnovation, measurementVariance_);
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

    void SetMeasurementSigmas(double accelSigma, double altSigma) {
        if (std::isfinite(accelSigma) && accelSigma > 0.0) {
            accelVariance_ = accelSigma * accelSigma;
        }
        if (std::isfinite(altSigma) && altSigma > 0.0) {
            altitudeVariance_ = altSigma * altSigma;
        }
    }

    void SetBiasProcessSigma(double biasSigma) {
        biasProcessSigma_ = std::max(0.0, biasSigma);
    }

    void Reset() {
        memset(state_, 0, sizeof(state_));
        memset(covariance_, 0, sizeof(covariance_));
        covariance_[0][0] = 1000.0;
        covariance_[1][1] = 1000.0;
        covariance_[2][2] = 10.0;
        covariance_[3][3] = 4.0;
    }

    void Predict(double dt, double processSigma) {
        if (dt <= 0.0) {
            dt = 0.03;
        }
        const double dt2 = dt * dt;
        const double h = 0.5 * dt2;

        const double x0 = state_[0];
        const double x1 = state_[1];
        const double x2 = state_[2];
        state_[0] = x0 + dt * x1 + h * x2;
        state_[1] = x1 + dt * x2;

        const double transition[kStateDim][kStateDim] = {
            {1.0, dt, h, 0.0},
            {0.0, 1.0, dt, 0.0},
            {0.0, 0.0, 1.0, 0.0},
            {0.0, 0.0, 0.0, 1.0},
        };
        double propagated[kStateDim][kStateDim] = {{0.0}};
        double newCovariance[kStateDim][kStateDim] = {{0.0}};

        for (int i = 0; i < kStateDim; ++i) {
            for (int j = 0; j < kStateDim; ++j) {
                double sum = 0.0;
                for (int k = 0; k < kStateDim; ++k) {
                    sum += transition[i][k] * covariance_[k][j];
                }
                propagated[i][j] = sum;
            }
        }

        for (int i = 0; i < kStateDim; ++i) {
            for (int j = 0; j < kStateDim; ++j) {
                double sum = 0.0;
                for (int k = 0; k < kStateDim; ++k) {
                    sum += propagated[i][k] * transition[j][k];
                }
                newCovariance[i][j] = sum;
            }
        }

        const double accelProcessVariance = processSigma * processSigma;
        const double accelDrive[kStateDim] = {h, dt, 1.0, 0.0};
        for (int i = 0; i < kStateDim; ++i) {
            for (int j = 0; j < kStateDim; ++j) {
                newCovariance[i][j] += accelProcessVariance * accelDrive[i] * accelDrive[j];
            }
        }

        const double biasProcessVariance = biasProcessSigma_ * biasProcessSigma_ * dt;
        newCovariance[3][3] += biasProcessVariance;

        StoreSymmetricCovariance(newCovariance);
    }

    bool UpdateAccelOnly(double accelMeasurement, double accelGateSigma = 4.0) {
        constexpr double kMaxAccelResidual = 250.0;
        const double measurementModel[kStateDim] = {0.0, 0.0, 1.0, 1.0};
        return ApplyScalarMeasurement(
            measurementModel,
            accelMeasurement,
            accelVariance_,
            accelGateSigma,
            kMaxAccelResidual);
    }

    bool UpdateAltitudeOnly(double altitudeMeasurement,
                            double altitudeSigmaScale = 1.0,
                            double altitudeGateSigma = 3.5) {
        const double sigmaScale = std::max(1.0, altitudeSigmaScale);
        double effectiveAltitudeVariance = altitudeVariance_ * sigmaScale * sigmaScale;
        if (!std::isfinite(effectiveAltitudeVariance) || effectiveAltitudeVariance <= 0.0) {
            effectiveAltitudeVariance = altitudeVariance_;
        }
        constexpr double kMaxAltitudeResidual = 250.0;
        const double measurementModel[kStateDim] = {1.0, 0.0, 0.0, 0.0};
        return ApplyScalarMeasurement(
            measurementModel,
            altitudeMeasurement,
            effectiveAltitudeVariance,
            altitudeGateSigma,
            kMaxAltitudeResidual);
    }

    bool UpdateAccelAndAltitude(double accelMeasurement,
                                double altitudeMeasurement,
                                double altitudeSigmaScale = 1.0,
                                double altitudeGateSigma = 3.5,
                                double accelGateSigma = 4.0) {
        bool usedMeasurement = false;
        usedMeasurement = UpdateAccelOnly(accelMeasurement, accelGateSigma) || usedMeasurement;
        usedMeasurement =
            UpdateAltitudeOnly(altitudeMeasurement, altitudeSigmaScale, altitudeGateSigma) ||
            usedMeasurement;
        return usedMeasurement;
    }

    void ApplyGroundConstraints(double altitudeSigma, double velocitySigma) {
        const double altitudeMeasurementModel[kStateDim] = {1.0, 0.0, 0.0, 0.0};
        const double velocityMeasurementModel[kStateDim] = {0.0, 1.0, 0.0, 0.0};
        const double altitudeVariance = altitudeSigma * altitudeSigma;
        const double velocityVariance = velocitySigma * velocitySigma;
        ApplyScalarMeasurement(altitudeMeasurementModel, 0.0, altitudeVariance, 0.0, 0.0);
        ApplyScalarMeasurement(velocityMeasurementModel, 0.0, velocityVariance, 0.0, 0.0);
    }

    double Position() const { return state_[0]; }
    double Velocity() const { return state_[1]; }
    double Acceleration() const { return state_[2]; }
    double Bias() const { return state_[3]; }

  private:
    static constexpr int kStateDim = 4;
    static constexpr double kMinInnovationVariance = 1.0e-12;

    bool ApplyScalarMeasurement(const double measurementModel[kStateDim],
                                double measurement,
                                double measurementVariance,
                                double gateSigma,
                                double residualClamp) {
        if (!std::isfinite(measurement) ||
            !std::isfinite(measurementVariance) ||
            measurementVariance <= 0.0) {
            return false;
        }

        double predictedMeasurement = 0.0;
        for (int i = 0; i < kStateDim; ++i) {
            predictedMeasurement += measurementModel[i] * state_[i];
        }
        double residual = measurement - predictedMeasurement;

        double covarianceTimesModel[kStateDim] = {0.0, 0.0, 0.0, 0.0};
        for (int i = 0; i < kStateDim; ++i) {
            for (int j = 0; j < kStateDim; ++j) {
                covarianceTimesModel[i] += covariance_[i][j] * measurementModel[j];
            }
        }

        double innovationVariance = measurementVariance;
        for (int i = 0; i < kStateDim; ++i) {
            innovationVariance += measurementModel[i] * covarianceTimesModel[i];
        }
        innovationVariance = std::max(kMinInnovationVariance, innovationVariance);

        if (gateSigma > 0.0) {
            const double innovationSigma = std::sqrt(innovationVariance);
            if (!std::isfinite(innovationSigma) ||
                std::fabs(residual) > gateSigma * innovationSigma) {
                return false;
            }
        }

        if (residualClamp > 0.0) {
            residual = std::clamp(residual, -residualClamp, residualClamp);
        }

        double kalmanGain[kStateDim] = {0.0, 0.0, 0.0, 0.0};
        for (int i = 0; i < kStateDim; ++i) {
            kalmanGain[i] = covarianceTimesModel[i] / innovationVariance;
            state_[i] += kalmanGain[i] * residual;
        }

        double josephLeft[kStateDim][kStateDim] = {{0.0}};
        for (int i = 0; i < kStateDim; ++i) {
            for (int j = 0; j < kStateDim; ++j) {
                josephLeft[i][j] = (i == j ? 1.0 : 0.0) - kalmanGain[i] * measurementModel[j];
            }
        }

        double leftTimesCovariance[kStateDim][kStateDim] = {{0.0}};
        double newCovariance[kStateDim][kStateDim] = {{0.0}};
        for (int i = 0; i < kStateDim; ++i) {
            for (int j = 0; j < kStateDim; ++j) {
                double sum = 0.0;
                for (int k = 0; k < kStateDim; ++k) {
                    sum += josephLeft[i][k] * covariance_[k][j];
                }
                leftTimesCovariance[i][j] = sum;
            }
        }

        for (int i = 0; i < kStateDim; ++i) {
            for (int j = 0; j < kStateDim; ++j) {
                double sum = 0.0;
                for (int k = 0; k < kStateDim; ++k) {
                    sum += leftTimesCovariance[i][k] * josephLeft[j][k];
                }
                newCovariance[i][j] = sum + measurementVariance * kalmanGain[i] * kalmanGain[j];
            }
        }

        StoreSymmetricCovariance(newCovariance);
        return true;
    }

    void StoreSymmetricCovariance(const double covariance[kStateDim][kStateDim]) {
        for (int i = 0; i < kStateDim; ++i) {
            for (int j = 0; j < kStateDim; ++j) {
                const double value =
                    (i == j) ? covariance[i][j] : 0.5 * (covariance[i][j] + covariance[j][i]);
                covariance_[i][j] = std::isfinite(value) ? value : 0.0;
            }
        }
    }

    double state_[kStateDim] = {0.0, 0.0, 0.0, 0.0};
    double covariance_[kStateDim][kStateDim] = {{0.0}};
    double accelVariance_ = 0.25;
    double altitudeVariance_ = 0.25;
    double biasProcessSigma_ = 0.05;
};
