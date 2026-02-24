#pragma once

#include <cmath>

#include "constants.h"
#include "environment_model.h"
#include "math_utils.h"
#include "settings.h"

// C++ equivalent to apogee.py and apogee_lib.py focused on ballistic prediction.

struct ApogeeVehicleParameters {
    double centerOfPressureOffsetMeters = 0.0;  // cp_cg in Python (m)
    double momentOfInertia = 1.0;               // kg*m^2
    double dryMass = 1.0;                       // kg
};

struct ApogeeForceTable {
    const double *acsAnglesDeg = nullptr;
    const double *atkAnglesDeg = nullptr;
    const double *machNumbers = nullptr;
    const double *axialForces = nullptr;
    const double *normalForces = nullptr;
    int acsCount = 0;
    int atkCount = 0;
    int machCount = 0;

    bool IsValid() const {
        return acsAnglesDeg && atkAnglesDeg && machNumbers && axialForces && normalForces &&
               acsCount > 1 && atkCount > 1 && machCount > 1;
    }
};

struct ApogeeState {
    double altitudeMeters = 0.0;
    double horizontalDistanceMeters = 0.0;
    double verticalVelocity = 0.0;
    double horizontalVelocity = 0.0;
    double zenith = 0.0;
    double angularVelocity = 0.0;
    double acsAngleDeg = 0.0;
};

class ApogeePredictor {
  public:
    ApogeePredictor(const EnvironmentModel &environment, const ApogeeVehicleParameters &vehicle)
        : environment_(&environment), vehicle_(vehicle) {}

    ApogeePredictor() = default;

    void SetEnvironment(const EnvironmentModel &environment) { environment_ = &environment; }
    void SetVehicleParameters(const ApogeeVehicleParameters &vehicle) { vehicle_ = vehicle; }
    void SetForceTable(const ApogeeForceTable *table) { forceTable_ = table; }

    void SetTimeStep(double dt) { timeStep_ = dt; }
    void SetMaxIntegrationSteps(int steps) { maxIntegrationSteps_ = (steps > 0) ? steps : 1; }

    double PredictApogee(const ApogeeState &initialState) {
        if (initialState.verticalVelocity <= minVerticalVelocityForPrediction_) {
            return initialState.altitudeMeters;
        }
        ApogeeState state = initialState;
        InterpHintSet hints;
        int steps = 0;
        while (state.verticalVelocity > 0.0 && steps < maxIntegrationSteps_) {
            state = IntegrateStep(state, timeStep_, hints);
            ++steps;
        }
        return state.altitudeMeters;
    }

  private:
    static double WrapToPi(double angle) {
        constexpr double kPi = 3.14159265358979323846;
        constexpr double kTwoPi = 6.28318530717958647692;
        while (angle > kPi) {
            angle -= kTwoPi;
        }
        while (angle < -kPi) {
            angle += kTwoPi;
        }
        return angle;
    }

    struct AxisInterp {
        int lower;
        int upper;
        double t;
    };

    struct AxisHint {
        int lower = -1;
        bool valid = false;
    };

    struct InterpHintSet {
        AxisHint acs;
        AxisHint atk;
        AxisHint mach;
    };

    struct Derivative {
        double altitudeRate;
        double horizontalRate;
        double verticalAcceleration;
        double horizontalAcceleration;
        double zenithRate;
        double angularAcceleration;
    };

    struct InterpolatedForces {
        double axial;
        double normal;
    };

    struct AccelResult {
        double linearX;
        double linearY;
        double linearZ;
        double angular;
    };

    ApogeeState IntegrateStep(const ApogeeState &state, double dt, InterpHintSet &hints) {
        const double halfDt = dt * 0.5;
        const double sixthDt = dt * (1.0 / 6.0);
        const Derivative k1 = Evaluate(state, hints);
        const Derivative k2 = Evaluate(Apply(state, k1, halfDt), hints);
        const Derivative k3 = Evaluate(Apply(state, k2, halfDt), hints);
        const Derivative k4 = Evaluate(Apply(state, k3, dt), hints);

        ApogeeState result = state;
        result.altitudeMeters += sixthDt * (k1.altitudeRate + 2.0 * k2.altitudeRate + 2.0 * k3.altitudeRate + k4.altitudeRate);
        result.horizontalDistanceMeters += sixthDt * (k1.horizontalRate + 2.0 * k2.horizontalRate + 2.0 * k3.horizontalRate + k4.horizontalRate);
        result.verticalVelocity += sixthDt * (k1.verticalAcceleration + 2.0 * k2.verticalAcceleration + 2.0 * k3.verticalAcceleration + k4.verticalAcceleration);
        result.horizontalVelocity += sixthDt * (k1.horizontalAcceleration + 2.0 * k2.horizontalAcceleration + 2.0 * k3.horizontalAcceleration + k4.horizontalAcceleration);
        result.zenith += sixthDt * (k1.zenithRate + 2.0 * k2.zenithRate + 2.0 * k3.zenithRate + k4.zenithRate);
        result.angularVelocity += sixthDt * (k1.angularAcceleration + 2.0 * k2.angularAcceleration + 2.0 * k3.angularAcceleration + k4.angularAcceleration);
        return result;
    }

    ApogeeState Apply(const ApogeeState &state, const Derivative &derivative, double dt) const {
        ApogeeState result = state;
        result.altitudeMeters += derivative.altitudeRate * dt;
        result.horizontalDistanceMeters += derivative.horizontalRate * dt;
        result.verticalVelocity += derivative.verticalAcceleration * dt;
        result.horizontalVelocity += derivative.horizontalAcceleration * dt;
        result.zenith += derivative.zenithRate * dt;
        result.angularVelocity += derivative.angularAcceleration * dt;
        return result;
    }

    Derivative Evaluate(const ApogeeState &state, InterpHintSet &hints) {
        const AccelResult accel = ComputeAcceleration(state, hints);
        Derivative derivative;
        derivative.altitudeRate = state.verticalVelocity;
        derivative.horizontalRate = state.horizontalVelocity;
        derivative.verticalAcceleration = accel.linearX;
        derivative.horizontalAcceleration = accel.linearY;
        derivative.zenithRate = state.angularVelocity;
        derivative.angularAcceleration = accel.angular;
        return derivative;
    }

    static AxisInterp InterpolateAxis(const double *grid, int count, double value, AxisHint *hint) {
        AxisInterp result{0, 0, 0.0};
        if (grid == nullptr || count < 2) {
            return result;
        }
        if (hint != nullptr && hint->valid && hint->lower >= 0 && hint->lower + 1 < count) {
            int low = hint->lower;
            if (value < grid[low]) {
                while (low > 0 && value < grid[low]) {
                    --low;
                }
            } else if (value > grid[low + 1]) {
                while (low + 2 < count && value > grid[low + 1]) {
                    ++low;
                }
            }
            if (value >= grid[low] && value <= grid[low + 1]) {
                result.lower = low;
                result.upper = low + 1;
                const double denom = grid[result.upper] - grid[result.lower];
                result.t = (denom != 0.0) ? (value - grid[result.lower]) / denom : 0.0;
                hint->lower = low;
                hint->valid = true;
                return result;
            }
        }
        if (value <= grid[0]) {
            result.lower = 0;
            result.upper = 1;
        } else if (value >= grid[count - 1]) {
            result.lower = count - 2;
            result.upper = count - 1;
        } else {
            int low = 0;
            int high = count - 1;
            while (high - low > 1) {
                const int mid = (low + high) / 2;
                if (grid[mid] <= value) {
                    low = mid;
                } else {
                    high = mid;
                }
            }
            result.lower = low;
            result.upper = low + 1;
        }
        const double denom = grid[result.upper] - grid[result.lower];
        result.t = (denom != 0.0) ? (value - grid[result.lower]) / denom : 0.0;
        if (hint != nullptr) {
            hint->lower = result.lower;
            hint->valid = true;
        }
        return result;
    }

    static double SampleTable(const double *table, int atkCount, int machCount, int i, int j, int k) {
        const int index = (i * atkCount + j) * machCount + k;
        return table[index];
    }

    static InterpolatedForces InterpolateForces(const ApogeeForceTable &table,
                                                double acsDeg,
                                                double atkDeg,
                                                double mach,
                                                InterpHintSet *hints) {
        AxisHint *acsHint = (hints != nullptr) ? &hints->acs : nullptr;
        AxisHint *atkHint = (hints != nullptr) ? &hints->atk : nullptr;
        AxisHint *mchHint = (hints != nullptr) ? &hints->mach : nullptr;
        const AxisInterp acs = InterpolateAxis(table.acsAnglesDeg, table.acsCount, acsDeg, acsHint);
        const AxisInterp atk = InterpolateAxis(table.atkAnglesDeg, table.atkCount, atkDeg, atkHint);
        const AxisInterp mch = InterpolateAxis(table.machNumbers, table.machCount, mach, mchHint);

        auto interpValues = [&](const double *values) -> double {
            const double v000 = SampleTable(values, table.atkCount, table.machCount, acs.lower, atk.lower, mch.lower);
            const double v100 = SampleTable(values, table.atkCount, table.machCount, acs.upper, atk.lower, mch.lower);
            const double v010 = SampleTable(values, table.atkCount, table.machCount, acs.lower, atk.upper, mch.lower);
            const double v110 = SampleTable(values, table.atkCount, table.machCount, acs.upper, atk.upper, mch.lower);
            const double v001 = SampleTable(values, table.atkCount, table.machCount, acs.lower, atk.lower, mch.upper);
            const double v101 = SampleTable(values, table.atkCount, table.machCount, acs.upper, atk.lower, mch.upper);
            const double v011 = SampleTable(values, table.atkCount, table.machCount, acs.lower, atk.upper, mch.upper);
            const double v111 = SampleTable(values, table.atkCount, table.machCount, acs.upper, atk.upper, mch.upper);

            const double v00 = v000 + (v100 - v000) * acs.t;
            const double v10 = v010 + (v110 - v010) * acs.t;
            const double v01 = v001 + (v101 - v001) * acs.t;
            const double v11 = v011 + (v111 - v011) * acs.t;

            const double v0 = v00 + (v10 - v00) * atk.t;
            const double v1 = v01 + (v11 - v01) * atk.t;

            return v0 + (v1 - v0) * mch.t;
        };

        InterpolatedForces result;
        result.axial = interpValues(table.axialForces);
        result.normal = interpValues(table.normalForces);
        return result;
    }

    AccelResult ComputeAcceleration(const ApogeeState &state, InterpHintSet &hints) {
        const double velX = state.verticalVelocity;
        const double velY = state.horizontalVelocity;
        double relX = velX;
        double relY = velY;
        double relZ = 0.0;
        double temperature = 288.15;
        if (environment_ != nullptr) {
            const math_utils::Vec3 wind = environment_->GradientWind();
            relX -= static_cast<double>(wind.x);
            relY -= static_cast<double>(wind.y);
            relZ -= static_cast<double>(wind.z);
            temperature = environment_->TemperatureKelvin(state.altitudeMeters);
        }
        float speedOfSound = 0.0f;
        if (temperature > 0.0) {
            const float tempF = static_cast<float>(temperature);
            const float gamma = static_cast<float>(constants::kGamma);
            const float gasConstant = static_cast<float>(constants::kGasConstant);
            speedOfSound = math_utils::FastSqrt(gamma * gasConstant * tempF);
        }
        double mach = 0.0;
        if (speedOfSound > 0.0f) {
            const float velMag =
                math_utils::FastSqrt(static_cast<float>(relX * relX + relY * relY + relZ * relZ));
            mach = static_cast<double>(velMag / speedOfSound);
        }

        const double gravityX = -static_cast<double>(constants::kGravity);
        const double gravityY = 0.0;
        const double gravityZ = 0.0;
        double linearAccelX = gravityX;
        double linearAccelY = gravityY;
        double linearAccelZ = gravityZ;
        double angularAccel = 0.0;

        const ApogeeForceTable *table = forceTable_;
        if (mach >= 0.025 && table != nullptr && table->IsValid() &&
            vehicle_.dryMass > 0.0 && vehicle_.momentOfInertia > 0.0) {
            // Signed AoA from body axis angle minus relative-velocity angle.
            // Use atan2 + wrap so crosswind/quadrant behavior is physically consistent.
            const double velocityAngle = static_cast<double>(math_utils::FastAtan2(static_cast<float>(relY),
                                                                                    static_cast<float>(relX)));
            const double signedAtkAngle = WrapToPi(state.zenith - velocityAngle);
            const bool liftState = (signedAtkAngle >= 0.0);
            const double atkAngle = std::fabs(signedAtkAngle);

            constexpr double kRadToDeg = 57.29577951308232;
            const double atkDeg = atkAngle * kRadToDeg;
            const double acsDeg = state.acsAngleDeg;

            const InterpolatedForces forces = InterpolateForces(*table, acsDeg, atkDeg, mach, &hints);
            const double axialForceMag = forces.axial;
            const double normalForceMag = forces.normal;

            float sinZf = 0.0f;
            float cosZf = 1.0f;
            math_utils::FastSinCos(static_cast<float>(state.zenith), sinZf, cosZf);
            const double sinZ = static_cast<double>(sinZf);
            const double cosZ = static_cast<double>(cosZf);

            double axialForceX = -axialForceMag * cosZ;
            double axialForceY = -axialForceMag * sinZ;
            double normalForceX = -normalForceMag * sinZ;
            double normalForceY = normalForceMag * cosZ;

            double aeroMoment = -normalForceMag * vehicle_.centerOfPressureOffsetMeters;
            aeroMoment *= 0.2;
            angularAccel = aeroMoment / vehicle_.momentOfInertia;

            if (!liftState) {
                normalForceX = -normalForceX;
                normalForceY = -normalForceY;
                angularAccel = -angularAccel;
            }

            const double invMass = 1.0 / vehicle_.dryMass;
            const double aeroAccelX = (axialForceX + normalForceX) * invMass;
            const double aeroAccelY = (axialForceY + normalForceY) * invMass;
            linearAccelX = gravityX + aeroAccelX;
            linearAccelY = gravityY + aeroAccelY;
        }

        return AccelResult{linearAccelX, linearAccelY, linearAccelZ, angularAccel};
    }

    const EnvironmentModel *environment_ = nullptr;
    ApogeeVehicleParameters vehicle_;
    const ApogeeForceTable *forceTable_ = nullptr;
    double timeStep_ = 0.1;
    int maxIntegrationSteps_ = settings::flight::kApogeePredictorMaxSteps;
    double minVerticalVelocityForPrediction_ = 0.0;
};
