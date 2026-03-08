#pragma once

#include <cmath>

#include "constants.h"
#include "environment_model.h"
#include "math_utils.h"
#include "settings.h"

/// Vehicle properties consumed by the runtime apogee predictor.
///
/// These values are treated as constant over a single flight and are cached
/// internally by `ApogeePredictor` into reciprocal/scaled forms to avoid
/// repeated divisions inside the hot path.

struct ApogeeVehicleParameters {
    double centerOfPressureOffsetMeters = 0.0;  // cp_cg in Python (m)
    double momentOfInertia = 1.0;               // kg*m^2
    double dryMass = 1.0;                       // kg
};

/// Trilinear CFD force table used when aerodynamic prediction is enabled.
///
/// The table is indexed by ACS angle, angle of attack, and Mach number. The
/// arrays are owned by the caller and must remain valid for the lifetime of
/// the predictor configuration that references them.
struct ApogeeForceTable {
    const double *acsAnglesDeg = nullptr;
    const double *atkAnglesDeg = nullptr;
    const double *machNumbers = nullptr;
    const double *axialForces = nullptr;
    const double *normalForces = nullptr;
    int acsCount = 0;
    int atkCount = 0;
    int machCount = 0;

    /// Returns true when the table pointers and axis sizes are populated.
    bool IsValid() const {
        return acsAnglesDeg && atkAnglesDeg && machNumbers && axialForces && normalForces &&
               acsCount > 1 && atkCount > 1 && machCount > 1;
    }
};

/// Predictor state propagated during coast/apogee integration.
///
/// The state intentionally stays compact so the actuation optimizer can run
/// multiple candidate predictions per control interval.
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
    /// Numerical integration modes exposed by the runtime predictor.
    ///
    /// RK4 is the default high-accuracy mode. Midpoint is used in the
    /// actuation sweep to rank candidate flap angles more cheaply, followed by
    /// an RK4 validation of the chosen winner.
    enum class IntegrationMethod {
        RK4,
        Midpoint
    };

    /// Constructs a predictor bound to an environment model and vehicle.
    ApogeePredictor(const EnvironmentModel &environment, const ApogeeVehicleParameters &vehicle)
        : environment_(&environment), vehicle_(vehicle) {
        UpdateCachedVehicleConstants();
    }

    /// Constructs a predictor with default vehicle constants.
    ApogeePredictor() { UpdateCachedVehicleConstants(); }

    /// Rebinds the environment model used for wind and temperature queries.
    void SetEnvironment(const EnvironmentModel &environment) { environment_ = &environment; }

    /// Replaces vehicle parameters and refreshes cached reciprocals/scales.
    void SetVehicleParameters(const ApogeeVehicleParameters &vehicle) {
        vehicle_ = vehicle;
        UpdateCachedVehicleConstants();
    }

    /// Sets the optional CFD force table. A null table falls back to ballistic-only motion.
    void SetForceTable(const ApogeeForceTable *table) { forceTable_ = table; }

    /// Sets the fixed integration step in seconds.
    void SetTimeStep(double dt) { timeStep_ = dt; }

    /// Sets the maximum number of integration steps allowed per prediction.
    void SetMaxIntegrationSteps(int steps) { maxIntegrationSteps_ = (steps > 0) ? steps : 1; }

    /// Predicts apogee using the default RK4 integration path.
    double PredictApogee(const ApogeeState &initialState) {
        return PredictApogee(initialState, IntegrationMethod::RK4);
    }

    /// Predicts apogee using midpoint/RK2 integration.
    ///
    /// This path exists for the flap-angle sweep where candidate ordering is
    /// more important than sub-meter absolute accuracy.
    double PredictApogeeMidpoint(const ApogeeState &initialState) {
        return PredictApogee(initialState, IntegrationMethod::Midpoint);
    }

    /// Predicts apogee using the requested integration method.
    ///
    /// The predictor stops when vertical velocity crosses zero or when the
    /// configured step limit is reached. Zero-crossing refinement is used to
    /// avoid returning the overshot altitude from the final integration step.
    double PredictApogee(const ApogeeState &initialState, IntegrationMethod method) {
        if (initialState.verticalVelocity <= minVerticalVelocityForPrediction_) {
            return initialState.altitudeMeters;
        }
        ApogeeState state = initialState;
        InterpHintSet hints;
        int steps = 0;
        while (state.verticalVelocity > 0.0 && steps < maxIntegrationSteps_) {
            const ApogeeState previousState = state;
            state = IntegrateStep(state, timeStep_, hints, method);
            ++steps;
            if (previousState.verticalVelocity > 0.0 && state.verticalVelocity <= 0.0) {
                return RefineApogeeAtZeroCrossing(previousState, state);
            }
        }
        return state.altitudeMeters;
    }

  private:
    /// Precomputes constant factors used in aerodynamic acceleration updates.
    void UpdateCachedVehicleConstants() {
        invDryMass_ = (vehicle_.dryMass > 0.0) ? (1.0 / vehicle_.dryMass) : 0.0;
        invMomentOfInertia_ = (vehicle_.momentOfInertia > 0.0) ? (1.0 / vehicle_.momentOfInertia) : 0.0;
        aeroMomentScale_ = -0.2 * vehicle_.centerOfPressureOffsetMeters;
    }

    /// Wraps an angle into [-pi, pi] so angle-of-attack sign handling stays stable.
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

    /// Refines the apogee altitude when the last step overshoots the top of flight.
    ///
    /// The integrator brackets the zero crossing of vertical velocity. This
    /// helper linearly interpolates within that final bracket instead of
    /// returning the already-descending altitude from the last state.
    static double RefineApogeeAtZeroCrossing(const ApogeeState &ascendingState,
                                             const ApogeeState &descendingState) {
        const double previousVz = ascendingState.verticalVelocity;
        const double currentVz = descendingState.verticalVelocity;
        const double vzDelta = previousVz - currentVz;
        if (!std::isfinite(previousVz) || !std::isfinite(currentVz) || std::fabs(vzDelta) < 1.0e-9) {
            return std::max(ascendingState.altitudeMeters, descendingState.altitudeMeters);
        }

        const double alpha = std::clamp(previousVz / vzDelta, 0.0, 1.0);
        const double refinedAltitude =
            ascendingState.altitudeMeters +
            alpha * (descendingState.altitudeMeters - ascendingState.altitudeMeters);
        if (!std::isfinite(refinedAltitude)) {
            return std::max(ascendingState.altitudeMeters, descendingState.altitudeMeters);
        }
        return refinedAltitude;
    }

    /// Bracketing result for one interpolation axis in the CFD table.
    struct AxisInterp {
        int lower;
        int upper;
        double t;
    };

    /// Hint state for one interpolation axis so nearby samples can skip binary search.
    struct AxisHint {
        int lower = -1;
        bool valid = false;
    };

    /// Cached interpolation state reused across RK substages.
    ///
    /// The axis hints avoid repeated binary searches and the cached corners
    /// avoid reloading the same 2x2x2 cell when a substep stays in the same
    /// CFD cell.
    struct InterpHintSet {
        AxisHint acs;
        AxisHint atk;
        AxisHint mach;
        int cachedAcsLower = -1;
        int cachedAtkLower = -1;
        int cachedMachLower = -1;
        bool cachedCellValid = false;
        double cachedAxialCorners[8] = {};
        double cachedNormalCorners[8] = {};
    };

    /// State derivative returned by `Evaluate`.
    struct Derivative {
        double altitudeRate;
        double horizontalRate;
        double verticalAcceleration;
        double horizontalAcceleration;
        double zenithRate;
        double angularAcceleration;
    };

    /// Aerodynamic force pair returned after CFD interpolation.
    struct InterpolatedForces {
        double axial;
        double normal;
    };

    /// Linear/angular acceleration bundle used by the integrator.
    struct AccelResult {
        double linearX;
        double linearY;
        double linearZ;
        double angular;
    };

    /// Advances the predictor by one fixed step using the selected integrator.
    ApogeeState IntegrateStep(const ApogeeState &state, double dt, InterpHintSet &hints, IntegrationMethod method) {
        if (method == IntegrationMethod::Midpoint) {
            const double halfDt = dt * 0.5;
            const Derivative k1 = Evaluate(state, hints);
            const Derivative k2 = Evaluate(Apply(state, k1, halfDt), hints);

            // Midpoint/RK2 keeps the candidate sweep cheap while preserving the
            // same state model as the RK4 validation path.
            ApogeeState result = state;
            result.altitudeMeters += dt * k2.altitudeRate;
            result.horizontalDistanceMeters += dt * k2.horizontalRate;
            result.verticalVelocity += dt * k2.verticalAcceleration;
            result.horizontalVelocity += dt * k2.horizontalAcceleration;
            result.zenith += dt * k2.zenithRate;
            result.angularVelocity += dt * k2.angularAcceleration;
            return result;
        }

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

    /// Applies a derivative to a state for an explicit integrator substage.
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

    /// Evaluates the state derivative at the supplied predictor state.
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

    /// Locates the two table samples that bracket `value` on one CFD axis.
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

    /// Reads a single scalar from the flattened 3D CFD table storage.
    static double SampleTable(const double *table, int atkCount, int machCount, int i, int j, int k) {
        const int index = (i * atkCount + j) * machCount + k;
        return table[index];
    }

    /// Loads the eight corner values for one CFD cell into a reusable cache.
    static void PopulateCachedCorners(const ApogeeForceTable &table,
                                      int acsLower,
                                      int atkLower,
                                      int machLower,
                                      double *axialCorners,
                                      double *normalCorners) {
        const int acsUpper = acsLower + 1;
        const int atkUpper = atkLower + 1;
        const int machUpper = machLower + 1;
        axialCorners[0] = SampleTable(table.axialForces, table.atkCount, table.machCount, acsLower, atkLower, machLower);
        axialCorners[1] = SampleTable(table.axialForces, table.atkCount, table.machCount, acsUpper, atkLower, machLower);
        axialCorners[2] = SampleTable(table.axialForces, table.atkCount, table.machCount, acsLower, atkUpper, machLower);
        axialCorners[3] = SampleTable(table.axialForces, table.atkCount, table.machCount, acsUpper, atkUpper, machLower);
        axialCorners[4] = SampleTable(table.axialForces, table.atkCount, table.machCount, acsLower, atkLower, machUpper);
        axialCorners[5] = SampleTable(table.axialForces, table.atkCount, table.machCount, acsUpper, atkLower, machUpper);
        axialCorners[6] = SampleTable(table.axialForces, table.atkCount, table.machCount, acsLower, atkUpper, machUpper);
        axialCorners[7] = SampleTable(table.axialForces, table.atkCount, table.machCount, acsUpper, atkUpper, machUpper);

        normalCorners[0] = SampleTable(table.normalForces, table.atkCount, table.machCount, acsLower, atkLower, machLower);
        normalCorners[1] = SampleTable(table.normalForces, table.atkCount, table.machCount, acsUpper, atkLower, machLower);
        normalCorners[2] = SampleTable(table.normalForces, table.atkCount, table.machCount, acsLower, atkUpper, machLower);
        normalCorners[3] = SampleTable(table.normalForces, table.atkCount, table.machCount, acsUpper, atkUpper, machLower);
        normalCorners[4] = SampleTable(table.normalForces, table.atkCount, table.machCount, acsLower, atkLower, machUpper);
        normalCorners[5] = SampleTable(table.normalForces, table.atkCount, table.machCount, acsUpper, atkLower, machUpper);
        normalCorners[6] = SampleTable(table.normalForces, table.atkCount, table.machCount, acsLower, atkUpper, machUpper);
        normalCorners[7] = SampleTable(table.normalForces, table.atkCount, table.machCount, acsUpper, atkUpper, machUpper);
    }

    /// Performs trilinear interpolation over a cached 2x2x2 cell.
    static double TrilinearFromCorners(const double *corners, double acsT, double atkT, double machT) {
        const double v00 = corners[0] + (corners[1] - corners[0]) * acsT;
        const double v10 = corners[2] + (corners[3] - corners[2]) * acsT;
        const double v01 = corners[4] + (corners[5] - corners[4]) * acsT;
        const double v11 = corners[6] + (corners[7] - corners[6]) * acsT;

        const double v0 = v00 + (v10 - v00) * atkT;
        const double v1 = v01 + (v11 - v01) * atkT;
        return v0 + (v1 - v0) * machT;
    }

    /// Interpolates aerodynamic axial and normal forces from the CFD table.
    ///
    /// When hints are supplied, both axis search results and cell corner values
    /// are cached across RK substages and across nearby candidate evaluations.
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

        if (hints != nullptr) {
            const bool sameCell = hints->cachedCellValid &&
                                  hints->cachedAcsLower == acs.lower &&
                                  hints->cachedAtkLower == atk.lower &&
                                  hints->cachedMachLower == mch.lower;
            if (!sameCell) {
                PopulateCachedCorners(table,
                                      acs.lower,
                                      atk.lower,
                                      mch.lower,
                                      hints->cachedAxialCorners,
                                      hints->cachedNormalCorners);
                hints->cachedAcsLower = acs.lower;
                hints->cachedAtkLower = atk.lower;
                hints->cachedMachLower = mch.lower;
                hints->cachedCellValid = true;
            }
            return {
                TrilinearFromCorners(hints->cachedAxialCorners, acs.t, atk.t, mch.t),
                TrilinearFromCorners(hints->cachedNormalCorners, acs.t, atk.t, mch.t)};
        }

        InterpolatedForces result;
        double axialCorners[8];
        double normalCorners[8];
        PopulateCachedCorners(table, acs.lower, atk.lower, mch.lower, axialCorners, normalCorners);
        result.axial = TrilinearFromCorners(axialCorners, acs.t, atk.t, mch.t);
        result.normal = TrilinearFromCorners(normalCorners, acs.t, atk.t, mch.t);
        return result;
    }

    /// Computes linear and angular acceleration from gravity and aerodynamic loads.
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
        const double relSpeedSquared = relX * relX + relY * relY + relZ * relZ;
        double mach = 0.0;
        if (speedOfSound > 0.0f && relSpeedSquared > 0.0) {
            const float velMag = math_utils::FastSqrt(static_cast<float>(relSpeedSquared));
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
        constexpr double kMinMachForAero = 0.025;
        const double minAeroSpeed = static_cast<double>(speedOfSound) * kMinMachForAero;
        const double minAeroSpeedSquared = minAeroSpeed * minAeroSpeed;
        if (relSpeedSquared >= minAeroSpeedSquared && table != nullptr && table->IsValid() &&
            invDryMass_ > 0.0 && invMomentOfInertia_ > 0.0) {
            // Signed AoA comes from the difference between body angle and
            // relative-velocity angle; atan2+wrap keeps the quadrant handling stable.
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

            const double aeroMoment = normalForceMag * aeroMomentScale_;
            angularAccel = aeroMoment * invMomentOfInertia_;

            if (!liftState) {
                normalForceX = -normalForceX;
                normalForceY = -normalForceY;
                angularAccel = -angularAccel;
            }

            const double aeroAccelX = (axialForceX + normalForceX) * invDryMass_;
            const double aeroAccelY = (axialForceY + normalForceY) * invDryMass_;
            linearAccelX = gravityX + aeroAccelX;
            linearAccelY = gravityY + aeroAccelY;
        }

        return AccelResult{linearAccelX, linearAccelY, linearAccelZ, angularAccel};
    }

    const EnvironmentModel *environment_ = nullptr;
    ApogeeVehicleParameters vehicle_;
    const ApogeeForceTable *forceTable_ = nullptr;
    double invDryMass_ = 1.0;
    double invMomentOfInertia_ = 1.0;
    double aeroMomentScale_ = 0.0;
    double timeStep_ = 0.1;
    int maxIntegrationSteps_ = settings::flight::kApogeePredictorMaxSteps;
    double minVerticalVelocityForPrediction_ = 0.0;
};
