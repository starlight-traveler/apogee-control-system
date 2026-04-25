#pragma once

#include <algorithm>
#include <cmath>

#include "constants.h"
#include "environment_model.h"
#include "math_utils.h"
#include "predictor_seed.h"
#include "settings.h"

/*
 * Apogee predictor mental model:
 *
 * The flight computer estimates "where the rocket is right now."  This class
 * answers "if the rocket coasted from that state with this flap command, where
 * would vertical velocity reach zero?"
 *
 * The model is intentionally coast-only. It integrates gravity, wind-relative
 * aerodynamic force, simple pitch dynamics, and servo lag. It does not model
 * motor thrust or changing mass during burn, so predictions during Burn are
 * useful mainly for telemetry and should not be treated as final coast truth
 * until burnout/holdoff gates have passed.
 */

/// Vehicle properties consumed by the runtime apogee predictor.
///
/// These values are treated as constant over a single flight and are cached
/// internally by `ApogeePredictor` into reciprocal/scaled forms to avoid
/// repeated divisions inside the hot path.

struct ApogeeVehicleParameters {
    // Distance from center of gravity to center of pressure. This turns normal
    // aerodynamic force into pitch moment.
    double centerOfPressureOffsetMeters = 0.0;  // cp_cg in Python (m)
    // Longitudinal pitch inertia used by the one-axis attitude model.
    double momentOfInertia = 1.0;               // kg*m^2
    // Coast mass. The predictor assumes burn is over and mass is constant.
    double dryMass = 1.0;                       // kg
};

/// Mach-dependent drag scale storage for adaptive predictor.
/// Each bin tracks an independent scale factor learned during coast.
struct MachDependentDragScale {
    static constexpr int kMaxBins = 4;
    float scales[kMaxBins] = {1.0f, 1.0f, 1.0f, 1.0f};

    /// Clamps the configured Mach-bin count to the storage available here.
    static int ActiveBinCount() {
        const int configuredCount = settings::predictor::kMachBinCount;
        if (configuredCount < 1) {
            return 1;
        }
        if (configuredCount > kMaxBins) {
            return kMaxBins;
        }
        return configuredCount;
    }

    /// Interpolates the drag scale at the given Mach number.
    float InterpolateScale(float mach) const {
        if (!settings::predictor::kEnableMachDependentDrag) {
            return scales[0];  // Use first bin as fallback scalar
        }
        const int binCount = ActiveBinCount();
        if (binCount <= 1 || !std::isfinite(mach)) {
            return scales[0];
        }
        // Find the two Mach bins around the current speed, then blend between
        // their learned drag scales instead of jumping at a bin edge.
        int lowerIdx = 0;
        for (int i = 0; i < binCount - 1; ++i) {
            if (mach >= settings::predictor::kMachBinEdges[i]) {
                lowerIdx = i;
            }
        }
        int upperIdx = std::min(lowerIdx + 1, binCount - 1);
        if (lowerIdx == upperIdx) {
            return scales[lowerIdx];
        }
        // Linear interpolation makes the scale change smoothly as Mach changes.
        const float lowerMach = settings::predictor::kMachBinEdges[lowerIdx];
        const float upperMach = settings::predictor::kMachBinEdges[upperIdx];
        const float denom = upperMach - lowerMach;
        if (denom <= 0.0f) {
            return scales[lowerIdx];
        }
        const float t = std::clamp((mach - lowerMach) / denom, 0.0f, 1.0f);
        return scales[lowerIdx] + t * (scales[upperIdx] - scales[lowerIdx]);
    }

    /// Updates neighboring bins with distance-weighted contributions.
    ///
    /// The residual belongs mostly to the closest Mach bin, but the adjacent
    /// bin is updated a little too. That keeps the learned drag curve smooth.
    void AdaptScale(float mach, float targetScale, float alpha) {
        if (!std::isfinite(targetScale)) {
            return;
        }
        const float safeAlpha = std::clamp(std::isfinite(alpha) ? alpha : 0.0f, 0.0f, 1.0f);
        if (!settings::predictor::kEnableMachDependentDrag) {
            scales[0] = std::clamp(scales[0] + safeAlpha * (targetScale - scales[0]),
                                   settings::predictor::kMachDragScaleMin,
                                   settings::predictor::kMachDragScaleMax);
            return;
        }

        const int binCount = ActiveBinCount();
        if (binCount <= 1 || !std::isfinite(mach)) {
            scales[0] = std::clamp(scales[0] + safeAlpha * (targetScale - scales[0]),
                                   settings::predictor::kMachDragScaleMin,
                                   settings::predictor::kMachDragScaleMax);
            return;
        }

        // Use the same bracket as interpolation so learning and prediction see
        // the same local drag curve.
        int lowerIdx = 0;
        for (int i = 0; i < binCount - 1; ++i) {
            if (mach >= settings::predictor::kMachBinEdges[i]) {
                lowerIdx = i;
            }
        }
        int upperIdx = std::min(lowerIdx + 1, binCount - 1);

        // Compute how far this sample is between the lower and upper bins.
        const float lowerMach = settings::predictor::kMachBinEdges[lowerIdx];
        const float upperMach = settings::predictor::kMachBinEdges[upperIdx];
        const float denom = upperMach - lowerMach;

        if (lowerIdx == upperIdx || denom <= 0.0f) {
            // At or beyond the useful bin range, update only the edge bin.
            scales[lowerIdx] = std::clamp(scales[lowerIdx] + safeAlpha * (targetScale - scales[lowerIdx]),
                                          settings::predictor::kMachDragScaleMin,
                                          settings::predictor::kMachDragScaleMax);
            return;
        }

        const float t = std::clamp((mach - lowerMach) / denom, 0.0f, 1.0f);
        const float wLower = 1.0f - t;  // Weight for lower bin
        const float wUpper = t;          // Weight for upper bin

        // Move both bins toward the target scale, weighted by proximity.
        scales[lowerIdx] = std::clamp(scales[lowerIdx] + safeAlpha * wLower * (targetScale - scales[lowerIdx]),
                                      settings::predictor::kMachDragScaleMin,
                                      settings::predictor::kMachDragScaleMax);
        scales[upperIdx] = std::clamp(scales[upperIdx] + safeAlpha * wUpper * (targetScale - scales[upperIdx]),
                                      settings::predictor::kMachDragScaleMin,
                                      settings::predictor::kMachDragScaleMax);
    }

    /// Resets all bins to nominal (1.0).
    void Reset() {
        for (int i = 0; i < kMaxBins; ++i) {
            scales[i] = 1.0f;
        }
    }
};

/// Basic result of a single apogee prediction.
struct PredictResult {
    // AGL altitude where modeled vertical velocity reaches zero.
    double altitude = 0.0;       // Predicted apogee altitude (meters)
    // Integrated time from seed state until vertical velocity crosses zero.
    double timeToApogee = 0.0;   // Time to reach apogee (seconds)
};

/// Result of apogee prediction with uncertainty bounds.
struct ApogeePredictionResult {
    double nominal = 0.0;        // Best estimate
    double lower = 0.0;          // Lower bound (high drag scenario)
    double upper = 0.0;          // Upper bound (low drag scenario)
    double timeToApogee = 0.0;   // Estimated time to apogee (seconds)
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
    // Vertical AGL state. This is the main quantity the controller cares about.
    double altitudeMeters = 0.0;
    // Horizontal distance is propagated for completeness but not used by control output.
    double horizontalDistanceMeters = 0.0;
    // Vertical velocity in the predictor's up-positive axis.
    double verticalVelocity = 0.0;
    // Horizontal speed magnitude in the simplified 2D flight plane.
    double horizontalVelocity = 0.0;
    // Rocket body tilt away from vertical in radians.
    double zenith = 0.0;
    // Pitch angular rate in radians per second.
    double angularVelocity = 0.0;
    // Modeled physical flap angle after servo lag.
    double acsAngleDeg = 0.0;
    // Requested flap command that the physical angle is chasing.
    double acsCommandDeg = 0.0;
};

struct AirRelativeState {
    // Air-relative velocity components in predictor axes.
    double relX = 0.0;
    double relY = 0.0;
    double relZ = 0.0;
    // Atmosphere values at the current altitude.
    double temperatureK = 288.15;
    double densityRatio = 1.0;
    double mach = 0.0;
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

    /// Sets the runtime axial drag correction scale (legacy single-value API).
    void SetAxialDragScale(double scale) {
        axialDragScale_ = std::clamp(scale,
                                     static_cast<double>(settings::flight::kAdaptiveAxialDragScaleMin),
                                     static_cast<double>(settings::flight::kAdaptiveAxialDragScaleMax));
    }

    /// Restores the predictor to its nominal unadapted drag scale.
    void ResetAxialDragScale() {
        axialDragScale_ = 1.0;
        machDragScale_.Reset();
    }

    /// Returns the active runtime axial drag correction scale.
    double AxialDragScale() const { return axialDragScale_; }
    /// Returns flags that describe clamping or degraded validity in the last prediction/model call.
    uint32_t LastPredictionFlags() const { return lastPredictionFlags_; }

    /// Returns reference to the Mach-dependent drag scale for adaptation.
    MachDependentDragScale &MachDragScale() { return machDragScale_; }
    const MachDependentDragScale &MachDragScale() const { return machDragScale_; }

    /// Computes air-relative Mach using the same wind/temperature path as force evaluation.
    double ComputeAirRelativeMach(const ApogeeState &state) const {
        return BuildAirRelativeState(state).mach;
    }

    /// Computes adaptive drag learning time constant based on time-to-apogee.
    /// Fast learning early in coast for quick convergence, slow near apogee for stability.
    static double ComputeAdaptiveTau(double timeToApogee) {
        const double tStart = static_cast<double>(settings::predictor::kMachDragAdaptTauTransitionStart);
        const double tEnd = static_cast<double>(settings::predictor::kMachDragAdaptTauTransitionEnd);
        const double tauEarly = static_cast<double>(settings::predictor::kMachDragAdaptTauSecondsEarly);
        const double tauLate = static_cast<double>(settings::predictor::kMachDragAdaptTauSecondsLate);

        if (timeToApogee >= tStart) {
            return tauEarly;  // Aggressive learning far from apogee
        }
        if (timeToApogee <= tEnd) {
            return tauLate;   // Conservative near apogee
        }
        // Linear interpolation between early and late tau. As apogee gets
        // closer, adaptation slows down so late noise does not rewrite the drag
        // model right before the decision matters most.
        const double t = (timeToApogee - tEnd) / (tStart - tEnd);
        return tauLate + t * (tauEarly - tauLate);
    }

    /// Adapts the Mach-dependent drag scale at the given Mach number.
    /// Uses adaptive time constant based on time-to-apogee for fast early convergence.
    void AdaptMachDragScale(double mach, double residualAccel, double modelAxialAccel,
                            double dtSeconds, double timeToApogee = 10.0) {
        if (!settings::predictor::kEnableMachDependentDrag) {
            return;
        }
        const double minAxialAccel = static_cast<double>(settings::flight::kAdaptiveAxialAccelMinAbsMps2);
        if (std::fabs(modelAxialAccel) < minAxialAccel) {
            return;
        }
        // residual/model is the fractional drag miss. Positive residual means
        // measured acceleration was larger than modeled in this axis, so the
        // drag scale is nudged in that direction through the filtered target.
        const float currentScale = machDragScale_.InterpolateScale(static_cast<float>(mach));
        const double targetScale =
            currentScale + currentScale * (residualAccel / modelAxialAccel);
        // Convert the time constant into a per-loop smoothing factor.
        const double tauSeconds = ComputeAdaptiveTau(timeToApogee);
        const double alpha = (dtSeconds > 0.0 && tauSeconds > 0.0)
                                 ? (1.0 - std::exp(-dtSeconds / tauSeconds))
                                 : 0.0;
        machDragScale_.AdaptScale(static_cast<float>(mach),
                                  static_cast<float>(targetScale),
                                  static_cast<float>(alpha));
    }

    /// Sets the fixed integration step in seconds.
    void SetTimeStep(double dt) {
        if (std::isfinite(dt) && dt > 0.0 && dt <= 1.0) {
            timeStep_ = dt;
        }
    }

    /// Sets the maximum number of integration steps allowed per prediction.
    void SetMaxIntegrationSteps(int steps) { maxIntegrationSteps_ = (steps > 0) ? steps : 1; }

    /// Predicts apogee using the default RK4 integration path (altitude only).
    double PredictApogee(const ApogeeState &initialState) {
        return PredictApogeeWithTime(initialState, IntegrationMethod::RK4).altitude;
    }

    /// Predicts apogee using midpoint/RK2 integration (altitude only).
    ///
    /// This path exists for the flap-angle sweep where candidate ordering is
    /// more important than sub-meter absolute accuracy.
    double PredictApogeeMidpoint(const ApogeeState &initialState) {
        return PredictApogeeWithTime(initialState, IntegrationMethod::Midpoint).altitude;
    }

    /// Predicts apogee and time-to-apogee using the default RK4 integration.
    PredictResult PredictApogeeWithTime(const ApogeeState &initialState) {
        return PredictApogeeWithTime(initialState, IntegrationMethod::RK4);
    }

    /// Predicts apogee and time-to-apogee using the requested integration method.
    ///
    /// The predictor stops when vertical velocity crosses zero or when the
    /// configured step limit is reached. Zero-crossing refinement is used to
    /// avoid returning the overshot altitude from the final integration step.
    PredictResult PredictApogeeWithTime(const ApogeeState &initialState, IntegrationMethod method) {
        /*
         * The integration stops when vertical velocity crosses zero. That crossing
         * is apogee in this reduced model because altitude is the integral of
         * vertical velocity. The final step almost always overshoots the crossing,
         * so the code refines inside that step instead of returning the first
         * descending state.
         */
        lastPredictionFlags_ = 0;
        PredictResult result;
        if (initialState.verticalVelocity <= minVerticalVelocityForPrediction_) {
            result.altitude = initialState.altitudeMeters;
            result.timeToApogee = 0.0;
            return result;
        }
        ApogeeState state = initialState;
        InterpHintSet hints;
        int steps = 0;
        double elapsedTime = 0.0;
        while (state.verticalVelocity > 0.0 && steps < maxIntegrationSteps_) {
            const ApogeeState previousState = state;
            state = IntegrateStep(state, timeStep_, hints, method);
            elapsedTime += timeStep_;
            ++steps;
            if (previousState.verticalVelocity > 0.0 && state.verticalVelocity <= 0.0) {
                return RefineZeroCrossingByBisection(previousState,
                                                    state,
                                                    elapsedTime - timeStep_,
                                                    timeStep_,
                                                    method);
            }
        }
        if (state.verticalVelocity > 0.0 && steps >= maxIntegrationSteps_) {
            lastPredictionFlags_ |= kPredictorSeedFlagPredictionStepLimit |
                                    kPredictorSeedFlagPredictionUncertain;
            // The integrator ran out of allowed steps while still climbing.
            // Return a no-drag gravity continuation as a telemetry bound, but
            // mark it uncertain so actuation code does not treat it as trusted.
            //
            // This value can be high because it ignores drag. That is acceptable
            // as a conservative bound, not as a flap-control truth source.
            result.altitude = state.altitudeMeters +
                              (state.verticalVelocity * state.verticalVelocity) /
                                  (2.0 * constants::kGravity);
            result.timeToApogee = elapsedTime + state.verticalVelocity / constants::kGravity;
            return result;
        }
        result.altitude = state.altitudeMeters;
        result.timeToApogee = elapsedTime;
        return result;
    }

    /// Predicts apogee with uncertainty bounds by perturbing drag only.
    /// Returns nominal prediction plus lower/upper confidence bounds.
    ApogeePredictionResult PredictApogeeWithBounds(const ApogeeState &initialState) {
        /*
         * Bounds are not a formal probability interval. They are a practical
         * sensitivity test: if a small drag perturbation changes apogee a lot,
         * then the optimizer should avoid making a high-consequence command from
         * the nominal number alone.
         */
        ApogeePredictionResult result;
        uint32_t aggregateFlags = 0;

        // Run the normal model once, then run two nearby drag cases so control
        // can see whether the answer is tight or model-sensitive.
        const PredictResult nominalResult = PredictApogeeWithTime(initialState);
        aggregateFlags |= lastPredictionFlags_;
        result.nominal = nominalResult.altitude;
        result.timeToApogee = nominalResult.timeToApogee;

        if (!settings::predictor::kEnableUncertaintyBounds) {
            result.lower = result.nominal;
            result.upper = result.nominal;
            lastPredictionFlags_ = aggregateFlags;
            return result;
        }

        const double originalAxialScale = axialDragScale_;
        const MachDependentDragScale originalMachScale = machDragScale_;

        const auto restoreScales = [&]() {
            axialDragScale_ = originalAxialScale;
            machDragScale_ = originalMachScale;
        };

        const auto predictWithDragPerturbation = [&](double perturbation) {
            axialDragScale_ = originalAxialScale;
            machDragScale_ = originalMachScale;

            if (settings::predictor::kEnableMachDependentDrag) {
                for (int i = 0; i < MachDependentDragScale::kMaxBins; ++i) {
                    machDragScale_.scales[i] =
                        std::clamp(static_cast<float>(originalMachScale.scales[i] * perturbation),
                                   settings::predictor::kMachDragScaleMin,
                                   settings::predictor::kMachDragScaleMax);
                }
            } else {
                axialDragScale_ =
                    std::clamp(originalAxialScale * perturbation,
                               static_cast<double>(settings::flight::kAdaptiveAxialDragScaleMin),
                               static_cast<double>(settings::flight::kAdaptiveAxialDragScaleMax));
            }

            const double prediction = PredictApogee(initialState);
            aggregateFlags |= lastPredictionFlags_;
            return prediction;
        };

        const double perturbFraction =
            static_cast<double>(settings::predictor::kUncertaintyDragPerturbFraction);
        const double highDragPerturbation = std::max(0.0, 1.0 + perturbFraction);
        const double lowDragPerturbation = std::max(0.0, 1.0 - perturbFraction);

        const double highDragPrediction = predictWithDragPerturbation(highDragPerturbation);
        const double lowDragPrediction = predictWithDragPerturbation(lowDragPerturbation);

        restoreScales();

        result.lower = std::min(result.nominal, std::min(highDragPrediction, lowDragPrediction));
        result.upper = std::max(result.nominal, std::max(highDragPrediction, lowDragPrediction));
        lastPredictionFlags_ = aggregateFlags;
        if ((result.upper - result.lower) >
            (2.0 * static_cast<double>(settings::actuation::kApogeeErrorDeadbandMeters))) {
            // A wide high-drag/low-drag bracket means the flap command should
            // be conservative even if the nominal number looks reasonable.
            lastPredictionFlags_ |= kPredictorSeedFlagPredictionUncertain;
        }

        return result;
    }

    /// Estimates time to apogee in seconds.
    /// This is a convenience wrapper; prefer PredictApogeeWithTime() when you
    /// need both altitude and time to avoid redundant integration.
    double EstimateTimeToApogee(const ApogeeState &state) {
        return PredictApogeeWithTime(state, IntegrationMethod::Midpoint).timeToApogee;
    }

    /// Evaluates the current model vertical acceleration at one predictor seed.
    ///
    /// This exists for the adaptive drag update and intentionally avoids a full
    /// apogee integration when only the local model residual is needed.
    double ComputeVerticalAcceleration(const ApogeeState &state, double *axialVerticalAcceleration = nullptr) {
        lastPredictionFlags_ = 0;
        InterpHintSet hints;
        const AccelResult accel = ComputeAcceleration(state, hints);
        if (axialVerticalAcceleration != nullptr) {
            *axialVerticalAcceleration = accel.axialLinearX;
        }
        return accel.linearX;
    }

  private:
    /// Precomputes constant factors used in aerodynamic acceleration updates.
    void UpdateCachedVehicleConstants() {
        /*
         * Cache reciprocals because ComputeAcceleration is called many times per
         * control interval: every flap candidate, every RK substage, and every
         * uncertainty bound. Avoiding repeated divisions matters on the MCU.
         */
        invDryMass_ = (vehicle_.dryMass > 0.0) ? (1.0 / vehicle_.dryMass) : 0.0;
        invMomentOfInertia_ = (vehicle_.momentOfInertia > 0.0) ? (1.0 / vehicle_.momentOfInertia) : 0.0;
        aeroMomentScale_ = -0.2 * vehicle_.centerOfPressureOffsetMeters;
    }

    double ComputeActuationRate(const ApogeeState &state) const {
        const double maxAngle = static_cast<double>(settings::actuation::kServoMaxActuationDeg);
        const double commandDeg = std::clamp(state.acsCommandDeg, 0.0, maxAngle);
        const double angleDeg = std::clamp(state.acsAngleDeg, 0.0, maxAngle);
        const double tauSeconds =
            std::max(static_cast<double>(settings::actuation::kServoLatencySeconds), 1.0e-6);
        // Model the servo as a first-order lag toward the requested angle. This
        // matches the actuator logger's command/effective split and keeps the
        // predictor from assuming instant drag changes.
        return (commandDeg - angleDeg) / tauSeconds;
    }

    /// Wraps an angle into [-pi, pi] so angle-of-attack sign handling stays stable.
    static double WrapToPi(double angle) {
        constexpr double kTwoPi = 6.28318530717958647692;
        if (!std::isfinite(angle)) {
            return 0.0;
        }
        return std::remainder(angle, kTwoPi);
    }

    /// Returns true when all predictor state terms are finite.
    static bool IsFiniteState(const ApogeeState &state) {
        return std::isfinite(state.altitudeMeters) &&
               std::isfinite(state.horizontalDistanceMeters) &&
               std::isfinite(state.verticalVelocity) &&
               std::isfinite(state.horizontalVelocity) &&
               std::isfinite(state.zenith) &&
               std::isfinite(state.angularVelocity) &&
               std::isfinite(state.acsAngleDeg) &&
               std::isfinite(state.acsCommandDeg);
    }

    /// Provides a cheap kinematic fallback when bracket refinement cannot run safely.
    static PredictResult RefineZeroCrossingQuadraticFallback(const ApogeeState &ascendingState,
                                                             const ApogeeState &descendingState,
                                                             double bracketStartTime,
                                                             double dt) {
        PredictResult result;

        const double v0 = ascendingState.verticalVelocity;
        const double v1 = descendingState.verticalVelocity;
        const double denom = v0 - v1;
        if (std::isfinite(v0) && std::isfinite(v1) &&
            std::isfinite(dt) && dt > 0.0 &&
            std::fabs(denom) > 1.0e-9) {
            // Linear interpolation in velocity gives the local time where
            // velocity crosses zero. Then constant acceleration gives altitude
            // at that local time.
            const double alpha = std::clamp(v0 / denom, 0.0, 1.0);
            const double localTime = alpha * dt;
            const double acceleration = (v1 - v0) / dt;

            double altitude = ascendingState.altitudeMeters +
                              v0 * localTime +
                              0.5 * acceleration * localTime * localTime;
            if (!std::isfinite(altitude)) {
                altitude = std::max(ascendingState.altitudeMeters, descendingState.altitudeMeters);
            }

            result.altitude = altitude;
            result.timeToApogee = bracketStartTime + localTime;
            return result;
        }

        result.altitude = std::max(ascendingState.altitudeMeters, descendingState.altitudeMeters);
        // If the bracket is unusable, return the safer of the two altitudes and
        // keep time at the bracket start rather than inventing a precise root.
        result.timeToApogee = bracketStartTime;
        return result;
    }

    /// Bracketing result for one interpolation axis in the CFD table.
    struct AxisInterp {
        // lower/upper are neighboring grid indices; t is the [0,1] blend between them.
        int lower;
        int upper;
        double t;
        // True when the requested value was outside the table and edge-clamped.
        bool clamped;
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
        // Axis hints remember where the last lookup landed.
        AxisHint acs;
        AxisHint atk;
        AxisHint mach;
        // Cached lower corner indices for the currently loaded CFD cell.
        int cachedAcsLower = -1;
        int cachedAtkLower = -1;
        int cachedMachLower = -1;
        bool cachedCellValid = false;
        // Flattened 2x2x2 corner values for axial and normal force.
        double cachedAxialCorners[8] = {};
        double cachedNormalCorners[8] = {};
    };

    /// State derivative returned by `Evaluate`.
    struct Derivative {
        // Time derivatives of ApogeeState in the same field order as the state.
        double altitudeRate;
        double horizontalRate;
        double verticalAcceleration;
        double horizontalAcceleration;
        double zenithRate;
        double angularAcceleration;
        double flapAngleRate;
    };

    /// Aerodynamic force pair returned after CFD interpolation.
    struct InterpolatedForces {
        // Body-axis force magnitudes from the CFD table before density/drag scaling.
        double axial;
        double normal;
        // CFD clamp flags propagated into predictor confidence flags.
        uint32_t flags = 0;
    };

    /// Linear/angular acceleration bundle used by the integrator.
    struct AccelResult {
        double linearX;
        double linearY;
        double linearZ;
        double angular;
        double axialLinearX;
    };

    /// Advances the predictor by one fixed step using the selected integrator.
    ApogeeState IntegrateStep(const ApogeeState &state, double dt, InterpHintSet &hints, IntegrationMethod method) {
        if (method == IntegrationMethod::Midpoint) {
            const double halfDt = dt * 0.5;
            const Derivative k1 = Evaluate(state, hints);
            const Derivative k2 = Evaluate(Apply(state, k1, halfDt), hints);

            // Midpoint/RK2 samples the derivative halfway through the step.
            // It is cheaper than RK4 and good enough for ranking flap angles.
            ApogeeState result = state;
            result.altitudeMeters += dt * k2.altitudeRate;
            result.horizontalDistanceMeters += dt * k2.horizontalRate;
            result.verticalVelocity += dt * k2.verticalAcceleration;
            result.horizontalVelocity += dt * k2.horizontalAcceleration;
            result.zenith += dt * k2.zenithRate;
            result.angularVelocity += dt * k2.angularAcceleration;
            result.acsAngleDeg += dt * k2.flapAngleRate;
            result.acsAngleDeg =
                std::clamp(result.acsAngleDeg,
                           0.0,
                           static_cast<double>(settings::actuation::kServoMaxActuationDeg));
            result.acsCommandDeg =
                std::clamp(result.acsCommandDeg,
                           0.0,
                           static_cast<double>(settings::actuation::kServoMaxActuationDeg));
            return result;
        }

        const double halfDt = dt * 0.5;
        const double sixthDt = dt * (1.0 / 6.0);
        // RK4 evaluates the dynamics at the start, two midpoint estimates, and
        // the end of the interval. The weighted average gives much better
        // behavior when drag changes quickly with Mach/AoA.
        const Derivative k1 = Evaluate(state, hints);
        const Derivative k2 = Evaluate(Apply(state, k1, halfDt), hints);
        const Derivative k3 = Evaluate(Apply(state, k2, halfDt), hints);
        const Derivative k4 = Evaluate(Apply(state, k3, dt), hints);

        // RK4 averages four slope estimates across the step. This reduces the
        // error from rapidly changing drag and attitude without shrinking dt.
        ApogeeState result = state;
        result.altitudeMeters += sixthDt * (k1.altitudeRate + 2.0 * k2.altitudeRate + 2.0 * k3.altitudeRate + k4.altitudeRate);
        result.horizontalDistanceMeters += sixthDt * (k1.horizontalRate + 2.0 * k2.horizontalRate + 2.0 * k3.horizontalRate + k4.horizontalRate);
        result.verticalVelocity += sixthDt * (k1.verticalAcceleration + 2.0 * k2.verticalAcceleration + 2.0 * k3.verticalAcceleration + k4.verticalAcceleration);
        result.horizontalVelocity += sixthDt * (k1.horizontalAcceleration + 2.0 * k2.horizontalAcceleration + 2.0 * k3.horizontalAcceleration + k4.horizontalAcceleration);
        result.zenith += sixthDt * (k1.zenithRate + 2.0 * k2.zenithRate + 2.0 * k3.zenithRate + k4.zenithRate);
        result.angularVelocity += sixthDt * (k1.angularAcceleration + 2.0 * k2.angularAcceleration + 2.0 * k3.angularAcceleration + k4.angularAcceleration);
        result.acsAngleDeg += sixthDt * (k1.flapAngleRate + 2.0 * k2.flapAngleRate +
                                         2.0 * k3.flapAngleRate + k4.flapAngleRate);
        result.acsAngleDeg =
            std::clamp(result.acsAngleDeg,
                       0.0,
                       static_cast<double>(settings::actuation::kServoMaxActuationDeg));
        result.acsCommandDeg =
            std::clamp(result.acsCommandDeg,
                       0.0,
                       static_cast<double>(settings::actuation::kServoMaxActuationDeg));
        return result;
    }

    /// Refines the last ascent bracket until the zero-vertical-velocity crossing is localized.
    PredictResult RefineZeroCrossingByBisection(const ApogeeState &ascendingState,
                                                const ApogeeState &descendingState,
                                                double bracketStartTime,
                                                double dt,
                                                IntegrationMethod method) {
        if (!IsFiniteState(ascendingState) ||
            !IsFiniteState(descendingState) ||
            !std::isfinite(bracketStartTime) ||
            !std::isfinite(dt) ||
            dt <= 0.0 ||
            ascendingState.verticalVelocity <= 0.0 ||
            descendingState.verticalVelocity > 0.0) {
            return RefineZeroCrossingQuadraticFallback(ascendingState,
                                                       descendingState,
                                                       bracketStartTime,
                                                       dt);
        }

        double low = 0.0;
        double high = dt;
        constexpr int kBisectionIterations = 10;
        // Ten bisection iterations localize the root to about dt/1024, which is
        // far below the model uncertainty and cheap compared with another full
        // control sweep.
        for (int i = 0; i < kBisectionIterations; ++i) {
            const double mid = 0.5 * (low + high);
            InterpHintSet localHints;
            const ApogeeState midState = IntegrateStep(ascendingState, mid, localHints, method);
            if (!IsFiniteState(midState)) {
                return RefineZeroCrossingQuadraticFallback(ascendingState,
                                                           descendingState,
                                                           bracketStartTime,
                                                           dt);
            }
            if (midState.verticalVelocity > 0.0) {
                // Still climbing at mid-step, so apogee is later in the bracket.
                low = mid;
            } else {
                // Already descending at mid-step, so apogee is earlier.
                high = mid;
            }
        }

        const double rootTime = 0.5 * (low + high);
        InterpHintSet finalHints;
        const ApogeeState rootState = IntegrateStep(ascendingState, rootTime, finalHints, method);
        if (!IsFiniteState(rootState)) {
            return RefineZeroCrossingQuadraticFallback(ascendingState,
                                                       descendingState,
                                                       bracketStartTime,
                                                       dt);
        }

        PredictResult result;
        result.altitude = std::max(rootState.altitudeMeters, ascendingState.altitudeMeters);
        result.timeToApogee = bracketStartTime + rootTime;
        return result;
    }

    /// Applies a derivative to a state for an explicit integrator substage.
    ApogeeState Apply(const ApogeeState &state, const Derivative &derivative, double dt) const {
        // This helper constructs RK substates. It deliberately clamps flap angle
        // after every substage because the actuator cannot move outside travel.
        ApogeeState result = state;
        result.altitudeMeters += derivative.altitudeRate * dt;
        result.horizontalDistanceMeters += derivative.horizontalRate * dt;
        result.verticalVelocity += derivative.verticalAcceleration * dt;
        result.horizontalVelocity += derivative.horizontalAcceleration * dt;
        result.zenith += derivative.zenithRate * dt;
        result.angularVelocity += derivative.angularAcceleration * dt;
        result.acsAngleDeg += derivative.flapAngleRate * dt;
        result.acsAngleDeg =
            std::clamp(result.acsAngleDeg,
                       0.0,
                       static_cast<double>(settings::actuation::kServoMaxActuationDeg));
        result.acsCommandDeg =
            std::clamp(result.acsCommandDeg,
                       0.0,
                       static_cast<double>(settings::actuation::kServoMaxActuationDeg));
        return result;
    }

    /// Evaluates the state derivative at the supplied predictor state.
    Derivative Evaluate(const ApogeeState &state, InterpHintSet &hints) {
        const AccelResult accel = ComputeAcceleration(state, hints);
        Derivative derivative;
        // Kinematics: altitude derivative is vertical velocity, and velocity
        // derivative is acceleration from gravity/aero.
        derivative.altitudeRate = state.verticalVelocity;
        derivative.horizontalRate = state.horizontalVelocity;
        derivative.verticalAcceleration = accel.linearX;
        derivative.horizontalAcceleration = accel.linearY;
        derivative.zenithRate = state.angularVelocity;
        derivative.angularAcceleration = accel.angular;
        // The flap angle is part of the predicted state so a command change is
        // rolled forward with servo latency instead of appearing instantly.
        derivative.flapAngleRate = ComputeActuationRate(state);
        return derivative;
    }

    /// Locates the two table samples that bracket `value` on one CFD axis.
    ///
    /// Values outside the sampled CFD envelope clamp to the nearest edge cell
    /// rather than extrapolating beyond the available force data.
    static AxisInterp InterpolateAxis(const double *grid, int count, double value, AxisHint *hint) {
        AxisInterp result{0, 0, 0.0, false};
        if (grid == nullptr || count < 2) {
            return result;
        }
        if (hint != nullptr && hint->valid && hint->lower >= 0 && hint->lower + 1 < count) {
            // Consecutive RK substages usually stay in the same CFD cell or a
            // neighboring one, so check the previous bracket before binary search.
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
                result.t = (denom != 0.0)
                               ? std::clamp((value - grid[result.lower]) / denom, 0.0, 1.0)
                               : 0.0;
                hint->lower = low;
                hint->valid = true;
                return result;
            }
        }
        if (value <= grid[0]) {
            // Clamp below the table; caller will mark the prediction as using
            // edge data so actuation can treat it with less confidence.
            result.lower = 0;
            result.upper = 1;
            result.clamped = value < grid[0];
        } else if (value >= grid[count - 1]) {
            // Clamp above the table for the same reason. Extrapolating CFD force
            // beyond sampled AoA/Mach is usually worse than an explicit clamp flag.
            result.lower = count - 2;
            result.upper = count - 1;
            result.clamped = value > grid[count - 1];
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
        result.t = (denom != 0.0)
                       ? std::clamp((value - grid[result.lower]) / denom, 0.0, 1.0)
                       : 0.0;
        if (hint != nullptr) {
            hint->lower = result.lower;
            hint->valid = true;
        }
        return result;
    }

    /// Reads a single scalar from the flattened 3D CFD table storage.
    static double SampleTable(const double *table, int atkCount, int machCount, int i, int j, int k) {
        // Flattening order matches cfd_table.cpp: ACS major, then AoA, then Mach.
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
        // First interpolate along ACS at each AoA/Mach corner, then AoA, then
        // Mach. The order is arbitrary for linear interpolation but mirrors the
        // table layout for readability.
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
        uint32_t flags = 0;
        if (acs.clamped) {
            flags |= kPredictorSeedFlagCfdAcsClamped;
        }
        if (atk.clamped) {
            flags |= kPredictorSeedFlagCfdAtkClamped;
        }
        if (mch.clamped) {
            flags |= kPredictorSeedFlagCfdMachClamped;
        }

        if (hints != nullptr) {
            const bool sameCell = hints->cachedCellValid &&
                                  hints->cachedAcsLower == acs.lower &&
                                  hints->cachedAtkLower == atk.lower &&
                                  hints->cachedMachLower == mch.lower;
            if (!sameCell) {
                // Cache the eight cell corners because axial and normal force
                // interpolation reuse the same 2x2x2 CFD cube.
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
                TrilinearFromCorners(hints->cachedNormalCorners, acs.t, atk.t, mch.t),
                flags};
        }

        InterpolatedForces result;
        double axialCorners[8];
        double normalCorners[8];
        PopulateCachedCorners(table, acs.lower, atk.lower, mch.lower, axialCorners, normalCorners);
        result.axial = TrilinearFromCorners(axialCorners, acs.t, atk.t, mch.t);
        result.normal = TrilinearFromCorners(normalCorners, acs.t, atk.t, mch.t);
        result.flags = flags;
        return result;
    }

    /// Computes linear and angular acceleration from gravity and aerodynamic loads.
    AccelResult ComputeAcceleration(const ApogeeState &state, InterpHintSet &hints) {
        /*
         * Coordinate convention inside the predictor:
         *
         *   x = vertical/up flight direction
         *   y = horizontal cross-range direction
         *   zenith = body tilt away from vertical in the x/y plane
         *
         * The CFD table gives axial and normal force magnitudes in body-related
         * coordinates. This function rotates those forces into the predictor x/y
         * axes, adds gravity, and converts force to acceleration with dry mass.
         */
        const AirRelativeState air = BuildAirRelativeState(state);
        const double relX = air.relX;
        const double relY = air.relY;
        const double densityRatio = air.densityRatio;
        const double mach = air.mach;

        const double gravityX = -static_cast<double>(constants::kGravity);
        const double gravityY = 0.0;
        const double gravityZ = 0.0;
        double linearAccelX = gravityX;
        double linearAccelY = gravityY;
        double linearAccelZ = gravityZ;
        double angularAccel = 0.0;

        const ApogeeForceTable *table = forceTable_;
        constexpr double kMinMachForAero = 0.025;
        if (std::isfinite(mach) && mach >= kMinMachForAero &&
            std::isfinite(densityRatio) && densityRatio > 0.0 &&
            table != nullptr && table->IsValid() &&
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
            lastPredictionFlags_ |= forces.flags;
            /*
             * CFD clamping is allowed so the simulation remains finite, but the
             * flags must follow the result. Clamped AoA/Mach/ACS means the model
             * is using the edge of the known table, not measured force data at
             * the actual condition.
             */

            // Apply Mach-dependent drag scale if enabled, otherwise use the
            // legacy single scalar learned from coast residuals.
            double effectiveDragScale = axialDragScale_;
            if (settings::predictor::kEnableMachDependentDrag) {
                effectiveDragScale *= machDragScale_.InterpolateScale(static_cast<float>(mach));
            }

            // CFD forces are referenced to the nominal density, so scale them
            // by current density ratio before turning them into accelerations.
            const double axialForceMag = forces.axial * effectiveDragScale * densityRatio;
            const double normalForceMag = forces.normal * densityRatio;

            float sinZf = 0.0f;
            float cosZf = 1.0f;
            math_utils::FastSinCos(static_cast<float>(state.zenith), sinZf, cosZf);
            const double sinZ = static_cast<double>(sinZf);
            const double cosZ = static_cast<double>(cosZf);

            double axialForceX = -axialForceMag * cosZ;
            double axialForceY = -axialForceMag * sinZ;
            double normalForceX = -normalForceMag * sinZ;
            double normalForceY = normalForceMag * cosZ;
            /*
             * Force projection:
             *
             * Axial drag points opposite the rocket body axis. Normal force is
             * perpendicular to the body axis and changes sign with AoA. Both are
             * rotated into predictor vertical/horizontal axes before dividing
             * by mass.
             */

            // Normal force creates a pitching moment through CP-CG offset.
            const double aeroMoment = normalForceMag * aeroMomentScale_;
            angularAccel = aeroMoment * invMomentOfInertia_;

            if (!liftState) {
                // The table stores force magnitude; the AoA sign decides which
                // side of the rocket the normal force and moment point toward.
                normalForceX = -normalForceX;
                normalForceY = -normalForceY;
                angularAccel = -angularAccel;
            }

            const double aeroAccelX = (axialForceX + normalForceX) * invDryMass_;
            const double aeroAccelY = (axialForceY + normalForceY) * invDryMass_;
            const double axialAccelX = axialForceX * invDryMass_;
            linearAccelX = gravityX + aeroAccelX;
            linearAccelY = gravityY + aeroAccelY;
            return AccelResult{linearAccelX, linearAccelY, linearAccelZ, angularAccel, axialAccelX};
        }

        return AccelResult{linearAccelX, linearAccelY, linearAccelZ, angularAccel, 0.0};
    }

    /// Builds velocity relative to the air and derives Mach/density inputs.
    AirRelativeState BuildAirRelativeState(const ApogeeState &state) const {
        AirRelativeState air;
        air.relX = state.verticalVelocity;
        air.relY = state.horizontalVelocity;
        air.relZ = 0.0;
        if (environment_ != nullptr) {
            // Aero depends on airspeed, not ground speed, so subtract the wind
            // before computing Mach and angle of attack.
            const math_utils::Vec3 wind = environment_->EffectiveWind();
            air.relX -= static_cast<double>(wind.x);
            air.relY -= static_cast<double>(wind.y);
            air.relZ -= static_cast<double>(wind.z);
            air.temperatureK = environment_->TemperatureKelvin(state.altitudeMeters);
            air.densityRatio = environment_->DensityRatio(state.altitudeMeters);
        }

        if (air.temperatureK > 0.0) {
            // Speed of sound rises with temperature. Mach is the air-relative
            // speed divided by that local speed of sound.
            const float tempK = static_cast<float>(air.temperatureK);
            const float gamma = static_cast<float>(constants::kGamma);
            const float gasConstant = static_cast<float>(constants::kGasConstant);
            const float speedOfSound = math_utils::FastSqrt(gamma * gasConstant * tempK);
            const double relSpeedSquared =
                air.relX * air.relX + air.relY * air.relY + air.relZ * air.relZ;
            if (speedOfSound > 0.0f && relSpeedSquared > 0.0) {
                const float relSpeed = math_utils::FastSqrt(static_cast<float>(relSpeedSquared));
                air.mach = static_cast<double>(relSpeed / speedOfSound);
            }
        }

        // If atmosphere data is invalid, mach stays at zero and aero will be
        // skipped by ComputeAcceleration's minimum-Mach/finite checks.
        return air;
    }

    const EnvironmentModel *environment_ = nullptr;
    ApogeeVehicleParameters vehicle_;
    const ApogeeForceTable *forceTable_ = nullptr;
    double invDryMass_ = 1.0;
    double invMomentOfInertia_ = 1.0;
    double aeroMomentScale_ = 0.0;
    double axialDragScale_ = 1.0;
    MachDependentDragScale machDragScale_;
    double timeStep_ = 0.1;
    int maxIntegrationSteps_ = settings::flight::kApogeePredictorMaxSteps;
    double minVerticalVelocityForPrediction_ = 0.0;
    uint32_t lastPredictionFlags_ = 0;
};
