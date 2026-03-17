#pragma once

#include <algorithm>
#include <cmath>

#include "math_utils.h"
#include "settings.h"

/// Predictor-only horizontal velocity state.
///
/// The estimator intentionally does not publish integrated XY state because it
/// has no horizontal measurement update. This tracker exists only to seed the
/// apogee predictor with a bounded horizontal speed estimate.
struct PredictorHorizontalVelocityTracker {
    double vx = 0.0;
    double vy = 0.0;
};

/// Bitfield stored in telemetry to explain which predictor seed guards were active.
enum PredictorSeedConfidenceFlag : uint32_t {
    kPredictorSeedFlagControlActive = 1u << 0,
    kPredictorSeedFlagPositiveVerticalVelocity = 1u << 1,
    kPredictorSeedFlagUsingHorizontalModel = 1u << 2,
    kPredictorSeedFlagHorizontalSpeedCapped = 1u << 3,
    kPredictorSeedFlagZenithClamped = 1u << 4,
    kPredictorSeedFlagAngularRateClamped = 1u << 5,
};

/// Clears the horizontal predictor seed state.
inline void ResetPredictorHorizontalVelocityTracker(PredictorHorizontalVelocityTracker &tracker) {
    tracker.vx = 0.0;
    tracker.vy = 0.0;
}

/// Returns true when a predictor seed sample is recent enough to trust.
///
/// Large or non-positive `dt` values usually indicate stale timing or skipped
/// samples, in which case the predictor should degrade toward a simpler seed.
inline bool PredictorSeedHasFreshSample(double dtSeconds) {
    return std::isfinite(dtSeconds) && dtSeconds > 1.0e-4 && dtSeconds <= 0.25;
}

/// Clamps predictor zenith to the configured safe operating envelope.
inline double ClampPredictorZenithRadians(double zenithRadians) {
    const double maxZenithRadians =
        static_cast<double>(settings::flight::kPredictorMaxSeedZenithDeg) * 0.017453292519943295;
    return std::clamp(zenithRadians, -maxZenithRadians, maxZenithRadians);
}

/// Normalizes invalid zenith input to zero before applying the configured clamp.
inline double SanitizePredictorZenithRadians(double zenithRadians) {
    if (!std::isfinite(zenithRadians)) {
        return 0.0;
    }
    return ClampPredictorZenithRadians(zenithRadians);
}

/// Clamps predictor angular rate to the configured safe operating envelope.
inline double ClampPredictorAngularRate(double angularRateRadPerSec) {
    return std::clamp(angularRateRadPerSec,
                      -static_cast<double>(settings::flight::kPredictorMaxSeedAngularRateRadPerSec),
                      static_cast<double>(settings::flight::kPredictorMaxSeedAngularRateRadPerSec));
}

/// Computes angular rate from successive zenith samples when the timing is fresh.
inline double ComputePredictorAngularRate(double currentZenithRadians,
                                          double previousZenithRadians,
                                          double dtSeconds) {
    if (!PredictorSeedHasFreshSample(dtSeconds) ||
        !std::isfinite(currentZenithRadians) ||
        !std::isfinite(previousZenithRadians)) {
        return 0.0;
    }
    return (currentZenithRadians - previousZenithRadians) / dtSeconds;
}

/// Computes the horizontal-speed cap implied by vertical speed and tilt.
///
/// This prevents a noisy tilt estimate from exploding the horizontal seed speed
/// and causing excessive drag prediction.
inline double PredictorHorizontalSpeedCap(double verticalVelocityMps, double zenithRadians) {
    const double effectiveZenith =
        std::min(std::fabs(zenithRadians),
                 static_cast<double>(settings::flight::kPredictorMaxSeedZenithDeg) * 0.017453292519943295);
    const double tiltCap = std::fabs(verticalVelocityMps) * std::tan(effectiveZenith) +
                           static_cast<double>(settings::flight::kPredictorHorizontalSpeedMarginMps);
    return std::clamp(tiltCap,
                      static_cast<double>(settings::flight::kPredictorMinHorizontalSpeedCapMps),
                      static_cast<double>(settings::flight::kPredictorMaxHorizontalSpeedMps));
}

/// Estimates horizontal speed directly from the measured tilt/vertical velocity.
///
/// This gives the predictor an immediate lower bound on cross-axis motion
/// instead of waiting for the bounded XY acceleration integrator to ramp up.
inline double PredictorGeometricHorizontalSpeed(double verticalVelocityMps, double zenithRadians) {
    if (!std::isfinite(verticalVelocityMps) || !std::isfinite(zenithRadians)) {
        return 0.0;
    }

    const double verticalSpeed = std::fabs(verticalVelocityMps);
    if (verticalSpeed <= 0.0) {
        return 0.0;
    }

    float sinZenith = 0.0f;
    float cosZenith = 1.0f;
    math_utils::FastSinCos(static_cast<float>(zenithRadians), sinZenith, cosZenith);
    const double clampedCosZenith = std::clamp(std::fabs(static_cast<double>(cosZenith)), 0.1, 1.0);
    const double speedAlongAxis = verticalSpeed / clampedCosZenith;
    const double horizontalSpeedSquared = speedAlongAxis * speedAlongAxis - verticalSpeed * verticalSpeed;
    return (horizontalSpeedSquared > 0.0) ? math_utils::FastSqrt(horizontalSpeedSquared) : 0.0;
}

/// Merges the tracked XY-speed estimate with a capped tilt-derived lower bound.
inline double ResolvePredictorHorizontalSpeed(double trackedHorizontalSpeedMps,
                                              double verticalVelocityMps,
                                              double zenithRadians) {
    const double cap = PredictorHorizontalSpeedCap(verticalVelocityMps, zenithRadians);
    const double geometricSpeed = PredictorGeometricHorizontalSpeed(verticalVelocityMps, zenithRadians);
    return std::clamp(std::max(trackedHorizontalSpeedMps, geometricSpeed), 0.0, cap);
}

/// Updates the bounded predictor-only horizontal speed estimate.
///
/// The estimate integrates inertial XY acceleration with exponential decay,
/// then clamps the result against a tilt-based cap. It is deliberately more
/// conservative than a free-running navigation solution.
inline double UpdatePredictorHorizontalSpeed(PredictorHorizontalVelocityTracker &tracker,
                                             double accelXMps2,
                                             double accelYMps2,
                                             double dtSeconds,
                                             bool allowIntegration,
                                             double verticalVelocityMps,
                                             double zenithRadians) {
    const auto currentSpeed = [&tracker]() {
        return math_utils::FastSqrt(tracker.vx * tracker.vx + tracker.vy * tracker.vy);
    };
    if (dtSeconds <= 0.0) {
        return currentSpeed();
    }

    const double clampedDt = std::min(dtSeconds, 0.25);
    const double accelLimit = static_cast<double>(settings::flight::kPredictorHorizontalAccelLimitMps2);
    const double ax = std::clamp(accelXMps2, -accelLimit, accelLimit);
    const double ay = std::clamp(accelYMps2, -accelLimit, accelLimit);
    const double decayTau = static_cast<double>(settings::flight::kPredictorHorizontalDecayTauSeconds);
    const double decay = (decayTau > 0.0) ? std::exp(-clampedDt / decayTau) : 0.0;

    if (allowIntegration && verticalVelocityMps > 0.0) {
        tracker.vx = (tracker.vx + ax * clampedDt) * decay;
        tracker.vy = (tracker.vy + ay * clampedDt) * decay;
    } else {
        tracker.vx *= decay;
        tracker.vy *= decay;
    }

    const double speedSquared = tracker.vx * tracker.vx + tracker.vy * tracker.vy;
    const double cap = PredictorHorizontalSpeedCap(verticalVelocityMps, zenithRadians);
    const double capSquared = cap * cap;
    if (speedSquared > capSquared && speedSquared > 1.0e-18) {
        const double speed = math_utils::FastSqrt(speedSquared);
        const double scale = cap / speed;
        tracker.vx *= scale;
        tracker.vy *= scale;
        return cap;
    }
    return math_utils::FastSqrt(speedSquared);
}
