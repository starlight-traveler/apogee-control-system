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
    kPredictorSeedFlagCoastEntryBlendActive = 1u << 6,
    kPredictorSeedFlagCfdAcsClamped = 1u << 7,
    kPredictorSeedFlagCfdAtkClamped = 1u << 8,
    kPredictorSeedFlagCfdMachClamped = 1u << 9,
    kPredictorSeedFlagPredictionStepLimit = 1u << 10,
    kPredictorSeedFlagPredictionUncertain = 1u << 11,
};

inline bool PredictorFlagsHasCfdClamp(uint32_t flags) {
    return (flags & (kPredictorSeedFlagCfdAcsClamped |
                     kPredictorSeedFlagCfdAtkClamped |
                     kPredictorSeedFlagCfdMachClamped)) != 0u;
}

inline bool PredictorFlagsHasModelInvalidity(uint32_t flags) {
    return PredictorFlagsHasCfdClamp(flags) || (flags & kPredictorSeedFlagPredictionStepLimit) != 0u;
}

/// Clears the horizontal predictor seed state.
inline void ResetPredictorHorizontalVelocityTracker(PredictorHorizontalVelocityTracker &tracker) {
    tracker.vx = 0.0;
    tracker.vy = 0.0;
}

/// Returns true when predictor seed timing is recent enough to trust.
///
/// Large or non-positive `dt` values usually indicate stale timing or skipped
/// samples, in which case the predictor should degrade toward a simpler seed.
inline bool PredictorSeedHasFreshSample(double dtSeconds) {
    return std::isfinite(dtSeconds) && dtSeconds > 1.0e-4 && dtSeconds <= 0.25;
}

/// Returns true when a predictor seed can safely use accelerometer-driven terms.
///
/// XY seed integration and adaptive drag learning should require both fresh
/// timing and a genuinely fresh accelerometer measurement, not just cached data.
inline bool PredictorSeedHasFreshAccelSample(double dtSeconds, bool hasFreshAccelMeasurement) {
    return hasFreshAccelMeasurement && PredictorSeedHasFreshSample(dtSeconds);
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

/// Returns the coast-entry soft-start applied to zenith/AoA predictor seeding.
///
/// Corrected attitude can step noticeably at burnout on historical data and
/// during the estimator handoff into coast. Ramp the predictor tilt from a
/// conservative initial factor back to the full measured zenith over a short
/// post-burnout window so early-coast apogee calls do not overreact.
inline double ComputePredictorCoastEntryZenithBlend(double timeSinceBurnoutSeconds) {
    const double initialBlend = std::clamp(
        static_cast<double>(settings::flight::kPredictorCoastEntryZenithInitialBlendFactor),
        0.0,
        1.0);
    const double rampSeconds =
        static_cast<double>(settings::flight::kPredictorCoastEntryZenithRampSeconds);
    if (initialBlend >= 0.999999 || rampSeconds <= 0.0 || !std::isfinite(timeSinceBurnoutSeconds)) {
        return 1.0;
    }
    if (timeSinceBurnoutSeconds <= 0.0) {
        return initialBlend;
    }
    const double t = std::clamp(timeSinceBurnoutSeconds / rampSeconds, 0.0, 1.0);
    return initialBlend + (1.0 - initialBlend) * t;
}

/// Applies the coast-entry soft-start to a predictor tilt-like quantity.
inline double ApplyPredictorCoastEntryZenithBlend(double value, double timeSinceBurnoutSeconds) {
    return value * ComputePredictorCoastEntryZenithBlend(timeSinceBurnoutSeconds);
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

/// Resolves the bounded predictor-only horizontal speed estimate.
///
/// The tracker is intentionally conservative: do not force a geometric lower
/// bound from body tilt because that can erase real angle-of-attack whenever
/// attitude and velocity direction diverge.
inline double ResolvePredictorHorizontalSpeed(double trackedHorizontalSpeedMps,
                                              double verticalVelocityMps,
                                              double zenithRadians) {
    (void)verticalVelocityMps;
    (void)zenithRadians;
    const double cap = PredictorHorizontalSpeedCap(verticalVelocityMps, zenithRadians);
    return std::clamp(trackedHorizontalSpeedMps, 0.0, cap);
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
