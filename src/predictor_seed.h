#pragma once

#include <algorithm>
#include <cmath>

#include "settings.h"

struct PredictorHorizontalVelocityTracker {
    double vx = 0.0;
    double vy = 0.0;
};

enum PredictorSeedConfidenceFlag : uint32_t {
    kPredictorSeedFlagControlActive = 1u << 0,
    kPredictorSeedFlagPositiveVerticalVelocity = 1u << 1,
    kPredictorSeedFlagUsingHorizontalModel = 1u << 2,
    kPredictorSeedFlagHorizontalSpeedCapped = 1u << 3,
    kPredictorSeedFlagZenithClamped = 1u << 4,
    kPredictorSeedFlagAngularRateClamped = 1u << 5,
};

inline void ResetPredictorHorizontalVelocityTracker(PredictorHorizontalVelocityTracker &tracker) {
    tracker.vx = 0.0;
    tracker.vy = 0.0;
}

inline double ClampPredictorZenithRadians(double zenithRadians) {
    const double maxZenithRadians =
        static_cast<double>(settings::flight::kPredictorMaxSeedZenithDeg) * 0.017453292519943295;
    return std::clamp(zenithRadians, -maxZenithRadians, maxZenithRadians);
}

inline double ClampPredictorAngularRate(double angularRateRadPerSec) {
    return std::clamp(angularRateRadPerSec,
                      -static_cast<double>(settings::flight::kPredictorMaxSeedAngularRateRadPerSec),
                      static_cast<double>(settings::flight::kPredictorMaxSeedAngularRateRadPerSec));
}

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

inline double UpdatePredictorHorizontalSpeed(PredictorHorizontalVelocityTracker &tracker,
                                             double accelXMps2,
                                             double accelYMps2,
                                             double dtSeconds,
                                             bool allowIntegration,
                                             double verticalVelocityMps,
                                             double zenithRadians) {
    if (dtSeconds <= 0.0) {
        return std::hypot(tracker.vx, tracker.vy);
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

    const double speed = std::hypot(tracker.vx, tracker.vy);
    const double cap = PredictorHorizontalSpeedCap(verticalVelocityMps, zenithRadians);
    if (speed > cap && speed > 1.0e-9) {
        const double scale = cap / speed;
        tracker.vx *= scale;
        tracker.vy *= scale;
        return cap;
    }
    return speed;
}
