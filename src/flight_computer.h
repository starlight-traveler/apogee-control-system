#pragma once

#include <Arduino.h>

#include "apogee_model.h"
#include "environment_model.h"
#include "kalman_filters.h"
#include "math_utils.h"
#include "predictor_seed.h"

/// Raw sensor/control sample written to logs and consumed by the estimator.

struct SensorData {
    float timestamp = 0.0f;
    float altitudeFeet = 0.0f;
    float accelBNO[3] = {0.0f, 0.0f, 0.0f};
    float accelICM[3] = {0.0f, 0.0f, 0.0f};
    float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float gyro[3] = {0.0f, 0.0f, 0.0f};
    float icmQuaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float icmYprDeg[3] = {0.0f, 0.0f, 0.0f};
    float altimeterSigmaScale = 1.0f;
    float altimeterGateSigma = 3.5f;
    float autoCommandDeg = 0.0f;
    float optimizerBestPredictedApogeeM = 0.0f;
    float optimizerBestCost = 0.0f;
    float optimizerTimeToApogeeS = 0.0f;
    float actuationIsSettling = 0.0f;
    float predictorSeedHorizontalSpeedMps = 0.0f;
    float predictorSeedClampedZenithRad = 0.0f;
    float predictorSeedClampedAngularRateRadPerSec = 0.0f;
    float predictorSeedConfidenceFlags = 0.0f;
    bool hasQuaternion = false;
    bool hasIcmQuaternion = false;
    bool hasIcmYpr = false;
};

/// Published estimator state used by telemetry, replay, and actuation logic.
struct FilteredState {
    float time = 0.0f;
    float position[3] = {0.0f, 0.0f, 0.0f};
    float velocity[3] = {0.0f, 0.0f, 0.0f};
    float acceleration[3] = {0.0f, 0.0f, 0.0f};
    float inertialAcceleration[3] = {0.0f, 0.0f, 0.0f};
    float zenith = 0.0f;
    float apogeeEstimate = 0.0f;
};

/// High-level flight phases used for event detection and control gating.
enum class FlightStatus { Ground, Burn, Coast, Overshoot, Descent };

/// Flight-state estimator and apogee predictor coordinator.
class FlightComputer {
  public:
    /// Constructs the flight computer in a reset state.
    FlightComputer();

    /// Configures filters, environment, and apogee predictor dependencies.
    void Begin(double sigmaAccelXY,
               double sigmaAccelZ,
               double sigmaAltimeter,
               double processXY,
               double processZ,
               double apogeeTargetMeters,
               const EnvironmentModel::Config &environmentConfig,
               const ApogeeVehicleParameters &vehicleParameters,
               const ApogeeForceTable *forceTable = nullptr);

    /// Reapplies runtime-editable predictor dependencies without resetting the estimator.
    void ReconfigurePredictor(const EnvironmentModel::Config &environmentConfig,
                              const ApogeeVehicleParameters &vehicleParameters,
                              const ApogeeForceTable *forceTable = nullptr);

    /// Ingests one sensor sample and publishes the latest filtered state.
    ///
    /// @return false when the sample cannot be used, usually because no valid
    /// accelerometer source is available.
    bool Update(const SensorData &data, FilteredState &output);

    /// Compatibility no-op retained for older call sites.
    void SetSerialReportingEnabled(bool enabled) { (void)enabled; }

    /// Returns the current flight phase.
    FlightStatus Status() const { return status_; }
    /// Returns the latest predicted apogee in meters.
    double ApogeePrediction() const { return lastApogeePrediction_; }
    /// Returns true once apogee has been latched on descent transition.
    bool ApogeeReached() const { return apogeeRecorded_; }
    /// Returns the observed apogee altitude once available.
    double ApogeeAltitude() const { return apogeeAltitude_; }
    /// Returns the burn start timestamp in seconds.
    double BurnTime() const { return burnTimestamp_; }
    /// Returns the burnout timestamp in seconds.
    double BurnoutTime() const { return burnoutTimestamp_; }
    /// Returns the apogee timestamp in seconds.
    double ApogeeTime() const { return apogeeTimestamp_; }

  private:
    /// Resets filter state, phase counters, and predictor-side caches.
    void ResetInternalState();
    /// Emits a human-readable event to the serial log.
    void ReportEvent(bool includeAltitude, float timeSeconds, const char *label);
    /// Propagates attitude with gyro-only integration during ascent/coast.
    math_utils::Quaternion TeasleyFilter(const math_utils::Quaternion &quat, const float gyro[3], float dt);
    /// Converts a raw quaternion array into the internal math type.
    math_utils::Quaternion ArrayToQuaternion(const float values[4]) const;

    KalmanFilterAccel kalmanX_;
    KalmanFilterAccel kalmanY_;
    KalmanFilterAccelAlt kalmanZ_;

    EnvironmentModel environment_;
    ApogeePredictor apogeePredictor_;
    double apogeeTargetMeters_ = 1550.0;

    FlightStatus status_ = FlightStatus::Ground;
    bool initialized_ = false;
    double lastTimestamp_ = 0.0;
    double zenithRadians_ = 0.0;
    double lastZenith_ = 0.0;
    bool quaternionValid_ = false;
    math_utils::Quaternion previousQuaternion_ = math_utils::MakeQuaternion(1.0f, 0.0f, 0.0f, 0.0f);

    double lastApogeePrediction_ = 0.0;
    double apogeeAltitude_ = 0.0;
    bool apogeeRecorded_ = false;
    double burnTimestamp_ = 0.0;
    double burnoutTimestamp_ = 0.0;
    double apogeeTimestamp_ = 0.0;
    uint8_t liftoffCandidateCount_ = 0;
    uint8_t burnoutCandidateCount_ = 0;

    double processNoiseXY_ = 0.5;
    double processNoiseZ_ = 1.0;

    // Output-only smoothing state (does not affect estimator internals/status decisions).
    bool outputFilterInitialized_ = false;
    double smoothedVelocity_[3] = {0.0, 0.0, 0.0};
    double smoothedAcceleration_[3] = {0.0, 0.0, 0.0};
    PredictorHorizontalVelocityTracker predictorHorizontalVelocity_;

};

const char *FlightStatusToString(FlightStatus status);
