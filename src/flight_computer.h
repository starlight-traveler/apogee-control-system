#pragma once

#include <Arduino.h>

#include "apogee_model.h"
#include "environment_model.h"
#include "kalman_filters.h"
#include "math_utils.h"
#include "predictor_seed.h"

/// Raw sensor/control sample written to logs and consumed by the estimator.

enum class MainQuaternionSource : uint8_t {
    None = 0,
    Bno = 1,
    Icm = 2,
    Lsm = 3,
    Blended = 4,
    Pulse = 5,
};

struct SensorData {
    float timestamp = 0.0f;
    float altitudeFeet = 0.0f;
    float accelBNO[3] = {0.0f, 0.0f, 0.0f};
    float gyroBNO[3] = {0.0f, 0.0f, 0.0f};
    float quaternionBNO[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float accelICM[3] = {0.0f, 0.0f, 0.0f};
    float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float gyro[3] = {0.0f, 0.0f, 0.0f};
    float icmQuaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float icmYprDeg[3] = {0.0f, 0.0f, 0.0f};
    float accelLSM[3] = {0.0f, 0.0f, 0.0f};
    float gyroLSM[3] = {0.0f, 0.0f, 0.0f};
    float quaternionLSM[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float lsmYprDeg[3] = {0.0f, 0.0f, 0.0f};
    float accelPulse[3] = {0.0f, 0.0f, 0.0f};
    float gyroPulse[3] = {0.0f, 0.0f, 0.0f};
    float quaternionPulse[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float pulseYprDeg[3] = {0.0f, 0.0f, 0.0f};
    float icmTemperatureC = 0.0f;
    float icmAhrsDt = 0.0f;
    float icmAccelTrust = 0.0f;
    float icmMagTrust = 0.0f;
    float icmGyroBias[3] = {0.0f, 0.0f, 0.0f};
    float altimeterSigmaScale = 1.0f;
    float altimeterGateSigma = 3.5f;
    float autoCommandDeg = 0.0f;
    float optimizerBestPredictedApogeeM = 0.0f;
    float optimizerBestCost = 0.0f;
    float optimizerTimeToApogeeS = 0.0f;
    float flapCommandDeg = 0.0f;
    float flapEffectiveDeg = 0.0f;
    float actuationIsSettling = 0.0f;
    float predictorSeedHorizontalSpeedMps = 0.0f;
    float predictorSeedClampedZenithRad = 0.0f;
    float predictorSeedClampedAngularRateRadPerSec = 0.0f;
    float predictorSeedConfidenceFlags = 0.0f;
    uint8_t mainQuaternionSource = static_cast<uint8_t>(MainQuaternionSource::None);
    bool icmSampleFresh = false;
    bool lsmSampleFresh = false;
    bool baroSampleFresh = false;
    bool hasBnoQuaternion = false;
    bool hasQuaternion = false;
    bool hasIcmQuaternion = false;
    bool hasIcmYpr = false;
    bool hasLsmQuaternion = false;
    bool hasLsmYpr = false;
    bool hasPulseQuaternion = false;
    bool hasPulseYpr = false;
    bool icmAccelSaturated = false;
    bool icmGyroSaturated = false;
    bool icmRailConstrained = false;
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
    float padReferenceDriftMps = 0.0f;
    float padReferenceSettled = 0.0f;
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

    /// Resets pad-referenced altitude/velocity latches while staying in Ground.
    void ResetGroundReference();

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
    /// Returns the current adaptive axial drag scale used by the predictor.
    double AdaptiveAxialDragScale() const { return apogeePredictor_.AxialDragScale(); }

  private:
    /// Resets filter state, phase counters, and predictor-side caches.
    void ResetInternalState();
    /// Emits a human-readable event to the serial log.
    void ReportEvent(bool includeAltitude, float timeSeconds, const char *label);
    /// Propagates attitude with gyro-only integration during ascent/coast.
    math_utils::Quaternion TeasleyFilter(const math_utils::Quaternion &quat,
                                        const float gyro[3],
                                        float dt,
                                        bool *validOut = nullptr);
    /// Converts a raw quaternion array into the internal math type.
    bool ArrayToQuaternion(const float values[4], math_utils::Quaternion &out) const;
    /// Updates the predictor's adaptive axial drag scale from measured/model accel mismatch.
    void UpdateAdaptiveDragScale(const ApogeeState &predictorState,
                                 double measuredVerticalAcceleration,
                                 double dtSeconds,
                                 double timeToApogeeSeconds,
                                 double flapCommandDeg,
                                 double flapEffectiveDeg,
                                 bool actuationIsSettling);
    KalmanFilterAccel kalmanX_;
    KalmanFilterAccel kalmanY_;
    KalmanFilterAccelAlt kalmanZ_;

    EnvironmentModel environment_;
    ApogeePredictor apogeePredictor_;
    double apogeeTargetMeters_ = 1550.0;

    FlightStatus status_ = FlightStatus::Ground;
    bool initialized_ = false;
    double lastTimestamp_ = 0.0;
    bool altitudeReferenceInitialized_ = false;
    double altitudeReferenceMeters_ = 0.0;
    double lastGroundRelativeAltitudeMeters_ = 0.0;
    double groundRelativeVelocityMps_ = 0.0;
    double zenithRadians_ = 0.0;
    double lastZenith_ = 0.0;
    bool quaternionValid_ = false;
    math_utils::Quaternion previousQuaternion_ = math_utils::MakeQuaternion(1.0f, 0.0f, 0.0f, 0.0f);

    double lastApogeePrediction_ = 0.0;
    double apogeeAltitude_ = 0.0;
    bool apogeeRecorded_ = false;
    double burnTimestamp_ = 0.0;
    double burnDetectTimestamp_ = 0.0;
    double burnoutTimestamp_ = 0.0;
    double apogeeTimestamp_ = 0.0;
    uint8_t liftoffCandidateCount_ = 0;
    uint8_t burnoutCandidateCount_ = 0;

    double processNoiseXY_ = 0.5;
    double processNoiseZ_ = 1.0;
    double accelSigmaXY_ = 0.8;
    double accelSigmaZ_ = 0.7;
    double altitudeSigma_ = 1.0;

    // Output-only smoothing state (does not affect estimator internals/status decisions).
    bool outputFilterInitialized_ = false;
    double smoothedVelocity_[3] = {0.0, 0.0, 0.0};
    double smoothedAcceleration_[3] = {0.0, 0.0, 0.0};
    PredictorHorizontalVelocityTracker predictorHorizontalVelocity_;

    // Wind estimation state (real-time horizontal acceleration residual tracking).
    double windEstimateHorizontalMps_ = 0.0;
    double coastStartTime_ = 0.0;
    bool windEstimationActive_ = false;
    double groundReferenceDriftRateMps_ = 0.0;
    double groundReferenceStableSince_ = 0.0;
    bool groundReferenceSettled_ = false;

};

const char *FlightStatusToString(FlightStatus status);
