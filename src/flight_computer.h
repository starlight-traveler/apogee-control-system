#pragma once

#include <Arduino.h>

#include "apogee_model.h"
#include "environment_model.h"
#include "kalman_filters.h"
#include "math_utils.h"

// Data structures mirroring the layout of the Python flight.py script.

struct SensorData {
    float timestamp = 0.0f;
    float altitudeFeet = 0.0f;
    float accelBNO[3] = {0.0f, 0.0f, 0.0f};
    float accelICM[3] = {0.0f, 0.0f, 0.0f};
    float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float gyro[3] = {0.0f, 0.0f, 0.0f};
    bool hasQuaternion = false;
};

struct FilteredState {
    float time = 0.0f;
    float position[3] = {0.0f, 0.0f, 0.0f};
    float velocity[3] = {0.0f, 0.0f, 0.0f};
    float acceleration[3] = {0.0f, 0.0f, 0.0f};
    float inertialAcceleration[3] = {0.0f, 0.0f, 0.0f};
    float zenith = 0.0f;
    float apogeeEstimate = 0.0f;
};

enum class FlightStatus { Ground, Burn, Coast, Overshoot, Descent };

class FlightComputer {
  public:
    FlightComputer();

    void Begin(float sigmaAccelXY,
               float sigmaAccelZ,
               float sigmaAltimeter,
               float processXY,
               float processZ,
               float apogeeTargetMeters,
               const EnvironmentModel::Config &environmentConfig,
               const ApogeeVehicleParameters &vehicleParameters);

    bool Update(const SensorData &data, FilteredState &output);

    void SetSerialReportingEnabled(bool enabled) { serialReportingEnabled_ = enabled; }

    FlightStatus Status() const { return status_; }
    float ApogeePrediction() const { return lastApogeePrediction_; }
    bool ApogeeReached() const { return apogeeRecorded_; }
    float ApogeeAltitude() const { return apogeeAltitude_; }
    float BurnTime() const { return burnTimestamp_; }
    float BurnoutTime() const { return burnoutTimestamp_; }
    float ApogeeTime() const { return apogeeTimestamp_; }

  private:
    void ResetInternalState();
    void ReportEvent(bool includeAltitude, float timeSeconds, const char *label);
    math_utils::Quaternion TeasleyFilter(const math_utils::Quaternion &quat, const float gyro[3], float dt);
    math_utils::Quaternion ArrayToQuaternion(const float values[4]) const;

    KalmanFilterAccel kalmanX_;
    KalmanFilterAccel kalmanY_;
    KalmanFilterAccelAlt kalmanZ_;

    EnvironmentModel environment_;
    ApogeePredictor apogeePredictor_;
    float apogeeTargetMeters_ = 1550.0f;

    FlightStatus status_ = FlightStatus::Ground;
    bool initialized_ = false;
    float lastTimestamp_ = 0.0f;
    float zenithRadians_ = 0.0f;
    float lastZenith_ = 0.0f;
    bool quaternionValid_ = false;
    math_utils::Quaternion previousQuaternion_ = math_utils::MakeQuaternion(1.0f, 0.0f, 0.0f, 0.0f);

    float lastApogeePrediction_ = 0.0f;
    float apogeeAltitude_ = 0.0f;
    bool apogeeRecorded_ = false;
    float burnTimestamp_ = 0.0f;
    float burnoutTimestamp_ = 0.0f;
    float apogeeTimestamp_ = 0.0f;

    float processNoiseXY_ = 0.5f;
    float processNoiseZ_ = 1.0f;
    float apogeeUpdateAccumulator_ = 0.0f;
    float apogeeUpdateInterval_ = 0.05f;  // Predict apogee at ~20 Hz to save time.

    float groundReferenceAltitude_ = 0.0f;
    bool groundReferenceValid_ = false;
    float lastRelativeAltitude_ = 0.0f;
    bool relativeAltitudeInitialized_ = false;
    float relativeClimbRateFiltered_ = 0.0f;

    bool serialReportingEnabled_ = true;

    float liftoffTimer_ = 0.0f;
    float burnoutTimer_ = 0.0f;
    float descentTimer_ = 0.0f;
};

const char *FlightStatusToString(FlightStatus status);
