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
    float icmQuaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float icmYprDeg[3] = {0.0f, 0.0f, 0.0f};
    bool hasQuaternion = false;
    bool hasIcmQuaternion = false;
    bool hasIcmYpr = false;
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

    void Begin(double sigmaAccelXY,
               double sigmaAccelZ,
               double sigmaAltimeter,
               double processXY,
               double processZ,
               double apogeeTargetMeters,
               const EnvironmentModel::Config &environmentConfig,
               const ApogeeVehicleParameters &vehicleParameters,
               const ApogeeForceTable *forceTable = nullptr);

    bool Update(const SensorData &data, FilteredState &output);

    void SetSerialReportingEnabled(bool enabled) { (void)enabled; }

    FlightStatus Status() const { return status_; }
    double ApogeePrediction() const { return lastApogeePrediction_; }
    bool ApogeeReached() const { return apogeeRecorded_; }
    double ApogeeAltitude() const { return apogeeAltitude_; }
    double BurnTime() const { return burnTimestamp_; }
    double BurnoutTime() const { return burnoutTimestamp_; }
    double ApogeeTime() const { return apogeeTimestamp_; }

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

};

const char *FlightStatusToString(FlightStatus status);
