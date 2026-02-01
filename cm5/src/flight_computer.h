#pragma once

#include <stdint.h>

#include "apogee_model.h"
#include "environment_model.h"
#include "kalman_filters.h"
#include "math_utils.h"
#include "telemetry_types.h"

/// \file flight_computer.h
/// \brief Flight computer API and state types.

/// \brief Data structures mirroring the layout of the Python flight.py script.

/// \brief Filtered state output from the flight computer.
struct FilteredState {
    /// \brief Time in seconds.
    float time = 0.0f;
    /// \brief Position vector in meters.
    float position[3] = {0.0f, 0.0f, 0.0f};
    /// \brief Velocity vector in meters per second.
    float velocity[3] = {0.0f, 0.0f, 0.0f};
    /// \brief Acceleration vector in meters per second squared.
    float acceleration[3] = {0.0f, 0.0f, 0.0f};
    /// \brief Acceleration in the inertial frame.
    float inertialAcceleration[3] = {0.0f, 0.0f, 0.0f};
    /// \brief Zenith angle in radians.
    float zenith = 0.0f;
    /// \brief Predicted apogee altitude in meters.
    float apogeeEstimate = 0.0f;
};

/// \brief Discrete flight status labels.
enum class FlightStatus { Ground, Burn, Coast, Overshoot, Descent };

/// \brief Implements state estimation and apogee prediction logic.
class FlightComputer {
  public:
    /// \brief Construct a flight computer instance.
    FlightComputer();

    /// \brief Initialize filters, environment, and target parameters.
    /// \param sigmaAccelXY Std dev of acceleration measurement in X/Y.
    /// \param sigmaAccelZ Std dev of acceleration measurement in Z.
    /// \param sigmaAltimeter Std dev of altimeter measurement.
    /// \param processXY Std dev of process noise in X/Y.
    /// \param processZ Std dev of process noise in Z.
    /// \param apogeeTargetMeters Target apogee altitude in meters.
    /// \param environmentConfig Environment model configuration.
    /// \param vehicleParameters Vehicle parameters for apogee prediction.
    void Begin(float sigmaAccelXY,
               float sigmaAccelZ,
               float sigmaAltimeter,
               float processXY,
               float processZ,
               float apogeeTargetMeters,
               const EnvironmentModel::Config &environmentConfig,
               const ApogeeVehicleParameters &vehicleParameters);

    /// \brief Update the estimator with new sensor data.
    /// \param data Latest sensor sample.
    /// \param output Filtered state output.
    /// \return True if a state estimate was produced.
    bool Update(const SensorData &data, FilteredState &output);
    /// \brief Load a CFD lookup table from a CSV file.
    /// \param path Path to the CSV file.
    /// \return True if the table was loaded successfully.
    bool LoadCfdTable(const char *path);

    /// \brief Enable or disable serial event reporting (ARDUINO builds).
    /// \param enabled True to enable reporting.
    void SetSerialReportingEnabled(bool enabled) { serialReportingEnabled_ = enabled; }

    /// \brief Current flight status.
    FlightStatus Status() const { return status_; }
    /// \brief Last predicted apogee altitude.
    float ApogeePrediction() const { return lastApogeePrediction_; }
    /// \brief True once apogee has been recorded.
    bool ApogeeReached() const { return apogeeRecorded_; }
    /// \brief Recorded apogee altitude.
    float ApogeeAltitude() const { return apogeeAltitude_; }
    /// \brief Timestamp of engine burn start.
    float BurnTime() const { return burnTimestamp_; }
    /// \brief Timestamp of engine burnout.
    float BurnoutTime() const { return burnoutTimestamp_; }
    /// \brief Timestamp of apogee.
    float ApogeeTime() const { return apogeeTimestamp_; }

  private:
    /// \brief Reset internal state for a new flight.
    void ResetInternalState();
    /// \brief Report a state transition event over serial when enabled.
    /// \param includeAltitude Whether to include altitude in the output.
    /// \param timeSeconds Event time in seconds.
    /// \param label Event label string.
    void ReportEvent(bool includeAltitude, float timeSeconds, const char *label);
    /// \brief Integrate gyroscope readings using the Teasley filter.
    /// \param quat Previous orientation.
    /// \param gyro Gyroscope readings in radians per second.
    /// \param dt Time step in seconds.
    /// \return Updated orientation.
    math_utils::Quaternion TeasleyFilter(const math_utils::Quaternion &quat, const float gyro[3], float dt);
    /// \brief Convert a raw quaternion array to a normalized quaternion.
    /// \param values Quaternion array in wxyz order.
    /// \return Normalized quaternion.
    math_utils::Quaternion ArrayToQuaternion(const float values[4]) const;

    /// \brief Kalman filter for X-axis motion.
    KalmanFilterAccel kalmanX_;
    /// \brief Kalman filter for Y-axis motion.
    KalmanFilterAccel kalmanY_;
    /// \brief Kalman filter for Z-axis motion and altitude.
    KalmanFilterAccelAlt kalmanZ_;

    /// \brief Environment model for winds and temperature.
    EnvironmentModel environment_;
    /// \brief Apogee predictor for ballistic estimation.
    ApogeePredictor apogeePredictor_;
    /// \brief Target apogee altitude in meters.
    float apogeeTargetMeters_ = 1550.0f;

    /// \brief Current flight status.
    FlightStatus status_ = FlightStatus::Ground;
    /// \brief True after the first valid update.
    bool initialized_ = false;
    /// \brief Timestamp of the previous update.
    float lastTimestamp_ = 0.0f;
    /// \brief Current zenith angle in radians.
    float zenithRadians_ = 0.0f;
    /// \brief Previous zenith angle in radians.
    float lastZenith_ = 0.0f;
    /// \brief True once a valid quaternion has been received.
    bool quaternionValid_ = false;
    /// \brief Last known orientation quaternion.
    math_utils::Quaternion previousQuaternion_ = math_utils::MakeQuaternion(1.0f, 0.0f, 0.0f, 0.0f);

    /// \brief Last predicted apogee altitude.
    float lastApogeePrediction_ = 0.0f;
    /// \brief Recorded apogee altitude.
    float apogeeAltitude_ = 0.0f;
    /// \brief True once apogee is recorded.
    bool apogeeRecorded_ = false;
    /// \brief Timestamp of engine burn.
    float burnTimestamp_ = 0.0f;
    /// \brief Timestamp of engine burnout.
    float burnoutTimestamp_ = 0.0f;
    /// \brief Timestamp of apogee.
    float apogeeTimestamp_ = 0.0f;

    /// \brief Process noise in X/Y.
    float processNoiseXY_ = 0.5f;
    /// \brief Process noise in Z.
    float processNoiseZ_ = 1.0f;

    /// \brief Toggle for serial reporting (ARDUINO builds).
    bool serialReportingEnabled_ = true;

};

/// \brief Convert a flight status enum to a string.
/// \param status Flight status value.
/// \return String description of the status.
const char *FlightStatusToString(FlightStatus status);
