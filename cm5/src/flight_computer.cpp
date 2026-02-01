/// \file flight_computer.cpp
/// \brief FlightComputer implementation and state transitions.

#include "flight_computer.h"

#include <math.h>

#include "constants.h"
#include "math_utils.h"

namespace {

/// \brief Default integration time step in seconds.
constexpr float kDefaultDt = 0.03f;

/// \brief Acceleration threshold for liftoff detection.
constexpr float kLiftoffAccelerationThreshold = 5.0f;   // m/s^2
/// \brief Altitude threshold for liftoff detection.
constexpr float kLiftoffAltitudeThreshold = 1.0f;       // m above pad

/// \brief Acceleration threshold for burnout detection.
constexpr float kBurnoutAccelerationThreshold = 0.0f;   // m/s^2
/// \brief Velocity threshold for burnout detection.
constexpr float kBurnoutVelocityThreshold = 0.0f;       // still ascending

/// \brief Velocity threshold for descent detection.
constexpr float kDescentVelocityThreshold = 0.0f;       // m/s downward or zero
/// \brief Acceleration threshold for descent detection.
constexpr float kDescentAccelerationThreshold = 0.0f;   // ensure net downward accel

/// \brief Rotate body-frame acceleration into the inertial frame.
/// \param bodyAccel Body-frame acceleration.
/// \param zenith Zenith angle in radians.
/// \return Acceleration in the inertial frame.
math_utils::Vec3 RotateBodyToInertial(const math_utils::Vec3 &bodyAccel, float zenith) {
    const float angle = zenith - 1.5707963267948966f;
    float sinA;
    float cosA;
    math_utils::FastSinCos(angle, sinA, cosA);

    math_utils::Vec3 result;
    result.x = bodyAccel.x * cosA + bodyAccel.z * sinA;
    result.y = bodyAccel.y;
    result.z = -bodyAccel.x * sinA + bodyAccel.z * cosA - constants::kGravity;
    return result;
}

/// \brief Convert quaternion to Euler angles using the Python convert.py convention.
/// \param q Input quaternion in wxyz order.
/// \param yaw Output yaw in radians.
/// \param pitch Output pitch in radians.
/// \param roll Output roll in radians.
void QuaternionToEulerPython(const math_utils::Quaternion &q, float &yaw, float &pitch, float &roll) {
    const float w = q.w;
    const float x = q.x;
    const float y = q.y;
    const float z = q.z;

    const float r11 = 2.0f * w * w - 1.0f + 2.0f * x * x;
    const float r21 = 2.0f * (x * y - w * z);
    const float r31 = 2.0f * (x * z + w * y);
    const float r32 = 2.0f * (y * z - w * x);
    const float r33 = 2.0f * w * w - 1.0f + 2.0f * z * z;

    roll = atan2f(r32, r33);
    const float denom = sqrtf(fmaxf(0.0f, 1.0f - r31 * r31));
    if (denom == 0.0f) {
        pitch = -copysignf(1.5707963267948966f, r31);
    } else {
        pitch = -atanf(r31 / denom);
    }
    yaw = atan2f(r21, r11);
}

}  // namespace

/// \brief Construct the flight computer.
FlightComputer::FlightComputer() = default;

/// \brief Load a CFD lookup table into the predictor.
/// \param path Path to CFD CSV file.
/// \return True if the table loaded successfully.
bool FlightComputer::LoadCfdTable(const char *path) {
    return apogeePredictor_.LoadCfdTable(path);
}

/// \brief Configure filters, environment, and prediction parameters.
void FlightComputer::Begin(float sigmaAccelXY,
                           float sigmaAccelZ,
                           float sigmaAltimeter,
                           float processXY,
                           float processZ,
                           float apogeeTargetMeters,
                           const EnvironmentModel::Config &environmentConfig,
                           const ApogeeVehicleParameters &vehicleParameters) {
    environment_.Configure(environmentConfig);
    apogeePredictor_.SetEnvironment(environment_);
    apogeePredictor_.SetVehicleParameters(vehicleParameters);

    kalmanX_.Configure(sigmaAccelXY);
    kalmanY_.Configure(sigmaAccelXY);
    kalmanZ_.Configure(sigmaAccelZ, sigmaAltimeter);

    processNoiseXY_ = processXY;
    processNoiseZ_ = processZ;
    apogeeTargetMeters_ = apogeeTargetMeters;

    ResetInternalState();
}

/// \brief Update the filter state with new sensor data.
bool FlightComputer::Update(const SensorData &data, FilteredState &output) {
    float dt = kDefaultDt;
    if (!initialized_) {
        initialized_ = true;
    } else {
        dt = data.timestamp - lastTimestamp_;
    }
    lastTimestamp_ = data.timestamp;
    const float altitudeMeters = data.altitudeFeet * constants::kFeetToMeters;

    float accelBody[3];
    if (status_ == FlightStatus::Ground || status_ == FlightStatus::Burn) {
        accelBody[0] = data.accelICM[0];
        accelBody[1] = data.accelICM[1];
        accelBody[2] = data.accelICM[2];
    } else {
        accelBody[0] = data.accelBNO[0];
        accelBody[1] = data.accelBNO[1];
        accelBody[2] = data.accelBNO[2];
        const float tmpX = accelBody[0];
        const float tmpY = accelBody[1];
        accelBody[0] = -tmpY;
        accelBody[1] = tmpX;
    }

    const float bodyX = accelBody[0];
    const float bodyY = accelBody[1];
    const float bodyZ = accelBody[2];
    accelBody[0] = bodyZ;
    accelBody[1] = bodyY;
    accelBody[2] = bodyX;

    if (data.hasQuaternion) {
        previousQuaternion_ = ArrayToQuaternion(data.quaternion);
        quaternionValid_ = true;
    }

    math_utils::Quaternion orientation = previousQuaternion_;
    if ((status_ == FlightStatus::Burn || status_ == FlightStatus::Coast) && quaternionValid_) {
        orientation = TeasleyFilter(previousQuaternion_, data.gyro, dt);
        previousQuaternion_ = orientation;
    }

    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    QuaternionToEulerPython(orientation, yaw, pitch, roll);
    zenithRadians_ = math_utils::EulerToZenith(pitch, roll);

    const math_utils::Vec3 bodyAccel = math_utils::MakeVec3(accelBody[0], accelBody[1], accelBody[2]);
    const math_utils::Vec3 inertialAcceleration = RotateBodyToInertial(bodyAccel, zenithRadians_);

    kalmanX_.Predict(dt, processNoiseXY_);
    kalmanY_.Predict(dt, processNoiseXY_);
    kalmanZ_.Predict(dt, processNoiseZ_);

    kalmanX_.Update(inertialAcceleration.x);
    kalmanY_.Update(inertialAcceleration.y);
    kalmanZ_.Update(inertialAcceleration.z, altitudeMeters);

    const float posX = kalmanX_.Position();
    const float posY = kalmanY_.Position();
    const float posZ = kalmanZ_.Position();
    const float velX = kalmanX_.Velocity();
    const float velY = kalmanY_.Velocity();
    const float velZ = kalmanZ_.Velocity();
    const float accX = kalmanX_.Acceleration();
    const float accY = kalmanY_.Acceleration();
    const float accZ = kalmanZ_.Acceleration();

    if (status_ == FlightStatus::Coast) {
        ApogeeState predictorState;
        predictorState.altitudeMeters = posZ;
        predictorState.horizontalDistanceMeters = math_utils::Magnitude2(posX, posY);
        predictorState.verticalVelocity = velZ;
        predictorState.horizontalVelocity = math_utils::Magnitude2(velX, velY);
        predictorState.zenith = zenithRadians_;
        predictorState.angularVelocity = (zenithRadians_ - lastZenith_) / dt;
        lastApogeePrediction_ = apogeePredictor_.PredictApogee(predictorState);
    }

    if (status_ == FlightStatus::Ground) {
        if (accZ > kLiftoffAccelerationThreshold && fabsf(posZ) > kLiftoffAltitudeThreshold) {
            status_ = FlightStatus::Burn;
            burnTimestamp_ = data.timestamp;
            ReportEvent(false, data.timestamp, "Engine burn");
        }
    }

    if (status_ == FlightStatus::Burn) {
        if (accZ < kBurnoutAccelerationThreshold && posZ < apogeeTargetMeters_ && velZ > kBurnoutVelocityThreshold) {
            status_ = FlightStatus::Coast;
            burnoutTimestamp_ = data.timestamp;
            ReportEvent(false, data.timestamp, "Engine burnout");
        }
    }

    if (status_ == FlightStatus::Coast) {
        if (accZ < kBurnoutAccelerationThreshold && posZ >= apogeeTargetMeters_) {
            status_ = FlightStatus::Overshoot;
            ReportEvent(false, data.timestamp, "Overshoot");
        }
    }

    if (status_ == FlightStatus::Overshoot || status_ == FlightStatus::Coast) {
        if (accZ < kDescentAccelerationThreshold && velZ <= kDescentVelocityThreshold) {
            status_ = FlightStatus::Descent;
            apogeeAltitude_ = posZ;
            apogeeTimestamp_ = data.timestamp;
            apogeeRecorded_ = true;
            ReportEvent(true, data.timestamp, "Apogee reached");
        }
    }

    output.time = data.timestamp;
    output.position[0] = posX;
    output.position[1] = posY;
    output.position[2] = posZ;
    output.velocity[0] = velX;
    output.velocity[1] = velY;
    output.velocity[2] = velZ;
    output.acceleration[0] = accX;
    output.acceleration[1] = accY;
    output.acceleration[2] = accZ;
    output.inertialAcceleration[0] = inertialAcceleration.x;
    output.inertialAcceleration[1] = inertialAcceleration.y;
    output.inertialAcceleration[2] = inertialAcceleration.z;
    output.zenith = zenithRadians_;
    output.apogeeEstimate = lastApogeePrediction_;

    lastZenith_ = zenithRadians_;

    return true;
}

/// \brief Reset state and filter history for a new flight.
void FlightComputer::ResetInternalState() {
    kalmanX_.Reset();
    kalmanY_.Reset();
    kalmanZ_.Reset();
    status_ = FlightStatus::Ground;
    initialized_ = false;
    zenithRadians_ = 0.0f;
    lastZenith_ = 0.0f;
    quaternionValid_ = false;
    previousQuaternion_ = math_utils::MakeQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
    lastApogeePrediction_ = 0.0f;
    apogeeAltitude_ = 0.0f;
    apogeeRecorded_ = false;
    burnTimestamp_ = 0.0f;
    burnoutTimestamp_ = 0.0f;
    apogeeTimestamp_ = 0.0f;
}

/// \brief Report an event via serial (ARDUINO builds only).
void FlightComputer::ReportEvent(bool includeAltitude, float timeSeconds, const char *label) {
#if defined(ARDUINO)
    if (!serialReportingEnabled_ || !Serial) {
        return;
    }
    Serial.print(label);
    Serial.print(" at t = ");
    Serial.print(timeSeconds, 4);
    Serial.print(" s");
    if (includeAltitude) {
        Serial.print(", altitude = ");
        Serial.print(apogeeAltitude_, 2);
        Serial.print(" m");
    }
    Serial.println();
#else
    (void)includeAltitude;
    (void)timeSeconds;
    (void)label;
#endif
}

/// \brief Update orientation using the Teasley filter.
math_utils::Quaternion FlightComputer::TeasleyFilter(const math_utils::Quaternion &quat, const float gyro[3], float dt) {
    const float half_dt = 0.5f * dt;
    const float qw = quat.w;
    const float qx = quat.x;
    const float qy = quat.y;
    const float qz = quat.z;
    const float gx = gyro[0];
    const float gy = gyro[1];
    const float gz = gyro[2];

    const float dq_w = (-qx * gx - qy * gy - qz * gz) * half_dt;
    const float dq_x = (qw * gx + qy * gz - qz * gy) * half_dt;
    const float dq_y = (qw * gy - qx * gz + qz * gx) * half_dt;
    const float dq_z = (qw * gz + qx * gy - qy * gx) * half_dt;

    math_utils::Quaternion updated = math_utils::MakeQuaternion(qw + dq_w, qx + dq_x, qy + dq_y, qz + dq_z);
    return math_utils::Normalize(updated);
}

/// \brief Normalize a quaternion from a float array.
math_utils::Quaternion FlightComputer::ArrayToQuaternion(const float values[4]) const {
    return math_utils::Normalize(math_utils::MakeQuaternion(values[0], values[1], values[2], values[3]));
}

/// \brief Convert a FlightStatus to a string label.
const char *FlightStatusToString(FlightStatus status) {
    switch (status) {
        case FlightStatus::Ground:
            return "ground";
        case FlightStatus::Burn:
            return "burn";
        case FlightStatus::Coast:
            return "coast";
        case FlightStatus::Overshoot:
            return "overshoot";
        case FlightStatus::Descent:
            return "descent";
        default:
            return "unknown";
    }
}
