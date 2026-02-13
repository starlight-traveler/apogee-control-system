#include "flight_computer.h"

#include <cmath>

#include "constants.h"
#include "math_utils.h"

namespace {

constexpr float kDefaultDt = 0.03f;

constexpr float kLiftoffAccelerationThreshold = 20.0;   // m/s^2
constexpr float kLiftoffAltitudeThreshold = 40.0f;       // m above pad

constexpr float kBurnoutAccelerationThreshold = 0.0f;   // m/s^2
constexpr float kBurnoutVelocityThreshold = 0.0f;       // still ascending

constexpr float kDescentVelocityThreshold = 0.0f;       // m/s downward or zero
constexpr float kDescentAccelerationThreshold = 0.0f;   // ensure net downward accel

math_utils::Vec3d RotateBodyToInertial(const math_utils::Vec3d &bodyAccel, double zenith) {
    const double angle = zenith - 1.5707963267948966;
    const double sinA = std::sin(angle);
    const double cosA = std::cos(angle);

    math_utils::Vec3d result;
    result.x = bodyAccel.x * cosA + bodyAccel.z * sinA;
    result.y = bodyAccel.y;
    result.z = -bodyAccel.x * sinA + bodyAccel.z * cosA - static_cast<double>(constants::kGravity);
    return result;
}

}  // namespace

FlightComputer::FlightComputer() = default;

void FlightComputer::Begin(double sigmaAccelXY,
                           double sigmaAccelZ,
                           double sigmaAltimeter,
                           double processXY,
                           double processZ,
                           double apogeeTargetMeters,
                           const EnvironmentModel::Config &environmentConfig,
                           const ApogeeVehicleParameters &vehicleParameters,
                           const ApogeeForceTable *forceTable) {
    environment_.Configure(environmentConfig);
    apogeePredictor_.SetEnvironment(environment_);
    apogeePredictor_.SetVehicleParameters(vehicleParameters);
    apogeePredictor_.SetForceTable(forceTable);

    kalmanX_.Configure(sigmaAccelXY);
    kalmanY_.Configure(sigmaAccelXY);
    kalmanZ_.Configure(sigmaAccelZ, sigmaAltimeter);

    processNoiseXY_ = processXY;
    processNoiseZ_ = processZ;
    apogeeTargetMeters_ = apogeeTargetMeters;

    ResetInternalState();
}

bool FlightComputer::Update(const SensorData &data, FilteredState &output) {

    if ((data.accelBNO[0] == 0.0 && data.accelBNO[1] == 0.0 && data.accelBNO[2] == 0.0) ||
        (data.accelICM[0] == 0.0 && data.accelICM[1] == 0.0 && data.accelICM[2] == 0.0)) {
        return false; // No valid accelerometer data; skip this update
    }
    
    double dt = static_cast<double>(kDefaultDt);
    if (!initialized_) {
        initialized_ = true;
    } else {
        dt = static_cast<double>(data.timestamp) - lastTimestamp_;
    }
    if (dt <= 0.0 || dt > 1.0) {
        dt = static_cast<double>(kDefaultDt);
    }
    lastTimestamp_ = static_cast<double>(data.timestamp);
    const double altitudeMeters = static_cast<double>(data.altitudeFeet) * constants::kFeetToMeters;

    float accelBody[3];
    if (status_ == FlightStatus::Ground || status_ == FlightStatus::Burn) {
        accelBody[0] = data.accelICM[0];
        accelBody[1] = data.accelICM[1];
        accelBody[2] = data.accelICM[2];
    } else {
        accelBody[0] = data.accelBNO[0];
        accelBody[1] = data.accelBNO[1];
        accelBody[2] = data.accelBNO[2];
    }

    math_utils::Quaternion orientation = previousQuaternion_;
    if (!quaternionValid_ && data.hasQuaternion) {
        previousQuaternion_ = ArrayToQuaternion(data.quaternion);
        quaternionValid_ = true;
        orientation = previousQuaternion_;
    } else if ((status_ == FlightStatus::Burn || status_ == FlightStatus::Coast) && quaternionValid_) {
        orientation = TeasleyFilter(previousQuaternion_, data.gyro, static_cast<float>(dt));
        previousQuaternion_ = orientation;
    } else if (data.hasQuaternion) {
        orientation = ArrayToQuaternion(data.quaternion);
        previousQuaternion_ = orientation;
        quaternionValid_ = true;
    }

    const math_utils::Quaterniond orientationD = math_utils::MakeQuaternion(
        static_cast<double>(orientation.w),
        static_cast<double>(orientation.x),
        static_cast<double>(orientation.y),
        static_cast<double>(orientation.z));
    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
    math_utils::QuaternionToEuler(orientationD, yaw, pitch, roll);
    zenithRadians_ = math_utils::EulerToZenith(pitch, roll);

    const math_utils::Vec3d bodyAccel = math_utils::MakeVec3d(
        static_cast<double>(accelBody[0]),
        static_cast<double>(accelBody[1]),
        static_cast<double>(accelBody[2]));
    const math_utils::Vec3d inertialAcceleration = RotateBodyToInertial(bodyAccel, zenithRadians_);

    kalmanX_.Predict(dt, processNoiseXY_);
    kalmanY_.Predict(dt, processNoiseXY_);
    kalmanZ_.Predict(dt, processNoiseZ_);

    kalmanX_.Update(inertialAcceleration.x);
    kalmanY_.Update(inertialAcceleration.y);
    kalmanZ_.Update(inertialAcceleration.z, altitudeMeters);

    const double posX = kalmanX_.Position();
    const double posY = kalmanY_.Position();
    const double posZ = kalmanZ_.Position();
    const double velX = kalmanX_.Velocity();
    const double velY = kalmanY_.Velocity();
    const double velZ = kalmanZ_.Velocity();
    const double accX = kalmanX_.Acceleration();
    const double accY = kalmanY_.Acceleration();
    const double accZ = kalmanZ_.Acceleration();

    if (status_ == FlightStatus::Coast) {
        ApogeeState predictorState;
        predictorState.altitudeMeters = posZ;
        predictorState.horizontalDistanceMeters = math_utils::Magnitude2(posX, posY);
        predictorState.verticalVelocity = velZ;
        predictorState.horizontalVelocity = math_utils::Magnitude2(velX, velY);
        predictorState.zenith = zenithRadians_;
        predictorState.angularVelocity = (dt != 0.0) ? (zenithRadians_ - lastZenith_) / dt : 0.0;
        lastApogeePrediction_ = apogeePredictor_.PredictApogee(predictorState);
    }

    if (status_ == FlightStatus::Ground) {
        if (accZ > kLiftoffAccelerationThreshold && std::fabs(posZ) > kLiftoffAltitudeThreshold) {
            status_ = FlightStatus::Burn;
            burnTimestamp_ = static_cast<double>(data.timestamp);
            ReportEvent(false, data.timestamp, "Engine burn");
        }
    }

    if (status_ == FlightStatus::Burn) {
        if (accZ < kBurnoutAccelerationThreshold && posZ < apogeeTargetMeters_ && velZ > kBurnoutVelocityThreshold) {
            status_ = FlightStatus::Coast;
            burnoutTimestamp_ = static_cast<double>(data.timestamp);
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
            apogeeTimestamp_ = static_cast<double>(data.timestamp);
            apogeeRecorded_ = true;
            ReportEvent(true, data.timestamp, "Apogee reached");
        }
    }

    output.time = data.timestamp;
    output.position[0] = static_cast<float>(posX);
    output.position[1] = static_cast<float>(posY);
    output.position[2] = static_cast<float>(posZ);
    output.velocity[0] = static_cast<float>(velX);
    output.velocity[1] = static_cast<float>(velY);
    output.velocity[2] = static_cast<float>(velZ);
    output.acceleration[0] = static_cast<float>(accX);
    output.acceleration[1] = static_cast<float>(accY);
    output.acceleration[2] = static_cast<float>(accZ);
    output.inertialAcceleration[0] = static_cast<float>(inertialAcceleration.x);
    output.inertialAcceleration[1] = static_cast<float>(inertialAcceleration.y);
    output.inertialAcceleration[2] = static_cast<float>(inertialAcceleration.z);
    output.zenith = static_cast<float>(zenithRadians_);
    output.apogeeEstimate = static_cast<float>(lastApogeePrediction_);

    lastZenith_ = zenithRadians_;

    return true;
}

void FlightComputer::ResetInternalState() {
    kalmanX_.Reset();
    kalmanY_.Reset();
    kalmanZ_.Reset();
    status_ = FlightStatus::Ground;
    initialized_ = false;
    zenithRadians_ = 0.0;
    lastZenith_ = 0.0;
    quaternionValid_ = false;
    previousQuaternion_ = math_utils::MakeQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
    lastApogeePrediction_ = 0.0;
    apogeeAltitude_ = 0.0;
    apogeeRecorded_ = false;
    burnTimestamp_ = 0.0;
    burnoutTimestamp_ = 0.0;
    apogeeTimestamp_ = 0.0;
}

void FlightComputer::ReportEvent(bool includeAltitude, float timeSeconds, const char *label) {
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
}

math_utils::Quaternion FlightComputer::TeasleyFilter(const math_utils::Quaternion &quat, const float gyro[3], float dt) {
    const double half_dt = 0.5 * static_cast<double>(dt);
    const double qw = quat.w;
    const double qx = quat.x;
    const double qy = quat.y;
    const double qz = quat.z;
    const double gx = gyro[0];
    const double gy = gyro[1];
    const double gz = gyro[2];

    const double dq_w = (-qx * gx - qy * gy - qz * gz) * half_dt;
    const double dq_x = (qw * gx + qy * gz - qz * gy) * half_dt;
    const double dq_y = (qw * gy - qx * gz + qz * gx) * half_dt;
    const double dq_z = (qw * gz + qx * gy - qy * gx) * half_dt;

    math_utils::Quaternion updated = math_utils::MakeQuaternion(
        static_cast<float>(qw + dq_w),
        static_cast<float>(qx + dq_x),
        static_cast<float>(qy + dq_y),
        static_cast<float>(qz + dq_z));
    return math_utils::Normalize(updated);
}

math_utils::Quaternion FlightComputer::ArrayToQuaternion(const float values[4]) const {
    return math_utils::Normalize(math_utils::MakeQuaternion(values[0], values[1], values[2], values[3]));
}

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
