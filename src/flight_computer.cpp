#include "flight_computer.h"

#include <cmath>

#include "constants.h"
#include "math_utils.h"
#include "serial_logging.h"
#include "settings.h"

namespace {

/// Rotates body-frame acceleration into the simplified inertial frame used by the filters.
math_utils::Vec3 RotateBodyToInertial(const math_utils::Vec3 &bodyAccel, float zenith) {
    constexpr float kHalfPi = 1.5707963267948966f;
    const float angle = zenith - kHalfPi;
    float sinA = 0.0f;
    float cosA = 1.0f;
    math_utils::FastSinCos(angle, sinA, cosA);

    math_utils::Vec3 result;
    result.x = bodyAccel.x * cosA + bodyAccel.z * sinA;
    result.y = bodyAccel.y;
    result.z = -bodyAccel.x * sinA + bodyAccel.z * cosA - constants::kGravity;
    return result;
}

/// Returns the discrete-time smoothing alpha for the requested time constant.
double ComputeSmoothingAlpha(double dt, double tauSeconds) {
    if (dt <= 0.0 || tauSeconds <= 0.0) {
        return 1.0;
    }
    const double alpha = dt / (tauSeconds + dt);
    if (alpha < 0.0) {
        return 0.0;
    }
    if (alpha > 1.0) {
        return 1.0;
    }
    return alpha;
}

/// Limits per-sample output movement so published telemetry does not jump abruptly.
double ApplySlewLimit(double previous, double target, double maxDeltaPerStep) {
    if (maxDeltaPerStep <= 0.0) {
        return target;
    }
    const double delta = target - previous;
    if (delta > maxDeltaPerStep) {
        return previous + maxDeltaPerStep;
    }
    if (delta < -maxDeltaPerStep) {
        return previous - maxDeltaPerStep;
    }
    return target;
}

}  // namespace

FlightComputer::FlightComputer() = default;

/// Configures filters and predictor dependencies for a new flight.
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

/// Updates the runtime-configurable environment/vehicle parameters in-place.
void FlightComputer::ReconfigurePredictor(const EnvironmentModel::Config &environmentConfig,
                                          const ApogeeVehicleParameters &vehicleParameters,
                                          const ApogeeForceTable *forceTable) {
    environment_.Configure(environmentConfig);
    apogeePredictor_.SetEnvironment(environment_);
    apogeePredictor_.SetVehicleParameters(vehicleParameters);
    apogeePredictor_.SetForceTable(forceTable);
}

/// Processes one sensor sample and updates the filtered flight state.
bool FlightComputer::Update(const SensorData &data, FilteredState &output) {
    const bool hasBnoAccel =
        !(data.accelBNO[0] == 0.0f && data.accelBNO[1] == 0.0f && data.accelBNO[2] == 0.0f);
    const bool hasIcmAccel =
        !(data.accelICM[0] == 0.0f && data.accelICM[1] == 0.0f && data.accelICM[2] == 0.0f);
    if (!hasBnoAccel && !hasIcmAccel) {
        return false;  // No usable acceleration source means the filters cannot advance safely.
    }
    
    double dt = static_cast<double>(settings::flight::kDefaultDtSeconds);
    if (!initialized_) {
        initialized_ = true;
    } else {
        dt = static_cast<double>(data.timestamp) - lastTimestamp_;
    }
    if (dt <= 0.0 || dt > 1.0) {
        dt = static_cast<double>(settings::flight::kDefaultDtSeconds);
    }
    lastTimestamp_ = static_cast<double>(data.timestamp);
    const double altitudeMeters = static_cast<double>(data.altitudeFeet) * constants::kFeetToMeters;

    float accelBody[3];
    // Prefer the ICM path during the higher-dynamic ground/burn phases when it
    // is available; otherwise fall back to the BNO source.
    const bool preferIcm = (status_ == FlightStatus::Ground || status_ == FlightStatus::Burn) && hasIcmAccel;
    if (preferIcm || !hasBnoAccel) {
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
        // Propagate attitude through ascent with gyro-only integration so brief
        // quaternion dropouts do not immediately collapse the predictor seed.
        orientation = TeasleyFilter(previousQuaternion_, data.gyro, static_cast<float>(dt));
        previousQuaternion_ = orientation;
    } else if (data.hasQuaternion) {
        orientation = ArrayToQuaternion(data.quaternion);
        previousQuaternion_ = orientation;
        quaternionValid_ = true;
    }

    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    math_utils::QuaternionToEuler(orientation, yaw, pitch, roll);
    zenithRadians_ = static_cast<double>(math_utils::EulerToZenith(pitch, roll));

    const math_utils::Vec3 bodyAccel = math_utils::MakeVec3(
        accelBody[0],
        accelBody[1],
        accelBody[2]);
    const math_utils::Vec3 inertialAcceleration = RotateBodyToInertial(bodyAccel, static_cast<float>(zenithRadians_));

    kalmanX_.Predict(dt, processNoiseXY_);
    kalmanY_.Predict(dt, processNoiseXY_);
    kalmanZ_.Predict(dt, processNoiseZ_);

    kalmanX_.Update(inertialAcceleration.x);
    kalmanY_.Update(inertialAcceleration.y);
    kalmanZ_.Update(static_cast<float>(inertialAcceleration.z),
                    static_cast<float>(altitudeMeters),
                    static_cast<double>(data.altimeterSigmaScale),
                    static_cast<double>(data.altimeterGateSigma));

    const double posZ = kalmanZ_.Position();
    const double velZ = kalmanZ_.Velocity();
    const double accX = kalmanX_.Acceleration();
    const double accY = kalmanY_.Acceleration();
    const double accZ = kalmanZ_.Acceleration();
    UpdatePredictorHorizontalSpeed(predictorHorizontalVelocity_,
                                   accX,
                                   accY,
                                   dt,
                                   (status_ == FlightStatus::Burn || status_ == FlightStatus::Coast),
                                   velZ,
                                   zenithRadians_);

    if (status_ == FlightStatus::Ground) {
        const bool accelerationSuggestsLiftoff =
            accZ > settings::flight::kLiftoffAccelerationThresholdMps2;
        const bool altitudeSuggestsLiftoff =
            std::fabs(posZ) > settings::flight::kLiftoffAltitudeThresholdM;
        const bool velocitySuggestsLiftoff =
            velZ > settings::flight::kLiftoffVelocityThresholdMps;

        if (accelerationSuggestsLiftoff && altitudeSuggestsLiftoff && velocitySuggestsLiftoff) {
            if (liftoffCandidateCount_ < 255) {
                ++liftoffCandidateCount_;
            }
        } else {
            liftoffCandidateCount_ = 0;
        }

        if (liftoffCandidateCount_ >= settings::flight::kLiftoffConfirmSamples) {
            status_ = FlightStatus::Burn;
            burnTimestamp_ = static_cast<double>(data.timestamp);
            liftoffCandidateCount_ = 0;
            burnoutCandidateCount_ = 0;
            ReportEvent(false, data.timestamp, "Engine burn");
        }
    }

    if (status_ == FlightStatus::Burn) {
        const double timeSinceBurn = static_cast<double>(data.timestamp) - burnTimestamp_;
        const bool afterMinimumBurn = timeSinceBurn >= settings::flight::kBurnoutMinDurationSeconds;
        const bool accelerationSuggestsBurnout = accZ < settings::flight::kBurnoutAccelerationThresholdMps2;
        const bool stillAscending = velZ > settings::flight::kBurnoutVelocityThresholdMps;
        const bool belowTarget = posZ < apogeeTargetMeters_;

        if (afterMinimumBurn && accelerationSuggestsBurnout && stillAscending && belowTarget) {
            if (burnoutCandidateCount_ < 255) {
                ++burnoutCandidateCount_;
            }
        } else {
            burnoutCandidateCount_ = 0;
        }

        if (burnoutCandidateCount_ >= settings::flight::kBurnoutConfirmSamples) {
            status_ = FlightStatus::Coast;
            burnoutTimestamp_ = static_cast<double>(data.timestamp);
            burnoutCandidateCount_ = 0;
            ReportEvent(false, data.timestamp, "Engine burnout");
        }
    }

    if (status_ == FlightStatus::Coast) {
        if (accZ < settings::flight::kBurnoutAccelerationThresholdMps2 && posZ >= apogeeTargetMeters_) {
            status_ = FlightStatus::Overshoot;
            ReportEvent(false, data.timestamp, "Overshoot");
        }
    }

    if (status_ == FlightStatus::Overshoot || status_ == FlightStatus::Coast) {
        if (accZ < settings::flight::kDescentAccelerationThresholdMps2 &&
            velZ <= settings::flight::kDescentVelocityThresholdMps) {
            status_ = FlightStatus::Descent;
            apogeeAltitude_ = posZ;
            apogeeTimestamp_ = static_cast<double>(data.timestamp);
            apogeeRecorded_ = true;
            ReportEvent(true, data.timestamp, "Apogee reached");
        }
    }

    const bool shouldPredictApogee =
        (status_ == FlightStatus::Burn || status_ == FlightStatus::Coast) && velZ > 0.0;
    if (shouldPredictApogee) {
        // Deliberately degrade to a simpler predictor seed whenever attitude
        // freshness is questionable rather than integrating unstable XY terms.
        const double seedZenith = quaternionValid_ ? SanitizePredictorZenithRadians(zenithRadians_) : 0.0;
        const bool freshSeedSample = PredictorSeedHasFreshSample(dt);
        const bool canUseHorizontalSeed = quaternionValid_ && freshSeedSample;
        if (!canUseHorizontalSeed) {
            ResetPredictorHorizontalVelocityTracker(predictorHorizontalVelocity_);
        }
        const double predictorHorizontalVelocity =
            canUseHorizontalSeed
                ? UpdatePredictorHorizontalSpeed(predictorHorizontalVelocity_,
                                                 kalmanX_.Acceleration(),
                                                 kalmanY_.Acceleration(),
                                                 dt,
                                                 true,
                                                 velZ,
                                                 seedZenith)
                : 0.0;
        const double predictorAngularRate =
            canUseHorizontalSeed
                ? ClampPredictorAngularRate(ComputePredictorAngularRate(zenithRadians_, lastZenith_, dt))
                : 0.0;

        ApogeeState predictorState;
        predictorState.altitudeMeters = posZ;
        predictorState.horizontalDistanceMeters = 0.0;
        predictorState.verticalVelocity = velZ;
        predictorState.horizontalVelocity = predictorHorizontalVelocity;
        predictorState.zenith = seedZenith;
        predictorState.angularVelocity = predictorAngularRate;
        lastApogeePrediction_ = apogeePredictor_.PredictApogee(predictorState);
    } else {
        ResetPredictorHorizontalVelocityTracker(predictorHorizontalVelocity_);
    }

    // Smooth only published outputs to reduce telemetry/log oscillation.
    // Phase transitions above remain on raw Kalman values so state-machine
    // timing is not delayed by presentation-oriented filtering.
    constexpr double kVelocityTauSeconds = 0.0;
    constexpr double kAccelerationTauSeconds = 0.0;
    constexpr double kMaxOutputAccelMps2 = 1.0e9;
    const double alphaVel = ComputeSmoothingAlpha(dt, kVelocityTauSeconds);
    const double alphaAcc = ComputeSmoothingAlpha(dt, kAccelerationTauSeconds);
    if (!outputFilterInitialized_) {
        smoothedVelocity_[0] = 0.0;
        smoothedVelocity_[1] = 0.0;
        smoothedVelocity_[2] = velZ;
        smoothedAcceleration_[0] = accX;
        smoothedAcceleration_[1] = accY;
        smoothedAcceleration_[2] = accZ;
        outputFilterInitialized_ = true;
    } else {
        const double maxDv = kMaxOutputAccelMps2 * dt;
        const double targetVx = 0.0;
        const double targetVy = 0.0;
        const double targetVz = smoothedVelocity_[2] + alphaVel * (velZ - smoothedVelocity_[2]);
        smoothedVelocity_[0] = ApplySlewLimit(smoothedVelocity_[0], targetVx, maxDv);
        smoothedVelocity_[1] = ApplySlewLimit(smoothedVelocity_[1], targetVy, maxDv);
        smoothedVelocity_[2] = ApplySlewLimit(smoothedVelocity_[2], targetVz, maxDv);
        smoothedAcceleration_[0] += alphaAcc * (accX - smoothedAcceleration_[0]);
        smoothedAcceleration_[1] += alphaAcc * (accY - smoothedAcceleration_[1]);
        smoothedAcceleration_[2] += alphaAcc * (accZ - smoothedAcceleration_[2]);
    }

    // XY position/velocity are unobservable in the current estimator because there is
    // no horizontal measurement update. Publish zero instead of integrated drift.
    constexpr double kPublishedPosX = 0.0;
    constexpr double kPublishedPosY = 0.0;

    output.time = data.timestamp;
    output.position[0] = static_cast<float>(kPublishedPosX);
    output.position[1] = static_cast<float>(kPublishedPosY);
    output.position[2] = static_cast<float>(posZ);
    output.velocity[0] = static_cast<float>(smoothedVelocity_[0]);
    output.velocity[1] = static_cast<float>(smoothedVelocity_[1]);
    output.velocity[2] = static_cast<float>(smoothedVelocity_[2]);
    output.acceleration[0] = static_cast<float>(smoothedAcceleration_[0]);
    output.acceleration[1] = static_cast<float>(smoothedAcceleration_[1]);
    output.acceleration[2] = static_cast<float>(smoothedAcceleration_[2]);
    output.inertialAcceleration[0] = static_cast<float>(inertialAcceleration.x);
    output.inertialAcceleration[1] = static_cast<float>(inertialAcceleration.y);
    output.inertialAcceleration[2] = static_cast<float>(inertialAcceleration.z);
    output.zenith = static_cast<float>(zenithRadians_);
    output.apogeeEstimate = static_cast<float>(lastApogeePrediction_);

    lastZenith_ = zenithRadians_;

    return true;
}

/// Resets filters, phase latches, and predictor-side history.
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
    liftoffCandidateCount_ = 0;
    burnoutCandidateCount_ = 0;
    outputFilterInitialized_ = false;
    smoothedVelocity_[0] = 0.0;
    smoothedVelocity_[1] = 0.0;
    smoothedVelocity_[2] = 0.0;
    smoothedAcceleration_[0] = 0.0;
    smoothedAcceleration_[1] = 0.0;
    smoothedAcceleration_[2] = 0.0;
    ResetPredictorHorizontalVelocityTracker(predictorHorizontalVelocity_);
}

/// Emits a human-readable flight event to the serial logger.
void FlightComputer::ReportEvent(bool includeAltitude, float timeSeconds, const char *label) {
    LOG_PRINT(label);
    LOG_PRINT(" at t = ");
    LOG_PRINT(timeSeconds, 4);
    LOG_PRINT(" s");
    if (includeAltitude) {
        LOG_PRINT(", altitude = ");
        LOG_PRINT(apogeeAltitude_, 2);
        LOG_PRINT(" m");
    }
    LOG_PRINTLN();
}

/// Integrates quaternion attitude one sample forward using gyro data only.
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

    math_utils::Quaternion updated = math_utils::MakeQuaternion(
        qw + dq_w,
        qx + dq_x,
        qy + dq_y,
        qz + dq_z);
    return math_utils::Normalize(updated);
}

/// Converts raw telemetry quaternion storage into the internal math type.
math_utils::Quaternion FlightComputer::ArrayToQuaternion(const float values[4]) const {
    return math_utils::Normalize(math_utils::MakeQuaternion(values[0], values[1], values[2], values[3]));
}

/// Returns a stable string label for a flight status value.
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
