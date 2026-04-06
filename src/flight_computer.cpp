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
    apogeePredictor_.ResetAxialDragScale();
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
    if (!altitudeReferenceInitialized_ && std::isfinite(altitudeMeters)) {
        altitudeReferenceMeters_ = altitudeMeters;
        altitudeReferenceInitialized_ = true;
        lastGroundRelativeAltitudeMeters_ = 0.0;
        groundRelativeVelocityMps_ = 0.0;
    }
    const double relativeAltitudeMeters =
        altitudeReferenceInitialized_ ? (altitudeMeters - altitudeReferenceMeters_) : 0.0;

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
        math_utils::Quaternion inputQuaternion = math_utils::MakeQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
        if (ArrayToQuaternion(data.quaternion, inputQuaternion)) {
            previousQuaternion_ = inputQuaternion;
            quaternionValid_ = true;
            orientation = previousQuaternion_;
        }
    } else if ((status_ == FlightStatus::Burn || status_ == FlightStatus::Coast) && quaternionValid_) {
        // Propagate attitude through ascent with gyro-only integration so brief
        // quaternion dropouts do not immediately collapse the predictor seed.
        bool propagatedQuaternionValid = true;
        orientation = TeasleyFilter(previousQuaternion_, data.gyro, static_cast<float>(dt), &propagatedQuaternionValid);
        quaternionValid_ = propagatedQuaternionValid;

        // When the runtime source selector marks the main quaternion as BNO-led
        // or blended, treat it as a slow external reference and trim the
        // propagated attitude back toward it instead of hard-switching.
        // During coast, use more aggressive correction to quickly fix any
        // gyro drift accumulated during burn.
        const MainQuaternionSource correctionSource =
            static_cast<MainQuaternionSource>(data.mainQuaternionSource);
        if (settings::ahrs::kEnableBnoReferenceCorrection &&
            (status_ == FlightStatus::Burn || status_ == FlightStatus::Coast) &&
            data.hasQuaternion &&
            (correctionSource == MainQuaternionSource::Bno ||
             correctionSource == MainQuaternionSource::Blended)) {
            math_utils::Quaternion referenceQuat = math_utils::MakeQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
            if (ArrayToQuaternion(data.quaternion, referenceQuat)) {
                // Use higher blend factor during coast to aggressively correct drift
                const float baseBlendFactor = (status_ == FlightStatus::Coast)
                    ? settings::ahrs::kBnoCoastCorrectionBlendFactor
                    : settings::ahrs::kBnoReferenceCorrectionBlendFactor;
                const float blendFactor = baseBlendFactor * std::max(0.0f, data.icmAccelTrust);
                if (blendFactor > 0.0f) {
                    orientation = math_utils::Slerp(orientation, referenceQuat, blendFactor);
                    quaternionValid_ = math_utils::ValidateQuaternion(orientation);
                }
            }
        }

        previousQuaternion_ = orientation;
    } else if (data.hasQuaternion) {
        math_utils::Quaternion inputQuaternion = math_utils::MakeQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
        if (ArrayToQuaternion(data.quaternion, inputQuaternion)) {
            orientation = inputQuaternion;
            previousQuaternion_ = orientation;
            quaternionValid_ = true;
        }
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
                    static_cast<float>(relativeAltitudeMeters),
                    static_cast<double>(data.altimeterSigmaScale),
                    static_cast<double>(data.altimeterGateSigma));

    const double rawPosZ = kalmanZ_.Position();
    const double rawVelZ = kalmanZ_.Velocity();
    const double accX = kalmanX_.Acceleration();
    const double accY = kalmanY_.Acceleration();
    const double accZ = kalmanZ_.Acceleration();
    double publishedPosZ = rawPosZ;
    double publishedVelZ = rawVelZ;

    if (status_ == FlightStatus::Ground) {
        const double rawGroundVelocityMps =
            (dt > 0.0) ? ((relativeAltitudeMeters - lastGroundRelativeAltitudeMeters_) / dt) : 0.0;
        constexpr double kGroundVelocityBlend = 0.2;
        groundRelativeVelocityMps_ +=
            kGroundVelocityBlend * (rawGroundVelocityMps - groundRelativeVelocityMps_);
        lastGroundRelativeAltitudeMeters_ = relativeAltitudeMeters;

        const bool accelerationSuggestsLiftoff =
            accZ > settings::flight::kLiftoffAccelerationThresholdMps2;
        const bool altitudeSuggestsLiftoff =
            std::fabs(relativeAltitudeMeters) > settings::flight::kLiftoffAltitudeThresholdM;
        const bool velocitySuggestsLiftoff =
            groundRelativeVelocityMps_ > settings::flight::kLiftoffVelocityThresholdMps;

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
            groundRelativeVelocityMps_ = 0.0;
        } else {
            // Hold the vertical filter at the pad while grounded so z/vz
            // cannot drift away from zero before liftoff is confirmed.
            kalmanZ_.Reset();
            publishedPosZ = 0.0;
            publishedVelZ = 0.0;
            smoothedVelocity_[2] = 0.0;
        }
    }

    if (status_ == FlightStatus::Burn) {
        const double timeSinceBurn = static_cast<double>(data.timestamp) - burnTimestamp_;
        const bool afterMinimumBurn = timeSinceBurn >= settings::flight::kBurnoutMinDurationSeconds;
        const bool accelerationSuggestsBurnout = accZ < settings::flight::kBurnoutAccelerationThresholdMps2;
        const bool stillAscending = rawVelZ > settings::flight::kBurnoutVelocityThresholdMps;
        const bool belowTarget = rawPosZ < apogeeTargetMeters_;

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
            // Initialize wind estimation at coast start.
            coastStartTime_ = static_cast<double>(data.timestamp);
            windEstimationActive_ = settings::predictor::kEnableWindEstimation;
            ReportEvent(false, data.timestamp, "Engine burnout");
        }
    }

    if (status_ == FlightStatus::Coast) {
        if (accZ < settings::flight::kBurnoutAccelerationThresholdMps2 && rawPosZ >= apogeeTargetMeters_) {
            status_ = FlightStatus::Overshoot;
            ReportEvent(false, data.timestamp, "Overshoot");
        }
    }

    if (status_ == FlightStatus::Overshoot || status_ == FlightStatus::Coast) {
        if (accZ < settings::flight::kDescentAccelerationThresholdMps2 &&
            rawVelZ <= settings::flight::kDescentVelocityThresholdMps) {
            status_ = FlightStatus::Descent;
            apogeeAltitude_ = rawPosZ;
            apogeeTimestamp_ = static_cast<double>(data.timestamp);
            apogeeRecorded_ = true;
            ReportEvent(true, data.timestamp, "Apogee reached");
        }
    }

    const bool shouldPredictApogee =
        (status_ == FlightStatus::Burn || status_ == FlightStatus::Coast) && rawVelZ > 0.0;
    if (shouldPredictApogee) {
        // Deliberately degrade to a simpler predictor seed whenever attitude
        // freshness is questionable rather than integrating unstable XY terms.
        const double seedZenith = quaternionValid_ ? SanitizePredictorZenithRadians(zenithRadians_) : 0.0;
        const bool freshSeedSample = PredictorSeedHasFreshSample(dt);
        const bool canUseHorizontalSeed = quaternionValid_ && freshSeedSample;
        if (!canUseHorizontalSeed) {
            ResetPredictorHorizontalVelocityTracker(predictorHorizontalVelocity_);
        }
        const double trackedHorizontalVelocity =
            canUseHorizontalSeed
                ? UpdatePredictorHorizontalSpeed(predictorHorizontalVelocity_,
                                                 inertialAcceleration.x,
                                                 inertialAcceleration.y,
                                                 dt,
                                                 true,
                                                 rawVelZ,
                                                 seedZenith)
                : 0.0;
        const double predictorHorizontalVelocity =
            canUseHorizontalSeed
                ? ResolvePredictorHorizontalSpeed(trackedHorizontalVelocity, rawVelZ, seedZenith)
                : 0.0;
        const double predictorAngularRate =
            canUseHorizontalSeed
                ? ClampPredictorAngularRate(ComputePredictorAngularRate(zenithRadians_, lastZenith_, dt))
                : 0.0;

        ApogeeState predictorState;
        predictorState.altitudeMeters = rawPosZ;
        predictorState.horizontalDistanceMeters = 0.0;
        predictorState.verticalVelocity = rawVelZ;
        predictorState.horizontalVelocity = predictorHorizontalVelocity;
        predictorState.zenith = seedZenith;
        predictorState.angularVelocity = predictorAngularRate;
        // Single integration pass returns both altitude and time-to-apogee
        const PredictResult prediction = apogeePredictor_.PredictApogeeWithTime(predictorState);
        UpdateAdaptiveDragScale(predictorState, accZ, dt, prediction.timeToApogee);
        lastApogeePrediction_ = prediction.altitude;
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
        smoothedVelocity_[2] = publishedVelZ;
        smoothedAcceleration_[0] = accX;
        smoothedAcceleration_[1] = accY;
        smoothedAcceleration_[2] = accZ;
        outputFilterInitialized_ = true;
    } else {
        const double maxDv = kMaxOutputAccelMps2 * dt;
        const double targetVx = 0.0;
        const double targetVy = 0.0;
        const double targetVz =
            smoothedVelocity_[2] + alphaVel * (publishedVelZ - smoothedVelocity_[2]);
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
    output.position[2] = static_cast<float>(publishedPosZ);
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

/// Updates the adaptive axial drag correction during coast.
/// Now supports both legacy single-scale and Mach-dependent adaptation.
void FlightComputer::UpdateAdaptiveDragScale(const ApogeeState &predictorState,
                                             double measuredVerticalAcceleration,
                                             double dtSeconds,
                                             double timeToApogeeSeconds) {
    if (status_ != FlightStatus::Coast ||
        predictorState.verticalVelocity <= 0.0 ||
        !quaternionValid_ ||
        !PredictorSeedHasFreshSample(dtSeconds) ||
        !std::isfinite(measuredVerticalAcceleration) ||
        !std::isfinite(predictorState.zenith)) {
        return;
    }

    double axialModelAcceleration = 0.0;
    const double predictedVerticalAcceleration =
        apogeePredictor_.ComputeVerticalAcceleration(predictorState, &axialModelAcceleration);
    if (!std::isfinite(predictedVerticalAcceleration) || !std::isfinite(axialModelAcceleration)) {
        return;
    }

    const double minAxialAccel =
        static_cast<double>(settings::flight::kAdaptiveAxialAccelMinAbsMps2);
    if (std::fabs(axialModelAcceleration) < minAxialAccel) {
        return;
    }

    const double residualClamp =
        static_cast<double>(settings::flight::kAdaptiveAxialDragResidualClampMps2);
    const double residual =
        std::clamp(measuredVerticalAcceleration - predictedVerticalAcceleration,
                   -residualClamp,
                   residualClamp);

    // Compute Mach number for Mach-dependent adaptation.
    double mach = 0.0;
    const double temperature = environment_.TemperatureKelvin(predictorState.altitudeMeters);
    if (temperature > 0.0) {
        const double speedOfSound = std::sqrt(constants::kGamma * constants::kGasConstant * temperature);
        const double totalSpeed = std::sqrt(predictorState.verticalVelocity * predictorState.verticalVelocity +
                                            predictorState.horizontalVelocity * predictorState.horizontalVelocity);
        if (speedOfSound > 0.0) {
            mach = totalSpeed / speedOfSound;
        }
    }

    // Update Mach-dependent drag scale if enabled.
    if (settings::predictor::kEnableMachDependentDrag) {
        apogeePredictor_.AdaptMachDragScale(mach,
                                           residual,
                                           axialModelAcceleration,
                                           dtSeconds,
                                           std::max(0.0, timeToApogeeSeconds));
        // Keep the legacy scalar neutral when the Mach-binned learner is
        // active so the same residual does not adapt two drag models at once.
        apogeePredictor_.SetAxialDragScale(1.0);
        return;
    }

    // Legacy single-scale adaptation path when Mach-dependent learning is disabled.
    const double currentScale = apogeePredictor_.AxialDragScale();
    const double minScale =
        static_cast<double>(settings::flight::kAdaptiveAxialDragScaleMin);
    const double maxScale =
        static_cast<double>(settings::flight::kAdaptiveAxialDragScaleMax);
    const double targetScale =
        std::clamp(currentScale + (residual / axialModelAcceleration), minScale, maxScale);

    const double tauSeconds =
        static_cast<double>(settings::flight::kAdaptiveAxialDragTauSeconds);
    const double alpha = (dtSeconds > 0.0 && tauSeconds > 0.0)
                             ? (1.0 - std::exp(-dtSeconds / tauSeconds))
                             : 0.0;
    const double updatedScale =
        std::clamp(currentScale + alpha * (targetScale - currentScale), minScale, maxScale);
    apogeePredictor_.SetAxialDragScale(updatedScale);

    // Wind estimation: track horizontal acceleration residual.
    if (settings::predictor::kEnableWindEstimation && windEstimationActive_) {
        // Horizontal acceleration residual suggests wind offset.
        // This is a simplified estimation - assumes horizontal accel mismatch is wind-induced.
        const double timeSinceCoast = lastTimestamp_ - coastStartTime_;
        if (timeSinceCoast >= settings::predictor::kWindEstimateMinCoastTimeSec) {
            // Low-pass filter the wind estimate based on horizontal accel.
            // For simplicity, we estimate wind as affecting the horizontal velocity seed.
            const double blendRate = settings::predictor::kWindEstimateBlendRate;
            const double maxWind = settings::predictor::kWindEstimateMaxMps;
            // Update the environment model's wind offset.
            math_utils::Vec3 currentOffset = environment_.WindOffset();
            // Use horizontal accel residual as proxy for wind effect.
            // This is an approximation - true wind estimation would require more state.
            const double horizAccelResidual = 0.0;  // Placeholder for future horizontal accel model comparison
            currentOffset.y = static_cast<float>(std::clamp(
                static_cast<double>(currentOffset.y) + blendRate * horizAccelResidual,
                -maxWind, maxWind));
            environment_.SetWindOffset(currentOffset);
        }
    }
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
    altitudeReferenceInitialized_ = false;
    altitudeReferenceMeters_ = 0.0;
    lastGroundRelativeAltitudeMeters_ = 0.0;
    groundRelativeVelocityMps_ = 0.0;
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
    apogeePredictor_.ResetAxialDragScale();
    // Reset wind estimation state.
    windEstimateHorizontalMps_ = 0.0;
    coastStartTime_ = 0.0;
    windEstimationActive_ = false;
    environment_.SetWindOffset(math_utils::MakeVec3(0.0f, 0.0f, 0.0f));
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
/// Uses exponential map when enabled for reduced integration error.
math_utils::Quaternion FlightComputer::TeasleyFilter(const math_utils::Quaternion &quat,
                                                     const float gyro[3],
                                                     float dt,
                                                     bool *validOut) {
    math_utils::Quaternion validatedQuat = quat;
    if (!math_utils::ValidateQuaternion(validatedQuat)) {
        if (validOut != nullptr) {
            *validOut = false;
        }
        return validatedQuat;
    }

    if (settings::ahrs::kEnableExponentialMap) {
        // Exponential map integration using Rodrigues formula.
        // More accurate than first-order Euler: reduces O(dt^2) error per step.
        math_utils::Quaternion updated =
            math_utils::ExponentialMapUpdate(validatedQuat, gyro[0], gyro[1], gyro[2], dt);
        const bool isValid = math_utils::ValidateQuaternion(updated);
        if (validOut != nullptr) {
            *validOut = isValid;
        }
        return updated;
    }

    // Legacy first-order Euler integration.
    const float half_dt = 0.5f * dt;
    const float qw = validatedQuat.w;
    const float qx = validatedQuat.x;
    const float qy = validatedQuat.y;
    const float qz = validatedQuat.z;
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
    const bool isValid = math_utils::ValidateQuaternion(updated);
    if (validOut != nullptr) {
        *validOut = isValid;
    }
    return updated;
}

/// Converts raw telemetry quaternion storage into the internal math type.
bool FlightComputer::ArrayToQuaternion(const float values[4], math_utils::Quaternion &out) const {
    out = math_utils::MakeQuaternion(values[0], values[1], values[2], values[3]);
    return math_utils::ValidateQuaternion(out);
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
