#include "flight_computer.h"

#include <cmath>

#include "constants.h"
#include "math_utils.h"
#include "serial_logging.h"
#include "settings.h"

namespace {

/// Rotates body-frame acceleration into the filter frame using the main quaternion.
///
/// The filter publishes vertical acceleration in `.z`; keep that vertical
/// axis aligned with the quaternion zenith convention (`R[2][2]`).
math_utils::Vec3 RotateBodyToInertial(const math_utils::Vec3 &bodyAccel,
                                      const math_utils::Quaternion &orientation) {
    const float w = orientation.w;
    const float x = orientation.x;
    const float y = orientation.y;
    const float z = orientation.z;

    const float r00 = 1.0f - 2.0f * (y * y + z * z);
    const float r01 = 2.0f * (x * y - w * z);
    const float r02 = 2.0f * (x * z + w * y);
    const float r10 = 2.0f * (x * y + w * z);
    const float r11 = 1.0f - 2.0f * (x * x + z * z);
    const float r12 = 2.0f * (y * z - w * x);
    const float r20 = 2.0f * (x * z - w * y);
    const float r21 = 2.0f * (y * z + w * x);
    const float r22 = 1.0f - 2.0f * (x * x + y * y);

    const float earthX = r00 * bodyAccel.x + r10 * bodyAccel.y + r20 * bodyAccel.z;
    const float earthY = r01 * bodyAccel.x + r11 * bodyAccel.y + r21 * bodyAccel.z;
    const float earthZ = r02 * bodyAccel.x + r12 * bodyAccel.y + r22 * bodyAccel.z;

    math_utils::Vec3 result;
    result.x = earthX;
    result.y = earthY;
    result.z = earthZ - constants::kGravity;
    return result;
}

bool GravityBodyVectorFromQuaternion(const math_utils::Quaternion &input,
                                     math_utils::Vec3 *gravityBodyOut) {
    if (gravityBodyOut == nullptr) {
        return false;
    }
    math_utils::Quaternion orientation = input;
    if (!math_utils::ValidateQuaternion(orientation)) {
        return false;
    }
    const float w = orientation.w;
    const float x = orientation.x;
    const float y = orientation.y;
    const float z = orientation.z;

    gravityBodyOut->x = -2.0f * (x * z - w * y);
    gravityBodyOut->y = -2.0f * (y * z + w * x);
    gravityBodyOut->z = -(1.0f - 2.0f * (x * x + y * y));
    if (gravityBodyOut->z < 0.0f) {
        gravityBodyOut->x = -gravityBodyOut->x;
        gravityBodyOut->y = -gravityBodyOut->y;
        gravityBodyOut->z = -gravityBodyOut->z;
    }
    const float magnitudeSq =
        gravityBodyOut->x * gravityBodyOut->x +
        gravityBodyOut->y * gravityBodyOut->y +
        gravityBodyOut->z * gravityBodyOut->z;
    if (!std::isfinite(magnitudeSq) || magnitudeSq <= 1.0e-12f) {
        return false;
    }
    const float invMagnitude = 1.0f / std::sqrt(magnitudeSq);
    gravityBodyOut->x *= invMagnitude;
    gravityBodyOut->y *= invMagnitude;
    gravityBodyOut->z *= invMagnitude;
    return true;
}

bool QuaternionTiltDifferenceDeg(const math_utils::Quaternion &a,
                                 const math_utils::Quaternion &b,
                                 float *differenceDegOut) {
    if (differenceDegOut == nullptr) {
        return false;
    }
    math_utils::Vec3 gravityA = math_utils::MakeVec3(0.0f, 0.0f, 0.0f);
    math_utils::Vec3 gravityB = math_utils::MakeVec3(0.0f, 0.0f, 0.0f);
    if (!GravityBodyVectorFromQuaternion(a, &gravityA) ||
        !GravityBodyVectorFromQuaternion(b, &gravityB)) {
        return false;
    }
    const float dot = math_utils::Clamp(math_utils::Dot(gravityA, gravityB), -1.0f, 1.0f);
    *differenceDegOut = acosf(dot) * (180.0f / 3.14159265358979323846f);
    return std::isfinite(*differenceDegOut);
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

struct FilterPhaseTuning {
    double accelSigmaScale = 1.0;
    double altitudeSigmaScale = 1.0;
    double processNoiseXYScale = 1.0;
    double processNoiseZScale = 1.0;
};

FilterPhaseTuning ComputeFilterPhaseTuning(FlightStatus status) {
    switch (status) {
        case FlightStatus::Ground:
            return {settings::flight::kGroundAccelSigmaScale,
                    settings::flight::kGroundAltSigmaScale,
                    settings::flight::kGroundProcessNoiseXYScale,
                    settings::flight::kGroundProcessNoiseZScale};
        case FlightStatus::Burn:
            return {settings::flight::kBurnAccelSigmaScale,
                    settings::flight::kBurnAltSigmaScale,
                    settings::flight::kBurnProcessNoiseXYScale,
                    settings::flight::kBurnProcessNoiseZScale};
        case FlightStatus::Descent:
            return {settings::flight::kDescentAccelSigmaScale,
                    settings::flight::kDescentAltSigmaScale,
                    settings::flight::kDescentProcessNoiseXYScale,
                    settings::flight::kDescentProcessNoiseZScale};
        case FlightStatus::Coast:
        case FlightStatus::Overshoot:
        default:
            return {settings::flight::kCoastAccelSigmaScale,
                    settings::flight::kCoastAltSigmaScale,
                    settings::flight::kCoastProcessNoiseXYScale,
                    settings::flight::kCoastProcessNoiseZScale};
    }
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
    kalmanZ_.SetBiasProcessSigma(settings::flight::kProcessNoiseZBias);

    accelSigmaXY_ = sigmaAccelXY;
    accelSigmaZ_ = sigmaAccelZ;
    altitudeSigma_ = sigmaAltimeter;
    processNoiseXY_ = processXY;
    processNoiseZ_ = processZ;
    apogeeTargetMeters_ = apogeeTargetMeters;

    ResetInternalState();
}

/// Updates the runtime-configurable environment/vehicle parameters in-place.
void FlightComputer::ReconfigurePredictor(const EnvironmentModel::Config &environmentConfig,
                                          const ApogeeVehicleParameters &vehicleParameters,
                                          const ApogeeForceTable *forceTable,
                                          bool preserveAdaptiveState) {
    environment_.Configure(environmentConfig);
    apogeePredictor_.SetEnvironment(environment_);
    apogeePredictor_.SetVehicleParameters(vehicleParameters);
    apogeePredictor_.SetForceTable(forceTable);
    if (!preserveAdaptiveState) {
        apogeePredictor_.ResetAxialDragScale();
    }
}

void FlightComputer::ResetGroundReference() {
    altitudeReferenceInitialized_ = false;
    altitudeReferenceMeters_ = 0.0;
    lastGroundRelativeAltitudeMeters_ = 0.0;
    lastGroundRelativeAltitudeTimestamp_ = 0.0;
    groundRelativeVelocityMps_ = 0.0;
    maxObservedAltitude_ = 0.0;
    groundReferenceDriftRateMps_ = 0.0;
    groundReferenceStableSince_ = 0.0;
    groundReferenceSettled_ = false;
    liftoffCandidateCount_ = 0;
    baroLiftoffCandidateCount_ = 0;
    burnDetectTimestamp_ = 0.0;
    burnoutCandidateCount_ = 0;
    kalmanZ_.Reset();
    smoothedVelocity_[2] = 0.0;
    bnoFreshPostBurnoutQuaternionCount_ = 0;
    lastBnoReferenceCorrectionSampleMicros_ = 0;
    ResetBaroVelocityGuardState();
}

/// Processes one sensor sample and updates the filtered flight state.
bool FlightComputer::Update(const SensorData &data, FilteredState &output) {
    const bool hasIcmAccel =
        !(data.accelICM[0] == 0.0f && data.accelICM[1] == 0.0f && data.accelICM[2] == 0.0f);
    const bool hasLsmAccel =
        !(data.accelLSM[0] == 0.0f && data.accelLSM[1] == 0.0f && data.accelLSM[2] == 0.0f);
    const bool hasPulseAccel =
        !(data.accelPulse[0] == 0.0f && data.accelPulse[1] == 0.0f && data.accelPulse[2] == 0.0f);
    const bool hasIcmGyro =
        !(data.gyro[0] == 0.0f && data.gyro[1] == 0.0f && data.gyro[2] == 0.0f);
    const bool hasLsmGyro =
        !(data.gyroLSM[0] == 0.0f && data.gyroLSM[1] == 0.0f && data.gyroLSM[2] == 0.0f);
    const bool hasPulseGyro =
        !(data.gyroPulse[0] == 0.0f && data.gyroPulse[1] == 0.0f && data.gyroPulse[2] == 0.0f);
    const bool hasAnyAccel = hasIcmAccel || hasLsmAccel || hasPulseAccel;
    if (!hasAnyAccel && !data.baroSampleFresh) {
        return false;  // No fresh altitude and no accel data means the filter cannot advance safely.
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
    if (data.baroSampleFresh && std::isfinite(altitudeMeters)) {
        if (!altitudeReferenceInitialized_) {
            altitudeReferenceMeters_ = altitudeMeters;
            altitudeReferenceInitialized_ = true;
            lastGroundRelativeAltitudeMeters_ = 0.0;
            groundRelativeVelocityMps_ = 0.0;
            groundReferenceDriftRateMps_ = 0.0;
            groundReferenceStableSince_ = static_cast<double>(data.timestamp);
            groundReferenceSettled_ = false;
        } else if (status_ == FlightStatus::Ground && burnDetectTimestamp_ <= 0.0) {
            const double previousReferenceMeters = altitudeReferenceMeters_;
            const double referenceAlpha = ComputeSmoothingAlpha(
                dt,
                settings::flight::kGroundAltitudeReferenceTauSeconds);
            altitudeReferenceMeters_ += referenceAlpha * (altitudeMeters - altitudeReferenceMeters_);
            if (dt > 1.0e-4) {
                const double rawDriftRateMps = (altitudeReferenceMeters_ - previousReferenceMeters) / dt;
                const double driftAlpha = ComputeSmoothingAlpha(
                    dt,
                    settings::flight::kPadReferenceDriftTauSeconds);
                groundReferenceDriftRateMps_ +=
                    driftAlpha * (rawDriftRateMps - groundReferenceDriftRateMps_);
            }
            const bool driftIsStable =
                std::fabs(groundReferenceDriftRateMps_) <= settings::flight::kPadReadyMaxDriftMps;
            if (driftIsStable) {
                if (groundReferenceStableSince_ <= 0.0) {
                    groundReferenceStableSince_ = static_cast<double>(data.timestamp);
                }
                groundReferenceSettled_ =
                    (static_cast<double>(data.timestamp) - groundReferenceStableSince_) >=
                    settings::flight::kPadReadyHoldSeconds;
            } else {
                groundReferenceStableSince_ = 0.0;
                groundReferenceSettled_ = false;
            }
        }
    }
    const double relativeAltitudeMeters =
        altitudeReferenceInitialized_ ? (altitudeMeters - altitudeReferenceMeters_) : 0.0;
    const FilterPhaseTuning phaseTuning = ComputeFilterPhaseTuning(status_);
    const bool baroVzGuardActiveBeforeUpdate =
        baroVzGuardPersistenceCount_ >= settings::flight::kBaroVzGuardPersistenceSamples;
    const double zAccelSigmaScale =
        phaseTuning.accelSigmaScale *
        (baroVzGuardActiveBeforeUpdate ? settings::flight::kBaroVzGuardAccelSigmaScale : 1.0);
    kalmanX_.SetMeasurementSigma(accelSigmaXY_ * phaseTuning.accelSigmaScale);
    kalmanY_.SetMeasurementSigma(accelSigmaXY_ * phaseTuning.accelSigmaScale);
    kalmanZ_.SetMeasurementSigmas(accelSigmaZ_ * zAccelSigmaScale,
                                  altitudeSigma_ * phaseTuning.altitudeSigmaScale);

    float accelBody[3];
    float gyroBody[3] = {0.0f, 0.0f, 0.0f};
    const MainQuaternionSource selectedQuaternionSource =
        static_cast<MainQuaternionSource>(data.mainQuaternionSource);
    if (status_ != FlightStatus::Coast && status_ != FlightStatus::Overshoot) {
        bnoFreshPostBurnoutQuaternionCount_ = 0;
        lastBnoReferenceCorrectionSampleMicros_ = 0;
    }
    const bool hasFreshIcmAccel = hasIcmAccel && data.icmSampleFresh;
    const bool hasFreshLsmAccel = hasLsmAccel && data.lsmSampleFresh;
    const bool hasFreshPulseAccel = hasPulseAccel && data.pulseSampleFresh;
    const bool hasFreshAccelMeasurement =
        hasFreshIcmAccel || hasFreshLsmAccel || hasFreshPulseAccel;
    const bool hasFreshIcmGyro = hasIcmGyro && data.icmSampleFresh;
    const bool hasFreshLsmGyro = hasLsmGyro && data.lsmSampleFresh;
    const bool hasFreshPulseGyro = hasPulseGyro && data.pulseSampleFresh;
    const bool hasFreshAltitudeMeasurement =
        data.baroSampleFresh && altitudeReferenceInitialized_ && std::isfinite(relativeAltitudeMeters);
    if (hasFreshAltitudeMeasurement) {
        RecordBaroAltitudeSample(static_cast<double>(data.timestamp), relativeAltitudeMeters);
    }

    auto loadIcmGyro = [&]() {
        if (hasFreshIcmGyro) {
            gyroBody[0] = data.gyro[0];
            gyroBody[1] = data.gyro[1];
            gyroBody[2] = data.gyro[2];
        } else if (hasFreshLsmGyro) {
            gyroBody[0] = data.gyroLSM[0];
            gyroBody[1] = data.gyroLSM[1];
            gyroBody[2] = data.gyroLSM[2];
        } else if (hasIcmGyro) {
            gyroBody[0] = data.gyro[0];
            gyroBody[1] = data.gyro[1];
            gyroBody[2] = data.gyro[2];
        } else if (hasLsmGyro) {
            gyroBody[0] = data.gyroLSM[0];
            gyroBody[1] = data.gyroLSM[1];
            gyroBody[2] = data.gyroLSM[2];
        }
    };
    auto loadLsmGyro = [&]() {
        if (hasFreshLsmGyro) {
            gyroBody[0] = data.gyroLSM[0];
            gyroBody[1] = data.gyroLSM[1];
            gyroBody[2] = data.gyroLSM[2];
        } else if (hasFreshIcmGyro) {
            gyroBody[0] = data.gyro[0];
            gyroBody[1] = data.gyro[1];
            gyroBody[2] = data.gyro[2];
        } else if (hasLsmGyro) {
            gyroBody[0] = data.gyroLSM[0];
            gyroBody[1] = data.gyroLSM[1];
            gyroBody[2] = data.gyroLSM[2];
        } else if (hasIcmGyro) {
            gyroBody[0] = data.gyro[0];
            gyroBody[1] = data.gyro[1];
            gyroBody[2] = data.gyro[2];
        }
    };
    auto loadPulseGyro = [&]() {
        if (hasFreshPulseGyro) {
            gyroBody[0] = data.gyroPulse[0];
            gyroBody[1] = data.gyroPulse[1];
            gyroBody[2] = data.gyroPulse[2];
        } else if (hasFreshLsmGyro) {
            gyroBody[0] = data.gyroLSM[0];
            gyroBody[1] = data.gyroLSM[1];
            gyroBody[2] = data.gyroLSM[2];
        } else if (hasFreshIcmGyro) {
            gyroBody[0] = data.gyro[0];
            gyroBody[1] = data.gyro[1];
            gyroBody[2] = data.gyro[2];
        } else if (hasPulseGyro) {
            gyroBody[0] = data.gyroPulse[0];
            gyroBody[1] = data.gyroPulse[1];
            gyroBody[2] = data.gyroPulse[2];
        } else if (hasLsmGyro) {
            gyroBody[0] = data.gyroLSM[0];
            gyroBody[1] = data.gyroLSM[1];
            gyroBody[2] = data.gyroLSM[2];
        } else if (hasIcmGyro) {
            gyroBody[0] = data.gyro[0];
            gyroBody[1] = data.gyro[1];
            gyroBody[2] = data.gyro[2];
        }
    };

    // Keep single-rail attitude and accel/gyro measurements on the same rail
    // when possible. Blended/default attitude keeps the normal LSM -> ICM -> Pulse
    // preference. Cached accel samples are not reused for measurement updates.
    if (selectedQuaternionSource == MainQuaternionSource::Icm && hasFreshIcmAccel) {
        accelBody[0] = data.accelICM[0];
        accelBody[1] = data.accelICM[1];
        accelBody[2] = data.accelICM[2];
        loadIcmGyro();
    } else if (selectedQuaternionSource == MainQuaternionSource::Lsm && hasFreshLsmAccel) {
        accelBody[0] = data.accelLSM[0];
        accelBody[1] = data.accelLSM[1];
        accelBody[2] = data.accelLSM[2];
        loadLsmGyro();
    } else if (selectedQuaternionSource == MainQuaternionSource::Pulse && hasFreshPulseAccel) {
        accelBody[0] = data.accelPulse[0];
        accelBody[1] = data.accelPulse[1];
        accelBody[2] = data.accelPulse[2];
        loadPulseGyro();
    } else if (hasFreshLsmAccel) {
        accelBody[0] = data.accelLSM[0];
        accelBody[1] = data.accelLSM[1];
        accelBody[2] = data.accelLSM[2];
        loadLsmGyro();
    } else if (hasFreshIcmAccel) {
        accelBody[0] = data.accelICM[0];
        accelBody[1] = data.accelICM[1];
        accelBody[2] = data.accelICM[2];
        loadIcmGyro();
    } else if (hasFreshPulseAccel) {
        accelBody[0] = data.accelPulse[0];
        accelBody[1] = data.accelPulse[1];
        accelBody[2] = data.accelPulse[2];
        loadPulseGyro();
    } else {
        accelBody[0] = 0.0f;
        accelBody[1] = 0.0f;
        accelBody[2] = 0.0f;
        loadPulseGyro();
    }

    math_utils::Quaternion orientation = previousQuaternion_;
    float bnoReferenceTiltErrorDeg = NAN;
    bool bnoReferenceCorrectionApplied = false;
    if (!quaternionValid_ && data.hasQuaternion) {
        math_utils::Quaternion inputQuaternion = math_utils::MakeQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
        if (ArrayToQuaternion(data.quaternion, inputQuaternion)) {
            previousQuaternion_ = inputQuaternion;
            quaternionValid_ = true;
            orientation = previousQuaternion_;
        }
    } else if ((status_ == FlightStatus::Burn ||
                status_ == FlightStatus::Coast ||
                status_ == FlightStatus::Overshoot) && quaternionValid_) {
        // Propagate attitude through ascent with gyro-only integration so brief
        // quaternion dropouts do not immediately collapse the predictor seed.
        bool propagatedQuaternionValid = true;
        orientation = TeasleyFilter(previousQuaternion_, gyroBody, static_cast<float>(dt), &propagatedQuaternionValid);
        quaternionValid_ = propagatedQuaternionValid;

        // The BNO055 saturates well before peak boost acceleration, so only use
        // it as a trim reference once the vehicle has transitioned into coast.
        // Ramp its influence up from the configured coast blend factor so the
        // zenith estimate does not jump immediately at burnout.
        if (settings::ahrs::kEnableBnoReferenceCorrection &&
            (status_ == FlightStatus::Coast || status_ == FlightStatus::Overshoot) &&
            data.hasBnoQuaternion &&
            (selectedQuaternionSource == MainQuaternionSource::Icm ||
             selectedQuaternionSource == MainQuaternionSource::Lsm ||
             selectedQuaternionSource == MainQuaternionSource::Blended)) {
            math_utils::Quaternion referenceQuat = math_utils::MakeQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
            if (ArrayToQuaternion(data.quaternionBNO, referenceQuat)) {
                const bool hasFreshPostBurnoutBnoQuaternion =
                    data.bnoQuaternionFresh &&
                    data.bnoQuaternionSampleMicros != 0u &&
                    burnoutTimestamp_ > 0.0 &&
                    data.bnoQuaternionTimestampS > burnoutTimestamp_ &&
                    std::isfinite(data.bnoQuaternionAgeMs) &&
                    data.bnoQuaternionAgeMs >= 0.0f &&
                    data.bnoQuaternionAgeMs <= settings::ahrs::kBnoCoastCorrectionMaxSampleAgeMs;
                if (hasFreshPostBurnoutBnoQuaternion &&
                    QuaternionTiltDifferenceDeg(orientation,
                                                referenceQuat,
                                                &bnoReferenceTiltErrorDeg) &&
                    bnoReferenceTiltErrorDeg <= settings::ahrs::kBnoCoastCorrectionMaxTiltAgreementDeg) {
                    if (bnoFreshPostBurnoutQuaternionCount_ <
                        settings::ahrs::kBnoCoastCorrectionMinFreshSamples) {
                        ++bnoFreshPostBurnoutQuaternionCount_;
                    }
                    if (bnoFreshPostBurnoutQuaternionCount_ >=
                            settings::ahrs::kBnoCoastCorrectionMinFreshSamples &&
                        data.bnoQuaternionSampleMicros != lastBnoReferenceCorrectionSampleMicros_) {
                        float blendFactor = settings::ahrs::kBnoCoastCorrectionBlendFactor;
                        if (settings::ahrs::kEnableBnoCoastBlending) {
                            const float initialBlendFactor = settings::ahrs::kBnoCoastBlendFactor;
                            blendFactor = initialBlendFactor;
                            const double rampDurationSeconds =
                                static_cast<double>(settings::ahrs::kBnoCoastCorrectionRampSeconds);
                            if (rampDurationSeconds > 0.0 && burnoutTimestamp_ > 0.0) {
                                const double timeSinceBurnout =
                                    std::max(0.0, static_cast<double>(data.timestamp) - burnoutTimestamp_);
                                const double rampFraction =
                                    std::clamp(timeSinceBurnout / rampDurationSeconds, 0.0, 1.0);
                                blendFactor = static_cast<float>(
                                    static_cast<double>(initialBlendFactor) +
                                    rampFraction * static_cast<double>(
                                        settings::ahrs::kBnoCoastCorrectionBlendFactor -
                                        initialBlendFactor));
                            }
                        }
                        if (blendFactor > 0.0f) {
                            orientation = math_utils::Slerp(orientation, referenceQuat, blendFactor);
                            quaternionValid_ = math_utils::ValidateQuaternion(orientation);
                            if (quaternionValid_) {
                                lastBnoReferenceCorrectionSampleMicros_ = data.bnoQuaternionSampleMicros;
                                bnoReferenceCorrectionApplied = true;
                            }
                        }
                    }
                } else if (data.bnoQuaternionFresh) {
                    bnoFreshPostBurnoutQuaternionCount_ = 0;
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

    if (quaternionValid_) {
        zenithRadians_ = static_cast<double>(math_utils::QuaternionToZenith(orientation));
        if (!std::isfinite(zenithRadians_)) {
            zenithRadians_ = 0.0;
            quaternionValid_ = false;
        }
    } else {
        zenithRadians_ = 0.0;
    }

    const math_utils::Vec3 bodyAccel = math_utils::MakeVec3(
        accelBody[0],
        accelBody[1],
        accelBody[2]);
    const math_utils::Vec3 inertialAcceleration = hasFreshAccelMeasurement
        ? (quaternionValid_
               ? RotateBodyToInertial(bodyAccel, orientation)
               : RotateBodyToInertial(bodyAccel, previousQuaternion_))
        : math_utils::MakeVec3(0.0f, 0.0f, 0.0f);

    kalmanX_.Predict(dt, processNoiseXY_ * phaseTuning.processNoiseXYScale);
    kalmanY_.Predict(dt, processNoiseXY_ * phaseTuning.processNoiseXYScale);
    kalmanZ_.Predict(dt, processNoiseZ_ * phaseTuning.processNoiseZScale);

    bool zAccelUpdateUsed = false;
    if (hasFreshAccelMeasurement) {
        kalmanX_.Update(inertialAcceleration.x);
        kalmanY_.Update(inertialAcceleration.y);
    }
    if (hasFreshAccelMeasurement && hasFreshAltitudeMeasurement) {
        zAccelUpdateUsed =
            kalmanZ_.UpdateAccelOnly(static_cast<double>(inertialAcceleration.z),
                                     static_cast<double>(settings::flight::kAccelInnovationGateSigma));
        kalmanZ_.UpdateAltitudeOnly(relativeAltitudeMeters,
                                    static_cast<double>(data.altimeterSigmaScale),
                                    static_cast<double>(data.altimeterGateSigma));
    } else if (hasFreshAccelMeasurement) {
        zAccelUpdateUsed =
            kalmanZ_.UpdateAccelOnly(static_cast<double>(inertialAcceleration.z),
                                     static_cast<double>(settings::flight::kAccelInnovationGateSigma));
    } else if (hasFreshAltitudeMeasurement) {
        kalmanZ_.UpdateAltitudeOnly(relativeAltitudeMeters,
                                    static_cast<double>(data.altimeterSigmaScale),
                                    static_cast<double>(data.altimeterGateSigma));
    }

    double rawPosZ = kalmanZ_.Position();
    double rawVelZ = kalmanZ_.Velocity();
    const double accX = kalmanX_.Acceleration();
    const double accY = kalmanY_.Acceleration();
    double accZ = kalmanZ_.Acceleration();
    double publishedPosZ = rawPosZ;
    double publishedVelZ = rawVelZ;
    double baroVerticalVelocityMps = NAN;
    double baroVerticalVelocitySigmaMps = NAN;
    double baroVerticalVelocityResidualMps = NAN;
    bool baroVerticalVelocityUpdateUsed = false;

    if (status_ == FlightStatus::Coast || status_ == FlightStatus::Overshoot) {
        const bool afterBaroVzGuardDelay =
            burnoutTimestamp_ > 0.0 &&
            (static_cast<double>(data.timestamp) - burnoutTimestamp_) >=
                settings::flight::kBaroVzGuardStartDelaySeconds;
        if (afterBaroVzGuardDelay &&
            hasFreshAltitudeMeasurement &&
            ComputeBaroVelocityEstimate(static_cast<double>(data.altimeterSigmaScale),
                                        &baroVerticalVelocityMps,
                                        &baroVerticalVelocitySigmaMps)) {
            baroVerticalVelocityResidualMps = rawVelZ - baroVerticalVelocityMps;
            const double residualThresholdMps = std::max(
                static_cast<double>(settings::flight::kBaroVzResidualGuardFloorMps),
                static_cast<double>(settings::flight::kBaroVzResidualGuardSigmaMultiplier) *
                    baroVerticalVelocitySigmaMps);
            if (std::fabs(baroVerticalVelocityResidualMps) > residualThresholdMps) {
                if (baroVzGuardPersistenceCount_ < 255) {
                    ++baroVzGuardPersistenceCount_;
                }
            } else if (baroVzGuardPersistenceCount_ > 0) {
                --baroVzGuardPersistenceCount_;
            }

            baroVerticalVelocityUpdateUsed =
                kalmanZ_.UpdateVelocityOnly(baroVerticalVelocityMps,
                                            baroVerticalVelocitySigmaMps,
                                            static_cast<double>(settings::flight::kBaroVzInnovationGateSigma));
            if (baroVerticalVelocityUpdateUsed) {
                rawPosZ = kalmanZ_.Position();
                rawVelZ = kalmanZ_.Velocity();
                accZ = kalmanZ_.Acceleration();
                publishedPosZ = rawPosZ;
                publishedVelZ = rawVelZ;
            }
        }
    } else {
        baroVzGuardPersistenceCount_ = 0;
    }

    if (status_ == FlightStatus::Ground) {
        if (hasFreshAltitudeMeasurement) {
            const double altitudeSampleTimestamp = static_cast<double>(data.timestamp);
            const double altitudeSampleDt =
                altitudeSampleTimestamp - lastGroundRelativeAltitudeTimestamp_;
            if (lastGroundRelativeAltitudeTimestamp_ > 0.0 && altitudeSampleDt > 0.0) {
                const double rawGroundVelocityMps =
                    (relativeAltitudeMeters - lastGroundRelativeAltitudeMeters_) / altitudeSampleDt;
                constexpr double kGroundVelocityBlend = 0.2;
                groundRelativeVelocityMps_ +=
                    kGroundVelocityBlend * (rawGroundVelocityMps - groundRelativeVelocityMps_);
            }
            lastGroundRelativeAltitudeMeters_ = relativeAltitudeMeters;
            lastGroundRelativeAltitudeTimestamp_ = altitudeSampleTimestamp;
        }

        const bool hasFreshLiftoffAcceleration = hasFreshAccelMeasurement;
        const double liftoffAccelerationMps2 =
            hasFreshLiftoffAcceleration ? static_cast<double>(inertialAcceleration.z) : accZ;
        const bool accelerationSuggestsLiftoff =
            hasFreshLiftoffAcceleration &&
            liftoffAccelerationMps2 > settings::flight::kLiftoffAccelerationThresholdMps2;
        const bool altitudeSuggestsLiftoff =
            hasFreshAltitudeMeasurement &&
            relativeAltitudeMeters > settings::flight::kLiftoffAltitudeThresholdM;
        const bool velocitySuggestsLiftoff =
            hasFreshAltitudeMeasurement &&
            groundRelativeVelocityMps_ > settings::flight::kLiftoffVelocityThresholdMps;
        const bool baroSuggestsLiftoff = altitudeSuggestsLiftoff && velocitySuggestsLiftoff;

        if (hasFreshLiftoffAcceleration) {
            if (accelerationSuggestsLiftoff) {
                if (liftoffCandidateCount_ < 255) {
                    ++liftoffCandidateCount_;
                }
            } else {
                liftoffCandidateCount_ = 0;
                burnDetectTimestamp_ = 0.0;
            }
        }
        if (baroSuggestsLiftoff) {
            if (baroLiftoffCandidateCount_ < 255) {
                ++baroLiftoffCandidateCount_;
            }
        } else if (hasFreshAltitudeMeasurement) {
            baroLiftoffCandidateCount_ = 0;
        }

        if (burnDetectTimestamp_ <= 0.0 &&
            liftoffCandidateCount_ >= settings::flight::kLiftoffConfirmSamples) {
            burnDetectTimestamp_ = static_cast<double>(data.timestamp);
        }
        const bool baroLiftoffConfirmed =
            baroLiftoffCandidateCount_ >= settings::flight::kLiftoffConfirmSamples;
        if (burnDetectTimestamp_ <= 0.0 && baroLiftoffConfirmed) {
            burnDetectTimestamp_ = static_cast<double>(data.timestamp);
        }

        if (burnDetectTimestamp_ > 0.0 &&
            ((velocitySuggestsLiftoff || altitudeSuggestsLiftoff) || baroLiftoffConfirmed)) {
            status_ = FlightStatus::Burn;
            burnTimestamp_ = burnDetectTimestamp_;
            liftoffCandidateCount_ = 0;
            baroLiftoffCandidateCount_ = 0;
            burnoutCandidateCount_ = 0;
            ReportEvent(false, static_cast<float>(burnTimestamp_), "Engine burn");
            groundRelativeVelocityMps_ = 0.0;
        } else {
            // Keep the vertical filter alive on the pad so covariance and bias
            // can settle before launch, while strongly constraining z and vz to
            // the grounded condition.
            kalmanZ_.ApplyGroundConstraints(
                settings::flight::kGroundConstraintAltitudeSigma,
                settings::flight::kGroundConstraintVelocitySigma);
            rawPosZ = kalmanZ_.Position();
            rawVelZ = kalmanZ_.Velocity();
            accZ = kalmanZ_.Acceleration();
            publishedPosZ = 0.0;
            publishedVelZ = 0.0;
            smoothedVelocity_[2] = 0.0;
        }
    }

    if (status_ == FlightStatus::Burn ||
        status_ == FlightStatus::Coast ||
        status_ == FlightStatus::Overshoot) {
        maxObservedAltitude_ = std::max(maxObservedAltitude_, rawPosZ);
    }

    if (status_ == FlightStatus::Burn) {
        const double timeSinceBurn = static_cast<double>(data.timestamp) - burnTimestamp_;
        const bool afterMinimumBurn = timeSinceBurn >= settings::flight::kBurnoutMinDurationSeconds;
        const bool accelerationSuggestsBurnout = accZ < settings::flight::kBurnoutAccelerationThresholdMps2;
        const bool stillAscending = rawVelZ > settings::flight::kBurnoutVelocityThresholdMps;

        if (afterMinimumBurn && accelerationSuggestsBurnout && stillAscending) {
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
            apogeeAltitude_ = maxObservedAltitude_;
            apogeeTimestamp_ = static_cast<double>(data.timestamp);
            apogeeRecorded_ = true;
            ReportEvent(true, data.timestamp, "Apogee reached");
        }
    }

    // Keep predicting through overshoot while the vehicle is still ascending.
    // Overshoot only means "above target apogee", not "at apogee".
    const bool shouldPredictApogee =
        (status_ == FlightStatus::Burn ||
         status_ == FlightStatus::Coast ||
         status_ == FlightStatus::Overshoot) &&
        rawVelZ > 0.0;
    double predictorHorizontalVelocity = 0.0;
    double predictorClampedZenith = 0.0;
    double predictorClampedAngularRate = 0.0;
    double predictorTimeToApogee = 0.0;
    uint32_t predictorFlags = 0;
    if (shouldPredictApogee) {
        predictorFlags |= kPredictorSeedFlagControlActive;
        predictorFlags |= kPredictorSeedFlagPositiveVerticalVelocity;
        // Deliberately degrade to a simpler predictor seed whenever attitude
        // freshness is questionable rather than integrating unstable XY terms.
        double seedZenith = quaternionValid_ ? SanitizePredictorZenithRadians(zenithRadians_) : 0.0;
        double previousSeedZenith = SanitizePredictorZenithRadians(lastZenith_);
        const double rawSeedZenith = quaternionValid_ ? zenithRadians_ : 0.0;
        if (quaternionValid_ && std::fabs(seedZenith - rawSeedZenith) > 1.0e-9) {
            predictorFlags |= kPredictorSeedFlagZenithClamped;
        }
        bool coastEntryBlendActive = false;
        if (burnoutTimestamp_ > 0.0 &&
            (status_ == FlightStatus::Coast || status_ == FlightStatus::Overshoot)) {
            const double timeSinceBurnout =
                std::max(0.0, static_cast<double>(data.timestamp) - burnoutTimestamp_);
            const double previousTimeSinceBurnout =
                std::max(0.0, timeSinceBurnout - std::max(dt, 0.0));
            const double blendRampSeconds =
                static_cast<double>(settings::flight::kPredictorCoastEntryZenithRampSeconds);
            coastEntryBlendActive = timeSinceBurnout < blendRampSeconds;
            seedZenith = ApplyPredictorCoastEntryZenithBlend(seedZenith, timeSinceBurnout);
            previousSeedZenith =
                ApplyPredictorCoastEntryZenithBlend(previousSeedZenith, previousTimeSinceBurnout);
        }
        if (coastEntryBlendActive) {
            predictorFlags |= kPredictorSeedFlagCoastEntryBlendActive;
        }
        const bool freshAccelSeedSample =
            PredictorSeedHasFreshAccelSample(dt, hasFreshAccelMeasurement);
        const bool canUseHorizontalSeed = quaternionValid_ && freshAccelSeedSample;
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
        predictorHorizontalVelocity =
            canUseHorizontalSeed
                ? ResolvePredictorHorizontalSpeed(trackedHorizontalVelocity, rawVelZ, seedZenith)
                : 0.0;
        if (canUseHorizontalSeed) {
            predictorFlags |= kPredictorSeedFlagUsingHorizontalModel;
        }
        if (baroVzGuardPersistenceCount_ >= settings::flight::kBaroVzGuardPersistenceSamples) {
            predictorFlags |= kPredictorSeedFlagBaroVzGuardActive;
        }
        if (baroVerticalVelocityUpdateUsed) {
            predictorFlags |= kPredictorSeedFlagBaroVzCorrectionUsed;
        }
        if (baroVzGuardActiveBeforeUpdate) {
            predictorFlags |= kPredictorSeedFlagVerticalAccelDeweighted;
        }
        const double rawPredictorAngularRate =
            ComputePredictorAngularRate(seedZenith, previousSeedZenith, dt);
        predictorClampedAngularRate =
            canUseHorizontalSeed
                ? ClampPredictorAngularRate(rawPredictorAngularRate)
                : 0.0;
        predictorClampedZenith = seedZenith;
        if (canUseHorizontalSeed &&
            std::fabs(predictorClampedAngularRate - rawPredictorAngularRate) > 1.0e-9) {
            predictorFlags |= kPredictorSeedFlagAngularRateClamped;
        }
        const double horizontalSpeedCap = PredictorHorizontalSpeedCap(rawVelZ, seedZenith);
        if (canUseHorizontalSeed &&
            predictorHorizontalVelocity >= (horizontalSpeedCap - 1.0e-6)) {
            predictorFlags |= kPredictorSeedFlagHorizontalSpeedCapped;
        }

        ApogeeState predictorState;
        predictorState.altitudeMeters = rawPosZ;
        predictorState.horizontalDistanceMeters = 0.0;
        predictorState.verticalVelocity = rawVelZ;
        predictorState.horizontalVelocity = predictorHorizontalVelocity;
        predictorState.zenith = predictorClampedZenith;
        predictorState.angularVelocity = predictorClampedAngularRate;
        predictorState.acsAngleDeg =
            std::clamp(static_cast<double>(data.flapEffectiveDeg),
                       0.0,
                       static_cast<double>(settings::actuation::kServoMaxActuationDeg));
        predictorState.acsCommandDeg =
            std::clamp(static_cast<double>(data.flapCommandDeg),
                       0.0,
                       static_cast<double>(settings::actuation::kServoMaxActuationDeg));
        // Single integration pass returns both altitude and time-to-apogee
        const PredictResult prediction = apogeePredictor_.PredictApogeeWithTime(predictorState);
        predictorFlags |= apogeePredictor_.LastPredictionFlags();
        predictorTimeToApogee = prediction.timeToApogee;
        double reportedApogeePrediction = prediction.altitude;
        const bool highAoAFallbackEligible =
            settings::predictor::kEnableHighAoAFallback &&
            (predictorFlags & kPredictorSeedFlagCfdAtkClamped) != 0u &&
            rawVelZ > 0.0;
        if (highAoAFallbackEligible) {
            const double ballisticBound =
                rawPosZ + (rawVelZ * rawVelZ) / (2.0 * constants::kGravity);
            if (std::isfinite(ballisticBound) &&
                std::isfinite(reportedApogeePrediction) &&
                ballisticBound > reportedApogeePrediction) {
                const double timeSinceBurnout =
                    (burnoutTimestamp_ > 0.0)
                        ? std::max(0.0, static_cast<double>(data.timestamp) - burnoutTimestamp_)
                        : 0.0;
                const double entryBlend =
                    static_cast<double>(settings::predictor::kHighAoAFallbackEntryBlend);
                const double peakBlend =
                    static_cast<double>(settings::predictor::kHighAoAFallbackPeakBlend);
                const double exitBlend =
                    static_cast<double>(settings::predictor::kHighAoAFallbackExitBlend);
                const double peakTime =
                    std::max(0.0, static_cast<double>(settings::predictor::kHighAoAFallbackPeakTimeSeconds));
                const double exitTime =
                    std::max(peakTime + 1.0e-3,
                             static_cast<double>(settings::predictor::kHighAoAFallbackExitTimeSeconds));
                double requestedBlend = entryBlend;
                if (timeSinceBurnout <= peakTime) {
                    const double fraction =
                        (peakTime > 1.0e-6) ? std::clamp(timeSinceBurnout / peakTime, 0.0, 1.0) : 1.0;
                    requestedBlend = entryBlend + fraction * (peakBlend - entryBlend);
                } else {
                    const double fraction =
                        std::clamp((timeSinceBurnout - peakTime) / (exitTime - peakTime), 0.0, 1.0);
                    requestedBlend = peakBlend + fraction * (exitBlend - peakBlend);
                }
                const double blend = std::clamp(
                    requestedBlend,
                    0.0,
                    1.0);
                reportedApogeePrediction += blend * (ballisticBound - reportedApogeePrediction);
                predictorFlags |= kPredictorSeedFlagPredictionUncertain;
            }
        }
        UpdateAdaptiveDragScale(predictorState,
                                predictorFlags,
                                accZ,
                                dt,
                                hasFreshAccelMeasurement,
                                prediction.timeToApogee,
                                static_cast<double>(data.flapCommandDeg),
                                static_cast<double>(data.flapEffectiveDeg),
                                data.actuationIsSettling > 0.5f);
        lastApogeePrediction_ = reportedApogeePrediction;
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
    output.predictorTimeToApogeeS = static_cast<float>(std::max(0.0, predictorTimeToApogee));
    output.predictorSeedHorizontalSpeedMps = static_cast<float>(predictorHorizontalVelocity);
    output.predictorSeedClampedZenithRad = static_cast<float>(predictorClampedZenith);
    output.predictorSeedClampedAngularRateRadPerSec = static_cast<float>(predictorClampedAngularRate);
    output.predictorSeedConfidenceFlags = static_cast<float>(predictorFlags);
    output.padReferenceDriftMps = static_cast<float>(groundReferenceDriftRateMps_);
    output.padReferenceSettled = groundReferenceSettled_ ? 1.0f : 0.0f;
    output.baroVerticalVelocityMps = static_cast<float>(baroVerticalVelocityMps);
    output.baroVerticalVelocitySigmaMps = static_cast<float>(baroVerticalVelocitySigmaMps);
    output.baroVerticalVelocityResidualMps = static_cast<float>(baroVerticalVelocityResidualMps);
    output.zAccelSigmaScale = static_cast<float>(zAccelSigmaScale);
    output.baroVerticalVelocityUpdateUsed = baroVerticalVelocityUpdateUsed ? 1.0f : 0.0f;
    output.baroVerticalVelocityGuardActive =
        (baroVzGuardPersistenceCount_ >= settings::flight::kBaroVzGuardPersistenceSamples) ? 1.0f : 0.0f;
    output.zAccelUpdateUsed = zAccelUpdateUsed ? 1.0f : 0.0f;
    output.bnoReferenceCorrectionApplied = bnoReferenceCorrectionApplied ? 1.0f : 0.0f;
    output.bnoReferenceTiltErrorDeg = bnoReferenceTiltErrorDeg;
    output.bnoQuaternionAgeMs = data.bnoQuaternionAgeMs;

    lastZenith_ = zenithRadians_;

    return true;
}

/// Updates the adaptive axial drag correction during coast.
/// Now supports both legacy single-scale and Mach-dependent adaptation.
void FlightComputer::UpdateAdaptiveDragScale(const ApogeeState &predictorState,
                                             uint32_t predictorFlags,
                                             double measuredVerticalAcceleration,
                                             double dtSeconds,
                                             bool hasFreshAccelMeasurement,
                                             double timeToApogeeSeconds,
                                             double flapCommandDeg,
                                             double flapEffectiveDeg,
                                             bool actuationIsSettling) {
    if (status_ != FlightStatus::Coast ||
        predictorState.verticalVelocity <= 0.0 ||
        !quaternionValid_ ||
        !PredictorSeedHasFreshAccelSample(dtSeconds, hasFreshAccelMeasurement) ||
        !std::isfinite(measuredVerticalAcceleration) ||
        !std::isfinite(predictorState.zenith) ||
        (predictorFlags & kPredictorSeedFlagCoastEntryBlendActive) != 0u ||
        PredictorFlagsHasModelInvalidity(predictorFlags)) {
        return;
    }

    if (burnoutTimestamp_ > 0.0) {
        const double timeSinceBurnout = std::max(0.0, lastTimestamp_ - burnoutTimestamp_);
        const double minAdaptDelay = std::max(
            static_cast<double>(settings::actuation::kPostBurnoutHoldoffSeconds),
            static_cast<double>(settings::flight::kPredictorCoastEntryZenithRampSeconds));
        if (timeSinceBurnout < minAdaptDelay) {
            return;
        }
    }

    const double flapTrackingErrorDeg = std::fabs(flapCommandDeg - flapEffectiveDeg);
    if (actuationIsSettling ||
        !std::isfinite(flapTrackingErrorDeg) ||
        flapTrackingErrorDeg > static_cast<double>(settings::actuation::kServoSettlingAngleEpsilonDeg)) {
        return;
    }

    double axialModelAcceleration = 0.0;
    const double predictedVerticalAcceleration =
        apogeePredictor_.ComputeVerticalAcceleration(predictorState, &axialModelAcceleration);
    const uint32_t modelFlags = apogeePredictor_.LastPredictionFlags();
    if (!std::isfinite(predictedVerticalAcceleration) || !std::isfinite(axialModelAcceleration) ||
        PredictorFlagsHasModelInvalidity(modelFlags)) {
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
    const double mach = apogeePredictor_.ComputeAirRelativeMach(predictorState);

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
        std::clamp(currentScale + currentScale * (residual / axialModelAcceleration), minScale, maxScale);

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

void FlightComputer::ResetBaroVelocityGuardState() {
    baroVelocityHistoryTimeSeconds_.fill(0.0);
    baroVelocityHistoryAltitudeMeters_.fill(0.0);
    baroVelocityHistoryCount_ = 0;
    baroVelocityHistoryNextIndex_ = 0;
    baroVzGuardPersistenceCount_ = 0;
}

void FlightComputer::RecordBaroAltitudeSample(double timeSeconds, double relativeAltitudeMeters) {
    if (!std::isfinite(timeSeconds) || !std::isfinite(relativeAltitudeMeters)) {
        return;
    }
    baroVelocityHistoryTimeSeconds_[baroVelocityHistoryNextIndex_] = timeSeconds;
    baroVelocityHistoryAltitudeMeters_[baroVelocityHistoryNextIndex_] = relativeAltitudeMeters;
    baroVelocityHistoryNextIndex_ =
        (baroVelocityHistoryNextIndex_ + 1u) % kBaroVelocityHistoryCapacity;
    if (baroVelocityHistoryCount_ < kBaroVelocityHistoryCapacity) {
        ++baroVelocityHistoryCount_;
    }
}

bool FlightComputer::ComputeBaroVelocityEstimate(double altitudeSigmaScale,
                                                 double *velocityMpsOut,
                                                 double *sigmaMpsOut) const {
    if (velocityMpsOut == nullptr || sigmaMpsOut == nullptr || baroVelocityHistoryCount_ == 0) {
        return false;
    }
    const std::size_t newestIndex =
        (baroVelocityHistoryNextIndex_ + kBaroVelocityHistoryCapacity - 1u) % kBaroVelocityHistoryCapacity;
    const double newestTime = baroVelocityHistoryTimeSeconds_[newestIndex];
    if (!std::isfinite(newestTime)) {
        return false;
    }

    constexpr std::size_t kMaxSamples = kBaroVelocityHistoryCapacity;
    std::array<double, kMaxSamples> times{};
    std::array<double, kMaxSamples> altitudes{};
    std::size_t sampleCount = 0;
    double oldestTime = newestTime;
    for (std::size_t offset = 0; offset < baroVelocityHistoryCount_; ++offset) {
        const std::size_t index =
            (baroVelocityHistoryNextIndex_ + kBaroVelocityHistoryCapacity - 1u - offset) %
            kBaroVelocityHistoryCapacity;
        const double sampleTime = baroVelocityHistoryTimeSeconds_[index];
        const double sampleAltitude = baroVelocityHistoryAltitudeMeters_[index];
        if (!std::isfinite(sampleTime) || !std::isfinite(sampleAltitude)) {
            continue;
        }
        const double ageSeconds = newestTime - sampleTime;
        if (ageSeconds < -1.0e-6) {
            continue;
        }
        if (ageSeconds > settings::flight::kBaroVzWindowSeconds) {
            break;
        }
        times[sampleCount] = sampleTime;
        altitudes[sampleCount] = sampleAltitude;
        oldestTime = sampleTime;
        ++sampleCount;
    }

    if (sampleCount < static_cast<std::size_t>(settings::flight::kBaroVzMinWindowSamples) ||
        (newestTime - oldestTime) < settings::flight::kBaroVzMinWindowSpanSeconds) {
        return false;
    }

    double meanTime = 0.0;
    double meanAltitude = 0.0;
    for (std::size_t i = 0; i < sampleCount; ++i) {
        meanTime += times[i];
        meanAltitude += altitudes[i];
    }
    meanTime /= static_cast<double>(sampleCount);
    meanAltitude /= static_cast<double>(sampleCount);

    double sumCenteredTimeSq = 0.0;
    double sumCenteredTimeAltitude = 0.0;
    for (std::size_t i = 0; i < sampleCount; ++i) {
        const double centeredTime = times[i] - meanTime;
        sumCenteredTimeSq += centeredTime * centeredTime;
        sumCenteredTimeAltitude += centeredTime * (altitudes[i] - meanAltitude);
    }
    if (!std::isfinite(sumCenteredTimeSq) || sumCenteredTimeSq <= 1.0e-6) {
        return false;
    }

    const double velocityMps = sumCenteredTimeAltitude / sumCenteredTimeSq;
    if (!std::isfinite(velocityMps)) {
        return false;
    }

    const double effectiveAltitudeSigma =
        altitudeSigma_ * std::max(1.0, altitudeSigmaScale);
    double sigmaMps = effectiveAltitudeSigma / std::sqrt(sumCenteredTimeSq);
    sigmaMps = std::clamp(sigmaMps,
                          static_cast<double>(settings::flight::kBaroVzSigmaFloorMps),
                          static_cast<double>(settings::flight::kBaroVzSigmaCeilMps));
    if (!std::isfinite(sigmaMps) || sigmaMps <= 0.0) {
        return false;
    }

    *velocityMpsOut = velocityMps;
    *sigmaMpsOut = sigmaMps;
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
    altitudeReferenceInitialized_ = false;
    altitudeReferenceMeters_ = 0.0;
    lastGroundRelativeAltitudeMeters_ = 0.0;
    lastGroundRelativeAltitudeTimestamp_ = 0.0;
    groundRelativeVelocityMps_ = 0.0;
    groundReferenceDriftRateMps_ = 0.0;
    groundReferenceStableSince_ = 0.0;
    groundReferenceSettled_ = false;
    quaternionValid_ = false;
    previousQuaternion_ = math_utils::MakeQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
    lastApogeePrediction_ = 0.0;
    apogeeAltitude_ = 0.0;
    maxObservedAltitude_ = 0.0;
    apogeeRecorded_ = false;
    burnTimestamp_ = 0.0;
    burnDetectTimestamp_ = 0.0;
    burnoutTimestamp_ = 0.0;
    apogeeTimestamp_ = 0.0;
    liftoffCandidateCount_ = 0;
    baroLiftoffCandidateCount_ = 0;
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
    bnoFreshPostBurnoutQuaternionCount_ = 0;
    lastBnoReferenceCorrectionSampleMicros_ = 0;
    ResetBaroVelocityGuardState();
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
