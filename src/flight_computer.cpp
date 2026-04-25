#include "flight_computer.h"

#include <cmath>

#include "constants.h"
#include "math_utils.h"
#include "serial_logging.h"
#include "settings.h"

namespace {

/*
 * FlightComputer owns the state estimate and phase machine.  It deliberately
 * separates three ideas that are easy to mix up:
 *
 *   - measurement freshness: did a sensor produce a real new sample this loop?
 *   - estimator state: what do the Kalman filters currently believe?
 *   - predictor seed: what subset of that state is safe to use for apogee?
 *
 * Fresh measurements can update filters. Cached measurements can preserve output
 * continuity, but should not create new phase transitions or control authority.
 * The predictor seed is even stricter: if attitude, acceleration, CFD coverage,
 * or baro-derived velocity look questionable, it sets flags so actuation can
 * retract instead of adding drag from a fragile estimate.
 */

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
    // Accelerometers measure specific force, so a stationary upright rocket
    // reads about +1 g. Subtract gravity after rotating into inertial axes to
    // get translational acceleration for the Kalman filter.
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

    // This is the gravity direction the quaternion implies in body axes. It is
    // used for tilt agreement, so only direction matters and the vector is normalized below.
    gravityBodyOut->x = -2.0f * (x * z - w * y);
    gravityBodyOut->y = -2.0f * (y * z + w * x);
    gravityBodyOut->z = -(1.0f - 2.0f * (x * x + y * y));
    if (gravityBodyOut->z < 0.0f) {
        // For tilt comparison we only care about the "up-ish" body direction.
        // Flip the vector if needed so equivalent inverted sign conventions do
        // not create a fake 180 degree disagreement.
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
    // The dot product of two unit gravity vectors gives cos(angle) between
    // their tilt estimates. Clamp protects acos from tiny floating-point drift.
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
    // This discrete alpha is stable for variable dt and behaves like a first
    // order low-pass without requiring exp() in presentation smoothing paths.
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
    // Each phase trusts sensors differently: boost has violent acceleration,
    // coast is where apogee prediction matters, and ground/descent should be quiet.
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
        // Changing vehicle/environment assumptions invalidates learned drag
        // unless the caller explicitly wants continuity.
        apogeePredictor_.ResetAxialDragScale();
    }
}

void FlightComputer::ResetGroundReference() {
    // Reset only pad-relative state. This is called when the operator wants a
    // new zero reference without rebuilding all configured predictor settings.
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
    /*
     * The update order is intentional:
     *
     *   1. establish timing and pad-relative altitude,
     *   2. choose a fresh accel/gyro rail that matches the selected attitude,
     *   3. propagate/correct attitude,
     *   4. predict/update Kalman filters,
     *   5. run phase detection,
     *   6. build a guarded predictor seed.
     *
     * Phase detection uses raw filter values. Published telemetry may be smoothed
     * later, but state-machine timing should not be delayed by display filtering.
     */
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
        // Bad timestamps happen in replay/boot edges. Use a bounded nominal dt
        // instead of letting filters integrate a negative or huge step.
        dt = static_cast<double>(settings::flight::kDefaultDtSeconds);
    }
    lastTimestamp_ = static_cast<double>(data.timestamp);
    const double altitudeMeters = static_cast<double>(data.altitudeFeet) * constants::kFeetToMeters;
    if (data.baroSampleFresh && std::isfinite(altitudeMeters)) {
        if (!altitudeReferenceInitialized_) {
            // First valid baro sample defines pad altitude for AGL calculations.
            altitudeReferenceMeters_ = altitudeMeters;
            altitudeReferenceInitialized_ = true;
            lastGroundRelativeAltitudeMeters_ = 0.0;
            groundRelativeVelocityMps_ = 0.0;
            groundReferenceDriftRateMps_ = 0.0;
            groundReferenceStableSince_ = static_cast<double>(data.timestamp);
            groundReferenceSettled_ = false;
        } else if (status_ == FlightStatus::Ground && burnDetectTimestamp_ <= 0.0) {
            // While still idle on the pad, slowly chase barometer drift so AGL
            // stays near zero. Stop doing this once launch is suspected.
            //
            // Conceptually, this is a moving "zero altitude" while the rocket is
            // still sitting still. The moment launch is suspected, the reference
            // freezes so upward motion becomes real AGL instead of being averaged
            // back into the pad altitude.
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
                // Pad reference is considered ready only after the drift has
                // stayed small for the configured hold time.
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
    // When baro-derived velocity keeps disagreeing with inertial velocity,
    // downweight vertical accel so the filter can be pulled back by pressure altitude.
    kalmanX_.SetMeasurementSigma(accelSigmaXY_ * phaseTuning.accelSigmaScale);
    kalmanY_.SetMeasurementSigma(accelSigmaXY_ * phaseTuning.accelSigmaScale);
    kalmanZ_.SetMeasurementSigmas(accelSigmaZ_ * zAccelSigmaScale,
                                  altitudeSigma_ * phaseTuning.altitudeSigmaScale);

    float accelBody[3];
    float gyroBody[3] = {0.0f, 0.0f, 0.0f};
    const MainQuaternionSource selectedQuaternionSource =
        static_cast<MainQuaternionSource>(data.mainQuaternionSource);
    /*
     * The selected quaternion source matters for acceleration too. If the main
     * attitude is ICM, the best acceleration measurement is fresh ICM accel; if
     * it is LSM, use fresh LSM accel. Mixing attitude from one rail with accel
     * from another rail can rotate thrust/gravity through slightly different
     * frame errors and create a vertical acceleration bias.
     */
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
        // Keep a small pressure-altitude history for the coast velocity guard.
        RecordBaroAltitudeSample(static_cast<double>(data.timestamp), relativeAltitudeMeters);
    }

    auto loadIcmGyro = [&]() {
        // Prefer gyro from the same rail as selected attitude, but allow a fresh
        // secondary gyro before falling all the way back to cached data.
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
        // Same policy as ICM, mirrored for LSM-selected attitude.
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
        // Pulse is slower/sidecar, so if Pulse gyro is not fresh, prefer fresh
        // onboard fast rails before cached Pulse data.
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
            // First valid main quaternion seeds the propagation state.
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
                    // Require several fresh BNO samples after burnout before it
                    // can trim the propagated attitude. That prevents one stale
                    // or late BNO packet from moving the coast predictor.
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
                                // BNO correction authority ramps in after
                                // burnout so the predictor does not see a step
                                // change in zenith exactly at coast entry.
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
                            // Slerp gives a small quaternion trim while
                            // preserving unit length and avoiding Euler angles.
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

    // Predict every filter axis first, then apply whichever fresh measurements
    // are available this loop. Missing samples should grow uncertainty, not reuse stale values.
    kalmanX_.Predict(dt, processNoiseXY_ * phaseTuning.processNoiseXYScale);
    kalmanY_.Predict(dt, processNoiseXY_ * phaseTuning.processNoiseXYScale);
    kalmanZ_.Predict(dt, processNoiseZ_ * phaseTuning.processNoiseZScale);

    bool zAccelUpdateUsed = false;
    if (hasFreshAccelMeasurement) {
        kalmanX_.Update(inertialAcceleration.x);
        kalmanY_.Update(inertialAcceleration.y);
    }
    if (hasFreshAccelMeasurement && hasFreshAltitudeMeasurement) {
        // When both channels are fresh, update accel and altitude separately so
        // either one can be gated without discarding the other.
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
            // Compare the inertial vertical velocity against a local slope of
            // pressure altitude. Persistent disagreement means the accel path is suspect.
            baroVerticalVelocityResidualMps = rawVelZ - baroVerticalVelocityMps;
            const double residualThresholdMps = std::max(
                static_cast<double>(settings::flight::kBaroVzResidualGuardFloorMps),
                static_cast<double>(settings::flight::kBaroVzResidualGuardSigmaMultiplier) *
                    baroVerticalVelocitySigmaMps);
            if (std::fabs(baroVerticalVelocityResidualMps) > residualThresholdMps) {
                // This is a consistency check, not a replacement estimator. If
                // inertial vertical velocity and a local baro slope disagree for
                // several samples, reduce trust in accel-derived vertical motion.
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
                // Pull the raw state after the pseudo-measurement so phase
                // checks and predictor seed see the corrected vertical velocity.
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
        /*
         * Liftoff detection has two paths:
         *
         *   - accel path: primary, fast, but can fail if the chosen rail saturates
         *     or goes stale,
         *   - baro path: backup, slower, and requires positive AGL plus positive
         *     baro-derived velocity over confirmation samples.
         *
         * The baro path exists to avoid staying in Ground after a real launch. It
         * is intentionally not a single-sample trigger.
         */
        if (hasFreshAltitudeMeasurement) {
            // Baro-only liftoff uses fresh pressure samples only. This avoids a
            // stale/default altitude pretending the rocket has left the pad.
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
            // Accel is the primary launch detector, but it still needs repeated
            // samples so one spike does not leave Ground.
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
            // Baro backup is intentionally strict: positive AGL, positive
            // pressure-derived velocity, and confirmation samples.
            if (baroLiftoffCandidateCount_ < 255) {
                ++baroLiftoffCandidateCount_;
            }
        } else if (hasFreshAltitudeMeasurement) {
            baroLiftoffCandidateCount_ = 0;
        }

        if (burnDetectTimestamp_ <= 0.0 &&
            liftoffCandidateCount_ >= settings::flight::kLiftoffConfirmSamples) {
            // Latch the first confirmed launch time. The actual state
            // transition below still asks for altitude/velocity evidence so a
            // pure accel spike cannot immediately enter Burn.
            burnDetectTimestamp_ = static_cast<double>(data.timestamp);
        }
        const bool baroLiftoffConfirmed =
            baroLiftoffCandidateCount_ >= settings::flight::kLiftoffConfirmSamples;
        if (burnDetectTimestamp_ <= 0.0 && baroLiftoffConfirmed) {
            burnDetectTimestamp_ = static_cast<double>(data.timestamp);
        }

        if (burnDetectTimestamp_ > 0.0 &&
            ((velocitySuggestsLiftoff || altitudeSuggestsLiftoff) || baroLiftoffConfirmed)) {
            // Enter Burn only after the launch timestamp is latched and there
            // is still independent altitude or velocity evidence of liftoff.
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
        /*
         * Burnout detection is the handoff from "motor is still adding energy" to
         * "the coast predictor physics are meaningful." The predictor does not
         * model motor thrust, so declaring Coast too early is more dangerous than
         * declaring it a little late.
         */
        const double timeSinceBurn = static_cast<double>(data.timestamp) - burnTimestamp_;
        const bool afterMinimumBurn = timeSinceBurn >= settings::flight::kBurnoutMinDurationSeconds;
        const bool accelerationSuggestsBurnout = accZ < settings::flight::kBurnoutAccelerationThresholdMps2;
        const bool stillAscending = rawVelZ > settings::flight::kBurnoutVelocityThresholdMps;

        if (afterMinimumBurn && accelerationSuggestsBurnout && stillAscending) {
            // Burnout requires time, low acceleration, and upward motion. This
            // avoids declaring coast from early thrust noise or pad handling.
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
            // Overshoot here means the measured state has already exceeded the
            // target altitude; it is not the predictor saying apogee is high.
            status_ = FlightStatus::Overshoot;
            ReportEvent(false, data.timestamp, "Overshoot");
        }
    }

    if (status_ == FlightStatus::Overshoot || status_ == FlightStatus::Coast) {
        if (accZ < settings::flight::kDescentAccelerationThresholdMps2 &&
            rawVelZ <= settings::flight::kDescentVelocityThresholdMps) {
            // Record the maximum altitude seen so a late descent transition
            // does not lose the actual peak.
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
        /*
         * The predictor seed is not a full navigation solution. It uses the
         * vertical Kalman state, a bounded attitude-derived horizontal speed, and
         * bounded angular rate. If those terms are not fresh and sane, the seed
         * falls back toward a simpler vertical-only rollout and raises flags.
         */
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
            // Right after burnout, attitude can still be settling from boost.
            // Blend the predictor seed toward vertical before trusting full tilt.
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
            // Without fresh attitude/accel, horizontal speed is more dangerous
            // than useful because it can turn stale tilt into a false drag path.
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
        // The tracker integrates lateral acceleration, then Resolve caps it
        // against a physically plausible value implied by vertical speed and
        // tilt. That keeps unobservable XY state from dominating apogee.
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
        // Angular rate is derived from the clamped zenith change. Clamp it too
        // so a single quaternion jump does not dominate the aero rollout.
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
        // Seed the apogee model with the live vertical state, optional
        // horizontal/attitude terms, and the physical flap position/command.
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
        // Single integration pass returns both altitude and time-to-apogee.
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
                // If AoA is outside the CFD table, the clamped table value can
                // be too drag-heavy. Blend toward a gravity-only upper bound
                // for telemetry, and flag the prediction as uncertain.
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
                    // Early high-AoA fallback is strongest while the table clamp
                    // is most likely to overstate drag from a bad AoA seed.
                    const double fraction =
                        (peakTime > 1.0e-6) ? std::clamp(timeSinceBurnout / peakTime, 0.0, 1.0) : 1.0;
                    requestedBlend = entryBlend + fraction * (peakBlend - entryBlend);
                } else {
                    // Fade fallback out later so the predictor returns toward
                    // the actual CFD table as attitude settles.
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
        // Only learn drag in clean coast. If the predictor seed is already
        // flagged uncertain, adapting on that residual would teach the model
        // from bad input.
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
        // Do not adapt aero while the flaps are moving; model residual then
        // mixes true aero error with actuator transient error.
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
        // If axial drag contribution is tiny, residual/model is too noisy to
        // produce a meaningful scale update.
        return;
    }

    const double residualClamp =
        static_cast<double>(settings::flight::kAdaptiveAxialDragResidualClampMps2);
    const double residual =
        std::clamp(measuredVerticalAcceleration - predictedVerticalAcceleration,
                   -residualClamp,
                   residualClamp);

    // The drag learner compares measured vertical acceleration with model
    // vertical acceleration, then nudges drag scale in the Mach bin being flown.
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
    // Clear pressure-slope history and guard persistence together so a reset
    // cannot keep an old "accel is suspect" state alive.
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
    // Ring buffer keeps the latest samples without moving arrays in the flight loop.
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
    // Walk backward through recent pressure samples and keep only a short,
    // contiguous window. The slope of this window is baro vertical velocity.
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
    // Centering the line fit improves numerical stability because flight times
    // are large absolute values while the window is only a few tenths of a second.
    for (std::size_t i = 0; i < sampleCount; ++i) {
        const double centeredTime = times[i] - meanTime;
        sumCenteredTimeSq += centeredTime * centeredTime;
        sumCenteredTimeAltitude += centeredTime * (altitudes[i] - meanAltitude);
    }
    if (!std::isfinite(sumCenteredTimeSq) || sumCenteredTimeSq <= 1.0e-6) {
        return false;
    }

    // Least-squares line slope: altitude change per second over the pressure window.
    const double velocityMps = sumCenteredTimeAltitude / sumCenteredTimeSq;
    if (!std::isfinite(velocityMps)) {
        return false;
    }

    // More spread in sample times reduces velocity uncertainty; noisier baro
    // samples increase it. Clamp the result so the guard cannot become overconfident.
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
