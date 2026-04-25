#include "synced_flap_actuation.h"

#include <Servo.h>
#include <algorithm>
#include <cmath>

#include "settings.h"

namespace {

/*
 * This class separates commanded angle from effective angle.
 *
 * The command is what the controller wants. The effective angle is a simple
 * first-order model of where the flaps probably are after servo latency. The
 * predictor uses effective angle for current drag and command angle for where
 * the actuator is moving next.
 */

constexpr uint8_t kTopServoPin = settings::hardware::kTopServoPin;
constexpr uint8_t kBottomServoPin = settings::hardware::kBottomServoPin;
constexpr float kServoMaxActuationDeg = settings::actuation::kServoMaxActuationDeg;
constexpr int kServoAttachMinPulseUs = settings::actuation::kServoAttachMinPulseUs;
constexpr int kServoAttachMaxPulseUs = settings::actuation::kServoAttachMaxPulseUs;
constexpr int kTopServoClosedPwmUs = settings::actuation::kTopServoClosedPwmUs;
constexpr int kTopServoOpenPwmUs = settings::actuation::kTopServoOpenPwmUs;
constexpr int kBottomServoClosedPwmUs = settings::actuation::kBottomServoClosedPwmUs;
constexpr int kBottomServoOpenPwmUs = settings::actuation::kBottomServoOpenPwmUs;
constexpr float kServoLatencySeconds = settings::actuation::kServoLatencySeconds;
constexpr float kCommandDeadbandDeg = settings::actuation::kAngleCommandDeadbandDeg;
constexpr uint32_t kServoMinStepIntervalMs = settings::actuation::kServoMinStepIntervalMs;
constexpr uint32_t kServoSettlingDurationMs = settings::actuation::kServoSettlingDurationMs;
constexpr float kServoSettlingAngleEpsilonDeg = settings::actuation::kServoSettlingAngleEpsilonDeg;

// Servo objects are file-static because the actuator wrapper is the single
// owner of the physical PWM outputs.
Servo g_topServo;
Servo g_bottomServo;

}  // namespace

void SyncedFlapActuator::Begin() {
    // Attach both servos with explicit pulse limits so accidental commands
    // outside the calibrated travel range are clipped by the Servo library too.
    g_topServo.attach(kTopServoPin, kServoAttachMinPulseUs, kServoAttachMaxPulseUs);
    g_bottomServo.attach(kBottomServoPin, kServoAttachMinPulseUs, kServoAttachMaxPulseUs);
    attached_ = true;
    hasLastUpdateMs_ = false;
    settling_ = false;
    pendingActuationEvent_ = false;
    pendingSettlingTimerFiredEvent_ = false;
    lastUpdateMs_ = 0;
    lastPwmChangeMs_ = 0;
    settlingDeadlineMs_ = 0;
    settlingTimerFired_ = false;
    commandAngleDeg_ = 0.0f;
    effectiveAngleDeg_ = 0.0f;
    currentTopPwmUs_ = -1;
    currentBottomPwmUs_ = -1;

    // Start physically closed so the actuator state and servo outputs agree
    // before any automatic command is allowed.
    ApplyPwm(0,
             InterpolateServoPwmUs(0.0f, kTopServoClosedPwmUs, kTopServoOpenPwmUs),
             InterpolateServoPwmUs(0.0f, kBottomServoClosedPwmUs, kBottomServoOpenPwmUs));
}

void SyncedFlapActuator::Update(uint32_t nowMs, float commandedAngleDeg) {
    if (!attached_) {
        return;
    }

    // Every external command enters through the same clamp. Fault handling,
    // manual override, and automatic apogee control therefore all share the same
    // mechanical limit.
    const float clampedCommand = std::clamp(commandedAngleDeg, 0.0f, kServoMaxActuationDeg);
    if (std::fabs(clampedCommand - commandAngleDeg_) > kCommandDeadbandDeg) {
        // Ignore tiny command changes so PWM does not chatter around one angle.
        commandAngleDeg_ = clampedCommand;
    }

    float dtSeconds = 0.0f;
    if (hasLastUpdateMs_) {
        dtSeconds = static_cast<float>(nowMs - lastUpdateMs_) * 1.0e-3f;
        if (dtSeconds < 0.0f || dtSeconds > 1.0f) {
            // Ignore impossible/long gaps instead of letting one bad timestamp
            // jump the modeled flap angle.
            dtSeconds = 0.0f;
        }
    }
    lastUpdateMs_ = nowMs;
    hasLastUpdateMs_ = true;

    const float alpha = ComputeSmoothingAlpha(dtSeconds, kServoLatencySeconds);
    // effectiveAngleDeg_ is the modeled physical flap angle. It approaches the
    // command over the configured servo latency instead of jumping instantly.
    effectiveAngleDeg_ += alpha * (commandAngleDeg_ - effectiveAngleDeg_);
    effectiveAngleDeg_ = std::clamp(effectiveAngleDeg_, 0.0f, kServoMaxActuationDeg);

    const int topPwmUs = InterpolateServoPwmUs(effectiveAngleDeg_, kTopServoClosedPwmUs, kTopServoOpenPwmUs);
    const int bottomPwmUs =
        InterpolateServoPwmUs(effectiveAngleDeg_, kBottomServoClosedPwmUs, kBottomServoOpenPwmUs);
    // Top and bottom servos may be mounted in opposite directions, so each gets
    // its own calibrated PWM endpoint pair even though the flap angle is shared.
    const bool pwmChanged = (topPwmUs != currentTopPwmUs_) || (bottomPwmUs != currentBottomPwmUs_);
    const bool dwellElapsed = (nowMs - lastPwmChangeMs_) >= kServoMinStepIntervalMs;
    if (pwmChanged && (lastPwmChangeMs_ == 0 || dwellElapsed)) {
        // Enforce a minimum interval between PWM writes so the servo command
        // stream is rate-limited even if the control loop runs faster.
        ApplyPwm(nowMs, topPwmUs, bottomPwmUs);
    }

    const bool timerExpired = (settlingDeadlineMs_ != 0u) && (static_cast<int32_t>(nowMs - settlingDeadlineMs_) >= 0);
    if (timerExpired && !settlingTimerFired_) {
        pendingSettlingTimerFiredEvent_ = true;
        settlingTimerFired_ = true;
    }

    const bool withinAngleTolerance = std::fabs(commandAngleDeg_ - effectiveAngleDeg_) <= kServoSettlingAngleEpsilonDeg;
    // Settling stays true until both the timer has expired and the modeled flap
    // angle is close to the requested angle.
    settling_ = !withinAngleTolerance || !timerExpired;
}

void SyncedFlapActuator::ApplyPwm(uint32_t nowMs, int topPwmUs, int bottomPwmUs) {
    if (!attached_) {
        return;
    }
    if (topPwmUs != currentTopPwmUs_) {
        // Only write changed channels. Some servo libraries disable interrupts
        // briefly during writes, so unnecessary writes are avoided.
        g_topServo.writeMicroseconds(topPwmUs);
        currentTopPwmUs_ = topPwmUs;
    }
    if (bottomPwmUs != currentBottomPwmUs_) {
        g_bottomServo.writeMicroseconds(bottomPwmUs);
        currentBottomPwmUs_ = bottomPwmUs;
    }
    pendingActuationEvent_ = true;
    // A new PWM step invalidates the previous settling-timer-fired event.
    pendingSettlingTimerFiredEvent_ = false;
    settlingTimerFired_ = false;
    lastPwmChangeMs_ = nowMs;
    // Any physical PWM change restarts the settling timer for telemetry and
    // adaptive-drag gating.
    settlingDeadlineMs_ = nowMs + kServoSettlingDurationMs;
}

bool SyncedFlapActuator::ConsumeActuationEvent() {
    // Events are latches because the logger/main loop may run at a different
    // cadence than the actuator update.
    if (!pendingActuationEvent_) {
        return false;
    }
    pendingActuationEvent_ = false;
    return true;
}

bool SyncedFlapActuator::ConsumeSettlingTimerFiredEvent() {
    // Consuming the event clears only the notification, not the physical
    // settling state reported by IsSettling().
    if (!pendingSettlingTimerFiredEvent_) {
        return false;
    }
    pendingSettlingTimerFiredEvent_ = false;
    return true;
}

int SyncedFlapActuator::InterpolateServoPwmUs(float angleDeg, int closedPwmUs, int openPwmUs) {
    const float clampedAngle = std::clamp(angleDeg, 0.0f, kServoMaxActuationDeg);
    if (kServoMaxActuationDeg <= 0.0f) {
        // Defensive guard for bad configuration; closed is the least-drag state.
        return closedPwmUs;
    }
    const float blend = clampedAngle / kServoMaxActuationDeg;
    // Linear calibration from flap angle to servo pulse width. Top and bottom
    // servos can have different endpoints but share the same commanded angle.
    const float pwmUs = static_cast<float>(closedPwmUs) + blend * static_cast<float>(openPwmUs - closedPwmUs);
    return static_cast<int>(lroundf(pwmUs));
}

float SyncedFlapActuator::ComputeSmoothingAlpha(float dtSeconds, float tauSeconds) {
    if (dtSeconds <= 0.0f || tauSeconds <= 0.0f) {
        // With no usable time constant, snap the modeled angle to the command.
        return 1.0f;
    }
    // First-order low-pass alpha: small dt or large tau means slow movement.
    const float alpha = dtSeconds / (tauSeconds + dtSeconds);
    return std::clamp(alpha, 0.0f, 1.0f);
}
