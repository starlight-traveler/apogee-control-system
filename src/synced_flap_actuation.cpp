#include "synced_flap_actuation.h"

#include <Servo.h>
#include <algorithm>
#include <cmath>

#include "settings.h"

namespace {

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

Servo g_topServo;
Servo g_bottomServo;

}  // namespace

void SyncedFlapActuator::Begin() {
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

    ApplyPwm(0,
             InterpolateServoPwmUs(0.0f, kTopServoClosedPwmUs, kTopServoOpenPwmUs),
             InterpolateServoPwmUs(0.0f, kBottomServoClosedPwmUs, kBottomServoOpenPwmUs));
}

void SyncedFlapActuator::Update(uint32_t nowMs, float commandedAngleDeg) {
    if (!attached_) {
        return;
    }

    const float clampedCommand = std::clamp(commandedAngleDeg, 0.0f, kServoMaxActuationDeg);
    if (std::fabs(clampedCommand - commandAngleDeg_) > kCommandDeadbandDeg) {
        commandAngleDeg_ = clampedCommand;
    }

    float dtSeconds = 0.0f;
    if (hasLastUpdateMs_) {
        dtSeconds = static_cast<float>(nowMs - lastUpdateMs_) * 1.0e-3f;
        if (dtSeconds < 0.0f || dtSeconds > 1.0f) {
            dtSeconds = 0.0f;
        }
    }
    lastUpdateMs_ = nowMs;
    hasLastUpdateMs_ = true;

    const float alpha = ComputeSmoothingAlpha(dtSeconds, kServoLatencySeconds);
    effectiveAngleDeg_ += alpha * (commandAngleDeg_ - effectiveAngleDeg_);
    effectiveAngleDeg_ = std::clamp(effectiveAngleDeg_, 0.0f, kServoMaxActuationDeg);

    const int topPwmUs = InterpolateServoPwmUs(effectiveAngleDeg_, kTopServoClosedPwmUs, kTopServoOpenPwmUs);
    const int bottomPwmUs =
        InterpolateServoPwmUs(effectiveAngleDeg_, kBottomServoClosedPwmUs, kBottomServoOpenPwmUs);
    const bool pwmChanged = (topPwmUs != currentTopPwmUs_) || (bottomPwmUs != currentBottomPwmUs_);
    const bool dwellElapsed = (nowMs - lastPwmChangeMs_) >= kServoMinStepIntervalMs;
    if (pwmChanged && (lastPwmChangeMs_ == 0 || dwellElapsed)) {
        ApplyPwm(nowMs, topPwmUs, bottomPwmUs);
    }

    const bool timerExpired = (settlingDeadlineMs_ != 0u) && (static_cast<int32_t>(nowMs - settlingDeadlineMs_) >= 0);
    if (timerExpired && !settlingTimerFired_) {
        pendingSettlingTimerFiredEvent_ = true;
        settlingTimerFired_ = true;
    }

    const bool withinAngleTolerance = std::fabs(commandAngleDeg_ - effectiveAngleDeg_) <= kServoSettlingAngleEpsilonDeg;
    settling_ = !withinAngleTolerance || !timerExpired;
}

void SyncedFlapActuator::ApplyPwm(uint32_t nowMs, int topPwmUs, int bottomPwmUs) {
    if (!attached_) {
        return;
    }
    if (topPwmUs != currentTopPwmUs_) {
        g_topServo.writeMicroseconds(topPwmUs);
        currentTopPwmUs_ = topPwmUs;
    }
    if (bottomPwmUs != currentBottomPwmUs_) {
        g_bottomServo.writeMicroseconds(bottomPwmUs);
        currentBottomPwmUs_ = bottomPwmUs;
    }
    pendingActuationEvent_ = true;
    pendingSettlingTimerFiredEvent_ = false;
    settlingTimerFired_ = false;
    lastPwmChangeMs_ = nowMs;
    settlingDeadlineMs_ = nowMs + kServoSettlingDurationMs;
}

bool SyncedFlapActuator::ConsumeActuationEvent() {
    if (!pendingActuationEvent_) {
        return false;
    }
    pendingActuationEvent_ = false;
    return true;
}

bool SyncedFlapActuator::ConsumeSettlingTimerFiredEvent() {
    if (!pendingSettlingTimerFiredEvent_) {
        return false;
    }
    pendingSettlingTimerFiredEvent_ = false;
    return true;
}

int SyncedFlapActuator::InterpolateServoPwmUs(float angleDeg, int closedPwmUs, int openPwmUs) {
    const float clampedAngle = std::clamp(angleDeg, 0.0f, kServoMaxActuationDeg);
    if (kServoMaxActuationDeg <= 0.0f) {
        return closedPwmUs;
    }
    const float blend = clampedAngle / kServoMaxActuationDeg;
    const float pwmUs = static_cast<float>(closedPwmUs) + blend * static_cast<float>(openPwmUs - closedPwmUs);
    return static_cast<int>(lroundf(pwmUs));
}

float SyncedFlapActuator::ComputeSmoothingAlpha(float dtSeconds, float tauSeconds) {
    if (dtSeconds <= 0.0f || tauSeconds <= 0.0f) {
        return 1.0f;
    }
    const float alpha = dtSeconds / (tauSeconds + dtSeconds);
    return std::clamp(alpha, 0.0f, 1.0f);
}
