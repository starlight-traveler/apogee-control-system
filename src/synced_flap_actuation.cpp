#include "synced_flap_actuation.h"

#include <Servo.h>

#include "settings.h"

namespace {

constexpr uint8_t kTopServoPin = settings::hardware::kTopServoPin;
constexpr uint8_t kBottomServoPin = settings::hardware::kBottomServoPin;
constexpr uint32_t kDeploymentDurationMs = settings::actuation::kDeploymentDurationMs;
constexpr int kTopServoInitialPwmUs = settings::actuation::kTopServoInitialPwmUs;
constexpr int kBottomServoInitialPwmUs = settings::actuation::kBottomServoInitialPwmUs;
constexpr int kTopServoExtendPwmUs = settings::actuation::kTopServoExtendPwmUs;
constexpr int kBottomServoExtendPwmUs = settings::actuation::kBottomServoExtendPwmUs;
constexpr int kTopServoRetractPwmUs = settings::actuation::kTopServoRetractPwmUs;
constexpr int kBottomServoRetractPwmUs = settings::actuation::kBottomServoRetractPwmUs;

Servo g_topServo;
Servo g_bottomServo;

}  // namespace

void SyncedFlapActuator::Begin() {
    g_topServo.attach(kTopServoPin);
    g_bottomServo.attach(kBottomServoPin);
    attached_ = true;
    triggered_ = false;
    extendUntilMs_ = 0;
    currentPosition_ = Position::Unknown;
    ApplyPosition(Position::Initial);
}

void SyncedFlapActuator::Update(uint32_t nowMs, bool autoDeployTrigger, bool manualForceExtend) {
    if (!attached_) {
        return;
    }

    if (autoDeployTrigger) {
        triggered_ = true;
        extendUntilMs_ = nowMs + kDeploymentDurationMs;
    }

    if (manualForceExtend) {
        ApplyPosition(Position::Extend);
        return;
    }

    if (triggered_ && !DeadlineReached(nowMs, extendUntilMs_)) {
        ApplyPosition(Position::Extend);
        return;
    }

    if (triggered_) {
        triggered_ = false;
        ApplyPosition(Position::Initial);
        return;
    }

    ApplyPosition(Position::Initial);
}

float SyncedFlapActuator::CommandFraction() const {
    return IsExtended() ? 1.0f : 0.0f;
}

float SyncedFlapActuator::EffectiveFraction() const {
    return IsExtended() ? 1.0f : 0.0f;
}

void SyncedFlapActuator::ApplyPosition(Position position) {
    if (!attached_ || position == currentPosition_) {
        return;
    }

    switch (position) {
        case Position::Unknown:
            return;
        case Position::Initial:
            g_topServo.writeMicroseconds(kTopServoInitialPwmUs);
            g_bottomServo.writeMicroseconds(kBottomServoInitialPwmUs);
            break;
        case Position::Extend:
            g_topServo.writeMicroseconds(kTopServoExtendPwmUs);
            g_bottomServo.writeMicroseconds(kBottomServoExtendPwmUs);
            break;
        case Position::Retract:
            g_topServo.writeMicroseconds(kTopServoRetractPwmUs);
            g_bottomServo.writeMicroseconds(kBottomServoRetractPwmUs);
            break;
    }

    currentPosition_ = position;
}

bool SyncedFlapActuator::DeadlineReached(uint32_t nowMs, uint32_t deadlineMs) {
    return static_cast<int32_t>(nowMs - deadlineMs) >= 0;
}
