#pragma once

#include <Arduino.h>

class SyncedFlapActuator {
  public:
    void Begin();
    void Update(uint32_t nowMs, bool autoDeployTrigger, bool manualForceExtend);

    bool HasTriggered() const { return triggered_; }
    bool IsExtended() const { return currentPosition_ == Position::Extend; }
    float CommandFraction() const;
    float EffectiveFraction() const;

  private:
    enum class Position : uint8_t { Unknown, Initial, Extend, Retract };

    void ApplyPosition(Position position);
    static bool DeadlineReached(uint32_t nowMs, uint32_t deadlineMs);

    bool attached_ = false;
    bool triggered_ = false;
    uint32_t extendUntilMs_ = 0;
    Position currentPosition_ = Position::Unknown;
};
