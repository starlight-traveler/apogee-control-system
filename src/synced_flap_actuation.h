#pragma once

#include <Arduino.h>

#include "settings.h"

class SyncedFlapActuator {
  public:
    void Begin();
    void Update(uint32_t nowMs, float commandedAngleDeg);

    float CommandAngleDeg() const { return commandAngleDeg_; }
    float EffectiveAngleDeg() const { return effectiveAngleDeg_; }
    bool IsSettling() const { return settling_; }
    bool ConsumeActuationEvent();
    bool ConsumeSettlingTimerFiredEvent();

  private:
    void ApplyPwm(uint32_t nowMs, int topPwmUs, int bottomPwmUs);
    static settings::actuation::ServoCalibrationPoint LookupNearestPoint(float angleDeg);
    static float ComputeSmoothingAlpha(float dtSeconds, float tauSeconds);

    bool attached_ = false;
    bool hasLastUpdateMs_ = false;
    bool settling_ = false;
    bool pendingActuationEvent_ = false;
    bool pendingSettlingTimerFiredEvent_ = false;
    uint32_t lastUpdateMs_ = 0;
    uint32_t lastPwmChangeMs_ = 0;
    uint32_t settlingDeadlineMs_ = 0;
    bool settlingTimerFired_ = false;
    float commandAngleDeg_ = 0.0f;
    float effectiveAngleDeg_ = 0.0f;
    int currentTopPwmUs_ = -1;
    int currentBottomPwmUs_ = -1;
};
