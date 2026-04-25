#pragma once

#include <Arduino.h>

#include "settings.h"

/**
 * @brief drives the paired flap servos as one synchronized actuator.
 *
 * the controller asks for an angle in degrees; this class turns that into top
 * and bottom servo pwm while tracking the smoothed effective angle for logging
 * and predictor feedback.
 *
 * `commandAngleDeg_` is the guidance request after clamp limits. `effectiveAngleDeg_`
 * is a simple model of what the hardware has probably reached after rate and
 * smoothing behavior. The predictor should use the effective angle when it
 * wants to model drag that is already physically present.
 */
class SyncedFlapActuator {
  public:
    /// @brief attaches the servo outputs and commands the closed position.
    void Begin();
    /// @brief advances command smoothing, pwm output, and settling timers.
    void Update(uint32_t nowMs, float commandedAngleDeg);

    /// @brief latest requested flap angle after clamping.
    float CommandAngleDeg() const { return commandAngleDeg_; }
    /// @brief smoothed angle assumed to be physically reached by the flaps.
    float EffectiveAngleDeg() const { return effectiveAngleDeg_; }
    /// @brief returns true while the actuator may still be moving.
    bool IsSettling() const { return settling_; }
    /// @brief consumes the one-shot "flap moved" event latch.
    bool ConsumeActuationEvent();
    /// @brief consumes the one-shot "settling timer fired" event latch.
    bool ConsumeSettlingTimerFiredEvent();

  private:
    // Pwm writes are gated so normal update loops do not spam identical servo commands.
    /// @brief writes pwm to both servos when the requested pulse changed.
    void ApplyPwm(uint32_t nowMs, int topPwmUs, int bottomPwmUs);
    /// @brief maps flap angle to servo pulse width.
    static int InterpolateServoPwmUs(float angleDeg, int closedPwmUs, int openPwmUs);
    /// @brief converts a time constant into a discrete smoothing alpha.
    static float ComputeSmoothingAlpha(float dtSeconds, float tauSeconds);

    // True after the Servo outputs have been attached.
    bool attached_ = false;
    // Guards the first update so dt starts from zero instead of an arbitrary timestamp.
    bool hasLastUpdateMs_ = false;
    // True while the modeled actuator has not settled at the requested angle.
    bool settling_ = false;
    // One-shot logging event raised whenever PWM output changes.
    bool pendingActuationEvent_ = false;
    // One-shot logging event raised when the settling timer first expires.
    bool pendingSettlingTimerFiredEvent_ = false;
    // Timestamp of the last Update call.
    uint32_t lastUpdateMs_ = 0;
    // Timestamp of the last physical PWM write.
    uint32_t lastPwmChangeMs_ = 0;
    // Settling timer is event-facing; it does not block the actuator from tracking new commands.
    uint32_t settlingDeadlineMs_ = 0;
    // Prevents repeatedly raising the same settling-timer event.
    bool settlingTimerFired_ = false;
    // Latest clamped controller request.
    float commandAngleDeg_ = 0.0f;
    // Modeled physical flap angle after latency smoothing.
    float effectiveAngleDeg_ = 0.0f;
    // Cached PWM values so unchanged commands are not rewritten.
    int currentTopPwmUs_ = -1;
    int currentBottomPwmUs_ = -1;
};
