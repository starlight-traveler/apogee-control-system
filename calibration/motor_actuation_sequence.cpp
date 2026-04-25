#include <Arduino.h>

#include "settings.h"
#include "synced_flap_actuation.h"

#ifndef ACS_BUILD_MOTOR_ACTUATION_SEQUENCE
#error "Use the motor_actuation_sequence PlatformIO environment to build this target."
#endif

namespace {

constexpr uint32_t kConsoleBaud = 115200;
constexpr uint32_t kSerialTimeoutMs = 2000;
constexpr uint32_t kCommandServiceIntervalMs = 20;
constexpr uint32_t kMinimumStepHoldMs = 1500;
constexpr uint32_t kMaximumSettleWaitMs = 5000;
constexpr float kFullActuationDeg = settings::actuation::kServoMaxActuationDeg;

struct ActuationStep {
    const char *name;
    float angleDeg;
};

constexpr ActuationStep kSequence[] = {
    {"full", kFullActuationDeg},
    {"half", kFullActuationDeg * 0.5f},
    {"quarter", kFullActuationDeg * 0.25f},
    {"in", 0.0f},
    {"full", kFullActuationDeg},
    {"in", 0.0f},
};

SyncedFlapActuator g_flapActuator;
bool g_sequenceComplete = false;

void ServiceCommand(float angleDeg) {
    g_flapActuator.Update(millis(), angleDeg);
    delay(kCommandServiceIntervalMs);
}

void HoldCommand(const ActuationStep &step) {
    Serial.print("[motor_sequence] command ");
    Serial.print(step.name);
    Serial.print(" angle_deg=");
    Serial.println(step.angleDeg, 2);

    const uint32_t startMs = millis();
    do {
        ServiceCommand(step.angleDeg);
        const uint32_t elapsedMs = millis() - startMs;
        if (elapsedMs >= kMinimumStepHoldMs && !g_flapActuator.IsSettling()) {
            break;
        }
    } while ((millis() - startMs) < kMaximumSettleWaitMs);
}

}  // namespace

void setup() {
    Serial.begin(kConsoleBaud);
    const uint32_t serialStartMs = millis();
    while (!Serial && (millis() - serialStartMs) < kSerialTimeoutMs) {
        delay(10);
    }

    Serial.println("[motor_sequence] starting");
    g_flapActuator.Begin();

    for (const ActuationStep &step : kSequence) {
        HoldCommand(step);
    }

    Serial.println("[motor_sequence] complete; holding in");
    g_sequenceComplete = true;
}

void loop() {
    ServiceCommand(0.0f);
    if (g_sequenceComplete) {
        return;
    }
}
