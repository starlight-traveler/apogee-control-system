#include <Arduino.h>
#include <string.h>

#include "serial_link.h"
#include "serial_protocol.h"

namespace {

// Update to match your hardware PWM output pins.
constexpr uint8_t kPwmPins[] = {2, 3, 4, 5};
constexpr uint16_t kMaxDuty = (1u << 12) - 1;
constexpr uint8_t kFlagHasFrequency = 1;

bool g_initialized = false;

void EnsureInitialized() {
    if (g_initialized) {
        return;
    }
    analogWriteResolution(12);
    for (uint8_t pin : kPwmPins) {
        pinMode(pin, OUTPUT);
        analogWrite(pin, 0);
    }
    g_initialized = true;
}

}  // namespace

bool SerialLinkHandleCustomCommand(uint8_t commandId, const uint8_t *payload, uint16_t length) {
    if (commandId != static_cast<uint8_t>(serial_protocol::CommandId::SetPwm)) {
        return false;
    }
    if (length != sizeof(serial_protocol::PwmCommandPayload)) {
        return false;
    }

    serial_protocol::PwmCommandPayload cmd{};
    memcpy(&cmd, payload, sizeof(cmd));
    if (cmd.channel >= sizeof(kPwmPins)) {
        return false;
    }

    EnsureInitialized();

    const uint8_t pin = kPwmPins[cmd.channel];
    if ((cmd.flags & kFlagHasFrequency) != 0 && cmd.frequencyHz > 0) {
        analogWriteFrequency(pin, cmd.frequencyHz);
    }

    uint16_t duty = cmd.duty;
    if (duty > kMaxDuty) {
        duty = kMaxDuty;
    }
    analogWrite(pin, duty);

    return true;
}
