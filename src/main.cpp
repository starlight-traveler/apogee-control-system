#include <Arduino.h>

#include "bno085_sensor.h"
#include "bmp581_sensor.h"
#include "serial_link.h"

constexpr uint32_t kSerialBaud = 921600;

namespace {

constexpr uint8_t kStatusLedPin = LED_BUILTIN;
constexpr uint32_t kErrorBlinkIntervalMs = 120;
constexpr uint32_t kRecoveryBlinkIntervalMs = 60;

enum class SystemError : uint8_t {
    BnoInitialization = 0,
    BmpInitialization = 1,
};

uint8_t BlinkCountForError(SystemError error) {
    switch (error) {
        case SystemError::BnoInitialization:
            return 2;
        case SystemError::BmpInitialization:
            return 3;
    }
    return 1;
}

void BlinkPattern(uint8_t count, uint32_t intervalMs) {
    for (uint8_t i = 0; i < count; ++i) {
        digitalWrite(kStatusLedPin, HIGH);
        delay(intervalMs);
        digitalWrite(kStatusLedPin, LOW);
        delay(intervalMs);
    }
}

void IndicateError(SystemError error) {
    BlinkPattern(BlinkCountForError(error), kErrorBlinkIntervalMs);
    delay(kErrorBlinkIntervalMs * 2);
}

void IndicateRecovery() {
    BlinkPattern(3, kRecoveryBlinkIntervalMs);
    delay(kRecoveryBlinkIntervalMs * 2);
}

bool InitializeWithRecovery(SystemError error, bool (*initializer)(), const char *failureMessage) {
    bool hadFailure = false;
    bool loggedFailure = false;
    uint32_t retryDelayMs = 200;
    while (!initializer()) {
        hadFailure = true;
        if (!loggedFailure && failureMessage != nullptr && Serial) {
            Serial.println(failureMessage);
            loggedFailure = true;
        }
        IndicateError(error);
        delay(retryDelayMs);
        if (retryDelayMs < 2000) {
            retryDelayMs += 200;
            if (retryDelayMs > 2000) {
                retryDelayMs = 2000;
            }
        }
    }

    if (hadFailure) {
        IndicateRecovery();
    }

    if (loggedFailure && failureMessage != nullptr && Serial) {
        Serial.println("Recovered successfully.");
    }

    return true;
}

}  // namespace

static bool AcquireSensorData(SensorData &data) {
    bool hasImu = Bno085SensorAcquire(data);
    bool hasAltimeter = Bmp581SensorAcquire(data);
    return hasImu || hasAltimeter;
}

void setup() {
    pinMode(kStatusLedPin, OUTPUT);
    digitalWrite(kStatusLedPin, LOW);

    SerialLinkBegin(kSerialBaud);
    while (!Serial && millis() < 2000) {
    }

    InitializeWithRecovery(SystemError::BnoInitialization,
                           &Bno085SensorBegin,
                           "Failed to initialize BNO085 sensor.");

    InitializeWithRecovery(SystemError::BmpInitialization,
                           &Bmp581SensorBegin,
                           "Failed to initialize BMP581 sensor.");

    if (Serial) {
        Serial.println("Teensy telemetry link initialized.");
    }
}

void loop() {
    SerialLinkProcessInput();

    SensorData data;
    if (!AcquireSensorData(data)) {
        return;
    }
    if (SerialLinkShouldSend(micros())) {
        SerialLinkSendTelemetry(data);
    }
}
