#include <Arduino.h>

#include "bno085_sensor.h"
#include "bmp581_sensor.h"
#include "data_logger.h"
#include "flight_computer.h"

constexpr bool kEnableSerialTelemetry = true;

namespace {

constexpr uint8_t kStatusLedPin = LED_BUILTIN;
constexpr uint32_t kErrorBlinkIntervalMs = 120;
constexpr uint32_t kRecoveryBlinkIntervalMs = 60;

enum class SystemError : uint8_t {
    BnoInitialization = 0,
    BmpInitialization = 1,
    DataLoggerInitialization = 2,
};

uint8_t BlinkCountForError(SystemError error) {
    switch (error) {
        case SystemError::BnoInitialization:
            return 2;
        case SystemError::BmpInitialization:
            return 3;
        case SystemError::DataLoggerInitialization:
            return 4;
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
        if (!loggedFailure && failureMessage != nullptr && kEnableSerialTelemetry && Serial) {
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

    if (loggedFailure && failureMessage != nullptr && kEnableSerialTelemetry && Serial) {
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

static FlightComputer flightComputer;
static FlightStatus g_lastLoggedStatus = FlightStatus::Ground;
static bool g_hasLoggedStatus = false;

void setup() {
    pinMode(kStatusLedPin, OUTPUT);
    digitalWrite(kStatusLedPin, LOW);

    Serial.begin(115200);
    if (kEnableSerialTelemetry) {
        while (!Serial && millis() < 2000) {
        }
    }

    DataLoggerSetSerialLoggingEnabled(kEnableSerialTelemetry);

    InitializeWithRecovery(SystemError::BnoInitialization,
                           &Bno085SensorBegin,
                           "Failed to initialize BNO085 sensor.");

    InitializeWithRecovery(SystemError::BmpInitialization,
                           &Bmp581SensorBegin,
                           "Failed to initialize BMP581 sensor.");

    InitializeWithRecovery(SystemError::DataLoggerInitialization,
                           &DataLoggerBegin,
                           "Sensor logging is disabled.");

    const EnvironmentModel::Config environmentConfig;
    ApogeeVehicleParameters vehicleParameters;

    const float sigmaAccelXY = 0.5f;
    const float sigmaAccelZ = 0.5f;
    const float sigmaAltimeter = 0.5f;
    const float processXY = 0.5f;
    const float processZ = 1.0f;
    const float apogeeTargetMeters = 1550.0f;

    flightComputer.Begin(sigmaAccelXY,
                         sigmaAccelZ,
                         sigmaAltimeter,
                         processXY,
                         processZ,
                         apogeeTargetMeters,
                         environmentConfig,
                         vehicleParameters);

    flightComputer.SetSerialReportingEnabled(kEnableSerialTelemetry);

    if (kEnableSerialTelemetry && Serial) {
        Serial.println("Flight computer initialized.");
    }

    g_lastLoggedStatus = flightComputer.Status();
    g_hasLoggedStatus = false;
}

void loop() {
    DataLoggerService();

    if (!DataLoggerIsInitialized()) {
        InitializeWithRecovery(SystemError::DataLoggerInitialization,
                               &DataLoggerBegin,
                               "Data logger unavailable. Retrying...");
        return;
    }

    SensorData data;
    if (!AcquireSensorData(data)) {
        delay(1);
        return;
    }

    FilteredState state;
    const bool hasFilteredState = flightComputer.Update(data, state);

    DataLoggerLogTelemetry(data, flightComputer.Status(), hasFilteredState ? &state : nullptr);

    if (hasFilteredState) {
        const FlightStatus status = flightComputer.Status();
        if (!g_hasLoggedStatus || status != g_lastLoggedStatus) {
            DataLoggerLogEvent(FlightEventType::StageChange,
                               status,
                               state.time,
                               state.position[2],
                               state.velocity[2],
                               state.apogeeEstimate);
            g_lastLoggedStatus = status;
            g_hasLoggedStatus = true;
        }
    }

    if (hasFilteredState && kEnableSerialTelemetry && Serial) {
        Serial.print("t=");
        Serial.print(state.time, 3);
        Serial.print(" z=");
        Serial.print(state.position[2], 2);
        Serial.print(" vz=");
        Serial.print(state.velocity[2], 2);
        Serial.print(" az=");
        Serial.print(state.acceleration[2], 2);
        Serial.print(" apg=");
        Serial.print(state.apogeeEstimate, 2);
        Serial.print(" status=");
        Serial.println(FlightStatusToString(flightComputer.Status()));
    }

    DataLoggerService();
}
