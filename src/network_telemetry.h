#pragma once

#include "flight_computer.h"
#include "runtime_settings.h"
#include "telemetry_packet.h"

struct TelemetrySnapshot {
    const SensorData *sensor = nullptr;
    const FilteredState *state = nullptr;
    FlightStatus status = FlightStatus::Ground;
    float servoCommandDeg = 0.0f;
    float servoEffectiveDeg = 0.0f;
    float altitudeAglFeet = 0.0f;
    bool hasPadAltitude = false;
    bool manualActuationOverride = false;
};

void NetworkTelemetryBegin();
void NetworkTelemetryPollControl();
void NetworkTelemetryService(const TelemetrySnapshot &snapshot);
void NetworkTelemetrySetRuntimeSettingsSnapshot(const RuntimeSettings &settings,
                                                const RuntimeSettingsStorageStatus &storageStatus,
                                                uint32_t settingsRevision,
                                                uint32_t appliedRequestId,
                                                uint8_t lastCommandResult);
bool NetworkTelemetryConnected();
bool NetworkTelemetrySubscriberActive();
bool NetworkTelemetryManualActuationOverride(float &angleDegOut);
bool NetworkTelemetryConsumeSettingsCommand(telemetry::SettingsCommandV1 &commandOut);
