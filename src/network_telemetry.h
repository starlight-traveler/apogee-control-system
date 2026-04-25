#pragma once

#include "flight_computer.h"
#include "runtime_settings.h"
#include "telemetry_packet.h"

/**
 * @brief one telemetry packet's worth of pointers and scalar state.
 *
 * main.cpp fills this with the newest sensor/filter/servo data, and the
 * network layer packs it into the wire format. The network layer does not own
 * the pointed-to objects; it snapshots them immediately while building the UDP
 * packet.
 *
 * This struct intentionally carries both raw sensor pointers and derived
 * scalars such as AGL altitude and flap angles. That keeps telemetry encoding
 * dumb: main.cpp decides what "current flight state" means, and networking only
 * serializes that decision.
 */
struct TelemetrySnapshot {
    const SensorData *sensor = nullptr;
    const FilteredState *state = nullptr;
    FlightStatus status = FlightStatus::Ground;
    // Requested flap angle and the angle after actuator limits/rate behavior.
    float servoCommandDeg = 0.0f;
    float servoEffectiveDeg = 0.0f;
    // AGL is supplied by main.cpp because it owns pad-altitude initialization.
    float altitudeAglFeet = 0.0f;
    bool hasPadAltitude = false;
    // Indicates the ground station is temporarily commanding flap position.
    bool manualActuationOverride = false;
};

/// @brief initializes wifi/udp telemetry if networking is enabled.
void NetworkTelemetryBegin();
/// @brief polls inbound telemetry control/settings packets.
void NetworkTelemetryPollControl();
/// @brief publishes one telemetry snapshot when the send interval has elapsed.
void NetworkTelemetryService(const TelemetrySnapshot &snapshot);
/// @brief updates the settings snapshot included in telemetry responses.
void NetworkTelemetrySetRuntimeSettingsSnapshot(const RuntimeSettings &settings,
                                                const RuntimeSettingsStorageStatus &storageStatus,
                                                uint32_t settingsRevision,
                                                uint32_t appliedRequestId,
                                                uint8_t lastCommandResult);
/// @brief returns true when the wifi link is connected.
bool NetworkTelemetryConnected();
/// @brief returns true when a ground station has subscribed recently.
bool NetworkTelemetrySubscriberActive();
/// @brief returns true and fills `angleDegOut` when manual flap override is active.
bool NetworkTelemetryManualActuationOverride(float &angleDegOut);
/// @brief consumes one pending runtime-settings command from telemetry.
bool NetworkTelemetryConsumeSettingsCommand(telemetry::SettingsCommandV1 &commandOut);
