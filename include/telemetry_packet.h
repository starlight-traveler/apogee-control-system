#pragma once

#include <stdint.h>

namespace telemetry {

constexpr uint32_t kPacketMagic = 0x31504C54u;  // "TLP1"
constexpr uint16_t kPacketVersion = 1;
constexpr uint32_t kHeartbeatMagic = 0x31424854u;  // "THB1"
constexpr uint16_t kHeartbeatVersion = 1;
constexpr uint32_t kActuationCommandMagic = 0x31434154u;  // "TAC1"
constexpr uint16_t kActuationCommandVersion = 1;
constexpr uint32_t kTelemetryControlMagic = 0x31544354u;  // "TCT1"
constexpr uint16_t kTelemetryControlVersion = 1;
constexpr uint32_t kSettingsCommandMagic = 0x31534354u;  // "TCS1"
constexpr uint16_t kSettingsCommandVersion = 1;
constexpr uint32_t kSettingsSnapshotMagic = 0x31535354u;  // "TSS1"
constexpr uint16_t kSettingsSnapshotVersion = 1;

constexpr uint8_t kFlagHasFilteredState = 1u << 0;
constexpr uint8_t kFlagHasPadAltitude = 1u << 1;
constexpr uint8_t kFlagManualActuationOverride = 1u << 2;

constexpr uint8_t kActuationModeAuto = 0u;
constexpr uint8_t kActuationModeManual = 1u;

constexpr uint8_t kSettingsOpRequestCurrent = 0u;
constexpr uint8_t kSettingsOpApplyAndPersist = 1u;
constexpr uint8_t kSettingsOpRestoreDefaults = 2u;

constexpr uint8_t kSettingsResultNone = 0u;
constexpr uint8_t kSettingsResultApplied = 1u;
constexpr uint8_t kSettingsResultRejected = 2u;
constexpr uint8_t kSettingsResultPersistFailed = 3u;
constexpr uint8_t kSettingsResultStorageUnavailable = 4u;

constexpr uint8_t kSettingsStatusStorageAvailable = 1u << 0;
constexpr uint8_t kSettingsStatusFilePresent = 1u << 1;
constexpr uint8_t kSettingsStatusUsingDefaults = 1u << 2;
constexpr uint8_t kSettingsStatusLastLoadSucceeded = 1u << 3;
constexpr uint8_t kSettingsStatusLastSaveSucceeded = 1u << 4;
constexpr uint8_t kSettingsStatusCreatedDefaultFile = 1u << 5;

#pragma pack(push, 1)
struct PacketV1 {
    uint32_t magic = kPacketMagic;
    uint16_t version = kPacketVersion;
    uint16_t size = sizeof(PacketV1);
    uint32_t sequence = 0;
    uint32_t uptimeMs = 0;
    uint8_t flightStatus = 0;
    uint8_t flags = 0;
    uint16_t reserved = 0;

    float sensorTimestamp = 0.0f;
    float sensorAltitudeFeet = 0.0f;
    float sensorAccelBno[3] = {0.0f, 0.0f, 0.0f};
    float sensorAccelIcm[3] = {0.0f, 0.0f, 0.0f};
    float sensorGyro[3] = {0.0f, 0.0f, 0.0f};
    float sensorQuaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float sensorIcmQuaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float sensorIcmYprDeg[3] = {0.0f, 0.0f, 0.0f};
    uint8_t sensorHasQuaternion = 0;
    uint8_t sensorHasIcmQuaternion = 0;
    uint8_t sensorHasIcmYpr = 0;
    uint8_t sensorReserved = 0;

    float stateTime = 0.0f;
    float statePosition[3] = {0.0f, 0.0f, 0.0f};
    float stateVelocity[3] = {0.0f, 0.0f, 0.0f};
    float stateAcceleration[3] = {0.0f, 0.0f, 0.0f};
    float stateInertialAcceleration[3] = {0.0f, 0.0f, 0.0f};
    float stateZenith = 0.0f;
    float stateApogeeEstimate = 0.0f;

    float servoCommandDeg = 0.0f;
    float servoEffectiveDeg = 0.0f;
    float altitudeAglFeet = 0.0f;
};
#pragma pack(pop)

static_assert(sizeof(PacketV1) == 184, "PacketV1 size changed; update desktop parser and firmware together.");

#pragma pack(push, 1)
struct HeartbeatV1 {
    uint32_t magic = kHeartbeatMagic;
    uint16_t version = kHeartbeatVersion;
    uint16_t size = sizeof(HeartbeatV1);
};
#pragma pack(pop)

static_assert(sizeof(HeartbeatV1) == 8, "HeartbeatV1 size changed; update sender/receiver together.");

#pragma pack(push, 1)
struct ActuationCommandV1 {
    uint32_t magic = kActuationCommandMagic;
    uint16_t version = kActuationCommandVersion;
    uint16_t size = sizeof(ActuationCommandV1);
    uint8_t mode = kActuationModeAuto;
    uint8_t reserved[3] = {0, 0, 0};
    float angleDeg = 0.0f;
};
#pragma pack(pop)

static_assert(sizeof(ActuationCommandV1) == 16,
              "ActuationCommandV1 size changed; update sender/receiver together.");

#pragma pack(push, 1)
struct TelemetryControlV1 {
    uint32_t magic = kTelemetryControlMagic;
    uint16_t version = kTelemetryControlVersion;
    uint16_t size = sizeof(TelemetryControlV1);
    uint8_t telemetryEnabled = 1u;
    uint8_t reserved[3] = {0, 0, 0};
};
#pragma pack(pop)

static_assert(sizeof(TelemetryControlV1) == 12,
              "TelemetryControlV1 size changed; update sender/receiver together.");

#pragma pack(push, 1)
struct RuntimeSettingsPayloadV1 {
    double groundTemperatureF = 0.0;
    double windSpeedMph = 0.0;
    double windDirectionDeg = 0.0;
    double launchDirectionDeg = 0.0;
    double roughnessLengthMeters = 0.0;
    double gradientHeightMeters = 0.0;
    double measurementHeightMeters = 0.0;
    double centerOfPressureOffsetMeters = 0.0;
    double momentOfInertiaKgM2 = 0.0;
    double dryMassKg = 0.0;
};
#pragma pack(pop)

static_assert(sizeof(RuntimeSettingsPayloadV1) == 80,
              "RuntimeSettingsPayloadV1 size changed; update sender/receiver together.");

#pragma pack(push, 1)
struct SettingsCommandV1 {
    uint32_t magic = kSettingsCommandMagic;
    uint16_t version = kSettingsCommandVersion;
    uint16_t size = sizeof(SettingsCommandV1);
    uint8_t operation = kSettingsOpRequestCurrent;
    uint8_t reserved[3] = {0, 0, 0};
    uint32_t requestId = 0;
    RuntimeSettingsPayloadV1 payload{};
};
#pragma pack(pop)

static_assert(sizeof(SettingsCommandV1) == 96,
              "SettingsCommandV1 size changed; update sender/receiver together.");

#pragma pack(push, 1)
struct SettingsSnapshotV1 {
    uint32_t magic = kSettingsSnapshotMagic;
    uint16_t version = kSettingsSnapshotVersion;
    uint16_t size = sizeof(SettingsSnapshotV1);
    uint32_t settingsRevision = 0;
    uint32_t appliedRequestId = 0;
    uint8_t statusFlags = 0;
    uint8_t lastCommandResult = kSettingsResultNone;
    uint16_t reserved = 0;
    RuntimeSettingsPayloadV1 payload{};
};
#pragma pack(pop)

static_assert(sizeof(SettingsSnapshotV1) == 100,
              "SettingsSnapshotV1 size changed; update sender/receiver together.");

}  // namespace telemetry
