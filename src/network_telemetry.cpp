#include "network_telemetry.h"

#include <Arduino.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <string.h>

#include "serial_logging.h"
#include "settings.h"
#include "telemetry_packet.h"

namespace {

WiFiUDP g_udp;
uint32_t g_lastSendMs = 0;
uint32_t g_sequence = 0;
bool g_udpStarted = false;
bool g_wifiReady = false;
bool g_wifiConnectedCached = false;
uint32_t g_lastWifiStatusCheckMs = 0;
bool g_hasSubscriber = false;
uint32_t g_lastSubscriberMs = 0;
IPAddress g_subscriberIp;
uint16_t g_subscriberPort = settings::network::kTelemetryUdpRemotePort;
bool g_manualActuationOverride = false;
float g_manualActuationAngleDeg = 0.0f;
bool g_telemetryStreamingEnabled = true;
telemetry::SettingsSnapshotV1 g_settingsSnapshot{};
bool g_hasPendingSettingsCommand = false;
telemetry::SettingsCommandV1 g_pendingSettingsCommand{};
bool g_settingsSnapshotDirty = true;
uint32_t g_lastSettingsSendMs = 0;

constexpr uint32_t kSettingsIntervalMs = 1000;

void UpdateSubscriberEndpoint(uint32_t nowMs, const IPAddress &remoteIp, uint16_t remotePort) {
    g_hasSubscriber = true;
    g_lastSubscriberMs = nowMs;
    g_subscriberIp = remoteIp;
    g_subscriberPort = remotePort;
}

bool SubscriberMatches(const IPAddress &remoteIp, uint16_t remotePort) {
    return g_hasSubscriber && g_subscriberIp == remoteIp && g_subscriberPort == remotePort;
}

IPAddress ConfiguredRemoteIp() {
    return IPAddress(settings::network::kTelemetryRemoteIp0,
                     settings::network::kTelemetryRemoteIp1,
                     settings::network::kTelemetryRemoteIp2,
                     settings::network::kTelemetryRemoteIp3);
}

bool StartUdp() {
    if (g_udpStarted) {
        return true;
    }
    if (!g_udp.begin(settings::network::kTelemetryUdpLocalPort)) {
        return false;
    }
    g_udpStarted = true;
    return true;
}

bool InitWiFi() {
    WiFi.setPins(settings::network::kAirliftSsPin,
                 settings::network::kAirliftAckPin,
                 settings::network::kAirliftResetPin,
                 settings::network::kAirliftGpio0Pin,
                 &SPI);

    if (WiFi.status() == WL_NO_MODULE) {
        LOG_PRINTLN("WiFi module unavailable.");
        return false;
    }

    int status = WL_IDLE_STATUS;
    if (settings::network::kUseAccessPointMode) {
        const size_t passwordLength = strlen(settings::network::kPassword);
        if (passwordLength >= 8 && passwordLength <= 63) {
            status = WiFi.beginAP(settings::network::kSsid, settings::network::kPassword);
        } else {
            status = WiFi.beginAP(settings::network::kSsid);
        }
    } else {
        status = WiFi.begin(settings::network::kSsid, settings::network::kPassword);
    }

    const bool connected =
        (status == WL_CONNECTED) || (status == WL_AP_LISTENING) || (status == WL_AP_CONNECTED);

    if (!connected) {
        LOG_PRINT("WiFi init failed (status=");
        LOG_PRINT(status);
        LOG_PRINTLN(")");
        return false;
    }
    g_wifiConnectedCached = true;
    g_lastWifiStatusCheckMs = millis();
    return true;
}

bool WiFiConnected(uint32_t nowMs) {
    if ((nowMs - g_lastWifiStatusCheckMs) >= settings::network::kWiFiStatusCheckIntervalMs) {
        const int wifiStatus = WiFi.status();
        g_wifiConnectedCached =
            (wifiStatus == WL_CONNECTED) || (wifiStatus == WL_AP_LISTENING) || (wifiStatus == WL_AP_CONNECTED);
        g_lastWifiStatusCheckMs = nowMs;
    }
    return g_wifiConnectedCached;
}

bool SubscriberActive(uint32_t nowMs) {
    if (!g_hasSubscriber) {
        return false;
    }
    const bool active = (nowMs - g_lastSubscriberMs) <= settings::network::kSubscriberHeartbeatTimeoutMs;
    if (!active) {
        // Force a fresh heartbeat before telemetry resumes.
        g_hasSubscriber = false;
        g_subscriberIp = IPAddress();
        g_subscriberPort = settings::network::kTelemetryUdpRemotePort;
        g_manualActuationOverride = false;
        g_manualActuationAngleDeg = 0.0f;
    }
    return active;
}

bool CommandAuthorized(uint32_t nowMs, const IPAddress &remoteIp, uint16_t remotePort) {
    if (!settings::network::kRequireSubscriberHeartbeat) {
        UpdateSubscriberEndpoint(nowMs, remoteIp, remotePort);
        return true;
    }
    return SubscriberActive(nowMs) && SubscriberMatches(remoteIp, remotePort);
}

void FillPacket(const TelemetrySnapshot &snapshot, telemetry::PacketV1 &packet) {
    packet.sequence = g_sequence++;
    packet.uptimeMs = millis();
    packet.flightStatus = static_cast<uint8_t>(snapshot.status);
    packet.flags = 0;

    if (snapshot.sensor != nullptr) {
        packet.sensorTimestamp = snapshot.sensor->timestamp;
        packet.sensorAltitudeFeet = snapshot.sensor->altitudeFeet;
        for (int i = 0; i < 3; ++i) {
            packet.sensorAccelBno[i] = snapshot.sensor->accelBNO[i];
            packet.sensorAccelIcm[i] = snapshot.sensor->accelICM[i];
            packet.sensorGyro[i] = snapshot.sensor->gyro[i];
        }
        for (int i = 0; i < 4; ++i) {
            packet.sensorQuaternion[i] = snapshot.sensor->quaternion[i];
            packet.sensorIcmQuaternion[i] = snapshot.sensor->icmQuaternion[i];
        }
        for (int i = 0; i < 3; ++i) {
            packet.sensorIcmYprDeg[i] = snapshot.sensor->icmYprDeg[i];
        }
        packet.sensorAltimeterSigmaScale = snapshot.sensor->altimeterSigmaScale;
        packet.sensorAltimeterGateSigma = snapshot.sensor->altimeterGateSigma;
        packet.sensorAutoCommandDeg = snapshot.sensor->autoCommandDeg;
        packet.sensorOptimizerBestPredictedApogeeM = snapshot.sensor->optimizerBestPredictedApogeeM;
        packet.sensorOptimizerBestCost = snapshot.sensor->optimizerBestCost;
        packet.sensorOptimizerTimeToApogeeS = snapshot.sensor->optimizerTimeToApogeeS;
        packet.sensorActuationIsSettling = snapshot.sensor->actuationIsSettling;
        packet.sensorPredictorSeedHorizontalSpeedMps = snapshot.sensor->predictorSeedHorizontalSpeedMps;
        packet.sensorPredictorSeedClampedZenithRad = snapshot.sensor->predictorSeedClampedZenithRad;
        packet.sensorPredictorSeedClampedAngularRateRadPerSec =
            snapshot.sensor->predictorSeedClampedAngularRateRadPerSec;
        packet.sensorPredictorSeedConfidenceFlags = snapshot.sensor->predictorSeedConfidenceFlags;
        packet.sensorMainQuaternionSource = snapshot.sensor->mainQuaternionSource;
        packet.sensorHasQuaternion = snapshot.sensor->hasQuaternion ? 1u : 0u;
        packet.sensorHasIcmQuaternion = snapshot.sensor->hasIcmQuaternion ? 1u : 0u;
        packet.sensorHasIcmYpr = snapshot.sensor->hasIcmYpr ? 1u : 0u;
    }

    if (snapshot.state != nullptr) {
        packet.flags |= telemetry::kFlagHasFilteredState;
        packet.stateTime = snapshot.state->time;
        for (int i = 0; i < 3; ++i) {
            packet.statePosition[i] = snapshot.state->position[i];
            packet.stateVelocity[i] = snapshot.state->velocity[i];
            packet.stateAcceleration[i] = snapshot.state->acceleration[i];
            packet.stateInertialAcceleration[i] = snapshot.state->inertialAcceleration[i];
        }
        packet.stateZenith = snapshot.state->zenith;
        packet.stateApogeeEstimate = snapshot.state->apogeeEstimate;
    }

    packet.servoCommandDeg = snapshot.servoCommandDeg;
    packet.servoEffectiveDeg = snapshot.servoEffectiveDeg;
    if (snapshot.hasPadAltitude) {
        packet.flags |= telemetry::kFlagHasPadAltitude;
        packet.altitudeAglFeet = snapshot.altitudeAglFeet;
    }
    if (snapshot.manualActuationOverride) {
        packet.flags |= telemetry::kFlagManualActuationOverride;
    }
}

void FillSettingsPayload(const RuntimeSettings &settings, telemetry::RuntimeSettingsPayloadV1 &payload) {
    payload.groundTemperatureF = static_cast<double>(settings.environment.groundTemperatureF);
    payload.seaLevelPressureHpa = static_cast<double>(settings.environment.seaLevelPressureHpa);
    payload.windSpeedMph = static_cast<double>(settings.environment.windSpeedMph);
    payload.windDirectionDeg = static_cast<double>(settings.environment.windDirectionDeg);
    payload.launchDirectionDeg = static_cast<double>(settings.environment.launchDirectionDeg);
    payload.roughnessLengthMeters = static_cast<double>(settings.environment.roughnessLengthMeters);
    payload.gradientHeightMeters = static_cast<double>(settings.environment.gradientHeightMeters);
    payload.measurementHeightMeters = static_cast<double>(settings.environment.measurementHeightMeters);
    payload.centerOfPressureOffsetMeters = settings.vehicle.centerOfPressureOffsetMeters;
    payload.momentOfInertiaKgM2 = settings.vehicle.momentOfInertia;
    payload.dryMassKg = settings.vehicle.dryMass;
}

void PollSubscriberPackets(uint32_t nowMs) {
    int packetBytes = g_udp.parsePacket();
    while (packetBytes > 0) {
        const IPAddress remoteIp = g_udp.remoteIP();
        const uint16_t remotePort = g_udp.remotePort();
        if (packetBytes == static_cast<int>(sizeof(telemetry::HeartbeatV1))) {
            telemetry::HeartbeatV1 heartbeat{};
            const int n = g_udp.read(reinterpret_cast<uint8_t *>(&heartbeat), sizeof(heartbeat));
            if (n == static_cast<int>(sizeof(heartbeat)) &&
                heartbeat.magic == telemetry::kHeartbeatMagic &&
                heartbeat.version == telemetry::kHeartbeatVersion &&
                heartbeat.size == sizeof(telemetry::HeartbeatV1)) {
                UpdateSubscriberEndpoint(nowMs, remoteIp, remotePort);
            }
        } else if (packetBytes == static_cast<int>(sizeof(telemetry::ActuationCommandV1))) {
            telemetry::ActuationCommandV1 command{};
            const int n = g_udp.read(reinterpret_cast<uint8_t *>(&command), sizeof(command));
            if (n == static_cast<int>(sizeof(command)) &&
                command.magic == telemetry::kActuationCommandMagic &&
                command.version == telemetry::kActuationCommandVersion &&
                command.size == sizeof(telemetry::ActuationCommandV1) &&
                CommandAuthorized(nowMs, remoteIp, remotePort)) {
                if (command.mode == telemetry::kActuationModeManual) {
                    g_manualActuationOverride = true;
                    g_manualActuationAngleDeg = command.angleDeg;
                    if (g_manualActuationAngleDeg < 0.0f) {
                        g_manualActuationAngleDeg = 0.0f;
                    }
                } else {
                    g_manualActuationOverride = false;
                    g_manualActuationAngleDeg = 0.0f;
                }
            }
        } else if (packetBytes == static_cast<int>(sizeof(telemetry::TelemetryControlV1))) {
            telemetry::TelemetryControlV1 control{};
            const int n = g_udp.read(reinterpret_cast<uint8_t *>(&control), sizeof(control));
            if (n == static_cast<int>(sizeof(control)) &&
                control.magic == telemetry::kTelemetryControlMagic &&
                control.version == telemetry::kTelemetryControlVersion &&
                control.size == sizeof(telemetry::TelemetryControlV1) &&
                CommandAuthorized(nowMs, remoteIp, remotePort)) {
                g_telemetryStreamingEnabled = control.telemetryEnabled != 0u;
                if (!g_telemetryStreamingEnabled) {
                    g_manualActuationOverride = false;
                    g_manualActuationAngleDeg = 0.0f;
                }
            }
        } else if (packetBytes == static_cast<int>(sizeof(telemetry::SettingsCommandV1))) {
            telemetry::SettingsCommandV1 command{};
            const int n = g_udp.read(reinterpret_cast<uint8_t *>(&command), sizeof(command));
            if (n == static_cast<int>(sizeof(command)) &&
                command.magic == telemetry::kSettingsCommandMagic &&
                command.version == telemetry::kSettingsCommandVersion &&
                command.size == sizeof(telemetry::SettingsCommandV1) &&
                CommandAuthorized(nowMs, remoteIp, remotePort)) {
                g_pendingSettingsCommand = command;
                g_hasPendingSettingsCommand = true;
                g_settingsSnapshotDirty = true;
            }
        } else {
            while (packetBytes-- > 0) {
                g_udp.read();
            }
        }

        packetBytes = g_udp.parsePacket();
    }
}

}  // namespace

void NetworkTelemetryBegin() {
    if (!settings::network::kEnableTelemetry) {
        return;
    }

    g_wifiReady = InitWiFi();
    if (!g_wifiReady) {
        return;
    }

    if (!StartUdp()) {
        LOG_PRINTLN("UDP startup failed.");
        g_wifiReady = false;
        return;
    }

    LOG_PRINT("Telemetry UDP local/remote ports: ");
    LOG_PRINT(settings::network::kTelemetryUdpLocalPort);
    LOG_PRINT('/');
    LOG_PRINTLN(settings::network::kTelemetryUdpRemotePort);
}

void NetworkTelemetryPollControl() {
    if (!settings::network::kEnableTelemetry || !g_wifiReady || !g_udpStarted) {
        return;
    }

    PollSubscriberPackets(millis());
}

void NetworkTelemetryService(const TelemetrySnapshot &snapshot) {
    if (!settings::network::kEnableTelemetry || !g_wifiReady || !g_udpStarted || !g_telemetryStreamingEnabled) {
        return;
    }

    const uint32_t now = millis();
    const bool subscriberActive = SubscriberActive(now);

    if (!g_telemetryStreamingEnabled) {
        return;
    }

    if ((now - g_lastSendMs) < settings::network::kTelemetryIntervalMs) {
        return;
    }
    g_lastSendMs = now;

    if (!WiFiConnected(now)) {
        return;
    }

    telemetry::PacketV1 packet{};
    FillPacket(snapshot, packet);

    const IPAddress targetIp = subscriberActive ? g_subscriberIp : ConfiguredRemoteIp();
    const uint16_t targetPort = subscriberActive ? g_subscriberPort : settings::network::kTelemetryUdpRemotePort;

    if (!g_udp.beginPacket(targetIp, targetPort)) {
        return;
    }
    g_udp.write(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
    g_udp.endPacket();

    if (subscriberActive &&
        (g_settingsSnapshotDirty || (now - g_lastSettingsSendMs) >= kSettingsIntervalMs)) {
        if (g_udp.beginPacket(targetIp, targetPort)) {
            g_udp.write(reinterpret_cast<const uint8_t *>(&g_settingsSnapshot), sizeof(g_settingsSnapshot));
            g_udp.endPacket();
            g_lastSettingsSendMs = now;
            g_settingsSnapshotDirty = false;
        }
    }
}

bool NetworkTelemetryConnected() {
    return WiFiConnected(millis());
}

bool NetworkTelemetrySubscriberActive() {
    return SubscriberActive(millis());
}

bool NetworkTelemetryManualActuationOverride(float &angleDegOut) {
    if (!SubscriberActive(millis())) {
        angleDegOut = 0.0f;
        return false;
    }
    angleDegOut = g_manualActuationAngleDeg;
    return g_manualActuationOverride;
}

void NetworkTelemetrySetRuntimeSettingsSnapshot(const RuntimeSettings &settings,
                                                const RuntimeSettingsStorageStatus &storageStatus,
                                                uint32_t settingsRevision,
                                                uint32_t appliedRequestId,
                                                uint8_t lastCommandResult) {
    telemetry::SettingsSnapshotV1 snapshot{};
    snapshot.settingsRevision = settingsRevision;
    snapshot.appliedRequestId = appliedRequestId;
    snapshot.lastCommandResult = lastCommandResult;
    if (storageStatus.storageAvailable) {
        snapshot.statusFlags |= telemetry::kSettingsStatusStorageAvailable;
    }
    if (storageStatus.filePresent) {
        snapshot.statusFlags |= telemetry::kSettingsStatusFilePresent;
    }
    if (storageStatus.usingDefaults) {
        snapshot.statusFlags |= telemetry::kSettingsStatusUsingDefaults;
    }
    if (storageStatus.lastLoadSucceeded) {
        snapshot.statusFlags |= telemetry::kSettingsStatusLastLoadSucceeded;
    }
    if (storageStatus.lastSaveSucceeded) {
        snapshot.statusFlags |= telemetry::kSettingsStatusLastSaveSucceeded;
    }
    if (storageStatus.createdDefaultFile) {
        snapshot.statusFlags |= telemetry::kSettingsStatusCreatedDefaultFile;
    }
    FillSettingsPayload(settings, snapshot.payload);
    g_settingsSnapshot = snapshot;
    g_settingsSnapshotDirty = true;
}

bool NetworkTelemetryConsumeSettingsCommand(telemetry::SettingsCommandV1 &commandOut) {
    if (!g_hasPendingSettingsCommand) {
        return false;
    }
    commandOut = g_pendingSettingsCommand;
    g_hasPendingSettingsCommand = false;
    return true;
}
