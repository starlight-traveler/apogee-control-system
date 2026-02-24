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

void PollSubscriberPackets(uint32_t nowMs) {
    int packetBytes = g_udp.parsePacket();
    while (packetBytes > 0) {
        if (packetBytes == static_cast<int>(sizeof(telemetry::HeartbeatV1))) {
            telemetry::HeartbeatV1 heartbeat{};
            const int n = g_udp.read(reinterpret_cast<uint8_t *>(&heartbeat), sizeof(heartbeat));
            if (n == static_cast<int>(sizeof(heartbeat)) &&
                heartbeat.magic == telemetry::kHeartbeatMagic &&
                heartbeat.version == telemetry::kHeartbeatVersion &&
                heartbeat.size == sizeof(telemetry::HeartbeatV1)) {
                g_hasSubscriber = true;
                g_lastSubscriberMs = nowMs;
                g_subscriberIp = g_udp.remoteIP();
                g_subscriberPort = g_udp.remotePort();
            }
        } else if (packetBytes == static_cast<int>(sizeof(telemetry::ActuationCommandV1))) {
            telemetry::ActuationCommandV1 command{};
            const int n = g_udp.read(reinterpret_cast<uint8_t *>(&command), sizeof(command));
            if (n == static_cast<int>(sizeof(command)) &&
                command.magic == telemetry::kActuationCommandMagic &&
                command.version == telemetry::kActuationCommandVersion &&
                command.size == sizeof(telemetry::ActuationCommandV1)) {
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
                g_hasSubscriber = true;
                g_lastSubscriberMs = nowMs;
                g_subscriberIp = g_udp.remoteIP();
                g_subscriberPort = g_udp.remotePort();
            }
        } else {
            while (packetBytes-- > 0) {
                g_udp.read();
            }
        }

        packetBytes = g_udp.parsePacket();
    }
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

void NetworkTelemetryService(const TelemetrySnapshot &snapshot) {
    if (!settings::network::kEnableTelemetry || !g_wifiReady || !g_udpStarted) {
        return;
    }

    const uint32_t now = millis();
    PollSubscriberPackets(now);

    if (!SubscriberActive(now)) {
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

    if (!g_hasSubscriber || !g_udp.beginPacket(g_subscriberIp, g_subscriberPort)) {
        return;
    }
    g_udp.write(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
    g_udp.endPacket();
}

bool NetworkTelemetryConnected() {
    return WiFiConnected(millis());
}

bool NetworkTelemetrySubscriberActive() {
    return SubscriberActive(millis());
}

bool NetworkTelemetryManualActuationOverride(float &angleDegOut) {
    angleDegOut = g_manualActuationAngleDeg;
    return g_manualActuationOverride;
}
