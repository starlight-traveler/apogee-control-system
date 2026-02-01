#include "serial_link.h"

#include <string.h>

#include "serial_protocol.h"

namespace {

constexpr uint16_t kMaxPayload = 128;
constexpr float kDefaultStreamRateHz = 200.0f;
constexpr uint32_t kMinIntervalFloorMicros = 1000;

enum class ParseState { Sync1, Sync2, Header, Payload, Crc };

serial_protocol::FrameHeader g_header{};
uint8_t g_payload[kMaxPayload] = {};
uint16_t g_expectedPayload = 0;
uint16_t g_payloadOffset = 0;
uint16_t g_crcBytes = 0;
uint8_t g_crcBuffer[2] = {};
ParseState g_state = ParseState::Sync1;

uint32_t g_sequence = 0;
uint32_t g_lastSendMicros = 0;
uint32_t g_minIntervalMicros = 0;
bool g_streamEnabled = true;

uint16_t ComputeCrc(const serial_protocol::FrameHeader &header, const uint8_t *payload, uint16_t length) {
    const uint8_t *headerBytes = reinterpret_cast<const uint8_t *>(&header);
    const size_t headerLen = sizeof(serial_protocol::FrameHeader) - 2;
    uint16_t crc = serial_protocol::Crc16Ccitt(headerBytes + 2, headerLen);
    return serial_protocol::Crc16Ccitt(payload, length, crc);
}

void SendFrame(serial_protocol::MessageType type,
               uint32_t sequence,
               const uint8_t *payload,
               uint16_t length) {
    serial_protocol::FrameHeader header{};
    header.sync1 = serial_protocol::kSync1;
    header.sync2 = serial_protocol::kSync2;
    header.version = serial_protocol::kVersion;
    header.messageType = static_cast<uint8_t>(type);
    header.payloadLength = length;
    header.sequence = sequence;

    const uint16_t crc = ComputeCrc(header, payload, length);

    Serial.write(reinterpret_cast<const uint8_t *>(&header), sizeof(header));
    if (length > 0 && payload != nullptr) {
        Serial.write(payload, length);
    }
    Serial.write(reinterpret_cast<const uint8_t *>(&crc), sizeof(crc));
#if defined(ARDUINO)
    Serial.send_now();
#endif
}

void SendAck(uint8_t commandId, serial_protocol::AckStatus status, uint32_t commandSequence) {
    serial_protocol::AckPayload ack{};
    ack.commandId = commandId;
    ack.status = static_cast<uint8_t>(status);
    ack.commandSequence = commandSequence;
    SendFrame(serial_protocol::MessageType::Ack, ++g_sequence,
              reinterpret_cast<const uint8_t *>(&ack),
              sizeof(ack));
}

void ApplyStreamRate(float rateHz) {
    if (rateHz <= 0.0f) {
        g_minIntervalMicros = 0;
        return;
    }
    const float intervalMicros = 1.0e6f / rateHz;
    uint32_t interval = static_cast<uint32_t>(intervalMicros);
    if (interval < kMinIntervalFloorMicros) {
        interval = kMinIntervalFloorMicros;
    }
    g_minIntervalMicros = interval;
}

void HandleCommand(uint32_t sequence, const uint8_t *payload, uint16_t length) {
    if (length < sizeof(serial_protocol::CommandPacket)) {
        SendAck(0, serial_protocol::AckStatus::InvalidLength, sequence);
        return;
    }

    serial_protocol::CommandPacket cmd{};
    memcpy(&cmd, payload, sizeof(cmd));
    if (cmd.payloadLength + sizeof(cmd) != length) {
        SendAck(cmd.commandId, serial_protocol::AckStatus::InvalidLength, sequence);
        return;
    }

    const uint8_t *commandPayload = payload + sizeof(cmd);

    switch (static_cast<serial_protocol::CommandId>(cmd.commandId)) {
        case serial_protocol::CommandId::Ping:
            SendAck(cmd.commandId, serial_protocol::AckStatus::Ok, sequence);
            break;
        case serial_protocol::CommandId::SetStreamEnabled:
            if (cmd.payloadLength != 1) {
                SendAck(cmd.commandId, serial_protocol::AckStatus::InvalidLength, sequence);
                break;
            }
            g_streamEnabled = commandPayload[0] != 0;
            SendAck(cmd.commandId, serial_protocol::AckStatus::Ok, sequence);
            break;
        case serial_protocol::CommandId::SetStreamRateHz:
        {
            if (cmd.payloadLength != sizeof(float)) {
                SendAck(cmd.commandId, serial_protocol::AckStatus::InvalidLength, sequence);
                break;
            }
            float rateHz = 0.0f;
            memcpy(&rateHz, commandPayload, sizeof(rateHz));
            ApplyStreamRate(rateHz);
            SendAck(cmd.commandId, serial_protocol::AckStatus::Ok, sequence);
            break;
        }
        default:
            if (SerialLinkHandleCustomCommand(cmd.commandId, commandPayload, cmd.payloadLength)) {
                SendAck(cmd.commandId, serial_protocol::AckStatus::Ok, sequence);
            } else {
                SendAck(cmd.commandId, serial_protocol::AckStatus::Unsupported, sequence);
            }
            break;
    }
}

void ResetParser() {
    g_state = ParseState::Sync1;
    g_expectedPayload = 0;
    g_payloadOffset = 0;
    g_crcBytes = 0;
}

void ProcessByte(uint8_t value) {
    switch (g_state) {
        case ParseState::Sync1:
            if (value == serial_protocol::kSync1) {
                g_header = {};
                g_header.sync1 = value;
                g_state = ParseState::Sync2;
            }
            break;
        case ParseState::Sync2:
            if (value == serial_protocol::kSync2) {
                g_header.sync2 = value;
                g_state = ParseState::Header;
                g_payloadOffset = 0;
            } else {
                g_state = ParseState::Sync1;
            }
            break;
        case ParseState::Header: {
            uint8_t *headerBytes = reinterpret_cast<uint8_t *>(&g_header);
            headerBytes[2 + g_payloadOffset] = value;
            ++g_payloadOffset;
            if (g_payloadOffset >= sizeof(serial_protocol::FrameHeader) - 2) {
                if (g_header.version != serial_protocol::kVersion || g_header.payloadLength > kMaxPayload) {
                    ResetParser();
                    break;
                }
                g_expectedPayload = g_header.payloadLength;
                g_payloadOffset = 0;
                g_state = g_expectedPayload > 0 ? ParseState::Payload : ParseState::Crc;
            }
            break;
        }
        case ParseState::Payload:
            g_payload[g_payloadOffset++] = value;
            if (g_payloadOffset >= g_expectedPayload) {
                g_state = ParseState::Crc;
                g_crcBytes = 0;
            }
            break;
        case ParseState::Crc:
            g_crcBuffer[g_crcBytes++] = value;
            if (g_crcBytes >= sizeof(uint16_t)) {
                uint16_t receivedCrc = 0;
                memcpy(&receivedCrc, g_crcBuffer, sizeof(receivedCrc));
                const uint16_t computedCrc = ComputeCrc(g_header, g_payload, g_expectedPayload);
                if (receivedCrc == computedCrc &&
                    g_header.messageType == static_cast<uint8_t>(serial_protocol::MessageType::Command)) {
                    HandleCommand(g_header.sequence, g_payload, g_expectedPayload);
                }
                ResetParser();
            }
            break;
    }
}

}  // namespace

__attribute__((weak)) bool SerialLinkHandleCustomCommand(uint8_t commandId,
                                                         const uint8_t *payload,
                                                         uint16_t length) {
    (void)commandId;
    (void)payload;
    (void)length;
    return false;
}

bool SerialLinkBegin(uint32_t baud) {
    Serial.begin(baud);
    ApplyStreamRate(kDefaultStreamRateHz);
    g_streamEnabled = true;
    g_lastSendMicros = 0;
    return true;
}

void SerialLinkProcessInput() {
    while (Serial.available() > 0) {
        const uint8_t value = static_cast<uint8_t>(Serial.read());
        ProcessByte(value);
    }
}

void SerialLinkSetStreamRateHz(float rateHz) {
    ApplyStreamRate(rateHz);
}

void SerialLinkSetStreamEnabled(bool enabled) {
    g_streamEnabled = enabled;
}

bool SerialLinkShouldSend(uint32_t nowMicros) {
    if (!g_streamEnabled) {
        return false;
    }
    if (g_minIntervalMicros == 0) {
        return true;
    }
    const uint32_t elapsed = nowMicros - g_lastSendMicros;
    if (elapsed >= g_minIntervalMicros) {
        g_lastSendMicros = nowMicros;
        return true;
    }
    return false;
}

bool SerialLinkSendTelemetry(const SensorData &data) {
    serial_protocol::TelemetryPayload payload{};
    payload.timestamp = data.timestamp;
    payload.altitudeFeet = data.altitudeFeet;
    memcpy(payload.accelBNO, data.accelBNO, sizeof(payload.accelBNO));
    memcpy(payload.accelICM, data.accelICM, sizeof(payload.accelICM));
    memcpy(payload.quaternion, data.quaternion, sizeof(payload.quaternion));
    memcpy(payload.gyro, data.gyro, sizeof(payload.gyro));
    payload.hasQuaternion = data.hasQuaternion ? 1 : 0;

    SendFrame(serial_protocol::MessageType::Telemetry, ++g_sequence,
              reinterpret_cast<const uint8_t *>(&payload),
              sizeof(payload));
    return true;
}
