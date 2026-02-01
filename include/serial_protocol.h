#pragma once

#include <stddef.h>
#include <stdint.h>

#if defined(__GNUC__)
#define SERIAL_PROTOCOL_PACKED __attribute__((packed))
#else
#define SERIAL_PROTOCOL_PACKED
#endif

namespace serial_protocol {

constexpr uint8_t kSync1 = 0xA5;
constexpr uint8_t kSync2 = 0x5A;
constexpr uint8_t kVersion = 1;

enum class MessageType : uint8_t { Telemetry = 1, Command = 2, Ack = 3 };

enum class CommandId : uint8_t {
    Ping = 1,
    SetStreamEnabled = 2,
    SetStreamRateHz = 3,
    SetPwm = 0x80,
};

enum class AckStatus : uint8_t {
    Ok = 0,
    InvalidLength = 1,
    Unsupported = 2,
    BadCrc = 3,
};

struct SERIAL_PROTOCOL_PACKED FrameHeader {
    uint8_t sync1;
    uint8_t sync2;
    uint8_t version;
    uint8_t messageType;
    uint16_t payloadLength;
    uint32_t sequence;
};

struct SERIAL_PROTOCOL_PACKED TelemetryPayload {
    float timestamp;
    float altitudeFeet;
    float accelBNO[3];
    float accelICM[3];
    float quaternion[4];
    float gyro[3];
    uint8_t hasQuaternion;
    uint8_t reserved[3];
};

struct SERIAL_PROTOCOL_PACKED CommandPacket {
    uint8_t commandId;
    uint8_t flags;
    uint16_t payloadLength;
};

struct SERIAL_PROTOCOL_PACKED AckPayload {
    uint8_t commandId;
    uint8_t status;
    uint16_t reserved;
    uint32_t commandSequence;
};

struct SERIAL_PROTOCOL_PACKED PwmCommandPayload {
    uint8_t channel;
    uint8_t flags;
    uint16_t duty;
    uint32_t frequencyHz;
};

inline uint16_t Crc16Ccitt(const uint8_t *data, size_t len, uint16_t crc = 0xFFFF) {
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

}  // namespace serial_protocol
