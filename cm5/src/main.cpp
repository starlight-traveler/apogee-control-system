/// \file main.cpp
/// \brief Host-side telemetry reader and logger for the cm5 flight computer.

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <csignal>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <string>

#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#endif

#include "flight_computer.h"
#include "serial_protocol.h"

namespace {

/// \brief Maximum payload size supported by the frame parser.
constexpr uint16_t kMaxPayload = 256;
/// \brief Telemetry stream rate in Hz.
constexpr float kStreamRateHz = 200.0f;
/// \brief Default serial baud rate.
constexpr int kDefaultBaud = 921600;
/// \brief Enable verbose console output when true.
constexpr bool kVerbose = false;
/// \brief Realtime scheduling priority on Linux.
constexpr int kRealtimePriority = 20;
/// \brief CPU affinity core index, or -1 for no pinning.
constexpr int kAffinityCore = -1;

/// \brief Global run flag controlled by signal handlers.
bool g_running = true;

/// \brief Signal handler to request shutdown.
void HandleSignal(int) {
    g_running = false;
}

/// \brief Convert an integer baud rate to termios speed_t.
/// \param baud Baud rate integer.
/// \return termios speed constant.
speed_t BaudToSpeed(int baud) {
    switch (baud) {
        case 115200:
            return B115200;
        case 230400:
            return B230400;
#ifdef B460800
        case 460800:
            return B460800;
#endif
#ifdef B921600
        case 921600:
            return B921600;
#endif
#ifdef B1000000
        case 1000000:
            return B1000000;
#endif
        default:
            return B115200;
    }
}

/// \brief Minimal serial port wrapper for blocking IO.
class SerialPort {
  public:
    /// \brief Open the serial device.
    /// \param device Path to the serial device.
    /// \param baud Baud rate.
    /// \return True on success.
    bool Open(const std::string &device, int baud) {
        fd_ = open(device.c_str(), O_RDWR | O_NOCTTY);
        if (fd_ < 0) {
            std::fprintf(stderr, "Failed to open %s: %s\n", device.c_str(), std::strerror(errno));
            return false;
        }

        termios options{};
        if (tcgetattr(fd_, &options) != 0) {
            std::fprintf(stderr, "tcgetattr failed: %s\n", std::strerror(errno));
            return false;
        }

        cfmakeraw(&options);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~CRTSCTS;
        options.c_cc[VMIN] = 1;
        options.c_cc[VTIME] = 0;

        const speed_t speed = BaudToSpeed(baud);
        cfsetispeed(&options, speed);
        cfsetospeed(&options, speed);

        if (tcsetattr(fd_, TCSANOW, &options) != 0) {
            std::fprintf(stderr, "tcsetattr failed: %s\n", std::strerror(errno));
            return false;
        }
        tcflush(fd_, TCIOFLUSH);

        return true;
    }

    /// \brief Read from the serial device.
    /// \param buffer Destination buffer.
    /// \param length Maximum bytes to read.
    /// \return Bytes read or negative on error.
    ssize_t Read(uint8_t *buffer, size_t length) {
        if (fd_ < 0) {
            return -1;
        }
        return read(fd_, buffer, length);
    }

    /// \brief Write to the serial device.
    /// \param buffer Data to send.
    /// \param length Bytes to write.
    /// \return Bytes written or negative on error.
    ssize_t Write(const uint8_t *buffer, size_t length) {
        if (fd_ < 0) {
            return -1;
        }
        return write(fd_, buffer, length);
    }

    /// \brief Close the serial device.
    void Close() {
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
    }

    /// \brief Close the device on destruction.
    ~SerialPort() { Close(); }

  private:
    /// \brief File descriptor for the serial port.
    int fd_ = -1;
};

/// \brief Buffer holding one decoded frame.
struct FrameBuffer {
    /// \brief Parsed frame header.
    serial_protocol::FrameHeader header{};
    /// \brief Payload buffer.
    std::array<uint8_t, kMaxPayload> payload{};
    /// \brief Payload length in bytes.
    uint16_t payloadLength = 0;
};

/// \brief Parser state machine states.
enum class ParseState { Sync1, Sync2, Header, Payload, Crc };

/// \brief Incremental parser for the serial frame format.
class FrameParser {
  public:
    /// \brief Feed one byte into the parser.
    /// \param byte Next byte from the stream.
    /// \param frame Output frame when a full message is decoded.
    /// \return True when a full frame is available.
    bool Feed(uint8_t byte, FrameBuffer &frame) {
        switch (state_) {
            case ParseState::Sync1:
                if (byte == serial_protocol::kSync1) {
                    header_ = {};
                    header_.sync1 = byte;
                    state_ = ParseState::Sync2;
                }
                break;
            case ParseState::Sync2:
                if (byte == serial_protocol::kSync2) {
                    header_.sync2 = byte;
                    offset_ = 0;
                    state_ = ParseState::Header;
                } else {
                    state_ = ParseState::Sync1;
                }
                break;
            case ParseState::Header: {
                uint8_t *headerBytes = reinterpret_cast<uint8_t *>(&header_);
                headerBytes[2 + offset_] = byte;
                ++offset_;
                if (offset_ >= sizeof(serial_protocol::FrameHeader) - 2) {
                    if (header_.version != serial_protocol::kVersion || header_.payloadLength > kMaxPayload) {
                        Reset();
                        break;
                    }
                    payloadLength_ = header_.payloadLength;
                    offset_ = 0;
                    state_ = payloadLength_ > 0 ? ParseState::Payload : ParseState::Crc;
                }
                break;
            }
            case ParseState::Payload:
                payload_[offset_++] = byte;
                if (offset_ >= payloadLength_) {
                    crcBytes_ = 0;
                    state_ = ParseState::Crc;
                }
                break;
            case ParseState::Crc:
                crcBuffer_[crcBytes_++] = byte;
                if (crcBytes_ >= sizeof(uint16_t)) {
                    uint16_t receivedCrc = 0;
                    std::memcpy(&receivedCrc, crcBuffer_, sizeof(receivedCrc));
                    const uint16_t computed = ComputeCrc();
                    if (receivedCrc == computed) {
                        frame.header = header_;
                        frame.payloadLength = payloadLength_;
                        std::memcpy(frame.payload.data(), payload_.data(), payloadLength_);
                        Reset();
                        return true;
                    }
                    Reset();
                }
                break;
        }
        return false;
    }

  private:
    /// \brief Compute the CRC over the current header and payload.
    /// \return CRC value.
    uint16_t ComputeCrc() const {
        const uint8_t *headerBytes = reinterpret_cast<const uint8_t *>(&header_);
        const size_t headerLen = sizeof(serial_protocol::FrameHeader) - 2;
        uint16_t crc = serial_protocol::Crc16Ccitt(headerBytes + 2, headerLen);
        return serial_protocol::Crc16Ccitt(payload_.data(), payloadLength_, crc);
    }

    /// \brief Reset parser state to look for sync bytes.
    void Reset() {
        state_ = ParseState::Sync1;
        offset_ = 0;
        payloadLength_ = 0;
        crcBytes_ = 0;
    }

    /// \brief Current parser state.
    ParseState state_ = ParseState::Sync1;
    /// \brief Working header being filled.
    serial_protocol::FrameHeader header_{};
    /// \brief Working payload buffer.
    std::array<uint8_t, kMaxPayload> payload_{};
    /// \brief Length of the payload for the current frame.
    uint16_t payloadLength_ = 0;
    /// \brief Offset within the header or payload.
    uint16_t offset_ = 0;
    /// \brief CRC byte buffer.
    uint8_t crcBuffer_[2] = {};
    /// \brief CRC byte count.
    uint16_t crcBytes_ = 0;
};

/// \brief Compute the CRC for a full frame.
/// \param header Frame header.
/// \param payload Payload bytes.
/// \param length Payload length.
/// \return CRC value.
uint16_t ComputeCrc(const serial_protocol::FrameHeader &header,
                    const uint8_t *payload,
                    uint16_t length) {
    const uint8_t *headerBytes = reinterpret_cast<const uint8_t *>(&header);
    const size_t headerLen = sizeof(serial_protocol::FrameHeader) - 2;
    uint16_t crc = serial_protocol::Crc16Ccitt(headerBytes + 2, headerLen);
    return serial_protocol::Crc16Ccitt(payload, length, crc);
}

/// \brief Send a full frame to the serial port.
/// \param port Serial port to write to.
/// \param type Message type.
/// \param sequence Sequence number.
/// \param payload Payload bytes.
/// \param length Payload length.
/// \return True on success.
bool SendFrame(SerialPort &port,
               serial_protocol::MessageType type,
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

    if (port.Write(reinterpret_cast<const uint8_t *>(&header), sizeof(header)) < 0) {
        return false;
    }
    if (length > 0 && payload != nullptr) {
        if (port.Write(payload, length) < 0) {
            return false;
        }
    }
    if (port.Write(reinterpret_cast<const uint8_t *>(&crc), sizeof(crc)) < 0) {
        return false;
    }
    return true;
}

/// \brief Send a command packet over the serial port.
/// \param port Serial port to write to.
/// \param commandId Command identifier.
/// \param payload Optional payload pointer.
/// \param payloadLength Payload length.
/// \param sequence Sequence number.
/// \return True on success.
bool SendCommand(SerialPort &port,
                 uint8_t commandId,
                 const void *payload,
                 uint16_t payloadLength,
                 uint32_t sequence) {
    std::array<uint8_t, 64> buffer{};
    if (payloadLength + sizeof(serial_protocol::CommandPacket) > buffer.size()) {
        return false;
    }

    serial_protocol::CommandPacket cmd{};
    cmd.commandId = commandId;
    cmd.payloadLength = payloadLength;
    std::memcpy(buffer.data(), &cmd, sizeof(cmd));
    if (payloadLength > 0 && payload != nullptr) {
        std::memcpy(buffer.data() + sizeof(cmd), payload, payloadLength);
    }

    return SendFrame(port,
                     serial_protocol::MessageType::Command,
                     sequence,
                     buffer.data(),
                     static_cast<uint16_t>(sizeof(cmd) + payloadLength));
}

/// \brief Send a PWM command packet.
/// \param port Serial port to write to.
/// \param channel PWM channel.
/// \param duty PWM duty cycle.
/// \param hasFrequency True if frequency is set.
/// \param frequencyHz Frequency in Hz.
/// \param sequence Sequence number.
/// \return True on success.
bool SendPwmCommand(SerialPort &port,
                    uint8_t channel,
                    uint16_t duty,
                    bool hasFrequency,
                    uint32_t frequencyHz,
                    uint32_t sequence) {
    serial_protocol::PwmCommandPayload payload{};
    payload.channel = channel;
    payload.flags = hasFrequency ? 1 : 0;
    payload.duty = duty;
    payload.frequencyHz = frequencyHz;

    return SendCommand(port,
                       static_cast<uint8_t>(serial_protocol::CommandId::SetPwm),
                       &payload,
                       sizeof(payload),
                       sequence);
}

}  // namespace

/// \brief Configure low-latency scheduling and affinity on Linux.
void ConfigureLowLatency() {
#if defined(__linux__)
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0 && kVerbose) {
        std::fprintf(stderr, "mlockall failed: %s\n", std::strerror(errno));
    }

    sched_param params{};
    params.sched_priority = kRealtimePriority;
    if (sched_setscheduler(0, SCHED_FIFO, &params) != 0 && kVerbose) {
        std::fprintf(stderr, "sched_setscheduler failed: %s\n", std::strerror(errno));
    }

    if (kAffinityCore >= 0) {
        cpu_set_t mask;
        CPU_ZERO(&mask);
        CPU_SET(static_cast<size_t>(kAffinityCore), &mask);
        if (pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask) != 0 && kVerbose) {
            std::fprintf(stderr, "pthread_setaffinity_np failed: %s\n", std::strerror(errno));
        }
    }
#endif
}

/// \brief Log record written to the binary telemetry file.
struct LogRecord {
    /// \brief Raw sensor data.
    SensorData sensor;
    /// \brief Filtered state data.
    FilteredState state;
    /// \brief Flag indicating state validity.
    uint8_t hasState;
    /// \brief Encoded flight status.
    uint8_t status;
    /// \brief Reserved padding.
    uint16_t reserved;
};

/// \brief Open a binary log file with buffered IO.
/// \param path Path to the log file.
/// \return File pointer or nullptr on failure.
FILE *OpenLogFile(const std::string &path) {
    FILE *file = std::fopen(path.c_str(), "wb");
    if (!file) {
        std::fprintf(stderr, "Failed to open log file %s: %s\n", path.c_str(), std::strerror(errno));
        return nullptr;
    }
    static std::array<char, 1024 * 1024> buffer{};
    setvbuf(file, buffer.data(), _IOFBF, buffer.size());
    return file;
}

/// \brief Write one telemetry record to the log file.
/// \param file Log file handle.
/// \param sensor Raw sensor data.
/// \param state Filtered state data.
/// \param hasState True if state data is valid.
/// \param status Current flight status.
void WriteLogRecord(FILE *file, const SensorData &sensor, const FilteredState &state, bool hasState, FlightStatus status) {
    if (!file) {
        return;
    }
    LogRecord record{};
    record.sensor = sensor;
    record.state = state;
    record.hasState = hasState ? 1 : 0;
    record.status = static_cast<uint8_t>(status);
    std::fwrite(&record, sizeof(record), 1, file);
}

/// \brief Application entry point.
int main(int argc, char **argv) {
    const std::string device = argc > 1 ? argv[1] : "/dev/ttyACM0";
    const int baud = argc > 2 ? std::atoi(argv[2]) : kDefaultBaud;
    const std::string logPath = argc > 3 ? argv[3] : "cm5_telemetry.bin";
    const std::string cfdPath = argc > 4 ? argv[4] : "../../lib/cfd.csv";
    const bool sendPwm = argc > 6;
    const uint8_t pwmChannel = sendPwm ? static_cast<uint8_t>(std::atoi(argv[5])) : 0;
    const uint16_t pwmDuty = sendPwm ? static_cast<uint16_t>(std::atoi(argv[6])) : 0;
    const bool pwmHasFrequency = argc > 7;
    const uint32_t pwmFrequency = pwmHasFrequency ? static_cast<uint32_t>(std::atoi(argv[7])) : 0;

    SerialPort port;
    if (!port.Open(device, baud)) {
        return 1;
    }

    ConfigureLowLatency();

    FILE *logFile = OpenLogFile(logPath);
    if (!logFile) {
        return 1;
    }

    signal(SIGINT, HandleSignal);
    signal(SIGTERM, HandleSignal);

    FlightComputer flightComputer;
    EnvironmentModel::Config environmentConfig;
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
    if (!flightComputer.LoadCfdTable(cfdPath.c_str()) && kVerbose) {
        std::fprintf(stderr, "CFD table not loaded: %s\n", cfdPath.c_str());
    }

    uint32_t txSequence = 0;
    SendCommand(port,
                static_cast<uint8_t>(serial_protocol::CommandId::SetStreamEnabled),
                "\x01",
                1,
                ++txSequence);
    SendCommand(port,
                static_cast<uint8_t>(serial_protocol::CommandId::SetStreamRateHz),
                &kStreamRateHz,
                sizeof(kStreamRateHz),
                ++txSequence);
    if (sendPwm) {
        SendPwmCommand(port, pwmChannel, pwmDuty, pwmHasFrequency, pwmFrequency, ++txSequence);
    }

    FrameParser parser;
    FrameBuffer frame;
    std::array<uint8_t, 256> readBuffer{};
    FlightStatus lastStatus = flightComputer.Status();

    auto lastPing = std::chrono::steady_clock::now();

    while (g_running) {
        const ssize_t bytesRead = port.Read(readBuffer.data(), readBuffer.size());
        if (bytesRead > 0) {
            for (ssize_t i = 0; i < bytesRead; ++i) {
                if (!parser.Feed(readBuffer[i], frame)) {
                    continue;
                }

                if (frame.header.messageType ==
                    static_cast<uint8_t>(serial_protocol::MessageType::Telemetry)) {
                    if (frame.payloadLength != sizeof(serial_protocol::TelemetryPayload)) {
                        continue;
                    }
                    serial_protocol::TelemetryPayload payload{};
                    std::memcpy(&payload, frame.payload.data(), sizeof(payload));

                    SensorData data{};
                    data.timestamp = payload.timestamp;
                    data.altitudeFeet = payload.altitudeFeet;
                    std::memcpy(data.accelBNO, payload.accelBNO, sizeof(payload.accelBNO));
                    std::memcpy(data.accelICM, payload.accelICM, sizeof(payload.accelICM));
                    std::memcpy(data.quaternion, payload.quaternion, sizeof(payload.quaternion));
                    std::memcpy(data.gyro, payload.gyro, sizeof(payload.gyro));
                    data.hasQuaternion = payload.hasQuaternion != 0;

                    FilteredState state;
                    const bool hasState = flightComputer.Update(data, state);
                    const FlightStatus status = flightComputer.Status();
                    WriteLogRecord(logFile, data, state, hasState, status);
                    if (kVerbose && status != lastStatus) {
                        std::printf("Status=%s t=%.3f z=%.2f apg=%.2f\n",
                                    FlightStatusToString(status),
                                    state.time,
                                    state.position[2],
                                    state.apogeeEstimate);
                        lastStatus = status;
                    }
                } else if (frame.header.messageType ==
                           static_cast<uint8_t>(serial_protocol::MessageType::Ack)) {
                    if (frame.payloadLength == sizeof(serial_protocol::AckPayload)) {
                        serial_protocol::AckPayload ack{};
                        std::memcpy(&ack, frame.payload.data(), sizeof(ack));
                        if (kVerbose) {
                            std::printf("Ack cmd=%u status=%u seq=%u\n",
                                        ack.commandId,
                                        ack.status,
                                        ack.commandSequence);
                        }
                    }
                }
            }
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - lastPing > std::chrono::seconds(2)) {
            SendCommand(port,
                        static_cast<uint8_t>(serial_protocol::CommandId::Ping),
                        nullptr,
                        0,
                        ++txSequence);
            lastPing = now;
        }
    }

    std::fclose(logFile);
    return 0;
}
