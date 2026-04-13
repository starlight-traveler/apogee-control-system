#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <unistd.h>

#if defined(ACS_ENABLE_IMGUI_DECODER)
#include "../../../src/apogee_model.h"
#include "../../../src/flight_computer.h"
#include "../../../src/predictor_seed.h"
#endif

#if defined(ACS_ENABLE_IMGUI_DECODER)
#if defined(__APPLE__)
#include <OpenGL/gl3.h>
#else
#include <GL/gl.h>
#endif

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#endif

namespace fs = std::filesystem;

namespace {

// The constants below must stay aligned with the firmware-side binary layout in
// `src/data_logger.h` and `src/data_logger.cpp`. The native decoder is strict
// about record sizes because it decodes directly from raw bytes.
constexpr std::size_t kHeaderSize = 4;
constexpr std::size_t kSensorSize = 248;
constexpr std::size_t kFilteredSize = 0;
constexpr std::size_t kTelemetryPayloadSize = kSensorSize + kFilteredSize;
constexpr std::size_t kEventPayloadSize = 24;
constexpr std::size_t kLogPreambleSize = 64;

constexpr std::array<const char *, 5> kFlightStatusNames = {
    "ground", "burn", "coast", "overshoot", "descent"};

constexpr std::array<const char *, 3> kEventNames = {
    "stage_change", "flap_actuated", "flap_settling_timer_fired"};

constexpr std::array<uint8_t, 8> kLogMagic = {'A', 'C', 'S', 'N', 'D', 'R', 'T', '1'};
constexpr int kSensorFloatCount = 58;
constexpr int kSensorU8Count = 1;
constexpr int kSensorBoolCount = 12;
constexpr int kStateFloatCount = 0;

enum class ColumnClass {
    Sensor,
    State,
    Control,
    Derived,
    Event,
    Replay,
    Unknown
};

struct ColumnMeta {
    const char *units = "";
    ColumnClass klass = ColumnClass::Unknown;
    float expectedMin = std::numeric_limits<float>::quiet_NaN();
    float expectedMax = std::numeric_limits<float>::quiet_NaN();
};

struct LogPreamble {
    uint8_t magic[8];
    uint16_t formatVersion = 0;
    uint16_t schemaVersion = 0;
    char firmwareGitHash[40];
    uint8_t reserved[12];
};

/// Describes where a decoded telemetry field comes from inside one binary record.
enum class TelemetryFieldSource {
    StatusRaw,
    HasFilteredState,
    SensorFloat,
    SensorU8,
    SensorBool,
    StateFloat
};

struct TelemetryFieldDescriptor {
    const char *name;
    TelemetryFieldSource source;
    int index;
    ColumnMeta meta;
};

// Single source of truth for the compact binary telemetry schema used by the decoder.
// Each entry corresponds to one CSV column / GUI series and is kept in lockstep
// with the firmware's `LoggedTelemetrySample` layout.
constexpr std::array<TelemetryFieldDescriptor, 73> kTelemetryFields = {{
    {"flight_status_raw", TelemetryFieldSource::StatusRaw, 0, {"enum", ColumnClass::State, 0.0f, 4.0f}},
    {"has_filtered_state", TelemetryFieldSource::HasFilteredState, 0, {"bool", ColumnClass::State, 0.0f, 1.0f}},
    {"sensor_timestamp", TelemetryFieldSource::SensorFloat, 0, {"s", ColumnClass::Sensor}},
    {"sensor_altitude_feet", TelemetryFieldSource::SensorFloat, 1, {"ft", ColumnClass::Sensor}},
    {"sensor_accel_icm_x", TelemetryFieldSource::SensorFloat, 2, {"m/s^2", ColumnClass::Sensor}},
    {"sensor_accel_icm_y", TelemetryFieldSource::SensorFloat, 3, {"m/s^2", ColumnClass::Sensor}},
    {"sensor_accel_icm_z", TelemetryFieldSource::SensorFloat, 4, {"m/s^2", ColumnClass::Sensor}},
    {"sensor_gyro_x", TelemetryFieldSource::SensorFloat, 5, {"rad/s", ColumnClass::Sensor}},
    {"sensor_gyro_y", TelemetryFieldSource::SensorFloat, 6, {"rad/s", ColumnClass::Sensor}},
    {"sensor_gyro_z", TelemetryFieldSource::SensorFloat, 7, {"rad/s", ColumnClass::Sensor}},
    {"sensor_quat_w", TelemetryFieldSource::SensorFloat, 8, {"quat", ColumnClass::Sensor, -1.0f, 1.0f}},
    {"sensor_quat_x", TelemetryFieldSource::SensorFloat, 9, {"quat", ColumnClass::Sensor, -1.0f, 1.0f}},
    {"sensor_quat_y", TelemetryFieldSource::SensorFloat, 10, {"quat", ColumnClass::Sensor, -1.0f, 1.0f}},
    {"sensor_quat_z", TelemetryFieldSource::SensorFloat, 11, {"quat", ColumnClass::Sensor, -1.0f, 1.0f}},
    {"sensor_icm_quat_w", TelemetryFieldSource::SensorFloat, 12, {"quat", ColumnClass::Sensor, -1.0f, 1.0f}},
    {"sensor_icm_quat_x", TelemetryFieldSource::SensorFloat, 13, {"quat", ColumnClass::Sensor, -1.0f, 1.0f}},
    {"sensor_icm_quat_y", TelemetryFieldSource::SensorFloat, 14, {"quat", ColumnClass::Sensor, -1.0f, 1.0f}},
    {"sensor_icm_quat_z", TelemetryFieldSource::SensorFloat, 15, {"quat", ColumnClass::Sensor, -1.0f, 1.0f}},
    {"sensor_accel_lsm_x", TelemetryFieldSource::SensorFloat, 16, {"m/s^2", ColumnClass::Sensor}},
    {"sensor_accel_lsm_y", TelemetryFieldSource::SensorFloat, 17, {"m/s^2", ColumnClass::Sensor}},
    {"sensor_accel_lsm_z", TelemetryFieldSource::SensorFloat, 18, {"m/s^2", ColumnClass::Sensor}},
    {"sensor_gyro_lsm_x", TelemetryFieldSource::SensorFloat, 19, {"rad/s", ColumnClass::Sensor}},
    {"sensor_gyro_lsm_y", TelemetryFieldSource::SensorFloat, 20, {"rad/s", ColumnClass::Sensor}},
    {"sensor_gyro_lsm_z", TelemetryFieldSource::SensorFloat, 21, {"rad/s", ColumnClass::Sensor}},
    {"sensor_lsm_quat_w", TelemetryFieldSource::SensorFloat, 22, {"quat", ColumnClass::Sensor, -1.0f, 1.0f}},
    {"sensor_lsm_quat_x", TelemetryFieldSource::SensorFloat, 23, {"quat", ColumnClass::Sensor, -1.0f, 1.0f}},
    {"sensor_lsm_quat_y", TelemetryFieldSource::SensorFloat, 24, {"quat", ColumnClass::Sensor, -1.0f, 1.0f}},
    {"sensor_lsm_quat_z", TelemetryFieldSource::SensorFloat, 25, {"quat", ColumnClass::Sensor, -1.0f, 1.0f}},
    {"sensor_flap_command_deg", TelemetryFieldSource::SensorFloat, 26, {"deg", ColumnClass::Control}},
    {"sensor_flap_effective_deg", TelemetryFieldSource::SensorFloat, 27, {"deg", ColumnClass::Control}},
    {"state_altitude_agl_feet", TelemetryFieldSource::SensorFloat, 28, {"ft", ColumnClass::State}},
    {"state_vertical_velocity_fps", TelemetryFieldSource::SensorFloat, 29, {"ft/s", ColumnClass::State}},
    {"state_zenith_deg", TelemetryFieldSource::SensorFloat, 30, {"deg", ColumnClass::State}},
    {"state_apogee_estimate_feet", TelemetryFieldSource::SensorFloat, 31, {"ft", ColumnClass::State}},
    {"sensor_accel_bno_x", TelemetryFieldSource::SensorFloat, 32, {"m/s^2", ColumnClass::Sensor}},
    {"sensor_accel_bno_y", TelemetryFieldSource::SensorFloat, 33, {"m/s^2", ColumnClass::Sensor}},
    {"sensor_accel_bno_z", TelemetryFieldSource::SensorFloat, 34, {"m/s^2", ColumnClass::Sensor}},
    {"sensor_gyro_bno_x", TelemetryFieldSource::SensorFloat, 35, {"rad/s", ColumnClass::Sensor}},
    {"sensor_gyro_bno_y", TelemetryFieldSource::SensorFloat, 36, {"rad/s", ColumnClass::Sensor}},
    {"sensor_gyro_bno_z", TelemetryFieldSource::SensorFloat, 37, {"rad/s", ColumnClass::Sensor}},
    {"sensor_bno_quat_w", TelemetryFieldSource::SensorFloat, 38, {"quat", ColumnClass::Sensor, -1.0f, 1.0f}},
    {"sensor_bno_quat_x", TelemetryFieldSource::SensorFloat, 39, {"quat", ColumnClass::Sensor, -1.0f, 1.0f}},
    {"sensor_bno_quat_y", TelemetryFieldSource::SensorFloat, 40, {"quat", ColumnClass::Sensor, -1.0f, 1.0f}},
    {"sensor_bno_quat_z", TelemetryFieldSource::SensorFloat, 41, {"quat", ColumnClass::Sensor, -1.0f, 1.0f}},
    {"sensor_bno_yaw_deg", TelemetryFieldSource::SensorFloat, 42, {"deg", ColumnClass::Sensor}},
    {"sensor_bno_pitch_deg", TelemetryFieldSource::SensorFloat, 43, {"deg", ColumnClass::Sensor}},
    {"sensor_bno_roll_deg", TelemetryFieldSource::SensorFloat, 44, {"deg", ColumnClass::Sensor}},
    {"sensor_accel_wt901_x", TelemetryFieldSource::SensorFloat, 45, {"m/s^2", ColumnClass::Sensor}},
    {"sensor_accel_wt901_y", TelemetryFieldSource::SensorFloat, 46, {"m/s^2", ColumnClass::Sensor}},
    {"sensor_accel_wt901_z", TelemetryFieldSource::SensorFloat, 47, {"m/s^2", ColumnClass::Sensor}},
    {"sensor_wt901_yaw_deg", TelemetryFieldSource::SensorFloat, 48, {"deg", ColumnClass::Sensor}},
    {"sensor_wt901_pitch_deg", TelemetryFieldSource::SensorFloat, 49, {"deg", ColumnClass::Sensor}},
    {"sensor_wt901_roll_deg", TelemetryFieldSource::SensorFloat, 50, {"deg", ColumnClass::Sensor}},
    {"sensor_gyro_wt901_x", TelemetryFieldSource::SensorFloat, 51, {"rad/s", ColumnClass::Sensor}},
    {"sensor_gyro_wt901_y", TelemetryFieldSource::SensorFloat, 52, {"rad/s", ColumnClass::Sensor}},
    {"sensor_gyro_wt901_z", TelemetryFieldSource::SensorFloat, 53, {"rad/s", ColumnClass::Sensor}},
    {"sensor_wt901_quat_w", TelemetryFieldSource::SensorFloat, 54, {"quat", ColumnClass::Sensor, -1.0f, 1.0f}},
    {"sensor_wt901_quat_x", TelemetryFieldSource::SensorFloat, 55, {"quat", ColumnClass::Sensor, -1.0f, 1.0f}},
    {"sensor_wt901_quat_y", TelemetryFieldSource::SensorFloat, 56, {"quat", ColumnClass::Sensor, -1.0f, 1.0f}},
    {"sensor_wt901_quat_z", TelemetryFieldSource::SensorFloat, 57, {"quat", ColumnClass::Sensor, -1.0f, 1.0f}},
    {"sensor_main_quaternion_source", TelemetryFieldSource::SensorU8, 0, {"enum", ColumnClass::Sensor, 0.0f, 5.0f}},
    {"sensor_has_quaternion", TelemetryFieldSource::SensorBool, 0, {"bool", ColumnClass::Sensor, 0.0f, 1.0f}},
    {"sensor_has_icm_quaternion", TelemetryFieldSource::SensorBool, 1, {"bool", ColumnClass::Sensor, 0.0f, 1.0f}},
    {"sensor_has_lsm_quaternion", TelemetryFieldSource::SensorBool, 2, {"bool", ColumnClass::Sensor, 0.0f, 1.0f}},
    {"sensor_icm_accel_saturated", TelemetryFieldSource::SensorBool, 3, {"bool", ColumnClass::Sensor, 0.0f, 1.0f}},
    {"sensor_icm_gyro_saturated", TelemetryFieldSource::SensorBool, 4, {"bool", ColumnClass::Sensor, 0.0f, 1.0f}},
    {"sensor_actuation_is_settling", TelemetryFieldSource::SensorBool, 5, {"bool", ColumnClass::Control, 0.0f, 1.0f}},
    {"sensor_has_bno_quaternion", TelemetryFieldSource::SensorBool, 6, {"bool", ColumnClass::Sensor, 0.0f, 1.0f}},
    {"sensor_has_bno_ypr", TelemetryFieldSource::SensorBool, 7, {"bool", ColumnClass::Sensor, 0.0f, 1.0f}},
    {"sensor_has_wt901_accel", TelemetryFieldSource::SensorBool, 8, {"bool", ColumnClass::Sensor, 0.0f, 1.0f}},
    {"sensor_has_wt901_ypr", TelemetryFieldSource::SensorBool, 9, {"bool", ColumnClass::Sensor, 0.0f, 1.0f}},
    {"sensor_has_wt901_gyro", TelemetryFieldSource::SensorBool, 10, {"bool", ColumnClass::Sensor, 0.0f, 1.0f}},
    {"sensor_has_wt901_quaternion", TelemetryFieldSource::SensorBool, 11, {"bool", ColumnClass::Sensor, 0.0f, 1.0f}},
}};

constexpr int kLogSchemaVersion = 10;
#if defined(ACS_FIRMWARE_GIT_HASH)
constexpr const char *kExpectedFirmwareGitHash = ACS_FIRMWARE_GIT_HASH;
#else
constexpr const char *kExpectedFirmwareGitHash = "unknown";
#endif

struct TelemetryRecordRef {
    uint8_t status = 0;
    uint8_t flags = 0;
    const uint8_t *payload = nullptr;
};

struct EventRecord {
    uint8_t eventType = 0;
    uint8_t flightStatus = 0;
    float timestamp = 0.0f;
    float altitudeAglFeet = 0.0f;
    float verticalVelocityFps = 0.0f;
    float apogeeEstimateFeet = 0.0f;
    float flapCommandDeg = 0.0f;
    float flapEffectiveDeg = 0.0f;
};

static_assert(sizeof(LogPreamble) == kLogPreambleSize, "LogPreamble size mismatch.");

template <typename T>
T ReadLE(const uint8_t *ptr) {
    T out{};
    std::memcpy(&out, ptr, sizeof(T));
    return out;
}

const char *FlightStatusName(uint8_t value) {
    if (value < kFlightStatusNames.size()) {
        return kFlightStatusNames[value];
    }
    return "unknown";
}

const char *EventTypeName(uint8_t value) {
    if (value < kEventNames.size()) {
        return kEventNames[value];
    }
    return "unknown";
}

void AppendCSVEscaped(std::string &line, const char *text) {
    // Current fields don't require escaping; keep for completeness.
    line.append(text);
}

void AppendFloat(std::string &line, float value) {
    if (std::isfinite(value)) {
        char buf[64];
        const int n = std::snprintf(buf, sizeof(buf), "%.9g", static_cast<double>(value));
        if (n > 0) {
            line.append(buf, static_cast<std::size_t>(n));
        }
    }
}

void AppendInt(std::string &line, int value) {
    char buf[32];
    const int n = std::snprintf(buf, sizeof(buf), "%d", value);
    if (n > 0) {
        line.append(buf, static_cast<std::size_t>(n));
    }
}

void AppendBoolWord(std::string &line, bool value) { line.append(value ? "True" : "False"); }

void AppendComma(std::string &line) { line.push_back(','); }

/// Reads one decoded telemetry field from a binary telemetry record.
bool ReadTelemetryFieldValue(const TelemetryRecordRef &rec, const TelemetryFieldDescriptor &field, float &outValue) {
    const bool hasFiltered = (rec.flags & 0x01u) != 0u;
    switch (field.source) {
        case TelemetryFieldSource::StatusRaw:
            outValue = static_cast<float>(rec.status);
            return true;
        case TelemetryFieldSource::HasFilteredState:
            outValue = hasFiltered ? 1.0f : 0.0f;
            return true;
        case TelemetryFieldSource::SensorFloat:
            if (field.index < 0 || field.index >= kSensorFloatCount) {
                return false;
            }
            outValue = ReadLE<float>(rec.payload + static_cast<std::size_t>(field.index) * sizeof(float));
            return true;
        case TelemetryFieldSource::SensorU8: {
            if (field.index < 0 || field.index >= kSensorU8Count) {
                return false;
            }
            const std::size_t u8Offset = static_cast<std::size_t>(kSensorFloatCount) * sizeof(float);
            outValue = static_cast<float>(rec.payload[u8Offset + static_cast<std::size_t>(field.index)]);
            return true;
        }
        case TelemetryFieldSource::SensorBool: {
            if (field.index < 0 || field.index >= kSensorBoolCount) {
                return false;
            }
            const std::size_t boolOffset =
                static_cast<std::size_t>(kSensorFloatCount) * sizeof(float) + static_cast<std::size_t>(kSensorU8Count);
            outValue = (rec.payload[boolOffset + static_cast<std::size_t>(field.index)] != 0u) ? 1.0f : 0.0f;
            return true;
        }
        case TelemetryFieldSource::StateFloat:
            if (!hasFiltered || field.index < 0 || field.index >= kStateFloatCount) {
                return false;
            }
            outValue = ReadLE<float>(rec.payload + kSensorSize + static_cast<std::size_t>(field.index) * sizeof(float));
            return true;
    }
    return false;
}

/// Serializes one telemetry record into the decoder's CSV output format.
void BuildTelemetryLine(const TelemetryRecordRef &rec, std::string &line) {
    line.clear();
    line.reserve(900);

    AppendCSVEscaped(line, FlightStatusName(rec.status));
    for (const auto &field : kTelemetryFields) {
        AppendComma(line);
        float v = std::numeric_limits<float>::quiet_NaN();
        if (!ReadTelemetryFieldValue(rec, field, v)) {
            continue;
        }
        if (field.source == TelemetryFieldSource::SensorBool ||
            field.source == TelemetryFieldSource::HasFilteredState) {
            AppendBoolWord(line, v > 0.5f);
        } else if (field.source == TelemetryFieldSource::SensorU8 ||
                   field.source == TelemetryFieldSource::StatusRaw) {
            AppendInt(line, static_cast<int>(std::lround(v)));
        } else {
            AppendFloat(line, v);
        }
    }
    line.push_back('\n');
}

/// Builds the CSV header from the shared schema descriptor table.
std::string TelemetryHeader() {
    std::string header = "flight_status";
    for (const auto &field : kTelemetryFields) {
        header.push_back(',');
        header += field.name;
    }
    header.push_back('\n');
    return header;
}

/// Emits decoder-side metadata lines that describe schema and provenance.
std::string TelemetryMetadataPreamble() {
    std::string out;
    out.reserve(128);
    out += "# acs_schema_version=" + std::to_string(kLogSchemaVersion) + "\n";
    out += "# firmware_git_hash=";
    out += kExpectedFirmwareGitHash;
    out += "\n";
    out += "# brand=ACS NDRT Rocketry\n";
    out += "# source_format=acs_ndrt_rocketry_teensy_sensorlog_v3\n";
    return out;
}

/// Parses a raw ACS binary log into telemetry/event record references.
///
/// The parser validates the log preamble when present, records schema warnings,
/// and leaves payload bytes in-place so downstream code can decode fields on
/// demand without copying the whole file again.
bool ParseLog(const std::vector<uint8_t> &data,
              std::vector<TelemetryRecordRef> &telemetry,
              std::vector<EventRecord> &events,
              std::unordered_map<std::string, std::string> *outMetadata,
              std::vector<std::string> *outSchemaWarnings,
              std::string &error) {
    telemetry.clear();
    events.clear();

    std::size_t off = 0;
    if (outMetadata != nullptr) {
        outMetadata->clear();
    }
    if (outSchemaWarnings != nullptr) {
        outSchemaWarnings->clear();
    }

    if (data.size() >= kLogPreambleSize) {
        const LogPreamble preamble = ReadLE<LogPreamble>(data.data());
        bool magicOk = true;
        for (std::size_t i = 0; i < kLogMagic.size(); ++i) {
            if (preamble.magic[i] != kLogMagic[i]) {
                magicOk = false;
                break;
            }
        }
        if (magicOk) {
            off = kLogPreambleSize;
            if (outMetadata != nullptr) {
                (*outMetadata)["format_version"] = std::to_string(static_cast<unsigned>(preamble.formatVersion));
                (*outMetadata)["acs_schema_version"] = std::to_string(static_cast<unsigned>(preamble.schemaVersion));
                (*outMetadata)["firmware_git_hash"] = std::string(preamble.firmwareGitHash);
                (*outMetadata)["source_format"] = "acs_ndrt_rocketry_teensy_sensorlog_v3";
            }
            if (outSchemaWarnings != nullptr) {
                if (preamble.schemaVersion != static_cast<uint16_t>(kLogSchemaVersion)) {
                    outSchemaWarnings->push_back("Schema version mismatch: file=" +
                                                 std::to_string(static_cast<unsigned>(preamble.schemaVersion)) +
                                                 " decoder=" + std::to_string(kLogSchemaVersion));
                }
                if (std::string(kExpectedFirmwareGitHash) != "unknown") {
                    if (std::string(preamble.firmwareGitHash) != std::string(kExpectedFirmwareGitHash)) {
                        outSchemaWarnings->push_back("Firmware git hash mismatch: file=" +
                                                     std::string(preamble.firmwareGitHash) +
                                                     " expected=" + std::string(kExpectedFirmwareGitHash));
                    }
                }
            }
        } else if (outSchemaWarnings != nullptr) {
            outSchemaWarnings->push_back("Legacy BIN format (no preamble). Validation limited.");
        }
    } else if (outSchemaWarnings != nullptr) {
        outSchemaWarnings->push_back("Legacy BIN format (no preamble). Validation limited.");
    }

    const std::size_t size = data.size();
    while (off + kHeaderSize <= size) {
        const uint8_t recordType = data[off + 0];
        const uint8_t subtype = data[off + 1];
        const uint8_t flags = data[off + 2];
        off += kHeaderSize;

        if (recordType == 0u) {
            if (off + kTelemetryPayloadSize > size) {
                error = "Truncated telemetry record.";
                return false;
            }
            TelemetryRecordRef ref;
            ref.status = subtype;
            ref.flags = flags;
            ref.payload = data.data() + off;
            telemetry.push_back(ref);
            off += kTelemetryPayloadSize;
        } else if (recordType == 1u) {
            if (off + kEventPayloadSize > size) {
                error = "Truncated event record.";
                return false;
            }
            const uint8_t *p = data.data() + off;
            EventRecord ev;
            ev.eventType = subtype;
            ev.flightStatus = flags;
            ev.timestamp = ReadLE<float>(p + 0);
            ev.altitudeAglFeet = ReadLE<float>(p + 4);
            ev.verticalVelocityFps = ReadLE<float>(p + 8);
            ev.apogeeEstimateFeet = ReadLE<float>(p + 12);
            ev.flapCommandDeg = ReadLE<float>(p + 16);
            ev.flapEffectiveDeg = ReadLE<float>(p + 20);
            events.push_back(ev);
            off += kEventPayloadSize;
        } else {
            error = "Unknown record type " + std::to_string(recordType) + ".";
            return false;
        }
    }

    if (off != size) {
        error = "Trailing bytes at end of file.";
        return false;
    }
    return true;
}

bool LoadFile(const fs::path &path, std::vector<uint8_t> &bytes, std::string &error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        error = "Failed to open input: " + path.string();
        return false;
    }

    in.seekg(0, std::ios::end);
    const std::streamoff s = in.tellg();
    if (s < 0) {
        error = "Failed to stat input size.";
        return false;
    }

    bytes.resize(static_cast<std::size_t>(s));
    in.seekg(0, std::ios::beg);
    if (!bytes.empty()) {
        in.read(reinterpret_cast<char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    }
    if (!in && !in.eof()) {
        error = "Failed reading input.";
        return false;
    }
    return true;
}

bool WriteEventsJson(const fs::path &eventsPath, const std::vector<EventRecord> &events, std::string &error) {
    std::ofstream out(eventsPath, std::ios::binary);
    if (!out) {
        error = "Failed to open events output: " + eventsPath.string();
        return false;
    }
    if (events.empty()) {
        out << "[]";
        return true;
    }

    out << "[\n";
    for (std::size_t i = 0; i < events.size(); ++i) {
        const auto &ev = events[i];
        out << "  {\"event_type\":\"" << EventTypeName(ev.eventType) << "\"," << "\"event_type_raw\":"
            << static_cast<int>(ev.eventType) << "," << "\"flight_status\":\"" << FlightStatusName(ev.flightStatus)
            << "\"," << "\"flight_status_raw\":" << static_cast<int>(ev.flightStatus) << "," << "\"timestamp\":"
            << ev.timestamp << "," << "\"altitude_agl_feet\":" << ev.altitudeAglFeet << ","
            << "\"vertical_velocity_fps\":" << ev.verticalVelocityFps << ","
            << "\"apogee_estimate_feet\":" << ev.apogeeEstimateFeet << ","
            << "\"flap_command_deg\":" << ev.flapCommandDeg << ","
            << "\"flap_effective_deg\":" << ev.flapEffectiveDeg << "}";
        if (i + 1 < events.size()) {
            out << ",";
        }
        out << "\n";
    }
    out << "]\n";
    return true;
}

bool ConcatenateFiles(const std::vector<fs::path> &parts, const fs::path &outPath, std::string &error) {
    std::ofstream out(outPath, std::ios::binary);
    if (!out) {
        error = "Failed to open CSV output: " + outPath.string();
        return false;
    }

    const std::string header = TelemetryHeader();
    const std::string preamble = TelemetryMetadataPreamble();
    out.write(preamble.data(), static_cast<std::streamsize>(preamble.size()));
    out.write(header.data(), static_cast<std::streamsize>(header.size()));

    std::vector<char> buffer(1 << 20);
    for (const auto &part : parts) {
        std::ifstream in(part, std::ios::binary);
        if (!in) {
            error = "Failed to read temp part: " + part.string();
            return false;
        }
        while (in) {
            in.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
            const std::streamsize got = in.gcount();
            if (got > 0) {
                out.write(buffer.data(), got);
            }
        }
    }
    return true;
}

bool WriteTelemetryCsvParallel(const fs::path &outPath,
                               const std::vector<TelemetryRecordRef> &records,
                               unsigned threadCount,
                               std::string &error) {
    if (records.empty()) {
        std::ofstream out(outPath, std::ios::binary);
        if (!out) {
            error = "Failed to open CSV output: " + outPath.string();
            return false;
        }
        const std::string header = TelemetryHeader();
        const std::string preamble = TelemetryMetadataPreamble();
        out.write(preamble.data(), static_cast<std::streamsize>(preamble.size()));
        out.write(header.data(), static_cast<std::streamsize>(header.size()));
        return true;
    }

    threadCount = std::max(1u, threadCount);
    threadCount = std::min<unsigned>(threadCount, static_cast<unsigned>(records.size()));

    const std::size_t chunkSize = (records.size() + threadCount - 1) / threadCount;
    const fs::path tmpDir = fs::temp_directory_path();
    const auto pid = static_cast<unsigned long>(::getpid());

    std::vector<fs::path> partFiles(threadCount);
    for (unsigned i = 0; i < threadCount; ++i) {
        partFiles[i] = tmpDir / ("acs_decode_" + std::to_string(pid) + "_" + std::to_string(i) + ".part");
    }

    std::atomic<bool> ok{true};
    std::vector<std::thread> workers;
    workers.reserve(threadCount);
    for (unsigned tid = 0; tid < threadCount; ++tid) {
        const std::size_t begin = static_cast<std::size_t>(tid) * chunkSize;
        const std::size_t end = std::min(records.size(), begin + chunkSize);
        workers.emplace_back([&, tid, begin, end]() {
            std::ofstream part(partFiles[tid], std::ios::binary);
            if (!part) {
                ok.store(false);
                return;
            }
            std::string line;
            for (std::size_t i = begin; i < end; ++i) {
                BuildTelemetryLine(records[i], line);
                part.write(line.data(), static_cast<std::streamsize>(line.size()));
                if (!part) {
                    ok.store(false);
                    return;
                }
            }
        });
    }
    for (auto &t : workers) {
        t.join();
    }

    if (!ok.load()) {
        error = "Parallel CSV formatting failed.";
        for (const auto &p : partFiles) {
            std::error_code ec;
            fs::remove(p, ec);
        }
        return false;
    }

    const bool merged = ConcatenateFiles(partFiles, outPath, error);
    for (const auto &p : partFiles) {
        std::error_code ec;
        fs::remove(p, ec);
    }
    return merged;
}

float SensorFloat(const TelemetryRecordRef &rec, int sensorFloatIndex) {
    return ReadLE<float>(rec.payload + static_cast<std::size_t>(sensorFloatIndex) * sizeof(float));
}

bool SensorBool(const TelemetryRecordRef &rec, int sensorBoolIndex) {
    const std::size_t boolOffset =
        static_cast<std::size_t>(kSensorFloatCount) * sizeof(float) + static_cast<std::size_t>(kSensorU8Count);
    return rec.payload[boolOffset + static_cast<std::size_t>(sensorBoolIndex)] != 0u;
}

bool HasFilteredState(const TelemetryRecordRef &rec) {
    return (rec.flags & 0x01u) != 0u;
}

std::size_t EstimateRowsForSeconds(const std::vector<TelemetryRecordRef> &records, float seconds) {
    if (records.size() < 3 || seconds <= 0.0f) {
        return 0;
    }
    const std::size_t sampleCount = std::min<std::size_t>(records.size() - 1, 5000);
    double sumDt = 0.0;
    std::size_t validDtCount = 0;
    float prevT = SensorFloat(records[0], 0);
    for (std::size_t i = 1; i <= sampleCount; ++i) {
        const float t = SensorFloat(records[i], 0);
        const float dt = t - prevT;
        if (std::isfinite(dt) && dt > 0.00001f && dt < 1.0f) {
            sumDt += static_cast<double>(dt);
            validDtCount++;
        }
        prevT = t;
    }
    float hz = 200.0f;
    if (validDtCount > 0) {
        const double avgDt = sumDt / static_cast<double>(validDtCount);
        if (avgDt > 0.0) {
            hz = static_cast<float>(1.0 / avgDt);
            hz = std::max(10.0f, std::min(hz, 5000.0f));
        }
    }
    return static_cast<std::size_t>(std::max(0.0f, seconds * hz));
}

bool ComputeSmartActiveWindow(const std::vector<TelemetryRecordRef> &records,
                              float minAltitudeDeltaFt,
                              float minVelocityFtPerSec,
                              float minCommandDeg,
                              std::size_t preRows,
                              std::size_t postRows,
                              std::size_t &outBegin,
                              std::size_t &outEnd) {
    outBegin = 0;
    outEnd = records.size();
    if (records.empty()) {
        return false;
    }

    std::vector<float> groundAltitudes;
    groundAltitudes.reserve(512);
    const std::size_t baselineScan = std::min<std::size_t>(records.size(), 3000);
    for (std::size_t i = 0; i < baselineScan; ++i) {
        if (records[i].status == 0u) {
            const float alt = SensorFloat(records[i], 1);
            if (std::isfinite(alt)) {
                groundAltitudes.push_back(alt);
            }
        }
    }
    if (groundAltitudes.empty()) {
        for (std::size_t i = 0; i < std::min<std::size_t>(records.size(), 300); ++i) {
            const float alt = SensorFloat(records[i], 1);
            if (std::isfinite(alt)) {
                groundAltitudes.push_back(alt);
            }
        }
    }

    float groundAltitude = 0.0f;
    if (!groundAltitudes.empty()) {
        double sum = 0.0;
        for (float v : groundAltitudes) {
            sum += static_cast<double>(v);
        }
        groundAltitude = static_cast<float>(sum / static_cast<double>(groundAltitudes.size()));
    }

    bool anyActive = false;
    std::size_t firstActive = 0;
    std::size_t lastActive = 0;
    for (std::size_t i = 0; i < records.size(); ++i) {
        const TelemetryRecordRef &r = records[i];
        const float alt = SensorFloat(r, 1);
        const float cmdDeg = SensorFloat(r, 26);
        float velZ = 0.0f;
        bool hasVel = false;
        if (HasFilteredState(r)) {
            velZ = SensorFloat(r, 29);
            hasVel = std::isfinite(velZ);
        }

        const bool statusActive = r.status != 0u;
        const bool altitudeActive = std::isfinite(alt) && std::fabs(alt - groundAltitude) >= minAltitudeDeltaFt;
        const bool velocityActive = hasVel && std::fabs(velZ) >= minVelocityFtPerSec;
        const bool cmdActive = std::isfinite(cmdDeg) && std::fabs(cmdDeg) >= minCommandDeg;
        const bool settlingActive = SensorBool(r, 5);
        const bool active = statusActive || altitudeActive || velocityActive || cmdActive || settlingActive;
        if (!active) {
            continue;
        }
        if (!anyActive) {
            anyActive = true;
            firstActive = i;
        }
        lastActive = i;
    }

    if (!anyActive) {
        return false;
    }

    outBegin = (firstActive > preRows) ? (firstActive - preRows) : 0;
    outEnd = std::min(records.size(), lastActive + postRows + 1);
    return outBegin < outEnd;
}

[[maybe_unused]] bool EndsWithCaseInsensitive(const std::string &value, const std::string &suffix) {
    if (value.size() < suffix.size()) {
        return false;
    }
    const std::size_t start = value.size() - suffix.size();
    for (std::size_t i = 0; i < suffix.size(); ++i) {
        char a = value[start + i];
        char b = suffix[i];
        if (a >= 'A' && a <= 'Z') {
            a = static_cast<char>(a - 'A' + 'a');
        }
        if (b >= 'A' && b <= 'Z') {
            b = static_cast<char>(b - 'A' + 'a');
        }
        if (a != b) {
            return false;
        }
    }
    return true;
}

double ComputeSeededAngularRate(float currentTimeSeconds,
                                float currentZenithRadians,
                                float previousTimeSeconds,
                                float previousZenithRadians,
                                bool hasPreviousZenith) {
    if (!hasPreviousZenith) {
        return 0.0;
    }
    const double dt = static_cast<double>(currentTimeSeconds) - static_cast<double>(previousTimeSeconds);
    if (dt <= 0.0) {
        return 0.0;
    }
    return (static_cast<double>(currentZenithRadians) - static_cast<double>(previousZenithRadians)) / dt;
}

#if defined(ACS_ENABLE_IMGUI_DECODER)

constexpr float kFeetToMeters = 0.3048f;
constexpr float kMetersToFeet = 3.280839895013123f;
constexpr float kDegToRad = 0.01745329251994329577f;
constexpr float kRadToDeg = 57.29577951308232f;

struct NumericSeries {
    std::string name;
    std::vector<float> values;
    bool selected = false;
};

struct TelemetryTable {
    std::vector<float> timeSeconds;
    std::vector<NumericSeries> series;
    bool timeSorted = true;

    std::size_t RowCount() const { return timeSeconds.size(); }
};

const NumericSeries *FindSeriesNamed(const TelemetryTable &table, const char *name) {
    for (const auto &series : table.series) {
        if (series.name == name) {
            return &series;
        }
    }
    return nullptr;
}

struct SeriesLookup {
    const NumericSeries *series = nullptr;
    float scale = 1.0f;
};

SeriesLookup FindSeriesByNamesWithScale(
    const TelemetryTable &table,
    std::initializer_list<std::pair<const char *, float>> candidates) {
    for (const auto &[name, scale] : candidates) {
        if (const NumericSeries *series = FindSeriesNamed(table, name); series != nullptr) {
            return {series, scale};
        }
    }
    return {};
}

void AppendScaledAliasSeries(TelemetryTable &table, const char *sourceName, const char *aliasName, float scale = 1.0f) {
    if (FindSeriesNamed(table, aliasName) != nullptr) {
        return;
    }
    const NumericSeries *source = FindSeriesNamed(table, sourceName);
    if (source == nullptr) {
        return;
    }

    NumericSeries alias;
    alias.name = aliasName;
    alias.values = source->values;
    if (scale != 1.0f) {
        for (float &value : alias.values) {
            if (std::isfinite(value)) {
                value *= scale;
            }
        }
    }
    table.series.push_back(std::move(alias));
}

void AddLegacyCompatibilityAliases(TelemetryTable &table) {
    AppendScaledAliasSeries(table, "sensor_timestamp", "state_time");
    AppendScaledAliasSeries(table, "state_altitude_agl_feet", "state_position_z", kFeetToMeters);
    AppendScaledAliasSeries(table, "state_vertical_velocity_fps", "state_velocity_z", kFeetToMeters);
    AppendScaledAliasSeries(table, "state_zenith_deg", "state_zenith", kDegToRad);
    AppendScaledAliasSeries(table, "state_apogee_estimate_feet", "state_apogee_estimate", kFeetToMeters);
    AppendScaledAliasSeries(table, "sensor_flap_command_deg", "sensor_auto_cmd_deg");
    AppendScaledAliasSeries(table, "state_apogee_estimate_feet", "sensor_optimizer_best_predicted_apogee_m", kFeetToMeters);
}

struct LoadedDataset {
    fs::path sourcePath;
    std::string sourceType;
    TelemetryTable table;
    std::vector<EventRecord> events;
    bool hasRawCsv = false;
    std::string rawCsvText;
    std::vector<std::pair<std::size_t, std::size_t>> rawCsvLineRanges;
    std::vector<std::string> rawCsvHeaders;
    std::size_t rawDataStartLine = 1;
    std::unordered_map<std::string, std::string> metadata;
    std::vector<std::string> schemaWarnings;
    std::vector<std::pair<float, std::size_t>> timeIndex;
};

struct ReplaySettings {
    char cfdPath[256] = "lib/cfd.csv";
    bool useCfd = true;
    float refreshIntervalSeconds = 0.1f;
    int maxPredictorSteps = settings::actuation::kActuationPredictorMaxSteps;
    bool showLoggedStateApogee = true;
    bool showLoggedOptimizerApogee = true;
    bool showReplaySaferApogee = true;
    bool showReplayBallisticApogee = true;
    bool showReplayErrorVsActual = true;
    bool showReplayErrorVsState = false;
    bool showReplayErrorVsOptimizer = false;
    bool showSeedHorizontalSpeed = true;
    bool showSeedZenith = true;
    bool showSeedAngularRate = true;
    bool showRawInspector = true;
    bool normalizeSeedPlots = false;
    bool normalizeErrorPlots = false;
};

struct ReplayAnalysis {
    TelemetryTable table;
    std::string statusText;
    std::vector<std::string> warnings;
    bool ready = false;
    float actualApogeeMeters = std::numeric_limits<float>::quiet_NaN();
    float replayApogeeMinMeters = std::numeric_limits<float>::quiet_NaN();
    float replayApogeeMaxMeters = std::numeric_limits<float>::quiet_NaN();
    float replayApogeeMeanMeters = std::numeric_limits<float>::quiet_NaN();
    float replayBallisticMeanMeters = std::numeric_limits<float>::quiet_NaN();
};

struct NativeCfdTableStorage {
    ApogeeForceTable table;
    std::vector<double> acs;
    std::vector<double> atk;
    std::vector<double> mach;
    std::vector<double> axial;
    std::vector<double> normal;
    bool loaded = false;
};

struct CsvCellSpan {
    const char *begin = nullptr;
    const char *end = nullptr;
};

std::string ToLower(std::string value) {
    for (char &ch : value) {
        if (ch >= 'A' && ch <= 'Z') {
            ch = static_cast<char>(ch - 'A' + 'a');
        }
    }
    return value;
}

void TrimToken(const char *&begin, const char *&end) {
    while (begin < end && (*begin == ' ' || *begin == '\t')) {
        ++begin;
    }
    while (end > begin && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r')) {
        --end;
    }
}

bool ParseNumericToken(const char *begin, const char *end, float &outValue) {
    TrimToken(begin, end);
    if (begin >= end) {
        return false;
    }

    const std::string lowered = ToLower(std::string(begin, end));
    if (lowered == "true") {
        outValue = 1.0f;
        return true;
    }
    if (lowered == "false") {
        outValue = 0.0f;
        return true;
    }

#if defined(__cpp_lib_to_chars) && (__cpp_lib_to_chars >= 201611L)
    float parsed = std::numeric_limits<float>::quiet_NaN();
    const std::from_chars_result fc = std::from_chars(begin, end, parsed);
    if (fc.ec == std::errc() && fc.ptr == end) {
        outValue = parsed;
        return true;
    }
#endif

    char stackBuf[128];
    char *buf = stackBuf;
    std::string heapBuf;
    const std::size_t len = static_cast<std::size_t>(end - begin);
    if (len + 1 > sizeof(stackBuf)) {
        heapBuf.assign(begin, end);
        buf = heapBuf.data();
    } else {
        std::memcpy(stackBuf, begin, len);
        stackBuf[len] = '\0';
    }

    errno = 0;
    char *tail = nullptr;
    const float parsed = std::strtof(buf, &tail);
    if (errno != 0 || tail == buf || *tail != '\0') {
        return false;
    }
    outValue = parsed;
    return true;
}

std::vector<std::string> SplitCsvHeader(const std::string &line) {
    std::vector<std::string> out;
    std::size_t start = 0;
    while (start <= line.size()) {
        std::size_t comma = line.find(',', start);
        if (comma == std::string::npos) {
            comma = line.size();
        }
        std::string token = line.substr(start, comma - start);
        while (!token.empty() && (token.back() == '\r' || token.back() == ' ' || token.back() == '\t')) {
            token.pop_back();
        }
        while (!token.empty() && (token.front() == ' ' || token.front() == '\t')) {
            token.erase(token.begin());
        }
        out.push_back(token);
        if (comma == line.size()) {
            break;
        }
        start = comma + 1;
    }
    return out;
}

std::size_t FindTimeColumn(const std::vector<std::string> &headers) {
    for (std::size_t i = 0; i < headers.size(); ++i) {
        if (headers[i] == "sensor_timestamp") {
            return i;
        }
    }
    for (std::size_t i = 0; i < headers.size(); ++i) {
        if (headers[i] == "state_time") {
            return i;
        }
    }
    for (std::size_t i = 0; i < headers.size(); ++i) {
        const std::string lowered = ToLower(headers[i]);
        if (lowered.find("time") != std::string::npos) {
            return i;
        }
    }
    return std::numeric_limits<std::size_t>::max();
}

bool IsMonotonicNonDecreasing(const std::vector<float> &values) {
    float prev = -std::numeric_limits<float>::infinity();
    for (float value : values) {
        if (!std::isfinite(value)) {
            return false;
        }
        if (value < prev) {
            return false;
        }
        prev = value;
    }
    return true;
}

bool ParseCsvTelemetry(const fs::path &csvPath,
                       unsigned threadCount,
                       TelemetryTable &table,
                       std::string &error,
                       std::string *outRawText = nullptr,
                       std::vector<std::pair<std::size_t, std::size_t>> *outRawLineRanges = nullptr,
                       std::vector<std::string> *outRawHeaders = nullptr,
                       std::unordered_map<std::string, std::string> *outMetadata = nullptr,
                       std::vector<std::string> *outSchemaWarnings = nullptr,
                       std::size_t *outRawDataStartLine = nullptr) {
    std::vector<uint8_t> bytes;
    if (!LoadFile(csvPath, bytes, error)) {
        return false;
    }
    if (bytes.empty()) {
        error = "CSV file is empty.";
        return false;
    }

    std::string text;
    text.assign(reinterpret_cast<const char *>(bytes.data()), bytes.size());

    std::vector<std::pair<std::size_t, std::size_t>> lines;
    lines.reserve(1 + bytes.size() / 80);
    std::size_t lineStart = 0;
    for (std::size_t i = 0; i < text.size(); ++i) {
        if (text[i] == '\n') {
            lines.push_back({lineStart, i});
            lineStart = i + 1;
        }
    }
    if (lineStart < text.size()) {
        lines.push_back({lineStart, text.size()});
    }

    if (lines.empty()) {
        error = "CSV has no lines.";
        return false;
    }

    std::unordered_map<std::string, std::string> metadata;
    std::vector<std::string> schemaWarnings;

    std::size_t headerLineIndex = 0;
    while (headerLineIndex < lines.size()) {
        const auto &span = lines[headerLineIndex];
        const char *lb = text.data() + span.first;
        const char *le = text.data() + span.second;
        while (lb < le && (*lb == ' ' || *lb == '\t')) {
            ++lb;
        }
        if (lb >= le || *lb != '#') {
            break;
        }
        ++lb;
        while (lb < le && (*lb == ' ' || *lb == '\t')) {
            ++lb;
        }
        const char *eq = lb;
        while (eq < le && *eq != '=') {
            ++eq;
        }
        if (eq < le) {
            const std::string key(lb, eq);
            const std::string value(eq + 1, le);
            metadata[key] = value;
        }
        headerLineIndex++;
    }
    if (headerLineIndex >= lines.size()) {
        error = "CSV header not found after metadata lines.";
        return false;
    }

    const std::string headerLine = text.substr(lines[headerLineIndex].first, lines[headerLineIndex].second - lines[headerLineIndex].first);
    const std::vector<std::string> headers = SplitCsvHeader(headerLine);
    if (headers.empty()) {
        error = "CSV header has no columns.";
        return false;
    }

    if (metadata.empty()) {
        schemaWarnings.push_back("No metadata preamble found (# acs_schema_version / # firmware_git_hash).");
    } else {
        const auto itSchema = metadata.find("acs_schema_version");
        if (itSchema == metadata.end()) {
            schemaWarnings.push_back("Missing metadata key: acs_schema_version");
        } else if (std::atoi(itSchema->second.c_str()) != kLogSchemaVersion) {
            schemaWarnings.push_back("Schema version mismatch: file=" + itSchema->second +
                                     " decoder=" + std::to_string(kLogSchemaVersion));
        }
        const auto itHash = metadata.find("firmware_git_hash");
        if (itHash == metadata.end()) {
            schemaWarnings.push_back("Missing metadata key: firmware_git_hash");
        } else if (std::string(kExpectedFirmwareGitHash) != "unknown" && itHash->second != kExpectedFirmwareGitHash) {
            schemaWarnings.push_back("Firmware git hash mismatch: file=" + itHash->second +
                                     " expected=" + std::string(kExpectedFirmwareGitHash));
        }
    }

    const std::size_t rowStartIndex = headerLineIndex + 1;
    if (outRawDataStartLine != nullptr) {
        *outRawDataStartLine = rowStartIndex;
    }
    const std::size_t rowCount = lines.size() > rowStartIndex ? lines.size() - rowStartIndex : 0;
    if (rowCount == 0) {
        table = TelemetryTable{};
        if (outMetadata != nullptr) {
            *outMetadata = std::move(metadata);
        }
        if (outSchemaWarnings != nullptr) {
            *outSchemaWarnings = std::move(schemaWarnings);
        }
        return true;
    }

    const std::size_t colCount = headers.size();
    std::vector<std::vector<float>> columnData(colCount);
    for (auto &column : columnData) {
        column.assign(rowCount, std::numeric_limits<float>::quiet_NaN());
    }

    threadCount = std::max(1u, threadCount);
    threadCount = std::min<unsigned>(threadCount, static_cast<unsigned>(rowCount));
    const std::size_t chunk = (rowCount + threadCount - 1) / threadCount;

    std::vector<std::vector<std::uint64_t>> numericCounts(threadCount, std::vector<std::uint64_t>(colCount, 0));
    std::atomic<bool> ok{true};

    std::vector<std::thread> workers;
    workers.reserve(threadCount);
    for (unsigned tid = 0; tid < threadCount; ++tid) {
        const std::size_t beginRow = static_cast<std::size_t>(tid) * chunk;
        const std::size_t endRow = std::min(rowCount, beginRow + chunk);
        workers.emplace_back([&, tid, beginRow, endRow]() {
            for (std::size_t localRow = beginRow; localRow < endRow; ++localRow) {
                const auto [lineBeginIdx, lineEndIdx] = lines[rowStartIndex + localRow];
                const char *lineBegin = text.data() + lineBeginIdx;
                const char *lineEnd = text.data() + lineEndIdx;

                std::size_t col = 0;
                const char *tokenStart = lineBegin;
                for (const char *p = lineBegin; p <= lineEnd; ++p) {
                    const bool atEnd = (p == lineEnd);
                    if (!atEnd && *p != ',') {
                        continue;
                    }
                    if (col < colCount) {
                        float parsed = std::numeric_limits<float>::quiet_NaN();
                        if (ParseNumericToken(tokenStart, p, parsed)) {
                            columnData[col][localRow] = parsed;
                            numericCounts[tid][col]++;
                        }
                    }
                    tokenStart = p + 1;
                    ++col;
                    if (atEnd) {
                        break;
                    }
                }
            }
        });
    }

    for (auto &worker : workers) {
        worker.join();
    }
    if (!ok.load()) {
        error = "CSV parsing failed.";
        return false;
    }

    std::vector<std::uint64_t> totalNumeric(colCount, 0);
    for (unsigned tid = 0; tid < threadCount; ++tid) {
        for (std::size_t col = 0; col < colCount; ++col) {
            totalNumeric[col] += numericCounts[tid][col];
        }
    }

    table = TelemetryTable{};
    table.series.reserve(colCount);
    for (std::size_t col = 0; col < colCount; ++col) {
        if (totalNumeric[col] == 0) {
            continue;
        }
        NumericSeries s;
        s.name = headers[col];
        s.values = std::move(columnData[col]);
        table.series.push_back(std::move(s));
    }

    if (table.series.empty()) {
        error = "CSV has no numeric columns.";
        return false;
    }

    std::size_t timeSeriesIndex = std::numeric_limits<std::size_t>::max();
    const std::size_t headerTimeIndex = FindTimeColumn(headers);
    if (headerTimeIndex != std::numeric_limits<std::size_t>::max()) {
        const std::string wantedName = headers[headerTimeIndex];
        for (std::size_t i = 0; i < table.series.size(); ++i) {
            if (table.series[i].name == wantedName) {
                timeSeriesIndex = i;
                break;
            }
        }
    }

    if (timeSeriesIndex != std::numeric_limits<std::size_t>::max()) {
        table.timeSeconds = table.series[timeSeriesIndex].values;
    } else {
        table.timeSeconds.resize(rowCount);
        for (std::size_t i = 0; i < rowCount; ++i) {
            table.timeSeconds[i] = static_cast<float>(i) * 0.01f;
        }
    }

    // Repair invalid timestamps in place to keep plotting/indexing stable.
    float current = 0.0f;
    for (std::size_t i = 0; i < table.timeSeconds.size(); ++i) {
        float t = table.timeSeconds[i];
        if (!std::isfinite(t) || (i > 0 && t < current)) {
            t = current + (i == 0 ? 0.0f : 0.001f);
        }
        table.timeSeconds[i] = t;
        current = t;
    }
    table.timeSorted = IsMonotonicNonDecreasing(table.timeSeconds);

    if (outRawText != nullptr) {
        *outRawText = std::move(text);
    }
    if (outRawLineRanges != nullptr) {
        *outRawLineRanges = std::move(lines);
    }
    if (outRawHeaders != nullptr) {
        *outRawHeaders = headers;
    }
    if (outMetadata != nullptr) {
        *outMetadata = std::move(metadata);
    }
    if (outSchemaWarnings != nullptr) {
        *outSchemaWarnings = std::move(schemaWarnings);
    }
    AddLegacyCompatibilityAliases(table);
    return true;
}

TelemetryTable BuildTelemetryTableFromBin(const std::vector<TelemetryRecordRef> &records) {
    TelemetryTable table;
    const std::size_t rows = records.size();

    table.series.resize(kTelemetryFields.size());
    for (std::size_t i = 0; i < kTelemetryFields.size(); ++i) {
        table.series[i].name = kTelemetryFields[i].name;
        table.series[i].values.assign(rows, std::numeric_limits<float>::quiet_NaN());
    }
    table.timeSeconds.assign(rows, 0.0f);

    for (std::size_t row = 0; row < rows; ++row) {
        const TelemetryRecordRef &rec = records[row];
        for (std::size_t col = 0; col < kTelemetryFields.size(); ++col) {
            float value = std::numeric_limits<float>::quiet_NaN();
            if (ReadTelemetryFieldValue(rec, kTelemetryFields[col], value)) {
                table.series[col].values[row] = value;
            }
        }

        table.timeSeconds[row] = table.series[2].values[row];
    }

    float prev = 0.0f;
    for (std::size_t i = 0; i < table.timeSeconds.size(); ++i) {
        float t = table.timeSeconds[i];
        if (!std::isfinite(t) || (i > 0 && t < prev)) {
            t = prev + (i == 0 ? 0.0f : 0.001f);
        }
        table.timeSeconds[i] = t;
        prev = t;
    }
    table.timeSorted = true;
    AddLegacyCompatibilityAliases(table);

    return table;
}

const NumericSeries *FindSeriesConst(const TelemetryTable &table, const char *name) {
    for (const auto &series : table.series) {
        if (series.name == name) {
            return &series;
        }
    }
    return nullptr;
}

std::optional<std::size_t> FindSeriesIndex(const TelemetryTable &table, const char *name) {
    for (std::size_t i = 0; i < table.series.size(); ++i) {
        if (table.series[i].name == name) {
            return i;
        }
    }
    return std::nullopt;
}

void AppendDerivedSeries(TelemetryTable &table, const std::string &name, std::vector<float> values) {
    NumericSeries s;
    s.name = name;
    s.values = std::move(values);
    s.selected = false;
    table.series.push_back(std::move(s));
}

void AddDerivedChannels(TelemetryTable &table) {
    const std::size_t n = table.RowCount();
    if (n == 0) {
        return;
    }

    const auto vxIdx = FindSeriesIndex(table, "state_velocity_x");
    const auto vyIdx = FindSeriesIndex(table, "state_velocity_y");
    const auto vzIdx = FindSeriesIndex(table, "state_velocity_z");
    const auto axIdx = FindSeriesIndex(table, "state_acceleration_x");
    const auto ayIdx = FindSeriesIndex(table, "state_acceleration_y");
    const auto azIdx = FindSeriesIndex(table, "state_acceleration_z");
    const auto altFeetIdx = FindSeriesIndex(table, "sensor_altitude_feet");
    const auto predApogeeMIdx = FindSeriesIndex(table, "sensor_optimizer_best_predicted_apogee_m");
    const auto stateApogeeIdx = FindSeriesIndex(table, "state_apogee_estimate");

    if (vxIdx.has_value() && vyIdx.has_value() && vzIdx.has_value()) {
        std::vector<float> speedMag(n, std::numeric_limits<float>::quiet_NaN());
        std::vector<float> mach(n, std::numeric_limits<float>::quiet_NaN());
        std::vector<float> qPa(n, std::numeric_limits<float>::quiet_NaN());
        for (std::size_t i = 0; i < n; ++i) {
            const float x = table.series[*vxIdx].values[i];
            const float y = table.series[*vyIdx].values[i];
            const float z = table.series[*vzIdx].values[i];
            if (!(std::isfinite(x) && std::isfinite(y) && std::isfinite(z))) {
                continue;
            }
            const float speed = std::sqrt(x * x + y * y + z * z);
            speedMag[i] = speed;
            mach[i] = speed / 343.0f;

            float rho = 1.225f;
            if (altFeetIdx.has_value() && std::isfinite(table.series[*altFeetIdx].values[i])) {
                const float altM = table.series[*altFeetIdx].values[i] * 0.3048f;
                rho = 1.225f * std::exp(-altM / 8500.0f);
            }
            qPa[i] = 0.5f * rho * speed * speed;
        }
        AppendDerivedSeries(table, "derived_speed_m_s", std::move(speedMag));
        AppendDerivedSeries(table, "derived_mach", std::move(mach));
        AppendDerivedSeries(table, "derived_dynamic_pressure_pa", std::move(qPa));
    }

    if (axIdx.has_value() && ayIdx.has_value() && azIdx.has_value()) {
        std::vector<float> accelMag(n, std::numeric_limits<float>::quiet_NaN());
        std::vector<float> jerkMag(n, std::numeric_limits<float>::quiet_NaN());
        for (std::size_t i = 0; i < n; ++i) {
            const float x = table.series[*axIdx].values[i];
            const float y = table.series[*ayIdx].values[i];
            const float z = table.series[*azIdx].values[i];
            if (!(std::isfinite(x) && std::isfinite(y) && std::isfinite(z))) {
                continue;
            }
            accelMag[i] = std::sqrt(x * x + y * y + z * z);
            if (i > 0) {
                const float dt = table.timeSeconds[i] - table.timeSeconds[i - 1];
                if (dt > 1e-5f && std::isfinite(accelMag[i - 1])) {
                    jerkMag[i] = (accelMag[i] - accelMag[i - 1]) / dt;
                }
            }
        }
        AppendDerivedSeries(table, "derived_accel_mag_m_s2", std::move(accelMag));
        AppendDerivedSeries(table, "derived_jerk_mag_m_s3", std::move(jerkMag));
    }

    if (predApogeeMIdx.has_value() && stateApogeeIdx.has_value()) {
        std::vector<float> error(n, std::numeric_limits<float>::quiet_NaN());
        for (std::size_t i = 0; i < n; ++i) {
            const float p = table.series[*predApogeeMIdx].values[i];
            const float s = table.series[*stateApogeeIdx].values[i];
            if (std::isfinite(p) && std::isfinite(s)) {
                error[i] = p - s;
            }
        }
        AppendDerivedSeries(table, "derived_prediction_error_raw", std::move(error));
    }

    if (const auto flagsIdx = FindSeriesIndex(table, "sensor_predictor_seed_confidence_flags"); flagsIdx.has_value()) {
        std::vector<float> controlActive(n, std::numeric_limits<float>::quiet_NaN());
        std::vector<float> positiveVz(n, std::numeric_limits<float>::quiet_NaN());
        std::vector<float> usingHorizontalModel(n, std::numeric_limits<float>::quiet_NaN());
        std::vector<float> speedCapped(n, std::numeric_limits<float>::quiet_NaN());
        std::vector<float> zenithClamped(n, std::numeric_limits<float>::quiet_NaN());
        std::vector<float> rateClamped(n, std::numeric_limits<float>::quiet_NaN());
        for (std::size_t i = 0; i < n; ++i) {
            const float rawFlags = table.series[*flagsIdx].values[i];
            if (!std::isfinite(rawFlags)) {
                continue;
            }
            const uint32_t flags = static_cast<uint32_t>(std::lround(rawFlags));
            controlActive[i] = (flags & kPredictorSeedFlagControlActive) ? 1.0f : 0.0f;
            positiveVz[i] = (flags & kPredictorSeedFlagPositiveVerticalVelocity) ? 1.0f : 0.0f;
            usingHorizontalModel[i] = (flags & kPredictorSeedFlagUsingHorizontalModel) ? 1.0f : 0.0f;
            speedCapped[i] = (flags & kPredictorSeedFlagHorizontalSpeedCapped) ? 1.0f : 0.0f;
            zenithClamped[i] = (flags & kPredictorSeedFlagZenithClamped) ? 1.0f : 0.0f;
            rateClamped[i] = (flags & kPredictorSeedFlagAngularRateClamped) ? 1.0f : 0.0f;
        }
        AppendDerivedSeries(table, "derived_predictor_conf_control_active", std::move(controlActive));
        AppendDerivedSeries(table, "derived_predictor_conf_positive_vertical_velocity", std::move(positiveVz));
        AppendDerivedSeries(table, "derived_predictor_conf_using_horizontal_model", std::move(usingHorizontalModel));
        AppendDerivedSeries(table, "derived_predictor_conf_horizontal_speed_capped", std::move(speedCapped));
        AppendDerivedSeries(table, "derived_predictor_conf_zenith_clamped", std::move(zenithClamped));
        AppendDerivedSeries(table, "derived_predictor_conf_angular_rate_clamped", std::move(rateClamped));
    }
}

void BuildTimeIndex(const TelemetryTable &table, std::vector<std::pair<float, std::size_t>> &index, std::size_t stepRows) {
    index.clear();
    if (table.timeSeconds.empty()) {
        return;
    }
    const std::size_t step = std::max<std::size_t>(1, stepRows);
    for (std::size_t i = 0; i < table.timeSeconds.size(); i += step) {
        index.emplace_back(table.timeSeconds[i], i);
    }
    if (index.empty() || index.back().second != table.timeSeconds.size() - 1) {
        index.emplace_back(table.timeSeconds.back(), table.timeSeconds.size() - 1);
    }
}

void WriteIdxSidecar(const fs::path &inputPath,
                     const TelemetryTable &table,
                     const std::vector<std::pair<float, std::size_t>> &index,
                     const std::vector<std::pair<std::size_t, std::size_t>> *rawLineRanges = nullptr,
                     std::size_t rawDataStartLine = 0) {
    if (index.empty()) {
        return;
    }
    std::error_code ec;
    const auto fileSize = fs::file_size(inputPath, ec);
    (void)fs::last_write_time(inputPath, ec);
    std::ofstream out(inputPath.string() + ".idx", std::ios::binary);
    if (!out) {
        return;
    }
    out << "# acs_idx_v1\n";
    out << "file=" << inputPath.filename().string() << "\n";
    out << "size_bytes=" << static_cast<unsigned long long>(fileSize) << "\n";
    out << "rows=" << table.RowCount() << "\n";
    out << "columns=time,row,byte_offset\n";
    for (const auto &entry : index) {
        std::size_t byteOffset = 0;
        if (rawLineRanges != nullptr && !rawLineRanges->empty()) {
            const std::size_t lineIndex = rawDataStartLine + entry.second;
            if (lineIndex < rawLineRanges->size()) {
                byteOffset = (*rawLineRanges)[lineIndex].first;
            }
        }
        out << entry.first << "," << entry.second << "," << byteOffset << "\n";
    }
}

struct SmartParserSettings {
    bool enabled = false;
    float preSeconds = 3.0f;
    float postSeconds = 10.0f;
    float minAltitudeDeltaFt = 25.0f;
    float minVelocityFtPerSec = 20.0f;
    float minCommandDeg = 0.5f;
};

struct SmartParserReport {
    bool applied = false;
    std::size_t originalRows = 0;
    std::size_t keptRows = 0;
    std::size_t beginRow = 0;
    std::size_t endRow = 0;
    float groundAltitudeFt = 0.0f;
};

std::size_t EstimateRowsForSecondsFromTable(const TelemetryTable &table, float seconds) {
    if (seconds <= 0.0f || table.timeSeconds.size() < 3) {
        return 0;
    }
    double sumDt = 0.0;
    std::size_t count = 0;
    const std::size_t n = std::min<std::size_t>(table.timeSeconds.size() - 1, 5000);
    for (std::size_t i = 1; i <= n; ++i) {
        const float dt = table.timeSeconds[i] - table.timeSeconds[i - 1];
        if (std::isfinite(dt) && dt > 0.00001f && dt < 1.0f) {
            sumDt += static_cast<double>(dt);
            count++;
        }
    }
    float hz = 200.0f;
    if (count > 0) {
        const double avgDt = sumDt / static_cast<double>(count);
        if (avgDt > 0.0) {
            hz = static_cast<float>(1.0 / avgDt);
            hz = std::max(10.0f, std::min(hz, 5000.0f));
        }
    }
    return static_cast<std::size_t>(std::max(0.0f, seconds * hz));
}

void TrimTelemetryTable(TelemetryTable &table, std::size_t beginRow, std::size_t endRow) {
    if (beginRow >= endRow || endRow > table.RowCount()) {
        return;
    }
    const std::size_t kept = endRow - beginRow;
    std::vector<float> newTime;
    newTime.reserve(kept);
    newTime.insert(newTime.end(), table.timeSeconds.begin() + static_cast<std::ptrdiff_t>(beginRow),
                   table.timeSeconds.begin() + static_cast<std::ptrdiff_t>(endRow));
    table.timeSeconds.swap(newTime);

    for (auto &series : table.series) {
        if (series.values.size() < endRow) {
            continue;
        }
        std::vector<float> keptVals;
        keptVals.reserve(kept);
        keptVals.insert(keptVals.end(), series.values.begin() + static_cast<std::ptrdiff_t>(beginRow),
                        series.values.begin() + static_cast<std::ptrdiff_t>(endRow));
        series.values.swap(keptVals);
    }
}

bool ApplySmartParserToDataset(LoadedDataset &dataset, const SmartParserSettings &settings, SmartParserReport &report) {
    report = SmartParserReport{};
    report.originalRows = dataset.table.RowCount();
    if (!settings.enabled || dataset.table.RowCount() < 10) {
        report.keptRows = report.originalRows;
        report.beginRow = 0;
        report.endRow = report.originalRows;
        return false;
    }

    const NumericSeries *status = FindSeriesConst(dataset.table, "flight_status_raw");
    const NumericSeries *alt = FindSeriesConst(dataset.table, "sensor_altitude_feet");
    const SeriesLookup vel = FindSeriesByNamesWithScale(
        dataset.table,
        {{"state_vertical_velocity_fps", 1.0f}, {"state_velocity_z", kMetersToFeet}});
    const NumericSeries *cmd = FindSeriesConst(dataset.table, "sensor_auto_cmd_deg");
    const NumericSeries *settling = FindSeriesConst(dataset.table, "sensor_actuation_is_settling");
    if (alt == nullptr) {
        report.keptRows = report.originalRows;
        report.beginRow = 0;
        report.endRow = report.originalRows;
        return false;
    }

    std::vector<float> groundSamples;
    groundSamples.reserve(256);
    if (status != nullptr && status->values.size() == alt->values.size()) {
        for (std::size_t i = 0; i < alt->values.size() && i < 5000; ++i) {
            const float s = status->values[i];
            const float a = alt->values[i];
            if (std::isfinite(s) && std::isfinite(a) && static_cast<int>(std::lround(s)) == 0) {
                groundSamples.push_back(a);
            }
        }
    }
    if (groundSamples.empty()) {
        for (std::size_t i = 0; i < std::min<std::size_t>(alt->values.size(), 300); ++i) {
            if (std::isfinite(alt->values[i])) {
                groundSamples.push_back(alt->values[i]);
            }
        }
    }
    float groundAlt = 0.0f;
    if (!groundSamples.empty()) {
        double sum = 0.0;
        for (float v : groundSamples) {
            sum += static_cast<double>(v);
        }
        groundAlt = static_cast<float>(sum / static_cast<double>(groundSamples.size()));
    }
    report.groundAltitudeFt = groundAlt;

    bool any = false;
    std::size_t first = 0;
    std::size_t last = 0;
    for (std::size_t i = 0; i < dataset.table.RowCount(); ++i) {
        const float altitude = alt->values[i];
        const float statusVal = (status != nullptr && i < status->values.size()) ? status->values[i] : 0.0f;
        const float velVal = (vel.series != nullptr && i < vel.series->values.size())
                                 ? (vel.series->values[i] * vel.scale)
                                 : 0.0f;
        const float cmdVal = (cmd != nullptr && i < cmd->values.size()) ? cmd->values[i] : 0.0f;
        const float settlingVal = (settling != nullptr && i < settling->values.size()) ? settling->values[i] : 0.0f;

        const bool statusActive = std::isfinite(statusVal) && static_cast<int>(std::lround(statusVal)) != 0;
        const bool altActive = std::isfinite(altitude) && std::fabs(altitude - groundAlt) >= settings.minAltitudeDeltaFt;
        const bool velActive = std::isfinite(velVal) && std::fabs(velVal) >= settings.minVelocityFtPerSec;
        const bool cmdActive = std::isfinite(cmdVal) && std::fabs(cmdVal) >= settings.minCommandDeg;
        const bool settlingActive = std::isfinite(settlingVal) && settlingVal > 0.5f;
        if (!(statusActive || altActive || velActive || cmdActive || settlingActive)) {
            continue;
        }
        if (!any) {
            any = true;
            first = i;
        }
        last = i;
    }

    if (!any) {
        report.keptRows = report.originalRows;
        report.beginRow = 0;
        report.endRow = report.originalRows;
        return false;
    }

    const std::size_t preRows = EstimateRowsForSecondsFromTable(dataset.table, settings.preSeconds);
    const std::size_t postRows = EstimateRowsForSecondsFromTable(dataset.table, settings.postSeconds);
    std::size_t begin = (first > preRows) ? (first - preRows) : 0;
    std::size_t end = std::min(dataset.table.RowCount(), last + postRows + 1);
    if (begin >= end) {
        report.keptRows = report.originalRows;
        report.beginRow = 0;
        report.endRow = report.originalRows;
        return false;
    }

    const float t0 = dataset.table.timeSeconds[begin];
    const float t1 = dataset.table.timeSeconds[end - 1];
    TrimTelemetryTable(dataset.table, begin, end);

    if (!dataset.events.empty()) {
        std::vector<EventRecord> trimmed;
        trimmed.reserve(dataset.events.size());
        for (const auto &ev : dataset.events) {
            if (std::isfinite(ev.timestamp) && ev.timestamp >= t0 && ev.timestamp <= t1) {
                trimmed.push_back(ev);
            }
        }
        dataset.events.swap(trimmed);
    }

    report.applied = true;
    report.beginRow = begin;
    report.endRow = end;
    report.keptRows = dataset.table.RowCount();
    return true;
}

bool LoadDatasetFromPath(const fs::path &path, unsigned threads, LoadedDataset &out, std::string &error) {
    out = LoadedDataset{};

    const std::string pathStr = path.string();
    if (EndsWithCaseInsensitive(pathStr, ".bin")) {
        std::vector<uint8_t> bytes;
        std::vector<TelemetryRecordRef> telemetry;
        std::vector<EventRecord> events;
        if (!LoadFile(path, bytes, error)) {
            return false;
        }
        if (!ParseLog(bytes, telemetry, events, &out.metadata, &out.schemaWarnings, error)) {
            return false;
        }
        out.sourcePath = path;
        out.sourceType = "BIN";
        out.table = BuildTelemetryTableFromBin(telemetry);
        AddDerivedChannels(out.table);
        BuildTimeIndex(out.table, out.timeIndex, 256);
        WriteIdxSidecar(path, out.table, out.timeIndex);
        out.events = std::move(events);
        return true;
    }

    if (EndsWithCaseInsensitive(pathStr, ".csv")) {
        TelemetryTable table;
        if (!ParseCsvTelemetry(path,
                               threads,
                               table,
                               error,
                               &out.rawCsvText,
                               &out.rawCsvLineRanges,
                               &out.rawCsvHeaders,
                               &out.metadata,
                               &out.schemaWarnings,
                               &out.rawDataStartLine)) {
            return false;
        }
        out.sourcePath = path;
        out.sourceType = "CSV";
        out.table = std::move(table);
        AddDerivedChannels(out.table);
        BuildTimeIndex(out.table, out.timeIndex, 256);
        WriteIdxSidecar(path, out.table, out.timeIndex, &out.rawCsvLineRanges, out.rawDataStartLine);
        out.hasRawCsv = true;
        return true;
    }

    error = "Unsupported file type. Use .BIN or .csv.";
    return false;
}

void SelectDefaultSeries(TelemetryTable &table) {
    const std::array<const char *, 5> preferred = {
        "sensor_altitude_feet",
        "state_apogee_estimate",
        "sensor_auto_cmd_deg",
        "sensor_optimizer_best_predicted_apogee_m",
        "sensor_actuation_is_settling"};

    std::size_t selectedCount = 0;
    for (auto &series : table.series) {
        series.selected = false;
        for (const char *name : preferred) {
            if (series.name == name) {
                series.selected = true;
                selectedCount++;
                break;
            }
        }
    }

    if (selectedCount > 0) {
        return;
    }

    for (auto &series : table.series) {
        if (series.name.find("time") != std::string::npos) {
            continue;
        }
        series.selected = true;
        selectedCount++;
        if (selectedCount >= 3) {
            break;
        }
    }
}

ColumnMeta ColumnMetadata(const std::string &name) {
    for (const auto &field : kTelemetryFields) {
        if (name == field.name) {
            return field.meta;
        }
    }
    if (name.rfind("derived_", 0) == 0) {
        if (name == "derived_speed_m_s") return {"m/s", ColumnClass::Derived, 0.0f, 1500.0f};
        if (name == "derived_mach") return {"Mach", ColumnClass::Derived, 0.0f, 5.0f};
        if (name == "derived_dynamic_pressure_pa") return {"Pa", ColumnClass::Derived, 0.0f, 300000.0f};
        if (name == "derived_accel_mag_m_s2") return {"m/s^2", ColumnClass::Derived, 0.0f, 500.0f};
        if (name == "derived_jerk_mag_m_s3") return {"m/s^3", ColumnClass::Derived, -10000.0f, 10000.0f};
        if (name == "derived_predictor_conf_control_active") return {"bool", ColumnClass::Derived, 0.0f, 1.0f};
        if (name == "derived_predictor_conf_positive_vertical_velocity") return {"bool", ColumnClass::Derived, 0.0f, 1.0f};
        if (name == "derived_predictor_conf_using_horizontal_model") return {"bool", ColumnClass::Derived, 0.0f, 1.0f};
        if (name == "derived_predictor_conf_horizontal_speed_capped") return {"bool", ColumnClass::Derived, 0.0f, 1.0f};
        if (name == "derived_predictor_conf_zenith_clamped") return {"bool", ColumnClass::Derived, 0.0f, 1.0f};
        if (name == "derived_predictor_conf_angular_rate_clamped") return {"bool", ColumnClass::Derived, 0.0f, 1.0f};
        if (name == "derived_replay_altitude_m") return {"m", ColumnClass::Replay};
        if (name == "derived_replay_logged_state_apogee_m") return {"m", ColumnClass::Replay};
        if (name == "derived_replay_logged_optimizer_apogee_m") return {"m", ColumnClass::Replay};
        if (name == "derived_replay_apogee_safer_m") return {"m", ColumnClass::Replay};
        if (name == "derived_replay_apogee_ballistic_m") return {"m", ColumnClass::Replay};
        if (name == "derived_replay_error_vs_actual_m") return {"m", ColumnClass::Replay};
        if (name == "derived_replay_error_vs_logged_state_m") return {"m", ColumnClass::Replay};
        if (name == "derived_replay_error_vs_logged_optimizer_m") return {"m", ColumnClass::Replay};
        if (name == "derived_replay_seed_horizontal_speed_mps") return {"m/s", ColumnClass::Replay};
        if (name == "derived_replay_seed_zenith_deg") return {"deg", ColumnClass::Replay};
        if (name == "derived_replay_seed_angular_rate_deg_s") return {"deg/s", ColumnClass::Replay};
        return {"raw", ColumnClass::Derived, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()};
    }
    if (name.find("event") != std::string::npos) {
        return {"event", ColumnClass::Event, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()};
    }
    return {"raw", ColumnClass::Unknown, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()};
}

const char *ColumnClassName(ColumnClass klass) {
    switch (klass) {
        case ColumnClass::Sensor:
            return "Sensor";
        case ColumnClass::State:
            return "State";
        case ColumnClass::Control:
            return "Control";
        case ColumnClass::Derived:
            return "Derived";
        case ColumnClass::Event:
            return "Event";
        case ColumnClass::Replay:
            return "Replay";
        default:
            return "Other";
    }
}

ImU32 ColorForSeries(const std::string &name) {
    const std::uint32_t seed = static_cast<std::uint32_t>(std::hash<std::string>{}(name));
    const int r = 70 + static_cast<int>((seed >> 0) & 0x7F);
    const int g = 70 + static_cast<int>((seed >> 7) & 0x7F);
    const int b = 70 + static_cast<int>((seed >> 14) & 0x7F);
    return IM_COL32(r, g, b, 255);
}

void DrawChartFrame(const ImVec2 &min,
                    const ImVec2 &max,
                    const char *label,
                    float minValue,
                    float maxValue,
                    float tMin,
                    float tMax,
                    std::size_t pointsPlotted) {
    ImDrawList *draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(min, max, IM_COL32(20, 24, 30, 255), 6.0f);
    draw->AddRect(min, max, IM_COL32(70, 78, 92, 255), 6.0f, 0, 1.0f);

    char legend[256];
    std::snprintf(legend,
                  sizeof(legend),
                  "%s  |  y:[%.3f, %.3f]  t:[%.3f, %.3f]  n=%zu",
                  label,
                  static_cast<double>(minValue),
                  static_cast<double>(maxValue),
                  static_cast<double>(tMin),
                  static_cast<double>(tMax),
                  pointsPlotted);
    draw->AddText(ImVec2(min.x + 8.0f, min.y + 6.0f), IM_COL32(220, 225, 235, 255), legend);
}

std::size_t LowerBoundTime(const std::vector<float> &timeValues, float needle) {
    auto it = std::lower_bound(timeValues.begin(), timeValues.end(), needle);
    return static_cast<std::size_t>(it - timeValues.begin());
}

std::size_t UpperBoundTime(const std::vector<float> &timeValues, float needle) {
    auto it = std::upper_bound(timeValues.begin(), timeValues.end(), needle);
    return static_cast<std::size_t>(it - timeValues.begin());
}

std::size_t NearestTimeIndex(const std::vector<float> &timeValues, float needle) {
    if (timeValues.empty()) {
        return 0;
    }
    const std::size_t lower = LowerBoundTime(timeValues, needle);
    if (lower == 0) {
        return 0;
    }
    if (lower >= timeValues.size()) {
        return timeValues.size() - 1;
    }
    const float prev = timeValues[lower - 1];
    const float next = timeValues[lower];
    return (std::fabs(needle - prev) <= std::fabs(next - needle)) ? (lower - 1) : lower;
}

void DrawSeriesChart(const TelemetryTable &table,
                     const NumericSeries &series,
                     const std::vector<EventRecord> &events,
                     float selectedTMin,
                     float selectedTMax,
                     bool drawEvents,
                     bool drawReferenceLine,
                     float referenceValue,
                     float chartHeight) {
    ImGui::PushID(series.name.c_str());
    ImVec2 plotSize(ImGui::GetContentRegionAvail().x, chartHeight);
    if (plotSize.x < 32.0f) {
        plotSize.x = 32.0f;
    }
    if (plotSize.y < 80.0f) {
        plotSize.y = 80.0f;
    }

    ImGui::InvisibleButton("plot", plotSize);
    const ImVec2 canvasMin = ImGui::GetItemRectMin();
    const ImVec2 canvasMax = ImGui::GetItemRectMax();
    const ImVec2 innerMin(canvasMin.x + 8.0f, canvasMin.y + 24.0f);
    const ImVec2 innerMax(canvasMax.x - 8.0f, canvasMax.y - 14.0f);

    if (table.timeSeconds.empty() || series.values.empty()) {
        DrawChartFrame(canvasMin, canvasMax, series.name.c_str(), 0.0f, 1.0f, 0.0f, 0.0f, 0);
        ImGui::PopID();
        return;
    }

    std::size_t begin = 0;
    std::size_t end = table.timeSeconds.size();
    if (table.timeSorted) {
        begin = LowerBoundTime(table.timeSeconds, selectedTMin);
        end = UpperBoundTime(table.timeSeconds, selectedTMax);
    }
    if (begin >= end || end > table.timeSeconds.size()) {
        DrawChartFrame(canvasMin,
                       canvasMax,
                       series.name.c_str(),
                       0.0f,
                       1.0f,
                       selectedTMin,
                       selectedTMax,
                       0);
        ImGui::PopID();
        return;
    }

    const std::size_t rawSpan = end - begin;
    const std::size_t stride = std::max<std::size_t>(1, rawSpan / static_cast<std::size_t>(std::max(120.0f, plotSize.x)));

    float minValue = std::numeric_limits<float>::infinity();
    float maxValue = -std::numeric_limits<float>::infinity();
    std::size_t finiteCount = 0;
    for (std::size_t i = begin; i < end; i += stride) {
        const float value = series.values[i];
        if (!std::isfinite(value)) {
            continue;
        }
        minValue = std::min(minValue, value);
        maxValue = std::max(maxValue, value);
        finiteCount++;
    }

    if (finiteCount == 0) {
        DrawChartFrame(canvasMin,
                       canvasMax,
                       series.name.c_str(),
                       0.0f,
                       1.0f,
                       selectedTMin,
                       selectedTMax,
                       0);
        ImGui::PopID();
        return;
    }

    if (std::fabs(maxValue - minValue) < 1e-6f) {
        minValue -= 0.5f;
        maxValue += 0.5f;
    }

    DrawChartFrame(canvasMin,
                   canvasMax,
                   series.name.c_str(),
                   minValue,
                   maxValue,
                   selectedTMin,
                   selectedTMax,
                   finiteCount);

    ImDrawList *draw = ImGui::GetWindowDrawList();
    const float spanT = std::max(1e-6f, selectedTMax - selectedTMin);
    const float spanY = std::max(1e-6f, maxValue - minValue);

    if (drawReferenceLine && referenceValue >= minValue && referenceValue <= maxValue) {
        const float yn = (referenceValue - minValue) / spanY;
        const float y = innerMax.y - yn * (innerMax.y - innerMin.y);
        draw->AddLine(ImVec2(innerMin.x, y), ImVec2(innerMax.x, y), IM_COL32(245, 180, 70, 180), 1.0f);
    }

    if (drawEvents) {
        for (const EventRecord &ev : events) {
            if (ev.timestamp < selectedTMin || ev.timestamp > selectedTMax) {
                continue;
            }
            const float xn = (ev.timestamp - selectedTMin) / spanT;
            const float x = innerMin.x + xn * (innerMax.x - innerMin.x);
            draw->AddLine(ImVec2(x, innerMin.y), ImVec2(x, innerMax.y), IM_COL32(220, 140, 120, 130), 1.0f);
        }
    }

    std::vector<ImVec2> points;
    points.reserve(finiteCount);
    for (std::size_t i = begin; i < end; i += stride) {
        const float t = table.timeSeconds[i];
        const float value = series.values[i];
        if (!std::isfinite(value)) {
            continue;
        }
        const float xn = (t - selectedTMin) / spanT;
        const float yn = (value - minValue) / spanY;
        points.emplace_back(innerMin.x + xn * (innerMax.x - innerMin.x), innerMax.y - yn * (innerMax.y - innerMin.y));
    }

    if (points.size() >= 2) {
        draw->AddPolyline(points.data(), static_cast<int>(points.size()), ColorForSeries(series.name), 0, 1.5f);
    }

    draw->AddText(ImVec2(innerMin.x, innerMax.y + 1.0f), IM_COL32(170, 180, 195, 255), "t");
    char maxLabel[64];
    std::snprintf(maxLabel, sizeof(maxLabel), "%.3f", static_cast<double>(selectedTMax));
    draw->AddText(ImVec2(innerMax.x - 46.0f, innerMax.y + 1.0f), IM_COL32(170, 180, 195, 255), maxLabel);

    ImGui::PopID();
}

struct SeriesStats {
    std::size_t samples = 0;
    float minValue = 0.0f;
    float maxValue = 0.0f;
    float meanValue = 0.0f;
    float stddevValue = 0.0f;
};

bool ComputeSeriesStats(const TelemetryTable &table,
                        const NumericSeries &series,
                        float selectedTMin,
                        float selectedTMax,
                        SeriesStats &out) {
    if (table.timeSeconds.empty() || series.values.empty()) {
        return false;
    }

    std::size_t begin = 0;
    std::size_t end = table.timeSeconds.size();
    if (table.timeSorted) {
        begin = LowerBoundTime(table.timeSeconds, selectedTMin);
        end = UpperBoundTime(table.timeSeconds, selectedTMax);
    }
    if (begin >= end || end > table.timeSeconds.size()) {
        return false;
    }

    double sum = 0.0;
    double sumSq = 0.0;
    float minValue = std::numeric_limits<float>::infinity();
    float maxValue = -std::numeric_limits<float>::infinity();
    std::size_t count = 0;
    for (std::size_t i = begin; i < end; ++i) {
        const float value = series.values[i];
        if (!std::isfinite(value)) {
            continue;
        }
        minValue = std::min(minValue, value);
        maxValue = std::max(maxValue, value);
        sum += static_cast<double>(value);
        sumSq += static_cast<double>(value) * static_cast<double>(value);
        count++;
    }
    if (count == 0) {
        return false;
    }

    const double mean = sum / static_cast<double>(count);
    double variance = (sumSq / static_cast<double>(count)) - mean * mean;
    if (variance < 0.0) {
        variance = 0.0;
    }

    out.samples = count;
    out.minValue = minValue;
    out.maxValue = maxValue;
    out.meanValue = static_cast<float>(mean);
    out.stddevValue = static_cast<float>(std::sqrt(variance));
    return true;
}

void DrawOverlayChart(const TelemetryTable &table,
                      const std::vector<const NumericSeries *> &selectedSeries,
                      const std::vector<EventRecord> &events,
                      float selectedTMin,
                      float selectedTMax,
                      bool drawEvents,
                      bool drawReferenceLine,
                      float referenceValue,
                      bool normalizeSeries,
                      float chartHeight,
                      bool *outHasHovered = nullptr,
                      std::size_t *outHoveredRow = nullptr,
                      float *outHoveredTime = nullptr) {
    ImVec2 plotSize(ImGui::GetContentRegionAvail().x, chartHeight);
    if (plotSize.x < 32.0f) {
        plotSize.x = 32.0f;
    }
    if (plotSize.y < 100.0f) {
        plotSize.y = 100.0f;
    }

    ImGui::PushID(selectedSeries.empty() ? nullptr : static_cast<const void *>(selectedSeries.front()));
    ImGui::InvisibleButton("overlay_plot", plotSize);
    const ImVec2 canvasMin = ImGui::GetItemRectMin();
    const ImVec2 canvasMax = ImGui::GetItemRectMax();
    const ImVec2 innerMin(canvasMin.x + 8.0f, canvasMin.y + 24.0f);
    const ImVec2 innerMax(canvasMax.x - 8.0f, canvasMax.y - 14.0f);

    if (table.timeSeconds.empty() || selectedSeries.empty()) {
        DrawChartFrame(canvasMin, canvasMax, "Overlay", 0.0f, 1.0f, selectedTMin, selectedTMax, 0);
        return;
    }

    std::size_t begin = 0;
    std::size_t end = table.timeSeconds.size();
    if (table.timeSorted) {
        begin = LowerBoundTime(table.timeSeconds, selectedTMin);
        end = UpperBoundTime(table.timeSeconds, selectedTMax);
    }
    if (begin >= end || end > table.timeSeconds.size()) {
        DrawChartFrame(canvasMin, canvasMax, "Overlay", 0.0f, 1.0f, selectedTMin, selectedTMax, 0);
        return;
    }

    const std::size_t rawSpan = end - begin;
    const std::size_t stride = std::max<std::size_t>(1, rawSpan / static_cast<std::size_t>(std::max(120.0f, plotSize.x)));

    float globalMin = std::numeric_limits<float>::infinity();
    float globalMax = -std::numeric_limits<float>::infinity();
    std::size_t finiteCount = 0;
    for (const NumericSeries *series : selectedSeries) {
        for (std::size_t i = begin; i < end; i += stride) {
            const float value = series->values[i];
            if (!std::isfinite(value)) {
                continue;
            }
            if (!normalizeSeries) {
                globalMin = std::min(globalMin, value);
                globalMax = std::max(globalMax, value);
            }
            finiteCount++;
        }
    }

    if (finiteCount == 0) {
        DrawChartFrame(canvasMin, canvasMax, "Overlay", 0.0f, 1.0f, selectedTMin, selectedTMax, 0);
        return;
    }

    if (normalizeSeries) {
        globalMin = 0.0f;
        globalMax = 1.0f;
    } else if (std::fabs(globalMax - globalMin) < 1e-6f) {
        globalMin -= 0.5f;
        globalMax += 0.5f;
    }

    DrawChartFrame(canvasMin,
                   canvasMax,
                   normalizeSeries ? "Overlay (normalized 0..1)" : "Overlay",
                   globalMin,
                   globalMax,
                   selectedTMin,
                   selectedTMax,
                   finiteCount);

    ImDrawList *draw = ImGui::GetWindowDrawList();
    const float spanT = std::max(1e-6f, selectedTMax - selectedTMin);
    const float globalSpanY = std::max(1e-6f, globalMax - globalMin);

    if (drawReferenceLine && referenceValue >= globalMin && referenceValue <= globalMax) {
        const float yn = (referenceValue - globalMin) / globalSpanY;
        const float y = innerMax.y - yn * (innerMax.y - innerMin.y);
        draw->AddLine(ImVec2(innerMin.x, y), ImVec2(innerMax.x, y), IM_COL32(245, 180, 70, 180), 1.0f);
    }

    if (drawEvents) {
        for (const EventRecord &ev : events) {
            if (ev.timestamp < selectedTMin || ev.timestamp > selectedTMax) {
                continue;
            }
            const float xn = (ev.timestamp - selectedTMin) / spanT;
            const float x = innerMin.x + xn * (innerMax.x - innerMin.x);
            draw->AddLine(ImVec2(x, innerMin.y), ImVec2(x, innerMax.y), IM_COL32(220, 140, 120, 130), 1.0f);
        }
    }

    for (const NumericSeries *series : selectedSeries) {
        float localMin = std::numeric_limits<float>::infinity();
        float localMax = -std::numeric_limits<float>::infinity();
        if (normalizeSeries) {
            for (std::size_t i = begin; i < end; i += stride) {
                const float value = series->values[i];
                if (!std::isfinite(value)) {
                    continue;
                }
                localMin = std::min(localMin, value);
                localMax = std::max(localMax, value);
            }
            if (!std::isfinite(localMin) || !std::isfinite(localMax)) {
                continue;
            }
            if (std::fabs(localMax - localMin) < 1e-6f) {
                localMin -= 0.5f;
                localMax += 0.5f;
            }
        }

        const float spanY = normalizeSeries ? std::max(1e-6f, localMax - localMin) : globalSpanY;

        std::vector<ImVec2> points;
        points.reserve(rawSpan / stride + 1);
        for (std::size_t i = begin; i < end; i += stride) {
            const float t = table.timeSeconds[i];
            const float value = series->values[i];
            if (!std::isfinite(value)) {
                continue;
            }
            const float xn = (t - selectedTMin) / spanT;
            const float yValue = normalizeSeries ? (value - localMin) : (value - globalMin);
            const float yn = yValue / spanY;
            points.emplace_back(innerMin.x + xn * (innerMax.x - innerMin.x),
                                innerMax.y - yn * (innerMax.y - innerMin.y));
        }

        if (points.size() >= 2) {
            draw->AddPolyline(points.data(), static_cast<int>(points.size()), ColorForSeries(series->name), 0, 1.6f);
        }
    }

    const bool isHovered = ImGui::IsItemHovered();
    if (outHasHovered != nullptr) {
        *outHasHovered = isHovered;
    }
    if (isHovered) {
        const ImVec2 mouse = ImGui::GetIO().MousePos;
        const float clampedX = std::clamp(mouse.x, innerMin.x, innerMax.x);
        const float hoverT = selectedTMin + ((clampedX - innerMin.x) / std::max(1.0f, innerMax.x - innerMin.x)) * spanT;
        const std::size_t hoverRow = NearestTimeIndex(table.timeSeconds, hoverT);
        const float x = innerMin.x + ((table.timeSeconds[hoverRow] - selectedTMin) / spanT) * (innerMax.x - innerMin.x);
        draw->AddLine(ImVec2(x, innerMin.y), ImVec2(x, innerMax.y), IM_COL32(255, 255, 255, 110), 1.0f);
        if (outHoveredRow != nullptr) {
            *outHoveredRow = hoverRow;
        }
        if (outHoveredTime != nullptr) {
            *outHoveredTime = table.timeSeconds[hoverRow];
        }

        ImGui::BeginTooltip();
        ImGui::Text("row=%zu  t=%.4f s", hoverRow, static_cast<double>(table.timeSeconds[hoverRow]));
        for (const NumericSeries *series : selectedSeries) {
            if (hoverRow >= series->values.size()) {
                continue;
            }
            const float value = series->values[hoverRow];
            if (!std::isfinite(value)) {
                continue;
            }
            ImGui::Text("%s: %.5g", series->name.c_str(), static_cast<double>(value));
        }
        ImGui::EndTooltip();
    }
    ImGui::PopID();
}

const NumericSeries *FindSeriesByName(const TelemetryTable &table, const char *name) {
    for (const NumericSeries &series : table.series) {
        if (series.name == name) {
            return &series;
        }
    }
    return nullptr;
}

void AppendEmptySeries(TelemetryTable &table, const std::string &name, std::size_t rows) {
    NumericSeries series;
    series.name = name;
    series.values.assign(rows, std::numeric_limits<float>::quiet_NaN());
    table.series.push_back(std::move(series));
}

std::optional<float> SeriesValueAt(const NumericSeries *series, std::size_t row) {
    if (series == nullptr || row >= series->values.size()) {
        return std::nullopt;
    }
    const float value = series->values[row];
    if (!std::isfinite(value)) {
        return std::nullopt;
    }
    return value;
}

std::string FormatOptionalFloat(std::optional<float> value, const char *units = nullptr) {
    if (!value.has_value() || !std::isfinite(*value)) {
        return "--";
    }
    char buffer[96];
    if (units != nullptr && units[0] != '\0') {
        std::snprintf(buffer, sizeof(buffer), "%.6g %s", static_cast<double>(*value), units);
    } else {
        std::snprintf(buffer, sizeof(buffer), "%.6g", static_cast<double>(*value));
    }
    return std::string(buffer);
}

bool ParseFiveColumnCsvLine(const std::string &line, std::array<double, 5> &values) {
    const char *ptr = line.c_str();
    for (int i = 0; i < 5; ++i) {
        char *end = nullptr;
        values[static_cast<std::size_t>(i)] = std::strtod(ptr, &end);
        if (end == ptr) {
            return false;
        }
        ptr = end;
        while (*ptr == ',' || *ptr == ' ' || *ptr == '\t') {
            ++ptr;
        }
    }
    return true;
}

bool LoadNativeCfdTable(const fs::path &path, NativeCfdTableStorage &storage, std::string &error) {
    std::ifstream in(path);
    if (!in) {
        error = "Failed to open CFD table: " + path.string();
        storage.loaded = false;
        return false;
    }

    storage = NativeCfdTableStorage{};
    std::vector<std::array<double, 5>> rows;
    rows.reserve(6000);
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) {
            continue;
        }
        std::array<double, 5> values{};
        if (!ParseFiveColumnCsvLine(line, values)) {
            continue;
        }
        rows.push_back(values);
        storage.acs.push_back(values[0]);
        storage.atk.push_back(values[1]);
        storage.mach.push_back(values[2]);
    }
    if (rows.empty()) {
        error = "CFD table has no numeric rows: " + path.string();
        return false;
    }

    auto sortUnique = [](std::vector<double> &values) {
        std::sort(values.begin(), values.end());
        values.erase(std::unique(values.begin(), values.end()), values.end());
    };
    sortUnique(storage.acs);
    sortUnique(storage.atk);
    sortUnique(storage.mach);

    const int acsCount = static_cast<int>(storage.acs.size());
    const int atkCount = static_cast<int>(storage.atk.size());
    const int machCount = static_cast<int>(storage.mach.size());
    if (acsCount < 2 || atkCount < 2 || machCount < 2) {
        error = "CFD table grid is too small: " + path.string();
        return false;
    }

    storage.axial.assign(static_cast<std::size_t>(acsCount * atkCount * machCount), std::numeric_limits<double>::quiet_NaN());
    storage.normal.assign(static_cast<std::size_t>(acsCount * atkCount * machCount), std::numeric_limits<double>::quiet_NaN());

    auto findIndex = [](const std::vector<double> &axis, double value) -> int {
        const auto it = std::lower_bound(axis.begin(), axis.end(), value);
        if (it == axis.end() || *it != value) {
            return -1;
        }
        return static_cast<int>(it - axis.begin());
    };

    for (const auto &row : rows) {
        const int i = findIndex(storage.acs, row[0]);
        const int j = findIndex(storage.atk, row[1]);
        const int k = findIndex(storage.mach, row[2]);
        if (i < 0 || j < 0 || k < 0) {
            continue;
        }
        const std::size_t index = static_cast<std::size_t>((i * atkCount + j) * machCount + k);
        storage.axial[index] = row[3];
        storage.normal[index] = row[4];
    }

    storage.table.acsAnglesDeg = storage.acs.data();
    storage.table.atkAnglesDeg = storage.atk.data();
    storage.table.machNumbers = storage.mach.data();
    storage.table.axialForces = storage.axial.data();
    storage.table.normalForces = storage.normal.data();
    storage.table.acsCount = acsCount;
    storage.table.atkCount = atkCount;
    storage.table.machCount = machCount;
    storage.loaded = true;
    return true;
}

bool ComputeReplayAnalysis(const LoadedDataset &dataset, const ReplaySettings &settings, ReplayAnalysis &analysis, std::string &error) {
    analysis = ReplayAnalysis{};
    const TelemetryTable &source = dataset.table;
    const std::size_t rows = source.RowCount();
    if (rows == 0) {
        error = "No telemetry rows loaded for replay.";
        return false;
    }

    const NumericSeries *hasFiltered = FindSeriesByName(source, "has_filtered_state");
    const NumericSeries *statusSeries = FindSeriesByName(source, "flight_status_raw");
    const SeriesLookup altitudeSeries = FindSeriesByNamesWithScale(
        source,
        {{"state_altitude_agl_feet", kFeetToMeters}, {"state_position_z", 1.0f}});
    const SeriesLookup velocitySeries = FindSeriesByNamesWithScale(
        source,
        {{"state_vertical_velocity_fps", kFeetToMeters}, {"state_velocity_z", 1.0f}});
    const SeriesLookup zenithSeries = FindSeriesByNamesWithScale(
        source,
        {{"state_zenith_deg", kDegToRad}, {"state_zenith", 1.0f}});
    if (altitudeSeries.series == nullptr || velocitySeries.series == nullptr || zenithSeries.series == nullptr) {
        error = "Replay requires altitude, vertical velocity, and zenith state columns.";
        return false;
    }

    const NumericSeries *velXSeries = FindSeriesByName(source, "state_velocity_x");
    const NumericSeries *velYSeries = FindSeriesByName(source, "state_velocity_y");
    const NumericSeries *inertialXSeries = FindSeriesByName(source, "state_inertial_acceleration_x");
    const NumericSeries *inertialYSeries = FindSeriesByName(source, "state_inertial_acceleration_y");
    const SeriesLookup loggedStateApogee = FindSeriesByNamesWithScale(
        source,
        {{"state_apogee_estimate_feet", kFeetToMeters}, {"state_apogee_estimate", 1.0f}});
    const SeriesLookup loggedOptimizerApogee = FindSeriesByNamesWithScale(
        source,
        {{"sensor_optimizer_best_predicted_apogee_m", 1.0f}, {"state_apogee_estimate_feet", kFeetToMeters}});

    analysis.table.timeSeconds = source.timeSeconds;
    analysis.table.timeSorted = source.timeSorted;
    AppendEmptySeries(analysis.table, "derived_replay_altitude_m", rows);
    AppendEmptySeries(analysis.table, "derived_replay_logged_state_apogee_m", rows);
    AppendEmptySeries(analysis.table, "derived_replay_logged_optimizer_apogee_m", rows);
    AppendEmptySeries(analysis.table, "derived_replay_apogee_safer_m", rows);
    AppendEmptySeries(analysis.table, "derived_replay_apogee_ballistic_m", rows);
    AppendEmptySeries(analysis.table, "derived_replay_error_vs_actual_m", rows);
    AppendEmptySeries(analysis.table, "derived_replay_error_vs_logged_state_m", rows);
    AppendEmptySeries(analysis.table, "derived_replay_error_vs_logged_optimizer_m", rows);
    AppendEmptySeries(analysis.table, "derived_replay_seed_horizontal_speed_mps", rows);
    AppendEmptySeries(analysis.table, "derived_replay_seed_zenith_deg", rows);
    AppendEmptySeries(analysis.table, "derived_replay_seed_angular_rate_deg_s", rows);

    EnvironmentModel environment;
    ApogeeVehicleParameters vehicleParameters;
    vehicleParameters.centerOfPressureOffsetMeters = settings::vehicle::kCenterOfPressureOffsetMeters;
    vehicleParameters.momentOfInertia = settings::vehicle::kMomentOfInertiaKgM2;
    vehicleParameters.dryMass = settings::vehicle::kDryMassKg;
    ApogeePredictor predictor(environment, vehicleParameters);
    predictor.SetMaxIntegrationSteps(std::max(1, settings.maxPredictorSteps));

    NativeCfdTableStorage cfdStorage;
    if (settings.useCfd) {
        std::string cfdError;
        if (LoadNativeCfdTable(fs::path(settings.cfdPath), cfdStorage, cfdError)) {
            predictor.SetForceTable(&cfdStorage.table);
            analysis.statusText = "Replay using CFD: " + std::string(settings.cfdPath);
        } else {
            analysis.warnings.push_back(cfdError + " | falling back to ballistic predictor");
        }
    } else {
        analysis.statusText = "Replay using ballistic predictor only";
    }

    PredictorHorizontalVelocityTracker tracker;
    bool hasPreviousZenith = false;
    float previousTime = 0.0f;
    float previousZenith = 0.0f;
    double lastPredictionMeters = std::numeric_limits<double>::quiet_NaN();
    float lastPredictionTime = 0.0f;
    bool hasLastPrediction = false;
    double saferPredictionSum = 0.0;
    double ballisticPredictionSum = 0.0;
    std::size_t saferPredictionCount = 0;
    std::size_t ballisticPredictionCount = 0;

    for (std::size_t row = 0; row < rows; ++row) {
        const float timeValue = source.timeSeconds[row];
        const auto altitudeRaw = SeriesValueAt(altitudeSeries.series, row);
        const auto verticalVelocityRaw = SeriesValueAt(velocitySeries.series, row);
        const auto zenithRaw = SeriesValueAt(zenithSeries.series, row);
        const std::optional<float> altitude =
            altitudeRaw.has_value() ? std::optional<float>(*altitudeRaw * altitudeSeries.scale) : std::nullopt;
        const std::optional<float> verticalVelocity =
            verticalVelocityRaw.has_value() ? std::optional<float>(*verticalVelocityRaw * velocitySeries.scale) : std::nullopt;
        const std::optional<float> zenith =
            zenithRaw.has_value() ? std::optional<float>(*zenithRaw * zenithSeries.scale) : std::nullopt;
        if (!std::isfinite(timeValue) || !altitude.has_value() || !verticalVelocity.has_value() || !zenith.has_value()) {
            continue;
        }
        if (hasFiltered != nullptr) {
            const auto filtered = SeriesValueAt(hasFiltered, row);
            if (filtered.has_value() && *filtered < 0.5f) {
                continue;
            }
        }

        analysis.actualApogeeMeters = std::isfinite(analysis.actualApogeeMeters)
                                          ? std::max(analysis.actualApogeeMeters, *altitude)
                                          : *altitude;

        FlightStatus status = FlightStatus::Ground;
        if (statusSeries != nullptr) {
            if (const auto rawStatus = SeriesValueAt(statusSeries, row); rawStatus.has_value()) {
                const int rounded = static_cast<int>(std::lround(*rawStatus));
                if (rounded >= 0 && rounded <= static_cast<int>(FlightStatus::Descent)) {
                    status = static_cast<FlightStatus>(rounded);
                }
            }
        }

        const float dt = hasPreviousZenith ? (timeValue - previousTime) : 0.0f;
        const double angularRate = ComputeSeededAngularRate(timeValue, *zenith, previousTime, previousZenith, hasPreviousZenith);
        const bool allowIntegration = (status == FlightStatus::Burn || status == FlightStatus::Coast);
        const double horizontalSpeed =
            UpdatePredictorHorizontalSpeed(tracker,
                                           SeriesValueAt(inertialXSeries, row).value_or(0.0f),
                                           SeriesValueAt(inertialYSeries, row).value_or(0.0f),
                                           dt,
                                           allowIntegration,
                                           *verticalVelocity,
                                           *zenith);
        const double clampedZenith = ClampPredictorZenithRadians(*zenith);
        const double clampedAngularRate = ClampPredictorAngularRate(angularRate);
        const double horizontalVelocity = std::isfinite(horizontalSpeed)
                                              ? horizontalSpeed
                                              : std::hypot(SeriesValueAt(velXSeries, row).value_or(0.0f),
                                                           SeriesValueAt(velYSeries, row).value_or(0.0f));

        const double ballisticApogee =
            (*verticalVelocity > 0.0f) ? (*altitude + (*verticalVelocity * *verticalVelocity) / (2.0 * constants::kGravity)) : *altitude;

        double replayApogee = hasLastPrediction ? lastPredictionMeters : ballisticApogee;
        const bool shouldPredict = (*verticalVelocity > 0.0f) && allowIntegration;
        if (shouldPredict && (!hasLastPrediction || (timeValue - lastPredictionTime) >= settings.refreshIntervalSeconds)) {
            ApogeeState predictorState;
            predictorState.altitudeMeters = *altitude;
            predictorState.horizontalDistanceMeters = 0.0;
            predictorState.verticalVelocity = *verticalVelocity;
            predictorState.horizontalVelocity = horizontalVelocity;
            predictorState.zenith = clampedZenith;
            predictorState.angularVelocity = clampedAngularRate;
            predictorState.acsAngleDeg = 0.0;
            replayApogee = predictor.PredictApogee(predictorState);
            lastPredictionMeters = replayApogee;
            lastPredictionTime = timeValue;
            hasLastPrediction = true;
        }

        analysis.table.series[0].values[row] = *altitude;
        if (const auto stateApogee = SeriesValueAt(loggedStateApogee.series, row); stateApogee.has_value()) {
            analysis.table.series[1].values[row] = *stateApogee * loggedStateApogee.scale;
        }
        if (const auto optimizerApogee = SeriesValueAt(loggedOptimizerApogee.series, row); optimizerApogee.has_value()) {
            analysis.table.series[2].values[row] = *optimizerApogee * loggedOptimizerApogee.scale;
        }
        analysis.table.series[3].values[row] = static_cast<float>(replayApogee);
        analysis.table.series[4].values[row] = static_cast<float>(ballisticApogee);
        analysis.table.series[8].values[row] = static_cast<float>(horizontalVelocity);
        analysis.table.series[9].values[row] = static_cast<float>(clampedZenith * 57.29577951308232);
        analysis.table.series[10].values[row] = static_cast<float>(clampedAngularRate * 57.29577951308232);

        if (std::isfinite(replayApogee)) {
            analysis.replayApogeeMinMeters = std::isfinite(analysis.replayApogeeMinMeters)
                                                 ? std::min(analysis.replayApogeeMinMeters, static_cast<float>(replayApogee))
                                                 : static_cast<float>(replayApogee);
            analysis.replayApogeeMaxMeters = std::isfinite(analysis.replayApogeeMaxMeters)
                                                 ? std::max(analysis.replayApogeeMaxMeters, static_cast<float>(replayApogee))
                                                 : static_cast<float>(replayApogee);
            saferPredictionSum += replayApogee;
            saferPredictionCount++;
        }
        if (std::isfinite(ballisticApogee)) {
            ballisticPredictionSum += ballisticApogee;
            ballisticPredictionCount++;
        }

        previousTime = timeValue;
        previousZenith = *zenith;
        hasPreviousZenith = true;
    }

    if (std::isfinite(analysis.actualApogeeMeters)) {
        NumericSeries &errorActual = analysis.table.series[5];
        NumericSeries &errorState = analysis.table.series[6];
        NumericSeries &errorOptimizer = analysis.table.series[7];
        const NumericSeries &saferSeries = analysis.table.series[3];
        for (std::size_t row = 0; row < rows; ++row) {
            const float replayApogee = saferSeries.values[row];
            if (!std::isfinite(replayApogee)) {
                continue;
            }
            errorActual.values[row] = replayApogee - analysis.actualApogeeMeters;
            if (const auto stateApogee = SeriesValueAt(loggedStateApogee.series, row); stateApogee.has_value()) {
                errorState.values[row] = replayApogee - (*stateApogee * loggedStateApogee.scale);
            }
            if (const auto optimizerApogee = SeriesValueAt(loggedOptimizerApogee.series, row); optimizerApogee.has_value()) {
                errorOptimizer.values[row] = replayApogee - (*optimizerApogee * loggedOptimizerApogee.scale);
            }
        }
    }

    if (saferPredictionCount > 0) {
        analysis.replayApogeeMeanMeters = static_cast<float>(saferPredictionSum / static_cast<double>(saferPredictionCount));
    }
    if (ballisticPredictionCount > 0) {
        analysis.replayBallisticMeanMeters = static_cast<float>(ballisticPredictionSum / static_cast<double>(ballisticPredictionCount));
    }

    if (analysis.statusText.empty()) {
        analysis.statusText = "Replay computed";
    }
    analysis.ready = true;
    return true;
}

struct SummaryMetrics {
    bool hasGroundAltitude = false;
    float groundAltitudeFeet = 0.0f;

    bool hasApogee = false;
    float apogeeAslFeet = 0.0f;
    float apogeeAglFeet = 0.0f;
    float apogeeTime = 0.0f;

    bool hasPredictedApogee = false;
    float predictedApogeeMeters = 0.0f;

    bool hasMaxVelocity = false;
    float maxVelocityFps = 0.0f;
    float maxVelocityTime = 0.0f;

    bool hasMaxAcceleration = false;
    float maxAcceleration = 0.0f;
    float maxAccelerationTime = 0.0f;

    bool hasFlightTime = false;
    float flightStartTime = 0.0f;
    float flightEndTime = 0.0f;
    float flightDuration = 0.0f;

    std::array<float, 5> phaseDurations{};
};

struct HealthDiagnostics {
    bool hasSampleRate = false;
    float avgSampleRateHz = 0.0f;
    float minDt = 0.0f;
    float maxDt = 0.0f;
    std::size_t gapCount = 0;
    std::size_t nonMonotonicCount = 0;
    std::size_t phaseJumpCount = 0;
    std::size_t nanCount = 0;
    std::size_t totalValues = 0;
    float qualityScore = 0.0f;
};

HealthDiagnostics ComputeHealthDiagnostics(const TelemetryTable &table) {
    HealthDiagnostics d;
    if (table.timeSeconds.size() >= 2) {
        double sumDt = 0.0;
        d.minDt = std::numeric_limits<float>::infinity();
        d.maxDt = 0.0f;
        std::size_t dtCount = 0;
        for (std::size_t i = 1; i < table.timeSeconds.size(); ++i) {
            const float dt = table.timeSeconds[i] - table.timeSeconds[i - 1];
            if (!std::isfinite(dt)) {
                continue;
            }
            if (dt <= 0.0f) {
                d.nonMonotonicCount++;
                continue;
            }
            d.minDt = std::min(d.minDt, dt);
            d.maxDt = std::max(d.maxDt, dt);
            sumDt += dt;
            dtCount++;
        }
        if (dtCount > 0) {
            const float avgDt = static_cast<float>(sumDt / static_cast<double>(dtCount));
            d.hasSampleRate = avgDt > 1e-6f;
            d.avgSampleRateHz = d.hasSampleRate ? (1.0f / avgDt) : 0.0f;
            const float gapThreshold = std::max(0.1f, avgDt * 4.0f);
            for (std::size_t i = 1; i < table.timeSeconds.size(); ++i) {
                const float dt = table.timeSeconds[i] - table.timeSeconds[i - 1];
                if (std::isfinite(dt) && dt > gapThreshold) {
                    d.gapCount++;
                }
            }
        }
    }

    const NumericSeries *status = FindSeriesConst(table, "flight_status_raw");
    if (status != nullptr) {
        int prev = -1;
        for (float v : status->values) {
            if (!std::isfinite(v)) {
                continue;
            }
            const int current = static_cast<int>(std::lround(v));
            if (prev >= 0 && std::abs(current - prev) > 1) {
                d.phaseJumpCount++;
            }
            prev = current;
        }
    }

    for (const auto &series : table.series) {
        for (float v : series.values) {
            d.totalValues++;
            if (!std::isfinite(v)) {
                d.nanCount++;
            }
        }
    }

    float score = 100.0f;
    score -= static_cast<float>(d.gapCount) * 2.0f;
    score -= static_cast<float>(d.nonMonotonicCount) * 5.0f;
    score -= static_cast<float>(d.phaseJumpCount) * 3.0f;
    if (d.totalValues > 0) {
        const float nanPct = 100.0f * static_cast<float>(d.nanCount) / static_cast<float>(d.totalValues);
        score -= std::min(40.0f, nanPct * 2.0f);
    }
    d.qualityScore = std::max(0.0f, std::min(100.0f, score));
    return d;
}

struct DetectedEventRow {
    float time = 0.0f;
    std::string label;
    std::string detail;
};

void PushDetectedEvent(std::vector<DetectedEventRow> &events,
                       float time,
                       const std::string &label,
                       const std::string &detail,
                       std::size_t maxEvents) {
    if (events.size() >= maxEvents) {
        return;
    }
    if (!std::isfinite(time)) {
        return;
    }
    events.push_back(DetectedEventRow{time, label, detail});
}

void AnalyzeDataset(const LoadedDataset &dataset, SummaryMetrics &summary, std::vector<DetectedEventRow> &detected) {
    summary = SummaryMetrics{};
    detected.clear();
    const std::size_t maxDetectedEvents = 800;

    const TelemetryTable &table = dataset.table;
    if (table.timeSeconds.empty()) {
        return;
    }

    const NumericSeries *altitudeFeet = FindSeriesByName(table, "sensor_altitude_feet");
    const SeriesLookup predictedApogeeM = FindSeriesByNamesWithScale(
        table,
        {{"sensor_optimizer_best_predicted_apogee_m", 1.0f}, {"state_apogee_estimate_feet", kFeetToMeters}});
    const SeriesLookup velocityZ = FindSeriesByNamesWithScale(
        table,
        {{"state_vertical_velocity_fps", 1.0f}, {"state_velocity_z", kMetersToFeet}});
    const NumericSeries *accelX = FindSeriesByName(table, "state_acceleration_x");
    const NumericSeries *accelY = FindSeriesByName(table, "state_acceleration_y");
    const NumericSeries *accelZ = FindSeriesByName(table, "state_acceleration_z");
    const NumericSeries *status = FindSeriesByName(table, "flight_status_raw");
    const NumericSeries *autoCmdDeg = FindSeriesByName(table, "sensor_auto_cmd_deg");
    const NumericSeries *isSettling = FindSeriesByName(table, "sensor_actuation_is_settling");

    if (altitudeFeet != nullptr) {
        std::vector<float> groundAltitudeSamples;
        groundAltitudeSamples.reserve(256);
        if (status != nullptr && status->values.size() == altitudeFeet->values.size()) {
            for (std::size_t i = 0; i < altitudeFeet->values.size(); ++i) {
                const float alt = altitudeFeet->values[i];
                const float st = status->values[i];
                if (!std::isfinite(alt) || !std::isfinite(st)) {
                    continue;
                }
                const int phase = static_cast<int>(std::lround(st));
                if (phase == 0) {
                    groundAltitudeSamples.push_back(alt);
                }
            }
        }
        if (groundAltitudeSamples.empty()) {
            for (std::size_t i = 0; i < altitudeFeet->values.size() && i < 200; ++i) {
                const float alt = altitudeFeet->values[i];
                if (std::isfinite(alt)) {
                    groundAltitudeSamples.push_back(alt);
                }
            }
        }
        if (!groundAltitudeSamples.empty()) {
            double sum = 0.0;
            for (float v : groundAltitudeSamples) {
                sum += static_cast<double>(v);
            }
            summary.hasGroundAltitude = true;
            summary.groundAltitudeFeet = static_cast<float>(sum / static_cast<double>(groundAltitudeSamples.size()));
        }

        for (std::size_t i = 0; i < altitudeFeet->values.size(); ++i) {
            const float value = altitudeFeet->values[i];
            if (!std::isfinite(value)) {
                continue;
            }
            if (!summary.hasApogee || value > summary.apogeeAslFeet) {
                summary.hasApogee = true;
                summary.apogeeAslFeet = value;
                summary.apogeeTime = table.timeSeconds[i];
            }
        }
        if (summary.hasApogee) {
            summary.apogeeAglFeet =
                summary.hasGroundAltitude ? (summary.apogeeAslFeet - summary.groundAltitudeFeet) : summary.apogeeAslFeet;
        }
        if (summary.hasApogee) {
            PushDetectedEvent(detected,
                              summary.apogeeTime,
                              "Apogee (altitude peak)",
                              "peak sensor_altitude_feet",
                              maxDetectedEvents);
        }
    }

    if (predictedApogeeM.series != nullptr) {
        for (float value : predictedApogeeM.series->values) {
            if (!std::isfinite(value)) {
                continue;
            }
            value *= predictedApogeeM.scale;
            if (!summary.hasPredictedApogee || value > summary.predictedApogeeMeters) {
                summary.hasPredictedApogee = true;
                summary.predictedApogeeMeters = value;
            }
        }
    }

    if (velocityZ.series != nullptr) {
        float prev = std::numeric_limits<float>::quiet_NaN();
        float prevT = 0.0f;
        for (std::size_t i = 0; i < velocityZ.series->values.size(); ++i) {
            const float v = velocityZ.series->values[i] * velocityZ.scale;
            const float t = table.timeSeconds[i];
            if (!std::isfinite(v)) {
                continue;
            }
            const float absV = std::fabs(v);
            if (!summary.hasMaxVelocity || absV > summary.maxVelocityFps) {
                summary.hasMaxVelocity = true;
                summary.maxVelocityFps = absV;
                summary.maxVelocityTime = t;
            }
            if (std::isfinite(prev) && prev > 0.0f && v <= 0.0f) {
                const float crossingTime = (prevT + t) * 0.5f;
                PushDetectedEvent(detected,
                                  crossingTime,
                                  "Vertical velocity zero-cross",
                                  "vertical velocity crossed from + to -",
                                  maxDetectedEvents);
            }
            prev = v;
            prevT = t;
        }
    }

    if (accelX != nullptr && accelY != nullptr && accelZ != nullptr) {
        for (std::size_t i = 0; i < accelX->values.size(); ++i) {
            const float ax = accelX->values[i];
            const float ay = accelY->values[i];
            const float az = accelZ->values[i];
            if (!(std::isfinite(ax) && std::isfinite(ay) && std::isfinite(az))) {
                continue;
            }
            const float mag = std::sqrt(ax * ax + ay * ay + az * az);
            if (!summary.hasMaxAcceleration || mag > summary.maxAcceleration) {
                summary.hasMaxAcceleration = true;
                summary.maxAcceleration = mag;
                summary.maxAccelerationTime = table.timeSeconds[i];
            }
        }
    }

    if (status != nullptr) {
        int prevStatus = -1;
        for (std::size_t i = 0; i < status->values.size(); ++i) {
            const float s = status->values[i];
            if (!std::isfinite(s)) {
                continue;
            }
            const int current = static_cast<int>(std::lround(s));
            if (current < 0 || current >= static_cast<int>(kFlightStatusNames.size())) {
                continue;
            }
            if (prevStatus >= 0 && current != prevStatus) {
                std::string detail = std::string(kFlightStatusNames[prevStatus]) + " -> " + kFlightStatusNames[current];
                PushDetectedEvent(detected, table.timeSeconds[i], "Phase transition", detail, maxDetectedEvents);
                if (prevStatus == 1 && current == 2) {
                    PushDetectedEvent(detected, table.timeSeconds[i], "Burnout", "burn -> coast", maxDetectedEvents);
                }
            }
            prevStatus = current;
        }

        int startIndex = -1;
        for (std::size_t i = 0; i < status->values.size(); ++i) {
            const int s = static_cast<int>(std::lround(status->values[i]));
            if (s > 0 && s < static_cast<int>(kFlightStatusNames.size())) {
                startIndex = static_cast<int>(i);
                break;
            }
        }
        if (startIndex >= 0) {
            summary.hasFlightTime = true;
            summary.flightStartTime = table.timeSeconds[static_cast<std::size_t>(startIndex)];
            summary.flightEndTime = table.timeSeconds.back();
            summary.flightDuration = summary.flightEndTime - summary.flightStartTime;
        }

        for (std::size_t i = 1; i < status->values.size(); ++i) {
            const float prevS = status->values[i - 1];
            const float dt = table.timeSeconds[i] - table.timeSeconds[i - 1];
            if (!std::isfinite(prevS) || !(dt >= 0.0f && dt < 5.0f)) {
                continue;
            }
            const int phase = static_cast<int>(std::lround(prevS));
            if (phase >= 0 && phase < static_cast<int>(summary.phaseDurations.size())) {
                summary.phaseDurations[static_cast<std::size_t>(phase)] += dt;
            }
        }
    } else {
        summary.hasFlightTime = true;
        summary.flightStartTime = table.timeSeconds.front();
        summary.flightEndTime = table.timeSeconds.back();
        summary.flightDuration = summary.flightEndTime - summary.flightStartTime;
    }

    if (autoCmdDeg != nullptr) {
        float prev = std::numeric_limits<float>::quiet_NaN();
        for (std::size_t i = 0; i < autoCmdDeg->values.size(); ++i) {
            const float cmd = autoCmdDeg->values[i];
            if (!std::isfinite(cmd)) {
                continue;
            }
            if (std::isfinite(prev)) {
                const float delta = std::fabs(cmd - prev);
                if (delta >= 0.5f) {
                    char detail[96];
                    std::snprintf(detail, sizeof(detail), "auto cmd %.2f -> %.2f deg", prev, cmd);
                    PushDetectedEvent(detected, table.timeSeconds[i], "Flap command step", detail, maxDetectedEvents);
                }
            }
            prev = cmd;
        }
    }

    if (isSettling != nullptr) {
        int prev = -1;
        for (std::size_t i = 0; i < isSettling->values.size(); ++i) {
            const float v = isSettling->values[i];
            if (!std::isfinite(v)) {
                continue;
            }
            const int current = (v >= 0.5f) ? 1 : 0;
            if (prev >= 0 && current != prev) {
                PushDetectedEvent(detected,
                                  table.timeSeconds[i],
                                  current ? "Settling start" : "Settling end",
                                  "sensor_actuation_is_settling toggled",
                                  maxDetectedEvents);
            }
            prev = current;
        }
    }

    for (const EventRecord &ev : dataset.events) {
        char detail[144];
        std::snprintf(detail,
                      sizeof(detail),
                      "status=%s alt_ft=%.2f vel_fps=%.2f apogee_ft=%.2f cmd=%.1f eff=%.1f",
                      FlightStatusName(ev.flightStatus),
                      ev.altitudeAglFeet,
                      ev.verticalVelocityFps,
                      ev.apogeeEstimateFeet,
                      ev.flapCommandDeg,
                      ev.flapEffectiveDeg);
        PushDetectedEvent(detected, ev.timestamp, std::string("Log event: ") + EventTypeName(ev.eventType), detail, maxDetectedEvents);
    }

    std::sort(detected.begin(), detected.end(), [](const DetectedEventRow &a, const DetectedEventRow &b) {
        return a.time < b.time;
    });
}

void DrawRawCsvViewer(const LoadedDataset &dataset,
                      bool showHeaderRow,
                      bool unlimitedScroll,
                      int &startLine,
                      int &rowsPerPage,
                      int &jumpToLineInput,
                      float &jumpToTimeInput) {
    if (!dataset.hasRawCsv || dataset.rawCsvLineRanges.empty()) {
        ImGui::TextDisabled("Raw CSV view is only available when a CSV file is loaded.");
        return;
    }

    const int totalLines = static_cast<int>(dataset.rawCsvLineRanges.size());
    const int firstAllowed =
        showHeaderRow ? 0 : std::min<int>(static_cast<int>(dataset.rawDataStartLine), totalLines);
    rowsPerPage = std::max(20, std::min(rowsPerPage, 5000));
    startLine = std::max(firstAllowed, std::min(startLine, std::max(firstAllowed, totalLines - rowsPerPage)));

    ImGui::Text("Total lines: %d", totalLines);
    if (!unlimitedScroll) {
        ImGui::Text("Visible lines: %d..%d", startLine + 1, std::min(totalLines, startLine + rowsPerPage));
        ImGui::InputInt("Rows/Page", &rowsPerPage, 100, 500);
        if (ImGui::Button("Prev Page")) {
            startLine = std::max(firstAllowed, startLine - rowsPerPage);
        }
        ImGui::SameLine();
        if (ImGui::Button("Next Page")) {
            startLine = std::min(std::max(firstAllowed, totalLines - rowsPerPage), startLine + rowsPerPage);
        }
        ImGui::SameLine();
        if (ImGui::Button("Top")) {
            startLine = firstAllowed;
        }
        ImGui::SameLine();
        if (ImGui::Button("Bottom")) {
            startLine = std::max(firstAllowed, totalLines - rowsPerPage);
        }

        jumpToLineInput = std::max(1, std::min(jumpToLineInput, totalLines));
        ImGui::InputInt("Jump to line", &jumpToLineInput, 100, 1000);
        ImGui::SameLine();
        if (ImGui::Button("Jump")) {
            startLine = std::max(firstAllowed, std::min(totalLines - 1, jumpToLineInput - 1));
        }
    } else {
        ImGui::TextDisabled("Unlimited scroll mode: use mouse/trackpad scrollbars.");
    }

    if (!dataset.timeIndex.empty()) {
        ImGui::InputFloat("Jump to time (s)", &jumpToTimeInput, 0.5f, 2.0f, "%.4f");
        ImGui::SameLine();
        if (ImGui::Button("Jump Time")) {
            auto it = std::lower_bound(dataset.timeIndex.begin(),
                                       dataset.timeIndex.end(),
                                       std::make_pair(jumpToTimeInput, static_cast<std::size_t>(0)),
                                       [](const std::pair<float, std::size_t> &a, const std::pair<float, std::size_t> &b) {
                                           return a.first < b.first;
                                       });
            if (it == dataset.timeIndex.end()) {
                it = dataset.timeIndex.end() - 1;
            }
            startLine = static_cast<int>(dataset.rawDataStartLine + it->second);
        }
    }

    auto rainbowColor = [](int column, float alpha) -> ImU32 {
        const float hue = std::fmod(0.12f + static_cast<float>(column) * 0.61803398875f, 1.0f);
        ImVec4 c = ImColor::HSV(hue, 0.35f, 0.95f);
        c.w = alpha;
        return ImGui::ColorConvertFloat4ToU32(c);
    };

    auto splitCsvLineSimple = [](const char *lineBegin, const char *lineEnd, std::vector<CsvCellSpan> &outCells) {
        outCells.clear();
        const char *cellStart = lineBegin;
        for (const char *p = lineBegin; p <= lineEnd; ++p) {
            const bool atEnd = (p == lineEnd);
            if (!atEnd && *p != ',') {
                continue;
            }
            outCells.push_back(CsvCellSpan{cellStart, p});
            cellStart = p + 1;
            if (atEnd) {
                break;
            }
        }
    };

    int columnCount = static_cast<int>(dataset.rawCsvHeaders.size());
    if (columnCount <= 0) {
        const auto headerSpan = dataset.rawCsvLineRanges[0];
        const char *b = dataset.rawCsvText.data() + headerSpan.first;
        const char *e = dataset.rawCsvText.data() + headerSpan.second;
        columnCount = 1;
        for (const char *p = b; p < e; ++p) {
            if (*p == ',') {
                columnCount++;
            }
        }
    }
    columnCount = std::max(1, columnCount);

    ImGui::Separator();
    ImGui::BeginChild("raw_csv_lines", ImVec2(0.0f, 0.0f), true);

    const ImGuiTableFlags tableFlags =
        ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollX | ImGuiTableFlags_ScrollY |
        ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable | ImGuiTableFlags_Hideable |
        ImGuiTableFlags_SizingFixedFit;
    if (ImGui::BeginTable("raw_csv_table", columnCount + 1, tableFlags, ImVec2(0.0f, 0.0f))) {
        ImGui::TableSetupScrollFreeze(1, 1);
        ImGui::TableSetupColumn("#", ImGuiTableColumnFlags_WidthFixed, 72.0f);
        for (int c = 0; c < columnCount; ++c) {
            std::string name = "C" + std::to_string(c);
            if (c < static_cast<int>(dataset.rawCsvHeaders.size()) && !dataset.rawCsvHeaders[c].empty()) {
                name = dataset.rawCsvHeaders[c];
            }
            ImGui::TableSetupColumn(name.c_str(), ImGuiTableColumnFlags_WidthFixed, 180.0f);
        }

        ImGui::TableNextRow(ImGuiTableRowFlags_Headers);
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted("#");
        for (int c = 0; c < columnCount; ++c) {
            ImGui::TableSetColumnIndex(c + 1);
            ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, rainbowColor(c, 0.35f));
            const char *label = nullptr;
            std::string fallback;
            if (c < static_cast<int>(dataset.rawCsvHeaders.size()) && !dataset.rawCsvHeaders[c].empty()) {
                label = dataset.rawCsvHeaders[c].c_str();
            } else {
                fallback = "C" + std::to_string(c);
                label = fallback.c_str();
            }
            ImGui::TextUnformatted(label);
        }

        const int renderStart = unlimitedScroll ? firstAllowed : startLine;
        const int lineEnd = unlimitedScroll ? totalLines : std::min(totalLines, startLine + rowsPerPage);
        const int visibleCount = std::max(0, lineEnd - renderStart);
        std::vector<CsvCellSpan> cells;
        cells.reserve(static_cast<std::size_t>(columnCount));

        ImGuiListClipper clipper;
        clipper.Begin(visibleCount);
        while (clipper.Step()) {
            for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i) {
                const int lineIndex = renderStart + i;
                if (!showHeaderRow && lineIndex == 0) {
                    continue;
                }

                const auto span = dataset.rawCsvLineRanges[static_cast<std::size_t>(lineIndex)];
                const char *lineBegin = dataset.rawCsvText.data() + span.first;
                const char *lineEndPtr = dataset.rawCsvText.data() + span.second;
                splitCsvLineSimple(lineBegin, lineEndPtr, cells);

                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("%d", lineIndex + 1);
                for (int c = 0; c < columnCount; ++c) {
                    ImGui::TableSetColumnIndex(c + 1);
                    ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, rainbowColor(c, 0.14f));
                    if (c < static_cast<int>(cells.size())) {
                        ImGui::TextUnformatted(cells[static_cast<std::size_t>(c)].begin,
                                               cells[static_cast<std::size_t>(c)].end);
                    } else {
                        ImGui::TextUnformatted("");
                    }
                }
            }
        }
        ImGui::EndTable();
    }

    ImGui::EndChild();
}

void ApplyTheme() {
    ImGuiStyle &style = ImGui::GetStyle();
    style.WindowRounding = 10.0f;
    style.ChildRounding = 8.0f;
    style.FrameRounding = 6.0f;
    style.GrabRounding = 6.0f;
    style.FramePadding = ImVec2(8.0f, 5.0f);
    style.ItemSpacing = ImVec2(8.0f, 7.0f);

    ImVec4 *colors = style.Colors;
    colors[ImGuiCol_WindowBg] = ImVec4(0.08f, 0.09f, 0.11f, 1.0f);
    colors[ImGuiCol_ChildBg] = ImVec4(0.12f, 0.13f, 0.16f, 0.96f);
    colors[ImGuiCol_FrameBg] = ImVec4(0.16f, 0.17f, 0.21f, 1.0f);
    colors[ImGuiCol_FrameBgHovered] = ImVec4(0.22f, 0.24f, 0.30f, 1.0f);
    colors[ImGuiCol_Button] = ImVec4(0.22f, 0.37f, 0.53f, 1.0f);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.28f, 0.45f, 0.62f, 1.0f);
    colors[ImGuiCol_ButtonActive] = ImVec4(0.19f, 0.30f, 0.44f, 1.0f);
    colors[ImGuiCol_Header] = ImVec4(0.21f, 0.30f, 0.43f, 0.85f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.25f, 0.36f, 0.51f, 1.0f);
    colors[ImGuiCol_CheckMark] = ImVec4(0.63f, 0.84f, 0.95f, 1.0f);
}

void PrintGuiUsage() {
    std::cout << "Usage: acs_ndrt_rocketry_decoder [input.{BIN|csv}] [--threads N]\n";
}

int RunGuiMain(int argc, char **argv) {
    unsigned threads = std::max(1u, std::thread::hardware_concurrency());
    std::string inputPath;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--threads" && i + 1 < argc) {
            threads = static_cast<unsigned>(std::max(1, std::atoi(argv[++i])));
        } else if (!arg.empty() && arg[0] != '-') {
            inputPath = arg;
        } else {
            PrintGuiUsage();
            return 2;
        }
    }

    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW.\n";
        return 1;
    }

    const char *glslVersion = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    GLFWwindow *window = glfwCreateWindow(1500, 920, "ACS NDRT Rocketry Decoder", nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        std::cerr << "Failed to create GLFW window.\n";
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ApplyTheme();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glslVersion);

    char pathBuffer[1024] = {};
    if (!inputPath.empty()) {
        std::snprintf(pathBuffer, sizeof(pathBuffer), "%s", inputPath.c_str());
    }

    LoadedDataset dataset;
    std::string statusText = "Load a .BIN or .csv file to analyze.";
    std::string errorText;
    float tMin = 0.0f;
    float tMax = 1.0f;
    bool hasRange = false;
    bool drawEventLines = true;
    bool drawReferenceLine = false;
    float referenceValue = 0.0f;
    float chartHeight = 170.0f;
    bool overlaySelectedPlots = true;
    bool normalizeOverlay = false;
    float jumpWindowHalfSpan = 0.6f;
    bool rawShowHeaderRow = true;
    bool rawUnlimitedScroll = true;
    int rawStartLine = 0;
    int rawRowsPerPage = 250;
    int rawJumpToLine = 1;
    float rawJumpToTime = 0.0f;
    SmartParserSettings smartSettings;
    SmartParserReport smartReport;
    char seriesFilter[128] = {};
    SummaryMetrics summary;
    HealthDiagnostics diagnostics;
    std::vector<DetectedEventRow> detectedEvents;
    ReplaySettings replaySettings;
    ReplayAnalysis replayAnalysis;
    std::size_t replayHoveredRow = 0;
    float replayHoveredTime = 0.0f;
    bool replayHasHoveredPoint = false;

    auto loadDataset = [&]() {
        errorText.clear();
        const std::string path = pathBuffer;
        if (path.empty()) {
            errorText = "Input path is empty.";
            return;
        }

        const auto started = std::chrono::steady_clock::now();
        LoadedDataset loaded;
        std::string loadError;
        if (!LoadDatasetFromPath(path, threads, loaded, loadError)) {
            errorText = loadError;
            return;
        }

        dataset = std::move(loaded);
        ApplySmartParserToDataset(dataset, smartSettings, smartReport);
        SelectDefaultSeries(dataset.table);
        AnalyzeDataset(dataset, summary, detectedEvents);
        diagnostics = ComputeHealthDiagnostics(dataset.table);
        std::string replayError;
        if (!ComputeReplayAnalysis(dataset, replaySettings, replayAnalysis, replayError)) {
            replayAnalysis = ReplayAnalysis{};
            replayAnalysis.warnings.push_back(replayError);
        }
        replayHoveredRow = 0;
        replayHoveredTime = 0.0f;
        replayHasHoveredPoint = false;
        rawStartLine = 0;
        rawJumpToLine = 1;
        rawJumpToTime = 0.0f;

        if (!dataset.table.timeSeconds.empty()) {
            tMin = dataset.table.timeSeconds.front();
            tMax = dataset.table.timeSeconds.back();
            if (!(std::isfinite(tMin) && std::isfinite(tMax) && tMax > tMin)) {
                tMin = 0.0f;
                tMax = static_cast<float>(dataset.table.RowCount()) * 0.01f;
            }
            hasRange = true;
        }

        const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                   std::chrono::steady_clock::now() - started)
                                   .count();
        statusText = "Loaded " + dataset.sourcePath.filename().string() + " (" + dataset.sourceType + ") in " +
                     std::to_string(elapsedMs) + " ms" + " | idx: " + dataset.sourcePath.filename().string() + ".idx";
        if (smartSettings.enabled) {
            statusText += " | smart kept " + std::to_string(smartReport.keptRows) + "/" +
                          std::to_string(smartReport.originalRows);
        }
    };

    if (!inputPath.empty()) {
        loadDataset();
    }

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowPos(ImGui::GetMainViewport()->Pos);
        ImGui::SetNextWindowSize(ImGui::GetMainViewport()->Size);
        ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                 ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus;
        ImGui::Begin("ACS NDRT Rocketry Decoder", nullptr, flags);

        ImGui::TextUnformatted("ACS NDRT Rocketry Telemetry Decoder");
        ImGui::SameLine();
        ImGui::TextDisabled("threads=%u", threads);

        ImGui::PushItemWidth(700.0f);
        ImGui::InputText("##input_path", pathBuffer, sizeof(pathBuffer));
        ImGui::PopItemWidth();
        ImGui::SameLine();
        if (ImGui::Button("Load")) {
            loadDataset();
        }

        if (!statusText.empty()) {
            ImGui::TextDisabled("%s", statusText.c_str());
        }
        if (!errorText.empty()) {
            ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.45f, 1.0f), "%s", errorText.c_str());
        }

        ImGui::Separator();

        if (dataset.table.RowCount() == 0) {
            ImGui::TextDisabled("No data loaded.");
            ImGui::End();

            ImGui::Render();
            int displayW = 0;
            int displayH = 0;
            glfwGetFramebufferSize(window, &displayW, &displayH);
            glViewport(0, 0, displayW, displayH);
            glClearColor(0.06f, 0.07f, 0.09f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
            glfwSwapBuffers(window);
            continue;
        }

        if (ImGui::BeginTabBar("decoder_tabs")) {
        if (ImGui::BeginTabItem("Analysis")) {
        const float sidebarWidth = 400.0f;
        ImGui::BeginChild("controls", ImVec2(sidebarWidth, 0.0f), true);

        ImGui::Text("Rows: %zu", dataset.table.RowCount());
        ImGui::Text("Series: %zu", dataset.table.series.size());
        ImGui::Text("Events: %zu", dataset.events.size());
        ImGui::Text("Index points: %zu", dataset.timeIndex.size());
        if (smartSettings.enabled) {
            ImGui::Text("Smart kept: %zu / %zu", smartReport.keptRows, smartReport.originalRows);
        }

        ImGui::SeparatorText("Smart Parser");
        ImGui::Checkbox("Enable Smart Parser (analysis trim)", &smartSettings.enabled);
        if (smartSettings.enabled) {
            ImGui::SliderFloat("Pre-roll (s)", &smartSettings.preSeconds, 0.0f, 30.0f, "%.1f");
            ImGui::SliderFloat("Post-roll (s)", &smartSettings.postSeconds, 0.0f, 60.0f, "%.1f");
            ImGui::InputFloat("Min Alt Delta (ft)", &smartSettings.minAltitudeDeltaFt, 1.0f, 10.0f, "%.1f");
            ImGui::InputFloat("Min |Vz| (ft/s)", &smartSettings.minVelocityFtPerSec, 1.0f, 10.0f, "%.1f");
            ImGui::InputFloat("Min Cmd (deg)", &smartSettings.minCommandDeg, 0.1f, 1.0f, "%.2f");
            if (smartReport.applied) {
                ImGui::TextDisabled("Window rows: [%zu, %zu), ground=%.2f ft",
                                    smartReport.beginRow,
                                    smartReport.endRow,
                                    static_cast<double>(smartReport.groundAltitudeFt));
            }
            ImGui::TextDisabled("Tip: change settings then click Load.");
        }

        if (!dataset.schemaWarnings.empty()) {
            ImGui::SeparatorText("Schema Warnings");
            for (const std::string &w : dataset.schemaWarnings) {
                ImGui::TextColored(ImVec4(0.98f, 0.62f, 0.36f, 1.0f), "%s", w.c_str());
            }
        }
        if (!dataset.metadata.empty()) {
            ImGui::SeparatorText("Metadata");
            for (const auto &kv : dataset.metadata) {
                ImGui::Text("%s: %s", kv.first.c_str(), kv.second.c_str());
            }
        }

        const float globalTMin = dataset.table.timeSeconds.front();
        const float globalTMax = dataset.table.timeSeconds.back();
        if (!hasRange) {
            tMin = globalTMin;
            tMax = globalTMax;
            hasRange = true;
        }

        tMin = std::max(globalTMin, std::min(tMin, tMax));
        tMax = std::min(globalTMax, std::max(tMax, tMin));

        ImGui::SeparatorText("Time Window");
        ImGui::SliderFloat("Start", &tMin, globalTMin, globalTMax, "%.4f s");
        ImGui::SliderFloat("End", &tMax, globalTMin, globalTMax, "%.4f s");
        if (tMin > tMax) {
            std::swap(tMin, tMax);
        }
        if (ImGui::Button("Reset Full Range")) {
            tMin = globalTMin;
            tMax = globalTMax;
        }

        ImGui::SeparatorText("Render");
        ImGui::SliderFloat("Chart Height", &chartHeight, 120.0f, 340.0f, "%.0f px");
        ImGui::SliderFloat("Jump Window +/-", &jumpWindowHalfSpan, 0.1f, 5.0f, "%.2f s");
        ImGui::Checkbox("Event Markers", &drawEventLines);
        ImGui::Checkbox("Overlay Selected", &overlaySelectedPlots);
        if (overlaySelectedPlots) {
            ImGui::Checkbox("Normalize Overlay", &normalizeOverlay);
        }
        ImGui::Checkbox("Reference Line", &drawReferenceLine);
        if (drawReferenceLine) {
            ImGui::InputFloat("Reference Y", &referenceValue, 0.1f, 1.0f, "%.4f");
        }

        ImGui::SeparatorText("Series Select");
        ImGui::InputText("Filter", seriesFilter, sizeof(seriesFilter));
        if (ImGui::Button("Defaults")) {
            SelectDefaultSeries(dataset.table);
        }
        ImGui::SameLine();
        if (ImGui::Button("Clear")) {
            for (auto &series : dataset.table.series) {
                series.selected = false;
            }
        }

        const std::string filterLower = ToLower(seriesFilter);
        ImGui::BeginChild("series_list", ImVec2(0.0f, 230.0f), true);
        const std::array<ColumnClass, 6> classOrder = {
            ColumnClass::Sensor, ColumnClass::State, ColumnClass::Control, ColumnClass::Derived, ColumnClass::Replay, ColumnClass::Unknown};
        for (ColumnClass klass : classOrder) {
            std::string sectionName = std::string(ColumnClassName(klass)) + " Columns";
            if (!ImGui::CollapsingHeader(sectionName.c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
                continue;
            }
            for (auto &series : dataset.table.series) {
                const ColumnMeta meta = ColumnMetadata(series.name);
                if (meta.klass != klass) {
                    continue;
                }
                if (!filterLower.empty()) {
                    const std::string lowered = ToLower(series.name);
                    if (lowered.find(filterLower) == std::string::npos) {
                        continue;
                    }
                }
                ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(ColorForSeries(series.name)));
                ImGui::Checkbox(series.name.c_str(), &series.selected);
                ImGui::PopStyleColor();
                if (ImGui::IsItemHovered()) {
                    ImGui::BeginTooltip();
                    ImGui::Text("type: %s", ColumnClassName(meta.klass));
                    ImGui::Text("units: %s", meta.units);
                    if (std::isfinite(meta.expectedMin) && std::isfinite(meta.expectedMax)) {
                        ImGui::Text("expected: [%.3f, %.3f]", static_cast<double>(meta.expectedMin), static_cast<double>(meta.expectedMax));
                    }
                    ImGui::EndTooltip();
                }
            }
        }
        ImGui::EndChild();

        ImGui::SeparatorText("Summary");
        ImGui::BeginChild("summary", ImVec2(0.0f, 220.0f), true);
        if (summary.hasGroundAltitude) {
            ImGui::Text("Ground Altitude (auto): %.2f ft", static_cast<double>(summary.groundAltitudeFeet));
        } else {
            ImGui::TextDisabled("Ground Altitude: n/a (using raw apogee as AGL fallback)");
        }
        if (summary.hasApogee) {
            ImGui::Text("Apogee AGL: %.2f ft @ %.3f s",
                        static_cast<double>(summary.apogeeAglFeet),
                        static_cast<double>(summary.apogeeTime));
            ImGui::Text("Apogee ASL: %.2f ft", static_cast<double>(summary.apogeeAslFeet));
        } else {
            ImGui::TextDisabled("Apogee: n/a");
        }
        if (summary.hasPredictedApogee) {
            const float predictedAglFt = summary.predictedApogeeMeters * 3.2808399f;
            ImGui::Text("Predicted Apogee (max): %.2f m", static_cast<double>(summary.predictedApogeeMeters));
            ImGui::Text("Predicted Apogee (raw): %.2f ft", static_cast<double>(predictedAglFt));
        } else {
            ImGui::TextDisabled("Predicted Apogee: n/a");
        }
        if (summary.hasMaxVelocity) {
            ImGui::Text("Max |Vertical Velocity|: %.2f ft/s @ %.3f s",
                        static_cast<double>(summary.maxVelocityFps),
                        static_cast<double>(summary.maxVelocityTime));
        } else {
            ImGui::TextDisabled("Max Velocity: n/a");
        }
        if (summary.hasMaxAcceleration) {
            ImGui::Text("Max Accel Magnitude: %.3f @ %.3f s",
                        static_cast<double>(summary.maxAcceleration),
                        static_cast<double>(summary.maxAccelerationTime));
        } else {
            ImGui::TextDisabled("Max Acceleration: n/a");
        }
        if (summary.hasFlightTime) {
            ImGui::Text("Flight Time: %.3f s (%.3f -> %.3f)",
                        static_cast<double>(summary.flightDuration),
                        static_cast<double>(summary.flightStartTime),
                        static_cast<double>(summary.flightEndTime));
        } else {
            ImGui::TextDisabled("Flight Time: n/a");
        }
        ImGui::Separator();
        ImGui::Text("Time in Phase:");
        for (std::size_t i = 0; i < summary.phaseDurations.size(); ++i) {
            ImGui::Text("  %s: %.3f s", FlightStatusName(static_cast<uint8_t>(i)), static_cast<double>(summary.phaseDurations[i]));
        }
        ImGui::EndChild();

        ImGui::SeparatorText("Health Diagnostics");
        ImGui::BeginChild("health", ImVec2(0.0f, 185.0f), true);
        ImGui::Text("Quality Score: %.1f / 100", static_cast<double>(diagnostics.qualityScore));
        if (diagnostics.hasSampleRate) {
            ImGui::Text("Sample Rate: %.2f Hz", static_cast<double>(diagnostics.avgSampleRateHz));
            ImGui::Text("dt min/max: %.5f / %.5f s",
                        static_cast<double>(diagnostics.minDt),
                        static_cast<double>(diagnostics.maxDt));
        } else {
            ImGui::TextDisabled("Sample Rate: n/a");
        }
        ImGui::Text("Time gaps: %zu", diagnostics.gapCount);
        ImGui::Text("Non-monotonic timestamps: %zu", diagnostics.nonMonotonicCount);
        ImGui::Text("Phase jump anomalies: %zu", diagnostics.phaseJumpCount);
        ImGui::Text("NaN values: %zu / %zu",
                    diagnostics.nanCount,
                    diagnostics.totalValues);
        ImGui::EndChild();

        ImGui::SeparatorText("Selected Stats");
        ImGui::BeginChild("stats", ImVec2(0.0f, 140.0f), true);
        std::size_t statsPrinted = 0;
        for (const auto &series : dataset.table.series) {
            if (!series.selected) {
                continue;
            }
            SeriesStats stats;
            if (!ComputeSeriesStats(dataset.table, series, tMin, tMax, stats)) {
                continue;
            }
            ImGui::Text("%s", series.name.c_str());
            ImGui::TextDisabled("n=%zu  min=%.4f  max=%.4f  mean=%.4f  std=%.4f",
                                stats.samples,
                                static_cast<double>(stats.minValue),
                                static_cast<double>(stats.maxValue),
                                static_cast<double>(stats.meanValue),
                                static_cast<double>(stats.stddevValue));
            statsPrinted++;
            if (statsPrinted >= 8) {
                break;
            }
        }
        if (statsPrinted == 0) {
            ImGui::TextDisabled("Select one or more series to show windowed stats.");
        }
        ImGui::EndChild();

        if (!dataset.events.empty()) {
            ImGui::SeparatorText("Events");
            ImGui::BeginChild("events", ImVec2(0.0f, 190.0f), true);
            for (const EventRecord &event : dataset.events) {
                ImGui::Text("t=%.4f  %s  alt_ft=%.2f  vel_fps=%.2f  cmd=%.1f  eff=%.1f",
                            static_cast<double>(event.timestamp),
                            EventTypeName(event.eventType),
                            static_cast<double>(event.altitudeAglFeet),
                            static_cast<double>(event.verticalVelocityFps),
                            static_cast<double>(event.flapCommandDeg),
                            static_cast<double>(event.flapEffectiveDeg));
            }
            ImGui::EndChild();
        }

        ImGui::SeparatorText("Smart Events");
        ImGui::BeginChild("smart_events", ImVec2(0.0f, 240.0f), true);
        if (detectedEvents.empty()) {
            ImGui::TextDisabled("No auto-detected events.");
        } else {
            for (std::size_t i = 0; i < detectedEvents.size(); ++i) {
                const DetectedEventRow &ev = detectedEvents[i];
                ImGui::PushID(static_cast<int>(i));
                ImGui::Text("%.3f s  %s", static_cast<double>(ev.time), ev.label.c_str());
                if (!ev.detail.empty()) {
                    ImGui::TextDisabled("%s", ev.detail.c_str());
                }
                ImGui::SameLine();
                if (ImGui::Button("Jump")) {
                    const float globalTMinClamp = dataset.table.timeSeconds.front();
                    const float globalTMaxClamp = dataset.table.timeSeconds.back();
                    tMin = std::max(globalTMinClamp, ev.time - jumpWindowHalfSpan);
                    tMax = std::min(globalTMaxClamp, ev.time + jumpWindowHalfSpan);
                    if (tMax < tMin) {
                        std::swap(tMin, tMax);
                    }
                }
                ImGui::Separator();
                ImGui::PopID();
                if (i >= 199) {
                    ImGui::TextDisabled("Showing first 200 events (of %zu).", detectedEvents.size());
                    break;
                }
            }
        }
        ImGui::EndChild();

        ImGui::EndChild();

        ImGui::SameLine();

        ImGui::BeginChild("plots", ImVec2(0.0f, 0.0f), true);
        std::size_t selectedCount = 0;
        std::vector<const NumericSeries *> selectedSeries;
        selectedSeries.reserve(dataset.table.series.size());
        for (const auto &series : dataset.table.series) {
            if (series.selected) {
                selectedCount++;
                selectedSeries.push_back(&series);
            }
        }

        if (selectedCount == 0) {
            ImGui::TextDisabled("No selected series. Pick fields from the left panel.");
        } else if (overlaySelectedPlots && selectedCount >= 2) {
            DrawOverlayChart(dataset.table,
                             selectedSeries,
                             dataset.events,
                             tMin,
                             tMax,
                             drawEventLines,
                             drawReferenceLine,
                             referenceValue,
                             normalizeOverlay,
                             chartHeight,
                             nullptr,
                             nullptr,
                             nullptr);
            ImGui::Dummy(ImVec2(0.0f, 8.0f));
            ImGui::BeginChild("overlay_legend", ImVec2(0.0f, 90.0f), true);
            for (const NumericSeries *series : selectedSeries) {
                ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(ColorForSeries(series->name)));
                ImGui::BulletText("%s", series->name.c_str());
                ImGui::PopStyleColor();
            }
            ImGui::EndChild();
        } else {
            for (const auto &series : dataset.table.series) {
                if (!series.selected) {
                    continue;
                }
                DrawSeriesChart(dataset.table,
                                series,
                                dataset.events,
                                tMin,
                                tMax,
                                drawEventLines,
                                drawReferenceLine,
                                referenceValue,
                                chartHeight);
                ImGui::Dummy(ImVec2(0.0f, 6.0f));
            }
        }
        ImGui::EndChild();
        ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Replay")) {
            ImGui::BeginChild("replay_controls", ImVec2(380.0f, 0.0f), true);
            ImGui::Text("In-process ACS replay from the loaded telemetry table");
            ImGui::Checkbox("Use CFD Table", &replaySettings.useCfd);
            ImGui::InputText("CFD Path", replaySettings.cfdPath, sizeof(replaySettings.cfdPath));
            ImGui::SliderFloat("Refresh Interval (s)", &replaySettings.refreshIntervalSeconds, 0.02f, 0.5f, "%.2f");
            ImGui::SliderInt("Predictor Max Steps", &replaySettings.maxPredictorSteps, 32, 512);
            ImGui::SeparatorText("Series");
            ImGui::Checkbox("Logged State Apogee", &replaySettings.showLoggedStateApogee);
            ImGui::Checkbox("Logged Optimizer Apogee", &replaySettings.showLoggedOptimizerApogee);
            ImGui::Checkbox("Replay Safer Apogee", &replaySettings.showReplaySaferApogee);
            ImGui::Checkbox("Replay Ballistic", &replaySettings.showReplayBallisticApogee);
            ImGui::Checkbox("Error vs Actual", &replaySettings.showReplayErrorVsActual);
            ImGui::Checkbox("Error vs Logged State", &replaySettings.showReplayErrorVsState);
            ImGui::Checkbox("Error vs Logged Optimizer", &replaySettings.showReplayErrorVsOptimizer);
            ImGui::Checkbox("Seed Horizontal Speed", &replaySettings.showSeedHorizontalSpeed);
            ImGui::Checkbox("Seed Zenith", &replaySettings.showSeedZenith);
            ImGui::Checkbox("Seed Angular Rate", &replaySettings.showSeedAngularRate);
            ImGui::SeparatorText("Display");
            ImGui::Checkbox("Normalize Seed Plots", &replaySettings.normalizeSeedPlots);
            ImGui::Checkbox("Normalize Error Plot", &replaySettings.normalizeErrorPlots);
            ImGui::Checkbox("Show Raw Inspector", &replaySettings.showRawInspector);
            if (ImGui::Button("Recompute Replay")) {
                std::string replayError;
                if (!ComputeReplayAnalysis(dataset, replaySettings, replayAnalysis, replayError)) {
                    replayAnalysis = ReplayAnalysis{};
                    replayAnalysis.warnings.push_back(replayError);
                }
                replayHoveredRow = 0;
                replayHoveredTime = 0.0f;
                replayHasHoveredPoint = false;
            }
            if (!replayAnalysis.statusText.empty()) {
                ImGui::TextDisabled("%s", replayAnalysis.statusText.c_str());
            }
            if (std::isfinite(replayAnalysis.actualApogeeMeters)) {
                ImGui::Text("Actual Max Altitude: %.2f m", static_cast<double>(replayAnalysis.actualApogeeMeters));
            }
            if (std::isfinite(replayAnalysis.replayApogeeMeanMeters)) {
                ImGui::Text("Replay Mean Apogee: %.2f m", static_cast<double>(replayAnalysis.replayApogeeMeanMeters));
            }
            if (std::isfinite(replayAnalysis.replayBallisticMeanMeters)) {
                ImGui::Text("Ballistic Mean Apogee: %.2f m", static_cast<double>(replayAnalysis.replayBallisticMeanMeters));
            }
            if (std::isfinite(replayAnalysis.replayApogeeMinMeters) && std::isfinite(replayAnalysis.replayApogeeMaxMeters)) {
                ImGui::Text("Replay Range: %.2f .. %.2f m",
                            static_cast<double>(replayAnalysis.replayApogeeMinMeters),
                            static_cast<double>(replayAnalysis.replayApogeeMaxMeters));
            }
            if (replayHasHoveredPoint) {
                ImGui::SeparatorText("Hover");
                ImGui::Text("Row: %zu", replayHoveredRow);
                ImGui::Text("Time: %.4f s", static_cast<double>(replayHoveredTime));
            }
            if (!replayAnalysis.warnings.empty()) {
                ImGui::SeparatorText("Replay Warnings");
                for (const std::string &warning : replayAnalysis.warnings) {
                    ImGui::TextColored(ImVec4(0.98f, 0.62f, 0.36f, 1.0f), "%s", warning.c_str());
                }
            }
            ImGui::EndChild();

            ImGui::SameLine();
            ImGui::BeginChild("replay_plots", ImVec2(0.0f, 0.0f), true);
            if (!replayAnalysis.ready || replayAnalysis.table.RowCount() == 0) {
                ImGui::TextDisabled("Replay results are not available.");
            } else {
                std::vector<const NumericSeries *> apogeeSeries;
                std::vector<const NumericSeries *> errorSeries;
                std::vector<const NumericSeries *> seedSeries;
                if (const NumericSeries *series = FindSeriesByName(replayAnalysis.table, "derived_replay_altitude_m"); series != nullptr) {
                    apogeeSeries.push_back(series);
                }
                if (replaySettings.showLoggedStateApogee) {
                    if (const NumericSeries *series = FindSeriesByName(replayAnalysis.table, "derived_replay_logged_state_apogee_m"); series != nullptr) {
                        apogeeSeries.push_back(series);
                    }
                }
                if (replaySettings.showLoggedOptimizerApogee) {
                    if (const NumericSeries *series = FindSeriesByName(replayAnalysis.table, "derived_replay_logged_optimizer_apogee_m"); series != nullptr) {
                        apogeeSeries.push_back(series);
                    }
                }
                if (replaySettings.showReplaySaferApogee) {
                    if (const NumericSeries *series = FindSeriesByName(replayAnalysis.table, "derived_replay_apogee_safer_m"); series != nullptr) {
                        apogeeSeries.push_back(series);
                    }
                }
                if (replaySettings.showReplayBallisticApogee) {
                    if (const NumericSeries *series = FindSeriesByName(replayAnalysis.table, "derived_replay_apogee_ballistic_m"); series != nullptr) {
                        apogeeSeries.push_back(series);
                    }
                }
                if (replaySettings.showReplayErrorVsActual) {
                    if (const NumericSeries *series = FindSeriesByName(replayAnalysis.table, "derived_replay_error_vs_actual_m"); series != nullptr) {
                        errorSeries.push_back(series);
                    }
                }
                if (replaySettings.showReplayErrorVsState) {
                    if (const NumericSeries *series = FindSeriesByName(replayAnalysis.table, "derived_replay_error_vs_logged_state_m"); series != nullptr) {
                        errorSeries.push_back(series);
                    }
                }
                if (replaySettings.showReplayErrorVsOptimizer) {
                    if (const NumericSeries *series = FindSeriesByName(replayAnalysis.table, "derived_replay_error_vs_logged_optimizer_m"); series != nullptr) {
                        errorSeries.push_back(series);
                    }
                }
                if (replaySettings.showSeedHorizontalSpeed) {
                    if (const NumericSeries *series = FindSeriesByName(replayAnalysis.table, "derived_replay_seed_horizontal_speed_mps"); series != nullptr) {
                        seedSeries.push_back(series);
                    }
                }
                if (replaySettings.showSeedZenith) {
                    if (const NumericSeries *series = FindSeriesByName(replayAnalysis.table, "derived_replay_seed_zenith_deg"); series != nullptr) {
                        seedSeries.push_back(series);
                    }
                }
                if (replaySettings.showSeedAngularRate) {
                    if (const NumericSeries *series = FindSeriesByName(replayAnalysis.table, "derived_replay_seed_angular_rate_deg_s"); series != nullptr) {
                        seedSeries.push_back(series);
                    }
                }

                const float replayRef = std::isfinite(replayAnalysis.actualApogeeMeters) ? replayAnalysis.actualApogeeMeters : referenceValue;
                replayHasHoveredPoint = false;
                if (!apogeeSeries.empty()) {
                    DrawOverlayChart(replayAnalysis.table,
                                     apogeeSeries,
                                     dataset.events,
                                     tMin,
                                     tMax,
                                     drawEventLines,
                                     std::isfinite(replayAnalysis.actualApogeeMeters),
                                     replayRef,
                                     false,
                                     chartHeight,
                                     &replayHasHoveredPoint,
                                     &replayHoveredRow,
                                     &replayHoveredTime);
                    ImGui::Dummy(ImVec2(0.0f, 8.0f));
                }
                if (!errorSeries.empty()) {
                    bool errorHovered = false;
                    DrawOverlayChart(replayAnalysis.table,
                                     errorSeries,
                                     dataset.events,
                                     tMin,
                                     tMax,
                                     false,
                                     true,
                                     0.0f,
                                     replaySettings.normalizeErrorPlots,
                                     chartHeight * 0.75f,
                                     &errorHovered,
                                     &replayHoveredRow,
                                     &replayHoveredTime);
                    replayHasHoveredPoint = replayHasHoveredPoint || errorHovered;
                    ImGui::Dummy(ImVec2(0.0f, 8.0f));
                }
                if (!seedSeries.empty()) {
                    bool seedHovered = false;
                    DrawOverlayChart(replayAnalysis.table,
                                     seedSeries,
                                     dataset.events,
                                     tMin,
                                     tMax,
                                     false,
                                     false,
                                     0.0f,
                                     replaySettings.normalizeSeedPlots,
                                     chartHeight * 0.8f,
                                     &seedHovered,
                                     &replayHoveredRow,
                                     &replayHoveredTime);
                    replayHasHoveredPoint = replayHasHoveredPoint || seedHovered;
                }
                if (replaySettings.showRawInspector && replayHasHoveredPoint && replayHoveredRow < replayAnalysis.table.RowCount() &&
                    replayHoveredRow < dataset.table.RowCount()) {
                    ImGui::Dummy(ImVec2(0.0f, 8.0f));
                    ImGui::SeparatorText("Hovered Sample");
                    ImGui::BeginChild("replay_hover_inspector", ImVec2(0.0f, 240.0f), true);
                    ImGui::Text("Row %zu  t=%.4f s", replayHoveredRow, static_cast<double>(replayHoveredTime));

                    const NumericSeries *altitude = FindSeriesByName(replayAnalysis.table, "derived_replay_altitude_m");
                    const NumericSeries *saferApogee = FindSeriesByName(replayAnalysis.table, "derived_replay_apogee_safer_m");
                    const NumericSeries *ballisticApogee = FindSeriesByName(replayAnalysis.table, "derived_replay_apogee_ballistic_m");
                    const NumericSeries *stateApogee = FindSeriesByName(replayAnalysis.table, "derived_replay_logged_state_apogee_m");
                    const NumericSeries *optimizerApogee = FindSeriesByName(replayAnalysis.table, "derived_replay_logged_optimizer_apogee_m");
                    const NumericSeries *errorActual = FindSeriesByName(replayAnalysis.table, "derived_replay_error_vs_actual_m");
                    const NumericSeries *errorState = FindSeriesByName(replayAnalysis.table, "derived_replay_error_vs_logged_state_m");
                    const NumericSeries *errorOptimizer = FindSeriesByName(replayAnalysis.table, "derived_replay_error_vs_logged_optimizer_m");
                    const NumericSeries *seedHorizontal = FindSeriesByName(replayAnalysis.table, "derived_replay_seed_horizontal_speed_mps");
                    const NumericSeries *seedZenith = FindSeriesByName(replayAnalysis.table, "derived_replay_seed_zenith_deg");
                    const NumericSeries *seedRate = FindSeriesByName(replayAnalysis.table, "derived_replay_seed_angular_rate_deg_s");
                    const SeriesLookup sourceVz = FindSeriesByNamesWithScale(
                        dataset.table,
                        {{"state_vertical_velocity_fps", kFeetToMeters}, {"state_velocity_z", 1.0f}});
                    const SeriesLookup sourceZenith = FindSeriesByNamesWithScale(
                        dataset.table,
                        {{"state_zenith_deg", kDegToRad}, {"state_zenith", 1.0f}});
                    const NumericSeries *sourceStatus = FindSeriesByName(dataset.table, "flight_status_raw");
                    const NumericSeries *sourceFlags = FindSeriesByName(dataset.table, "sensor_predictor_seed_confidence_flags");

                    ImGui::Columns(2, "replay_hover_columns", false);
                    ImGui::TextUnformatted("Replay");
                    ImGui::Separator();
                    ImGui::Text("Altitude: %s", FormatOptionalFloat(SeriesValueAt(altitude, replayHoveredRow), "m").c_str());
                    ImGui::Text("Safer Apogee: %s", FormatOptionalFloat(SeriesValueAt(saferApogee, replayHoveredRow), "m").c_str());
                    ImGui::Text("Ballistic Apogee: %s", FormatOptionalFloat(SeriesValueAt(ballisticApogee, replayHoveredRow), "m").c_str());
                    ImGui::Text("State Apogee: %s", FormatOptionalFloat(SeriesValueAt(stateApogee, replayHoveredRow), "m").c_str());
                    ImGui::Text("Optimizer Apogee: %s", FormatOptionalFloat(SeriesValueAt(optimizerApogee, replayHoveredRow), "m").c_str());
                    ImGui::Text("Err vs Actual: %s", FormatOptionalFloat(SeriesValueAt(errorActual, replayHoveredRow), "m").c_str());
                    ImGui::Text("Err vs State: %s", FormatOptionalFloat(SeriesValueAt(errorState, replayHoveredRow), "m").c_str());
                    ImGui::Text("Err vs Optimizer: %s", FormatOptionalFloat(SeriesValueAt(errorOptimizer, replayHoveredRow), "m").c_str());
                    ImGui::Text("Seed Horizontal: %s", FormatOptionalFloat(SeriesValueAt(seedHorizontal, replayHoveredRow), "m/s").c_str());
                    ImGui::Text("Seed Zenith: %s", FormatOptionalFloat(SeriesValueAt(seedZenith, replayHoveredRow), "deg").c_str());
                    ImGui::Text("Seed Rate: %s", FormatOptionalFloat(SeriesValueAt(seedRate, replayHoveredRow), "deg/s").c_str());

                    ImGui::NextColumn();
                    ImGui::TextUnformatted("Source");
                    ImGui::Separator();
                    ImGui::Text("Flight Status: %s", FormatOptionalFloat(SeriesValueAt(sourceStatus, replayHoveredRow)).c_str());
                    const std::optional<float> sourceVzValue =
                        SeriesValueAt(sourceVz.series, replayHoveredRow).has_value()
                            ? std::optional<float>(*SeriesValueAt(sourceVz.series, replayHoveredRow) * sourceVz.scale)
                            : std::nullopt;
                    const std::optional<float> sourceZenithValue =
                        SeriesValueAt(sourceZenith.series, replayHoveredRow).has_value()
                            ? std::optional<float>(*SeriesValueAt(sourceZenith.series, replayHoveredRow) * sourceZenith.scale)
                            : std::nullopt;
                    ImGui::Text("Vertical Velocity: %s", FormatOptionalFloat(sourceVzValue, "m/s").c_str());
                    ImGui::Text("Predictor Flags: %s", FormatOptionalFloat(SeriesValueAt(sourceFlags, replayHoveredRow)).c_str());
                    ImGui::Text("State X Vel: %s", FormatOptionalFloat(SeriesValueAt(FindSeriesByName(dataset.table, "state_velocity_x"), replayHoveredRow), "m/s").c_str());
                    ImGui::Text("State Y Vel: %s", FormatOptionalFloat(SeriesValueAt(FindSeriesByName(dataset.table, "state_velocity_y"), replayHoveredRow), "m/s").c_str());
                    ImGui::Text("Inertial Ax: %s", FormatOptionalFloat(SeriesValueAt(FindSeriesByName(dataset.table, "state_inertial_acceleration_x"), replayHoveredRow), "m/s^2").c_str());
                    ImGui::Text("Inertial Ay: %s", FormatOptionalFloat(SeriesValueAt(FindSeriesByName(dataset.table, "state_inertial_acceleration_y"), replayHoveredRow), "m/s^2").c_str());
                    ImGui::Text("Zenith Raw: %s", FormatOptionalFloat(sourceZenithValue, "rad").c_str());
                    ImGui::Text("Auto Cmd: %s", FormatOptionalFloat(SeriesValueAt(FindSeriesByName(dataset.table, "sensor_auto_cmd_deg"), replayHoveredRow), "deg").c_str());
                    ImGui::Text("Altimeter: %s", FormatOptionalFloat(SeriesValueAt(FindSeriesByName(dataset.table, "sensor_altitude_feet"), replayHoveredRow), "ft").c_str());
                    ImGui::Columns(1);
                    ImGui::EndChild();
                }
            }
            ImGui::EndChild();
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Raw CSV")) {
            ImGui::Checkbox("Show Header Row", &rawShowHeaderRow);
            ImGui::SameLine();
            ImGui::Checkbox("Unlimited Scroll", &rawUnlimitedScroll);
            if (smartSettings.enabled) {
                ImGui::TextDisabled("Raw CSV is untrimmed; smart parser affects analysis plots/stats.");
            }
            if (!rawShowHeaderRow && rawStartLine == 0) {
                rawStartLine = 1;
            }
            DrawRawCsvViewer(dataset,
                             rawShowHeaderRow,
                             rawUnlimitedScroll,
                             rawStartLine,
                             rawRowsPerPage,
                             rawJumpToLine,
                             rawJumpToTime);
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
        }

        ImGui::End();

        ImGui::Render();
        int displayW = 0;
        int displayH = 0;
        glfwGetFramebufferSize(window, &displayW, &displayH);
        glViewport(0, 0, displayW, displayH);
        glClearColor(0.05f, 0.06f, 0.08f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}

#endif  // ACS_ENABLE_IMGUI_DECODER

void PrintUsage() {
    std::cout << "Usage: acs_ndrt_rocketry_fast_decode <input.BIN> -o <output.csv> [--events-output <events.json>] [--no-events-output] [--threads N]\n"
              << "       [--smart-parser] [--smart-pre-seconds S] [--smart-post-seconds S]\n"
              << "       [--smart-min-alt-ft F] [--smart-min-vel-ftps F] [--smart-min-cmd-deg F]\n";
}

int RunCliMain(int argc, char **argv) {
    if (argc < 2) {
        PrintUsage();
        return 2;
    }

    fs::path inputPath;
    fs::path telemetryOut;
    fs::path eventsOut;
    bool noEvents = false;
    unsigned threads = std::max(1u, std::thread::hardware_concurrency());
    bool smartParser = false;
    float smartPreSeconds = 3.0f;
    float smartPostSeconds = 10.0f;
    float smartMinAltFt = 25.0f;
    float smartMinVelFtps = 20.0f;
    float smartMinCmdDeg = 0.5f;

    inputPath = fs::path(argv[1]);
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if ((arg == "-o" || arg == "--telemetry-output") && i + 1 < argc) {
            telemetryOut = fs::path(argv[++i]);
        } else if (arg == "--events-output" && i + 1 < argc) {
            eventsOut = fs::path(argv[++i]);
        } else if (arg == "--no-events-output") {
            noEvents = true;
        } else if (arg == "--threads" && i + 1 < argc) {
            threads = static_cast<unsigned>(std::max(1, std::atoi(argv[++i])));
        } else if (arg == "--smart-parser") {
            smartParser = true;
        } else if (arg == "--smart-pre-seconds" && i + 1 < argc) {
            smartParser = true;
            smartPreSeconds = static_cast<float>(std::atof(argv[++i]));
        } else if (arg == "--smart-post-seconds" && i + 1 < argc) {
            smartParser = true;
            smartPostSeconds = static_cast<float>(std::atof(argv[++i]));
        } else if (arg == "--smart-min-alt-ft" && i + 1 < argc) {
            smartParser = true;
            smartMinAltFt = static_cast<float>(std::atof(argv[++i]));
        } else if (arg == "--smart-min-vel-ftps" && i + 1 < argc) {
            smartParser = true;
            smartMinVelFtps = static_cast<float>(std::atof(argv[++i]));
        } else if (arg == "--smart-min-cmd-deg" && i + 1 < argc) {
            smartParser = true;
            smartMinCmdDeg = static_cast<float>(std::atof(argv[++i]));
        } else {
            std::cerr << "Unknown/invalid argument: " << arg << "\n";
            PrintUsage();
            return 2;
        }
    }

    if (telemetryOut.empty()) {
        std::cerr << "Missing required output path (-o <output.csv>).\n";
        return 2;
    }
    if (noEvents) {
        eventsOut.clear();
    } else if (eventsOut.empty()) {
        eventsOut = inputPath.parent_path() / (inputPath.stem().string() + "_events.json");
    }

    std::vector<uint8_t> bytes;
    std::vector<TelemetryRecordRef> telemetry;
    std::vector<EventRecord> events;
    std::string error;

    if (!LoadFile(inputPath, bytes, error)) {
        std::cerr << error << "\n";
        return 1;
    }
    std::unordered_map<std::string, std::string> binMetadata;
    std::vector<std::string> binWarnings;
    if (!ParseLog(bytes, telemetry, events, &binMetadata, &binWarnings, error)) {
        std::cerr << error << "\n";
        return 1;
    }

    std::size_t keptBegin = 0;
    std::size_t keptEnd = telemetry.size();
    if (smartParser && !telemetry.empty()) {
        const std::size_t preRows = EstimateRowsForSeconds(telemetry, std::max(0.0f, smartPreSeconds));
        const std::size_t postRows = EstimateRowsForSeconds(telemetry, std::max(0.0f, smartPostSeconds));
        if (ComputeSmartActiveWindow(telemetry,
                                     std::max(0.0f, smartMinAltFt),
                                     std::max(0.0f, smartMinVelFtps),
                                     std::max(0.0f, smartMinCmdDeg),
                                     preRows,
                                     postRows,
                                     keptBegin,
                                     keptEnd)) {
            std::vector<TelemetryRecordRef> trimmed(telemetry.begin() + static_cast<std::ptrdiff_t>(keptBegin),
                                                    telemetry.begin() + static_cast<std::ptrdiff_t>(keptEnd));
            telemetry.swap(trimmed);

            const float t0 = SensorFloat(telemetry.front(), 0);
            const float t1 = SensorFloat(telemetry.back(), 0);
            std::vector<EventRecord> trimmedEvents;
            trimmedEvents.reserve(events.size());
            for (const auto &ev : events) {
                if (std::isfinite(ev.timestamp) && ev.timestamp >= t0 && ev.timestamp <= t1) {
                    trimmedEvents.push_back(ev);
                }
            }
            events.swap(trimmedEvents);
        }
    }
    if (!WriteTelemetryCsvParallel(telemetryOut, telemetry, threads, error)) {
        std::cerr << error << "\n";
        return 1;
    }
    if (!eventsOut.empty() && !WriteEventsJson(eventsOut, events, error)) {
        std::cerr << error << "\n";
        return 1;
    }

    std::cout << "Decoded telemetry records: " << telemetry.size() << "\n";
    std::cout << "Decoded event records: " << events.size() << "\n";
    if (smartParser) {
        std::cout << "Smart parser: enabled\n";
        std::cout << "Kept record window: [" << keptBegin << ", " << keptEnd << ")\n";
    }
    std::cout << "CSV output: " << telemetryOut << "\n";
    if (!binWarnings.empty()) {
        for (const auto &w : binWarnings) {
            std::cout << "Warning: " << w << "\n";
        }
    }
    if (!eventsOut.empty()) {
        std::cout << "Events output: " << eventsOut << "\n";
    }
    return 0;
}

}  // namespace

int main(int argc, char **argv) {
#if defined(ACS_ENABLE_IMGUI_DECODER)
    return RunGuiMain(argc, argv);
#else
    return RunCliMain(argc, argv);
#endif
}
