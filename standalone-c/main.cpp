#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "include/Arduino.h"

#include "../cm5/src/constants.h"
#include "../cm5/src/flight_computer.h"
#include "../cm5/src/apogee_model.h"
#include <matplot/matplot.h>

namespace {

bool CaseInsensitiveEquals(const std::string &a, const std::string &b);
enum class SampleValueId;

enum class FieldId {
    Timestamp,
    AltitudeFeet,
    AltitudeMeters,
    AccelBnoX,
    AccelBnoY,
    AccelBnoZ,
    AccelIcmX,
    AccelIcmY,
    AccelIcmZ,
    GyroX,
    GyroY,
    GyroZ,
    QuatW,
    QuatX,
    QuatY,
    QuatZ,
    HasQuaternion,
};

struct FieldInfo {
    FieldId id;
    const char *cliName;
    std::vector<std::string> aliases;
};

const std::vector<FieldInfo> &AllFields() {
    static const std::vector<FieldInfo> kFields = {
        {FieldId::Timestamp, "timestamp", {"timestamp", "time", "t", "time_s", "sensor_timestamp"}},
        {FieldId::AltitudeFeet,
         "altitude_feet",
         {"altitude_ft", "altitude_feet", "altitude", "sensor_altitude_feet"}},
        {FieldId::AltitudeMeters,
         "altitude_meters",
         {"altitude_m", "altitude_meters", "alt_m", "sensor_altitude_meters"}},
        {FieldId::AccelBnoX,
         "accel_bno_x",
         {"accel_bno_x", "bno_ax", "accel_x_bno", "sensor_accel_bno_x"}},
        {FieldId::AccelBnoY,
         "accel_bno_y",
         {"accel_bno_y", "bno_ay", "accel_y_bno", "sensor_accel_bno_y"}},
        {FieldId::AccelBnoZ,
         "accel_bno_z",
         {"accel_bno_z", "bno_az", "accel_z_bno", "sensor_accel_bno_z"}},
        {FieldId::AccelIcmX,
         "accel_icm_x",
         {"accel_icm_x", "icm_ax", "accel_x", "sensor_accel_icm_x"}},
        {FieldId::AccelIcmY,
         "accel_icm_y",
         {"accel_icm_y", "icm_ay", "accel_y", "sensor_accel_icm_y"}},
        {FieldId::AccelIcmZ,
         "accel_icm_z",
         {"accel_icm_z", "icm_az", "accel_z", "sensor_accel_icm_z"}},
        {FieldId::GyroX, "gyro_x", {"gyro_x", "gx", "sensor_gyro_x"}},
        {FieldId::GyroY, "gyro_y", {"gyro_y", "gy", "sensor_gyro_y"}},
        {FieldId::GyroZ, "gyro_z", {"gyro_z", "gz", "sensor_gyro_z"}},
        {FieldId::QuatW, "quat_w", {"quat_w", "qw", "sensor_quat_w"}},
        {FieldId::QuatX, "quat_x", {"quat_x", "qx", "sensor_quat_x"}},
        {FieldId::QuatY, "quat_y", {"quat_y", "qy", "sensor_quat_y"}},
        {FieldId::QuatZ, "quat_z", {"quat_z", "qz", "sensor_quat_z"}},
        {FieldId::HasQuaternion,
         "has_quaternion",
         {"has_quaternion", "quat_valid", "sensor_has_quaternion"}},
    };
    return kFields;
}

const FieldInfo *FindFieldById(FieldId id) {
    for (const auto &field : AllFields()) {
        if (field.id == id) {
            return &field;
        }
    }
    return nullptr;
}

const FieldInfo *FindFieldByName(const std::string &name) {
    for (const auto &field : AllFields()) {
        if (CaseInsensitiveEquals(field.cliName, name)) {
            return &field;
        }
    }
    return nullptr;
}

struct FieldIdHash {
    std::size_t operator()(FieldId id) const noexcept { return static_cast<std::size_t>(id); }
};

using FieldOverrideMap = std::unordered_map<FieldId, std::string, FieldIdHash>;

struct ProgramOptions {
    std::string csvPath;
    bool showHelp = false;
    bool quiet = false;
    float sigmaAccelXY = 0.5f;
    float sigmaAccelZ = 0.5f;
    float sigmaAltimeter = 0.5f;
    float processXY = 0.5f;
    float processZ = 1.0f;
    float apogeeTargetMeters = 1550.0f;
    FieldOverrideMap fieldOverrides;
    std::vector<SampleValueId> extraOutputFields;
    std::vector<SampleValueId> graphFields;
};

struct FieldIndices {
    std::optional<std::size_t> timestamp;
    std::optional<std::size_t> altitudeFeet;
    std::optional<std::size_t> altitudeMeters;
    std::array<std::optional<std::size_t>, 3> accelBno{};
    std::array<std::optional<std::size_t>, 3> accelIcm{};
    std::array<std::optional<std::size_t>, 3> gyro{};
    std::array<std::optional<std::size_t>, 4> quaternion{};
    std::optional<std::size_t> hasQuaternionFlag;
};

std::string ToLower(std::string value) {
    for (char &ch : value) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return value;
}

bool CaseInsensitiveEquals(const std::string &a, const std::string &b) {
    if (a.size() != b.size()) {
        return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (std::tolower(static_cast<unsigned char>(a[i])) != std::tolower(static_cast<unsigned char>(b[i]))) {
            return false;
        }
    }
    return true;
}

std::string Trim(const std::string &value) {
    std::size_t start = 0;
    while (start < value.size() && std::isspace(static_cast<unsigned char>(value[start]))) {
        ++start;
    }
    if (start == value.size()) {
        return std::string();
    }
    std::size_t end = value.size() - 1;
    while (end > start && std::isspace(static_cast<unsigned char>(value[end]))) {
        --end;
    }
    return value.substr(start, end - start + 1);
}

std::string SanitizeFilename(const std::string &value) {
    std::string sanitized;
    sanitized.reserve(value.size());
    for (char ch : value) {
        if (std::isalnum(static_cast<unsigned char>(ch))) {
            sanitized.push_back(ch);
        } else if (ch == '-' || ch == '_') {
            sanitized.push_back(ch);
        } else {
            sanitized.push_back('_');
        }
    }
    if (sanitized.empty()) {
        sanitized = "graph";
    }
    return sanitized;
}

enum class SampleValueId {
    AltitudeMeters,
    VelocityMetersPerSecond,
    ApogeePredictionMeters,
    AltimeterRawMeters,
    AccelIcmX,
    AccelIcmY,
    AccelIcmZ,
    AccelBnoX,
    AccelBnoY,
    AccelBnoZ,
    GyroX,
    GyroY,
    GyroZ,
};

struct SampleValueInfo {
    SampleValueId id;
    const char *cliName;
    std::vector<std::string> aliases;
    const char *description;
    bool allowCsv;
    bool allowGraph;
};

const std::vector<SampleValueInfo> &AllSampleValues() {
    static const std::vector<SampleValueInfo> kValues = {
        {SampleValueId::AltitudeMeters,
         "altitude_m",
         {"altitude_m", "altitude", "alt"},
         "Filtered altitude in meters.",
         false,
         true},
        {SampleValueId::VelocityMetersPerSecond,
         "velocity_mps",
         {"velocity_mps", "velocity", "vel"},
         "Filtered vertical velocity in m/s.",
         false,
         true},
        {SampleValueId::ApogeePredictionMeters,
         "apogee_prediction_m",
         {"apogee_prediction_m", "apogee", "apogee_prediction"},
         "Current apogee prediction in meters.",
         false,
         true},
        {SampleValueId::AltimeterRawMeters,
         "altimeter_raw_m",
         {"altimeter_raw_m", "altimeter_m", "altimeter"},
         "Raw altimeter measurement in meters.",
         true,
         true},
        {SampleValueId::AccelIcmX,
         "accel_icm_x",
         {"accel_icm_x", "icm_ax", "accel_x"},
         "ICM accelerometer X axis (m/s^2).",
         true,
         true},
        {SampleValueId::AccelIcmY,
         "accel_icm_y",
         {"accel_icm_y", "icm_ay", "accel_y"},
         "ICM accelerometer Y axis (m/s^2).",
         true,
         true},
        {SampleValueId::AccelIcmZ,
         "accel_icm_z",
         {"accel_icm_z", "icm_az", "accel_z"},
         "ICM accelerometer Z axis (m/s^2).",
         true,
         true},
        {SampleValueId::AccelBnoX,
         "accel_bno_x",
         {"accel_bno_x", "bno_ax"},
         "BNO accelerometer X axis (m/s^2).",
         true,
         true},
        {SampleValueId::AccelBnoY,
         "accel_bno_y",
         {"accel_bno_y", "bno_ay"},
         "BNO accelerometer Y axis (m/s^2).",
         true,
         true},
        {SampleValueId::AccelBnoZ,
         "accel_bno_z",
         {"accel_bno_z", "bno_az"},
         "BNO accelerometer Z axis (m/s^2).",
         true,
         true},
        {SampleValueId::GyroX, "gyro_x", {"gyro_x", "gx"}, "Gyroscope X axis (rad/s).", true, true},
        {SampleValueId::GyroY, "gyro_y", {"gyro_y", "gy"}, "Gyroscope Y axis (rad/s).", true, true},
        {SampleValueId::GyroZ, "gyro_z", {"gyro_z", "gz"}, "Gyroscope Z axis (rad/s).", true, true},
    };
    return kValues;
}

const SampleValueInfo *FindSampleValueById(SampleValueId id) {
    for (const auto &value : AllSampleValues()) {
        if (value.id == id) {
            return &value;
        }
    }
    return nullptr;
}

const SampleValueInfo *FindSampleValueByName(const std::string &name) {
    for (const auto &value : AllSampleValues()) {
        if (CaseInsensitiveEquals(value.cliName, name)) {
            return &value;
        }
        for (const auto &alias : value.aliases) {
            if (CaseInsensitiveEquals(alias, name)) {
                return &value;
            }
        }
    }
    return nullptr;
}

std::vector<std::string> SplitCommaSeparated(const std::string &value) {
    std::vector<std::string> tokens;
    std::string current;
    for (char ch : value) {
        if (ch == ',') {
            const std::string trimmed = Trim(current);
            if (!trimmed.empty()) {
                tokens.push_back(trimmed);
            }
            current.clear();
        } else {
            current.push_back(ch);
        }
    }
    const std::string trimmed = Trim(current);
    if (!trimmed.empty()) {
        tokens.push_back(trimmed);
    }
    return tokens;
}

std::vector<SampleValueId> ExpandSampleValueToken(const std::string &token) {
    const std::string lowered = ToLower(Trim(token));
    if (lowered.empty()) {
        return {};
    }
    if (lowered == "accel_icm") {
        return {SampleValueId::AccelIcmX, SampleValueId::AccelIcmY, SampleValueId::AccelIcmZ};
    }
    if (lowered == "accel_bno") {
        return {SampleValueId::AccelBnoX, SampleValueId::AccelBnoY, SampleValueId::AccelBnoZ};
    }
    if (lowered == "gyro" || lowered == "gyros") {
        return {SampleValueId::GyroX, SampleValueId::GyroY, SampleValueId::GyroZ};
    }
    if (const SampleValueInfo *info = FindSampleValueByName(lowered)) {
        return {info->id};
    }
    return {};
}

bool AppendSampleValueId(SampleValueId id, bool forGraph, std::vector<SampleValueId> &target) {
    const SampleValueInfo *info = FindSampleValueById(id);
    if (!info) {
        return false;
    }
    if (forGraph && !info->allowGraph) {
        std::cerr << "Field '" << info->cliName << "' cannot be graphed." << std::endl;
        return false;
    }
    if (!forGraph && !info->allowCsv) {
        std::cerr << "Field '" << info->cliName << "' cannot be appended to the CSV output." << std::endl;
        return false;
    }
    if (std::find(target.begin(), target.end(), id) == target.end()) {
        target.push_back(id);
    }
    return true;
}

bool AppendSampleValueToken(const std::string &token, bool forGraph, std::vector<SampleValueId> &target) {
    const std::vector<SampleValueId> ids = ExpandSampleValueToken(token);
    if (ids.empty()) {
        std::cerr << "Unknown field name '" << token << "'." << std::endl;
        return false;
    }
    for (SampleValueId id : ids) {
        if (!AppendSampleValueId(id, forGraph, target)) {
            return false;
        }
    }
    return true;
}

bool ParseSampleValueList(const std::string &spec, bool forGraph, std::vector<SampleValueId> &target) {
    const std::vector<std::string> tokens = SplitCommaSeparated(spec);
    if (tokens.empty()) {
        std::cerr << "No field names were provided." << std::endl;
        return false;
    }
    for (const std::string &token : tokens) {
        if (!AppendSampleValueToken(token, forGraph, target)) {
            return false;
        }
    }
    return true;
}

std::vector<std::string> ParseCsvLine(const std::string &line) {
    std::vector<std::string> result;
    std::string field;
    bool inQuotes = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char ch = line[i];
        if (ch == '"') {
            if (inQuotes && i + 1 < line.size() && line[i + 1] == '"') {
                field.push_back('"');
                ++i;
            } else {
                inQuotes = !inQuotes;
            }
        } else if (ch == ',' && !inQuotes) {
            result.push_back(Trim(field));
            field.clear();
        } else {
            field.push_back(ch);
        }
    }
    result.push_back(Trim(field));
    return result;
}

std::optional<float> ParseFloat(const std::string &value) {
    if (value.empty()) {
        return std::nullopt;
    }
    char *end = nullptr;
    const float parsed = std::strtof(value.c_str(), &end);
    if (end == value.c_str()) {
        return std::nullopt;
    }
    return parsed;
}

std::optional<bool> ParseBool(const std::string &value) {
    if (value.empty()) {
        return std::nullopt;
    }
    const std::string lowered = ToLower(Trim(value));
    if (lowered == "true" || lowered == "1" || lowered == "yes") {
        return true;
    }
    if (lowered == "false" || lowered == "0" || lowered == "no") {
        return false;
    }
    return std::nullopt;
}

std::optional<std::size_t> FindHeaderIndex(const std::vector<std::string> &headers,
                                           const std::vector<std::string> &candidates) {
    for (const std::string &candidate : candidates) {
        for (std::size_t i = 0; i < headers.size(); ++i) {
            if (CaseInsensitiveEquals(headers[i], candidate)) {
                return i;
            }
        }
    }
    return std::nullopt;
}

std::optional<std::size_t> ResolveFieldIndex(FieldId field,
                                             const std::vector<std::string> &headers,
                                             const FieldOverrideMap &overrides,
                                             bool &usedOverride) {
    usedOverride = false;
    if (const auto it = overrides.find(field); it != overrides.end()) {
        usedOverride = true;
        return FindHeaderIndex(headers, {it->second});
    }
    const FieldInfo *info = FindFieldById(field);
    if (info == nullptr) {
        return std::nullopt;
    }
    return FindHeaderIndex(headers, info->aliases);
}

FieldIndices BuildFieldIndices(const std::vector<std::string> &headers, const FieldOverrideMap &overrides) {
    FieldIndices indices;
    auto resolve = [&](FieldId field, std::optional<std::size_t> &target) {
        bool usedOverride = false;
        std::optional<std::size_t> index = ResolveFieldIndex(field, headers, overrides, usedOverride);
        if (!index && usedOverride) {
            const FieldInfo *info = FindFieldById(field);
            std::cerr << "Warning: override for field '"
                      << (info ? info->cliName : "unknown")
                      << "' did not match any column." << std::endl;
        }
        target = index;
    };

    resolve(FieldId::Timestamp, indices.timestamp);
    resolve(FieldId::AltitudeFeet, indices.altitudeFeet);
    resolve(FieldId::AltitudeMeters, indices.altitudeMeters);
    resolve(FieldId::AccelBnoX, indices.accelBno[0]);
    resolve(FieldId::AccelBnoY, indices.accelBno[1]);
    resolve(FieldId::AccelBnoZ, indices.accelBno[2]);
    resolve(FieldId::AccelIcmX, indices.accelIcm[0]);
    resolve(FieldId::AccelIcmY, indices.accelIcm[1]);
    resolve(FieldId::AccelIcmZ, indices.accelIcm[2]);
    resolve(FieldId::GyroX, indices.gyro[0]);
    resolve(FieldId::GyroY, indices.gyro[1]);
    resolve(FieldId::GyroZ, indices.gyro[2]);
    resolve(FieldId::QuatW, indices.quaternion[0]);
    resolve(FieldId::QuatX, indices.quaternion[1]);
    resolve(FieldId::QuatY, indices.quaternion[2]);
    resolve(FieldId::QuatZ, indices.quaternion[3]);
    resolve(FieldId::HasQuaternion, indices.hasQuaternionFlag);
    return indices;
}

std::optional<float> ExtractFloat(const std::vector<std::string> &row, const std::optional<std::size_t> &index) {
    if (!index || *index >= row.size()) {
        return std::nullopt;
    }
    return ParseFloat(row[*index]);
}

std::optional<bool> ExtractBool(const std::vector<std::string> &row, const std::optional<std::size_t> &index) {
    if (!index || *index >= row.size()) {
        return std::nullopt;
    }
    return ParseBool(row[*index]);
}

bool AssignFloat(const std::vector<std::string> &row,
                 const std::optional<std::size_t> &index,
                 float &target) {
    const auto value = ExtractFloat(row, index);
    if (!value.has_value()) {
        return false;
    }
    target = *value;
    return true;
}

bool PopulateSensorData(const std::vector<std::string> &row,
                        const FieldIndices &indices,
                        SensorData &out,
                        float &altimeterMeasurementMeters,
                        std::string &error) {
    auto timestamp = ExtractFloat(row, indices.timestamp);
    if (!timestamp.has_value()) {
        error = "missing timestamp";
        return false;
    }
    out.timestamp = *timestamp;

    bool hasAltitude = false;
    altimeterMeasurementMeters = 0.0f;
    if (auto altFeet = ExtractFloat(row, indices.altitudeFeet); altFeet.has_value()) {
        out.altitudeFeet = *altFeet;
        altimeterMeasurementMeters = *altFeet * constants::kFeetToMeters;
        hasAltitude = true;
    } else if (auto altMeters = ExtractFloat(row, indices.altitudeMeters); altMeters.has_value()) {
        altimeterMeasurementMeters = *altMeters;
        out.altitudeFeet = *altMeters * constants::kMetersToFeet;
        hasAltitude = true;
    }
    if (!hasAltitude) {
        error = "missing altitude";
        return false;
    }

    bool hasIcmAccel = false;
    for (int i = 0; i < 3; ++i) {
        hasIcmAccel |= AssignFloat(row, indices.accelIcm[i], out.accelICM[i]);
    }
    bool hasBnoAccel = false;
    for (int i = 0; i < 3; ++i) {
        hasBnoAccel |= AssignFloat(row, indices.accelBno[i], out.accelBNO[i]);
    }
    if (!hasIcmAccel && hasBnoAccel) {
        out.accelICM[0] = out.accelBNO[0];
        out.accelICM[1] = out.accelBNO[1];
        out.accelICM[2] = out.accelBNO[2];
        hasIcmAccel = true;
    }
    if (!hasBnoAccel && hasIcmAccel) {
        out.accelBNO[0] = out.accelICM[0];
        out.accelBNO[1] = out.accelICM[1];
        out.accelBNO[2] = out.accelICM[2];
        hasBnoAccel = true;
    }

    for (int i = 0; i < 3; ++i) {
        AssignFloat(row, indices.gyro[i], out.gyro[i]);
    }

    bool hasQuaternionValues = true;
    for (int i = 0; i < 4; ++i) {
        if (!AssignFloat(row, indices.quaternion[i], out.quaternion[i])) {
            hasQuaternionValues = false;
        }
    }

    if (auto flag = ExtractBool(row, indices.hasQuaternionFlag); flag.has_value()) {
        out.hasQuaternion = *flag;
    } else {
        out.hasQuaternion = hasQuaternionValues;
    }

    return true;
}

struct SampleValueContext {
    const FilteredState &state;
    const SensorData &sensor;
    std::optional<float> altimeterMeters;
};

struct SampleSnapshot {
    FilteredState state;
    SensorData sensor;
    std::optional<float> altimeterMeters;
};

std::optional<float> ResolveSampleValue(SampleValueId id, const SampleValueContext &ctx) {
    switch (id) {
        case SampleValueId::AltitudeMeters:
            return ctx.state.position[2];
        case SampleValueId::VelocityMetersPerSecond:
            return ctx.state.velocity[2];
        case SampleValueId::ApogeePredictionMeters:
            return ctx.state.apogeeEstimate;
        case SampleValueId::AltimeterRawMeters:
            return ctx.altimeterMeters;
        case SampleValueId::AccelIcmX:
            return ctx.sensor.accelICM[0];
        case SampleValueId::AccelIcmY:
            return ctx.sensor.accelICM[1];
        case SampleValueId::AccelIcmZ:
            return ctx.sensor.accelICM[2];
        case SampleValueId::AccelBnoX:
            return ctx.sensor.accelBNO[0];
        case SampleValueId::AccelBnoY:
            return ctx.sensor.accelBNO[1];
        case SampleValueId::AccelBnoZ:
            return ctx.sensor.accelBNO[2];
        case SampleValueId::GyroX:
            return ctx.sensor.gyro[0];
        case SampleValueId::GyroY:
            return ctx.sensor.gyro[1];
        case SampleValueId::GyroZ:
            return ctx.sensor.gyro[2];
    }
    return std::nullopt;
}

void RenderAsciiGraph(const SampleValueInfo &info, const std::vector<std::pair<float, float>> &series) {
    if (series.empty()) {
        return;
    }

    constexpr std::size_t kWidth = 80;
    constexpr std::size_t kHeight = 20;

    float minTime = series.front().first;
    float maxTime = series.front().first;
    float minValue = series.front().second;
    float maxValue = series.front().second;

    for (const auto &[time, value] : series) {
        minTime = std::min(minTime, time);
        maxTime = std::max(maxTime, time);
        minValue = std::min(minValue, value);
        maxValue = std::max(maxValue, value);
    }

    float timeRange = maxTime - minTime;
    if (timeRange <= 0.0f) {
        timeRange = 1.0f;
        maxTime = minTime + timeRange;
    }

    float valueRange = maxValue - minValue;
    if (valueRange <= 0.0f) {
        const float padding = std::max(1.0f, std::abs(maxValue) * 0.1f + 1e-3f);
        minValue -= padding;
        maxValue += padding;
        valueRange = maxValue - minValue;
    }

    std::vector<std::string> canvas(kHeight, std::string(kWidth, ' '));
    for (std::size_t x = 0; x < kWidth; ++x) {
        canvas[kHeight - 1][x] = '-';
    }
    for (std::size_t y = 0; y < kHeight; ++y) {
        canvas[y][0] = '|';
    }
    canvas[kHeight - 1][0] = '+';

    for (const auto &[time, value] : series) {
        const float normalizedX = (time - minTime) / timeRange;
        const float normalizedY = (value - minValue) / valueRange;
        const float clampedX = std::clamp(normalizedX, 0.0f, 1.0f);
        const float clampedY = std::clamp(normalizedY, 0.0f, 1.0f);
        const std::size_t x = static_cast<std::size_t>(clampedX * (kWidth - 1) + 0.5f);
        const std::size_t y = static_cast<std::size_t>(clampedY * (kHeight - 1) + 0.5f);
        const std::size_t row = (kHeight - 1) - y;
        if (row < canvas.size() && x < canvas[row].size()) {
            canvas[row][x] = '*';
        }
    }

    std::cout << std::endl;
    std::cout << "Graph: " << info.cliName << " - " << info.description << std::endl;
    std::cout << "Time range: " << minTime << "s to " << maxTime << "s" << std::endl;
    std::cout << "Value range: " << minValue << " to " << maxValue << std::endl;
    for (const auto &row : canvas) {
        std::cout << row << std::endl;
    }
}

void RenderMatplotGraph(const SampleValueInfo &info, const std::vector<std::pair<float, float>> &series) {
    if (series.empty()) {
        return;
    }

    try {
        std::vector<double> times;
        std::vector<double> values;
        times.reserve(series.size());
        values.reserve(series.size());
        for (const auto &[time, value] : series) {
            times.push_back(static_cast<double>(time));
            values.push_back(static_cast<double>(value));
        }

        auto fig = matplot::figure(true);
        fig->size(1200, 600);
        auto ax = fig->current_axes();
        ax->plot(times, values);
        ax->xlabel("Time (s)");
        ax->ylabel(info.cliName);
        ax->title(std::string(info.cliName) + " - " + info.description);
        ax->grid(true);

        const std::string filename = "graph_" + SanitizeFilename(info.cliName) + ".png";
        fig->save(filename);
        std::cout << "Saved Matplot++ graph: " << filename << std::endl;
    } catch (const std::exception &ex) {
        std::cerr << "Failed to render Matplot++ graph for '" << info.cliName << "': " << ex.what()
                  << std::endl;
    }
}

void RenderRequestedGraphs(const std::vector<SampleSnapshot> &snapshots,
                           const std::vector<SampleValueId> &fields) {
    if (fields.empty()) {
        return;
    }
    if (snapshots.empty()) {
        std::cout << std::endl
                  << "Graphing was requested but no filtered samples were generated." << std::endl;
        return;
    }

    for (SampleValueId field : fields) {
        const SampleValueInfo *info = FindSampleValueById(field);
        if (!info) {
            continue;
        }
        std::vector<std::pair<float, float>> series;
        series.reserve(snapshots.size());
        for (const auto &snapshot : snapshots) {
            SampleValueContext ctx{snapshot.state, snapshot.sensor, snapshot.altimeterMeters};
            auto value = ResolveSampleValue(field, ctx);
            if (!value.has_value()) {
                continue;
            }
            series.emplace_back(snapshot.state.time, *value);
        }
        if (series.empty()) {
            std::cout << std::endl
                      << "Graph: " << info->cliName << " - no data available." << std::endl;
            continue;
        }
        RenderAsciiGraph(*info, series);
        RenderMatplotGraph(*info, series);
    }
}

void PrintUsage(const char *program) {
    std::cout << "Usage: " << program << " <csv-file> [options]\n"
              << "Options:\n"
              << "  --field <field>=<header>   Override which CSV column maps to a sensor field.\n"
              << "  --sigma-accel-xy <value>   Accelerometer XY measurement sigma (default 0.5).\n"
              << "  --sigma-accel-z <value>    Accelerometer Z measurement sigma (default 0.5).\n"
              << "  --sigma-altimeter <value>  Altimeter measurement sigma (default 0.5).\n"
              << "  --process-xy <value>       Process noise for XY axes (default 0.5).\n"
              << "  --process-z <value>        Process noise for Z axis (default 1.0).\n"
              << "  --apogee-target <value>    Target apogee altitude in meters (default 1550).\n"
              << "  --include-raw-altimeter    Append raw altimeter measurements to output CSV.\n"
              << "  --include-raw <fields>    Append raw sensor fields (comma-separated).\n"
             << "  --graph <fields>          Render ASCII graphs and Matplot++ images for the requested fields.\n"
              << "  --quiet                    Suppress per-sample output.\n"
              << "  -h, --help                 Show this message.\n"
              << "Available field names for --field overrides:\n";
    for (const auto &field : AllFields()) {
        std::cout << "  - " << field.cliName << std::endl;
    }
    std::cout << "Raw output field names (--include-raw):\n";
    for (const auto &value : AllSampleValues()) {
        if (value.allowCsv) {
            std::cout << "  - " << value.cliName << ": " << value.description << std::endl;
        }
    }
    std::cout << "Graphable field names (--graph):\n";
    for (const auto &value : AllSampleValues()) {
        if (value.allowGraph) {
            std::cout << "  - " << value.cliName << ": " << value.description << std::endl;
        }
    }
}

bool ParseFieldOverride(const std::string &spec, FieldOverrideMap &overrides) {
    const auto pos = spec.find('=');
    if (pos == std::string::npos) {
        std::cerr << "Invalid --field argument: " << spec << std::endl;
        return false;
    }
    const std::string fieldName = Trim(spec.substr(0, pos));
    const std::string headerName = Trim(spec.substr(pos + 1));
    if (headerName.empty()) {
        std::cerr << "Invalid --field argument (missing header name): " << spec << std::endl;
        return false;
    }
    const FieldInfo *info = FindFieldByName(fieldName);
    if (!info) {
        std::cerr << "Unknown field name in --field: " << fieldName << std::endl;
        return false;
    }
    overrides[info->id] = headerName;
    return true;
}

bool ParseArgs(int argc, char **argv, ProgramOptions &options) {
    if (argc <= 1) {
        options.showHelp = true;
        return true;
    }
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            options.showHelp = true;
            return true;
        }
        if (arg == "--quiet") {
            options.quiet = true;
            continue;
        }
        if (arg == "--include-raw-altimeter") {
            if (!AppendSampleValueId(SampleValueId::AltimeterRawMeters, false, options.extraOutputFields)) {
                return false;
            }
            continue;
        }
        if (arg == "--include-raw") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --include-raw" << std::endl;
                return false;
            }
            if (!ParseSampleValueList(argv[++i], false, options.extraOutputFields)) {
                return false;
            }
            continue;
        }
        if (arg.rfind("--include-raw=", 0) == 0) {
            if (!ParseSampleValueList(arg.substr(14), false, options.extraOutputFields)) {
                return false;
            }
            continue;
        }
        if (arg == "--graph") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --graph" << std::endl;
                return false;
            }
            if (!ParseSampleValueList(argv[++i], true, options.graphFields)) {
                return false;
            }
            continue;
        }
        if (arg.rfind("--graph=", 0) == 0) {
            if (!ParseSampleValueList(arg.substr(8), true, options.graphFields)) {
                return false;
            }
            continue;
        }
        auto parseFloatArg = [&](float &target) -> bool {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for option: " << arg << std::endl;
                return false;
            }
            std::optional<float> value = ParseFloat(argv[++i]);
            if (!value.has_value()) {
                std::cerr << "Invalid numeric value for option " << arg << ": " << argv[i] << std::endl;
                return false;
            }
            target = *value;
            return true;
        };

        if (arg == "--sigma-accel-xy") {
            if (!parseFloatArg(options.sigmaAccelXY)) {
                return false;
            }
            continue;
        }
        if (arg == "--sigma-accel-z") {
            if (!parseFloatArg(options.sigmaAccelZ)) {
                return false;
            }
            continue;
        }
        if (arg == "--sigma-altimeter") {
            if (!parseFloatArg(options.sigmaAltimeter)) {
                return false;
            }
            continue;
        }
        if (arg == "--process-xy") {
            if (!parseFloatArg(options.processXY)) {
                return false;
            }
            continue;
        }
        if (arg == "--process-z") {
            if (!parseFloatArg(options.processZ)) {
                return false;
            }
            continue;
        }
        if (arg == "--apogee-target") {
            if (!parseFloatArg(options.apogeeTargetMeters)) {
                return false;
            }
            continue;
        }
        if (arg == "--field") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --field" << std::endl;
                return false;
            }
            if (!ParseFieldOverride(argv[++i], options.fieldOverrides)) {
                return false;
            }
            continue;
        }
        if (arg.rfind("--field=", 0) == 0) {
            if (!ParseFieldOverride(arg.substr(8), options.fieldOverrides)) {
                return false;
            }
            continue;
        }
        if (!arg.empty() && arg[0] == '-') {
            std::cerr << "Unknown option: " << arg << std::endl;
            return false;
        }
        if (options.csvPath.empty()) {
            options.csvPath = arg;
        } else {
            std::cerr << "Unexpected extra argument: " << arg << std::endl;
            return false;
        }
    }
    if (options.csvPath.empty()) {
        std::cerr << "Missing CSV file path." << std::endl;
        return false;
    }
    return true;
}

void PrintFieldMappingSummary(const FieldIndices &indices, const std::vector<std::string> &headers) {
    auto printEntry = [&](const char *label, const std::optional<std::size_t> &index) {
        if (index && *index < headers.size()) {
            std::cout << "  " << label << " <- '" << headers[*index] << "'" << std::endl;
        }
    };
    std::cout << "Column mapping:" << std::endl;
    printEntry("timestamp", indices.timestamp);
    printEntry("altitude_feet", indices.altitudeFeet);
    printEntry("altitude_meters", indices.altitudeMeters);
    const char *accLabels[3] = {"_x", "_y", "_z"};
    for (int i = 0; i < 3; ++i) {
        std::string label = std::string("accel_icm") + accLabels[i];
        printEntry(label.c_str(), indices.accelIcm[i]);
    }
    for (int i = 0; i < 3; ++i) {
        std::string label = std::string("accel_bno") + accLabels[i];
        printEntry(label.c_str(), indices.accelBno[i]);
    }
    for (int i = 0; i < 3; ++i) {
        std::string label = std::string("gyro") + accLabels[i];
        printEntry(label.c_str(), indices.gyro[i]);
    }
    printEntry("quat_w", indices.quaternion[0]);
    printEntry("quat_x", indices.quaternion[1]);
    printEntry("quat_y", indices.quaternion[2]);
    printEntry("quat_z", indices.quaternion[3]);
}

}  // namespace

int main(int argc, char **argv) {
    ProgramOptions options;
    if (!ParseArgs(argc, argv, options)) {
        PrintUsage(argv[0]);
        return 1;
    }
    if (options.showHelp) {
        PrintUsage(argv[0]);
        return 0;
    }

    std::ifstream input(options.csvPath);
    if (!input.is_open()) {
        std::cerr << "Failed to open CSV file: " << options.csvPath << std::endl;
        return 1;
    }

    std::string headerLine;
    if (!std::getline(input, headerLine)) {
        std::cerr << "CSV file is empty: " << options.csvPath << std::endl;
        return 1;
    }
    if (!headerLine.empty() && static_cast<unsigned char>(headerLine[0]) == 0xEF) {
        headerLine = headerLine.substr(3);
    }
    std::vector<std::string> headers = ParseCsvLine(headerLine);
    FieldIndices indices = BuildFieldIndices(headers, options.fieldOverrides);

    if (!indices.timestamp.has_value()) {
        std::cerr << "Unable to locate a timestamp column. Use --field to specify one." << std::endl;
        return 1;
    }
    if (!indices.altitudeFeet.has_value() && !indices.altitudeMeters.has_value()) {
        std::cerr << "Unable to locate an altitude column (feet or meters). Use --field to specify one." << std::endl;
        return 1;
    }

    if (!options.quiet) {
        PrintFieldMappingSummary(indices, headers);
    }

    EnvironmentModel::Config environmentConfig;
    ApogeeVehicleParameters vehicleParameters;
    FlightComputer flightComputer;
    flightComputer.Begin(options.sigmaAccelXY,
                         options.sigmaAccelZ,
                         options.sigmaAltimeter,
                         options.processXY,
                         options.processZ,
                         options.apogeeTargetMeters,
                         environmentConfig,
                         vehicleParameters);
    flightComputer.SetSerialReportingEnabled(false);

    if (!options.quiet) {
        std::cout << "time_s,altitude_m,velocity_mps,apogee_prediction_m,status";
        for (SampleValueId field : options.extraOutputFields) {
            if (const SampleValueInfo *info = FindSampleValueById(field)) {
                std::cout << ',' << info->cliName;
            }
        }
        std::cout << std::endl;
    }

    const bool graphingEnabled = !options.graphFields.empty();
    std::vector<SampleSnapshot> snapshots;
    if (graphingEnabled) {
        snapshots.reserve(2048);
    }

    std::string line;
    std::size_t lineNumber = 1;
    std::size_t processedRows = 0;
    std::size_t skippedRows = 0;
    std::size_t emittedStates = 0;
    while (std::getline(input, line)) {
        ++lineNumber;
        if (Trim(line).empty()) {
            continue;
        }
        std::vector<std::string> row = ParseCsvLine(line);
        SensorData sample;
        float altimeterMeasurementMeters = 0.0f;
        std::string error;
        if (!PopulateSensorData(row, indices, sample, altimeterMeasurementMeters, error)) {
            ++skippedRows;
            std::cerr << "Skipping line " << lineNumber << ": " << error << std::endl;
            continue;
        }
        ++processedRows;
        FilteredState state;
        const bool hasState = flightComputer.Update(sample, state);
        if (hasState) {
            ++emittedStates;
            if (!options.quiet) {
                std::cout << state.time << ',' << state.position[2] << ',' << state.velocity[2] << ','
                          << state.apogeeEstimate << ',' << FlightStatusToString(flightComputer.Status());
                if (!options.extraOutputFields.empty()) {
                    SampleValueContext context{state, sample, altimeterMeasurementMeters};
                    for (SampleValueId field : options.extraOutputFields) {
                        std::cout << ',';
                        const auto value = ResolveSampleValue(field, context);
                        if (value.has_value()) {
                            std::cout << *value;
                        }
                    }
                }
                std::cout << std::endl;
            }
            if (graphingEnabled) {
                snapshots.push_back(SampleSnapshot{state, sample, altimeterMeasurementMeters});
            }
        }
    }

    if (graphingEnabled) {
        RenderRequestedGraphs(snapshots, options.graphFields);
    }

    std::cout << std::endl;
    std::cout << "Samples processed: " << processedRows << std::endl;
    std::cout << "Rows skipped: " << skippedRows << std::endl;
    std::cout << "States generated: " << emittedStates << std::endl;
    if (flightComputer.ApogeeReached()) {
        std::cout << "Apogee recorded at " << flightComputer.ApogeeAltitude() << " m" << std::endl;
    } else {
        std::cout << "Latest apogee prediction: " << flightComputer.ApogeePrediction() << " m" << std::endl;
    }
    if (flightComputer.BurnTime() > 0.0f) {
        std::cout << "Burn detected at t=" << flightComputer.BurnTime() << " s" << std::endl;
    }
    if (flightComputer.BurnoutTime() > 0.0f) {
        std::cout << "Burnout detected at t=" << flightComputer.BurnoutTime() << " s" << std::endl;
    }
    if (flightComputer.ApogeeTime() > 0.0f) {
        std::cout << "Apogee detected at t=" << flightComputer.ApogeeTime() << " s" << std::endl;
    }

    return 0;
}
