

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

#include "../../src/constants.h"
#include "../../src/flight_computer.h"
#include "../../src/apogee_model.h"
#include <matplot/matplot.h>

namespace {

bool CaseInsensitiveEquals(const std::string &a, const std::string &b);
enum class SampleValueId;

constexpr float kAltimeterMinFeet = 0.0f;
constexpr float kAltimeterMaxFeet = 6000.0f;
constexpr float kDegreesToRadians = 0.01745329251994329577f;

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
    AccelLsmX,
    AccelLsmY,
    AccelLsmZ,
    GyroLsmX,
    GyroLsmY,
    GyroLsmZ,
    QuatW,
    QuatX,
    QuatY,
    QuatZ,
    IcmQuatW,
    IcmQuatX,
    IcmQuatY,
    IcmQuatZ,
    LsmQuatW,
    LsmQuatX,
    LsmQuatY,
    LsmQuatZ,
    HasQuaternion,
    HasIcmQuaternion,
    HasLsmQuaternion,
    MainQuaternionSource,
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
        {FieldId::AccelLsmX,
         "accel_lsm_x",
         {"accel_lsm_x", "lsm_ax", "sensor_accel_lsm_x"}},
        {FieldId::AccelLsmY,
         "accel_lsm_y",
         {"accel_lsm_y", "lsm_ay", "sensor_accel_lsm_y"}},
        {FieldId::AccelLsmZ,
         "accel_lsm_z",
         {"accel_lsm_z", "lsm_az", "sensor_accel_lsm_z"}},
        {FieldId::GyroLsmX, "gyro_lsm_x", {"gyro_lsm_x", "lsm_gx", "sensor_gyro_lsm_x"}},
        {FieldId::GyroLsmY, "gyro_lsm_y", {"gyro_lsm_y", "lsm_gy", "sensor_gyro_lsm_y"}},
        {FieldId::GyroLsmZ, "gyro_lsm_z", {"gyro_lsm_z", "lsm_gz", "sensor_gyro_lsm_z"}},
        {FieldId::QuatW, "quat_w", {"quat_w", "qw", "sensor_quat_w"}},
        {FieldId::QuatX, "quat_x", {"quat_x", "qx", "sensor_quat_x"}},
        {FieldId::QuatY, "quat_y", {"quat_y", "qy", "sensor_quat_y"}},
        {FieldId::QuatZ, "quat_z", {"quat_z", "qz", "sensor_quat_z"}},
        {FieldId::IcmQuatW, "icm_quat_w", {"icm_quat_w", "sensor_icm_quat_w"}},
        {FieldId::IcmQuatX, "icm_quat_x", {"icm_quat_x", "sensor_icm_quat_x"}},
        {FieldId::IcmQuatY, "icm_quat_y", {"icm_quat_y", "sensor_icm_quat_y"}},
        {FieldId::IcmQuatZ, "icm_quat_z", {"icm_quat_z", "sensor_icm_quat_z"}},
        {FieldId::LsmQuatW, "lsm_quat_w", {"lsm_quat_w", "sensor_lsm_quat_w"}},
        {FieldId::LsmQuatX, "lsm_quat_x", {"lsm_quat_x", "sensor_lsm_quat_x"}},
        {FieldId::LsmQuatY, "lsm_quat_y", {"lsm_quat_y", "sensor_lsm_quat_y"}},
        {FieldId::LsmQuatZ, "lsm_quat_z", {"lsm_quat_z", "sensor_lsm_quat_z"}},
        {FieldId::HasQuaternion,
         "has_quaternion",
         {"has_quaternion", "quat_valid", "sensor_has_quaternion"}},
        {FieldId::HasIcmQuaternion,
         "has_icm_quaternion",
         {"has_icm_quaternion", "sensor_has_icm_quaternion"}},
        {FieldId::HasLsmQuaternion,
         "has_lsm_quaternion",
         {"has_lsm_quaternion", "sensor_has_lsm_quaternion"}},
        {FieldId::MainQuaternionSource,
         "main_quaternion_source",
         {"main_quaternion_source", "sensor_main_quaternion_source"}},
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
    std::string cfdPath = "lib/cfd.csv";
    bool showHelp = false;
    bool quiet = false;
    bool ignoreLoggedState = false;
    float sigmaAccelXY = 0.5f;
    float sigmaAccelZ = 0.5f;
    float sigmaAltimeter = 0.5f;
    float processXY = 0.5f;
    float processZ = 1.0f;
    float apogeeTargetMeters = 1550.0f;
    std::optional<float> signCheckTimeSeconds;
    float signCheckWindowSeconds = 0.05f;
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
    std::array<std::optional<std::size_t>, 3> accelLsm{};
    std::array<std::optional<std::size_t>, 3> gyro{};
    std::array<std::optional<std::size_t>, 3> gyroLsm{};
    std::array<std::optional<std::size_t>, 4> quaternion{};
    std::array<std::optional<std::size_t>, 4> icmQuaternion{};
    std::array<std::optional<std::size_t>, 4> lsmQuaternion{};
    std::optional<std::size_t> hasQuaternionFlag;
    std::optional<std::size_t> hasIcmQuaternionFlag;
    std::optional<std::size_t> hasLsmQuaternionFlag;
    std::optional<std::size_t> mainQuaternionSource;
};

struct ReplaySeedIndices {
    std::optional<std::size_t> hasFilteredState;
    std::optional<std::size_t> flightStatus;
    std::optional<std::size_t> stateTime;
    std::array<std::optional<std::size_t>, 3> position{};
    std::array<std::optional<std::size_t>, 3> velocity{};
    std::array<std::optional<std::size_t>, 3> acceleration{};
    std::array<std::optional<std::size_t>, 3> inertialAcceleration{};
    std::optional<std::size_t> zenith;
    std::optional<std::size_t> apogeeEstimate;
    bool positionZIsFeet = false;
    bool velocityZIsFeetPerSecond = false;
    bool zenithIsDegrees = false;
    bool apogeeEstimateIsFeet = false;

    bool HasAnySeedColumns() const {
        return stateTime.has_value() || position[2].has_value() || velocity[2].has_value() ||
               zenith.has_value() || apogeeEstimate.has_value();
    }

    bool HasRequiredStateColumns() const {
        return stateTime.has_value() && position[2].has_value() && velocity[2].has_value() &&
               zenith.has_value();
    }
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
    AltitudeAglMeters,
    VelocityMetersPerSecond,
    HorizontalVelocityMetersPerSecond,
    SpeedTotalMetersPerSecond,
    RelativeAirspeedMetersPerSecond,
    MachNumber,
    ZenithDegrees,
    ZenithRateDegreesPerSecond,
    AngleOfAttackDegrees,
    AngleOfAttackAbsDegrees,
    ApogeeErrorMeters,
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
        {SampleValueId::AltitudeAglMeters,
         "altitude_agl_m",
         {"altitude_agl_m", "agl_m", "alt_agl"},
         "Filtered altitude above launch reference in meters.",
         true,
         true},
        {SampleValueId::VelocityMetersPerSecond,
         "velocity_mps",
         {"velocity_mps", "velocity", "vel"},
         "Filtered vertical velocity in m/s.",
         false,
         true},
        {SampleValueId::HorizontalVelocityMetersPerSecond,
         "horizontal_velocity_mps",
         {"horizontal_velocity_mps", "hvel", "velocity_xy_mps"},
         "Horizontal speed magnitude in m/s.",
         true,
         true},
        {SampleValueId::SpeedTotalMetersPerSecond,
         "speed_total_mps",
         {"speed_total_mps", "speed_mps", "speed"},
         "Total speed magnitude in m/s.",
         true,
         true},
        {SampleValueId::RelativeAirspeedMetersPerSecond,
         "airspeed_rel_mps",
         {"airspeed_rel_mps", "airspeed", "relative_airspeed_mps"},
         "Relative airspeed magnitude after wind subtraction in m/s.",
         true,
         true},
        {SampleValueId::MachNumber,
         "mach",
         {"mach", "mach_number"},
         "Estimated Mach number from relative airspeed.",
         true,
         true},
        {SampleValueId::ZenithDegrees,
         "zenith_deg",
         {"zenith_deg", "zenith", "attitude_zenith_deg"},
         "Zenith angle in degrees.",
         true,
         true},
        {SampleValueId::ZenithRateDegreesPerSecond,
         "zenith_rate_dps",
         {"zenith_rate_dps", "zenith_rate", "attitude_rate_dps"},
         "Zenith angular rate in degrees/s.",
         true,
         true},
        {SampleValueId::AngleOfAttackDegrees,
         "aoa_deg",
         {"aoa_deg", "aoa", "angle_of_attack_deg"},
         "Signed angle of attack estimate in degrees.",
         true,
         true},
        {SampleValueId::AngleOfAttackAbsDegrees,
         "aoa_abs_deg",
         {"aoa_abs_deg", "aoa_abs", "angle_of_attack_abs_deg"},
         "Absolute angle of attack estimate in degrees.",
         true,
         true},
        {SampleValueId::ApogeeErrorMeters,
         "apogee_error_m",
         {"apogee_error_m", "apogee_error", "target_error_m"},
         "Predicted apogee minus configured target (meters).",
         true,
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
    if (lowered == "derived") {
        return {SampleValueId::AltitudeAglMeters,
                SampleValueId::HorizontalVelocityMetersPerSecond,
                SampleValueId::SpeedTotalMetersPerSecond,
                SampleValueId::RelativeAirspeedMetersPerSecond,
                SampleValueId::MachNumber,
                SampleValueId::ZenithDegrees,
                SampleValueId::ZenithRateDegreesPerSecond,
                SampleValueId::AngleOfAttackDegrees,
                SampleValueId::AngleOfAttackAbsDegrees,
                SampleValueId::ApogeeErrorMeters};
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

struct StandaloneCfdTableStorage {
    std::vector<double> acs;
    std::vector<double> atk;
    std::vector<double> mach;
    std::vector<double> axial;
    std::vector<double> normal;
    ApogeeForceTable table;
    bool loaded = false;
};

bool ParseCfdNumericRow(const std::string &line, std::array<double, 5> &out) {
    const std::vector<std::string> cols = ParseCsvLine(line);
    if (cols.size() < 5) {
        return false;
    }
    for (int i = 0; i < 5; ++i) {
        char *end = nullptr;
        const double value = std::strtod(cols[i].c_str(), &end);
        if (end == cols[i].c_str()) {
            return false;
        }
        out[static_cast<std::size_t>(i)] = value;
    }
    return true;
}

bool LoadStandaloneCfdTable(const std::string &path, StandaloneCfdTableStorage &storage) {
    std::ifstream input(path);
    if (!input.is_open()) {
        return false;
    }

    std::string header;
    if (!std::getline(input, header)) {
        return false;
    }

    struct RawRow {
        double acs;
        double atk;
        double mach;
        double axial;
        double normal;
    };

    std::vector<RawRow> rows;
    rows.reserve(6000);
    std::array<double, 5> parsed{};
    std::string line;
    while (std::getline(input, line)) {
        if (Trim(line).empty()) {
            continue;
        }
        if (!ParseCfdNumericRow(line, parsed)) {
            continue;
        }
        rows.push_back(RawRow{parsed[0], parsed[1], parsed[2], parsed[3], parsed[4]});
    }
    if (rows.empty()) {
        return false;
    }

    storage.acs.clear();
    storage.atk.clear();
    storage.mach.clear();
    storage.acs.reserve(rows.size());
    storage.atk.reserve(rows.size());
    storage.mach.reserve(rows.size());
    for (const RawRow &row : rows) {
        storage.acs.push_back(row.acs);
        storage.atk.push_back(row.atk);
        storage.mach.push_back(row.mach);
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
        return false;
    }

    const int total = acsCount * atkCount * machCount;
    storage.axial.assign(total, std::numeric_limits<double>::quiet_NaN());
    storage.normal.assign(total, std::numeric_limits<double>::quiet_NaN());

    auto findIndex = [](const std::vector<double> &values, double value) -> int {
        auto it = std::lower_bound(values.begin(), values.end(), value);
        if (it == values.end() || *it != value) {
            return -1;
        }
        return static_cast<int>(it - values.begin());
    };

    for (const RawRow &row : rows) {
        const int i = findIndex(storage.acs, row.acs);
        const int j = findIndex(storage.atk, row.atk);
        const int k = findIndex(storage.mach, row.mach);
        if (i < 0 || j < 0 || k < 0) {
            continue;
        }
        const int idx = (i * atkCount + j) * machCount + k;
        storage.axial[static_cast<std::size_t>(idx)] = row.axial;
        storage.normal[static_cast<std::size_t>(idx)] = row.normal;
    }

    storage.table.acsAnglesDeg = storage.acs.data();
    storage.table.atkAnglesDeg = storage.atk.data();
    storage.table.machNumbers = storage.mach.data();
    storage.table.axialForces = storage.axial.data();
    storage.table.normalForces = storage.normal.data();
    storage.table.acsCount = acsCount;
    storage.table.atkCount = atkCount;
    storage.table.machCount = machCount;
    storage.loaded = storage.table.IsValid();
    return storage.loaded;
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

std::optional<std::size_t> FindHeaderIndexSingle(const std::vector<std::string> &headers, const std::string &name) {
    return FindHeaderIndex(headers, {name});
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
    resolve(FieldId::AccelLsmX, indices.accelLsm[0]);
    resolve(FieldId::AccelLsmY, indices.accelLsm[1]);
    resolve(FieldId::AccelLsmZ, indices.accelLsm[2]);
    resolve(FieldId::GyroX, indices.gyro[0]);
    resolve(FieldId::GyroY, indices.gyro[1]);
    resolve(FieldId::GyroZ, indices.gyro[2]);
    resolve(FieldId::GyroLsmX, indices.gyroLsm[0]);
    resolve(FieldId::GyroLsmY, indices.gyroLsm[1]);
    resolve(FieldId::GyroLsmZ, indices.gyroLsm[2]);
    resolve(FieldId::QuatW, indices.quaternion[0]);
    resolve(FieldId::QuatX, indices.quaternion[1]);
    resolve(FieldId::QuatY, indices.quaternion[2]);
    resolve(FieldId::QuatZ, indices.quaternion[3]);
    resolve(FieldId::IcmQuatW, indices.icmQuaternion[0]);
    resolve(FieldId::IcmQuatX, indices.icmQuaternion[1]);
    resolve(FieldId::IcmQuatY, indices.icmQuaternion[2]);
    resolve(FieldId::IcmQuatZ, indices.icmQuaternion[3]);
    resolve(FieldId::LsmQuatW, indices.lsmQuaternion[0]);
    resolve(FieldId::LsmQuatX, indices.lsmQuaternion[1]);
    resolve(FieldId::LsmQuatY, indices.lsmQuaternion[2]);
    resolve(FieldId::LsmQuatZ, indices.lsmQuaternion[3]);
    resolve(FieldId::HasQuaternion, indices.hasQuaternionFlag);
    resolve(FieldId::HasIcmQuaternion, indices.hasIcmQuaternionFlag);
    resolve(FieldId::HasLsmQuaternion, indices.hasLsmQuaternionFlag);
    resolve(FieldId::MainQuaternionSource, indices.mainQuaternionSource);
    return indices;
}

ReplaySeedIndices BuildReplaySeedIndices(const std::vector<std::string> &headers) {
    ReplaySeedIndices indices;
    indices.hasFilteredState = FindHeaderIndexSingle(headers, "has_filtered_state");
    indices.flightStatus = FindHeaderIndexSingle(headers, "flight_status");
    indices.stateTime = FindHeaderIndex(headers, {"state_time", "sensor_timestamp"});
    indices.position[0] = FindHeaderIndexSingle(headers, "state_position_x");
    indices.position[1] = FindHeaderIndexSingle(headers, "state_position_y");
    indices.position[2] = FindHeaderIndexSingle(headers, "state_position_z");
    if (!indices.position[2].has_value()) {
        indices.position[2] = FindHeaderIndexSingle(headers, "state_altitude_agl_feet");
        indices.positionZIsFeet = indices.position[2].has_value();
    }
    indices.velocity[0] = FindHeaderIndexSingle(headers, "state_velocity_x");
    indices.velocity[1] = FindHeaderIndexSingle(headers, "state_velocity_y");
    indices.velocity[2] = FindHeaderIndexSingle(headers, "state_velocity_z");
    if (!indices.velocity[2].has_value()) {
        indices.velocity[2] = FindHeaderIndexSingle(headers, "state_vertical_velocity_fps");
        indices.velocityZIsFeetPerSecond = indices.velocity[2].has_value();
    }
    indices.acceleration[0] = FindHeaderIndexSingle(headers, "state_acceleration_x");
    indices.acceleration[1] = FindHeaderIndexSingle(headers, "state_acceleration_y");
    indices.acceleration[2] = FindHeaderIndexSingle(headers, "state_acceleration_z");
    indices.inertialAcceleration[0] = FindHeaderIndexSingle(headers, "state_inertial_acceleration_x");
    indices.inertialAcceleration[1] = FindHeaderIndexSingle(headers, "state_inertial_acceleration_y");
    indices.inertialAcceleration[2] = FindHeaderIndexSingle(headers, "state_inertial_acceleration_z");
    indices.zenith = FindHeaderIndexSingle(headers, "state_zenith");
    if (!indices.zenith.has_value()) {
        indices.zenith = FindHeaderIndexSingle(headers, "state_zenith_deg");
        indices.zenithIsDegrees = indices.zenith.has_value();
    }
    indices.apogeeEstimate = FindHeaderIndexSingle(headers, "state_apogee_estimate");
    if (!indices.apogeeEstimate.has_value()) {
        indices.apogeeEstimate = FindHeaderIndexSingle(headers, "state_apogee_estimate_feet");
        indices.apogeeEstimateIsFeet = indices.apogeeEstimate.has_value();
    }
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
    out = SensorData{};
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
    bool hasLsmAccel = false;
    for (int i = 0; i < 3; ++i) {
        hasLsmAccel |= AssignFloat(row, indices.accelLsm[i], out.accelLSM[i]);
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
        AssignFloat(row, indices.gyroLsm[i], out.gyroLSM[i]);
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

    bool hasIcmQuaternionValues = true;
    for (int i = 0; i < 4; ++i) {
        if (!AssignFloat(row, indices.icmQuaternion[i], out.icmQuaternion[i])) {
            hasIcmQuaternionValues = false;
        }
    }
    if (auto flag = ExtractBool(row, indices.hasIcmQuaternionFlag); flag.has_value()) {
        out.hasIcmQuaternion = *flag;
    } else {
        out.hasIcmQuaternion = hasIcmQuaternionValues;
    }

    bool hasLsmQuaternionValues = true;
    for (int i = 0; i < 4; ++i) {
        if (!AssignFloat(row, indices.lsmQuaternion[i], out.quaternionLSM[i])) {
            hasLsmQuaternionValues = false;
        }
    }
    if (auto flag = ExtractBool(row, indices.hasLsmQuaternionFlag); flag.has_value()) {
        out.hasLsmQuaternion = *flag;
    } else {
        out.hasLsmQuaternion = hasLsmQuaternionValues;
    }

    if (const auto source = ExtractFloat(row, indices.mainQuaternionSource); source.has_value()) {
        const int sourceCode = static_cast<int>(std::lround(*source));
        if (sourceCode >= 0 && sourceCode <= 255) {
            out.mainQuaternionSource = static_cast<uint8_t>(sourceCode);
        }
    }

    if (!out.hasQuaternion) {
        const MainQuaternionSource source =
            static_cast<MainQuaternionSource>(out.mainQuaternionSource);
        if (source == MainQuaternionSource::Icm && out.hasIcmQuaternion) {
            std::copy(std::begin(out.icmQuaternion), std::end(out.icmQuaternion), std::begin(out.quaternion));
            out.hasQuaternion = true;
        } else if (source == MainQuaternionSource::Lsm && out.hasLsmQuaternion) {
            std::copy(std::begin(out.quaternionLSM), std::end(out.quaternionLSM), std::begin(out.quaternion));
            out.hasQuaternion = true;
        }
    }

    out.icmSampleFresh = hasIcmAccel || out.hasIcmQuaternion;
    out.lsmSampleFresh = hasLsmAccel || out.hasLsmQuaternion;
    out.baroSampleFresh = hasAltitude;

    return true;
}

std::optional<FlightStatus> ParseFlightStatusValue(const std::string &value) {
    const std::string lowered = ToLower(Trim(value));
    if (lowered == "ground") {
        return FlightStatus::Ground;
    }
    if (lowered == "burn") {
        return FlightStatus::Burn;
    }
    if (lowered == "coast") {
        return FlightStatus::Coast;
    }
    if (lowered == "overshoot") {
        return FlightStatus::Overshoot;
    }
    if (lowered == "descent") {
        return FlightStatus::Descent;
    }
    return std::nullopt;
}

bool TryPopulateSeededState(const std::vector<std::string> &row,
                            const ReplaySeedIndices &indices,
                            FilteredState &state,
                            FlightStatus &status) {
    if (!indices.HasRequiredStateColumns()) {
        return false;
    }
    if (indices.hasFilteredState.has_value()) {
        const auto hasFilteredState = ExtractBool(row, indices.hasFilteredState);
        if (!hasFilteredState.has_value() || !*hasFilteredState) {
            return false;
        }
    }

    const auto stateTime = ExtractFloat(row, indices.stateTime);
    const auto posZ = ExtractFloat(row, indices.position[2]);
    const auto velZ = ExtractFloat(row, indices.velocity[2]);
    const auto zenith = ExtractFloat(row, indices.zenith);
    const auto apogeeEstimate = ExtractFloat(row, indices.apogeeEstimate);
    if (!stateTime.has_value() || !posZ.has_value() || !velZ.has_value() || !zenith.has_value()) {
        return false;
    }

    FlightStatus parsedStatus = status;
    if (indices.flightStatus.has_value()) {
        const std::size_t index = *indices.flightStatus;
        if (index < row.size()) {
            const auto maybeStatus = ParseFlightStatusValue(row[index]);
            if (maybeStatus.has_value()) {
                parsedStatus = *maybeStatus;
            }
        }
    }

    state = FilteredState{};
    state.time = *stateTime;
    state.position[2] = indices.positionZIsFeet ? (*posZ * constants::kFeetToMeters) : *posZ;
    state.velocity[2] = indices.velocityZIsFeetPerSecond ? (*velZ * constants::kFeetToMeters) : *velZ;
    state.zenith = indices.zenithIsDegrees ? (*zenith * kDegreesToRadians) : *zenith;
    state.apogeeEstimate = apogeeEstimate.has_value()
                               ? (indices.apogeeEstimateIsFeet
                                      ? (*apogeeEstimate * constants::kFeetToMeters)
                                      : *apogeeEstimate)
                               : state.position[2];
    for (int i = 0; i < 3; ++i) {
        if (const auto value = ExtractFloat(row, indices.position[i]); value.has_value()) {
            state.position[i] = (i == 2 && indices.positionZIsFeet)
                                    ? (*value * constants::kFeetToMeters)
                                    : *value;
        }
        if (const auto value = ExtractFloat(row, indices.velocity[i]); value.has_value()) {
            state.velocity[i] = (i == 2 && indices.velocityZIsFeetPerSecond)
                                    ? (*value * constants::kFeetToMeters)
                                    : *value;
        }
        if (const auto value = ExtractFloat(row, indices.acceleration[i]); value.has_value()) {
            state.acceleration[i] = *value;
        }
        if (const auto value = ExtractFloat(row, indices.inertialAcceleration[i]); value.has_value()) {
            state.inertialAcceleration[i] = *value;
        }
    }
    status = parsedStatus;
    return true;
}

double ComputeSeededHorizontalVelocityOption1(const FilteredState &state) {
    const double cosZenith = std::cos(static_cast<double>(state.zenith));
    const double clampedCos = std::clamp(cosZenith, 0.1, 1.0);
    const double speedAlongAxis = static_cast<double>(state.velocity[2]) / clampedCos;
    const double verticalSquared =
        static_cast<double>(state.velocity[2]) * static_cast<double>(state.velocity[2]);
    const double speedSquared = speedAlongAxis * speedAlongAxis;
    const double horizontalSquared = speedSquared - verticalSquared;
    return (horizontalSquared > 0.0) ? std::sqrt(horizontalSquared) : 0.0;
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

double RecomputeSeededApogeeEstimate(const FilteredState &state,
                                     FlightStatus status,
                                     double angularRate,
                                     ApogeePredictor &predictor,
                                     double &lastPredictionMeters,
                                     float &lastPredictionTimeSeconds,
                                     bool &hasLastPrediction) {
    const bool ascending = state.velocity[2] > 0.0f;
    const bool shouldPredict =
        ascending && (status == FlightStatus::Burn || status == FlightStatus::Coast);
    if (shouldPredict) {
        const bool shouldRefresh = !hasLastPrediction ||
                                   (state.time - lastPredictionTimeSeconds) >= 0.1f;
        if (shouldRefresh) {
            ApogeeState predictorState;
            predictorState.altitudeMeters = static_cast<double>(state.position[2]);
            predictorState.horizontalDistanceMeters = 0.0;
            predictorState.verticalVelocity = static_cast<double>(state.velocity[2]);
            predictorState.horizontalVelocity = ComputeSeededHorizontalVelocityOption1(state);
            predictorState.zenith = static_cast<double>(state.zenith);
            predictorState.angularVelocity = angularRate;
            predictorState.acsAngleDeg = 0.0;
            lastPredictionMeters = predictor.PredictApogee(predictorState);
            lastPredictionTimeSeconds = state.time;
            hasLastPrediction = true;
        }
        return lastPredictionMeters;
    }

    if (hasLastPrediction) {
        return lastPredictionMeters;
    }
    return state.apogeeEstimate;
}

struct SampleValueContext {
    const FilteredState &state;
    const SensorData &sensor;
    std::optional<float> altimeterMeters;
    struct DerivedMetrics {
        float altitudeAglMeters = 0.0f;
        float horizontalVelocityMps = 0.0f;
        float speedTotalMps = 0.0f;
        float relativeAirspeedMps = 0.0f;
        float mach = 0.0f;
        float zenithDeg = 0.0f;
        float zenithRateDps = 0.0f;
        float aoaDeg = 0.0f;
        float aoaAbsDeg = 0.0f;
        float apogeeErrorMeters = 0.0f;
    } derived;
};

struct SampleSnapshot {
    FilteredState state;
    SensorData sensor;
    std::optional<float> altimeterMeters;
    SampleValueContext::DerivedMetrics derived;
};

SampleValueContext::DerivedMetrics ComputeDerivedMetrics(const FilteredState &state,
                                                         const EnvironmentModel &environment,
                                                         float apogeeTargetMeters,
                                                         float altitudeReferenceMeters,
                                                         bool hasAltitudeReference,
                                                         float previousTimeSeconds,
                                                         float previousZenithRadians,
                                                         bool hasPreviousZenith) {
    SampleValueContext::DerivedMetrics metrics;

    const float velZ = state.velocity[2];
    const float velHorizontal = math_utils::Magnitude2(state.velocity[0], state.velocity[1]);
    metrics.horizontalVelocityMps = velHorizontal;
    metrics.speedTotalMps = math_utils::Magnitude2(velHorizontal, velZ);

    const math_utils::Vec3 wind = environment.GradientWind();
    const float relX = velZ - wind.x;
    const float relY = velHorizontal - wind.y;
    metrics.relativeAirspeedMps = math_utils::Magnitude2(relX, relY);

    const float temperatureK = static_cast<float>(environment.TemperatureKelvin(state.position[2]));
    if (temperatureK > 0.0f) {
        const float speedOfSound =
            math_utils::FastSqrt(static_cast<float>(constants::kGamma * constants::kGasConstant) * temperatureK);
        if (speedOfSound > 0.0f) {
            metrics.mach = metrics.relativeAirspeedMps / speedOfSound;
        }
    }

    constexpr float kRadToDeg = 57.29577951308232f;
    metrics.zenithDeg = state.zenith * kRadToDeg;
    const float flowAngle = std::fabs(math_utils::FastAtan2(relY, relX));
    const float aoaRad = state.zenith - flowAngle;
    metrics.aoaDeg = aoaRad * kRadToDeg;
    metrics.aoaAbsDeg = std::fabs(metrics.aoaDeg);

    if (hasPreviousZenith) {
        const float dt = state.time - previousTimeSeconds;
        if (dt > 1.0e-5f) {
            metrics.zenithRateDps = (state.zenith - previousZenithRadians) * kRadToDeg / dt;
        }
    }

    metrics.apogeeErrorMeters = state.apogeeEstimate - apogeeTargetMeters;

    if (hasAltitudeReference) {
        metrics.altitudeAglMeters = state.position[2] - altitudeReferenceMeters;
        if (metrics.altitudeAglMeters < 0.0f) {
            metrics.altitudeAglMeters = 0.0f;
        }
    }

    return metrics;
}

std::optional<float> ResolveSampleValue(SampleValueId id, const SampleValueContext &ctx) {
    switch (id) {
        case SampleValueId::AltitudeMeters:
            return ctx.state.position[2];
        case SampleValueId::AltitudeAglMeters:
            return ctx.derived.altitudeAglMeters;
        case SampleValueId::VelocityMetersPerSecond:
            return ctx.state.velocity[2];
        case SampleValueId::HorizontalVelocityMetersPerSecond:
            return ctx.derived.horizontalVelocityMps;
        case SampleValueId::SpeedTotalMetersPerSecond:
            return ctx.derived.speedTotalMps;
        case SampleValueId::RelativeAirspeedMetersPerSecond:
            return ctx.derived.relativeAirspeedMps;
        case SampleValueId::MachNumber:
            return ctx.derived.mach;
        case SampleValueId::ZenithDegrees:
            return ctx.derived.zenithDeg;
        case SampleValueId::ZenithRateDegreesPerSecond:
            return ctx.derived.zenithRateDps;
        case SampleValueId::AngleOfAttackDegrees:
            return ctx.derived.aoaDeg;
        case SampleValueId::AngleOfAttackAbsDegrees:
            return ctx.derived.aoaAbsDeg;
        case SampleValueId::ApogeeErrorMeters:
            return ctx.derived.apogeeErrorMeters;
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
        // Keep metric names literal (e.g. accel_icm_z) instead of gnuplot enhanced-text parsing.
        ax->title_enhanced(false);
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
            SampleValueContext ctx{
                snapshot.state,
                snapshot.sensor,
                snapshot.altimeterMeters,
                snapshot.derived,
            };
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
              << "  --cfd-path <path>          CFD CSV path for sign check (default lib/cfd.csv).\n"
              << "  --sign-check-time <sec>    Evaluate apogee at ACS 0/10/20 deg near this time.\n"
              << "  --sign-check-window <sec>  Match window for sign-check sample (default 0.05).\n"
              << "  --ignore-logged-state      Recompute filtered state instead of smart-seeding from logged state columns.\n"
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
        if (arg == "--ignore-logged-state") {
            options.ignoreLoggedState = true;
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
        if (arg == "--sign-check-time") {
            float value = 0.0f;
            if (!parseFloatArg(value)) {
                return false;
            }
            options.signCheckTimeSeconds = value;
            continue;
        }
        if (arg == "--sign-check-window") {
            if (!parseFloatArg(options.signCheckWindowSeconds)) {
                return false;
            }
            continue;
        }
        if (arg == "--cfd-path") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --cfd-path" << std::endl;
                return false;
            }
            options.cfdPath = argv[++i];
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
        if (arg.rfind("--sign-check-time=", 0) == 0) {
            std::optional<float> value = ParseFloat(arg.substr(18));
            if (!value.has_value()) {
                std::cerr << "Invalid numeric value for --sign-check-time: " << arg.substr(18)
                          << std::endl;
                return false;
            }
            options.signCheckTimeSeconds = *value;
            continue;
        }
        if (arg.rfind("--sign-check-window=", 0) == 0) {
            std::optional<float> value = ParseFloat(arg.substr(20));
            if (!value.has_value()) {
                std::cerr << "Invalid numeric value for --sign-check-window: " << arg.substr(20)
                          << std::endl;
                return false;
            }
            options.signCheckWindowSeconds = *value;
            continue;
        }
        if (arg.rfind("--cfd-path=", 0) == 0) {
            options.cfdPath = arg.substr(11);
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
        std::string label = std::string("accel_lsm") + accLabels[i];
        printEntry(label.c_str(), indices.accelLsm[i]);
    }
    for (int i = 0; i < 3; ++i) {
        std::string label = std::string("gyro") + accLabels[i];
        printEntry(label.c_str(), indices.gyro[i]);
    }
    for (int i = 0; i < 3; ++i) {
        std::string label = std::string("gyro_lsm") + accLabels[i];
        printEntry(label.c_str(), indices.gyroLsm[i]);
    }
    printEntry("quat_w", indices.quaternion[0]);
    printEntry("quat_x", indices.quaternion[1]);
    printEntry("quat_y", indices.quaternion[2]);
    printEntry("quat_z", indices.quaternion[3]);
    printEntry("icm_quat_w", indices.icmQuaternion[0]);
    printEntry("icm_quat_x", indices.icmQuaternion[1]);
    printEntry("icm_quat_y", indices.icmQuaternion[2]);
    printEntry("icm_quat_z", indices.icmQuaternion[3]);
    printEntry("lsm_quat_w", indices.lsmQuaternion[0]);
    printEntry("lsm_quat_x", indices.lsmQuaternion[1]);
    printEntry("lsm_quat_y", indices.lsmQuaternion[2]);
    printEntry("lsm_quat_z", indices.lsmQuaternion[3]);
    printEntry("has_quaternion", indices.hasQuaternionFlag);
    printEntry("has_icm_quaternion", indices.hasIcmQuaternionFlag);
    printEntry("has_lsm_quaternion", indices.hasLsmQuaternionFlag);
    printEntry("main_quaternion_source", indices.mainQuaternionSource);
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
    const ReplaySeedIndices replaySeedIndices = BuildReplaySeedIndices(headers);

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
        if (replaySeedIndices.HasAnySeedColumns() && !options.ignoreLoggedState) {
            std::cout << "Smart seed: logged filtered-state columns detected; using them when available and"
                         " recomputing apogee look-ahead from the seeded state."
                      << std::endl;
        } else if (replaySeedIndices.HasAnySeedColumns() && options.ignoreLoggedState) {
            std::cout << "Ignoring logged filtered-state columns; replaying the hosted flight computer from sensor rails."
                      << std::endl;
        }
    }

    EnvironmentModel::Config environmentConfig;
    ApogeeVehicleParameters vehicleParameters;
    vehicleParameters.centerOfPressureOffsetMeters = settings::vehicle::kCenterOfPressureOffsetMeters;
    vehicleParameters.momentOfInertia = settings::vehicle::kMomentOfInertiaKgM2;
    vehicleParameters.dryMass = settings::vehicle::kDryMassKg;
    EnvironmentModel environment(environmentConfig);
    StandaloneCfdTableStorage seededReplayCfdStorage;
    const bool loadedSeededReplayCfd = LoadStandaloneCfdTable(options.cfdPath, seededReplayCfdStorage) ||
                                       (options.cfdPath == "lib/cfd.csv" &&
                                        LoadStandaloneCfdTable("../lib/cfd.csv", seededReplayCfdStorage));
    FlightComputer flightComputer;
    flightComputer.Begin(options.sigmaAccelXY,
                         options.sigmaAccelZ,
                         options.sigmaAltimeter,
                         options.processXY,
                         options.processZ,
                         options.apogeeTargetMeters,
                         environmentConfig,
                         vehicleParameters,
                         loadedSeededReplayCfd ? &seededReplayCfdStorage.table : nullptr);
    flightComputer.SetSerialReportingEnabled(false);
    ApogeePredictor seededReplayPredictor;
    seededReplayPredictor.SetEnvironment(environment);
    seededReplayPredictor.SetVehicleParameters(vehicleParameters);
    seededReplayPredictor.SetMaxIntegrationSteps(settings::flight::kApogeePredictorMaxSteps);
    if (loadedSeededReplayCfd) {
        seededReplayPredictor.SetForceTable(&seededReplayCfdStorage.table);
    }

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
    std::size_t outlierAltimeterRows = 0;
    std::size_t emittedStates = 0;
    bool hasAltitudeReference = false;
    float altitudeReferenceMeters = 0.0f;
    bool hasPreviousZenith = false;
    float previousZenithRadians = 0.0f;
    float previousTimeSeconds = 0.0f;
    bool hasSignCheckState = false;
    float signCheckAngularRate = 0.0f;
    float signCheckMatchedTime = 0.0f;
    float signCheckBestDelta = std::numeric_limits<float>::infinity();
    FilteredState signCheckState{};
    bool usedSeededStateOutput = false;
    double lastEmittedApogeeMeters = 0.0;
    FlightStatus lastEmittedStatus = FlightStatus::Ground;
    bool hasSeededReplayPrediction = false;
    float lastSeededReplayPredictionTimeSeconds = 0.0f;
    double lastSeededReplayPredictionMeters = 0.0;
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
        if (sample.altitudeFeet < kAltimeterMinFeet || sample.altitudeFeet > kAltimeterMaxFeet) {
            ++skippedRows;
            ++outlierAltimeterRows;
            std::cerr << "Skipping line " << lineNumber << ": altimeter out of range ("
                      << sample.altitudeFeet << " ft)" << std::endl;
            continue;
        }
        ++processedRows;
        FilteredState state;
        FlightStatus emittedStatus = flightComputer.Status();
        bool hasState = !options.ignoreLoggedState &&
                        TryPopulateSeededState(row, replaySeedIndices, state, emittedStatus);
        if (hasState) {
            usedSeededStateOutput = true;
            const double seededAngularRate =
                ComputeSeededAngularRate(state.time,
                                         state.zenith,
                                         previousTimeSeconds,
                                         previousZenithRadians,
                                         hasPreviousZenith);
            state.apogeeEstimate =
                static_cast<float>(RecomputeSeededApogeeEstimate(state,
                                                                 emittedStatus,
                                                                 seededAngularRate,
                                                                 seededReplayPredictor,
                                                                 lastSeededReplayPredictionMeters,
                                                                 lastSeededReplayPredictionTimeSeconds,
                                                                 hasSeededReplayPrediction));
        } else {
            hasState = flightComputer.Update(sample, state);
            emittedStatus = flightComputer.Status();
        }
        if (hasState) {
            ++emittedStates;
            lastEmittedApogeeMeters = state.apogeeEstimate;
            lastEmittedStatus = emittedStatus;
            if (!hasAltitudeReference) {
                altitudeReferenceMeters = state.position[2];
                hasAltitudeReference = true;
            }
            const SampleValueContext::DerivedMetrics derived = ComputeDerivedMetrics(state,
                                                                                     environment,
                                                                                     options.apogeeTargetMeters,
                                                                                     altitudeReferenceMeters,
                                                                                     hasAltitudeReference,
                                                                                     previousTimeSeconds,
                                                                                     previousZenithRadians,
                                                                                     hasPreviousZenith);
            if (!options.quiet) {
                std::cout << state.time << ',' << state.position[2] << ',' << state.velocity[2] << ','
                          << state.apogeeEstimate << ',' << FlightStatusToString(emittedStatus);
                if (!options.extraOutputFields.empty()) {
                    SampleValueContext context{
                        state,
                        sample,
                        altimeterMeasurementMeters,
                        derived,
                    };
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
                snapshots.push_back(SampleSnapshot{state, sample, altimeterMeasurementMeters, derived});
            }
            if (options.signCheckTimeSeconds.has_value()) {
                const float delta = std::fabs(state.time - *options.signCheckTimeSeconds);
                if (delta <= options.signCheckWindowSeconds && delta < signCheckBestDelta) {
                    signCheckBestDelta = delta;
                    signCheckState = state;
                    signCheckMatchedTime = state.time;
                    signCheckAngularRate = derived.zenithRateDps * 0.017453292519943295f;
                    hasSignCheckState = true;
                }
            }
            hasPreviousZenith = true;
            previousZenithRadians = state.zenith;
            previousTimeSeconds = state.time;
        }
    }

    if (graphingEnabled) {
        RenderRequestedGraphs(snapshots, options.graphFields);
    }

    std::cout << std::endl;
    std::cout << "Samples processed: " << processedRows << std::endl;
    std::cout << "Rows skipped: " << skippedRows << std::endl;
    if (outlierAltimeterRows > 0) {
        std::cout << "Altimeter outliers skipped: " << outlierAltimeterRows << std::endl;
    }
    std::cout << "States generated: " << emittedStates << std::endl;
    if (usedSeededStateOutput) {
        std::cout << "Latest emitted apogee: " << lastEmittedApogeeMeters << " m" << std::endl;
        std::cout << "Latest emitted status: " << FlightStatusToString(lastEmittedStatus) << std::endl;
    } else if (flightComputer.ApogeeReached()) {
        std::cout << "Apogee recorded at " << flightComputer.ApogeeAltitude() << " m" << std::endl;
    } else {
        std::cout << "Latest apogee prediction: " << flightComputer.ApogeePrediction() << " m" << std::endl;
    }
    if (options.signCheckTimeSeconds.has_value()) {
        std::cout << std::endl;
        std::cout << "Apogee sign-check requested at t=" << *options.signCheckTimeSeconds
                  << " s (window +/-" << options.signCheckWindowSeconds << " s)" << std::endl;
        if (!hasSignCheckState) {
            std::cout << "No filtered sample matched the requested time window." << std::endl;
        } else {
            StandaloneCfdTableStorage cfdStorage;
            if (!LoadStandaloneCfdTable(options.cfdPath, cfdStorage)) {
                std::cout << "Failed to load CFD table from '" << options.cfdPath << "'." << std::endl;
            } else {
                ApogeePredictor predictor;
                predictor.SetEnvironment(environment);
                predictor.SetVehicleParameters(vehicleParameters);
                predictor.SetForceTable(&cfdStorage.table);
                predictor.SetMaxIntegrationSteps(settings::actuation::kActuationPredictorMaxSteps);

                ApogeeState baseState;
                baseState.altitudeMeters = signCheckState.position[2];
                baseState.horizontalDistanceMeters =
                    math_utils::Magnitude2(signCheckState.position[0], signCheckState.position[1]);
                baseState.verticalVelocity = signCheckState.velocity[2];
                baseState.horizontalVelocity =
                    math_utils::Magnitude2(signCheckState.velocity[0], signCheckState.velocity[1]);
                baseState.zenith = signCheckState.zenith;
                baseState.angularVelocity = signCheckAngularRate;

                const math_utils::Vec3 wind = environment.GradientWind();
                const double relX = static_cast<double>(baseState.verticalVelocity) - static_cast<double>(wind.x);
                const double relY = static_cast<double>(baseState.horizontalVelocity) - static_cast<double>(wind.y);
                const double relSpeed = std::sqrt(relX * relX + relY * relY);
                const double tempK = environment.TemperatureKelvin(baseState.altitudeMeters);
                const double speedOfSound =
                    (tempK > 0.0) ? std::sqrt(constants::kGamma * constants::kGasConstant * tempK) : 0.0;
                const double mach = (speedOfSound > 0.0) ? (relSpeed / speedOfSound) : 0.0;

                auto predictAt = [&](double acsDeg) {
                    ApogeeState state = baseState;
                    state.acsAngleDeg = acsDeg;
                    return predictor.PredictApogee(state);
                };

                const double apg0 = predictAt(0.0);
                const double apg10 = predictAt(10.0);
                const double apg20 = predictAt(20.0);
                const double apg40 = predictAt(40.0);
                const bool monotonicDrop = (apg10 < apg0) && (apg20 < apg10) && (apg40 < apg20);
                const bool stateAscending = baseState.verticalVelocity > 0.0;
                const bool enoughDynamicPressure = mach >= 0.08;

                std::cout << "Matched sample at t=" << signCheckMatchedTime << " s" << std::endl;
                std::cout << "Matched state: z=" << baseState.altitudeMeters
                          << " m, vz=" << baseState.verticalVelocity
                          << " m/s, vh=" << baseState.horizontalVelocity
                          << " m/s, mach=" << mach << std::endl;
                std::cout << "Predicted apogee @ ACS  0 deg: " << apg0 << " m" << std::endl;
                std::cout << "Predicted apogee @ ACS 10 deg: " << apg10 << " m" << std::endl;
                std::cout << "Predicted apogee @ ACS 20 deg: " << apg20 << " m" << std::endl;
                std::cout << "Predicted apogee @ ACS 40 deg: " << apg40 << " m" << std::endl;
                std::cout << "Delta (10-0): " << (apg10 - apg0) << " m" << std::endl;
                std::cout << "Delta (20-10): " << (apg20 - apg10) << " m" << std::endl;
                std::cout << "Delta (40-20): " << (apg40 - apg20) << " m" << std::endl;
                if (!stateAscending) {
                    std::cout << "Sign-check: INCONCLUSIVE (matched sample is not ascending)." << std::endl;
                } else if (!enoughDynamicPressure) {
                    std::cout << "Sign-check: INCONCLUSIVE (mach too low for strong aero sensitivity)." << std::endl;
                } else {
                    std::cout << "Sign-check: "
                              << (monotonicDrop ? "PASS (more ACS lowers apogee)"
                                                : "FAIL (ACS direction likely incorrect)")
                              << std::endl;
                }
            }
        }
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
