#include "runtime_settings.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "data_logger.h"
#include "settings.h"

namespace {

constexpr const char *kRuntimeSettingsPath = "ACSCFG.TXT";

struct ParseContext {
    RuntimeSettings settings = RuntimeSettingsDefaults();
    bool sawAnySetting = false;
    bool parseError = false;
};

bool ParseDoubleValue(const char *text, double &outValue) {
    if (text == nullptr || *text == '\0') {
        return false;
    }
    char *end = nullptr;
    const double value = strtod(text, &end);
    if (end == text || !isfinite(value)) {
        return false;
    }
    outValue = value;
    return true;
}

bool ParseLine(const char *line, void *context) {
    if (line == nullptr || context == nullptr) {
        return false;
    }

    ParseContext *parse = static_cast<ParseContext *>(context);
    while (*line == ' ' || *line == '\t') {
        ++line;
    }
    if (*line == '\0' || *line == '#') {
        return true;
    }

    const char *separator = strchr(line, '=');
    if (separator == nullptr) {
        parse->parseError = true;
        return false;
    }

    char key[64];
    const size_t keyLength = static_cast<size_t>(separator - line);
    if (keyLength == 0 || keyLength >= sizeof(key)) {
        parse->parseError = true;
        return false;
    }
    memcpy(key, line, keyLength);
    key[keyLength] = '\0';

    const char *valueText = separator + 1;
    double value = 0.0;
    if (!ParseDoubleValue(valueText, value)) {
        parse->parseError = true;
        return false;
    }

    parse->sawAnySetting = true;
    if (strcmp(key, "environment.ground_temperature_f") == 0) {
        parse->settings.environment.groundTemperatureF = static_cast<float>(value);
    } else if (strcmp(key, "environment.sea_level_pressure_hpa") == 0) {
        parse->settings.environment.seaLevelPressureHpa = static_cast<float>(value);
    } else if (strcmp(key, "environment.wind_speed_mph") == 0) {
        parse->settings.environment.windSpeedMph = static_cast<float>(value);
    } else if (strcmp(key, "environment.wind_direction_deg") == 0) {
        parse->settings.environment.windDirectionDeg = static_cast<float>(value);
    } else if (strcmp(key, "environment.launch_direction_deg") == 0) {
        parse->settings.environment.launchDirectionDeg = static_cast<float>(value);
    } else if (strcmp(key, "environment.roughness_length_m") == 0) {
        parse->settings.environment.roughnessLengthMeters = static_cast<float>(value);
    } else if (strcmp(key, "environment.gradient_height_m") == 0) {
        parse->settings.environment.gradientHeightMeters = static_cast<float>(value);
    } else if (strcmp(key, "environment.measurement_height_m") == 0) {
        parse->settings.environment.measurementHeightMeters = static_cast<float>(value);
    } else if (strcmp(key, "vehicle.center_of_pressure_offset_m") == 0) {
        parse->settings.vehicle.centerOfPressureOffsetMeters = value;
    } else if (strcmp(key, "vehicle.moment_of_inertia_kg_m2") == 0) {
        parse->settings.vehicle.momentOfInertia = value;
    } else if (strcmp(key, "vehicle.dry_mass_kg") == 0) {
        parse->settings.vehicle.dryMass = value;
    } else {
        parse->parseError = true;
        return false;
    }

    return true;
}

bool PositiveFinite(double value) {
    return isfinite(value) && value > 0.0;
}

bool InClosedRange(double value, double minValue, double maxValue) {
    return isfinite(value) && value >= minValue && value <= maxValue;
}

bool BuildFileContents(const RuntimeSettings &settings, char *buffer, size_t bufferSize) {
    if (buffer == nullptr || bufferSize == 0) {
        return false;
    }

    const int written = snprintf(
        buffer,
        bufferSize,
        "# ACS runtime settings\n"
        "environment.ground_temperature_f=%.6f\n"
        "environment.sea_level_pressure_hpa=%.6f\n"
        "environment.wind_speed_mph=%.6f\n"
        "environment.wind_direction_deg=%.6f\n"
        "environment.launch_direction_deg=%.6f\n"
        "environment.roughness_length_m=%.6f\n"
        "environment.gradient_height_m=%.6f\n"
        "environment.measurement_height_m=%.6f\n"
        "vehicle.center_of_pressure_offset_m=%.10f\n"
        "vehicle.moment_of_inertia_kg_m2=%.10f\n"
        "vehicle.dry_mass_kg=%.10f\n",
        static_cast<double>(settings.environment.groundTemperatureF),
        static_cast<double>(settings.environment.seaLevelPressureHpa),
        static_cast<double>(settings.environment.windSpeedMph),
        static_cast<double>(settings.environment.windDirectionDeg),
        static_cast<double>(settings.environment.launchDirectionDeg),
        static_cast<double>(settings.environment.roughnessLengthMeters),
        static_cast<double>(settings.environment.gradientHeightMeters),
        static_cast<double>(settings.environment.measurementHeightMeters),
        settings.vehicle.centerOfPressureOffsetMeters,
        settings.vehicle.momentOfInertia,
        settings.vehicle.dryMass);
    return written > 0 && static_cast<size_t>(written) < bufferSize;
}

}  // namespace

RuntimeSettings RuntimeSettingsDefaults() {
    RuntimeSettings settings;
    settings.environment = EnvironmentModel::Config{};
    settings.vehicle.centerOfPressureOffsetMeters = settings::vehicle::kCenterOfPressureOffsetMeters;
    settings.vehicle.momentOfInertia = settings::vehicle::kMomentOfInertiaKgM2;
    settings.vehicle.dryMass = settings::vehicle::kDryMassKg;
    return settings;
}

bool RuntimeSettingsValidate(const RuntimeSettings &settings) {
    const EnvironmentModel::Config &environment = settings.environment;
    const ApogeeVehicleParameters &vehicle = settings.vehicle;
    return InClosedRange(environment.groundTemperatureF, -100.0, 150.0) &&
           InClosedRange(environment.seaLevelPressureHpa, 800.0, 1100.0) &&
           InClosedRange(environment.windSpeedMph, 0.0, 200.0) &&
           InClosedRange(environment.windDirectionDeg, 0.0, 360.0) &&
           InClosedRange(environment.launchDirectionDeg, 0.0, 360.0) &&
           InClosedRange(environment.roughnessLengthMeters, 1.0e-4, 100.0) &&
           InClosedRange(environment.gradientHeightMeters, 1.0, 10000.0) &&
           InClosedRange(environment.measurementHeightMeters, 0.1, 1000.0) &&
           InClosedRange(vehicle.centerOfPressureOffsetMeters, -10.0, 10.0) &&
           PositiveFinite(vehicle.momentOfInertia) &&
           PositiveFinite(vehicle.dryMass);
}

bool RuntimeSettingsLoadOrCreate(RuntimeSettings &settings, RuntimeSettingsStorageStatus &status) {
    status = RuntimeSettingsStorageStatus{};
    settings = RuntimeSettingsDefaults();

    if (!DataLoggerIsInitialized()) {
        status.usingDefaults = true;
        return false;
    }

    status.storageAvailable = true;
    ParseContext parse{};
    const bool readOk = DataLoggerReadTextFile(kRuntimeSettingsPath, &ParseLine, &parse);
    if (readOk && !parse.parseError && parse.sawAnySetting && RuntimeSettingsValidate(parse.settings)) {
        settings = parse.settings;
        status.filePresent = true;
        status.usingDefaults = false;
        status.lastLoadSucceeded = true;
        return true;
    }

    status.usingDefaults = true;
    char buffer[640];
    if (!BuildFileContents(settings, buffer, sizeof(buffer))) {
        return false;
    }
    status.createdDefaultFile = DataLoggerWriteTextFile(kRuntimeSettingsPath, buffer);
    status.filePresent = status.createdDefaultFile;
    status.lastSaveSucceeded = status.createdDefaultFile;
    return false;
}

bool RuntimeSettingsSave(const RuntimeSettings &settings, RuntimeSettingsStorageStatus &status) {
    if (!RuntimeSettingsValidate(settings)) {
        status.lastSaveSucceeded = false;
        return false;
    }
    if (!DataLoggerIsInitialized()) {
        status.storageAvailable = false;
        status.lastSaveSucceeded = false;
        return false;
    }

    char buffer[640];
    if (!BuildFileContents(settings, buffer, sizeof(buffer))) {
        status.lastSaveSucceeded = false;
        return false;
    }

    status.storageAvailable = true;
    status.filePresent = true;
    status.usingDefaults = false;
    status.createdDefaultFile = false;
    status.lastSaveSucceeded = DataLoggerWriteTextFile(kRuntimeSettingsPath, buffer);
    return status.lastSaveSucceeded;
}
