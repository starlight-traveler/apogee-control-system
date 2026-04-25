#include "runtime_settings.h"

#include <math.h>
#include <stdio.h>

#include "data_logger.h"
#include "settings.h"

namespace {

constexpr const char *kRuntimeSettingsPath = "ACSCFG.TXT";

bool PositiveFinite(double value) {
    return isfinite(value) && value > 0.0;
}

bool InClosedRange(double value, double minValue, double maxValue) {
    return isfinite(value) && value >= minValue && value <= maxValue;
}

bool RuntimeSettingsFilePresent() {
    FsFile file;
    if (!DataLoggerOpenReadFile(kRuntimeSettingsPath, file)) {
        return false;
    }
    file.close();
    return true;
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

bool RuntimeSettingsUseFlashedDefaults(RuntimeSettings &settings, RuntimeSettingsStorageStatus &status) {
    status = RuntimeSettingsStorageStatus{};
    settings = RuntimeSettingsDefaults();
    status.usingDefaults = true;

    if (!DataLoggerIsInitialized()) {
        return false;
    }

    status.storageAvailable = true;
    const bool filePresent = RuntimeSettingsFilePresent();
    status.filePresent = filePresent;
    char buffer[640];
    if (!BuildFileContents(settings, buffer, sizeof(buffer))) {
        return false;
    }

    status.lastSaveSucceeded = DataLoggerWriteTextFile(kRuntimeSettingsPath, buffer);
    status.createdDefaultFile = !filePresent && status.lastSaveSucceeded;
    status.filePresent = filePresent || status.lastSaveSucceeded;
    return status.lastSaveSucceeded;
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
