#pragma once

#include <stdint.h>

#include "apogee_model.h"
#include "environment_model.h"

/// Environment/vehicle settings that can be edited at runtime and persisted.
struct RuntimeSettings {
    EnvironmentModel::Config environment;
    ApogeeVehicleParameters vehicle;
};

/// Storage status published alongside runtime settings so the GUI can report
/// whether the SD-backed copy matches the active runtime values.
struct RuntimeSettingsStorageStatus {
    bool storageAvailable = false;
    bool filePresent = false;
    bool usingDefaults = true;
    bool lastLoadSucceeded = false;
    bool lastSaveSucceeded = false;
    bool createdDefaultFile = false;
};

/// Returns the compile-time defaults from `settings.h`.
RuntimeSettings RuntimeSettingsDefaults();
/// Validates one candidate settings payload before it is applied.
bool RuntimeSettingsValidate(const RuntimeSettings &settings);
/// Uses compile-time defaults and mirrors them to SD when storage is available.
bool RuntimeSettingsUseFlashedDefaults(RuntimeSettings &settings, RuntimeSettingsStorageStatus &status);
/// Persists the provided settings to the SD-backed runtime settings file.
bool RuntimeSettingsSave(const RuntimeSettings &settings, RuntimeSettingsStorageStatus &status);
