#pragma once

#include <stdint.h>

#include "apogee_model.h"
#include "environment_model.h"

/**
 * @brief environment and vehicle settings that can be edited at runtime.
 *
 * these values feed the predictor without recompiling firmware.
 */
struct RuntimeSettings {
    // Atmosphere model used to convert flight state into density, pressure, and sound speed.
    EnvironmentModel::Config environment;
    // Mass/inertia/aero geometry used by the coast apogee predictor.
    ApogeeVehicleParameters vehicle;
};

/**
 * @brief sd-card persistence status for the active runtime settings.
 *
 * this tells telemetry whether the active settings are just flashed defaults
 * or are also mirrored onto the sd card.
 */
struct RuntimeSettingsStorageStatus {
    // True when SD-backed settings can be read or written at all.
    bool storageAvailable = false;
    // True when the settings file was found on the card.
    bool filePresent = false;
    // True when active values came from flashed defaults instead of an SD file.
    bool usingDefaults = true;
    // Last operation results are telemetry-facing, not hard startup requirements.
    bool lastLoadSucceeded = false;
    bool lastSaveSucceeded = false;
    // Set when firmware had to create a first settings file from defaults.
    bool createdDefaultFile = false;
};

/// @brief returns the compile-time defaults from `settings.h`.
RuntimeSettings RuntimeSettingsDefaults();
/// @brief validates one candidate settings payload before it is applied.
bool RuntimeSettingsValidate(const RuntimeSettings &settings);
/// @brief uses flashed defaults and mirrors them to sd when storage is available.
bool RuntimeSettingsUseFlashedDefaults(RuntimeSettings &settings, RuntimeSettingsStorageStatus &status);
/// @brief persists the provided settings to the sd-backed runtime settings file.
bool RuntimeSettingsSave(const RuntimeSettings &settings, RuntimeSettingsStorageStatus &status);
