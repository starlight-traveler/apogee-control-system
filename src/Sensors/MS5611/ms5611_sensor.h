#pragma once

#include "barometer_diagnostics.h"

/**
 * @brief Secondary MS5611 barometer interface.
 *
 * This sensor is treated as an independent pressure rail. It is useful for
 * comparing pressure trends and catching primary-barometer weirdness, but it
 * should not silently replace the primary altitude path unless the caller makes
 * that choice explicitly.
 */
/// Initializes the secondary MS5611 barometer.
bool Ms5611SensorBegin();
/// Acquires one fresh MS5611 sample and updates cached diagnostics.
bool Ms5611SensorAcquire();
/// Returns cached timing/latest-sample diagnostics for the MS5611.
BarometerDiagnostics Ms5611SensorGetDiagnostics();
/// Updates the sea-level pressure reference used by pressure-to-altitude conversion.
void Ms5611SensorSetSeaLevelPressureHpa(float pressureHpa);
