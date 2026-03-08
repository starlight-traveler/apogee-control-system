#pragma once

#include "barometer_diagnostics.h"

/// Initializes the secondary MS5611 barometer.
bool Ms5611SensorBegin();
/// Acquires one MS5611 sample and updates cached diagnostics.
bool Ms5611SensorAcquire();
/// Returns cached timing/latest-sample diagnostics for the MS5611.
BarometerDiagnostics Ms5611SensorGetDiagnostics();
