#pragma once

#include "barometer_diagnostics.h"
#include "flight_computer.h"

/// Initializes the BMP585 over SPI and configures its runtime sampling mode.
bool Bmp585SensorBegin();
/// Acquires the latest BMP585 sample and updates cached altitude in `SensorData`.
bool Bmp585SensorAcquire(SensorData &out);
/// Returns true once the BMP585 has been initialized successfully.
bool Bmp585SensorIsInitialized();
/// Returns cached timing and latest-sample diagnostics for the BMP585.
BarometerDiagnostics Bmp585SensorGetDiagnostics();
/// Updates the sea-level pressure reference used by altitude conversion.
void Bmp585SensorSetSeaLevelPressureHpa(float pressureHpa);
