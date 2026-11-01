#pragma once

#include "barometer_diagnostics.h"
#include "flight_computer.h"

/**
 * @brief Primary BMP585 barometer interface.
 *
 * The barometer path is intentionally explicit about freshness. A cached
 * altitude value is useful for telemetry continuity, but phase detection and
 * control decisions should only trust samples from a successful `Acquire`.
 * This prevents a missing pressure read from looking like a new zero-altitude
 * measurement.
 */
/// Initializes the BMP585 over SPI and configures its runtime sampling mode.
bool Bmp585SensorBegin();
/// Acquires one fresh BMP585 sample and updates cached altitude in `SensorData`.
bool Bmp585SensorAcquire(SensorData &out);
/// Returns true once the BMP585 has been initialized successfully.
bool Bmp585SensorIsInitialized();
/// Returns cached timing and latest-sample diagnostics for the BMP585.
BarometerDiagnostics Bmp585SensorGetDiagnostics();
/// Updates the sea-level pressure reference used by pressure-to-altitude conversion.
void Bmp585SensorSetSeaLevelPressureHpa(float pressureHpa);
