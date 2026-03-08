#pragma once

#include "barometer_diagnostics.h"
#include "flight_computer.h"

bool Bmp585SensorBegin();
bool Bmp585SensorAcquire(SensorData &out);
bool Bmp585SensorIsInitialized();
BarometerDiagnostics Bmp585SensorGetDiagnostics();
