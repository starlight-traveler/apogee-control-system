#pragma once

#include "barometer_diagnostics.h"
#include "flight_computer.h"

bool Bmp585SensorBegin();
bool Bmp585SensorAcquire(SensorData &out);
BarometerDiagnostics Bmp585SensorGetDiagnostics();
