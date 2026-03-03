#pragma once

#include "barometer_diagnostics.h"

bool Ms5611SensorBegin();
bool Ms5611SensorAcquire();
BarometerDiagnostics Ms5611SensorGetDiagnostics();
