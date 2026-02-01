#pragma once

#include "telemetry_types.h"

bool Bmp581SensorBegin();
bool Bmp581SensorAcquire(SensorData &out);
