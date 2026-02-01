#pragma once

#include "telemetry_types.h"

bool Bno085SensorBegin();
bool Bno085SensorAcquire(SensorData &out);
