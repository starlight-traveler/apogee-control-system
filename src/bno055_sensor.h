#pragma once

#include "flight_computer.h"

bool Bno055SensorBegin();
bool Bno055SensorAcquire(SensorData &out);
