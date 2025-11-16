#pragma once

#include "flight_computer.h"

bool Bmp581SensorBegin();
bool Bmp581SensorAcquire(SensorData &out);
