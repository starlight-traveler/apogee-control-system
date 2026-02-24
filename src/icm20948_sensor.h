#pragma once

#include "flight_computer.h"

bool Icm20948SensorBegin();
bool Icm20948SensorAcquire(SensorData &out);
