#pragma once

#include "flight_computer.h"

bool DataLoggerBegin();
void DataLoggerLog(const SensorData &data);
void DataLoggerService();