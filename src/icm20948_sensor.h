#pragma once

#include "flight_computer.h"

/// Initializes the optional ICM-20948 IMU path.
bool Icm20948SensorBegin();
/// Acquires one ICM-20948 sample and updates the fused orientation outputs.
bool Icm20948SensorAcquire(SensorData &out);
