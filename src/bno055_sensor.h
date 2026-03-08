#pragma once

#include "flight_computer.h"

/// Initializes the BNO055 orientation/IMU path.
bool Bno055SensorBegin();
/// Acquires the latest BNO055 IMU sample into `SensorData`.
bool Bno055SensorAcquire(SensorData &out);
/// Returns true once the BNO055 path is initialized and healthy enough to serve data.
bool Bno055SensorIsInitialized();
