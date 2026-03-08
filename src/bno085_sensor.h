#pragma once

#include "flight_computer.h"

/// Initializes the BNO085 orientation/IMU path.
bool Bno085SensorBegin();
/// Acquires the latest BNO085 IMU sample into `SensorData`.
bool Bno085SensorAcquire(SensorData &out);
/// Returns true once the BNO085 path is initialized and healthy enough to serve data.
bool Bno085SensorIsInitialized();
