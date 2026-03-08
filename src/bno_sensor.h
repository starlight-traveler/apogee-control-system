#pragma once

#include "flight_computer.h"

/// Supported BNO-family device names as user-facing strings.
const char *BnoSensorModelName();
/// Supported BNO transport name as a user-facing string.
const char *BnoSensorTransportName();
/// Initializes whichever BNO family device is selected in settings.
bool BnoSensorBegin();
/// Acquires the latest BNO-family IMU sample into `SensorData`.
bool BnoSensorAcquire(SensorData &out);
/// Returns true once the selected BNO-family device is initialized.
bool BnoSensorIsInitialized();
