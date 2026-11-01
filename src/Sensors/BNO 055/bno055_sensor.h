#pragma once

#include "bno_sensor.h"

/**
 * @brief BNO055-specific implementation behind the generic BNO wrapper.
 *
 * The BNO055 can provide a fused orientation directly from the chip. This file
 * keeps that product isolated from the BNO085 path so main flight code can ask
 * for "the selected BNO sample" without caring which chip is installed.
 */
/// Initializes the BNO055 orientation/IMU path.
bool Bno055SensorBegin();
/// Acquires the latest fresh BNO055 IMU sample into `SensorData`.
bool Bno055SensorAcquire(SensorData &out);
/// Returns true once the BNO055 path is initialized and healthy enough to serve data.
bool Bno055SensorIsInitialized();
/// Returns the current transport and health of the BNO055 path.
BnoDiagnostics Bno055SensorGetDiagnostics();
/// Returns the latest cached BNO055 sample in body-frame coordinates.
BnoSample Bno055SensorGetSample();
