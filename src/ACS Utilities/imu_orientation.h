#pragma once

#include <stdint.h>

namespace imu_orientation {

/*
 * Shared sensor-to-body orientation parameters for all IMU rails.
 *
 * Every sensor has its own chip coordinate frame and board mounting. The flight
 * code wants one rocket body frame:
 *
 *   +X: rocket-forward / along the airframe reference axis used by the estimator
 *   +Y: lateral body axis
 *   +Z: body-up axis for the mounted avionics stack
 *
 * Keeping these transforms centralized makes it easier to compare rails. If two
 * sensors disagree after these matrices, the disagreement is more likely sensor
 * quality/calibration than a hidden axis convention mismatch.
 */

// BNO055/BNO085 mounting:
// body +X = sensor -X
// body +Y = sensor -Y
// body +Z = sensor +Z
inline constexpr float kBnoMountRotation[3][3] = {
    {-1.0f, 0.0f, 0.0f},
    {0.0f, -1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
};

// WT901 mounting:
// body +X = sensor +Z
// body +Y = sensor -X
// body +Z = sensor +Y
inline constexpr float kWt901MountRotation[3][3] = {
    {0.0f, 0.0f, 1.0f},
    {-1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
};

// Pulse 20 / Ellipse rail mounting:
// body +X = sensor -Y
// body +Y = sensor +X
// body +Z = sensor +Z
inline constexpr float kEllipse20MountRotation[3][3] = {
    {0.0f, -1.0f, 0.0f},
    {1.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
};

// LSM9DS1 mounting:
// body +X = sensor +X
// body +Y = sensor -Y
// body +Z = sensor +Z
inline constexpr uint8_t kLsm9ds1AxisMap[3] = {0, 1, 2};
inline constexpr int8_t kLsm9ds1AxisSign[3] = {1, -1, 1};
inline constexpr float kLsm9ds1MountRotation[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
};

// ICM-20948 mounting:
// body +X = sensor -X
// body +Y = sensor -Y
// body +Z = sensor +Z
inline constexpr float kIcm20948MountRotation[3][3] = {
    {-1.0f, 0.0f, 0.0f},
    {0.0f, -1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
};

// ICM magnetometer axis remap into the calibrated accel/gyro frame.
inline constexpr uint8_t kIcm20948MagAxisMap[3] = {0, 1, 2};
inline constexpr int8_t kIcm20948MagAxisSign[3] = {1, -1, -1};

}  // namespace imu_orientation
