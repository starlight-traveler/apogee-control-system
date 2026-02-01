#pragma once

#include <stdint.h>

struct SensorData {
    float timestamp = 0.0f;
    float altitudeFeet = 0.0f;
    float accelBNO[3] = {0.0f, 0.0f, 0.0f};
    float accelICM[3] = {0.0f, 0.0f, 0.0f};
    float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float gyro[3] = {0.0f, 0.0f, 0.0f};
    bool hasQuaternion = false;
};

