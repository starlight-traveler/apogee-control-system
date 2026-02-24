#pragma once

#include "flight_computer.h"

struct TelemetrySnapshot {
    const SensorData *sensor = nullptr;
    const FilteredState *state = nullptr;
    FlightStatus status = FlightStatus::Ground;
    float servoCommandDeg = 0.0f;
    float servoEffectiveDeg = 0.0f;
    float altitudeAglFeet = 0.0f;
    bool hasPadAltitude = false;
    bool manualActuationOverride = false;
};

void NetworkTelemetryBegin();
void NetworkTelemetryService(const TelemetrySnapshot &snapshot);
bool NetworkTelemetryConnected();
bool NetworkTelemetrySubscriberActive();
bool NetworkTelemetryManualActuationOverride(float &angleDegOut);
