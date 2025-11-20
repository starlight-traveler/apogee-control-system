#pragma once

#include <stdint.h>

#include "flight_computer.h"

enum class LogRecordType : uint8_t { Telemetry = 0, Event = 1 };

enum class FlightEventType : uint8_t { StageChange = 0 };

struct LogRecordHeader {
    uint8_t recordType = 0;
    uint8_t subtype = 0;
    uint8_t flags = 0;
    uint8_t reserved = 0;
};

struct TelemetryLogRecord {
    LogRecordHeader header;
    SensorData sensor;
    FilteredState state;
};

struct EventLogRecord {
    LogRecordHeader header;
    float timestamp = 0.0f;
    float altitudeMeters = 0.0f;
    float verticalVelocity = 0.0f;
    float apogeeEstimate = 0.0f;
};

bool DataLoggerBegin();
void DataLoggerSetSerialLoggingEnabled(bool enabled);
void DataLoggerLogTelemetry(const SensorData &sensor,
                            FlightStatus status,
                            const FilteredState *state);
void DataLoggerLogEvent(FlightEventType type,
                        FlightStatus status,
                        float timestamp,
                        float altitudeMeters,
                        float verticalVelocity,
                        float apogeeEstimate);
void DataLoggerService();
bool DataLoggerIsInitialized();