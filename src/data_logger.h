#pragma once

#include <stdint.h>

#include <SdFat.h>

#include "flight_computer.h"

enum class LogRecordType : uint8_t { Telemetry = 0, Event = 1 };

enum class FlightEventType : uint8_t {
    StageChange = 0,
    FlapActuated = 1,
    FlapSettlingTimerFired = 2,
};

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

struct LogFilePreamble {
    uint8_t magic[8];
    uint16_t formatVersion = 1;
    uint16_t schemaVersion = 1;
    char firmwareGitHash[40];
    uint8_t reserved[12];
};

struct DataLoggerDiagnostics {
    bool initialized = false;
    bool syncPending = false;
    size_t bufferedBytes = 0;
    uint32_t lastWriteDurationUs = 0;
    uint32_t maxWriteDurationUs = 0;
    uint32_t lastSyncDurationUs = 0;
    uint32_t maxSyncDurationUs = 0;
    uint32_t droppedTelemetryRecords = 0;
    uint32_t appendFailures = 0;
};

bool DataLoggerBegin();
void DataLoggerLogTelemetry(const SensorData &sensor,
                            FlightStatus status,
                            const FilteredState *state);
void DataLoggerLogEvent(FlightEventType type,
                        FlightStatus status,
                        float timestamp,
                        float altitudeMeters,
                        float verticalVelocity,
                        float apogeeEstimate);
void DataLoggerForceSync();
void DataLoggerService();
bool DataLoggerIsInitialized();
DataLoggerDiagnostics DataLoggerGetDiagnostics();
// Reads a text file from the SD card line-by-line and invokes the callback for each line.
// Returns false if the SD card or file is unavailable.
bool DataLoggerReadTextFile(const char *path, bool (*lineCallback)(const char *line, void *context), void *context);
// Opens a text file for streaming reads (CSV replay).
bool DataLoggerOpenReadFile(const char *path, FsFile &file);
// Reads the next line into the buffer. Returns true when a line is read.
bool DataLoggerReadLine(FsFile &file, char *line, size_t lineSize);
