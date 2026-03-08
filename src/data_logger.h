#pragma once

#include <stdint.h>

#include <SdFat.h>

#include "flight_computer.h"

/// Binary log record type stored in the SD-card sensor log.
enum class LogRecordType : uint8_t { Telemetry = 0, Event = 1 };

/// Event subtypes stored in `EventLogRecord.header.subtype`.
enum class FlightEventType : uint8_t {
    StageChange = 0,
    FlapActuated = 1,
    FlapSettlingTimerFired = 2,
};

/// Common 4-byte header that prefixes every binary log record.
struct LogRecordHeader {
    uint8_t recordType = 0;
    uint8_t subtype = 0;
    uint8_t flags = 0;
    uint8_t reserved = 0;
};

/// Telemetry payload written for each main-loop sample.
///
/// `SensorData` and `FilteredState` sizes are part of the on-disk schema and
/// must stay aligned with the decoder descriptor table.
struct TelemetryLogRecord {
    LogRecordHeader header;
    SensorData sensor;
    FilteredState state;
};

/// Event payload written for sparse flight events.
struct EventLogRecord {
    LogRecordHeader header;
    float timestamp = 0.0f;
    float altitudeMeters = 0.0f;
    float verticalVelocity = 0.0f;
    float apogeeEstimate = 0.0f;
};

/// File preamble written once at the top of every binary log.
///
/// The decoder uses this to confirm file identity, schema version, and
/// firmware provenance before interpreting the binary payloads.
struct LogFilePreamble {
    uint8_t magic[8];
    uint16_t formatVersion = 1;
    uint16_t schemaVersion = 1;
    char firmwareGitHash[40];
    uint8_t reserved[12];
};

/// Runtime diagnostics exported by the logger for timing and backpressure analysis.
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

/// Initializes the SD logger and creates the next sequential log file.
bool DataLoggerBegin();
/// Appends one telemetry record to the RAM buffer.
void DataLoggerLogTelemetry(const SensorData &sensor,
                            FlightStatus status,
                            const FilteredState *state);
/// Appends a high-priority event record and requests an earlier sync.
void DataLoggerLogEvent(FlightEventType type,
                        FlightStatus status,
                        float timestamp,
                        float altitudeMeters,
                        float verticalVelocity,
                        float apogeeEstimate);
/// Flushes and syncs the current log file immediately.
void DataLoggerForceSync();
/// Services buffered writes and deferred syncs from the main loop.
void DataLoggerService();
/// Returns true while the logger is available for appends.
bool DataLoggerIsInitialized();
/// Returns current logger timing/backpressure diagnostics.
DataLoggerDiagnostics DataLoggerGetDiagnostics();
/// Reads a text file from the SD card line-by-line and invokes the callback for each line.
bool DataLoggerReadTextFile(const char *path, bool (*lineCallback)(const char *line, void *context), void *context);
/// Replaces a text file on the SD card with `contents`.
bool DataLoggerWriteTextFile(const char *path, const char *contents);
/// Opens a text file for streaming reads, primarily for CSV replay.
bool DataLoggerOpenReadFile(const char *path, FsFile &file);
/// Reads the next newline-delimited text row into `line`.
bool DataLoggerReadLine(FsFile &file, char *line, size_t lineSize);
