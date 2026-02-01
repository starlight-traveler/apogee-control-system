#include "data_logger.h"

#include <Arduino.h>
#include <SdFat.h>
#include <stdio.h>
#include <string.h>
#include <type_traits>

namespace {

SdFs g_sd;
FsFile g_logFile;

constexpr size_t kBufferSize = 4096;
alignas(uint32_t) uint8_t g_buffer[kBufferSize];
size_t g_bufferPosition = 0;
uint32_t g_lastFlushMicros = 0;
bool g_loggerInitialized = false;
bool g_serialLoggingEnabled = true;

constexpr uint32_t kFlushIntervalMicros = 50000;  // Flush at least every 50 ms.

constexpr const char *kLogPrefix = "SENS";
constexpr const char *kLogExtension = "BIN";

static_assert(sizeof(SensorData) == 64, "SensorData size mismatch.");
static_assert(sizeof(FilteredState) == 60, "FilteredState size mismatch.");
static_assert(sizeof(TelemetryLogRecord) == 128, "TelemetryLogRecord size mismatch.");
static_assert(sizeof(EventLogRecord) == 20, "EventLogRecord size mismatch.");

static_assert(std::is_trivially_copyable<SensorData>::value, "SensorData must be trivially copyable.");
static_assert(std::is_trivially_copyable<FilteredState>::value, "FilteredState must be trivially copyable.");
static_assert(std::is_trivially_copyable<TelemetryLogRecord>::value,
              "TelemetryLogRecord must be trivially copyable.");
static_assert(std::is_trivially_copyable<EventLogRecord>::value,
              "EventLogRecord must be trivially copyable.");

bool FlushBuffer() {
    if (!g_loggerInitialized || g_bufferPosition == 0) {
        return true;
    }

    const size_t bytesWritten = g_logFile.write(g_buffer, g_bufferPosition);
    if (bytesWritten != g_bufferPosition) {
        g_logFile.close();
        g_loggerInitialized = false;
        return false;
    }

    if (!g_logFile.sync()) {
        g_logFile.close();
        g_loggerInitialized = false;
        return false;
    }

    g_bufferPosition = 0;
    g_lastFlushMicros = micros();
    return true;
}

bool AppendRecord(const void *record, size_t size) {
    if (!g_loggerInitialized) {
        return false;
    }

    if (size > kBufferSize) {
        return false;
    }

    if (g_bufferPosition + size > kBufferSize) {
        if (!FlushBuffer()) {
            return false;
        }
    }

    memcpy(g_buffer + g_bufferPosition, record, size);
    g_bufferPosition += size;
    return true;
}

bool NextLogFilename(char *buffer, size_t length) {
    for (uint16_t index = 0; index < 1000; ++index) {
        const int written = snprintf(buffer, length, "%s%03u.%s", kLogPrefix, index, kLogExtension);
        if (written <= 0 || static_cast<size_t>(written) >= length) {
            return false;
        }

        if (!g_sd.exists(buffer)) {
            return true;
        }
    }
    return false;
}

}  // namespace

bool DataLoggerBegin() {
    if (g_loggerInitialized) {
        return true;
    }

    if (!g_sd.begin(SdioConfig(FIFO_SDIO))) {
        if (g_serialLoggingEnabled && Serial) {
            Serial.println("SD card initialization failed.");
        }
        return false;
    }

    char filename[32];
    if (!NextLogFilename(filename, sizeof(filename))) {
        if (g_serialLoggingEnabled && Serial) {
            Serial.println("Unable to create log filename.");
        }
        return false;
    }

    g_logFile = g_sd.open(filename, O_WRONLY | O_CREAT | O_TRUNC);
    if (!g_logFile) {
        if (g_serialLoggingEnabled && Serial) {
            Serial.println("Failed to open log file.");
        }
        return false;
    }

    g_bufferPosition = 0;
    g_lastFlushMicros = micros();

    g_loggerInitialized = true;
    if (g_serialLoggingEnabled && Serial) {
        Serial.print("Logging sensor data to ");
        Serial.println(filename);
    }
    return true;
}

void DataLoggerSetSerialLoggingEnabled(bool enabled) {
    g_serialLoggingEnabled = enabled;
}

void DataLoggerLogTelemetry(const SensorData &sensor, FlightStatus status, const FilteredState *state) {
    if (!g_loggerInitialized) {
        return;
    }

    TelemetryLogRecord record{};
    record.header.recordType = static_cast<uint8_t>(LogRecordType::Telemetry);
    record.header.subtype = static_cast<uint8_t>(status);
    record.header.flags = state != nullptr ? 1 : 0;
    record.sensor = sensor;
    if (state != nullptr) {
        record.state = *state;
    }

    if (!AppendRecord(&record, sizeof(record)) && g_serialLoggingEnabled && Serial) {
        Serial.println("Failed to append telemetry record to log.");
    }
}

void DataLoggerLogEvent(FlightEventType type,
                        FlightStatus status,
                        float timestamp,
                        float altitudeMeters,
                        float verticalVelocity,
                        float apogeeEstimate) {
    if (!g_loggerInitialized) {
        return;
    }

    EventLogRecord record{};
    record.header.recordType = static_cast<uint8_t>(LogRecordType::Event);
    record.header.subtype = static_cast<uint8_t>(type);
    record.header.flags = static_cast<uint8_t>(status);
    record.timestamp = timestamp;
    record.altitudeMeters = altitudeMeters;
    record.verticalVelocity = verticalVelocity;
    record.apogeeEstimate = apogeeEstimate;

    if (!AppendRecord(&record, sizeof(record)) && g_serialLoggingEnabled && Serial) {
        Serial.println("Failed to append event record to log.");
    }
}

void DataLoggerService() {
    if (!g_loggerInitialized) {
        return;
    }

    if (g_bufferPosition == 0) {
        return;
    }

    if (!FlushBuffer() && g_serialLoggingEnabled && Serial) {
        Serial.println("Failed to flush sensor log buffer.");
    }
}

bool DataLoggerIsInitialized() {
    return g_loggerInitialized;
}


// void DataLoggerService() {
//     if (!g_loggerInitialized) {
//         return;
//     }

//     const uint32_t now = micros();
//     if (g_bufferPosition == 0) {
//         return;
//     }

//     if ((now - g_lastFlushMicros >= kFlushIntervalMicros) || g_bufferPosition >= kBufferSize) {
//         if (!FlushBuffer()) {
//             Serial.println("Failed to flush sensor log buffer.");
//         }
//     }
// }