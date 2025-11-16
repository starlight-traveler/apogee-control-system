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

constexpr uint32_t kFlushIntervalMicros = 50000;  // Flush at least every 50 ms.

constexpr const char *kLogPrefix = "SENS";
constexpr const char *kLogExtension = "BIN";

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

    g_bufferPosition = 0;
    g_lastFlushMicros = micros();
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
        Serial.println("SD card initialization failed.");
        return false;
    }

    char filename[32];
    if (!NextLogFilename(filename, sizeof(filename))) {
        Serial.println("Unable to create log filename.");
        return false;
    }

    g_logFile = g_sd.open(filename, O_WRONLY | O_CREAT | O_TRUNC);
    if (!g_logFile) {
        Serial.println("Failed to open log file.");
        return false;
    }

    g_bufferPosition = 0;
    g_lastFlushMicros = micros();

    g_loggerInitialized = true;
    Serial.print("Logging sensor data to ");
    Serial.println(filename);
    return true;
}

void DataLoggerLog(const SensorData &data) {
    if (!g_loggerInitialized) {
        return;
    }

    static_assert(std::is_trivially_copyable<SensorData>::value, "SensorData must be trivially copyable.");

    if (g_bufferPosition + sizeof(SensorData) > kBufferSize) {
        if (!FlushBuffer()) {
            Serial.println("Failed to flush sensor log buffer.");
            return;
        }
    }

    memcpy(g_buffer + g_bufferPosition, &data, sizeof(SensorData));
    g_bufferPosition += sizeof(SensorData);
}

void DataLoggerService() {
    if (!g_loggerInitialized) {
        return;
    }

    if (g_bufferPosition == 0) {
        return;
    }

    // DEBUG: always flush
    if (!FlushBuffer()) {
        Serial.println("Failed to flush sensor log buffer.");
    }
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