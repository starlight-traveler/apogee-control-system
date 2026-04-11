#include "data_logger.h"

#include <Arduino.h>
#include <SdFat.h>
#include <stdio.h>
#include <string.h>
#include <type_traits>

#include "bno_sensor.h"
#include "constants.h"
#include "settings.h"
#include "serial_logging.h"
#include "wt901_sensor.h"

namespace {

SdFs g_sd;
FsFile g_logFile;

constexpr size_t kBufferSize = settings::build::kDataLoggerBufferSize;
alignas(uint32_t) uint8_t g_buffer[kBufferSize];
size_t g_bufferPosition = 0;
uint32_t g_lastFlushMicros = 0;
uint32_t g_lastSyncMicros = 0;
bool g_loggerInitialized = false;
bool g_syncPending = false;
bool g_highPriorityFlushPending = false;
DataLoggerDiagnostics g_diagnostics;

constexpr uint32_t kFlushIntervalMicros = settings::build::kDataLoggerFlushIntervalUs;
constexpr uint32_t kSyncIntervalMicros = settings::build::kDataLoggerSyncIntervalUs;
constexpr size_t kMinFlushBytes = settings::build::kDataLoggerMinFlushBytes;
constexpr uint32_t kPreallocateBytes = settings::build::kDataLoggerPreallocateBytes;
constexpr uint32_t kHardFlushIntervalMicros = kFlushIntervalMicros * 4u;

constexpr const char *kLogPrefix = "SENS";
constexpr const char *kLogExtension = "BIN";
constexpr uint16_t kLogFileFormatVersion = 1;
constexpr uint16_t kLogSchemaVersion = 10;
constexpr uint8_t kLogMagic[8] = {'A', 'C', 'S', 'N', 'D', 'R', 'T', '1'};

#if defined(ACS_FIRMWARE_GIT_HASH)
constexpr const char *kFirmwareGitHash = ACS_FIRMWARE_GIT_HASH;
#else
constexpr const char *kFirmwareGitHash = "unknown";
#endif

// These static asserts are the first line of defense for the binary schema.
// If any of them changes, the decoder table in tools/decode/native must be
// updated in lockstep before new logs are trusted.
static_assert(sizeof(LoggedTelemetrySample) == 248, "LoggedTelemetrySample size mismatch.");
static_assert(sizeof(TelemetryLogRecord) == 252, "TelemetryLogRecord size mismatch.");
static_assert(sizeof(EventLogRecord) == 28, "EventLogRecord size mismatch.");
static_assert(sizeof(LogFilePreamble) == 64, "LogFilePreamble size mismatch.");

static_assert(std::is_trivially_copyable<LoggedTelemetrySample>::value,
              "LoggedTelemetrySample must be trivially copyable.");
static_assert(std::is_trivially_copyable<TelemetryLogRecord>::value,
              "TelemetryLogRecord must be trivially copyable.");
static_assert(std::is_trivially_copyable<EventLogRecord>::value,
              "EventLogRecord must be trivially copyable.");

constexpr float kRadToDeg = 57.295779513082320876f;

void CopyVec3(const float src[3], float dst[3]) {
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
}

void CopyQuat(const float src[4], float dst[4]) {
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
    dst[3] = src[3];
}

LoggedTelemetrySample BuildLoggedTelemetrySample(const SensorData &sensor,
                                                 const FilteredState *state,
                                                 float flapCommandDeg,
                                                 float flapEffectiveDeg) {
    const BnoDiagnostics bnoDiagnostics = BnoSensorGetDiagnostics();
    const Wt901Diagnostics wt901Diagnostics = Wt901SensorGetDiagnostics();
    LoggedTelemetrySample sample{};
    sample.timestamp = sensor.timestamp;
    sample.altitudeFeet = sensor.altitudeFeet;
    CopyVec3(sensor.accelICM, sample.accelIcm);
    CopyVec3(sensor.gyro, sample.gyroIcm);
    CopyQuat(sensor.quaternion, sample.quaternionMain);
    CopyQuat(sensor.icmQuaternion, sample.quaternionIcm);
    CopyVec3(sensor.accelLSM, sample.accelLsm);
    CopyVec3(sensor.gyroLSM, sample.gyroLsm);
    CopyQuat(sensor.quaternionLSM, sample.quaternionLsm);
    sample.flapCommandDeg = flapCommandDeg;
    sample.flapEffectiveDeg = flapEffectiveDeg;
    sample.mainQuaternionSource = sensor.mainQuaternionSource;
    sample.hasQuaternion = sensor.hasQuaternion ? 1u : 0u;
    sample.hasIcmQuaternion = sensor.hasIcmQuaternion ? 1u : 0u;
    sample.hasLsmQuaternion = sensor.hasLsmQuaternion ? 1u : 0u;
    sample.icmAccelSaturated = sensor.icmAccelSaturated ? 1u : 0u;
    sample.icmGyroSaturated = sensor.icmGyroSaturated ? 1u : 0u;
    sample.actuationIsSettling = (sensor.actuationIsSettling > 0.5f) ? 1u : 0u;
    CopyVec3(sensor.accelBNO, sample.accelBno);
    CopyVec3(sensor.gyroBNO, sample.gyroBno);
    CopyQuat(sensor.quaternionBNO, sample.quaternionBno);
    CopyVec3(bnoDiagnostics.yprDeg, sample.bnoYprDeg);
    CopyVec3(wt901Diagnostics.accelBodyMps2, sample.accelWt901);
    CopyVec3(wt901Diagnostics.yprDeg, sample.wt901YprDeg);
    CopyVec3(wt901Diagnostics.gyroBodyRadPerSec, sample.gyroWt901);
    CopyQuat(wt901Diagnostics.quaternion, sample.quaternionWt901);
    sample.hasBnoQuaternion = sensor.hasBnoQuaternion ? 1u : 0u;
    sample.hasBnoYpr = bnoDiagnostics.hasQuaternion ? 1u : 0u;
    sample.hasWt901Accel = wt901Diagnostics.hasAccel ? 1u : 0u;
    sample.hasWt901Ypr = wt901Diagnostics.hasYpr ? 1u : 0u;
    sample.hasWt901Gyro = wt901Diagnostics.hasGyro ? 1u : 0u;
    sample.hasWt901Quaternion = wt901Diagnostics.hasQuaternion ? 1u : 0u;

    if (state != nullptr) {
        sample.altitudeAglFeet = state->position[2] * constants::kMetersToFeet;
        sample.verticalVelocityFps = state->velocity[2] * constants::kMetersToFeet;
        sample.zenithDeg = state->zenith * kRadToDeg;
        sample.apogeeEstimateFeet = state->apogeeEstimate * constants::kMetersToFeet;
    }

    return sample;
}

/// Tracks the maximum observed value for a timing diagnostic.
void UpdateMax(uint32_t sample, uint32_t &maximum) {
    if (sample > maximum) {
        maximum = sample;
    }
}

/// Tears down the logger after an unrecoverable write/sync failure.
void FailLogger() {
    g_logFile.close();
    g_loggerInitialized = false;
    g_syncPending = false;
    g_highPriorityFlushPending = false;
    g_bufferPosition = 0;
    g_diagnostics.initialized = false;
    g_diagnostics.syncPending = false;
    g_diagnostics.bufferedBytes = 0;
}

/// Syncs pending metadata/data to the SD card when a deferred sync is due.
bool SyncFile() {
    if (!g_loggerInitialized || !g_syncPending) {
        return true;
    }
    const uint32_t startMicros = micros();
    if (!g_logFile.sync()) {
        FailLogger();
        return false;
    }
    const uint32_t durationUs = micros() - startMicros;
    g_lastSyncMicros = micros();
    g_syncPending = false;
    g_diagnostics.lastSyncDurationUs = durationUs;
    UpdateMax(durationUs, g_diagnostics.maxSyncDurationUs);
    g_diagnostics.syncPending = false;
    return true;
}

/// Flushes the RAM buffer to the log file.
///
/// Telemetry writes are buffered to reduce loop latency; high-priority event
/// records request a later sync so flight-critical control work can continue.
bool FlushBuffer(bool requestSync) {
    
    if (!g_loggerInitialized || g_bufferPosition == 0) {
        if (requestSync) {
            g_syncPending = true;
            g_diagnostics.syncPending = true;
        }
        return true;
    }

    const uint32_t startMicros = micros();
    const size_t bytesWritten = g_logFile.write(g_buffer, g_bufferPosition);
    if (bytesWritten != g_bufferPosition) {
        FailLogger();
        return false;
    }
    const uint32_t durationUs = micros() - startMicros;
    g_diagnostics.lastWriteDurationUs = durationUs;
    UpdateMax(durationUs, g_diagnostics.maxWriteDurationUs);

    g_bufferPosition = 0;
    g_lastFlushMicros = micros();
    g_syncPending = g_syncPending || requestSync;
    g_highPriorityFlushPending = false;
    g_diagnostics.syncPending = g_syncPending;
    g_diagnostics.bufferedBytes = 0;
    return true;
}

/// Appends one binary record to the RAM buffer.
///
/// Low-priority telemetry may be dropped when the buffer is full; high-priority
/// event records force a flush instead.
bool AppendRecord(const void *record, size_t size, bool highPriority) {
    if (!g_loggerInitialized) {
        return false;
    }

    if (size > kBufferSize) {
        return false;
    }

    if (g_bufferPosition + size > kBufferSize) {
        if (!highPriority) {
            ++g_diagnostics.droppedTelemetryRecords;
            return true;
        }
        if (!FlushBuffer(false)) {
            ++g_diagnostics.appendFailures;
            return false;
        }
    }

    memcpy(g_buffer + g_bufferPosition, record, size);
    g_bufferPosition += size;
    g_diagnostics.bufferedBytes = g_bufferPosition;
    return true;
}

/// Finds the next sequential `SENSxxx.BIN` filename on the SD card.
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

/// Writes the log preamble that the native decoder validates before parsing.
bool WriteLogPreamble() {
    LogFilePreamble preamble{};
    memcpy(preamble.magic, kLogMagic, sizeof(kLogMagic));
    preamble.formatVersion = kLogFileFormatVersion;
    preamble.schemaVersion = kLogSchemaVersion;
    strncpy(preamble.firmwareGitHash, kFirmwareGitHash, sizeof(preamble.firmwareGitHash) - 1);
    const size_t bytesWritten = g_logFile.write(&preamble, sizeof(preamble));
    if (bytesWritten != sizeof(preamble)) {
        return false;
    }
    if (!g_logFile.sync()) {
        return false;
    }
    g_lastSyncMicros = micros();
    return true;
}

}  // namespace

/// Initializes the SD card, opens the next log file, and writes the preamble.
bool DataLoggerBegin() {
    if (g_loggerInitialized) {
        return true;
    }

    if (!g_sd.begin(SdioConfig(FIFO_SDIO))) {
        LOG_PRINTLN("SD card initialization failed.");
        return false;
    }

    char filename[32];
    if (!NextLogFilename(filename, sizeof(filename))) {
        LOG_PRINTLN("Unable to create log filename.");
        return false;
    }

    g_logFile = g_sd.open(filename, O_WRONLY | O_CREAT | O_TRUNC);
    if (!g_logFile) {
        LOG_PRINTLN("Failed to open log file.");
        return false;
    }

    if (kPreallocateBytes > 0 && !g_logFile.preAllocate(kPreallocateBytes)) {
        LOG_PRINTLN("SD preallocation skipped.");
    }

    if (!WriteLogPreamble()) {
        g_logFile.close();
        LOG_PRINTLN("Failed to write log preamble.");
        return false;
    }

    g_bufferPosition = 0;
    g_lastFlushMicros = micros();
    g_lastSyncMicros = g_lastFlushMicros;
    g_syncPending = false;
    g_highPriorityFlushPending = false;
    g_diagnostics = DataLoggerDiagnostics{};

    g_loggerInitialized = true;
    g_diagnostics.initialized = true;
    LOG_PRINT("Logging sensor data to ");
    LOG_PRINTLN(filename);
    return true;
}

/// Serializes and buffers one telemetry record.
void DataLoggerLogTelemetry(const SensorData &sensor,
                            FlightStatus status,
                            const FilteredState *state,
                            float flapCommandDeg,
                            float flapEffectiveDeg) {
    if (!g_loggerInitialized) {
        return;
    }

    TelemetryLogRecord record{};
    record.header.recordType = static_cast<uint8_t>(LogRecordType::Telemetry);
    record.header.subtype = static_cast<uint8_t>(status);
    record.header.flags = state != nullptr ? 1 : 0;
    record.sample = BuildLoggedTelemetrySample(sensor, state, flapCommandDeg, flapEffectiveDeg);

    if (!AppendRecord(&record, sizeof(record), false)) {
        LOG_PRINTLN("Failed to append telemetry record to log.");
    }
}

/// Serializes and buffers one high-priority event record.
void DataLoggerLogEvent(FlightEventType type,
                        FlightStatus status,
                        float timestamp,
                        float altitudeAglFeet,
                        float verticalVelocityFps,
                        float apogeeEstimateFeet,
                        float flapCommandDeg,
                        float flapEffectiveDeg) {
    if (!g_loggerInitialized) {
        return;
    }

    EventLogRecord record{};
    record.header.recordType = static_cast<uint8_t>(LogRecordType::Event);
    record.header.subtype = static_cast<uint8_t>(type);
    record.header.flags = static_cast<uint8_t>(status);
    record.timestamp = timestamp;
    record.altitudeAglFeet = altitudeAglFeet;
    record.verticalVelocityFps = verticalVelocityFps;
    record.apogeeEstimateFeet = apogeeEstimateFeet;
    record.flapCommandDeg = flapCommandDeg;
    record.flapEffectiveDeg = flapEffectiveDeg;

    if (!AppendRecord(&record, sizeof(record), true)) {
        LOG_PRINTLN("Failed to append event record to log.");
        return;
    }
    g_highPriorityFlushPending = true;
    g_syncPending = true;
    g_diagnostics.syncPending = true;
}

/// Forces an immediate flush and sync of the current log file.
void DataLoggerForceSync() {
    if (!g_loggerInitialized) {
        return;
    }
    if (g_bufferPosition > 0 && !FlushBuffer(true)) {
        LOG_PRINTLN("Failed to flush sensor log buffer before sync.");
        return;
    }
    if (!SyncFile()) {
        LOG_PRINTLN("Failed to sync sensor log.");
    }
}

/// Background logger service called from the main loop.
///
/// This is intentionally non-blocking in the common case: writes are buffered
/// and sync is deferred to reduce the chance of a long SD stall in control
/// code.
void DataLoggerService() {
    if (!g_loggerInitialized) {
        return;
    }

    const uint32_t now = micros();
    const uint32_t sinceLastFlush = now - g_lastFlushMicros;
    const bool bufferFull = g_bufferPosition >= kBufferSize;
    const bool preferredBatchReady = g_bufferPosition >= kMinFlushBytes;
    const bool flushIntervalExpired = sinceLastFlush >= kFlushIntervalMicros;
    const bool hardFlushExpired = sinceLastFlush >= kHardFlushIntervalMicros;
    if (g_bufferPosition > 0 &&
        (bufferFull || g_highPriorityFlushPending || hardFlushExpired ||
         (flushIntervalExpired && preferredBatchReady))) {
        if (!FlushBuffer(false)) {
            LOG_PRINTLN("Failed to flush sensor log buffer.");
            return;
        }
    }

    if (g_syncPending && (now - g_lastSyncMicros) >= kSyncIntervalMicros) {
        if (!SyncFile()) {
            LOG_PRINTLN("Failed to sync sensor log.");
        }
    }
}

/// Returns true while the logger can accept records.
bool DataLoggerIsInitialized() {
    return g_loggerInitialized;
}

/// Returns current logger diagnostics for timing/backpressure telemetry.
DataLoggerDiagnostics DataLoggerGetDiagnostics() {
    g_diagnostics.initialized = g_loggerInitialized;
    g_diagnostics.syncPending = g_syncPending;
    g_diagnostics.bufferedBytes = g_bufferPosition;
    return g_diagnostics;
}

/// Reads a text file line-by-line from the mounted SD card.
bool DataLoggerReadTextFile(const char *path, bool (*lineCallback)(const char *line, void *context), void *context) {
    if (!g_loggerInitialized || path == nullptr || lineCallback == nullptr) {
        return false;
    }

    FsFile file = g_sd.open(path, O_RDONLY);
    if (!file) {
        return false;
    }

    char line[128];
    size_t length = 0;
    while (true) {
        const int value = file.read();
        if (value < 0) {
            break;
        }
        const char ch = static_cast<char>(value);
        if (ch == '\r') {
            continue;
        }
        if (ch == '\n') {
            if (length > 0) {
                line[length] = '\0';
                lineCallback(line, context);
                length = 0;
            }
            continue;
        }
        if (length + 1 < sizeof(line)) {
            line[length++] = ch;
        }
    }

    if (length > 0) {
        line[length] = '\0';
        lineCallback(line, context);
    }

    file.close();
    return true;
}

/// Replaces a text file on the mounted SD card.
bool DataLoggerWriteTextFile(const char *path, const char *contents) {
    if (!g_loggerInitialized || path == nullptr || contents == nullptr) {
        return false;
    }

    FsFile file = g_sd.open(path, O_WRONLY | O_CREAT | O_TRUNC);
    if (!file) {
        return false;
    }

    const size_t length = strlen(contents);
    const size_t bytesWritten = file.write(contents, length);
    if (bytesWritten != length) {
        file.close();
        return false;
    }
    if (!file.sync()) {
        file.close();
        return false;
    }
    file.close();
    return true;
}

/// Opens a text file for sequential reads, primarily for replay and tooling.
bool DataLoggerOpenReadFile(const char *path, FsFile &file) {
    if (!g_loggerInitialized || path == nullptr) {
        return false;
    }
    file = g_sd.open(path, O_RDONLY);
    return static_cast<bool>(file);
}

/// Reads the next non-empty text line from an open file handle.
bool DataLoggerReadLine(FsFile &file, char *line, size_t lineSize) {
    if (!file || line == nullptr || lineSize == 0) {
        return false;
    }

    size_t length = 0;
    bool sawData = false;
    while (true) {
        const int value = file.read();
        if (value < 0) {
            break;
        }
        const char ch = static_cast<char>(value);
        if (ch == '\r') {
            continue;
        }
        if (ch == '\n') {
            if (sawData) {
                break;
            }
            continue;
        }
        sawData = true;
        if (length + 1 < lineSize) {
            line[length++] = ch;
        }
    }

    if (!sawData) {
        return false;
    }

    line[length] = '\0';
    return true;
}
