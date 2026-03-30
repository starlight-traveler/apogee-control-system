#include <Arduino.h>
#include <algorithm>
#include <cmath>
#include <ctype.h>
#include <limits>
#include <stdlib.h>
#include <string.h>

#include "bno085_sensor.h"
#include "bmp585_sensor.h"
#include "cfd_table.h"
#include "constants.h"
#include "data_logger.h"
#include "flight_computer.h"
#include "icm20948_sensor.h"
#include "lsm9ds1_sensor.h"
#include "math_utils.h"
#include "ms5611_sensor.h"
#include "network_telemetry.h"
#include "predictor_seed.h"
#include "runtime_settings.h"
#include "serial_logging.h"
#include "settings.h"
#include "status_leds.h"
#include "synced_flap_actuation.h"

namespace {

/// Flight-loop constants mirrored locally to keep `main.cpp` readable.
constexpr uint8_t kStatusLedPin = settings::hardware::kStatusLedPin;
constexpr uint8_t kBuzzerPin = settings::hardware::kBuzzerPin;
constexpr uint32_t kErrorBlinkIntervalMs = settings::flight::kErrorBlinkIntervalMs;
constexpr uint32_t kRecoveryBlinkIntervalMs = settings::flight::kRecoveryBlinkIntervalMs;
constexpr float kServoMaxActuationDeg = settings::actuation::kServoMaxActuationDeg;
constexpr float kActuationSweepStepDeg = settings::actuation::kControlSweepStepDeg;
constexpr bool kEnableCsvReplay = settings::replay::kEnableCsvReplay;
constexpr const char *kCsvReplayPath = settings::replay::kCsvReplayPath;
constexpr size_t kCsvLineBufferSize = settings::replay::kCsvLineBufferSize;
constexpr float kBarometerAgreementThresholdFeet = settings::sensors::ms5611::kAgreementThresholdFeet;
constexpr bool kBnoEnabled = settings::sensors::bno::kEnabled;
constexpr bool kLsmEnabled = settings::sensors::lsm9ds1::kEnabled;
constexpr uint32_t kRecoveryRetryInitialMs = settings::flight::kRecoveryRetryInitialMs;
constexpr uint32_t kRecoveryRetryStepMs = settings::flight::kRecoveryRetryStepMs;
constexpr uint32_t kRecoveryRetryMaxMs = settings::flight::kRecoveryRetryMaxMs;
constexpr uint32_t kTimingLogIntervalMs = settings::flight::kTimingLogIntervalMs;
constexpr uint8_t kCfdStartupRetryCount = 5;
constexpr uint32_t kCfdStartupRetryDelayMs = 200;
constexpr float kCrossCheckAccelFullTrustMps2 = settings::sensors::icm20948::crosscheck::kAccelDiffFullTrustMps2;
constexpr float kCrossCheckAccelZeroTrustMps2 = settings::sensors::icm20948::crosscheck::kAccelDiffZeroTrustMps2;
constexpr float kCrossCheckGyroFullTrustRadPerSec = settings::sensors::icm20948::crosscheck::kGyroDiffFullTrustRadPerSec;
constexpr float kCrossCheckGyroZeroTrustRadPerSec = settings::sensors::icm20948::crosscheck::kGyroDiffZeroTrustRadPerSec;
constexpr float kCrossCheckQuaternionFullTrustDeg = settings::sensors::icm20948::crosscheck::kQuaternionDiffFullTrustDeg;
constexpr float kCrossCheckQuaternionZeroTrustDeg = settings::sensors::icm20948::crosscheck::kQuaternionDiffZeroTrustDeg;
constexpr float kCrossCheckTrustBlend = settings::sensors::icm20948::crosscheck::kTrustBlend;
constexpr float kCrossCheckTrustRecoveryPerLoop = settings::sensors::icm20948::crosscheck::kTrustRecoveryPerLoop;
constexpr uint32_t kCrossCheckFastSampleMaxAgeUs = settings::sensors::icm20948::crosscheck::kFastSampleMaxAgeUs;
constexpr uint32_t kCrossCheckFastPairMaxSkewUs = settings::sensors::icm20948::crosscheck::kFastPairMaxSkewUs;
constexpr uint32_t kCrossCheckBnoSampleMaxAgeUs = settings::sensors::icm20948::crosscheck::kBnoSampleMaxAgeUs;
constexpr uint32_t kCrossCheckBnoPairMaxSkewUs = settings::sensors::icm20948::crosscheck::kBnoPairMaxSkewUs;
constexpr float kHealthyRailTrust = settings::ahrs::kHealthyRailTrust;
constexpr float kDegradedRailTrust = settings::ahrs::kDegradedRailTrust;
constexpr float kBnoReferenceCorrectionBlendFactor = settings::ahrs::kBnoReferenceCorrectionBlendFactor;

enum class SensorRailHealth : uint8_t {
    Unavailable = 0,
    Initializing = 1,
    Stale = 2,
    Degraded = 3,
    Healthy = 4,
};

enum class SystemError : uint8_t {
    BnoInitialization = 0,
    IcmInitialization = 1,
    BmpInitialization = 2,
    Ms5611Initialization = 3,
    DataLoggerInitialization = 4,
};

/// CSV replay cursor used to feed recorded telemetry back through the firmware.
///
/// Replay intentionally reuses `SensorData` parsing so the estimator and
/// controller exercise the same code paths as live flight data.
struct CsvReplayState {
    bool enabled = false;
    bool completed = false;
    FsFile file;
    bool headerParsed = false;
    int idxTimestamp = -1;
    int idxAltitudeFeet = -1;
    int idxAccelBno[3] = {-1, -1, -1};
    int idxAccelIcm[3] = {-1, -1, -1};
    int idxQuat[4] = {-1, -1, -1, -1};
    int idxGyro[3] = {-1, -1, -1};
    int idxIcmQuat[4] = {-1, -1, -1, -1};
    int idxIcmYpr[3] = {-1, -1, -1};
    int idxHasQuaternion = -1;
    int idxHasIcmQuaternion = -1;
    int idxHasIcmYpr = -1;
    float lastTimestamp = 0.0f;
    bool hasLastTimestamp = false;
    uint32_t lastSampleMicros = 0;
};

CsvReplayState g_csvReplay;

/// Maps each setup error to a unique LED blink count for field debugging.
uint8_t BlinkCountForError(SystemError error) {
    switch (error) {
        case SystemError::BnoInitialization:
            return 2;
        case SystemError::IcmInitialization:
            return 3;
        case SystemError::BmpInitialization:
            return 4;
        case SystemError::Ms5611Initialization:
            return 5;
        case SystemError::DataLoggerInitialization:
            return 6;
    }
    return 1;
}

/// Emits a timestamped setup checkpoint so boot stalls are easy to localize.
void LogSetupCheckpoint(const char *message) {
    LOG_PRINT("[setup ");
    LOG_PRINT(millis());
    LOG_PRINT(" ms] ");
    LOG_PRINTLN(message);
}

bool StartBnoDuringSetup() {
    if (!kBnoEnabled) {
        return false;
    }
    return Bno085SensorBegin();
}

bool LoadCfdTableDuringSetup(CfdTableStorage *storage) {
    static const char *kCfdPaths[] = {"cfd.csv", "lib/cfd.csv"};

    for (uint8_t attempt = 0; attempt < kCfdStartupRetryCount; ++attempt) {
        for (const char *path : kCfdPaths) {
            if (CfdTableLoadFromSd(path, storage)) {
                return true;
            }
        }
        if (attempt + 1 < kCfdStartupRetryCount) {
            LogSetupCheckpoint("CFD table retry pending");
            delay(kCfdStartupRetryDelayMs);
        }
    }
    return false;
}

/// Plays the current boot melody.
///
/// This is intentionally blocking and should be disabled for flight builds if
/// startup latency matters more than audible status.
void PlayStartupMarch() {
    struct Note {
        uint16_t frequencyHz;
        uint16_t durationMs;
    };

    static const Note kMelody[] = {
        {440, 500}, {440, 500}, {440, 500}, {349, 350}, {523, 150},
        {440, 500}, {349, 350}, {523, 150}, {440, 650},
        {659, 500}, {659, 500}, {659, 500}, {698, 350}, {523, 150},
        {415, 500}, {349, 350}, {523, 150}, {440, 650},
    };

    pinMode(kBuzzerPin, OUTPUT);
    for (const Note &note : kMelody) {
        tone(kBuzzerPin, note.frequencyHz, note.durationMs);
        delay(note.durationMs + 30);
    }
    noTone(kBuzzerPin);
}

/// Simple local clamp to avoid pulling in additional helpers from the hot path.
float ClampFloat(float value, float minValue, float maxValue) {
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

}  // namespace

/// Splits a mutable CSV line in place and returns field count.
static int SplitCsvLine(char *line, char **fields, int maxFields) {
    int count = 0;
    char *ptr = line;
    while (ptr && *ptr != '\0' && count < maxFields) {
        fields[count++] = ptr;
        char *comma = strchr(ptr, ',');
        if (!comma) {
            break;
        }
        *comma = '\0';
        ptr = comma + 1;
    }
    return count;
}

/// Parses one CSV field as float.
static bool ParseFloatField(const char *text, float &out) {
    if (text == nullptr || *text == '\0') {
        return false;
    }
    char *end = nullptr;
    const float value = strtof(text, &end);
    if (end == text) {
        return false;
    }
    out = value;
    return true;
}

/// Parses one CSV field as a permissive boolean.
static bool ParseBoolField(const char *text, bool &out) {
    if (text == nullptr || *text == '\0') {
        return false;
    }
    char lowered[8] = {0};
    size_t len = strlen(text);
    if (len >= sizeof(lowered)) {
        len = sizeof(lowered) - 1;
    }
    for (size_t i = 0; i < len; ++i) {
        lowered[i] = static_cast<char>(tolower(static_cast<unsigned char>(text[i])));
    }
    if (strcmp(lowered, "true") == 0 || strcmp(lowered, "1") == 0) {
        out = true;
        return true;
    }
    if (strcmp(lowered, "false") == 0 || strcmp(lowered, "0") == 0) {
        out = false;
        return true;
    }
    return false;
}

/// Records a CSV column index the first time a matching header name is seen.
static void CsvAssignIndexIfMatch(const char *field, const char *name, int &target, int index) {
    if (target < 0 && strcmp(field, name) == 0) {
        target = index;
    }
}

/// Opens the replay CSV and caches the column indices the firmware cares about.
static bool CsvReplayInit() {
    if (!kEnableCsvReplay) {
        return false;
    }
    if (!DataLoggerOpenReadFile(kCsvReplayPath, g_csvReplay.file)) {
        return false;
    }

    char line[kCsvLineBufferSize];
    if (!DataLoggerReadLine(g_csvReplay.file, line, sizeof(line))) {
        return false;
    }

    char *fields[128];
    const int count = SplitCsvLine(line, fields, 128);
    for (int i = 0; i < count; ++i) {
        CsvAssignIndexIfMatch(fields[i], "sensor_timestamp", g_csvReplay.idxTimestamp, i);
        CsvAssignIndexIfMatch(fields[i], "sensor_altitude_feet", g_csvReplay.idxAltitudeFeet, i);
        CsvAssignIndexIfMatch(fields[i], "sensor_accel_bno_x", g_csvReplay.idxAccelBno[0], i);
        CsvAssignIndexIfMatch(fields[i], "sensor_accel_bno_y", g_csvReplay.idxAccelBno[1], i);
        CsvAssignIndexIfMatch(fields[i], "sensor_accel_bno_z", g_csvReplay.idxAccelBno[2], i);
        CsvAssignIndexIfMatch(fields[i], "sensor_accel_icm_x", g_csvReplay.idxAccelIcm[0], i);
        CsvAssignIndexIfMatch(fields[i], "sensor_accel_icm_y", g_csvReplay.idxAccelIcm[1], i);
        CsvAssignIndexIfMatch(fields[i], "sensor_accel_icm_z", g_csvReplay.idxAccelIcm[2], i);
        CsvAssignIndexIfMatch(fields[i], "sensor_quat_w", g_csvReplay.idxQuat[0], i);
        CsvAssignIndexIfMatch(fields[i], "sensor_quat_x", g_csvReplay.idxQuat[1], i);
        CsvAssignIndexIfMatch(fields[i], "sensor_quat_y", g_csvReplay.idxQuat[2], i);
        CsvAssignIndexIfMatch(fields[i], "sensor_quat_z", g_csvReplay.idxQuat[3], i);
        CsvAssignIndexIfMatch(fields[i], "sensor_gyro_x", g_csvReplay.idxGyro[0], i);
        CsvAssignIndexIfMatch(fields[i], "sensor_gyro_y", g_csvReplay.idxGyro[1], i);
        CsvAssignIndexIfMatch(fields[i], "sensor_gyro_z", g_csvReplay.idxGyro[2], i);
        CsvAssignIndexIfMatch(fields[i], "sensor_icm_quat_w", g_csvReplay.idxIcmQuat[0], i);
        CsvAssignIndexIfMatch(fields[i], "sensor_icm_quat_x", g_csvReplay.idxIcmQuat[1], i);
        CsvAssignIndexIfMatch(fields[i], "sensor_icm_quat_y", g_csvReplay.idxIcmQuat[2], i);
        CsvAssignIndexIfMatch(fields[i], "sensor_icm_quat_z", g_csvReplay.idxIcmQuat[3], i);
        CsvAssignIndexIfMatch(fields[i], "sensor_icm_yaw_deg", g_csvReplay.idxIcmYpr[0], i);
        CsvAssignIndexIfMatch(fields[i], "sensor_icm_pitch_deg", g_csvReplay.idxIcmYpr[1], i);
        CsvAssignIndexIfMatch(fields[i], "sensor_icm_roll_deg", g_csvReplay.idxIcmYpr[2], i);
        CsvAssignIndexIfMatch(fields[i], "sensor_has_quaternion", g_csvReplay.idxHasQuaternion, i);
        CsvAssignIndexIfMatch(fields[i], "sensor_has_icm_quaternion", g_csvReplay.idxHasIcmQuaternion, i);
        CsvAssignIndexIfMatch(fields[i], "sensor_has_icm_ypr", g_csvReplay.idxHasIcmYpr, i);
    }

    if (g_csvReplay.idxTimestamp < 0 || g_csvReplay.idxAltitudeFeet < 0) {
        return false;
    }

    g_csvReplay.enabled = true;
    g_csvReplay.headerParsed = true;
    g_csvReplay.hasLastTimestamp = false;
    LOG_PRINT("CSV replay enabled: ");
    LOG_PRINTLN(kCsvReplayPath);
    return true;
}

/// Replays logged timing by delaying until the next recorded sample timestamp.
static void CsvReplayWaitForTimestamp(float timestamp) {
    if (!g_csvReplay.hasLastTimestamp) {
        g_csvReplay.lastTimestamp = timestamp;
        g_csvReplay.lastSampleMicros = micros();
        g_csvReplay.hasLastTimestamp = true;
        return;
    }

    float dtSeconds = timestamp - g_csvReplay.lastTimestamp;
    if (dtSeconds < 0.0f) {
        dtSeconds = 0.0f;
    }

    const uint32_t targetMicros = static_cast<uint32_t>(dtSeconds * 1.0e6f);
    const uint32_t nowMicros = micros();
    const uint32_t elapsedMicros = nowMicros - g_csvReplay.lastSampleMicros;

    if (elapsedMicros < targetMicros) {
        uint32_t remaining = targetMicros - elapsedMicros;
        if (remaining >= 1000) {
            delay(remaining / 1000);
            remaining %= 1000;
        }
        if (remaining > 0) {
            delayMicroseconds(remaining);
        }
    }

    g_csvReplay.lastTimestamp = timestamp;
    g_csvReplay.lastSampleMicros = micros();
}

/// Decodes the next replay CSV row into `SensorData`.
static bool CsvReplayNextSample(SensorData &data) {
    if (!g_csvReplay.enabled || g_csvReplay.completed) {
        return false;
    }

    char line[kCsvLineBufferSize];
    while (DataLoggerReadLine(g_csvReplay.file, line, sizeof(line))) {
        char *fields[128];
        const int count = SplitCsvLine(line, fields, 128);
        if (count <= g_csvReplay.idxTimestamp || count <= g_csvReplay.idxAltitudeFeet) {
            continue;
        }

        float timestamp = 0.0f;
        float altitudeFeet = 0.0f;
        if (!ParseFloatField(fields[g_csvReplay.idxTimestamp], timestamp)) {
            continue;
        }
        if (!ParseFloatField(fields[g_csvReplay.idxAltitudeFeet], altitudeFeet)) {
            continue;
        }

        CsvReplayWaitForTimestamp(timestamp);

        data = SensorData{};
        data.timestamp = timestamp;
        data.altitudeFeet = altitudeFeet;

        for (int i = 0; i < 3; ++i) {
            if (g_csvReplay.idxAccelBno[i] >= 0 && g_csvReplay.idxAccelBno[i] < count) {
                ParseFloatField(fields[g_csvReplay.idxAccelBno[i]], data.accelBNO[i]);
            }
            if (g_csvReplay.idxAccelIcm[i] >= 0 && g_csvReplay.idxAccelIcm[i] < count) {
                ParseFloatField(fields[g_csvReplay.idxAccelIcm[i]], data.accelICM[i]);
            }
            if (g_csvReplay.idxGyro[i] >= 0 && g_csvReplay.idxGyro[i] < count) {
                ParseFloatField(fields[g_csvReplay.idxGyro[i]], data.gyro[i]);
            }
        }

        bool hasQuatValues = true;
        for (int i = 0; i < 4; ++i) {
            if (g_csvReplay.idxQuat[i] >= 0 && g_csvReplay.idxQuat[i] < count) {
                if (!ParseFloatField(fields[g_csvReplay.idxQuat[i]], data.quaternion[i])) {
                    hasQuatValues = false;
                }
            } else {
                hasQuatValues = false;
            }
        }

        if (g_csvReplay.idxHasQuaternion >= 0 && g_csvReplay.idxHasQuaternion < count) {
            ParseBoolField(fields[g_csvReplay.idxHasQuaternion], data.hasQuaternion);
        } else {
            data.hasQuaternion = hasQuatValues;
        }

        bool hasIcmQuatValues = true;
        for (int i = 0; i < 4; ++i) {
            if (g_csvReplay.idxIcmQuat[i] >= 0 && g_csvReplay.idxIcmQuat[i] < count) {
                if (!ParseFloatField(fields[g_csvReplay.idxIcmQuat[i]], data.icmQuaternion[i])) {
                    hasIcmQuatValues = false;
                }
            } else {
                hasIcmQuatValues = false;
            }
        }

        bool hasIcmYprValues = true;
        for (int i = 0; i < 3; ++i) {
            if (g_csvReplay.idxIcmYpr[i] >= 0 && g_csvReplay.idxIcmYpr[i] < count) {
                if (!ParseFloatField(fields[g_csvReplay.idxIcmYpr[i]], data.icmYprDeg[i])) {
                    hasIcmYprValues = false;
                }
            } else {
                hasIcmYprValues = false;
            }
        }

        if (g_csvReplay.idxHasIcmQuaternion >= 0 && g_csvReplay.idxHasIcmQuaternion < count) {
            ParseBoolField(fields[g_csvReplay.idxHasIcmQuaternion], data.hasIcmQuaternion);
        } else {
            data.hasIcmQuaternion = hasIcmQuatValues;
        }

        if (g_csvReplay.idxHasIcmYpr >= 0 && g_csvReplay.idxHasIcmYpr < count) {
            ParseBoolField(fields[g_csvReplay.idxHasIcmYpr], data.hasIcmYpr);
        } else {
            data.hasIcmYpr = hasIcmYprValues;
        }

        return true;
    }

    g_csvReplay.completed = true;
    LOG_PRINTLN("CSV replay complete.");
    return false;
}

struct SensorAcquireStats {
    uint32_t loops = 0;
    uint32_t bnoHits = 0;
    uint32_t icmHits = 0;
    uint32_t icmFreshHits = 0;
    uint32_t icmCachedHits = 0;
    uint32_t lsmHits = 0;
    uint32_t lsmFreshHits = 0;
    uint32_t lsmCachedHits = 0;
    uint32_t bmpHits = 0;
    uint32_t noDataLoops = 0;
    uint32_t bnoToIcmFallbacks = 0;
    uint32_t lsmToIcmFallbacks = 0;
};

static SensorAcquireStats g_sensorAcquireStats;

struct SensorComparisonStats {
    struct PairStats {
        uint32_t samples = 0;
        float maxAccelDiffMps2 = 0.0f;
        float maxGyroDiffRadPerSec = 0.0f;
        float maxQuaternionAngleDeg = 0.0f;
    };

    PairStats icmLsm;
    PairStats icmBno;
    PairStats lsmBno;
};

static SensorComparisonStats g_sensorComparisonStats;
static float g_icmLsmCrossCheckTrust = 1.0f;
static float g_bnoReferenceCrossCheckTrust = 1.0f;
static SensorRailHealth g_icmHealth = SensorRailHealth::Unavailable;
static SensorRailHealth g_lsmHealth = SensorRailHealth::Unavailable;
static SensorRailHealth g_bnoHealth = SensorRailHealth::Unavailable;

// Trust hysteresis state: track previous trust direction to reduce oscillation.
static float g_icmLsmPreviousTrust = 1.0f;
static bool g_icmLsmTrustWasDecreasing = false;
static float g_bnoPreviousTrust = 1.0f;
static bool g_bnoTrustWasDecreasing = false;

static float VectorDiffNorm3(const float a[3], const float b[3]) {
    const float dx = a[0] - b[0];
    const float dy = a[1] - b[1];
    const float dz = a[2] - b[2];
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

static float QuaternionAngleDifferenceDeg(const float a[4], const float b[4]) {
    float dot = fabsf(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
    dot = std::max(0.0f, std::min(1.0f, dot));
    return 2.0f * acosf(dot) * (180.0f / 3.14159265358979323846f);
}

static float DescendingTrust(float value, float fullTrustMax, float zeroTrustMin) {
    if (value <= fullTrustMax) {
        return 1.0f;
    }
    if (value >= zeroTrustMin || !(zeroTrustMin > fullTrustMax)) {
        return 0.0f;
    }
    return (zeroTrustMin - value) / (zeroTrustMin - fullTrustMax);
}

static SensorRailHealth ResolveRailHealth(SensorRailHealth previous,
                                          bool available,
                                          bool ready,
                                          bool recent,
                                          bool comparisonAvailable,
                                          float trust) {
    if (!available) {
        return SensorRailHealth::Unavailable;
    }
    if (!ready) {
        return SensorRailHealth::Initializing;
    }
    if (!recent) {
        return SensorRailHealth::Stale;
    }
    if (!comparisonAvailable) {
        return SensorRailHealth::Healthy;
    }
    if (trust >= kHealthyRailTrust) {
        return SensorRailHealth::Healthy;
    }
    if (trust <= kDegradedRailTrust) {
        return SensorRailHealth::Degraded;
    }
    if (previous == SensorRailHealth::Healthy || previous == SensorRailHealth::Degraded) {
        return previous;
    }
    return SensorRailHealth::Healthy;
}

static char HealthCode(SensorRailHealth health) {
    switch (health) {
        case SensorRailHealth::Unavailable:
            return '-';
        case SensorRailHealth::Initializing:
            return 'i';
        case SensorRailHealth::Stale:
            return 's';
        case SensorRailHealth::Degraded:
            return 'd';
        case SensorRailHealth::Healthy:
            return 'h';
    }
    return '?';
}

static bool SampleAgeWithinUs(uint32_t nowUs, uint32_t sampleUs, uint32_t maxAgeUs) {
    if (sampleUs == 0 || maxAgeUs == 0) {
        return false;
    }
    return static_cast<uint32_t>(nowUs - sampleUs) <= maxAgeUs;
}

static bool SampleSkewWithinUs(uint32_t sampleAUs, uint32_t sampleBUs, uint32_t maxSkewUs) {
    if (sampleAUs == 0 || sampleBUs == 0) {
        return false;
    }
    const uint32_t skewUs = (sampleAUs >= sampleBUs) ? (sampleAUs - sampleBUs) : (sampleBUs - sampleAUs);
    return skewUs <= maxSkewUs;
}

static bool SamplesComparable(uint32_t nowUs,
                              uint32_t sampleAUs,
                              uint32_t maxAgeAUs,
                              uint32_t sampleBUs,
                              uint32_t maxAgeBUs,
                              uint32_t maxSkewUs) {
    return SampleAgeWithinUs(nowUs, sampleAUs, maxAgeAUs) &&
           SampleAgeWithinUs(nowUs, sampleBUs, maxAgeBUs) &&
           SampleSkewWithinUs(sampleAUs, sampleBUs, maxSkewUs);
}

static float ComputeCrossCheckTrust(const float accelA[3],
                                    const float gyroA[3],
                                    const bool hasQuaternionA,
                                    const float quaternionA[4],
                                    const float accelB[3],
                                    const float gyroB[3],
                                    const bool hasQuaternionB,
                                    const float quaternionB[4]) {
    const float accelTrust = DescendingTrust(VectorDiffNorm3(accelA, accelB),
                                             kCrossCheckAccelFullTrustMps2,
                                             kCrossCheckAccelZeroTrustMps2);
    const float gyroTrust = DescendingTrust(VectorDiffNorm3(gyroA, gyroB),
                                            kCrossCheckGyroFullTrustRadPerSec,
                                            kCrossCheckGyroZeroTrustRadPerSec);
    float trust = std::min(accelTrust, gyroTrust);
    if (hasQuaternionA && hasQuaternionB) {
        const float quaternionTrust = DescendingTrust(QuaternionAngleDifferenceDeg(quaternionA, quaternionB),
                                                      kCrossCheckQuaternionFullTrustDeg,
                                                      kCrossCheckQuaternionZeroTrustDeg);
        trust = std::min(trust, quaternionTrust);
    }
    return std::max(0.0f, std::min(1.0f, trust));
}

static float ComputeIcmLsmCrossCheckTrust(const SensorData &icmData, const SensorData &lsmData) {
    return ComputeCrossCheckTrust(icmData.accelICM,
                                  icmData.gyro,
                                  icmData.hasIcmQuaternion,
                                  icmData.icmQuaternion,
                                  lsmData.accelICM,
                                  lsmData.gyro,
                                  lsmData.hasIcmQuaternion,
                                  lsmData.icmQuaternion);
}

static float ComputeBnoIcmCrossCheckTrust(const Bno085Sample &bnoData, const SensorData &icmData) {
    return ComputeCrossCheckTrust(bnoData.accel,
                                  bnoData.gyro,
                                  bnoData.hasQuaternion,
                                  bnoData.quaternion,
                                  icmData.accelICM,
                                  icmData.gyro,
                                  icmData.hasIcmQuaternion,
                                  icmData.icmQuaternion);
}

static float ComputeBnoLsmCrossCheckTrust(const Bno085Sample &bnoData, const SensorData &lsmData) {
    return ComputeCrossCheckTrust(bnoData.accel,
                                  bnoData.gyro,
                                  bnoData.hasQuaternion,
                                  bnoData.quaternion,
                                  lsmData.accelICM,
                                  lsmData.gyro,
                                  lsmData.hasIcmQuaternion,
                                  lsmData.icmQuaternion);
}

static void UpdatePairComparisonStats(SensorComparisonStats::PairStats &stats,
                                      const float accelA[3],
                                      const float gyroA[3],
                                      const bool hasQuaternionA,
                                      const float quaternionA[4],
                                      const float accelB[3],
                                      const float gyroB[3],
                                      const bool hasQuaternionB,
                                      const float quaternionB[4]) {
    ++stats.samples;
    stats.maxAccelDiffMps2 = std::max(stats.maxAccelDiffMps2, VectorDiffNorm3(accelA, accelB));
    stats.maxGyroDiffRadPerSec = std::max(stats.maxGyroDiffRadPerSec, VectorDiffNorm3(gyroA, gyroB));
    if (hasQuaternionA && hasQuaternionB) {
        stats.maxQuaternionAngleDeg =
            std::max(stats.maxQuaternionAngleDeg, QuaternionAngleDifferenceDeg(quaternionA, quaternionB));
    }
}

static void CopyIcmLikeFields(SensorData &dst, const SensorData &src) {
    for (int i = 0; i < 3; ++i) {
        dst.accelICM[i] = src.accelICM[i];
        dst.gyro[i] = src.gyro[i];
        dst.icmYprDeg[i] = src.icmYprDeg[i];
        dst.icmGyroBias[i] = src.icmGyroBias[i];
    }
    for (int i = 0; i < 4; ++i) {
        dst.icmQuaternion[i] = src.icmQuaternion[i];
    }
    dst.icmTemperatureC = src.icmTemperatureC;
    dst.icmAhrsDt = src.icmAhrsDt;
    dst.icmAccelTrust = src.icmAccelTrust;
    dst.icmMagTrust = src.icmMagTrust;
    dst.hasIcmQuaternion = src.hasIcmQuaternion;
    dst.hasIcmYpr = src.hasIcmYpr;
    dst.icmAccelSaturated = src.icmAccelSaturated;
    dst.icmGyroSaturated = src.icmGyroSaturated;
    dst.icmRailConstrained = src.icmRailConstrained;
}

static void CopyBnoFields(SensorData &dst, const Bno085Sample &src) {
    for (int i = 0; i < 3; ++i) {
        dst.accelBNO[i] = src.accel[i];
        dst.gyroBNO[i] = src.gyro[i];
    }
    for (int i = 0; i < 4; ++i) {
        dst.quaternionBNO[i] = src.quaternion[i];
    }
    dst.hasBnoQuaternion = src.hasQuaternion;
}

static void CopyLsmFields(SensorData &dst, const SensorData &src) {
    for (int i = 0; i < 3; ++i) {
        dst.accelLSM[i] = src.accelICM[i];
        dst.gyroLSM[i] = src.gyro[i];
        dst.lsmYprDeg[i] = src.icmYprDeg[i];
    }
    for (int i = 0; i < 4; ++i) {
        dst.quaternionLSM[i] = src.icmQuaternion[i];
    }
    dst.hasLsmQuaternion = src.hasIcmQuaternion;
    dst.hasLsmYpr = src.hasIcmYpr;
}

static void SetMainQuaternion(SensorData &data, const float quaternion[4], MainQuaternionSource source) {
    for (int i = 0; i < 4; ++i) {
        data.quaternion[i] = quaternion[i];
    }
    data.hasQuaternion = true;
    data.mainQuaternionSource = static_cast<uint8_t>(source);
}

/// Acquires one sensor sample from replay or live hardware.
///
/// The live path aliases the active attitude source into the main quaternion
/// field and only mirrors BNO accel into ICM slots when the ICM sample is absent.
static bool AcquireSensorData(SensorData &data) {
    if (g_csvReplay.enabled) {
        return CsvReplayNextSample(data);
    }
    Icm20948SensorSetCrossCheckTrust(g_icmLsmCrossCheckTrust);
    if (kLsmEnabled) {
        Lsm9ds1SensorSetCrossCheckTrust(g_icmLsmCrossCheckTrust);
    }
    const bool hasBnoImu = kBnoEnabled ? Bno085SensorAcquire(data) : false;
    const Bno085Sample bnoData = kBnoEnabled ? Bno085SensorGetSample() : Bno085Sample{};
    const bool hasIcmImu = Icm20948SensorAcquire(data);
    const Icm20948Diagnostics icmDiagnostics = Icm20948SensorGetDiagnostics();
    SensorData lsmData;
    const bool hasLsmImu = kLsmEnabled ? Lsm9ds1SensorAcquire(lsmData) : false;
    const Lsm9ds1Diagnostics lsmDiagnostics = kLsmEnabled ? Lsm9ds1SensorGetDiagnostics() : Lsm9ds1Diagnostics{};
    const uint32_t comparisonNowUs = micros();
    if (bnoData.hasAccel || bnoData.hasGyro || bnoData.hasQuaternion) {
        CopyBnoFields(data, bnoData);
    }
    if (hasLsmImu) {
        CopyLsmFields(data, lsmData);
    }
    ++g_sensorAcquireStats.loops;
    if (hasBnoImu) {
        ++g_sensorAcquireStats.bnoHits;
    }
    if (hasIcmImu) {
        ++g_sensorAcquireStats.icmHits;
        if (icmDiagnostics.lastAcquireFresh) {
            ++g_sensorAcquireStats.icmFreshHits;
        } else if (icmDiagnostics.lastAcquireUsedCache) {
            ++g_sensorAcquireStats.icmCachedHits;
        }
    }
    if (hasLsmImu) {
        ++g_sensorAcquireStats.lsmHits;
        if (lsmDiagnostics.lastAcquireFresh) {
            ++g_sensorAcquireStats.lsmFreshHits;
        } else if (lsmDiagnostics.lastAcquireUsedCache) {
            ++g_sensorAcquireStats.lsmCachedHits;
        }
    }
    if (hasBnoImu && !hasIcmImu) {
        ++g_sensorAcquireStats.bnoToIcmFallbacks;
        for (int i = 0; i < 3; ++i) {
            data.accelICM[i] = data.accelBNO[i];
        }
        if (data.hasQuaternion && !data.hasIcmQuaternion) {
            for (int i = 0; i < 4; ++i) {
                data.icmQuaternion[i] = data.quaternion[i];
            }
            data.hasIcmQuaternion = true;
        }
    }
    bool updatedBnoReferenceTrust = false;
    float bnoReferenceTargetTrust = 1.0f;
    const bool compareIcmLsm =
        hasIcmImu && hasLsmImu &&
        SamplesComparable(comparisonNowUs,
                          icmDiagnostics.lastSampleMicros,
                          kCrossCheckFastSampleMaxAgeUs,
                          lsmDiagnostics.lastSampleMicros,
                          kCrossCheckFastSampleMaxAgeUs,
                          kCrossCheckFastPairMaxSkewUs);
    if (compareIcmLsm) {
        UpdatePairComparisonStats(g_sensorComparisonStats.icmLsm,
                                  data.accelICM,
                                  data.gyro,
                                  data.hasIcmQuaternion,
                                  data.icmQuaternion,
                                  lsmData.accelICM,
                                  lsmData.gyro,
                                  lsmData.hasIcmQuaternion,
                                  lsmData.icmQuaternion);
        const float targetTrust = ComputeIcmLsmCrossCheckTrust(data, lsmData);
        // Trust hysteresis: reduce blend rate when direction changes.
        float blendRate = kCrossCheckTrustBlend;
        if (settings::ahrs::kEnableTrustHysteresis) {
            const bool isDecreasing = targetTrust < g_icmLsmCrossCheckTrust;
            if (isDecreasing != g_icmLsmTrustWasDecreasing) {
                blendRate *= settings::ahrs::kTrustHysteresisReductionFactor;
            }
            g_icmLsmTrustWasDecreasing = isDecreasing;
        }
        g_icmLsmPreviousTrust = g_icmLsmCrossCheckTrust;
        g_icmLsmCrossCheckTrust += blendRate * (targetTrust - g_icmLsmCrossCheckTrust);
    } else {
        // Trust recovery with hysteresis.
        float recoveryRate = kCrossCheckTrustRecoveryPerLoop;
        if (settings::ahrs::kEnableTrustHysteresis && g_icmLsmTrustWasDecreasing) {
            recoveryRate *= settings::ahrs::kTrustHysteresisReductionFactor;
        }
        g_icmLsmTrustWasDecreasing = false;
        g_icmLsmCrossCheckTrust = std::min(1.0f, g_icmLsmCrossCheckTrust + recoveryRate);
    }
    const bool compareBnoIcm =
        hasBnoImu && hasIcmImu && bnoData.hasAccel && bnoData.hasGyro &&
        SamplesComparable(comparisonNowUs,
                          bnoData.sampleMicros,
                          kCrossCheckBnoSampleMaxAgeUs,
                          icmDiagnostics.lastSampleMicros,
                          kCrossCheckFastSampleMaxAgeUs,
                          kCrossCheckBnoPairMaxSkewUs);
    if (compareBnoIcm) {
        UpdatePairComparisonStats(g_sensorComparisonStats.icmBno,
                                  data.accelICM,
                                  data.gyro,
                                  data.hasIcmQuaternion,
                                  data.icmQuaternion,
                                  bnoData.accel,
                                  bnoData.gyro,
                                  bnoData.hasQuaternion,
                                  bnoData.quaternion);
        bnoReferenceTargetTrust = std::min(bnoReferenceTargetTrust, ComputeBnoIcmCrossCheckTrust(bnoData, data));
        updatedBnoReferenceTrust = true;
    }
    const bool compareBnoLsm =
        hasBnoImu && hasLsmImu && bnoData.hasAccel && bnoData.hasGyro &&
        SamplesComparable(comparisonNowUs,
                          bnoData.sampleMicros,
                          kCrossCheckBnoSampleMaxAgeUs,
                          lsmDiagnostics.lastSampleMicros,
                          kCrossCheckFastSampleMaxAgeUs,
                          kCrossCheckBnoPairMaxSkewUs);
    if (compareBnoLsm) {
        UpdatePairComparisonStats(g_sensorComparisonStats.lsmBno,
                                  lsmData.accelICM,
                                  lsmData.gyro,
                                  lsmData.hasIcmQuaternion,
                                  lsmData.icmQuaternion,
                                  bnoData.accel,
                                  bnoData.gyro,
                                  bnoData.hasQuaternion,
                                  bnoData.quaternion);
        bnoReferenceTargetTrust = std::min(bnoReferenceTargetTrust, ComputeBnoLsmCrossCheckTrust(bnoData, lsmData));
        updatedBnoReferenceTrust = true;
    }
    if (updatedBnoReferenceTrust) {
        // Trust hysteresis for BNO
        float blendRate = kCrossCheckTrustBlend;
        if (settings::ahrs::kEnableTrustHysteresis) {
            const bool isDecreasing = bnoReferenceTargetTrust < g_bnoReferenceCrossCheckTrust;
            if (isDecreasing != g_bnoTrustWasDecreasing) {
                blendRate *= settings::ahrs::kTrustHysteresisReductionFactor;
            }
            g_bnoTrustWasDecreasing = isDecreasing;
        }
        g_bnoPreviousTrust = g_bnoReferenceCrossCheckTrust;
        g_bnoReferenceCrossCheckTrust += blendRate * (bnoReferenceTargetTrust - g_bnoReferenceCrossCheckTrust);
    } else {
        // Trust recovery with hysteresis.
        float recoveryRate = kCrossCheckTrustRecoveryPerLoop;
        if (settings::ahrs::kEnableTrustHysteresis && g_bnoTrustWasDecreasing) {
            recoveryRate *= settings::ahrs::kTrustHysteresisReductionFactor;
        }
        g_bnoTrustWasDecreasing = false;
        g_bnoReferenceCrossCheckTrust =
            std::min(1.0f, g_bnoReferenceCrossCheckTrust + recoveryRate);
    }
    const bool icmRecent =
        SampleAgeWithinUs(comparisonNowUs, icmDiagnostics.lastSampleMicros, kCrossCheckFastSampleMaxAgeUs);
    const bool lsmRecent =
        SampleAgeWithinUs(comparisonNowUs, lsmDiagnostics.lastSampleMicros, kCrossCheckFastSampleMaxAgeUs);
    const bool bnoRecent =
        SampleAgeWithinUs(comparisonNowUs, bnoData.sampleMicros, kCrossCheckBnoSampleMaxAgeUs);
    g_icmHealth = ResolveRailHealth(g_icmHealth,
                                    icmDiagnostics.initialized,
                                    icmDiagnostics.alignmentReady && data.hasIcmQuaternion,
                                    icmRecent,
                                    compareIcmLsm,
                                    g_icmLsmCrossCheckTrust);
    g_lsmHealth = ResolveRailHealth(g_lsmHealth,
                                    lsmDiagnostics.initialized,
                                    lsmDiagnostics.alignmentReady && lsmData.hasIcmQuaternion,
                                    lsmRecent,
                                    compareIcmLsm,
                                    g_icmLsmCrossCheckTrust);
    g_bnoHealth = ResolveRailHealth(g_bnoHealth,
                                    kBnoEnabled && bnoData.hasAccel && bnoData.hasGyro,
                                    bnoData.hasQuaternion,
                                    bnoRecent,
                                    compareBnoIcm || compareBnoLsm,
                                    g_bnoReferenceCrossCheckTrust);
    Icm20948SensorSetCrossCheckTrust(g_icmLsmCrossCheckTrust);
    if (kLsmEnabled) {
        Lsm9ds1SensorSetCrossCheckTrust(g_icmLsmCrossCheckTrust);
    }
    if (hasLsmImu && !hasIcmImu) {
        ++g_sensorAcquireStats.lsmToIcmFallbacks;
        CopyIcmLikeFields(data, lsmData);
    }
    data.hasQuaternion = false;
    data.mainQuaternionSource = static_cast<uint8_t>(MainQuaternionSource::None);
    // Multi-IMU Quaternion Blending: When multiple sources have trust > threshold,
    // blend via weighted SLERP using cross-check trust values.
    if (settings::ahrs::kEnableQuaternionBlending) {
        const bool icmHealthy = data.hasIcmQuaternion && g_icmHealth == SensorRailHealth::Healthy;
        const bool lsmHealthy = lsmData.hasIcmQuaternion && g_lsmHealth == SensorRailHealth::Healthy;
        const bool bnoHealthy = data.hasBnoQuaternion && g_bnoHealth == SensorRailHealth::Healthy;
        const bool icmUsable = data.hasIcmQuaternion &&
                               g_icmHealth != SensorRailHealth::Unavailable &&
                               g_icmHealth != SensorRailHealth::Stale;
        const bool lsmUsable = lsmData.hasIcmQuaternion &&
                               g_lsmHealth != SensorRailHealth::Unavailable &&
                               g_lsmHealth != SensorRailHealth::Stale;

        const float *fastQuaternion = nullptr;
        MainQuaternionSource fastSource = MainQuaternionSource::None;
        if (icmHealthy) {
            fastQuaternion = data.icmQuaternion;
            fastSource = MainQuaternionSource::Icm;
        } else if (lsmHealthy) {
            fastQuaternion = lsmData.icmQuaternion;
            fastSource = MainQuaternionSource::Lsm;
        } else if (icmUsable) {
            fastQuaternion = data.icmQuaternion;
            fastSource = MainQuaternionSource::Icm;
        } else if (lsmUsable) {
            fastQuaternion = lsmData.icmQuaternion;
            fastSource = MainQuaternionSource::Lsm;
        }

        if (fastQuaternion != nullptr && bnoHealthy) {
            math_utils::Quaternion fastQuat = math_utils::Normalize(math_utils::MakeQuaternion(
                fastQuaternion[0], fastQuaternion[1], fastQuaternion[2], fastQuaternion[3]));
            math_utils::Quaternion bnoQuat = math_utils::Normalize(math_utils::MakeQuaternion(
                data.quaternionBNO[0], data.quaternionBNO[1], data.quaternionBNO[2], data.quaternionBNO[3]));
            const float correctionBlend =
                std::clamp(kBnoReferenceCorrectionBlendFactor * g_bnoReferenceCrossCheckTrust, 0.0f, 1.0f);
            const math_utils::Quaternion corrected =
                math_utils::Slerp(fastQuat, bnoQuat, correctionBlend);
            const float correctedQuat[4] = {corrected.w, corrected.x, corrected.y, corrected.z};
            SetMainQuaternion(data,
                              correctedQuat,
                              correctionBlend > 0.0f ? MainQuaternionSource::Blended : fastSource);
        } else if (fastQuaternion != nullptr) {
            SetMainQuaternion(data, fastQuaternion, fastSource);
        } else if (bnoHealthy) {
            SetMainQuaternion(data, data.quaternionBNO, MainQuaternionSource::Bno);
        } else if (icmUsable) {
            SetMainQuaternion(data, data.icmQuaternion, MainQuaternionSource::Icm);
        } else if (lsmUsable) {
            SetMainQuaternion(data, lsmData.icmQuaternion, MainQuaternionSource::Lsm);
        } else if (data.hasBnoQuaternion) {
            SetMainQuaternion(data, data.quaternionBNO, MainQuaternionSource::Bno);
        }
    } else if (!data.hasQuaternion && data.hasIcmQuaternion) {
        if (g_icmHealth != SensorRailHealth::Unavailable && g_icmHealth != SensorRailHealth::Stale) {
            SetMainQuaternion(data, data.icmQuaternion, MainQuaternionSource::Icm);
        } else if (lsmData.hasIcmQuaternion &&
                   g_lsmHealth != SensorRailHealth::Unavailable &&
                   g_lsmHealth != SensorRailHealth::Stale) {
            SetMainQuaternion(data, lsmData.icmQuaternion, MainQuaternionSource::Lsm);
        }
    }
    const bool hasAltimeter = Bmp585SensorAcquire(data);
    if (hasAltimeter) {
        ++g_sensorAcquireStats.bmpHits;
    }
    // Secondary barometer disabled for now.
    // const bool hasMs5611 = Ms5611SensorAcquire();
    const bool hasSensorData = hasBnoImu || hasIcmImu || hasLsmImu || hasAltimeter;
    if (!hasSensorData) {
        ++g_sensorAcquireStats.noDataLoops;
    }
    return hasSensorData;
}

static FlightComputer flightComputer;
static CfdTableStorage g_cfdTable;
static SyncedFlapActuator g_flapActuator;
static EnvironmentModel g_actuationEnvironment;
static ApogeePredictor g_actuationPredictor;
static FlightStatus g_lastLoggedStatus = FlightStatus::Ground;
static bool g_hasLoggedStatus = false;
static bool g_hasPadAltitude = false;
static float g_padAltitudeFeet = 0.0f;
static bool g_servoCycleTestMode = false;
static float g_servoCommandDeg = 0.0f;
static float g_servoEffectiveDeg = 0.0f;
static uint32_t g_lastNoDataLogMs = 0;
static uint32_t g_lastNoLoggerLogMs = 0;
static uint32_t g_lastBarometerLogMs = 0;
static uint32_t g_lastStateLogMs = 0;
static uint32_t g_altimeterTransientUntilMs = 0;
static bool g_actuationPredictorReady = false;
static bool g_actuationHasLastZenithSample = false;
static float g_actuationLastZenithRad = 0.0f;
static float g_actuationLastStateTime = 0.0f;
static bool g_actuationHasLastControlUpdate = false;
static uint32_t g_actuationLastControlUpdateMs = 0;
static float g_actuationLastCommandDeg = 0.0f;
static PredictorHorizontalVelocityTracker g_actuationPredictorHorizontalVelocity;
static RuntimeSettings g_runtimeSettings = RuntimeSettingsDefaults();
static RuntimeSettingsStorageStatus g_runtimeSettingsStorageStatus;
static uint32_t g_runtimeSettingsRevision = 0;
static uint32_t g_runtimeSettingsLastRequestId = 0;
static uint8_t g_runtimeSettingsLastCommandResult = telemetry::kSettingsResultNone;

struct AutoActuationTelemetry {
    float autoCommandDeg = std::numeric_limits<float>::quiet_NaN();
    float bestPredictedApogeeM = std::numeric_limits<float>::quiet_NaN();
    float bestCost = std::numeric_limits<float>::quiet_NaN();
    float timeToApogeeS = std::numeric_limits<float>::quiet_NaN();
    float predictorSeedHorizontalSpeedMps = 0.0f;
    float predictorSeedClampedZenithRad = 0.0f;
    float predictorSeedClampedAngularRateRadPerSec = 0.0f;
    float predictorSeedConfidenceFlags = 0.0f;
};

struct RetryState {
    uint32_t nextAttemptMs = 0;
    uint32_t retryDelayMs = kRecoveryRetryInitialMs;
    uint32_t attempts = 0;
    bool failureLogged = false;
};

struct TimingStats {
    uint32_t maxLoopUs = 0;
    uint32_t maxSensorAcquireUs = 0;
    uint32_t maxEstimatorUs = 0;
    uint32_t maxActuationPredictorUs = 0;
    uint32_t maxLoggerServiceUs = 0;
};

static RetryState g_dataLoggerRetry;
static RetryState g_icmRetry;
static RetryState g_bmpRetry;
static uint32_t g_lastTimingLogMs = 0;
static TimingStats g_timingStats;

/// Tracks the maximum of a rolling timing statistic for periodic diagnostics.
static void UpdateMaxTiming(uint32_t sampleUs, uint32_t &targetUs) {
    if (sampleUs > targetUs) {
        targetUs = sampleUs;
    }
}

/// Runs a non-blocking exponential-backoff retry for one subsystem initializer.
static bool ServiceRetry(uint32_t nowMs,
                         RetryState &retry,
                         bool alreadyReady,
                         bool (*initializer)(),
                         const char *name) {
    if (alreadyReady) {
        return true;
    }
    if (nowMs < retry.nextAttemptMs) {
        return false;
    }

    ++retry.attempts;
    const bool ok = initializer();
    if (ok) {
        if (retry.failureLogged) {
            LOG_PRINT("[recovery] ");
            LOG_PRINT(name);
            LOG_PRINT(" recovered after attempts=");
            LOG_PRINTLN(retry.attempts);
        }
        retry = RetryState{};
        return true;
    }

    LOG_PRINT("[recovery] ");
    LOG_PRINT(name);
    LOG_PRINT(" init failed attempt=");
    LOG_PRINTLN(retry.attempts);
    retry.failureLogged = true;
    retry.nextAttemptMs = nowMs + retry.retryDelayMs;
    retry.retryDelayMs = std::min(retry.retryDelayMs + kRecoveryRetryStepMs, kRecoveryRetryMaxMs);
    return false;
}

/// Emits periodic loop and logger timing diagnostics, then resets the maxima.
static void LogTimingDiagnostics(uint32_t nowMs) {
    if ((nowMs - g_lastTimingLogMs) < kTimingLogIntervalMs) {
        return;
    }
    g_lastTimingLogMs = nowMs;

    const DataLoggerDiagnostics logger = DataLoggerGetDiagnostics();
    const Bno085Diagnostics bno = kBnoEnabled ? Bno085SensorGetDiagnostics() : Bno085Diagnostics{};
    const Icm20948Diagnostics icm = Icm20948SensorGetDiagnostics();
    const Lsm9ds1Diagnostics lsm = kLsmEnabled ? Lsm9ds1SensorGetDiagnostics() : Lsm9ds1Diagnostics{};
    LOG_PRINT("[timing] loop_us=");
    LOG_PRINT(g_timingStats.maxLoopUs);
    LOG_PRINT(" sensor_us=");
    LOG_PRINT(g_timingStats.maxSensorAcquireUs);
    LOG_PRINT(" est_us=");
    LOG_PRINT(g_timingStats.maxEstimatorUs);
    LOG_PRINT(" act_us=");
    LOG_PRINT(g_timingStats.maxActuationPredictorUs);
    LOG_PRINT(" log_us=");
    LOG_PRINT(g_timingStats.maxLoggerServiceUs);
    LOG_PRINT(" sd_write_us=");
    LOG_PRINT(logger.maxWriteDurationUs);
    LOG_PRINT(" sd_sync_us=");
    LOG_PRINT(logger.maxSyncDurationUs);
    LOG_PRINT(" log_drop=");
    LOG_PRINT(logger.droppedTelemetryRecords);
    LOG_PRINT(" log_buf=");
    LOG_PRINT(static_cast<unsigned long>(logger.bufferedBytes));
    LOG_PRINT(" sensors=");
    LOG_PRINT(bno.transportReady ? "bno" : "-");
    LOG_PRINT('(');
    LOG_PRINT(bno.hasAccel ? 'a' : '-');
    LOG_PRINT(bno.hasGyro ? 'g' : '-');
    LOG_PRINT(bno.hasQuaternion ? 'q' : '-');
    LOG_PRINT(')');
    LOG_PRINT('/');
    LOG_PRINT(icm.initialized ? "icm" : "-");
    LOG_PRINT('(');
    LOG_PRINT(icm.alignmentReady ? 'q' : '-');
    LOG_PRINT(icm.lastAcquireFresh ? 'f' : (icm.lastAcquireUsedCache ? 'c' : '-'));
    LOG_PRINT(icm.interruptConfigured ? 'I' : '-');
    LOG_PRINT(icm.lastAcquireUsedInterrupt ? 'i' : '-');
    LOG_PRINT(')');
    LOG_PRINT('/');
    LOG_PRINT(lsm.initialized ? "lsm" : "-");
    LOG_PRINT('(');
    LOG_PRINT(lsm.alignmentReady ? 'q' : '-');
    LOG_PRINT(lsm.lastAcquireFresh ? 'f' : (lsm.lastAcquireUsedCache ? 'c' : '-'));
    LOG_PRINT(lsm.interruptConfigured ? 'I' : '-');
    LOG_PRINT(lsm.lastAcquireUsedInterrupt ? 'i' : (lsm.fifoEnabled ? 'F' : '-'));
    LOG_PRINT(lsm.hasAccel ? 'a' : '-');
    LOG_PRINT(lsm.hasGyro ? 'g' : '-');
    LOG_PRINT(')');
    LOG_PRINT('/');
    LOG_PRINT(Bmp585SensorIsInitialized() ? "bmp" : "-");
    LOG_PRINT(" acq=");
    LOG_PRINT("b:");
    LOG_PRINT(g_sensorAcquireStats.bnoHits);
    LOG_PRINT('/');
    LOG_PRINT(g_sensorAcquireStats.loops);
    LOG_PRINT(" i:");
    LOG_PRINT(g_sensorAcquireStats.icmHits);
    LOG_PRINT('/');
    LOG_PRINT(g_sensorAcquireStats.loops);
    LOG_PRINT("(f");
    LOG_PRINT(g_sensorAcquireStats.icmFreshHits);
    LOG_PRINT(" c");
    LOG_PRINT(g_sensorAcquireStats.icmCachedHits);
    LOG_PRINT(")");
    LOG_PRINT(" l:");
    LOG_PRINT(g_sensorAcquireStats.lsmHits);
    LOG_PRINT('/');
    LOG_PRINT(g_sensorAcquireStats.loops);
    LOG_PRINT("(f");
    LOG_PRINT(g_sensorAcquireStats.lsmFreshHits);
    LOG_PRINT(" c");
    LOG_PRINT(g_sensorAcquireStats.lsmCachedHits);
    LOG_PRINT(")");
    LOG_PRINT(" p:");
    LOG_PRINT(g_sensorAcquireStats.bmpHits);
    LOG_PRINT('/');
    LOG_PRINT(g_sensorAcquireStats.loops);
    LOG_PRINT(" miss:");
    LOG_PRINT(g_sensorAcquireStats.noDataLoops);
    LOG_PRINT(" fb:");
    LOG_PRINT(g_sensorAcquireStats.bnoToIcmFallbacks);
    LOG_PRINT("+");
    LOG_PRINT(g_sensorAcquireStats.lsmToIcmFallbacks);
    LOG_PRINT(" cmp:il=");
    LOG_PRINT(g_sensorComparisonStats.icmLsm.samples);
    LOG_PRINT(" ib=");
    LOG_PRINT(g_sensorComparisonStats.icmBno.samples);
    LOG_PRINT(" lb=");
    LOG_PRINT(g_sensorComparisonStats.lsmBno.samples);
    LOG_PRINT(" q=");
    LOG_PRINT(g_sensorComparisonStats.icmLsm.maxQuaternionAngleDeg, 1);
    LOG_PRINT("/");
    LOG_PRINT(g_sensorComparisonStats.icmBno.maxQuaternionAngleDeg, 1);
    LOG_PRINT("/");
    LOG_PRINT(g_sensorComparisonStats.lsmBno.maxQuaternionAngleDeg, 1);
    LOG_PRINT(" a=");
    LOG_PRINT(g_sensorComparisonStats.icmLsm.maxAccelDiffMps2, 2);
    LOG_PRINT("/");
    LOG_PRINT(g_sensorComparisonStats.icmBno.maxAccelDiffMps2, 2);
    LOG_PRINT("/");
    LOG_PRINT(g_sensorComparisonStats.lsmBno.maxAccelDiffMps2, 2);
    LOG_PRINT(" g=");
    LOG_PRINT(g_sensorComparisonStats.icmLsm.maxGyroDiffRadPerSec, 2);
    LOG_PRINT("/");
    LOG_PRINT(g_sensorComparisonStats.icmBno.maxGyroDiffRadPerSec, 2);
    LOG_PRINT("/");
    LOG_PRINT(g_sensorComparisonStats.lsmBno.maxGyroDiffRadPerSec, 2);
    LOG_PRINT(" h=");
    LOG_PRINT(HealthCode(g_icmHealth));
    LOG_PRINT("/");
    LOG_PRINT(HealthCode(g_lsmHealth));
    LOG_PRINT("/");
    LOG_PRINT(HealthCode(g_bnoHealth));
    LOG_PRINT(" t=");
    LOG_PRINT(g_icmLsmCrossCheckTrust, 2);
    LOG_PRINT("/");
    LOG_PRINT(g_bnoReferenceCrossCheckTrust, 2);
    LOG_PRINT(" logger=");
    LOG_PRINT(logger.initialized ? "ok" : "down");
    LOG_PRINTLN("");

    g_timingStats = TimingStats{};
    g_sensorAcquireStats = SensorAcquireStats{};
    g_sensorComparisonStats = SensorComparisonStats{};
}

/// Converts a telemetry packet payload into the runtime settings layout.
static RuntimeSettings RuntimeSettingsFromPayload(const telemetry::RuntimeSettingsPayloadV1 &payload) {
    RuntimeSettings settings;
    settings.environment.groundTemperatureF = static_cast<float>(payload.groundTemperatureF);
    settings.environment.windSpeedMph = static_cast<float>(payload.windSpeedMph);
    settings.environment.windDirectionDeg = static_cast<float>(payload.windDirectionDeg);
    settings.environment.launchDirectionDeg = static_cast<float>(payload.launchDirectionDeg);
    settings.environment.roughnessLengthMeters = static_cast<float>(payload.roughnessLengthMeters);
    settings.environment.gradientHeightMeters = static_cast<float>(payload.gradientHeightMeters);
    settings.environment.measurementHeightMeters = static_cast<float>(payload.measurementHeightMeters);
    settings.vehicle.centerOfPressureOffsetMeters = payload.centerOfPressureOffsetMeters;
    settings.vehicle.momentOfInertia = payload.momentOfInertiaKgM2;
    settings.vehicle.dryMass = payload.dryMassKg;
    return settings;
}

/// Rebinds the live predictor stack to the currently active runtime settings.
static void ApplyRuntimeSettingsToPredictors() {
    g_actuationEnvironment.Configure(g_runtimeSettings.environment);
    g_actuationPredictor.SetEnvironment(g_actuationEnvironment);
    g_actuationPredictor.SetVehicleParameters(g_runtimeSettings.vehicle);
    g_actuationPredictor.SetForceTable(g_cfdTable.loaded ? &g_cfdTable.table : nullptr);
    g_actuationPredictor.SetMaxIntegrationSteps(settings::actuation::kActuationPredictorMaxSteps);
    g_actuationPredictor.ResetAxialDragScale();
    g_actuationPredictorReady = g_cfdTable.loaded;
    g_actuationHasLastZenithSample = false;
    g_actuationHasLastControlUpdate = false;
    ResetPredictorHorizontalVelocityTracker(g_actuationPredictorHorizontalVelocity);
    flightComputer.ReconfigurePredictor(g_runtimeSettings.environment,
                                        g_runtimeSettings.vehicle,
                                        g_cfdTable.loaded ? &g_cfdTable.table : nullptr);
}

/// Publishes the latest runtime settings/status snapshot to telemetry subscribers.
static void PublishRuntimeSettingsSnapshot() {
    g_runtimeSettingsStorageStatus.storageAvailable = DataLoggerIsInitialized();
    NetworkTelemetrySetRuntimeSettingsSnapshot(g_runtimeSettings,
                                              g_runtimeSettingsStorageStatus,
                                              g_runtimeSettingsRevision,
                                              g_runtimeSettingsLastRequestId,
                                              g_runtimeSettingsLastCommandResult);
}

/// Loads SD-backed runtime settings when possible, otherwise keeps defaults.
static void InitializeRuntimeSettings() {
    g_runtimeSettings = RuntimeSettingsDefaults();
    g_runtimeSettingsStorageStatus = RuntimeSettingsStorageStatus{};
    if (DataLoggerIsInitialized()) {
        RuntimeSettings loadedSettings = RuntimeSettingsDefaults();
        RuntimeSettingsStorageStatus loadedStatus;
        if (RuntimeSettingsLoadOrCreate(loadedSettings, loadedStatus)) {
            g_runtimeSettings = loadedSettings;
        }
        g_runtimeSettingsStorageStatus = loadedStatus;
    }
    g_runtimeSettingsRevision = 1;
    g_runtimeSettingsLastRequestId = 0;
    g_runtimeSettingsLastCommandResult = telemetry::kSettingsResultNone;
}

/// Applies one settings command from the ground station and updates the live predictor config.
static void ServiceRuntimeSettingsCommands() {
    telemetry::SettingsCommandV1 command{};
    if (!NetworkTelemetryConsumeSettingsCommand(command)) {
        return;
    }

    g_runtimeSettingsLastRequestId = command.requestId;
    g_runtimeSettingsLastCommandResult = telemetry::kSettingsResultNone;

    if (command.operation == telemetry::kSettingsOpRequestCurrent) {
        PublishRuntimeSettingsSnapshot();
        return;
    }

    if (flightComputer.Status() != FlightStatus::Ground && !g_csvReplay.enabled) {
        g_runtimeSettingsLastCommandResult = telemetry::kSettingsResultRejected;
        PublishRuntimeSettingsSnapshot();
        return;
    }

    RuntimeSettings candidate =
        (command.operation == telemetry::kSettingsOpRestoreDefaults)
            ? RuntimeSettingsDefaults()
            : RuntimeSettingsFromPayload(command.payload);

    if (!RuntimeSettingsValidate(candidate)) {
        g_runtimeSettingsLastCommandResult = telemetry::kSettingsResultRejected;
        PublishRuntimeSettingsSnapshot();
        return;
    }

    RuntimeSettingsStorageStatus storageStatus = g_runtimeSettingsStorageStatus;
    storageStatus.createdDefaultFile = false;
    storageStatus.storageAvailable = DataLoggerIsInitialized();
    const bool persisted = RuntimeSettingsSave(candidate, storageStatus);
    if (!persisted) {
        g_runtimeSettingsStorageStatus = storageStatus;
        g_runtimeSettingsLastCommandResult = storageStatus.storageAvailable
                                                 ? telemetry::kSettingsResultPersistFailed
                                                 : telemetry::kSettingsResultStorageUnavailable;
        PublishRuntimeSettingsSnapshot();
        return;
    }

    g_runtimeSettings = candidate;
    g_runtimeSettingsStorageStatus = storageStatus;
    ++g_runtimeSettingsRevision;
    g_runtimeSettingsLastCommandResult = telemetry::kSettingsResultApplied;
    ApplyRuntimeSettingsToPredictors();
    PublishRuntimeSettingsSnapshot();
}

/// Computes the commanded flap angle for automatic apogee control.
///
/// The function builds a bounded predictor seed from the latest filtered
/// state, sweeps a continuous flap-angle range with a cheap midpoint predictor,
/// then validates the winning candidate with the full RK4 predictor.
static float ComputeAutoActuationCommandDeg(uint32_t nowMs,
                                            const FilteredState &state,
                                            FlightStatus status,
                                            float currentEffectiveAngleDeg,
                                            AutoActuationTelemetry *telemetry) {
    if (telemetry != nullptr) {
        telemetry->autoCommandDeg = std::numeric_limits<float>::quiet_NaN();
        telemetry->bestPredictedApogeeM = std::numeric_limits<float>::quiet_NaN();
        telemetry->bestCost = std::numeric_limits<float>::quiet_NaN();
        telemetry->timeToApogeeS = std::numeric_limits<float>::quiet_NaN();
        telemetry->predictorSeedHorizontalSpeedMps = 0.0f;
        telemetry->predictorSeedClampedZenithRad = 0.0f;
        telemetry->predictorSeedClampedAngularRateRadPerSec = 0.0f;
        telemetry->predictorSeedConfidenceFlags = 0.0f;
    }

    double angularRate = 0.0;
    double dtSeconds = 0.0;
    const double previousZenithRad = static_cast<double>(g_actuationLastZenithRad);
    if (g_actuationHasLastZenithSample) {
        dtSeconds = static_cast<double>(state.time) - static_cast<double>(g_actuationLastStateTime);
    }
    const bool freshSeedSample = PredictorSeedHasFreshSample(dtSeconds);
    angularRate = ComputePredictorAngularRate(static_cast<double>(state.zenith),
                                              previousZenithRad,
                                              dtSeconds);
    g_actuationHasLastZenithSample = true;
    g_actuationLastZenithRad = state.zenith;
    g_actuationLastStateTime = state.time;

    const bool canControl =
        g_actuationPredictorReady && (status == FlightStatus::Burn || status == FlightStatus::Coast) &&
        state.velocity[2] > 0.0f;
    uint32_t predictorSeedFlags = 0;
    if (status == FlightStatus::Burn || status == FlightStatus::Coast) {
        predictorSeedFlags |= kPredictorSeedFlagControlActive;
    }
    if (state.velocity[2] > 0.0f) {
        predictorSeedFlags |= kPredictorSeedFlagPositiveVerticalVelocity;
    }
    const float timeToApogeeS = std::max(0.0f, state.velocity[2] / static_cast<float>(constants::kGravity));
    if (telemetry != nullptr) {
        telemetry->timeToApogeeS = timeToApogeeS;
    }
    if (!canControl) {
        ResetPredictorHorizontalVelocityTracker(g_actuationPredictorHorizontalVelocity);
        g_actuationLastCommandDeg = 0.0f;
        if (telemetry != nullptr) {
            telemetry->autoCommandDeg = 0.0f;
        }
        return 0.0f;
    }

    if (g_actuationHasLastControlUpdate &&
        (nowMs - g_actuationLastControlUpdateMs) < settings::actuation::kControlUpdateIntervalMs) {
        return g_actuationLastCommandDeg;
    }
    g_actuationHasLastControlUpdate = true;
    g_actuationLastControlUpdateMs = nowMs;
    const double clampedZenith = SanitizePredictorZenithRadians(static_cast<double>(state.zenith));
    // A stale seed is forced back to a simpler vertical-only predictor input.
    const bool useHorizontalModel = freshSeedSample;
    if (!useHorizontalModel) {
        ResetPredictorHorizontalVelocityTracker(g_actuationPredictorHorizontalVelocity);
    }
    const double trackedHorizontalVelocity =
        useHorizontalModel
            ? UpdatePredictorHorizontalSpeed(g_actuationPredictorHorizontalVelocity,
                                             static_cast<double>(state.inertialAcceleration[0]),
                                             static_cast<double>(state.inertialAcceleration[1]),
                                             dtSeconds,
                                             status == FlightStatus::Burn || status == FlightStatus::Coast,
                                             static_cast<double>(state.velocity[2]),
                                             clampedZenith)
            : 0.0;
    const double predictorHorizontalVelocity =
        useHorizontalModel
            ? ResolvePredictorHorizontalSpeed(trackedHorizontalVelocity,
                                              static_cast<double>(state.velocity[2]),
                                              clampedZenith)
            : 0.0;
    if (useHorizontalModel) {
        predictorSeedFlags |= kPredictorSeedFlagUsingHorizontalModel;
    }
    const double clampedAngularRate = useHorizontalModel ? ClampPredictorAngularRate(angularRate) : 0.0;
    const double horizontalSpeedCap =
        PredictorHorizontalSpeedCap(static_cast<double>(state.velocity[2]), clampedZenith);
    if (std::isfinite(state.zenith) &&
        std::fabs(clampedZenith - static_cast<double>(state.zenith)) > 1.0e-9) {
        predictorSeedFlags |= kPredictorSeedFlagZenithClamped;
    }
    if (useHorizontalModel && std::fabs(clampedAngularRate - angularRate) > 1.0e-9) {
        predictorSeedFlags |= kPredictorSeedFlagAngularRateClamped;
    }
    if (useHorizontalModel && predictorHorizontalVelocity >= (horizontalSpeedCap - 1.0e-6)) {
        predictorSeedFlags |= kPredictorSeedFlagHorizontalSpeedCapped;
    }
    if (telemetry != nullptr) {
        telemetry->predictorSeedHorizontalSpeedMps = static_cast<float>(predictorHorizontalVelocity);
        telemetry->predictorSeedClampedZenithRad = static_cast<float>(clampedZenith);
        telemetry->predictorSeedClampedAngularRateRadPerSec = static_cast<float>(clampedAngularRate);
        telemetry->predictorSeedConfidenceFlags = static_cast<float>(predictorSeedFlags);
    }

    ApogeeState predictorState;
    predictorState.altitudeMeters = static_cast<double>(state.position[2]);
    predictorState.horizontalDistanceMeters = 0.0;
    predictorState.verticalVelocity = static_cast<double>(state.velocity[2]);
    predictorState.horizontalVelocity = predictorHorizontalVelocity;
    predictorState.zenith = clampedZenith;
    predictorState.angularVelocity = clampedAngularRate;

    const double targetApogeeMeters = settings::flight::kApogeeTargetMeters;
    const double deadbandMeters = static_cast<double>(settings::actuation::kApogeeErrorDeadbandMeters);
    const double undershootPenalty = static_cast<double>(settings::actuation::kUndershootPenalty);
    const double ratePenalty = static_cast<double>(settings::actuation::kRatePenalty);
    const double effortPenalty = static_cast<double>(settings::actuation::kEffortPenalty);
    const double maxAngle = static_cast<double>(kServoMaxActuationDeg);
    double maxAllowedAngle = maxAngle;

    if (status == FlightStatus::Coast) {
        const double hardDisableVz = static_cast<double>(settings::actuation::kCoastHardDisableVelocityMps);
        const double hardDisableTime = static_cast<double>(settings::actuation::kCoastHardDisableTimeToApogeeS);
        const double softDisableStart =
            static_cast<double>(settings::actuation::kCoastSoftDisableStartTimeToApogeeS);
        const double timeToApogee = std::max(0.0, static_cast<double>(state.velocity[2]) / constants::kGravity);

        if (state.velocity[2] <= hardDisableVz || timeToApogee <= hardDisableTime) {
            g_actuationLastCommandDeg = 0.0f;
            return 0.0f;
        }

        if (softDisableStart > hardDisableTime && timeToApogee < softDisableStart) {
            // Taper authority near apogee so the controller does not command a
            // large final flap motion for a tiny remaining correction.
            const double taper =
                std::clamp((timeToApogee - hardDisableTime) / (softDisableStart - hardDisableTime), 0.0, 1.0);
            maxAllowedAngle = maxAngle * taper;
        }
    }

    float bestAngleDeg = ClampFloat(g_actuationLastCommandDeg, 0.0f, static_cast<float>(maxAllowedAngle));
    double bestCost = INFINITY;
    float bestPredictedApogeeM = std::numeric_limits<float>::quiet_NaN();
    bool hasBestCandidate = false;

    const double sweepStepDeg = std::max(0.1, static_cast<double>(kActuationSweepStepDeg));
    const int sweepSteps = std::max(1, static_cast<int>(std::ceil(maxAllowedAngle / sweepStepDeg)));
    for (int step = 0; step <= sweepSteps; ++step) {
        const double candidateAngleDeg = std::min(maxAllowedAngle, static_cast<double>(step) * sweepStepDeg);
        predictorState.acsAngleDeg = candidateAngleDeg;
        // Use the cheaper midpoint predictor to rank candidates, then validate
        // the winner with RK4 below.
        const double predictedApogee = g_actuationPredictor.PredictApogeeMidpoint(predictorState);
        if (!std::isfinite(predictedApogee)) {
            continue;
        }

        const double apogeeError = predictedApogee - targetApogeeMeters;
        const double errorOutsideDeadband = std::max(0.0, std::fabs(apogeeError) - deadbandMeters);
        double errorCost = errorOutsideDeadband * errorOutsideDeadband;
        if (apogeeError < -deadbandMeters) {
            errorCost *= undershootPenalty;
        }

        const double deltaAngle = candidateAngleDeg - static_cast<double>(currentEffectiveAngleDeg);
        const double rateCost = ratePenalty * deltaAngle * deltaAngle;
        const double angleNorm = candidateAngleDeg / std::max(1.0, maxAngle);
        const double effortCost = effortPenalty * angleNorm * angleNorm * 100.0;
        const double totalCost = errorCost + rateCost + effortCost;

        if (totalCost < bestCost) {
            bestCost = totalCost;
            bestAngleDeg = static_cast<float>(candidateAngleDeg);
            bestPredictedApogeeM = static_cast<float>(predictedApogee);
            hasBestCandidate = true;
        }
    }

    if (!hasBestCandidate || !std::isfinite(bestCost)) {
        if (telemetry != nullptr) {
            telemetry->autoCommandDeg = g_actuationLastCommandDeg;
        }
        return g_actuationLastCommandDeg;
    }

    predictorState.acsAngleDeg = static_cast<double>(bestAngleDeg);
    // Re-run the winning angle with RK4 so telemetry and the final decision are
    // tied to the higher-accuracy predictor path.
    const double validatedApogee = g_actuationPredictor.PredictApogee(predictorState);
    if (std::isfinite(validatedApogee)) {
        bestPredictedApogeeM = static_cast<float>(validatedApogee);
    }

    if (std::fabs(bestAngleDeg - g_actuationLastCommandDeg) < settings::actuation::kAngleCommandDeadbandDeg) {
        bestAngleDeg = g_actuationLastCommandDeg;
    }

    g_actuationLastCommandDeg = ClampFloat(bestAngleDeg, 0.0f, kServoMaxActuationDeg);
    if (telemetry != nullptr) {
        telemetry->autoCommandDeg = g_actuationLastCommandDeg;
        telemetry->bestPredictedApogeeM = bestPredictedApogeeM;
        telemetry->bestCost = static_cast<float>(bestCost);
    }
    return g_actuationLastCommandDeg;
}

static void ServiceStatusLeds(uint32_t nowMs, bool manualOverrideActive) {
    // In replay mode missing sensors are not treated as a hardware fault.
    const bool faultActive =
        !DataLoggerIsInitialized() ||
        (!g_csvReplay.enabled && ((kBnoEnabled && !Bno085SensorIsInitialized()) || !Bmp585SensorIsInitialized()));
    StatusLedsSetFault(faultActive);
    StatusLedsSetFlightStatus(flightComputer.Status());
    StatusLedsSetComms(NetworkTelemetryConnected(), NetworkTelemetrySubscriberActive());
    StatusLedsSetManualOverride(manualOverrideActive);
    StatusLedsService(nowMs);
}

/// Emits side-by-side primary/secondary barometer timing and agreement metrics.
static void LogBarometerDiagnostics(uint32_t nowMs) {
    if ((nowMs - g_lastBarometerLogMs) < settings::flight::kDebugHeartbeatIntervalMs) {
        return;
    }
    g_lastBarometerLogMs = nowMs;

    const BarometerDiagnostics bmp = Bmp585SensorGetDiagnostics();
    const BarometerDiagnostics ms = Ms5611SensorGetDiagnostics();

    if (!bmp.initialized && !ms.initialized) {
        return;
    }

    LOG_PRINT("[baro] BMP585=");
    if (bmp.hasSample) {
        LOG_PRINT(bmp.altitudeFeet, 2);
        LOG_PRINT("ft");
    } else {
        LOG_PRINT("n/a");
    }
    LOG_PRINT(" MS5611=");
    if (ms.hasSample) {
        LOG_PRINT(ms.altitudeFeet, 2);
        LOG_PRINT("ft");
    } else {
        LOG_PRINT("n/a");
    }

    if (bmp.hasSample && ms.hasSample) {
        const float deltaFeet = ms.altitudeFeet - bmp.altitudeFeet;
        LOG_PRINT(" delta=");
        LOG_PRINT(deltaFeet, 2);
        LOG_PRINT("ft");
        LOG_PRINT(" agree=");
        LOG_PRINT(std::fabs(deltaFeet) <= kBarometerAgreementThresholdFeet ? "yes" : "NO");
    }

    LOG_PRINT(" bmpReadUs=");
    LOG_PRINT(bmp.averageReadDurationUs);
    LOG_PRINT(" msReadUs=");
    LOG_PRINT(ms.averageReadDurationUs);
    LOG_PRINT(" bmpUpdUs=");
    LOG_PRINT(bmp.averageUpdatePeriodUs);
    LOG_PRINT(" msUpdUs=");
    LOG_PRINT(ms.averageUpdatePeriodUs);
    LOG_PRINT(" faster=");

    if (bmp.hasSample && ms.hasSample && bmp.averageUpdatePeriodUs > 0 && ms.averageUpdatePeriodUs > 0) {
        if (bmp.averageUpdatePeriodUs < ms.averageUpdatePeriodUs) {
            LOG_PRINT("BMP585(update)");
        } else if (ms.averageUpdatePeriodUs < bmp.averageUpdatePeriodUs) {
            LOG_PRINT("MS5611(update)");
        } else {
            LOG_PRINT("tie(update)");
        }
    } else if (bmp.hasSample && ms.hasSample && bmp.averageReadDurationUs > 0 && ms.averageReadDurationUs > 0) {
        if (bmp.averageReadDurationUs < ms.averageReadDurationUs) {
            LOG_PRINT("BMP585(read)");
        } else if (ms.averageReadDurationUs < bmp.averageReadDurationUs) {
            LOG_PRINT("MS5611(read)");
        } else {
            LOG_PRINT("tie(read)");
        }
    } else {
        LOG_PRINT("pending");
    }

    LOG_PRINTLN("");
    Serial.flush();
}

/// Arduino setup entry point.
///
/// Setup favors degraded-mode startup over retry-forever behavior so the main
/// loop can continue running even if logging or a sensor is temporarily down.
void setup() {
    // pinMode(kStatusLedPin, OUTPUT);
    // digitalWrite(kStatusLedPin, LOW);

    LOG_BEGIN(115200);
    while (!Serial) {
        delay(10);
    }

    LogSetupCheckpoint("boot");
    LogSetupCheckpoint("attaching flap servos");
    g_flapActuator.Begin();
    LogSetupCheckpoint("flap servos initialized");

    LogSetupCheckpoint("starting data logger init");
    ServiceRetry(millis(), g_dataLoggerRetry, DataLoggerIsInitialized(), &DataLoggerBegin, "data_logger");
    LogSetupCheckpoint(DataLoggerIsInitialized() ? "data logger init complete" : "data logger unavailable");

    LogSetupCheckpoint("checking CSV replay");
    if (DataLoggerIsInitialized()) {
        CsvReplayInit();
    }
    LogSetupCheckpoint(g_csvReplay.enabled ? "CSV replay active" : "CSV replay disabled");

    if (!g_csvReplay.enabled) {
        if (kBnoEnabled) {
            LOG_PRINTLN("Configured BNO sensor: BNO085 over I2C");
            LogSetupCheckpoint("starting BNO init");
            StartBnoDuringSetup();
            LogSetupCheckpoint(Bno085SensorIsInitialized() ? "BNO init complete" : "BNO unavailable");
        } else {
            LogSetupCheckpoint("BNO disabled");
        }

        LogSetupCheckpoint("starting ICM-20948 init");
        ServiceRetry(millis(), g_icmRetry, Icm20948SensorIsInitialized(), &Icm20948SensorBegin, "icm20948");
        LogSetupCheckpoint(Icm20948SensorIsInitialized() ? "ICM-20948 init complete" : "ICM-20948 unavailable");

        if (kLsmEnabled) {
            LogSetupCheckpoint("starting LSM9DS1 init");
            Lsm9ds1SensorBegin();
            LogSetupCheckpoint(Lsm9ds1SensorIsInitialized() ? "LSM9DS1 init complete" : "LSM9DS1 unavailable");
        } else {
            LogSetupCheckpoint("LSM9DS1 disabled");
        }

        LogSetupCheckpoint("starting BMP585 init");
        ServiceRetry(millis(), g_bmpRetry, Bmp585SensorIsInitialized(), &Bmp585SensorBegin, "bmp585");
        LogSetupCheckpoint(Bmp585SensorIsInitialized() ? "BMP585 init complete" : "BMP585 unavailable");

    }

    LogSetupCheckpoint("loading CFD table");
    LoadCfdTableDuringSetup(&g_cfdTable);
    LogSetupCheckpoint(g_cfdTable.loaded ? "CFD table loaded" : "CFD table unavailable");

    LogSetupCheckpoint("loading runtime settings");
    InitializeRuntimeSettings();
    LogSetupCheckpoint(g_runtimeSettingsStorageStatus.usingDefaults ? "using default runtime settings"
                                                                   : "runtime settings loaded from SD");

    LogSetupCheckpoint("configuring flight computer");
    const EnvironmentModel::Config &environmentConfig = g_runtimeSettings.environment;
    const ApogeeVehicleParameters &vehicleParameters = g_runtimeSettings.vehicle;

    g_actuationEnvironment.Configure(environmentConfig);
    g_actuationPredictor.SetEnvironment(g_actuationEnvironment);
    g_actuationPredictor.SetVehicleParameters(vehicleParameters);
    g_actuationPredictor.SetForceTable(g_cfdTable.loaded ? &g_cfdTable.table : nullptr);
    g_actuationPredictor.SetMaxIntegrationSteps(settings::actuation::kActuationPredictorMaxSteps);
    g_actuationPredictor.ResetAxialDragScale();
    // The actuation predictor is only armed when the CFD table is available.
    g_actuationPredictorReady = g_cfdTable.loaded;

    const double sigmaAccelXY = settings::flight::kSigmaAccelXY;
    const double sigmaAccelZ = settings::flight::kSigmaAccelZ;
    const double sigmaAltimeter = settings::flight::kSigmaAltimeter;
    const double processXY = settings::flight::kProcessNoiseXY;
    const double processZ = settings::flight::kProcessNoiseZ;
    const double apogeeTargetMeters = settings::flight::kApogeeTargetMeters;

    flightComputer.Begin(sigmaAccelXY,
                         sigmaAccelZ,
                         sigmaAltimeter,
                         processXY,
                         processZ,
                         apogeeTargetMeters,
                         environmentConfig,
                         vehicleParameters,
                         g_cfdTable.loaded ? &g_cfdTable.table : nullptr);
    LogSetupCheckpoint("flight computer configured");

    LogSetupCheckpoint("starting network telemetry");
    NetworkTelemetryBegin();
    PublishRuntimeSettingsSnapshot();
    LogSetupCheckpoint("network telemetry ready");

    LogSetupCheckpoint("starting status LEDs");
    StatusLedsBegin();
    LogSetupCheckpoint("status LEDs ready");

    g_lastLoggedStatus = flightComputer.Status();
    g_hasLoggedStatus = false;
    g_servoCommandDeg = 0.0f;
    g_servoEffectiveDeg = 0.0f;
    g_lastNoDataLogMs = 0;
    g_lastNoLoggerLogMs = 0;
    g_lastBarometerLogMs = 0;
    g_lastStateLogMs = 0;
    g_altimeterTransientUntilMs = 0;
    g_actuationHasLastZenithSample = false;
    g_actuationLastZenithRad = 0.0f;
    g_actuationLastStateTime = 0.0f;
    g_actuationHasLastControlUpdate = false;
    g_actuationLastControlUpdateMs = 0;
    g_actuationLastCommandDeg = 0.0f;
    ResetPredictorHorizontalVelocityTracker(g_actuationPredictorHorizontalVelocity);
    g_lastTimingLogMs = 0;
    g_timingStats = TimingStats{};
    LogSetupCheckpoint("playing startup buzzer");
    PlayStartupMarch();
    LogSetupCheckpoint("setup complete");
}

/// Arduino loop entry point.
///
/// The loop is intentionally ordered as:
/// 1. poll control input/recovery
/// 2. acquire sensors
/// 3. update estimator
/// 4. compute actuation
/// 5. log/telemetry/service diagnostics
void loop() {
    const uint32_t loopStartUs = micros();
    const uint32_t nowMs = millis();
    NetworkTelemetryPollControl();
    ServiceRuntimeSettingsCommands();

    if (!g_csvReplay.enabled) {
        ServiceRetry(nowMs, g_dataLoggerRetry, DataLoggerIsInitialized(), &DataLoggerBegin, "data_logger");
        if (kEnableCsvReplay && DataLoggerIsInitialized() && !g_csvReplay.enabled && !g_csvReplay.completed) {
            CsvReplayInit();
        }
        if (DataLoggerIsInitialized() && !g_runtimeSettingsStorageStatus.storageAvailable) {
            g_runtimeSettingsStorageStatus.storageAvailable = true;
            PublishRuntimeSettingsSnapshot();
        }
    }

    if (!g_csvReplay.enabled) {
        ServiceRetry(nowMs, g_icmRetry, Icm20948SensorIsInitialized(), &Icm20948SensorBegin, "icm20948");
        if (kLsmEnabled && !Lsm9ds1SensorIsInitialized()) {
            Lsm9ds1SensorBegin();
        }
        ServiceRetry(nowMs, g_bmpRetry, Bmp585SensorIsInitialized(), &Bmp585SensorBegin, "bmp585");
    }

    Icm20948SensorSetFlightStatus(flightComputer.Status());
    if (kLsmEnabled) {
        Lsm9ds1SensorSetFlightStatus(flightComputer.Status());
    }

    if (g_servoCycleTestMode) {
        // Servo cycle mode intentionally bypasses the rest of the flight stack.
        ServiceStatusLeds(nowMs, false);
        delay(1000);
        return;
    }

    if (!DataLoggerIsInitialized()) {
        if ((nowMs - g_lastNoLoggerLogMs) >= settings::flight::kDebugHeartbeatIntervalMs) {
            LogSetupCheckpoint("data logger unavailable in loop");
            g_lastNoLoggerLogMs = nowMs;
        }
        ServiceStatusLeds(nowMs, false);
    }

    SensorData data;
    const uint32_t sensorAcquireStartUs = micros();
    const bool hasSensorData = AcquireSensorData(data);
    UpdateMaxTiming(micros() - sensorAcquireStartUs, g_timingStats.maxSensorAcquireUs);
    if (!hasSensorData) {
        if ((nowMs - g_lastNoDataLogMs) >= settings::flight::kDebugHeartbeatIntervalMs) {
            LogSetupCheckpoint("waiting for sensor data");
            g_lastNoDataLogMs = nowMs;
        }
        ServiceStatusLeds(nowMs, false);
        DataLoggerService();
        UpdateMaxTiming(micros() - loopStartUs, g_timingStats.maxLoopUs);
        LogTimingDiagnostics(nowMs);
        return;
    }

    if (!g_hasPadAltitude && data.altitudeFeet != 0.0f) {
        g_padAltitudeFeet = data.altitudeFeet;
        g_hasPadAltitude = true;
        LOG_PRINT("Pad altitude reference (ft): ");
        LOG_PRINTLN(g_padAltitudeFeet, 2);
    }

    float altitudeAglFeet = 0.0f;
    if (g_hasPadAltitude) {
        altitudeAglFeet = data.altitudeFeet - g_padAltitudeFeet;
        if (altitudeAglFeet < 0.0f) {
            altitudeAglFeet = 0.0f;
        }
    }

    // Barometer innovations are temporarily widened around flap motion so the
    // estimator does not overreact to local pressure disturbances.
    const bool flapTransientActive = g_flapActuator.IsSettling() || (nowMs < g_altimeterTransientUntilMs);
    data.altimeterGateSigma = flapTransientActive ? settings::actuation::kBaroInnovationGateSigmaTransient
                                                  : settings::actuation::kBaroInnovationGateSigmaNominal;
    data.altimeterSigmaScale =
        flapTransientActive ? settings::actuation::kBaroDeweightSigmaScale : 1.0f;

    FilteredState state;
    const uint32_t estimatorStartUs = micros();
    const bool hasFilteredState = flightComputer.Update(data, state);
    UpdateMaxTiming(micros() - estimatorStartUs, g_timingStats.maxEstimatorUs);
    if (hasFilteredState) {
        g_actuationPredictor.SetAxialDragScale(flightComputer.AdaptiveAxialDragScale());
    }

    float manualOverrideDeg = 0.0f;
    const bool manualOverrideActive = NetworkTelemetryManualActuationOverride(manualOverrideDeg);
    manualOverrideDeg = ClampFloat(manualOverrideDeg, 0.0f, kServoMaxActuationDeg);

    AutoActuationTelemetry autoTelemetry;
    float autoCommandDeg = 0.0f;
    const uint32_t actuationPredictorStartUs = micros();
    if (hasFilteredState) {
        autoCommandDeg =
            ComputeAutoActuationCommandDeg(nowMs, state, flightComputer.Status(), g_servoEffectiveDeg, &autoTelemetry);
    }
    UpdateMaxTiming(micros() - actuationPredictorStartUs, g_timingStats.maxActuationPredictorUs);
    float commandedActuationDeg = 0.0f;
    if (manualOverrideActive) {
        commandedActuationDeg = manualOverrideDeg;
    } else if (hasFilteredState) {
        commandedActuationDeg = autoCommandDeg;
    }
    g_flapActuator.Update(nowMs, commandedActuationDeg);
    g_servoCommandDeg = g_flapActuator.CommandAngleDeg();
    g_servoEffectiveDeg = g_flapActuator.EffectiveAngleDeg();

    data.autoCommandDeg = autoTelemetry.autoCommandDeg;
    data.optimizerBestPredictedApogeeM = autoTelemetry.bestPredictedApogeeM;
    data.optimizerBestCost = autoTelemetry.bestCost;
    data.optimizerTimeToApogeeS = autoTelemetry.timeToApogeeS;
    data.actuationIsSettling = g_flapActuator.IsSettling() ? 1.0f : 0.0f;
    data.predictorSeedHorizontalSpeedMps = autoTelemetry.predictorSeedHorizontalSpeedMps;
    data.predictorSeedClampedZenithRad = autoTelemetry.predictorSeedClampedZenithRad;
    data.predictorSeedClampedAngularRateRadPerSec = autoTelemetry.predictorSeedClampedAngularRateRadPerSec;
    data.predictorSeedConfidenceFlags = autoTelemetry.predictorSeedConfidenceFlags;

    if (!g_csvReplay.enabled) {
        DataLoggerLogTelemetry(data, flightComputer.Status(), hasFilteredState ? &state : nullptr);
    }

    if (hasFilteredState) {
        const FlightStatus status = flightComputer.Status();
        if (!g_hasLoggedStatus || status != g_lastLoggedStatus) {
            DataLoggerLogEvent(FlightEventType::StageChange,
                               status,
                               state.time,
                               state.position[2],
                               state.velocity[2],
                               state.apogeeEstimate);
            g_lastLoggedStatus = status;
            g_hasLoggedStatus = true;
        }
    }

    const float eventTimestamp = hasFilteredState ? state.time : data.timestamp;
    const float eventAltitudeMeters = hasFilteredState ? state.position[2] : 0.0f;
    const float eventVerticalVelocity = hasFilteredState ? state.velocity[2] : 0.0f;
    const float eventApogeeEstimate = hasFilteredState ? state.apogeeEstimate : 0.0f;
    if (g_flapActuator.ConsumeActuationEvent()) {
        DataLoggerLogEvent(FlightEventType::FlapActuated,
                           flightComputer.Status(),
                           eventTimestamp,
                           eventAltitudeMeters,
                           eventVerticalVelocity,
                           eventApogeeEstimate);
    }
    if (g_flapActuator.ConsumeSettlingTimerFiredEvent()) {
        DataLoggerLogEvent(FlightEventType::FlapSettlingTimerFired,
                           flightComputer.Status(),
                           eventTimestamp,
                           eventAltitudeMeters,
                           eventVerticalVelocity,
                           eventApogeeEstimate);
    }

    if (g_flapActuator.IsSettling()) {
        g_altimeterTransientUntilMs = nowMs + settings::actuation::kBaroDeweightDurationMs;
    }

    if (hasFilteredState && (nowMs - g_lastStateLogMs) >= settings::flight::kStateLogIntervalMs) {
        g_lastStateLogMs = nowMs;
        LOG_PRINT("t=");
        LOG_PRINT(state.time, 3);
        LOG_PRINT(" alt=");
        LOG_PRINT(data.altitudeFeet, 2);
        LOG_PRINT(" z=");
        LOG_PRINT(state.position[2], 2);
        LOG_PRINT(" vz=");
        LOG_PRINT(state.velocity[2], 2);
        LOG_PRINT(" az=");
        LOG_PRINT(state.acceleration[2], 2);
        LOG_PRINT(" iaz=");
        LOG_PRINT(state.inertialAcceleration[2], 2);
        LOG_PRINT(" apg=");
        LOG_PRINT(state.apogeeEstimate, 2);
        LOG_PRINT(" cmd=");
        LOG_PRINT(g_servoCommandDeg, 1);
        LOG_PRINT(" eff=");
        LOG_PRINT(g_servoEffectiveDeg, 1);
        LOG_PRINT(" mode=");
        LOG_PRINT(manualOverrideActive ? "manual" : "auto");
        LOG_PRINT(" status=");
        LOG_PRINTLN(FlightStatusToString(flightComputer.Status()));
    }

    TelemetrySnapshot telemetrySnapshot;
    telemetrySnapshot.sensor = &data;
    telemetrySnapshot.state = hasFilteredState ? &state : nullptr;
    telemetrySnapshot.status = flightComputer.Status();
    telemetrySnapshot.servoCommandDeg = g_servoCommandDeg;
    telemetrySnapshot.servoEffectiveDeg = g_servoEffectiveDeg;
    telemetrySnapshot.altitudeAglFeet = altitudeAglFeet;
    telemetrySnapshot.hasPadAltitude = g_hasPadAltitude;
    telemetrySnapshot.manualActuationOverride = manualOverrideActive;
    NetworkTelemetryService(telemetrySnapshot);
    ServiceStatusLeds(nowMs, manualOverrideActive);

    const uint32_t loggerServiceStartUs = micros();
    DataLoggerService();
    UpdateMaxTiming(micros() - loggerServiceStartUs, g_timingStats.maxLoggerServiceUs);
    UpdateMaxTiming(micros() - loopStartUs, g_timingStats.maxLoopUs);
    LogTimingDiagnostics(nowMs);
}
