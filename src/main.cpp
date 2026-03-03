#include <Arduino.h>
#include <cmath>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "bno085_sensor.h"
#include "bmp585_sensor.h"
#include "cfd_table.h"
#include "data_logger.h"
#include "flight_computer.h"
#include "icm20948_sensor.h"
#include "ms5611_sensor.h"
#include "network_telemetry.h"
#include "serial_logging.h"
#include "settings.h"
#include "status_leds.h"
#include "synced_flap_actuation.h"

namespace {

constexpr uint8_t kStatusLedPin = settings::hardware::kStatusLedPin;
constexpr uint8_t kBuzzerPin = settings::hardware::kBuzzerPin;
constexpr uint32_t kErrorBlinkIntervalMs = settings::flight::kErrorBlinkIntervalMs;
constexpr uint32_t kRecoveryBlinkIntervalMs = settings::flight::kRecoveryBlinkIntervalMs;
constexpr float kDeploymentTriggerAltitudeFeet = settings::actuation::kDeploymentTriggerAltitudeFeet;
constexpr float kServoMaxActuationDeg = settings::actuation::kServoMaxActuationDeg;
constexpr bool kEnableCsvReplay = settings::replay::kEnableCsvReplay;
constexpr const char *kCsvReplayPath = settings::replay::kCsvReplayPath;
constexpr size_t kCsvLineBufferSize = settings::replay::kCsvLineBufferSize;
constexpr uint32_t kDebugHeartbeatIntervalMs = 1000;
constexpr uint32_t kStateLogIntervalMs = 250;
constexpr uint8_t kDeploymentConfirmSamples = 30;
constexpr float kBarometerAgreementThresholdFeet = settings::sensors::ms5611::kAgreementThresholdFeet;

enum class SystemError : uint8_t {
    BnoInitialization = 0,
    IcmInitialization = 1,
    BmpInitialization = 2,
    Ms5611Initialization = 3,
    DataLoggerInitialization = 4,
};

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

void BlinkPattern(uint8_t count, uint32_t intervalMs) {
    for (uint8_t i = 0; i < count; ++i) {
        digitalWrite(kStatusLedPin, HIGH);
        delay(intervalMs);
        digitalWrite(kStatusLedPin, LOW);
        delay(intervalMs);
    }
}

void IndicateError(SystemError error) {
    BlinkPattern(BlinkCountForError(error), kErrorBlinkIntervalMs);
    delay(kErrorBlinkIntervalMs * 2);
}

void IndicateRecovery() {
    BlinkPattern(3, kRecoveryBlinkIntervalMs);
    delay(kRecoveryBlinkIntervalMs * 2);
}

void LogSetupCheckpoint(const char *message) {
    LOG_PRINT("[setup ");
    LOG_PRINT(millis());
    LOG_PRINT(" ms] ");
    LOG_PRINTLN(message);
}

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

bool InitializeWithRecovery(SystemError error, bool (*initializer)(), const char *failureMessage) {
    bool hadFailure = false;
    bool loggedFailure = false;
    uint32_t retryDelayMs = 200;
    uint32_t attempt = 1;
    while (true) {
        LOG_PRINT("[init attempt ");
        LOG_PRINT(attempt);
        LOG_PRINTLN("] starting");
        if (initializer()) {
            break;
        }

        hadFailure = true;
        if (!loggedFailure && failureMessage != nullptr) {
            LOG_PRINTLN(failureMessage);
            loggedFailure = true;
        }
        LOG_PRINT("[init attempt ");
        LOG_PRINT(attempt);
        LOG_PRINTLN("] failed");
        IndicateError(error);
        LOG_PRINT("[init retry delay ms] ");
        LOG_PRINTLN(retryDelayMs);
        delay(retryDelayMs);
        if (retryDelayMs < 2000) {
            retryDelayMs += 200;
            if (retryDelayMs > 2000) {
                retryDelayMs = 2000;
            }
        }
        ++attempt;
    }

    LOG_PRINT("[init] success after attempts=");
    LOG_PRINTLN(attempt);

    if (hadFailure) {
        IndicateRecovery();
    }

    if (loggedFailure && failureMessage != nullptr) {
        LOG_PRINTLN("Recovered successfully.");
    }

    return true;
}

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

static void CsvAssignIndexIfMatch(const char *field, const char *name, int &target, int index) {
    if (target < 0 && strcmp(field, name) == 0) {
        target = index;
    }
}

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

    char *fields[64];
    const int count = SplitCsvLine(line, fields, 64);
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

static bool CsvReplayNextSample(SensorData &data) {
    if (!g_csvReplay.enabled || g_csvReplay.completed) {
        return false;
    }

    char line[kCsvLineBufferSize];
    while (DataLoggerReadLine(g_csvReplay.file, line, sizeof(line))) {
        char *fields[64];
        const int count = SplitCsvLine(line, fields, 64);
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

static bool AcquireSensorData(SensorData &data) {
    if (g_csvReplay.enabled) {
        return CsvReplayNextSample(data);
    }
    const bool hasBnoImu = Bno085SensorAcquire(data);
    // ICM-20948 disabled for now.
    // const bool hasIcmImu = Icm20948SensorAcquire(data);
    const bool hasIcmImu = false;
    if (hasBnoImu && !hasIcmImu) {
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
    const bool hasAltimeter = Bmp585SensorAcquire(data);
    // Secondary barometer disabled for now.
    // const bool hasMs5611 = Ms5611SensorAcquire();
    return hasBnoImu || hasIcmImu || hasAltimeter;
}

static FlightComputer flightComputer;
static CfdTableStorage g_cfdTable;
static SyncedFlapActuator g_flapActuator;
static FlightStatus g_lastLoggedStatus = FlightStatus::Ground;
static bool g_hasLoggedStatus = false;
static bool g_hasPadAltitude = false;
static float g_padAltitudeFeet = 0.0f;
static bool g_servoCycleTestMode = false;
static float g_servoCommandDeg = 0.0f;
static float g_servoEffectiveDeg = 0.0f;
static bool g_wasAboveDeploymentThreshold = false;
static bool g_hasAutoDeployed = false;
static uint8_t g_aboveDeploymentThresholdCount = 0;
static uint32_t g_lastNoDataLogMs = 0;
static uint32_t g_lastNoLoggerLogMs = 0;
static uint32_t g_lastBarometerLogMs = 0;
static uint32_t g_lastStateLogMs = 0;

static void ServiceStatusLeds(uint32_t nowMs, bool manualOverrideActive) {
    StatusLedsSetFault(!DataLoggerIsInitialized());
    StatusLedsSetFlightStatus(flightComputer.Status());
    StatusLedsSetComms(NetworkTelemetryConnected(), NetworkTelemetrySubscriberActive());
    StatusLedsSetManualOverride(manualOverrideActive);
    StatusLedsService(nowMs);
}

static void LogBarometerDiagnostics(uint32_t nowMs) {
    if ((nowMs - g_lastBarometerLogMs) < kDebugHeartbeatIntervalMs) {
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

void setup() {
    // pinMode(kStatusLedPin, OUTPUT);
    // digitalWrite(kStatusLedPin, LOW);

    LOG_BEGIN(115200);

    LogSetupCheckpoint("boot");
    LogSetupCheckpoint("attaching flap servos");
    g_flapActuator.Begin();
    LogSetupCheckpoint("flap servos initialized");

    LogSetupCheckpoint("starting data logger init");
    InitializeWithRecovery(SystemError::DataLoggerInitialization,
                           &DataLoggerBegin,
                           "Sensor logging is disabled.");
    LogSetupCheckpoint("data logger init complete");

    LogSetupCheckpoint("checking CSV replay");
    CsvReplayInit();
    LogSetupCheckpoint(g_csvReplay.enabled ? "CSV replay active" : "CSV replay disabled");

    if (!g_csvReplay.enabled) {
        LogSetupCheckpoint("starting BNO085 init");
        InitializeWithRecovery(SystemError::BnoInitialization,
                               &Bno085SensorBegin,
                               "Failed to initialize BNO085 sensor.");
        LogSetupCheckpoint("BNO085 init complete");

        // ICM-20948 disabled for now.
        // LogSetupCheckpoint("starting ICM-20948 init");
        // InitializeWithRecovery(SystemError::IcmInitialization,
        //                        &Icm20948SensorBegin,
        //                        "Failed to initialize ICM-20948 sensor.");
        // LogSetupCheckpoint("ICM-20948 init complete");

        LogSetupCheckpoint("starting BMP585 init");
        InitializeWithRecovery(SystemError::BmpInitialization,
                               &Bmp585SensorBegin,
                               "Failed to initialize BMP585 sensor.");
        LogSetupCheckpoint("BMP585 init complete");

        // Secondary barometer disabled for now.
        // LogSetupCheckpoint("starting MS5611 init");
        // InitializeWithRecovery(SystemError::Ms5611Initialization,
        //                        &Ms5611SensorBegin,
        //                        "Failed to initialize MS5611 sensor.");
        // LogSetupCheckpoint("MS5611 init complete");
    }

    LogSetupCheckpoint("loading CFD table");
    if (!CfdTableLoadFromSd("cfd.csv", &g_cfdTable)) {
        LogSetupCheckpoint("cfd.csv missing, trying lib/cfd.csv");
        CfdTableLoadFromSd("lib/cfd.csv", &g_cfdTable);
    }
    LogSetupCheckpoint(g_cfdTable.loaded ? "CFD table loaded" : "CFD table unavailable");

    LogSetupCheckpoint("configuring flight computer");
    const EnvironmentModel::Config environmentConfig;
    ApogeeVehicleParameters vehicleParameters;
    vehicleParameters.centerOfPressureOffsetMeters = settings::vehicle::kCenterOfPressureOffsetMeters;
    vehicleParameters.momentOfInertia = settings::vehicle::kMomentOfInertiaKgM2;
    vehicleParameters.dryMass = settings::vehicle::kDryMassKg;

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
    LogSetupCheckpoint("network telemetry ready");

    LogSetupCheckpoint("starting status LEDs");
    StatusLedsBegin();
    LogSetupCheckpoint("status LEDs ready");

    g_lastLoggedStatus = flightComputer.Status();
    g_hasLoggedStatus = false;
    g_servoCommandDeg = 0.0f;
    g_servoEffectiveDeg = 0.0f;
    g_wasAboveDeploymentThreshold = false;
    g_hasAutoDeployed = false;
    g_aboveDeploymentThresholdCount = 0;
    g_lastNoDataLogMs = 0;
    g_lastNoLoggerLogMs = 0;
    g_lastBarometerLogMs = 0;
    g_lastStateLogMs = 0;
    LogSetupCheckpoint("playing startup buzzer");
    PlayStartupMarch();
    LogSetupCheckpoint("setup complete");
}

void loop() {
    const uint32_t nowMs = millis();
    NetworkTelemetryPollControl();

    if (g_servoCycleTestMode) {
        ServiceStatusLeds(nowMs, false);
        delay(1000);
        return;
    }

    if (!DataLoggerIsInitialized()) {
        if ((nowMs - g_lastNoLoggerLogMs) >= kDebugHeartbeatIntervalMs) {
            LogSetupCheckpoint("data logger unavailable in loop");
            g_lastNoLoggerLogMs = nowMs;
        }
        ServiceStatusLeds(nowMs, false);
        InitializeWithRecovery(SystemError::DataLoggerInitialization,
                               &DataLoggerBegin,
                               "Data logger unavailable. Retrying...");
        return;
    }

    SensorData data;
    if (!AcquireSensorData(data)) {
        if ((nowMs - g_lastNoDataLogMs) >= kDebugHeartbeatIntervalMs) {
            LogSetupCheckpoint("waiting for sensor data");
            g_lastNoDataLogMs = nowMs;
        }
        ServiceStatusLeds(nowMs, false);
        delay(1);
        return;
    }

    // Disable periodic barometer diagnostics in the flight loop for now.
    // LogBarometerDiagnostics(nowMs);

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

    FilteredState state;
    const bool hasFilteredState = flightComputer.Update(data, state);
    float manualOverrideDeg = 0.0f;
    const bool manualOverrideActive = NetworkTelemetryManualActuationOverride(manualOverrideDeg);
    manualOverrideDeg = ClampFloat(manualOverrideDeg, 0.0f, kServoMaxActuationDeg);

    if (!g_csvReplay.enabled) {
        DataLoggerLogTelemetry(data, flightComputer.Status(), hasFilteredState ? &state : nullptr);
    }

    const bool aboveDeploymentThreshold = altitudeAglFeet >= kDeploymentTriggerAltitudeFeet;
    if (aboveDeploymentThreshold) {
        if (g_aboveDeploymentThresholdCount < 255) {
            ++g_aboveDeploymentThresholdCount;
        }
    } else {
        g_aboveDeploymentThresholdCount = 0;
    }
    const bool confirmedAboveDeploymentThreshold = g_aboveDeploymentThresholdCount >= kDeploymentConfirmSamples;

    if (hasFilteredState) {
        const FlightStatus status = flightComputer.Status();
        const bool autoDeployTrigger =
            !g_hasAutoDeployed && confirmedAboveDeploymentThreshold && !g_wasAboveDeploymentThreshold;
        const bool manualForceExtend = manualOverrideActive;
        if (autoDeployTrigger) {
            g_hasAutoDeployed = true;
            LOG_PRINT("Flap deployment triggered at AGL ft: ");
            LOG_PRINTLN(altitudeAglFeet, 2);
        }

        g_flapActuator.Update(nowMs, autoDeployTrigger, manualForceExtend);
        g_servoCommandDeg = g_flapActuator.CommandFraction() * kServoMaxActuationDeg;
        g_servoEffectiveDeg = g_flapActuator.EffectiveFraction() * kServoMaxActuationDeg;

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
    } else if (manualOverrideActive) {
        const bool manualForceExtend = true;
        g_flapActuator.Update(nowMs, false, manualForceExtend);
        g_servoCommandDeg = g_flapActuator.CommandFraction() * kServoMaxActuationDeg;
        g_servoEffectiveDeg = g_flapActuator.EffectiveFraction() * kServoMaxActuationDeg;
    } else {
        g_flapActuator.Update(nowMs, false, false);
        g_servoCommandDeg = g_flapActuator.CommandFraction() * kServoMaxActuationDeg;
        g_servoEffectiveDeg = g_flapActuator.EffectiveFraction() * kServoMaxActuationDeg;
    }

    g_wasAboveDeploymentThreshold = confirmedAboveDeploymentThreshold;

    if (hasFilteredState && (nowMs - g_lastStateLogMs) >= kStateLogIntervalMs) {
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

    DataLoggerService();
}
