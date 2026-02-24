#include <Arduino.h>
#include <Servo.h>
#include <cmath>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "bno055_sensor.h"
#include "bmp581_sensor.h"
#include "cfd_table.h"
#include "data_logger.h"
#include "flight_computer.h"
#include "icm20948_sensor.h"
#include "network_telemetry.h"
#include "serial_logging.h"
#include "settings.h"
#include "status_leds.h"

namespace {

constexpr uint8_t kStatusLedPin = settings::hardware::kStatusLedPin;
constexpr uint32_t kErrorBlinkIntervalMs = settings::flight::kErrorBlinkIntervalMs;
constexpr uint32_t kRecoveryBlinkIntervalMs = settings::flight::kRecoveryBlinkIntervalMs;
constexpr uint8_t kServoPin = settings::hardware::kServoPin;
constexpr int kServoRetractAngle = settings::hardware::kServoRetractAngle;
constexpr float kServoMinExtendAltitudeFeet = settings::actuation::kServoMinExtendAltitudeFeet;
constexpr float kServoMaxActuationDeg = settings::actuation::kServoMaxActuationDeg;
constexpr float kServoLatencySeconds = settings::actuation::kServoLatencySeconds;
constexpr uint32_t kControlUpdateIntervalMs = settings::actuation::kControlUpdateIntervalMs;
constexpr float kAngleStepDeg = settings::actuation::kAngleStepDeg;
constexpr float kAngleCommandDeadbandDeg = settings::actuation::kAngleCommandDeadbandDeg;
constexpr float kApogeeErrorDeadbandMeters = settings::actuation::kApogeeErrorDeadbandMeters;
constexpr float kRatePenalty = settings::actuation::kRatePenalty;
constexpr float kEffortPenalty = settings::actuation::kEffortPenalty;
constexpr float kUndershootPenalty = settings::actuation::kUndershootPenalty;
constexpr int kActuationPredictorMaxSteps = settings::actuation::kActuationPredictorMaxSteps;
constexpr float kCoarseAngleStepDeg = settings::actuation::kCoarseAngleStepDeg;
constexpr float kCoarseAmbiguityCostThreshold = settings::actuation::kCoarseAmbiguityCostThreshold;
constexpr float kTargetApogeeMeters = static_cast<float>(settings::flight::kApogeeTargetMeters);
constexpr bool kEnableCsvReplay = settings::replay::kEnableCsvReplay;
constexpr const char *kCsvReplayPath = settings::replay::kCsvReplayPath;
constexpr size_t kCsvLineBufferSize = settings::replay::kCsvLineBufferSize;

enum class SystemError : uint8_t {
    BnoInitialization = 0,
    IcmInitialization = 1,
    BmpInitialization = 2,
    DataLoggerInitialization = 3,
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
        case SystemError::DataLoggerInitialization:
            return 5;
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

bool InitializeWithRecovery(SystemError error, bool (*initializer)(), const char *failureMessage) {
    bool hadFailure = false;
    bool loggedFailure = false;
    uint32_t retryDelayMs = 200;
    while (!initializer()) {
        hadFailure = true;
        if (!loggedFailure && failureMessage != nullptr) {
            LOG_PRINTLN(failureMessage);
            loggedFailure = true;
        }
        IndicateError(error);
        delay(retryDelayMs);
        if (retryDelayMs < 2000) {
            retryDelayMs += 200;
            if (retryDelayMs > 2000) {
                retryDelayMs = 2000;
            }
        }
    }

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

float ComputeLagBlend(float dtSeconds, float tauSeconds) {
    if (tauSeconds <= 0.0f || dtSeconds <= 0.0f) {
        return 1.0f;
    }
    const float alpha = dtSeconds / (tauSeconds + dtSeconds);
    return ClampFloat(alpha, 0.0f, 1.0f);
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
    const bool hasBnoImu = Bno055SensorAcquire(data);
    const bool hasIcmImu = Icm20948SensorAcquire(data);
    const bool hasAltimeter = Bmp581SensorAcquire(data);
    return hasBnoImu || hasIcmImu || hasAltimeter;
}

static FlightComputer flightComputer;
static CfdTableStorage g_cfdTable;
static EnvironmentModel g_actuationEnvironment;
static ApogeePredictor g_actuationPredictor;
static FlightStatus g_lastLoggedStatus = FlightStatus::Ground;
static bool g_hasLoggedStatus = false;
static bool g_hasPadAltitude = false;
static float g_padAltitudeFeet = 0.0f;
Servo g_servo;
static bool g_servoCycleTestMode = false;
static float g_servoCommandDeg = 0.0f;
static float g_servoEffectiveDeg = 0.0f;
static float g_lastControlTime = 0.0f;
static float g_lastZenith = 0.0f;
static bool g_hasLastZenith = false;
static uint32_t g_lastControlUpdateMs = 0;
static int g_lastServoWriteDeg = kServoRetractAngle;

static void ServiceStatusLeds(uint32_t nowMs, bool manualOverrideActive) {
    StatusLedsSetFault(!DataLoggerIsInitialized());
    StatusLedsSetFlightStatus(flightComputer.Status());
    StatusLedsSetComms(NetworkTelemetryConnected(), NetworkTelemetrySubscriberActive());
    StatusLedsSetManualOverride(manualOverrideActive);
    StatusLedsService(nowMs);
}

static float EstimateAngularVelocity(const FilteredState &state, float dtSeconds) {
    if (!g_hasLastZenith || dtSeconds <= 0.0f) {
        return 0.0f;
    }
    return (state.zenith - g_lastZenith) / dtSeconds;
}

static float SelectActuationCommandDeg(const FilteredState &state,
                                       float angularVelocityRadPerSec,
                                       float commandDeg,
                                       float effectiveDeg) {
    if (!g_cfdTable.loaded) {
        // Fail-safe: with no aero model available, keep ACS neutral.
        return 0.0f;
    }

    ApogeeState neutralState;
    neutralState.altitudeMeters = state.position[2];
    neutralState.horizontalDistanceMeters = math_utils::Magnitude2(state.position[0], state.position[1]);
    neutralState.verticalVelocity = state.velocity[2];
    neutralState.horizontalVelocity = math_utils::Magnitude2(state.velocity[0], state.velocity[1]);
    neutralState.zenith = state.zenith;
    neutralState.angularVelocity = angularVelocityRadPerSec;
    neutralState.acsAngleDeg = 0.0;

    const double neutralApogee = g_actuationPredictor.PredictApogee(neutralState);
    if (neutralApogee <= static_cast<double>(kTargetApogeeMeters + kApogeeErrorDeadbandMeters)) {
        return 0.0f;
    }

    const float assumedDt = static_cast<float>(kControlUpdateIntervalMs) * 1.0e-3f;
    const float lagBlend = ComputeLagBlend(assumedDt, kServoLatencySeconds);
    struct CostEvalEntry {
        float angleDeg = 0.0f;
        float cost = 0.0f;
        bool valid = false;
    };
    constexpr int kMaxCachedAngles = settings::actuation::kEvalCacheMaxEntries;
    CostEvalEntry evalCache[kMaxCachedAngles];
    int evalCacheCount = 0;

    auto evaluateCost = [&](float candidateDeg) -> float {
        for (int i = 0; i < evalCacheCount; ++i) {
            if (evalCache[i].valid &&
                std::fabs(evalCache[i].angleDeg - candidateDeg) <= settings::actuation::kEvalCacheMatchEpsilonDeg) {
                return evalCache[i].cost;
            }
        }

        const float predictedEffectiveDeg = effectiveDeg + (candidateDeg - effectiveDeg) * lagBlend;

        ApogeeState testState = neutralState;
        testState.acsAngleDeg = predictedEffectiveDeg;
        const float predictedApogee = static_cast<float>(g_actuationPredictor.PredictApogee(testState));
        float apogeeError = predictedApogee - kTargetApogeeMeters;
        float errorCost = std::fabs(apogeeError);
        if (apogeeError < 0.0f) {
            errorCost *= kUndershootPenalty;
        }

        const float rateCost = kRatePenalty * std::fabs(candidateDeg - commandDeg);
        const float normAngle = candidateDeg / kServoMaxActuationDeg;
        const float effortCost = kEffortPenalty * normAngle * normAngle;
        const float totalCost = errorCost + rateCost + effortCost;
        if (evalCacheCount < kMaxCachedAngles) {
            evalCache[evalCacheCount].angleDeg = candidateDeg;
            evalCache[evalCacheCount].cost = totalCost;
            evalCache[evalCacheCount].valid = true;
            ++evalCacheCount;
        }
        return totalCost;
    };

    float bestAngleDeg = commandDeg;
    float bestCost = evaluateCost(commandDeg);
    auto sweepRange = [&](float startDeg, float endDeg, float stepDeg, bool allowPrune) {
        int consecutiveWorse = 0;
        for (float candidate = startDeg; candidate <= endDeg + 0.001f; candidate += stepDeg) {
            const float bounded = ClampFloat(candidate, 0.0f, kServoMaxActuationDeg);
            const float totalCost = evaluateCost(bounded);
            if (totalCost < bestCost) {
                bestCost = totalCost;
                bestAngleDeg = bounded;
                consecutiveWorse = 0;
            } else if (allowPrune && settings::actuation::kEnableSweepPruning) {
                ++consecutiveWorse;
                // In full-range sweep, when cost has been worse for several consecutive bins,
                // additional angles are unlikely to beat the current minimum.
                if (consecutiveWorse >= settings::actuation::kSweepPruneConsecutiveWorse) {
                    break;
                }
            }
        }
    };

    const float coarseStep = (kCoarseAngleStepDeg >= kAngleStepDeg) ? kCoarseAngleStepDeg : kAngleStepDeg;
    float coarseBestAngle = 0.0f;
    float coarseBestCost = 1.0e30f;
    float coarseSecondBestCost = 1.0e30f;
    for (float candidate = 0.0f; candidate <= kServoMaxActuationDeg + 0.001f; candidate += coarseStep) {
        const float totalCost = evaluateCost(candidate);
        if (totalCost < coarseBestCost) {
            coarseSecondBestCost = coarseBestCost;
            coarseBestCost = totalCost;
            coarseBestAngle = candidate;
        } else if (totalCost < coarseSecondBestCost) {
            coarseSecondBestCost = totalCost;
        }
    }

    const bool ambiguousCoarse = (coarseSecondBestCost - coarseBestCost) <= kCoarseAmbiguityCostThreshold;
    if (ambiguousCoarse) {
        sweepRange(0.0f, kServoMaxActuationDeg, kAngleStepDeg, true);
    } else {
        const float refineHalfWindow = coarseStep;
        const float refineStart = coarseBestAngle - refineHalfWindow;
        const float refineEnd = coarseBestAngle + refineHalfWindow;
        sweepRange(refineStart, refineEnd, kAngleStepDeg, false);
    }

    if (std::fabs(bestAngleDeg - commandDeg) <= kAngleCommandDeadbandDeg) {
        return commandDeg;
    }
    return ClampFloat(bestAngleDeg, 0.0f, kServoMaxActuationDeg);
}

static void WriteServoAngleDeg(float angleDeg) {
    const int writeDeg = static_cast<int>(std::lround(ClampFloat(angleDeg, 0.0f, kServoMaxActuationDeg)));
    if (writeDeg == g_lastServoWriteDeg) {
        return;
    }
    g_servo.write(writeDeg);
    g_lastServoWriteDeg = writeDeg;
}

void setup() {
    pinMode(kStatusLedPin, OUTPUT);
    digitalWrite(kStatusLedPin, LOW);

    g_servo.attach(kServoPin);
    g_servo.write(kServoRetractAngle);

    LOG_BEGIN(115200);
#if ENABLE_SERIAL_LOGGING
    while (!Serial && millis() < 2000) {
    }
#endif

    InitializeWithRecovery(SystemError::DataLoggerInitialization,
                           &DataLoggerBegin,
                           "Sensor logging is disabled.");

    CsvReplayInit();


    if (!g_csvReplay.enabled) {
        InitializeWithRecovery(SystemError::BnoInitialization,
                               &Bno055SensorBegin,
                               "Failed to initialize BNO055 sensor.");

        InitializeWithRecovery(SystemError::IcmInitialization,
                               &Icm20948SensorBegin,
                               "Failed to initialize ICM-20948 sensor.");

        InitializeWithRecovery(SystemError::BmpInitialization,
                               &Bmp581SensorBegin,
                               "Failed to initialize BMP581 sensor.");
    }

    if (!CfdTableLoadFromSd("cfd.csv", &g_cfdTable)) {
        CfdTableLoadFromSd("lib/cfd.csv", &g_cfdTable);
    }

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

    g_actuationEnvironment.Configure(environmentConfig);
    g_actuationPredictor.SetEnvironment(g_actuationEnvironment);
    g_actuationPredictor.SetVehicleParameters(vehicleParameters);
    g_actuationPredictor.SetForceTable(g_cfdTable.loaded ? &g_cfdTable.table : nullptr);
    g_actuationPredictor.SetMaxIntegrationSteps(kActuationPredictorMaxSteps);

    LOG_PRINTLN("Flight computer initialized.");

    NetworkTelemetryBegin();
    StatusLedsBegin();

    g_lastLoggedStatus = flightComputer.Status();
    g_hasLoggedStatus = false;
    g_servoCommandDeg = 0.0f;
    g_servoEffectiveDeg = 0.0f;
    g_lastControlTime = 0.0f;
    g_hasLastZenith = false;
    g_lastControlUpdateMs = millis();
    g_lastServoWriteDeg = kServoRetractAngle;
}

void loop() {
    const uint32_t nowMs = millis();
    if (g_servoCycleTestMode) {
        ServiceStatusLeds(nowMs, false);
        delay(1000);
        return;
    }

    if (!DataLoggerIsInitialized()) {
        ServiceStatusLeds(nowMs, false);
        InitializeWithRecovery(SystemError::DataLoggerInitialization,
                               &DataLoggerBegin,
                               "Data logger unavailable. Retrying...");
        return;
    }

    SensorData data;
    if (!AcquireSensorData(data)) {
        ServiceStatusLeds(nowMs, false);
        delay(1);
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

    FilteredState state;
    const bool hasFilteredState = flightComputer.Update(data, state);
    float manualOverrideDeg = 0.0f;
    const bool manualOverrideActive = NetworkTelemetryManualActuationOverride(manualOverrideDeg);
    manualOverrideDeg = ClampFloat(manualOverrideDeg, 0.0f, kServoMaxActuationDeg);

    if (!g_csvReplay.enabled) {
        DataLoggerLogTelemetry(data, flightComputer.Status(), hasFilteredState ? &state : nullptr);
    }

    if (hasFilteredState) {
        const FlightStatus status = flightComputer.Status();
        float dtState = state.time - g_lastControlTime;
        if (dtState < 0.0f || dtState > 1.0f) {
            dtState = settings::flight::kDefaultDtSeconds;
        }
        const float angularVelocityRadPerSec = EstimateAngularVelocity(state, dtState);
        const float lagBlend = ComputeLagBlend(dtState, kServoLatencySeconds);
        g_servoEffectiveDeg += (g_servoCommandDeg - g_servoEffectiveDeg) * lagBlend;
        g_servoEffectiveDeg = ClampFloat(g_servoEffectiveDeg, 0.0f, kServoMaxActuationDeg);

        const bool aboveMinExtendAltitude = altitudeAglFeet >= kServoMinExtendAltitudeFeet;
        if (manualOverrideActive) {
            g_servoCommandDeg = manualOverrideDeg;
        } else if (status == FlightStatus::Coast && aboveMinExtendAltitude) {
            if ((nowMs - g_lastControlUpdateMs) >= kControlUpdateIntervalMs) {
                g_servoCommandDeg = SelectActuationCommandDeg(state,
                                                              angularVelocityRadPerSec,
                                                              g_servoCommandDeg,
                                                              g_servoEffectiveDeg);
                g_lastControlUpdateMs = nowMs;
            }
        } else {
            g_servoCommandDeg = 0.0f;
        }

        WriteServoAngleDeg(g_servoCommandDeg);

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
        g_lastControlTime = state.time;
        g_lastZenith = state.zenith;
        g_hasLastZenith = true;
    } else if (manualOverrideActive) {
        g_servoCommandDeg = manualOverrideDeg;
        g_servoEffectiveDeg = manualOverrideDeg;
        WriteServoAngleDeg(g_servoCommandDeg);
    }

    if (hasFilteredState) {
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
