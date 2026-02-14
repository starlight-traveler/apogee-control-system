#include <Arduino.h>
#include <Servo.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "bno055_sensor.h"
#include "bmp581_sensor.h"
#include "cfd_table.h"
#include "data_logger.h"
#include "flight_computer.h"
#include "settings.h"
constexpr bool kEnableSerialTelemetry = settings::build::kEnableSerialTelemetry;

namespace {

constexpr uint8_t kStatusLedPin = settings::hardware::kStatusLedPin;
constexpr uint32_t kErrorBlinkIntervalMs = settings::flight::kErrorBlinkIntervalMs;
constexpr uint32_t kRecoveryBlinkIntervalMs = settings::flight::kRecoveryBlinkIntervalMs;
constexpr uint8_t kServoPin = settings::hardware::kServoPin;
constexpr int kServoExtendAngle = settings::hardware::kServoExtendAngle;
constexpr int kServoRetractAngle = settings::hardware::kServoRetractAngle;
constexpr bool kEnableCsvReplay = settings::replay::kEnableCsvReplay;
constexpr const char *kCsvReplayPath = settings::replay::kCsvReplayPath;
constexpr size_t kCsvLineBufferSize = settings::replay::kCsvLineBufferSize;

enum class SystemError : uint8_t {
    BnoInitialization = 0,
    BmpInitialization = 1,
    DataLoggerInitialization = 2,
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
    int idxHasQuaternion = -1;
    float lastTimestamp = 0.0f;
    bool hasLastTimestamp = false;
    uint32_t lastSampleMicros = 0;
};

CsvReplayState g_csvReplay;

uint8_t BlinkCountForError(SystemError error) {
    switch (error) {
        case SystemError::BnoInitialization:
            return 2;
        case SystemError::BmpInitialization:
            return 3;
        case SystemError::DataLoggerInitialization:
            return 4;
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
        if (!loggedFailure && failureMessage != nullptr && kEnableSerialTelemetry && Serial) {
            Serial.println(failureMessage);
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

    if (loggedFailure && failureMessage != nullptr && kEnableSerialTelemetry && Serial) {
        Serial.println("Recovered successfully.");
    }

    return true;
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
        CsvAssignIndexIfMatch(fields[i], "sensor_has_quaternion", g_csvReplay.idxHasQuaternion, i);
    }

    if (g_csvReplay.idxTimestamp < 0 || g_csvReplay.idxAltitudeFeet < 0) {
        return false;
    }

    g_csvReplay.enabled = true;
    g_csvReplay.headerParsed = true;
    g_csvReplay.hasLastTimestamp = false;
    if (kEnableSerialTelemetry && Serial) {
        Serial.print("CSV replay enabled: ");
        Serial.println(kCsvReplayPath);
    }
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

        return true;
    }

    g_csvReplay.completed = true;
    if (kEnableSerialTelemetry && Serial) {
        Serial.println("CSV replay complete.");
    }
    return false;
}

static bool AcquireSensorData(SensorData &data) {
    if (g_csvReplay.enabled) {
        return CsvReplayNextSample(data);
    }
    bool hasImu = Bno055SensorAcquire(data);
    bool hasAltimeter = Bmp581SensorAcquire(data);
    return hasImu || hasAltimeter;
}

static FlightComputer flightComputer;
static CfdTableStorage g_cfdTable;
static FlightStatus g_lastLoggedStatus = FlightStatus::Ground;
static bool g_hasLoggedStatus = false;
static bool g_hasPadAltitude = false;
static float g_padAltitudeFeet = 0.0f;
static Servo g_servo;
static bool g_servoExtended = false;
static bool g_servoCycleTestMode = false;

static void RunServoCycleTest() {
    g_servo.attach(kServoPin);

    const uint32_t startMs = millis();
    bool extend = false;
    while ((millis() - startMs) < settings::test::kServoCycleDurationMs) {
        extend = !extend;
        g_servo.write(extend ? kServoExtendAngle : kServoRetractAngle);
        delay(settings::test::kServoCycleToggleIntervalMs);
    }

    g_servo.write(kServoRetractAngle);
}

void setup() {
    g_servoCycleTestMode = settings::test::kEnableServoCycleTest;

    pinMode(kStatusLedPin, OUTPUT);
    digitalWrite(kStatusLedPin, LOW);

    Serial.begin(115200);
    if (kEnableSerialTelemetry) {
        while (!Serial && millis() < 2000) {
        }
    }

    if (g_servoCycleTestMode) {
        if (kEnableSerialTelemetry && Serial) {
            Serial.println("Servo cycle test mode active.");
        }
        RunServoCycleTest();
        if (kEnableSerialTelemetry && Serial) {
            Serial.println("Servo cycle test complete.");
        }
        return;
    }

    DataLoggerSetSerialLoggingEnabled(kEnableSerialTelemetry);

    InitializeWithRecovery(SystemError::DataLoggerInitialization,
                           &DataLoggerBegin,
                           "Sensor logging is disabled.");

    CsvReplayInit();

    g_servo.attach(kServoPin);
    g_servo.write(kServoRetractAngle);

    if (!g_csvReplay.enabled) {
        InitializeWithRecovery(SystemError::BnoInitialization,
                               &Bno055SensorBegin,
                               "Failed to initialize BNO055 sensor.");

        InitializeWithRecovery(SystemError::BmpInitialization,
                               &Bmp581SensorBegin,
                               "Failed to initialize BMP581 sensor.");
    }

    if (!CfdTableLoadFromSd("cfd.csv", &g_cfdTable, kEnableSerialTelemetry)) {
        CfdTableLoadFromSd("lib/cfd.csv", &g_cfdTable, kEnableSerialTelemetry);
    }

    const EnvironmentModel::Config environmentConfig;
    ApogeeVehicleParameters vehicleParameters;

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

    flightComputer.SetSerialReportingEnabled(kEnableSerialTelemetry);

    if (kEnableSerialTelemetry && Serial) {
        Serial.println("Flight computer initialized.");
    }

    g_lastLoggedStatus = flightComputer.Status();
    g_hasLoggedStatus = false;
}

void loop() {
    if (g_servoCycleTestMode) {
        delay(1000);
        return;
    }

    if (!DataLoggerIsInitialized()) {
        InitializeWithRecovery(SystemError::DataLoggerInitialization,
                               &DataLoggerBegin,
                               "Data logger unavailable. Retrying...");
        return;
    }

    SensorData data;
    if (!AcquireSensorData(data)) {
        delay(1);
        return;
    }

    if (!g_hasPadAltitude && data.altitudeFeet != 0.0f) {
        g_padAltitudeFeet = data.altitudeFeet;
        g_hasPadAltitude = true;
        if (kEnableSerialTelemetry && Serial) {
            Serial.print("Pad altitude reference (ft): ");
            Serial.println(g_padAltitudeFeet, 2);
        }
    }

    FilteredState state;
    const bool hasFilteredState = flightComputer.Update(data, state);

    if (!g_csvReplay.enabled) {
        DataLoggerLogTelemetry(data, flightComputer.Status(), hasFilteredState ? &state : nullptr);
    }

    if (hasFilteredState) {
        const FlightStatus status = flightComputer.Status();
        if (!g_servoExtended && status == FlightStatus::Coast) {
            g_servo.write(kServoExtendAngle);
            g_servoExtended = true;
            if (kEnableSerialTelemetry && Serial) {
                Serial.println("Servo extended (coast).");
            }
        }
        if (g_servoExtended && status == FlightStatus::Descent) {
            g_servo.write(kServoRetractAngle);
            g_servoExtended = false;
            if (kEnableSerialTelemetry && Serial) {
                Serial.println("Servo retracted (apogee).");
            }
        }
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

    if (hasFilteredState && kEnableSerialTelemetry && Serial) {
        Serial.print("t=");
        Serial.print(state.time, 3);
        Serial.print(" alt=");
        Serial.print(data.altitudeFeet, 2);
        Serial.print(" z=");
        Serial.print(state.position[2], 2);
        Serial.print(" vz=");
        Serial.print(state.velocity[2], 2);
        Serial.print(" az=");
        Serial.print(state.acceleration[2], 2);
        Serial.print(" iaz=");
        Serial.print(state.inertialAcceleration[2], 2);
        Serial.print(" apg=");
        Serial.print(state.apogeeEstimate, 2);
        Serial.print(" status=");
        Serial.println(FlightStatusToString(flightComputer.Status()));
    }

    DataLoggerService();
}
