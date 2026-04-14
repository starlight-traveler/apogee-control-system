#include <Arduino.h>
#include <algorithm>
#include <cmath>
#include <ctype.h>
#include <limits>
#include <stdlib.h>
#include <string.h>
#include <SPI.h>

#include "bno_sensor.h"
#include "bmp585_sensor.h"
#include "cfd_table.h"
#include "constants.h"
#include "data_logger.h"
#include "ellipse_20.h"
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
#include "wt901_sensor.h"

namespace {

/// Flight-loop constants mirrored locally to keep `main.cpp` readable.
constexpr uint8_t kStatusLedPin = settings::hardware::kStatusLedPin;
constexpr uint8_t kBootLedPin = 0;
constexpr uint8_t kBuzzerPin = settings::hardware::kBuzzerPin;
constexpr int8_t kAirliftSsPin = settings::network::kAirliftSsPin;
constexpr int8_t kAirliftResetPin = settings::network::kAirliftResetPin;
constexpr int8_t kAirliftGpio0Pin = settings::network::kAirliftGpio0Pin;
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
constexpr bool kPulseEnabled = settings::sensors::ellipse20::kEnabled;
constexpr bool kWt901Enabled = settings::sensors::wt901::kEnabled;
constexpr int8_t kBnoResetPin = settings::sensors::bno::kModel == settings::sensors::bno::Model::Bno085
                                    ? settings::sensors::bno085::kResetPin
                                    : settings::sensors::bno055::kResetPin;
constexpr uint32_t kBnoResetPulseDelayMs = settings::sensors::bno::kModel == settings::sensors::bno::Model::Bno085
                                               ? settings::sensors::bno085::kResetPulseDelayMs
                                               : 0;
constexpr uint32_t kBnoPostResetBootDelayMs = settings::sensors::bno::kModel == settings::sensors::bno::Model::Bno085
                                                  ? settings::sensors::bno085::kPostResetBootDelayMs
                                                  : 0;
constexpr uint8_t kBmpChipSelectPin = 35;
constexpr uint8_t kMs5611ChipSelectPin = settings::sensors::ms5611::kChipSelectPin;
constexpr int kBnoChipSelectPin = settings::sensors::bno::kModel == settings::sensors::bno::Model::Bno085
                                      ? settings::sensors::bno085::kChipSelectPin
                                      : -1;
constexpr uint8_t kIcmChipSelectPin = settings::sensors::icm20948::kChipSelectPin;
constexpr uint8_t kLsmAccelGyroChipSelectPin = settings::sensors::lsm9ds1::kAccelGyroChipSelectPin;
constexpr uint8_t kLsmMagChipSelectPin = settings::sensors::lsm9ds1::kMagChipSelectPin;
constexpr uint32_t kPulseSampleMaxAgeUs = settings::sensors::ellipse20::kSampleMaxAgeUs;
constexpr bool kPulseUseAsMainQuaternionFallback =
    settings::sensors::ellipse20::kUseAsMainQuaternionFallback;
constexpr uint32_t kRecoveryRetryInitialMs = settings::flight::kRecoveryRetryInitialMs;
constexpr uint32_t kRecoveryRetryStepMs = settings::flight::kRecoveryRetryStepMs;
constexpr uint32_t kRecoveryRetryMaxMs = settings::flight::kRecoveryRetryMaxMs;
constexpr uint32_t kCriticalStartupResetAttempts = 5;
constexpr uint32_t kTimingLogIntervalMs = settings::flight::kTimingLogIntervalMs;
constexpr uint8_t kCfdStartupRetryCount = 5;
constexpr uint32_t kCfdStartupRetryDelayMs = 200;
constexpr uint32_t kSpiBusStartupSettleDelayMs = 50;
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

enum class SensorRailHealth : uint8_t {
    Unavailable = 0,
    Initializing = 1,
    Stale = 2,
    Degraded = 3,
    Healthy = 4,
};

struct RailHealthThresholds {
    float tiltErrorHealthyDeg = 0.0f;
    float tiltErrorDegradedDeg = 0.0f;
    float tiltRmsHealthyDeg = 0.0f;
    float tiltRmsDegradedDeg = 0.0f;
    float tiltRateHealthyDegPerSec = 0.0f;
    float tiltRateDegradedDegPerSec = 0.0f;
    float accelMagnitudeHealthyG = 0.0f;
    float accelMagnitudeDegradedG = 0.0f;
    bool useTiltRms = false;
    bool useTiltRate = false;
    bool useAccelMagnitude = false;
};

struct RailHealthState {
    bool hasPreviousTilt = false;
    float previousTiltVector[3] = {0.0f, 0.0f, 1.0f};
    uint32_t previousSampleUs = 0;
    bool hasTiltRms = false;
    float tiltRmsSqDeg = 0.0f;
    float lastTiltErrorDeg = INFINITY;
    float lastTiltRmsDeg = INFINITY;
    float lastTiltRateJumpDegPerSec = INFINITY;
    float lastAccelMagnitudeErrorG = INFINITY;
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

struct BuzzerNote {
    uint16_t frequencyHz;
    uint16_t durationMs;
};

// Note frequencies (Hz) - full chromatic scale
constexpr uint16_t kNoteRest = 0;
constexpr uint16_t kNoteC4 = 262;
constexpr uint16_t kNoteCS4 = 277;
constexpr uint16_t kNoteD4 = 294;
constexpr uint16_t kNoteDS4 = 311;
constexpr uint16_t kNoteE4 = 330;
constexpr uint16_t kNoteF4 = 349;
constexpr uint16_t kNoteFS4 = 370;
constexpr uint16_t kNoteG4 = 392;
constexpr uint16_t kNoteGS4 = 415;
constexpr uint16_t kNoteGSharp4 = 415;  // Alias for existing code
constexpr uint16_t kNoteA4 = 440;
constexpr uint16_t kNoteAS4 = 466;
constexpr uint16_t kNoteB4 = 494;
constexpr uint16_t kNoteC5 = 523;
constexpr uint16_t kNoteCS5 = 554;
constexpr uint16_t kNoteD5 = 587;
constexpr uint16_t kNoteDS5 = 622;
constexpr uint16_t kNoteE5 = 659;
constexpr uint16_t kNoteF5 = 698;
constexpr uint16_t kNoteFS5 = 740;
constexpr uint16_t kNoteG5 = 784;
constexpr uint16_t kNoteGS5 = 831;
constexpr uint16_t kNoteA5 = 880;
constexpr uint16_t kNoteAS5 = 932;
constexpr uint16_t kNoteB5 = 988;
constexpr uint16_t kNoteC6 = 1047;
constexpr uint16_t kNoteD6 = 1175;
constexpr uint16_t kNoteE6 = 1319;
constexpr uint16_t kNoteF6 = 1397;
constexpr uint16_t kNoteG6 = 1568;

// Number of startup melodies available
constexpr uint8_t kNumStartupMelodies = 9;

void PlayFailureSiren();

// Boot LED state tracking
bool g_bootLedInitialized = false;
bool g_bootLedState = false;
uint32_t g_bootLedLastToggleMs = 0;

/// Initializes the boot status LED on pin 2.
void BootLedBegin() {
    pinMode(kBootLedPin, OUTPUT);
    digitalWrite(kBootLedPin, LOW);
    g_bootLedInitialized = true;
    g_bootLedState = false;
    g_bootLedLastToggleMs = millis();
}

/// Toggles the boot LED for a slow blink pattern (long on/off during boot).
/// Call this periodically during setup to show boot progress.
void BootLedSlowBlink() {
    if (!g_bootLedInitialized) {
        BootLedBegin();
    }
    constexpr uint32_t kSlowBlinkIntervalMs = 500;
    const uint32_t nowMs = millis();
    if ((nowMs - g_bootLedLastToggleMs) >= kSlowBlinkIntervalMs) {
        g_bootLedState = !g_bootLedState;
        digitalWrite(kBootLedPin, g_bootLedState ? HIGH : LOW);
        g_bootLedLastToggleMs = nowMs;
    }
}

/// Runs a fast blink pattern (short on/off) to indicate boot failure.
/// This function loops forever and does not return.
[[noreturn]] void BootLedFailureBlink() {
    if (!g_bootLedInitialized) {
        BootLedBegin();
    }
    constexpr uint32_t kFastBlinkIntervalMs = 100;
    while (true) {
        digitalWrite(kBootLedPin, HIGH);
        delay(kFastBlinkIntervalMs);
        digitalWrite(kBootLedPin, LOW);
        delay(kFastBlinkIntervalMs);
    }
}

/// Turns the boot LED permanently on to indicate successful boot.
void BootLedSolidOn() {
    if (!g_bootLedInitialized) {
        BootLedBegin();
    }
    digitalWrite(kBootLedPin, HIGH);
    g_bootLedState = true;
}

[[noreturn]] void ForceTeensyReboot(const char *reason) {
    LOG_PRINT("[fatal] rebooting Teensy: ");
    LOG_PRINTLN(reason);
    Serial.flush();
    PlayFailureSiren();
    // Blink fast to indicate failure before reboot
    if (!g_bootLedInitialized) {
        BootLedBegin();
    }
    for (int i = 0; i < 20; ++i) {
        digitalWrite(kBootLedPin, HIGH);
        delay(100);
        digitalWrite(kBootLedPin, LOW);
        delay(100);
    }
    SCB_AIRCR = 0x05FA0004;
    __DSB();
    while (true) {
        // If reboot fails, keep blinking fast
        digitalWrite(kBootLedPin, HIGH);
        delay(100);
        digitalWrite(kBootLedPin, LOW);
        delay(100);
    }
}

bool StartBnoDuringSetup() {
    if (!kBnoEnabled) {
        return false;
    }
    return BnoSensorBegin();
}

bool StartPulseDuringSetup() {
    if (!kPulseEnabled) {
        return false;
    }
    return Ellipse20SensorBegin();
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

void DrivePinHighIfValid(int pin) {
    if (pin < 0) {
        return;
    }
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
}

void DrivePinLowIfValid(int pin) {
    if (pin < 0) {
        return;
    }
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

void PrepareSpiChipSelectPins() {
    DrivePinHighIfValid(kAirliftSsPin);
    DrivePinHighIfValid(kBmpChipSelectPin);
    DrivePinHighIfValid(kMs5611ChipSelectPin);
    DrivePinHighIfValid(kBnoChipSelectPin);
    DrivePinHighIfValid(kIcmChipSelectPin);
    DrivePinHighIfValid(kLsmAccelGyroChipSelectPin);
    DrivePinHighIfValid(kLsmMagChipSelectPin);
}

void PreparePeripheralResetPins() {
    DrivePinHighIfValid(kAirliftResetPin);
    if (kAirliftGpio0Pin >= 0) {
        pinMode(kAirliftGpio0Pin, INPUT_PULLUP);
    }
    if (kBnoResetPin >= 0) {
        DrivePinHighIfValid(kBnoResetPin);
    }
}

void ResetBnoIfWired() {
    if (kBnoResetPin < 0) {
        return;
    }
    DrivePinHighIfValid(kBnoResetPin);
    delay(kSpiBusStartupSettleDelayMs);
    DrivePinLowIfValid(kBnoResetPin);
    delay(kBnoResetPulseDelayMs);
    DrivePinHighIfValid(kBnoResetPin);
    delay(kBnoPostResetBootDelayMs);
}

void PrepareSpiBusesForStartup() {
    PrepareSpiChipSelectPins();
    PreparePeripheralResetPins();
    SPI.begin();
    SPI1.begin();
    PrepareSpiChipSelectPins();
    delay(kSpiBusStartupSettleDelayMs);
    ResetBnoIfWired();
    PrepareSpiChipSelectPins();
    delay(kSpiBusStartupSettleDelayMs);
}

/// Helper to play a melody array
void PlayMelody(const BuzzerNote *melody, size_t length) {
    pinMode(kBuzzerPin, OUTPUT);
    for (size_t i = 0; i < length; ++i) {
        if (melody[i].frequencyHz == kNoteRest) {
            noTone(kBuzzerPin);
            delay(melody[i].durationMs);
        } else {
            tone(kBuzzerPin, melody[i].frequencyHz, melody[i].durationMs);
            delay(melody[i].durationMs + 20);
        }
    }
    noTone(kBuzzerPin);
}

// =============================================================================
// STARTUP MELODIES (~5-10 seconds each)
// =============================================================================

// 0: Star Wars - Imperial March (original)
static const BuzzerNote kMelodyImperialMarch[] = {
    {kNoteA4, 500}, {kNoteA4, 500}, {kNoteA4, 500}, {kNoteF4, 350}, {kNoteC5, 150},
    {kNoteA4, 500}, {kNoteF4, 350}, {kNoteC5, 150}, {kNoteA4, 650},
    {kNoteE5, 500}, {kNoteE5, 500}, {kNoteE5, 500}, {kNoteF5, 350}, {kNoteC5, 150},
    {kNoteGSharp4, 500}, {kNoteF4, 350}, {kNoteC5, 150}, {kNoteA4, 650},
};

// 1: Ode to Joy - Beethoven
static const BuzzerNote kMelodyOdeToJoy[] = {
    {kNoteE4, 400}, {kNoteE4, 400}, {kNoteF4, 400}, {kNoteG4, 400},
    {kNoteG4, 400}, {kNoteF4, 400}, {kNoteE4, 400}, {kNoteD4, 400},
    {kNoteC4, 400}, {kNoteC4, 400}, {kNoteD4, 400}, {kNoteE4, 400},
    {kNoteE4, 600}, {kNoteD4, 200}, {kNoteD4, 800},
    {kNoteE4, 400}, {kNoteE4, 400}, {kNoteF4, 400}, {kNoteG4, 400},
    {kNoteG4, 400}, {kNoteF4, 400}, {kNoteE4, 400}, {kNoteD4, 400},
    {kNoteC4, 400}, {kNoteC4, 400}, {kNoteD4, 400}, {kNoteE4, 400},
    {kNoteD4, 600}, {kNoteC4, 200}, {kNoteC4, 800},
};

// 2: Für Elise - Beethoven
static const BuzzerNote kMelodyFurElise[] = {
    {kNoteE5, 150}, {kNoteDS5, 150}, {kNoteE5, 150}, {kNoteDS5, 150}, {kNoteE5, 150},
    {kNoteB4, 150}, {kNoteD5, 150}, {kNoteC5, 150}, {kNoteA4, 300},
    {kNoteRest, 150}, {kNoteC4, 150}, {kNoteE4, 150}, {kNoteA4, 150}, {kNoteB4, 300},
    {kNoteRest, 150}, {kNoteE4, 150}, {kNoteGS4, 150}, {kNoteB4, 150}, {kNoteC5, 300},
    {kNoteRest, 150}, {kNoteE4, 150},
    {kNoteE5, 150}, {kNoteDS5, 150}, {kNoteE5, 150}, {kNoteDS5, 150}, {kNoteE5, 150},
    {kNoteB4, 150}, {kNoteD5, 150}, {kNoteC5, 150}, {kNoteA4, 300},
    {kNoteRest, 150}, {kNoteC4, 150}, {kNoteE4, 150}, {kNoteA4, 150}, {kNoteB4, 300},
    {kNoteRest, 150}, {kNoteE4, 150}, {kNoteC5, 150}, {kNoteB4, 150}, {kNoteA4, 450},
};

// 3: Happy Birthday
static const BuzzerNote kMelodyHappyBirthday[] = {
    {kNoteC4, 300}, {kNoteC4, 200}, {kNoteD4, 500}, {kNoteC4, 500}, {kNoteF4, 500}, {kNoteE4, 900},
    {kNoteC4, 300}, {kNoteC4, 200}, {kNoteD4, 500}, {kNoteC4, 500}, {kNoteG4, 500}, {kNoteF4, 900},
    {kNoteC4, 300}, {kNoteC4, 200}, {kNoteC5, 500}, {kNoteA4, 500}, {kNoteF4, 500}, {kNoteE4, 500}, {kNoteD4, 800},
    {kNoteAS4, 300}, {kNoteAS4, 200}, {kNoteA4, 500}, {kNoteF4, 500}, {kNoteG4, 500}, {kNoteF4, 900},
};

// 4: Twinkle Twinkle Little Star
static const BuzzerNote kMelodyTwinkle[] = {
    {kNoteC4, 400}, {kNoteC4, 400}, {kNoteG4, 400}, {kNoteG4, 400},
    {kNoteA4, 400}, {kNoteA4, 400}, {kNoteG4, 800},
    {kNoteF4, 400}, {kNoteF4, 400}, {kNoteE4, 400}, {kNoteE4, 400},
    {kNoteD4, 400}, {kNoteD4, 400}, {kNoteC4, 800},
    {kNoteG4, 400}, {kNoteG4, 400}, {kNoteF4, 400}, {kNoteF4, 400},
    {kNoteE4, 400}, {kNoteE4, 400}, {kNoteD4, 800},
    {kNoteG4, 400}, {kNoteG4, 400}, {kNoteF4, 400}, {kNoteF4, 400},
    {kNoteE4, 400}, {kNoteE4, 400}, {kNoteD4, 800},
};

// 5: Jingle Bells
static const BuzzerNote kMelodyJingleBells[] = {
    {kNoteE4, 300}, {kNoteE4, 300}, {kNoteE4, 600},
    {kNoteE4, 300}, {kNoteE4, 300}, {kNoteE4, 600},
    {kNoteE4, 300}, {kNoteG4, 300}, {kNoteC4, 400}, {kNoteD4, 200}, {kNoteE4, 800},
    {kNoteF4, 300}, {kNoteF4, 300}, {kNoteF4, 400}, {kNoteF4, 200},
    {kNoteF4, 300}, {kNoteE4, 300}, {kNoteE4, 300}, {kNoteE4, 150}, {kNoteE4, 150},
    {kNoteE4, 300}, {kNoteD4, 300}, {kNoteD4, 300}, {kNoteE4, 300}, {kNoteD4, 500}, {kNoteG4, 500},
};

// 6: Harry Potter Theme (Hedwig's Theme)
static const BuzzerNote kMelodyHarryPotter[] = {
    {kNoteB4, 400}, {kNoteE5, 600}, {kNoteG5, 200}, {kNoteFS5, 400}, {kNoteE5, 800},
    {kNoteB5, 400}, {kNoteA5, 1200},
    {kNoteFS5, 1200},
    {kNoteE5, 600}, {kNoteG5, 200}, {kNoteFS5, 400}, {kNoteDS5, 800},
    {kNoteF5, 400}, {kNoteB4, 1600},
};

// 7: Tetris (Korobeiniki)
static const BuzzerNote kMelodyTetris[] = {
    {kNoteE5, 400}, {kNoteB4, 200}, {kNoteC5, 200}, {kNoteD5, 400}, {kNoteC5, 200}, {kNoteB4, 200},
    {kNoteA4, 400}, {kNoteA4, 200}, {kNoteC5, 200}, {kNoteE5, 400}, {kNoteD5, 200}, {kNoteC5, 200},
    {kNoteB4, 600}, {kNoteC5, 200}, {kNoteD5, 400}, {kNoteE5, 400},
    {kNoteC5, 400}, {kNoteA4, 400}, {kNoteA4, 800},
    {kNoteD5, 400}, {kNoteF5, 200}, {kNoteA5, 400}, {kNoteG5, 200}, {kNoteF5, 200},
    {kNoteE5, 600}, {kNoteC5, 200}, {kNoteE5, 400}, {kNoteD5, 200}, {kNoteC5, 200},
    {kNoteB4, 400}, {kNoteB4, 200}, {kNoteC5, 200}, {kNoteD5, 400}, {kNoteE5, 400},
    {kNoteC5, 400}, {kNoteA4, 400}, {kNoteA4, 400},
};

// 8: Never Gonna Give You Up (Rick Roll!)
static const BuzzerNote kMelodyRickRoll[] = {
    // "Never gonna give you up"
    {kNoteD4, 150}, {kNoteE4, 150}, {kNoteG4, 150}, {kNoteE4, 150},
    {kNoteB4, 450}, {kNoteB4, 450}, {kNoteA4, 600},
    {kNoteRest, 200},
    // "Never gonna let you down"
    {kNoteD4, 150}, {kNoteE4, 150}, {kNoteG4, 150}, {kNoteE4, 150},
    {kNoteA4, 450}, {kNoteA4, 450}, {kNoteG4, 450}, {kNoteFS4, 150}, {kNoteE4, 300},
    {kNoteRest, 200},
    // "Never gonna run around"
    {kNoteD4, 150}, {kNoteE4, 150}, {kNoteG4, 150}, {kNoteE4, 150},
    {kNoteG4, 450}, {kNoteA4, 300}, {kNoteFS4, 450},
    {kNoteRest, 100},
    {kNoteE4, 250}, {kNoteE4, 150}, {kNoteFS4, 150}, {kNoteE4, 150}, {kNoteD4, 500},
};

/// Plays a random startup melody (one of 9 songs).
///
/// This is intentionally blocking and should be disabled for flight builds if
/// startup latency matters more than audible status.
void PlayStartupMarch() {
    // Seed random with analog noise + millis for variety
    randomSeed(analogRead(0) ^ micros());
    const uint8_t choice = random(kNumStartupMelodies);

    LOG_PRINT("[buzzer] Playing melody #");
    LOG_PRINTLN(choice);

    switch (choice) {
        case 0:
            PlayMelody(kMelodyImperialMarch, sizeof(kMelodyImperialMarch) / sizeof(kMelodyImperialMarch[0]));
            break;
        case 1:
            PlayMelody(kMelodyOdeToJoy, sizeof(kMelodyOdeToJoy) / sizeof(kMelodyOdeToJoy[0]));
            break;
        case 2:
            PlayMelody(kMelodyFurElise, sizeof(kMelodyFurElise) / sizeof(kMelodyFurElise[0]));
            break;
        case 3:
            PlayMelody(kMelodyHappyBirthday, sizeof(kMelodyHappyBirthday) / sizeof(kMelodyHappyBirthday[0]));
            break;
        case 4:
            PlayMelody(kMelodyTwinkle, sizeof(kMelodyTwinkle) / sizeof(kMelodyTwinkle[0]));
            break;
        case 5:
            PlayMelody(kMelodyJingleBells, sizeof(kMelodyJingleBells) / sizeof(kMelodyJingleBells[0]));
            break;
        case 6:
            PlayMelody(kMelodyHarryPotter, sizeof(kMelodyHarryPotter) / sizeof(kMelodyHarryPotter[0]));
            break;
        case 7:
            PlayMelody(kMelodyTetris, sizeof(kMelodyTetris) / sizeof(kMelodyTetris[0]));
            break;
        case 8:
        default:
            PlayMelody(kMelodyRickRoll, sizeof(kMelodyRickRoll) / sizeof(kMelodyRickRoll[0]));
            break;
    }
}

/// Plays a 5-second warning siren for failure/reset conditions.
void PlayFailureSiren() {
    constexpr uint16_t kLowHz = 660;
    constexpr uint16_t kHighHz = 1320;
    constexpr uint16_t kStepMs = 200;
    constexpr uint32_t kSirenDurationMs = 5000;

    pinMode(kBuzzerPin, OUTPUT);

    uint32_t startMs = millis();
    while ((millis() - startMs) < kSirenDurationMs) {
        tone(kBuzzerPin, kLowHz, kStepMs);
        delay(kStepMs);
        tone(kBuzzerPin, kHighHz, kStepMs);
        delay(kStepMs);
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
    uint32_t pulseHits = 0;
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
    PairStats icmPulse;
    PairStats lsmPulse;
    PairStats pulseBno;
};

static SensorComparisonStats g_sensorComparisonStats;
static float g_icmLsmCrossCheckTrust = 1.0f;
static float g_bnoReferenceCrossCheckTrust = 1.0f;
static float g_pulseCrossCheckTrust = 1.0f;
static float g_icmRailTrust = 1.0f;
static float g_lsmRailTrust = 1.0f;
static float g_bnoRailTrust = 1.0f;
static float g_pulseRailTrust = 1.0f;
static SensorRailHealth g_icmHealth = SensorRailHealth::Unavailable;
static SensorRailHealth g_lsmHealth = SensorRailHealth::Unavailable;
static SensorRailHealth g_bnoHealth = SensorRailHealth::Unavailable;
static SensorRailHealth g_pulseHealth = SensorRailHealth::Unavailable;
static RailHealthState g_icmRailState;
static RailHealthState g_lsmRailState;
static RailHealthState g_bnoRailState;
static RailHealthState g_pulseRailState;
static bool g_haveMainQuaternionReference = false;
static float g_lastMainQuaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static FlightStatus g_healthPhase = FlightStatus::Ground;

// Trust hysteresis state: track previous trust direction to reduce oscillation.
static float g_icmLsmPreviousTrust = 1.0f;
static bool g_icmLsmTrustWasDecreasing = false;
static float g_bnoPreviousTrust = 1.0f;
static bool g_bnoTrustWasDecreasing = false;
static float g_pulsePreviousTrust = 1.0f;
static bool g_pulseTrustWasDecreasing = false;

static float VectorDiffNorm3(const float a[3], const float b[3]) {
    const float dx = a[0] - b[0];
    const float dy = a[1] - b[1];
    const float dz = a[2] - b[2];
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

static bool LoadValidatedQuaternion(const float values[4], math_utils::Quaternion &quat) {
    quat = math_utils::MakeQuaternion(values[0], values[1], values[2], values[3]);
    return math_utils::ValidateQuaternion(quat);
}

static bool QuaternionFromPitchRollDeg(float pitchDeg, float rollDeg, float quaternion[4]) {
    if (quaternion == nullptr || !std::isfinite(pitchDeg) || !std::isfinite(rollDeg)) {
        return false;
    }
    constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
    const float halfPitch = 0.5f * pitchDeg * kDegToRad;
    const float halfRoll = 0.5f * rollDeg * kDegToRad;
    float sinPitch = 0.0f;
    float cosPitch = 1.0f;
    float sinRoll = 0.0f;
    float cosRoll = 1.0f;
    math_utils::FastSinCos(halfPitch, sinPitch, cosPitch);
    math_utils::FastSinCos(halfRoll, sinRoll, cosRoll);
    const math_utils::Quaternion q = math_utils::Normalize(math_utils::MakeQuaternion(
        cosPitch * cosRoll,
        -cosPitch * sinRoll,
        -sinPitch * cosRoll,
        -sinPitch * sinRoll));
    quaternion[0] = q.w;
    quaternion[1] = q.x;
    quaternion[2] = q.y;
    quaternion[3] = q.z;
    return true;
}

static bool TiltQuaternionFromYprDeg(const float yprDeg[3], float quaternion[4]) {
    if (yprDeg == nullptr) {
        return false;
    }
    return QuaternionFromPitchRollDeg(yprDeg[1], yprDeg[2], quaternion);
}

static bool TiltQuaternionFromQuaternion(const float input[4], float quaternion[4]) {
    math_utils::Quaternion q = math_utils::MakeQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
    if (!LoadValidatedQuaternion(input, q)) {
        return false;
    }
    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    math_utils::QuaternionToEuler(q, yaw, pitch, roll);
    return QuaternionFromPitchRollDeg(pitch * (180.0f / 3.14159265358979323846f),
                                      roll * (180.0f / 3.14159265358979323846f),
                                      quaternion);
}

static void PreferTiltOnlyQuaternion(float quaternion[4], bool &hasQuaternion, const float bootstrapYprDeg[3], bool hasBootstrapYpr) {
    float tiltQuaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    if (hasBootstrapYpr && TiltQuaternionFromYprDeg(bootstrapYprDeg, tiltQuaternion)) {
        for (int i = 0; i < 4; ++i) {
            quaternion[i] = tiltQuaternion[i];
        }
        hasQuaternion = true;
        return;
    }
    if (hasQuaternion && TiltQuaternionFromQuaternion(quaternion, tiltQuaternion)) {
        for (int i = 0; i < 4; ++i) {
            quaternion[i] = tiltQuaternion[i];
        }
        return;
    }
    hasQuaternion = false;
}

static bool GravityVectorFromQuaternion(const float quaternion[4], float gravityBody[3]) {
    math_utils::Quaternion q = math_utils::MakeQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
    if (!LoadValidatedQuaternion(quaternion, q)) {
        return false;
    }
    const float w = q.w;
    const float x = q.x;
    const float y = q.y;
    const float z = q.z;

    gravityBody[0] = 2.0f * (x * z + w * y);
    gravityBody[1] = 2.0f * (y * z - w * x);
    gravityBody[2] = 1.0f - 2.0f * (x * x + y * y);
    const float normSq = gravityBody[0] * gravityBody[0] + gravityBody[1] * gravityBody[1] +
                         gravityBody[2] * gravityBody[2];
    return std::isfinite(normSq) && normSq > 1.0e-6f;
}

static bool GravityVectorFromYprDeg(const float yprDeg[3], float gravityBody[3]) {
    float tiltQuaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    if (!TiltQuaternionFromYprDeg(yprDeg, tiltQuaternion)) {
        return false;
    }
    return GravityVectorFromQuaternion(tiltQuaternion, gravityBody);
}

static bool Normalize3(float vector[3]) {
    const float normSq = vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2];
    if (!std::isfinite(normSq) || normSq <= 1.0e-6f) {
        return false;
    }
    const float invNorm = 1.0f / sqrtf(normSq);
    vector[0] *= invNorm;
    vector[1] *= invNorm;
    vector[2] *= invNorm;
    return true;
}

static float AngleBetweenUnitVectorsDeg(const float a[3], const float b[3]) {
    const float dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    return acosf(std::clamp(dot, -1.0f, 1.0f)) * (180.0f / 3.14159265358979323846f);
}

static bool GravityVectorFromAccelBody(const float accelBody[3], float gravityBody[3]) {
    gravityBody[0] = accelBody[0];
    gravityBody[1] = accelBody[1];
    gravityBody[2] = accelBody[2];
    return Normalize3(gravityBody);
}

static bool RailTiltVectorForPhase(const float accelBody[3],
                                   bool hasAccel,
                                   const float quaternion[4],
                                   bool hasQuaternion,
                                   FlightStatus phase,
                                   float gravityBody[3]) {
    const bool preferAccel = (phase == FlightStatus::Ground || phase == FlightStatus::Descent);
    if (preferAccel) {
        if (hasAccel && GravityVectorFromAccelBody(accelBody, gravityBody)) {
            return true;
        }
        return hasQuaternion && GravityVectorFromQuaternion(quaternion, gravityBody);
    }
    if (hasQuaternion && GravityVectorFromQuaternion(quaternion, gravityBody)) {
        return true;
    }
    return hasAccel && GravityVectorFromAccelBody(accelBody, gravityBody);
}

static RailHealthThresholds RailHealthThresholdsForPhase(FlightStatus phase) {
    switch (phase) {
        case FlightStatus::Ground:
            return RailHealthThresholds{5.0f, 15.0f, 4.0f, 10.0f, 45.0f, 180.0f, 0.15f, 0.40f, true, false, true};
        case FlightStatus::Burn:
            return RailHealthThresholds{15.0f, 35.0f, 12.0f, 25.0f, 240.0f, 900.0f, 1000.0f, 2000.0f, false, false, false};
        case FlightStatus::Coast:
        case FlightStatus::Overshoot:
            return RailHealthThresholds{10.0f, 25.0f, 8.0f, 18.0f, 120.0f, 360.0f, 1000.0f, 2000.0f, true, true, false};
        case FlightStatus::Descent:
            return RailHealthThresholds{6.0f, 18.0f, 5.0f, 12.0f, 75.0f, 240.0f, 0.20f, 0.50f, true, true, true};
    }
    return RailHealthThresholds{};
}

static float MetricDescendingTrust(float value, float fullTrustMax, float zeroTrustMin) {
    if (value <= fullTrustMax) {
        return 1.0f;
    }
    if (value >= zeroTrustMin || !(zeroTrustMin > fullTrustMax)) {
        return 0.0f;
    }
    return (zeroTrustMin - value) / (zeroTrustMin - fullTrustMax);
}

static float UpdateRailTrust(RailHealthState &state,
                             bool hasTiltVector,
                             const float tiltVector[3],
                             bool hasAccel,
                             const float accelBody[3],
                             uint32_t sampleUs,
                             bool hasReference,
                             const float referenceGravity[3],
                             FlightStatus phase) {
    const RailHealthThresholds thresholds = RailHealthThresholdsForPhase(phase);

    if (hasTiltVector && hasReference) {
        state.lastTiltErrorDeg = AngleBetweenUnitVectorsDeg(tiltVector, referenceGravity);
        const float dtSeconds = (state.previousSampleUs != 0 && sampleUs > state.previousSampleUs)
                                    ? static_cast<float>(sampleUs - state.previousSampleUs) * 1.0e-6f
                                    : 0.0f;
        if (state.hasPreviousTilt && dtSeconds > 1.0e-4f) {
            state.lastTiltRateJumpDegPerSec =
                AngleBetweenUnitVectorsDeg(tiltVector, state.previousTiltVector) / dtSeconds;
        } else {
            state.lastTiltRateJumpDegPerSec = 0.0f;
        }
        const float alpha = (dtSeconds > 0.0f) ? std::clamp(dtSeconds / (0.35f + dtSeconds), 0.05f, 1.0f) : 1.0f;
        const float errorSq = state.lastTiltErrorDeg * state.lastTiltErrorDeg;
        if (!state.hasTiltRms) {
            state.tiltRmsSqDeg = errorSq;
            state.hasTiltRms = true;
        } else {
            state.tiltRmsSqDeg += alpha * (errorSq - state.tiltRmsSqDeg);
        }
        state.lastTiltRmsDeg = sqrtf(std::max(0.0f, state.tiltRmsSqDeg));
        state.previousTiltVector[0] = tiltVector[0];
        state.previousTiltVector[1] = tiltVector[1];
        state.previousTiltVector[2] = tiltVector[2];
        state.hasPreviousTilt = true;
        state.previousSampleUs = sampleUs;
    } else {
        state.lastTiltErrorDeg = INFINITY;
        state.lastTiltRateJumpDegPerSec = INFINITY;
        state.lastTiltRmsDeg = state.hasTiltRms ? sqrtf(std::max(0.0f, state.tiltRmsSqDeg)) : INFINITY;
    }

    if (hasAccel) {
        const float magnitudeG =
            sqrtf(accelBody[0] * accelBody[0] + accelBody[1] * accelBody[1] + accelBody[2] * accelBody[2]) /
            constants::kGravity;
        state.lastAccelMagnitudeErrorG = fabsf(magnitudeG - 1.0f);
    } else {
        state.lastAccelMagnitudeErrorG = INFINITY;
    }

    if (!hasReference || !hasTiltVector) {
        return 1.0f;
    }

    float trust = 1.0f;
    trust = std::min(trust,
                     MetricDescendingTrust(state.lastTiltErrorDeg,
                                          thresholds.tiltErrorHealthyDeg,
                                          thresholds.tiltErrorDegradedDeg));
    if (thresholds.useTiltRms && state.hasTiltRms) {
        trust = std::min(trust,
                         MetricDescendingTrust(state.lastTiltRmsDeg,
                                              thresholds.tiltRmsHealthyDeg,
                                              thresholds.tiltRmsDegradedDeg));
    }
    if (thresholds.useTiltRate && std::isfinite(state.lastTiltRateJumpDegPerSec)) {
        trust = std::min(trust,
                         MetricDescendingTrust(state.lastTiltRateJumpDegPerSec,
                                              thresholds.tiltRateHealthyDegPerSec,
                                              thresholds.tiltRateDegradedDegPerSec));
    }
    if (thresholds.useAccelMagnitude && std::isfinite(state.lastAccelMagnitudeErrorG)) {
        trust = std::min(trust,
                         MetricDescendingTrust(state.lastAccelMagnitudeErrorG,
                                              thresholds.accelMagnitudeHealthyG,
                                              thresholds.accelMagnitudeDegradedG));
    }
    return std::clamp(trust, 0.0f, 1.0f);
}

static bool ZenithDegFromAccelBody(const float accelBody[3], float &zenithDeg) {
    const float normSq = accelBody[0] * accelBody[0] + accelBody[1] * accelBody[1] + accelBody[2] * accelBody[2];
    if (!std::isfinite(normSq) || normSq <= 1.0e-6f) {
        return false;
    }
    const float norm = sqrtf(normSq);
    const float cosZenith = std::fabs(accelBody[2] / norm);
    zenithDeg = acosf(std::clamp(cosZenith, 0.0f, 1.0f)) * (180.0f / 3.14159265358979323846f);
    return std::isfinite(zenithDeg);
}

static bool ZenithDifferenceDegFromAccelBody(const float accelA[3], const float accelB[3], float &differenceDeg) {
    float zenithA = 0.0f;
    float zenithB = 0.0f;
    if (!ZenithDegFromAccelBody(accelA, zenithA) || !ZenithDegFromAccelBody(accelB, zenithB)) {
        return false;
    }
    differenceDeg = fabsf(zenithA - zenithB);
    return std::isfinite(differenceDeg);
}

static bool AccelVectorAngleDeg(const float accelA[3], const float accelB[3], float &angleDeg) {
    const float normASq = accelA[0] * accelA[0] + accelA[1] * accelA[1] + accelA[2] * accelA[2];
    const float normBSq = accelB[0] * accelB[0] + accelB[1] * accelB[1] + accelB[2] * accelB[2];
    if (!std::isfinite(normASq) || !std::isfinite(normBSq) || normASq <= 1.0e-6f || normBSq <= 1.0e-6f) {
        return false;
    }
    const float invNormA = 1.0f / sqrtf(normASq);
    const float invNormB = 1.0f / sqrtf(normBSq);
    const float dot = (accelA[0] * accelB[0] + accelA[1] * accelB[1] + accelA[2] * accelB[2]) * invNormA * invNormB;
    angleDeg = acosf(std::clamp(dot, -1.0f, 1.0f)) * (180.0f / 3.14159265358979323846f);
    return std::isfinite(angleDeg);
}

static bool TiltDifferenceDegFromYprDeg(const float yprA[3], const float yprB[3], float &differenceDeg) {
    float gravityA[3] = {0.0f, 0.0f, 0.0f};
    float gravityB[3] = {0.0f, 0.0f, 0.0f};
    if (!GravityVectorFromYprDeg(yprA, gravityA) || !GravityVectorFromYprDeg(yprB, gravityB)) {
        return false;
    }
    const float dot = gravityA[0] * gravityB[0] + gravityA[1] * gravityB[1] + gravityA[2] * gravityB[2];
    differenceDeg = acosf(math_utils::Clamp(dot, -1.0f, 1.0f)) * (180.0f / 3.14159265358979323846f);
    return true;
}

static bool TiltDifferenceDegFromQuaternions(const float quaternionA[4],
                                             const float quaternionB[4],
                                             float &differenceDeg) {
    float gravityA[3] = {0.0f, 0.0f, 0.0f};
    float gravityB[3] = {0.0f, 0.0f, 0.0f};
    if (!GravityVectorFromQuaternion(quaternionA, gravityA) ||
        !GravityVectorFromQuaternion(quaternionB, gravityB)) {
        return false;
    }
    const float dot = gravityA[0] * gravityB[0] + gravityA[1] * gravityB[1] + gravityA[2] * gravityB[2];
    differenceDeg = acosf(math_utils::Clamp(dot, -1.0f, 1.0f)) * (180.0f / 3.14159265358979323846f);
    return true;
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
                                    const float accelB[3],
                                    const float gyroB[3]) {
    const float accelTrust = DescendingTrust(VectorDiffNorm3(accelA, accelB),
                                             kCrossCheckAccelFullTrustMps2,
                                             kCrossCheckAccelZeroTrustMps2);
    const float gyroTrust = DescendingTrust(VectorDiffNorm3(gyroA, gyroB),
                                            kCrossCheckGyroFullTrustRadPerSec,
                                            kCrossCheckGyroZeroTrustRadPerSec);
    return std::max(0.0f, std::min(1.0f, std::min(accelTrust, gyroTrust)));
}

static float ApplyTiltTrust(float baseTrust, bool hasTiltDifference, float tiltDifferenceDeg) {
    if (!hasTiltDifference) {
        return baseTrust;
    }
    const float tiltTrust = DescendingTrust(tiltDifferenceDeg,
                                            kCrossCheckQuaternionFullTrustDeg,
                                            kCrossCheckQuaternionZeroTrustDeg);
    return std::max(0.0f, std::min(1.0f, std::min(baseTrust, tiltTrust)));
}

static float ComputeTiltOnlyTrust(bool hasTiltDifference, float tiltDifferenceDeg) {
    if (!hasTiltDifference) {
        return 0.0f;
    }
    return DescendingTrust(tiltDifferenceDeg,
                           kCrossCheckQuaternionFullTrustDeg,
                           kCrossCheckQuaternionZeroTrustDeg);
}

static bool ComputeIcmLsmTiltDifference(const SensorData &icmData,
                                        const Icm20948Diagnostics &icmDiagnostics,
                                        const SensorData &lsmData,
                                        const Lsm9ds1Diagnostics &lsmDiagnostics,
                                        float &differenceDeg) {
    if (icmDiagnostics.hasBootstrapYpr && lsmDiagnostics.hasBootstrapYpr) {
        return TiltDifferenceDegFromYprDeg(icmDiagnostics.bootstrapYprDeg, lsmDiagnostics.bootstrapYprDeg, differenceDeg);
    }
    if (icmData.hasIcmQuaternion && lsmData.hasIcmQuaternion) {
        return TiltDifferenceDegFromQuaternions(icmData.icmQuaternion, lsmData.icmQuaternion, differenceDeg);
    }
    return false;
}

static bool ComputeBnoIcmTiltDifference(const BnoSample &bnoData,
                                        const BnoDiagnostics &bnoDiagnostics,
                                        const SensorData &icmData,
                                        const Icm20948Diagnostics &icmDiagnostics,
                                        float &differenceDeg) {
    if (bnoDiagnostics.hasBootstrapYpr && icmDiagnostics.hasBootstrapYpr) {
        return TiltDifferenceDegFromYprDeg(bnoDiagnostics.bootstrapYprDeg, icmDiagnostics.bootstrapYprDeg, differenceDeg);
    }
    if (bnoData.hasQuaternion && icmData.hasIcmQuaternion) {
        return TiltDifferenceDegFromQuaternions(bnoData.quaternion, icmData.icmQuaternion, differenceDeg);
    }
    return false;
}

static bool ComputeBnoLsmTiltDifference(const BnoSample &bnoData,
                                        const BnoDiagnostics &bnoDiagnostics,
                                        const SensorData &lsmData,
                                        const Lsm9ds1Diagnostics &lsmDiagnostics,
                                        float &differenceDeg) {
    if (bnoDiagnostics.hasBootstrapYpr && lsmDiagnostics.hasBootstrapYpr) {
        return TiltDifferenceDegFromYprDeg(bnoDiagnostics.bootstrapYprDeg, lsmDiagnostics.bootstrapYprDeg, differenceDeg);
    }
    if (bnoData.hasQuaternion && lsmData.hasIcmQuaternion) {
        return TiltDifferenceDegFromQuaternions(bnoData.quaternion, lsmData.icmQuaternion, differenceDeg);
    }
    return false;
}

static float ComputeIcmPulseCrossCheckTrust(const SensorData &icmData, const SensorData &pulseData) {
    const float baseTrust = ComputeCrossCheckTrust(icmData.accelICM,
                                                   icmData.gyro,
                                                   pulseData.accelPulse,
                                                   pulseData.gyroPulse);
    float tiltDifferenceDeg = 0.0f;
    const bool hasTiltDifference =
        icmData.hasIcmQuaternion &&
        pulseData.hasPulseQuaternion &&
        TiltDifferenceDegFromQuaternions(icmData.icmQuaternion, pulseData.quaternionPulse, tiltDifferenceDeg);
    return ApplyTiltTrust(baseTrust, hasTiltDifference, tiltDifferenceDeg);
}

static float ComputeLsmPulseCrossCheckTrust(const SensorData &lsmData, const SensorData &pulseData) {
    const float baseTrust = ComputeCrossCheckTrust(lsmData.accelICM,
                                                   lsmData.gyro,
                                                   pulseData.accelPulse,
                                                   pulseData.gyroPulse);
    float tiltDifferenceDeg = 0.0f;
    const bool hasTiltDifference =
        lsmData.hasIcmQuaternion &&
        pulseData.hasPulseQuaternion &&
        TiltDifferenceDegFromQuaternions(lsmData.icmQuaternion, pulseData.quaternionPulse, tiltDifferenceDeg);
    return ApplyTiltTrust(baseTrust, hasTiltDifference, tiltDifferenceDeg);
}

static float ComputeBnoPulseCrossCheckTrust(const BnoSample &bnoData, const SensorData &pulseData) {
    const float baseTrust = ComputeCrossCheckTrust(bnoData.accel,
                                                   bnoData.gyro,
                                                   pulseData.accelPulse,
                                                   pulseData.gyroPulse);
    float tiltDifferenceDeg = 0.0f;
    const bool hasTiltDifference =
        bnoData.hasQuaternion &&
        pulseData.hasPulseQuaternion &&
        TiltDifferenceDegFromQuaternions(bnoData.quaternion, pulseData.quaternionPulse, tiltDifferenceDeg);
    return ApplyTiltTrust(baseTrust, hasTiltDifference, tiltDifferenceDeg);
}

static void UpdatePairComparisonStats(SensorComparisonStats::PairStats &stats,
                                      const float accelA[3],
                                      const float gyroA[3],
                                      const float accelB[3],
                                      const float gyroB[3],
                                      const bool hasTiltDifference,
                                      float tiltDifferenceDeg) {
    ++stats.samples;
    stats.maxAccelDiffMps2 = std::max(stats.maxAccelDiffMps2, VectorDiffNorm3(accelA, accelB));
    stats.maxGyroDiffRadPerSec = std::max(stats.maxGyroDiffRadPerSec, VectorDiffNorm3(gyroA, gyroB));
    if (hasTiltDifference) {
        stats.maxQuaternionAngleDeg = std::max(stats.maxQuaternionAngleDeg, tiltDifferenceDeg);
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

static void CopyBnoFields(SensorData &dst, const BnoSample &src) {
    for (int i = 0; i < 3; ++i) {
        dst.accelBNO[i] = src.hasAccel ? src.accel[i] : 0.0f;
        dst.gyroBNO[i] = src.hasGyro ? src.gyro[i] : 0.0f;
    }
    for (int i = 0; i < 4; ++i) {
        dst.quaternionBNO[i] = src.hasQuaternion ? src.quaternion[i] : (i == 0 ? 1.0f : 0.0f);
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

static void CopyPulseFields(SensorData &dst, const SensorData &src) {
    for (int i = 0; i < 3; ++i) {
        dst.accelPulse[i] = src.accelPulse[i];
        dst.gyroPulse[i] = src.gyroPulse[i];
        dst.pulseYprDeg[i] = src.pulseYprDeg[i];
    }
    for (int i = 0; i < 4; ++i) {
        dst.quaternionPulse[i] = src.quaternionPulse[i];
    }
    dst.hasPulseQuaternion = src.hasPulseQuaternion;
    dst.hasPulseYpr = src.hasPulseYpr;
}

static void SetMainQuaternion(SensorData &data, const float quaternion[4], MainQuaternionSource source) {
    float tiltQuaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    if (!TiltQuaternionFromQuaternion(quaternion, tiltQuaternion)) {
        data.hasQuaternion = false;
        data.mainQuaternionSource = static_cast<uint8_t>(MainQuaternionSource::None);
        return;
    }
    data.quaternion[0] = tiltQuaternion[0];
    data.quaternion[1] = tiltQuaternion[1];
    data.quaternion[2] = tiltQuaternion[2];
    data.quaternion[3] = tiltQuaternion[3];
    data.hasQuaternion = true;
    data.mainQuaternionSource = static_cast<uint8_t>(source);
    for (int i = 0; i < 4; ++i) {
        g_lastMainQuaternion[i] = tiltQuaternion[i];
    }
    g_haveMainQuaternionReference = true;
}

static void SetMainQuaternion(SensorData &data, const math_utils::Quaternion &quaternion, MainQuaternionSource source) {
    const float values[4] = {quaternion.w, quaternion.x, quaternion.y, quaternion.z};
    SetMainQuaternion(data, values, source);
}

/// Acquires one sensor sample from replay or live hardware.
///
/// The live path aliases the active critical attitude source into the main quaternion
/// field while keeping comparison rails isolated from the estimator path.
static bool BnoComparableWithFastRail(uint32_t nowUs,
                                      const BnoSample &bnoData,
                                      uint32_t otherSampleMicros,
                                      uint32_t otherMaxAgeUs,
                                      bool requireQuaternion = false) {
    if (!bnoData.hasAccel || !bnoData.hasGyro) {
        return false;
    }
    if (requireQuaternion && !bnoData.hasQuaternion) {
        return false;
    }
    return SamplesComparable(nowUs,
                             bnoData.accelMicros,
                             kCrossCheckBnoSampleMaxAgeUs,
                             otherSampleMicros,
                             otherMaxAgeUs,
                             kCrossCheckBnoPairMaxSkewUs) &&
           SamplesComparable(nowUs,
                             bnoData.gyroMicros,
                             kCrossCheckBnoSampleMaxAgeUs,
                             otherSampleMicros,
                             otherMaxAgeUs,
                             kCrossCheckBnoPairMaxSkewUs) &&
           (!requireQuaternion ||
            SamplesComparable(nowUs,
                              bnoData.quaternionMicros,
                              kCrossCheckBnoSampleMaxAgeUs,
                              otherSampleMicros,
                              otherMaxAgeUs,
                              kCrossCheckBnoPairMaxSkewUs));
}

static bool AcquireSensorData(SensorData &data) {
    if (g_csvReplay.enabled) {
        return CsvReplayNextSample(data);
    }
    if (kPulseEnabled) {
        Ellipse20SensorSetCrossCheckTrust(g_pulseCrossCheckTrust);
    }
    Icm20948SensorSetCrossCheckTrust(g_icmLsmCrossCheckTrust);
    if (kLsmEnabled) {
        Lsm9ds1SensorSetCrossCheckTrust(g_icmLsmCrossCheckTrust);
    }
    SensorData pulseData;
    const bool hasPulseFresh = kPulseEnabled ? Ellipse20SensorAcquire(pulseData) : false;
    const Ellipse20Diagnostics pulseDiagnostics =
        kPulseEnabled ? Ellipse20SensorGetDiagnostics() : Ellipse20Diagnostics{};
    if (pulseDiagnostics.hasImu || pulseDiagnostics.hasQuaternion || pulseDiagnostics.hasYpr) {
        CopyPulseFields(data, pulseData);
    }
    const bool hasBnoImu = kBnoEnabled ? BnoSensorAcquire(data) : false;
    const BnoSample bnoData = kBnoEnabled ? BnoSensorGetSample() : BnoSample{};
    const BnoDiagnostics bnoDiagnostics = kBnoEnabled ? BnoSensorGetDiagnostics() : BnoDiagnostics{};
    const bool hasIcmImu = Icm20948SensorAcquire(data);
    const Icm20948Diagnostics icmDiagnostics = Icm20948SensorGetDiagnostics();
    if (kWt901Enabled) {
        Wt901SensorAcquire();
    }
    SensorData lsmData;
    const bool hasLsmImu = kLsmEnabled ? Lsm9ds1SensorAcquire(lsmData) : false;
    const Lsm9ds1Diagnostics lsmDiagnostics = kLsmEnabled ? Lsm9ds1SensorGetDiagnostics() : Lsm9ds1Diagnostics{};
    data.icmSampleFresh = hasIcmImu && icmDiagnostics.lastAcquireFresh;
    data.lsmSampleFresh = hasLsmImu && lsmDiagnostics.lastAcquireFresh;
    const uint32_t comparisonNowUs = micros();
    if (bnoData.hasAccel || bnoData.hasGyro || bnoData.hasQuaternion) {
        CopyBnoFields(data, bnoData);
    }
    if (hasLsmImu) {
        CopyLsmFields(data, lsmData);
    }
    PreferTiltOnlyQuaternion(data.icmQuaternion,
                             data.hasIcmQuaternion,
                             icmDiagnostics.bootstrapYprDeg,
                             icmDiagnostics.hasBootstrapYpr);
    PreferTiltOnlyQuaternion(lsmData.icmQuaternion,
                             lsmData.hasIcmQuaternion,
                             lsmDiagnostics.bootstrapYprDeg,
                             lsmDiagnostics.hasBootstrapYpr);
    PreferTiltOnlyQuaternion(data.quaternionBNO,
                             data.hasBnoQuaternion,
                             bnoDiagnostics.bootstrapYprDeg,
                             bnoDiagnostics.hasBootstrapYpr);
    PreferTiltOnlyQuaternion(data.quaternionPulse,
                             data.hasPulseQuaternion,
                             nullptr,
                             false);
    ++g_sensorAcquireStats.loops;
    if (hasBnoImu) {
        ++g_sensorAcquireStats.bnoHits;
    }
    if (hasPulseFresh) {
        ++g_sensorAcquireStats.pulseHits;
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
    bool updatedBnoReferenceTrust = false;
    float bnoReferenceTargetTrust = 1.0f;
    // Track ICM/LSM trust target - influenced by ICM-LSM comparison AND Pulse comparisons.
    float icmLsmTargetTrust = 1.0f;
    bool updatedIcmLsmTrust = false;
    const bool compareIcmLsm =
        hasIcmImu && hasLsmImu &&
        SamplesComparable(comparisonNowUs,
                          icmDiagnostics.lastSampleMicros,
                          kCrossCheckFastSampleMaxAgeUs,
                          lsmDiagnostics.lastSampleMicros,
                          kCrossCheckFastSampleMaxAgeUs,
                          kCrossCheckFastPairMaxSkewUs);
    if (compareIcmLsm) {
        float tiltDifferenceDeg = 0.0f;
        const bool hasTiltDifference =
            ComputeIcmLsmTiltDifference(data, icmDiagnostics, lsmData, lsmDiagnostics, tiltDifferenceDeg);
        UpdatePairComparisonStats(g_sensorComparisonStats.icmLsm,
                                  data.accelICM,
                                  data.gyro,
                                  lsmData.accelICM,
                                  lsmData.gyro,
                                  hasTiltDifference,
                                  tiltDifferenceDeg);
        const float trust = hasTiltDifference
                                ? ComputeTiltOnlyTrust(hasTiltDifference, tiltDifferenceDeg)
                                : ComputeCrossCheckTrust(data.accelICM,
                                                         data.gyro,
                                                         lsmData.accelICM,
                                                         lsmData.gyro);
        icmLsmTargetTrust = std::min(icmLsmTargetTrust, trust);
        updatedIcmLsmTrust = true;
    }
    const bool compareBnoIcm =
        hasBnoImu && hasIcmImu &&
        BnoComparableWithFastRail(comparisonNowUs,
                                  bnoData,
                                  icmDiagnostics.lastSampleMicros,
                                  kCrossCheckFastSampleMaxAgeUs,
                                  false);
    if (compareBnoIcm) {
        float tiltDifferenceDeg = 0.0f;
        const bool hasTiltDifference =
            ComputeBnoIcmTiltDifference(bnoData, bnoDiagnostics, data, icmDiagnostics, tiltDifferenceDeg);
        UpdatePairComparisonStats(g_sensorComparisonStats.icmBno,
                                  data.accelICM,
                                  data.gyro,
                                  bnoData.accel,
                                  bnoData.gyro,
                                  hasTiltDifference,
                                  tiltDifferenceDeg);
        const float trust = hasTiltDifference
                                ? ComputeTiltOnlyTrust(hasTiltDifference, tiltDifferenceDeg)
                                : ComputeCrossCheckTrust(data.accelICM,
                                                         data.gyro,
                                                         bnoData.accel,
                                                         bnoData.gyro);
        bnoReferenceTargetTrust = std::min(bnoReferenceTargetTrust, trust);
        updatedBnoReferenceTrust = true;
    }
    const bool compareBnoLsm =
        hasBnoImu && hasLsmImu &&
        BnoComparableWithFastRail(comparisonNowUs,
                                  bnoData,
                                  lsmDiagnostics.lastSampleMicros,
                                  kCrossCheckFastSampleMaxAgeUs,
                                  false);
    if (compareBnoLsm) {
        float tiltDifferenceDeg = 0.0f;
        const bool hasTiltDifference =
            ComputeBnoLsmTiltDifference(bnoData, bnoDiagnostics, lsmData, lsmDiagnostics, tiltDifferenceDeg);
        UpdatePairComparisonStats(g_sensorComparisonStats.lsmBno,
                                  lsmData.accelICM,
                                  lsmData.gyro,
                                  bnoData.accel,
                                  bnoData.gyro,
                                  hasTiltDifference,
                                  tiltDifferenceDeg);
        const float trust = hasTiltDifference
                                ? ComputeTiltOnlyTrust(hasTiltDifference, tiltDifferenceDeg)
                                : ComputeCrossCheckTrust(lsmData.accelICM,
                                                         lsmData.gyro,
                                                         bnoData.accel,
                                                         bnoData.gyro);
        bnoReferenceTargetTrust = std::min(bnoReferenceTargetTrust, trust);
        updatedBnoReferenceTrust = true;
    }
    float pulseTargetTrust = 1.0f;
    bool updatedPulseTrust = false;
    const bool compareIcmPulse =
        data.hasIcmQuaternion && data.hasPulseQuaternion &&
        SamplesComparable(comparisonNowUs,
                          icmDiagnostics.lastSampleMicros,
                          kCrossCheckFastSampleMaxAgeUs,
                          pulseDiagnostics.lastSampleMicros,
                          kPulseSampleMaxAgeUs,
                          kCrossCheckBnoPairMaxSkewUs);
    if (compareIcmPulse) {
        float tiltDifferenceDeg = 0.0f;
        const bool hasTiltDifference =
            data.hasIcmQuaternion &&
            pulseData.hasPulseQuaternion &&
            TiltDifferenceDegFromQuaternions(data.icmQuaternion, pulseData.quaternionPulse, tiltDifferenceDeg);
        UpdatePairComparisonStats(g_sensorComparisonStats.icmPulse,
                                  data.accelICM,
                                  data.gyro,
                                  pulseData.accelPulse,
                                  pulseData.gyroPulse,
                                  hasTiltDifference,
                                  tiltDifferenceDeg);
        const float icmPulseTrust = ComputeIcmPulseCrossCheckTrust(data, pulseData);
        pulseTargetTrust = std::min(pulseTargetTrust, icmPulseTrust);
        // Bidirectional: Pulse disagreement also affects ICM/LSM trust.
        icmLsmTargetTrust = std::min(icmLsmTargetTrust, icmPulseTrust);
        updatedPulseTrust = true;
        updatedIcmLsmTrust = true;
    }
    const bool compareLsmPulse =
        lsmData.hasIcmQuaternion && data.hasPulseQuaternion &&
        SamplesComparable(comparisonNowUs,
                          lsmDiagnostics.lastSampleMicros,
                          kCrossCheckFastSampleMaxAgeUs,
                          pulseDiagnostics.lastSampleMicros,
                          kPulseSampleMaxAgeUs,
                          kCrossCheckBnoPairMaxSkewUs);
    if (compareLsmPulse) {
        float tiltDifferenceDeg = 0.0f;
        const bool hasTiltDifference =
            lsmData.hasIcmQuaternion &&
            pulseData.hasPulseQuaternion &&
            TiltDifferenceDegFromQuaternions(lsmData.icmQuaternion, pulseData.quaternionPulse, tiltDifferenceDeg);
        UpdatePairComparisonStats(g_sensorComparisonStats.lsmPulse,
                                  lsmData.accelICM,
                                  lsmData.gyro,
                                  pulseData.accelPulse,
                                  pulseData.gyroPulse,
                                  hasTiltDifference,
                                  tiltDifferenceDeg);
        const float lsmPulseTrust = ComputeLsmPulseCrossCheckTrust(lsmData, pulseData);
        pulseTargetTrust = std::min(pulseTargetTrust, lsmPulseTrust);
        // Bidirectional: Pulse disagreement also affects ICM/LSM trust.
        icmLsmTargetTrust = std::min(icmLsmTargetTrust, lsmPulseTrust);
        updatedPulseTrust = true;
        updatedIcmLsmTrust = true;
    }
    const bool compareBnoPulse =
        hasBnoImu && data.hasPulseQuaternion &&
        BnoComparableWithFastRail(comparisonNowUs,
                                  bnoData,
                                  pulseDiagnostics.lastSampleMicros,
                                  kPulseSampleMaxAgeUs,
                                  true);
    if (compareBnoPulse) {
        float tiltDifferenceDeg = 0.0f;
        const bool hasTiltDifference =
            pulseData.hasPulseQuaternion &&
            bnoData.hasQuaternion &&
            TiltDifferenceDegFromQuaternions(pulseData.quaternionPulse, bnoData.quaternion, tiltDifferenceDeg);
        UpdatePairComparisonStats(g_sensorComparisonStats.pulseBno,
                                  pulseData.accelPulse,
                                  pulseData.gyroPulse,
                                  bnoData.accel,
                                  bnoData.gyro,
                                  hasTiltDifference,
                                  tiltDifferenceDeg);
        pulseTargetTrust = std::min(pulseTargetTrust, ComputeBnoPulseCrossCheckTrust(bnoData, pulseData));
        bnoReferenceTargetTrust = std::min(bnoReferenceTargetTrust, ComputeBnoPulseCrossCheckTrust(bnoData, pulseData));
        updatedPulseTrust = true;
        updatedBnoReferenceTrust = true;
    }
    // Apply ICM/LSM trust update (now includes bidirectional Pulse cross-checks).
    if (updatedIcmLsmTrust) {
        float blendRate = kCrossCheckTrustBlend;
        if (settings::ahrs::kEnableTrustHysteresis) {
            const bool isDecreasing = icmLsmTargetTrust < g_icmLsmCrossCheckTrust;
            if (isDecreasing != g_icmLsmTrustWasDecreasing) {
                blendRate *= settings::ahrs::kTrustHysteresisReductionFactor;
            }
            g_icmLsmTrustWasDecreasing = isDecreasing;
        }
        g_icmLsmPreviousTrust = g_icmLsmCrossCheckTrust;
        g_icmLsmCrossCheckTrust += blendRate * (icmLsmTargetTrust - g_icmLsmCrossCheckTrust);
    } else {
        // Trust recovery with hysteresis.
        float recoveryRate = kCrossCheckTrustRecoveryPerLoop;
        if (settings::ahrs::kEnableTrustHysteresis && g_icmLsmTrustWasDecreasing) {
            recoveryRate *= settings::ahrs::kTrustHysteresisReductionFactor;
        }
        g_icmLsmTrustWasDecreasing = false;
        g_icmLsmCrossCheckTrust = std::min(1.0f, g_icmLsmCrossCheckTrust + recoveryRate);
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
    if (updatedPulseTrust) {
        float blendRate = kCrossCheckTrustBlend;
        if (settings::ahrs::kEnableTrustHysteresis) {
            const bool isDecreasing = pulseTargetTrust < g_pulseCrossCheckTrust;
            if (isDecreasing != g_pulseTrustWasDecreasing) {
                blendRate *= settings::ahrs::kTrustHysteresisReductionFactor;
            }
            g_pulseTrustWasDecreasing = isDecreasing;
        }
        g_pulsePreviousTrust = g_pulseCrossCheckTrust;
        g_pulseCrossCheckTrust += blendRate * (pulseTargetTrust - g_pulseCrossCheckTrust);
    } else {
        float recoveryRate = kCrossCheckTrustRecoveryPerLoop;
        if (settings::ahrs::kEnableTrustHysteresis && g_pulseTrustWasDecreasing) {
            recoveryRate *= settings::ahrs::kTrustHysteresisReductionFactor;
        }
        g_pulseTrustWasDecreasing = false;
        g_pulseCrossCheckTrust = std::min(1.0f, g_pulseCrossCheckTrust + recoveryRate);
    }
    const bool icmRecent =
        SampleAgeWithinUs(comparisonNowUs, icmDiagnostics.lastSampleMicros, kCrossCheckFastSampleMaxAgeUs);
    const bool lsmRecent =
        SampleAgeWithinUs(comparisonNowUs, lsmDiagnostics.lastSampleMicros, kCrossCheckFastSampleMaxAgeUs);
    const bool bnoRecent =
        SampleAgeWithinUs(comparisonNowUs, bnoData.sampleMicros, kCrossCheckBnoSampleMaxAgeUs);
    const bool pulseRecent =
        SampleAgeWithinUs(comparisonNowUs, pulseDiagnostics.lastSampleMicros, kPulseSampleMaxAgeUs);
    g_pulseHealth = ResolveRailHealth(g_pulseHealth,
                                      pulseDiagnostics.initialized && pulseDiagnostics.hasImu,
                                      pulseDiagnostics.alignmentReady && data.hasPulseQuaternion,
                                      pulseRecent,
                                      compareIcmPulse || compareLsmPulse || compareBnoPulse,
                                      g_pulseCrossCheckTrust);
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
    float referenceGravity[3] = {0.0f, 0.0f, 0.0f};
    bool hasReferenceGravity = false;
    float icmTiltVector[3] = {0.0f, 0.0f, 0.0f};
    float lsmTiltVector[3] = {0.0f, 0.0f, 0.0f};
    float bnoTiltVector[3] = {0.0f, 0.0f, 0.0f};
    float pulseTiltVector[3] = {0.0f, 0.0f, 0.0f};
    const bool hasIcmTiltVector =
        RailTiltVectorForPhase(data.accelICM, true, data.icmQuaternion, data.hasIcmQuaternion, g_healthPhase, icmTiltVector);
    const bool hasLsmTiltVector =
        RailTiltVectorForPhase(lsmData.accelICM,
                               lsmDiagnostics.hasAccel,
                               lsmData.icmQuaternion,
                               lsmData.hasIcmQuaternion,
                               g_healthPhase,
                               lsmTiltVector);
    const bool hasBnoTiltVector =
        RailTiltVectorForPhase(bnoDiagnostics.accelBodyMps2,
                               bnoDiagnostics.hasAccel,
                               data.quaternionBNO,
                               data.hasBnoQuaternion,
                               g_healthPhase,
                               bnoTiltVector);
    const bool hasPulseTiltVector =
        RailTiltVectorForPhase(pulseData.accelPulse,
                               pulseDiagnostics.hasImu,
                               pulseData.quaternionPulse,
                               data.hasPulseQuaternion,
                               g_healthPhase,
                               pulseTiltVector);
    float fastConsensusGravity[3] = {0.0f, 0.0f, 0.0f};
    bool hasFastConsensus = false;
    bool fastConsensusReacquire = false;
    if (hasIcmTiltVector && hasLsmTiltVector) {
        fastConsensusGravity[0] = icmTiltVector[0] + lsmTiltVector[0];
        fastConsensusGravity[1] = icmTiltVector[1] + lsmTiltVector[1];
        fastConsensusGravity[2] = icmTiltVector[2] + lsmTiltVector[2];
        if (Normalize3(fastConsensusGravity)) {
            hasFastConsensus = true;
            const float fastAgreementDeg = AngleBetweenUnitVectorsDeg(icmTiltVector, lsmTiltVector);
            const float reacquireThresholdDeg =
                (g_healthPhase == FlightStatus::Ground) ? 6.0f :
                (g_healthPhase == FlightStatus::Burn) ? 10.0f : 8.0f;
            fastConsensusReacquire = fastAgreementDeg <= reacquireThresholdDeg;
        }
    } else if (g_healthPhase == FlightStatus::Ground) {
        if (hasIcmTiltVector) {
            fastConsensusGravity[0] = icmTiltVector[0];
            fastConsensusGravity[1] = icmTiltVector[1];
            fastConsensusGravity[2] = icmTiltVector[2];
            hasFastConsensus = true;
        } else if (hasLsmTiltVector) {
            fastConsensusGravity[0] = lsmTiltVector[0];
            fastConsensusGravity[1] = lsmTiltVector[1];
            fastConsensusGravity[2] = lsmTiltVector[2];
            hasFastConsensus = true;
        }
    }

    if (g_healthPhase == FlightStatus::Ground) {
        if (hasFastConsensus) {
            referenceGravity[0] = fastConsensusGravity[0];
            referenceGravity[1] = fastConsensusGravity[1];
            referenceGravity[2] = fastConsensusGravity[2];
            hasReferenceGravity = true;
        }
    } else if (g_healthPhase == FlightStatus::Burn) {
        if (hasFastConsensus) {
            referenceGravity[0] = fastConsensusGravity[0];
            referenceGravity[1] = fastConsensusGravity[1];
            referenceGravity[2] = fastConsensusGravity[2];
            hasReferenceGravity = true;
        }
    } else {
        hasReferenceGravity = g_haveMainQuaternionReference &&
                              GravityVectorFromQuaternion(g_lastMainQuaternion, referenceGravity);
        if (!hasReferenceGravity && hasFastConsensus) {
            referenceGravity[0] = fastConsensusGravity[0];
            referenceGravity[1] = fastConsensusGravity[1];
            referenceGravity[2] = fastConsensusGravity[2];
            hasReferenceGravity = true;
        }
    }

    g_icmRailTrust = UpdateRailTrust(g_icmRailState,
                                     hasIcmTiltVector,
                                     icmTiltVector,
                                     true,
                                     data.accelICM,
                                     icmDiagnostics.lastSampleMicros,
                                     hasReferenceGravity,
                                     referenceGravity,
                                     g_healthPhase);
    g_lsmRailTrust = UpdateRailTrust(g_lsmRailState,
                                     hasLsmTiltVector,
                                     lsmTiltVector,
                                     lsmDiagnostics.hasAccel,
                                     lsmData.accelICM,
                                     lsmDiagnostics.lastSampleMicros,
                                     hasReferenceGravity,
                                     referenceGravity,
                                     g_healthPhase);
    g_bnoRailTrust = UpdateRailTrust(g_bnoRailState,
                                     hasBnoTiltVector,
                                     bnoTiltVector,
                                     bnoDiagnostics.hasAccel,
                                     bnoDiagnostics.accelBodyMps2,
                                     bnoData.sampleMicros,
                                     hasReferenceGravity,
                                     referenceGravity,
                                     g_healthPhase);
    g_pulseRailTrust = UpdateRailTrust(g_pulseRailState,
                                       hasPulseTiltVector,
                                       pulseTiltVector,
                                       pulseDiagnostics.hasImu,
                                       pulseData.accelPulse,
                                       pulseDiagnostics.lastSampleMicros,
                                       hasReferenceGravity,
                                       referenceGravity,
                                       g_healthPhase);
    if (fastConsensusReacquire) {
        g_icmRailTrust = std::max(g_icmRailTrust, 0.98f);
        g_lsmRailTrust = std::max(g_lsmRailTrust, 0.98f);
    }

    g_icmHealth = ResolveRailHealth(g_icmHealth,
                                    icmDiagnostics.initialized,
                                    icmDiagnostics.alignmentReady && data.hasIcmQuaternion,
                                    icmRecent,
                                    hasReferenceGravity,
                                    g_icmRailTrust);
    g_lsmHealth = ResolveRailHealth(g_lsmHealth,
                                    lsmDiagnostics.initialized,
                                    lsmDiagnostics.alignmentReady && lsmData.hasIcmQuaternion,
                                    lsmRecent,
                                    hasReferenceGravity,
                                    g_lsmRailTrust);
    g_bnoHealth = ResolveRailHealth(g_bnoHealth,
                                    kBnoEnabled && bnoData.hasAccel && bnoData.hasGyro,
                                    bnoData.hasQuaternion,
                                    bnoRecent,
                                    hasReferenceGravity,
                                    g_bnoRailTrust);
    g_pulseHealth = ResolveRailHealth(g_pulseHealth,
                                      pulseDiagnostics.initialized && pulseDiagnostics.hasImu,
                                      pulseDiagnostics.alignmentReady && data.hasPulseQuaternion,
                                      pulseRecent,
                                      hasReferenceGravity,
                                      g_pulseRailTrust);

    g_icmLsmCrossCheckTrust = std::min(g_icmRailTrust, g_lsmRailTrust);
    g_bnoReferenceCrossCheckTrust = g_bnoRailTrust;
    g_pulseCrossCheckTrust = g_pulseRailTrust;
    Icm20948SensorSetCrossCheckTrust(g_icmRailTrust);
    if (kLsmEnabled) {
        Lsm9ds1SensorSetCrossCheckTrust(g_lsmRailTrust);
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
        const bool icmUsable = data.hasIcmQuaternion &&
                               g_icmHealth != SensorRailHealth::Unavailable &&
                               g_icmHealth != SensorRailHealth::Stale;
        const bool lsmUsable = lsmData.hasIcmQuaternion &&
                               g_lsmHealth != SensorRailHealth::Unavailable &&
                               g_lsmHealth != SensorRailHealth::Stale;
        math_utils::Quaternion blendedQuat = {1.0f, 0.0f, 0.0f, 0.0f};
        bool hasFastQuaternion = false;

        if (icmHealthy && lsmHealthy && g_icmLsmCrossCheckTrust >= settings::ahrs::kMinBlendTrust) {
            blendedQuat = math_utils::Normalize(math_utils::MakeQuaternion(
                data.icmQuaternion[0], data.icmQuaternion[1],
                data.icmQuaternion[2], data.icmQuaternion[3]));
            const math_utils::Quaternion lsmQuat = math_utils::Normalize(math_utils::MakeQuaternion(
                lsmData.icmQuaternion[0], lsmData.icmQuaternion[1],
                lsmData.icmQuaternion[2], lsmData.icmQuaternion[3]));
            blendedQuat = math_utils::Slerp(blendedQuat, lsmQuat, 0.5f);
            SetMainQuaternion(data, blendedQuat, MainQuaternionSource::Blended);
            hasFastQuaternion = true;
        } else if (icmHealthy) {
            SetMainQuaternion(data, data.icmQuaternion, MainQuaternionSource::Icm);
            hasFastQuaternion = true;
        } else if (lsmHealthy) {
            SetMainQuaternion(data, lsmData.icmQuaternion, MainQuaternionSource::Lsm);
            hasFastQuaternion = true;
        } else if (icmUsable && lsmUsable && g_icmLsmCrossCheckTrust >= settings::ahrs::kMinBlendTrust) {
            blendedQuat = math_utils::Normalize(math_utils::MakeQuaternion(
                data.icmQuaternion[0], data.icmQuaternion[1],
                data.icmQuaternion[2], data.icmQuaternion[3]));
            const math_utils::Quaternion lsmQuat = math_utils::Normalize(math_utils::MakeQuaternion(
                lsmData.icmQuaternion[0], lsmData.icmQuaternion[1],
                lsmData.icmQuaternion[2], lsmData.icmQuaternion[3]));
            blendedQuat = math_utils::Slerp(blendedQuat, lsmQuat, 0.5f);
            SetMainQuaternion(data, blendedQuat, MainQuaternionSource::Blended);
            hasFastQuaternion = true;
        } else if (icmUsable) {
            SetMainQuaternion(data, data.icmQuaternion, MainQuaternionSource::Icm);
            hasFastQuaternion = true;
        } else if (lsmUsable) {
            SetMainQuaternion(data, lsmData.icmQuaternion, MainQuaternionSource::Lsm);
            hasFastQuaternion = true;
        }

        if (!hasFastQuaternion && icmHealthy) {
            SetMainQuaternion(data, data.icmQuaternion, MainQuaternionSource::Icm);
        } else if (!hasFastQuaternion && lsmUsable) {
            SetMainQuaternion(data, lsmData.icmQuaternion, MainQuaternionSource::Lsm);
        }
    } else if (!data.hasQuaternion) {
        if (data.hasIcmQuaternion &&
                   g_icmHealth != SensorRailHealth::Unavailable &&
                   g_icmHealth != SensorRailHealth::Stale) {
            SetMainQuaternion(data, data.icmQuaternion, MainQuaternionSource::Icm);
        } else if (lsmData.hasIcmQuaternion &&
                   g_lsmHealth != SensorRailHealth::Unavailable &&
                   g_lsmHealth != SensorRailHealth::Stale) {
            SetMainQuaternion(data, lsmData.icmQuaternion, MainQuaternionSource::Lsm);
        }
    }
    const bool hasAltimeter = Bmp585SensorAcquire(data);
    data.baroSampleFresh = hasAltimeter;
    if (hasAltimeter) {
        ++g_sensorAcquireStats.bmpHits;
    }
    // Secondary barometer disabled for now.
    // const bool hasMs5611 = Ms5611SensorAcquire();
    const bool hasSensorData =
        data.icmSampleFresh || data.lsmSampleFresh || data.baroSampleFresh;
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
static bool g_autoActuationFirstMotionReleased = false;
static bool g_autoActuationSafetyLatched = false;
static PredictorHorizontalVelocityTracker g_actuationPredictorHorizontalVelocity;
static RuntimeSettings g_runtimeSettings = RuntimeSettingsDefaults();
static RuntimeSettingsStorageStatus g_runtimeSettingsStorageStatus;
static uint32_t g_runtimeSettingsRevision = 0;
static uint32_t g_runtimeSettingsLastRequestId = 0;
static uint8_t g_runtimeSettingsLastCommandResult = telemetry::kSettingsResultNone;
static bool g_setupComplete = false;

void HoldBootFlapCommand(float commandedAngleDeg, uint32_t durationMs) {
    const uint32_t startMs = millis();
    do {
        g_flapActuator.Update(millis(), commandedAngleDeg);
        delay(20);
    } while ((millis() - startMs) < durationMs);
}

void RunBootFlapExercise() {
    constexpr uint32_t kFullActuationHoldMs = 2000;
    constexpr uint32_t kIntermediateRetractHoldMs = 1000;
    constexpr uint32_t kHalfActuationHoldMs = 1000;
    constexpr float kHalfActuationDeg = kServoMaxActuationDeg * 0.5f;

    HoldBootFlapCommand(kServoMaxActuationDeg, kFullActuationHoldMs);
    HoldBootFlapCommand(0.0f, kIntermediateRetractHoldMs);
    HoldBootFlapCommand(kHalfActuationDeg, kHalfActuationHoldMs);
    HoldBootFlapCommand(0.0f, kIntermediateRetractHoldMs);
}

void ForceTeensyRebootIfSafe(const char *reason) {
    if (!g_setupComplete || flightComputer.Status() == FlightStatus::Ground) {
        ForceTeensyReboot(reason);
    }
    LOG_PRINT("[fatal] reboot suppressed outside ground/setup: ");
    LOG_PRINTLN(reason);
}

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

static bool ShouldBlockFirstAutoActuationMotion(const FilteredState &state,
                                                FlightStatus status,
                                                bool hasBaroAgl,
                                                float baroAltitudeAglMeters) {
    if (g_autoActuationFirstMotionReleased) {
        return false;
    }

    if (status != FlightStatus::Coast) {
        return true;
    }

    const double burnoutTime = flightComputer.BurnoutTime();
    if (burnoutTime <= 0.0) {
        return true;
    }

    const double timeSinceBurnout = static_cast<double>(state.time) - burnoutTime;
    if (timeSinceBurnout < static_cast<double>(settings::actuation::kPostBurnoutHoldoffSeconds)) {
        return true;
    }

    if (!hasBaroAgl) {
        return true;
    }

    const double altitudeAgreementError =
        std::fabs(static_cast<double>(state.position[2]) - static_cast<double>(baroAltitudeAglMeters));
    return altitudeAgreementError >
           static_cast<double>(settings::actuation::kFirstFlapBaroStateAgreementMeters);
}

static bool ShouldLatchAutoActuationSafety(const FilteredState &state,
                                           FlightStatus status,
                                           bool hasBaroAgl,
                                           float baroAltitudeAglMeters) {
    if (status != FlightStatus::Coast) {
        return false;
    }

    if (hasBaroAgl) {
        const double altitudeAgreementError =
            std::fabs(static_cast<double>(state.position[2]) - static_cast<double>(baroAltitudeAglMeters));
        if (!std::isfinite(altitudeAgreementError) ||
            altitudeAgreementError >
                static_cast<double>(settings::actuation::kCoastBaroStateAgreementMeters)) {
            return true;
        }
    }

    const double coastAccelerationLimit =
        static_cast<double>(settings::actuation::kCoastMaxUpwardAccelerationMps2);
    const double inertialAccelerationZ = static_cast<double>(state.inertialAcceleration[2]);
    const double filteredAccelerationZ = static_cast<double>(state.acceleration[2]);
    return !std::isfinite(inertialAccelerationZ) || !std::isfinite(filteredAccelerationZ) ||
           inertialAccelerationZ > coastAccelerationLimit || filteredAccelerationZ > coastAccelerationLimit;
}

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
static RetryState g_bnoRetry;
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

static void RetryServiceUntilReady(RetryState &retry,
                                   bool (*isReady)(),
                                   bool (*initializer)(),
                                   const char *serviceName,
                                   const char *waitingMessage,
                                   uint32_t resetAttempts = 0) {
    uint32_t lastWaitLogMs = 0;
    while (!isReady()) {
        const uint32_t nowMs = millis();
        BootLedSlowBlink();  // Keep LED blinking during retry loops
        ServiceRetry(nowMs, retry, isReady(), initializer, serviceName);
        if (!isReady() && resetAttempts > 0 && retry.attempts >= resetAttempts) {
            ForceTeensyReboot(serviceName);
        }
        if (!isReady() &&
            (lastWaitLogMs == 0 || (nowMs - lastWaitLogMs) >= settings::flight::kDebugHeartbeatIntervalMs)) {
            LogSetupCheckpoint(waitingMessage);
            lastWaitLogMs = nowMs;
        }
        if (!isReady()) {
            delay(10);
        }
    }
}

static void RetryCsvReplayUntilReady() {
    if (!kEnableCsvReplay) {
        return;
    }

    uint32_t lastWaitLogMs = 0;
    while (!g_csvReplay.enabled) {
        if (DataLoggerIsInitialized()) {
            CsvReplayInit();
        }
        if (!g_csvReplay.enabled) {
            const uint32_t nowMs = millis();
            if (lastWaitLogMs == 0 || (nowMs - lastWaitLogMs) >= settings::flight::kDebugHeartbeatIntervalMs) {
                LogSetupCheckpoint("CSV replay unavailable, retrying");
                lastWaitLogMs = nowMs;
            }
            delay(50);
        }
    }
}

static void RetryCfdTableUntilLoaded(CfdTableStorage *storage) {
    uint32_t lastWaitLogMs = 0;
    uint32_t attempts = 0;
    while (!storage->loaded) {
        ++attempts;
        BootLedSlowBlink();  // Keep LED blinking during retry loops
        if (LoadCfdTableDuringSetup(storage)) {
            return;
        }
        if (attempts >= kCriticalStartupResetAttempts) {
            ForceTeensyReboot("cfd_table");
        }
        const uint32_t nowMs = millis();
        if (lastWaitLogMs == 0 || (nowMs - lastWaitLogMs) >= settings::flight::kDebugHeartbeatIntervalMs) {
            LogSetupCheckpoint("CFD table unavailable, retrying");
            lastWaitLogMs = nowMs;
        }
        delay(kCfdStartupRetryDelayMs);
    }
}

/// Emits periodic loop and logger timing diagnostics, then resets the maxima.
static void LogTimingDiagnostics(uint32_t nowMs) {
    if ((nowMs - g_lastTimingLogMs) < kTimingLogIntervalMs) {
        return;
    }
    const uint32_t intervalMs = (g_lastTimingLogMs == 0) ? kTimingLogIntervalMs : (nowMs - g_lastTimingLogMs);
    const float intervalSec = static_cast<float>(intervalMs) * 1.0e-3f;
    g_lastTimingLogMs = nowMs;

    const BnoDiagnostics bno = kBnoEnabled ? BnoSensorGetDiagnostics() : BnoDiagnostics{};
    const Icm20948Diagnostics icm = Icm20948SensorGetDiagnostics();
    const Lsm9ds1Diagnostics lsm = kLsmEnabled ? Lsm9ds1SensorGetDiagnostics() : Lsm9ds1Diagnostics{};
    const Wt901Diagnostics wt = kWt901Enabled ? Wt901SensorGetDiagnostics() : Wt901Diagnostics{};
    auto printVec3 = [](const float values[3], uint8_t decimals) {
        LOG_PRINT("(");
        LOG_PRINT(values[0], decimals);
        LOG_PRINT(",");
        LOG_PRINT(values[1], decimals);
        LOG_PRINT(",");
        LOG_PRINT(values[2], decimals);
        LOG_PRINT(")");
    };
    auto printZenith = [&](const float accelBody[3]) {
        float zenithDeg = 0.0f;
        if (!ZenithDegFromAccelBody(accelBody, zenithDeg)) {
            LOG_PRINT("na");
            return;
        }
        LOG_PRINT(zenithDeg, 1);
    };
    auto printPairMetric = [&](const float accelA[3], bool validA, const float accelB[3], bool validB, bool zenithOnly) {
        if (!validA || !validB) {
            LOG_PRINT("na");
            return;
        }
        float valueDeg = 0.0f;
        const bool ok = zenithOnly ? ZenithDifferenceDegFromAccelBody(accelA, accelB, valueDeg)
                                   : AccelVectorAngleDeg(accelA, accelB, valueDeg);
        if (!ok) {
            LOG_PRINT("na");
            return;
        }
        LOG_PRINT(valueDeg, 1);
    };
    auto printHealth = [&](char label, SensorRailHealth health, float trust, bool enabled) {
        LOG_PRINT(label);
        LOG_PRINT(":");
        if (!enabled) {
            LOG_PRINT("-");
            return;
        }
        LOG_PRINT(HealthCode(health));
        LOG_PRINT("/");
        LOG_PRINT(trust, 2);
    };

    LOG_PRINT("[imu]");
    LOG_PRINT(" iacc=");
    printVec3(icm.accelBodyMps2, 2);
    LOG_PRINT(" bacc=");
    printVec3(bno.accelBodyMps2, 2);
    LOG_PRINT(" lacc=");
    printVec3(lsm.accelBodyMps2, 2);
    LOG_PRINT(" wacc=");
    if (wt.hasAccel) {
        printVec3(wt.accelBodyMps2, 2);
    } else {
        LOG_PRINT("na");
    }
    LOG_PRINT(" izen=");
    printZenith(icm.accelBodyMps2);
    LOG_PRINT(" bzen=");
    printZenith(bno.accelBodyMps2);
    LOG_PRINT(" lzen=");
    printZenith(lsm.accelBodyMps2);
    LOG_PRINT(" wzen=");
    if (wt.hasAccel) {
        printZenith(wt.accelBodyMps2);
    } else {
        LOG_PRINT("na");
    }
    LOG_PRINT(" zdif=(ib:");
    printPairMetric(icm.accelBodyMps2, true, bno.accelBodyMps2, bno.hasAccel, true);
    LOG_PRINT(",il:");
    printPairMetric(icm.accelBodyMps2, true, lsm.accelBodyMps2, lsm.hasAccel, true);
    LOG_PRINT(",bl:");
    printPairMetric(bno.accelBodyMps2, bno.hasAccel, lsm.accelBodyMps2, lsm.hasAccel, true);
    LOG_PRINT(",iw:");
    printPairMetric(icm.accelBodyMps2, true, wt.accelBodyMps2, wt.hasAccel, true);
    LOG_PRINT(",bw:");
    printPairMetric(bno.accelBodyMps2, bno.hasAccel, wt.accelBodyMps2, wt.hasAccel, true);
    LOG_PRINT(",lw:");
    printPairMetric(lsm.accelBodyMps2, lsm.hasAccel, wt.accelBodyMps2, wt.hasAccel, true);
    LOG_PRINT(")");
    LOG_PRINT(" adif=(ib:");
    printPairMetric(icm.accelBodyMps2, true, bno.accelBodyMps2, bno.hasAccel, false);
    LOG_PRINT(",il:");
    printPairMetric(icm.accelBodyMps2, true, lsm.accelBodyMps2, lsm.hasAccel, false);
    LOG_PRINT(",bl:");
    printPairMetric(bno.accelBodyMps2, bno.hasAccel, lsm.accelBodyMps2, lsm.hasAccel, false);
    LOG_PRINT(",iw:");
    printPairMetric(icm.accelBodyMps2, true, wt.accelBodyMps2, wt.hasAccel, false);
    LOG_PRINT(",bw:");
    printPairMetric(bno.accelBodyMps2, bno.hasAccel, wt.accelBodyMps2, wt.hasAccel, false);
    LOG_PRINT(",lw:");
    printPairMetric(lsm.accelBodyMps2, lsm.hasAccel, wt.accelBodyMps2, wt.hasAccel, false);
    LOG_PRINT(")");
    LOG_PRINT(" health=(");
    printHealth('i', g_icmHealth, g_icmRailTrust, true);
    LOG_PRINT(",");
    printHealth('l', g_lsmHealth, g_lsmRailTrust, kLsmEnabled);
    LOG_PRINT(",");
    printHealth('b', g_bnoHealth, g_bnoRailTrust, kBnoEnabled);
    LOG_PRINT(",");
    printHealth('p', g_pulseHealth, g_pulseRailTrust, kPulseEnabled);
    LOG_PRINT(")");
    LOG_PRINTLN("");

    const uint32_t freshEstimatorLoops = g_sensorAcquireStats.loops - g_sensorAcquireStats.noDataLoops;
    auto printCountRate = [&](const char *label, uint32_t count) {
        LOG_PRINT(label);
        LOG_PRINT(":");
        LOG_PRINT(count);
        LOG_PRINT("@");
        LOG_PRINT(intervalSec > 0.0f ? (static_cast<float>(count) / intervalSec) : 0.0f, 1);
    };

    LOG_PRINT("[rate]");
    LOG_PRINT(" dt=");
    LOG_PRINT(intervalSec, 2);
    LOG_PRINT("s");
    LOG_PRINT(" fresh=(");
    printCountRate("i", g_sensorAcquireStats.icmFreshHits);
    LOG_PRINT(",");
    printCountRate("l", g_sensorAcquireStats.lsmFreshHits);
    LOG_PRINT(",");
    printCountRate("b", g_sensorAcquireStats.bmpHits);
    LOG_PRINT(",");
    printCountRate("est", freshEstimatorLoops);
    LOG_PRINT(")");
    LOG_PRINT(" cached=(");
    LOG_PRINT("i:");
    LOG_PRINT(g_sensorAcquireStats.icmCachedHits);
    LOG_PRINT(",");
    LOG_PRINT("l:");
    LOG_PRINT(g_sensorAcquireStats.lsmCachedHits);
    LOG_PRINT(")");
    LOG_PRINT(" loops=");
    LOG_PRINT(g_sensorAcquireStats.loops);
    LOG_PRINT(" nodata=");
    LOG_PRINTLN(g_sensorAcquireStats.noDataLoops);

    g_timingStats = TimingStats{};
    g_sensorAcquireStats = SensorAcquireStats{};
    g_sensorComparisonStats = SensorComparisonStats{};
}

/// Converts a telemetry packet payload into the runtime settings layout.
static RuntimeSettings RuntimeSettingsFromPayload(const telemetry::RuntimeSettingsPayloadV1 &payload) {
    RuntimeSettings settings;
    settings.environment.groundTemperatureF = static_cast<float>(payload.groundTemperatureF);
    settings.environment.seaLevelPressureHpa = static_cast<float>(payload.seaLevelPressureHpa);
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

static void ApplyRuntimeSettingsToBarometers() {
    Bmp585SensorSetSeaLevelPressureHpa(g_runtimeSettings.environment.seaLevelPressureHpa);
    Ms5611SensorSetSeaLevelPressureHpa(g_runtimeSettings.environment.seaLevelPressureHpa);
}

/// Rebinds the live predictor stack to the currently active runtime settings.
static void ApplyRuntimeSettingsToPredictors() {
    ApplyRuntimeSettingsToBarometers();
    g_actuationEnvironment.Configure(g_runtimeSettings.environment);
    g_actuationPredictor.SetEnvironment(g_actuationEnvironment);
    g_actuationPredictor.SetVehicleParameters(g_runtimeSettings.vehicle);
    g_actuationPredictor.SetForceTable(g_cfdTable.loaded ? &g_cfdTable.table : nullptr);
    g_actuationPredictor.SetMaxIntegrationSteps(settings::actuation::kActuationPredictorMaxSteps);
    g_actuationPredictor.ResetAxialDragScale();
    g_actuationPredictorReady = g_cfdTable.loaded;
    g_actuationHasLastZenithSample = false;
    g_actuationHasLastControlUpdate = false;
    g_autoActuationFirstMotionReleased = false;
    g_autoActuationSafetyLatched = false;
    ResetPredictorHorizontalVelocityTracker(g_actuationPredictorHorizontalVelocity);
    flightComputer.ReconfigurePredictor(g_runtimeSettings.environment,
                                        g_runtimeSettings.vehicle,
                                        g_cfdTable.loaded ? &g_cfdTable.table : nullptr);
    if (flightComputer.Status() == FlightStatus::Ground && !g_csvReplay.enabled) {
        flightComputer.ResetGroundReference();
        g_hasPadAltitude = false;
        g_padAltitudeFeet = 0.0f;
    }
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
    ApplyRuntimeSettingsToBarometers();
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
    const double clampedZenith = SanitizePredictorZenithRadians(static_cast<double>(state.zenith));
    // A stale or inactive control window is forced back to a simpler
    // vertical-only predictor seed.
    const bool useHorizontalModel = canControl && freshSeedSample;
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
    const double timeToApogeeSeconds =
        (g_actuationPredictorReady && predictorState.verticalVelocity > 0.0)
            ? g_actuationPredictor.EstimateTimeToApogee(predictorState)
            : 0.0;
    if (telemetry != nullptr) {
        telemetry->timeToApogeeS = static_cast<float>(std::max(0.0, timeToApogeeSeconds));
    }
    if (!canControl) {
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
        const double timeToApogee = std::max(0.0, timeToApogeeSeconds);

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
        (!g_csvReplay.enabled && ((kBnoEnabled && !BnoSensorIsInitialized()) || !Bmp585SensorIsInitialized()));
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
/// Setup blocks on required startup dependencies so the system does not proceed
/// until logging, replay/CFD assets, and critical sensors are available.
void setup() {
    // Initialize boot LED immediately - slow blink during boot
    BootLedBegin();
    BootLedSlowBlink();

    LOG_BEGIN(115200);
    // Wait for serial with timeout - don't block forever on flight without USB
    constexpr uint32_t kSerialTimeoutMs = 2000;
    const uint32_t serialWaitStart = millis();
    while (!Serial && (millis() - serialWaitStart) < kSerialTimeoutMs) {
        BootLedSlowBlink();
        delay(10);
    }

    LogSetupCheckpoint("boot");
    BootLedSlowBlink();
    LogSetupCheckpoint("preparing SPI buses");
    PrepareSpiBusesForStartup();
    LogSetupCheckpoint("SPI buses ready");
    LogSetupCheckpoint("attaching flap servos");
    g_flapActuator.Begin();
    LogSetupCheckpoint("flap servos initialized");
    // LogSetupCheckpoint("running flap boot exercise");
    // RunBootFlapExercise();
    // LogSetupCheckpoint("flap boot exercise complete");

    LogSetupCheckpoint("starting data logger init");
    BootLedSlowBlink();
    RetryServiceUntilReady(g_dataLoggerRetry,
                           &DataLoggerIsInitialized,
                           &DataLoggerBegin,
                           "data_logger",
                           "data logger unavailable, retrying");
    LogSetupCheckpoint("data logger init complete");
    BootLedSlowBlink();

    LogSetupCheckpoint("checking CSV replay");
    if (kEnableCsvReplay) {
        RetryCsvReplayUntilReady();
        LogSetupCheckpoint("CSV replay active");
    } else {
        LogSetupCheckpoint("CSV replay disabled");
    }

    if (!g_csvReplay.enabled) {
        if (kBnoEnabled) {
            LOG_PRINT("Configured BNO sensor: ");
            LOG_PRINT(BnoSensorModelName());
            LOG_PRINT(" over ");
            LOG_PRINTLN(BnoSensorTransportName());
            LogSetupCheckpoint("starting BNO init");
            BootLedSlowBlink();
            RetryServiceUntilReady(g_bnoRetry,
                                   &BnoSensorIsInitialized,
                                   &StartBnoDuringSetup,
                                   "bno",
                                   "BNO unavailable, retrying",
                                   kCriticalStartupResetAttempts);
            LogSetupCheckpoint("BNO init complete");
            BootLedSlowBlink();
        } else {
            LogSetupCheckpoint("BNO disabled");
        }

        if (kPulseEnabled) {
            LogSetupCheckpoint("starting Pulse20 init");
            BootLedSlowBlink();
            StartPulseDuringSetup();
            LogSetupCheckpoint(Ellipse20SensorIsInitialized() ? "Pulse20 init complete" : "Pulse20 unavailable");
            BootLedSlowBlink();
        } else {
            LogSetupCheckpoint("Pulse20 disabled");
        }

        if (kWt901Enabled) {
            LogSetupCheckpoint("starting WT901 init");
            BootLedSlowBlink();
            Wt901SensorBegin();
            LogSetupCheckpoint(Wt901SensorIsInitialized() ? "WT901 init complete" : "WT901 unavailable");
            BootLedSlowBlink();
        } else {
            LogSetupCheckpoint("WT901 disabled");
        }

        LogSetupCheckpoint("starting ICM-20948 init");
        BootLedSlowBlink();
        ServiceRetry(millis(), g_icmRetry, Icm20948SensorIsInitialized(), &Icm20948SensorBegin, "icm20948");
        LogSetupCheckpoint(Icm20948SensorIsInitialized() ? "ICM-20948 init complete" : "ICM-20948 unavailable");
        BootLedSlowBlink();

        if (kLsmEnabled) {
            LogSetupCheckpoint("starting LSM9DS1 init");
            BootLedSlowBlink();
            Lsm9ds1SensorBegin();
            LogSetupCheckpoint(Lsm9ds1SensorIsInitialized() ? "LSM9DS1 init complete" : "LSM9DS1 unavailable");
            BootLedSlowBlink();
        } else {
            LogSetupCheckpoint("LSM9DS1 disabled");
        }

        LogSetupCheckpoint("starting BMP585 init");
        BootLedSlowBlink();
        RetryServiceUntilReady(g_bmpRetry,
                               &Bmp585SensorIsInitialized,
                               &Bmp585SensorBegin,
                               "bmp585",
                               "BMP585 unavailable, retrying",
                               kCriticalStartupResetAttempts);
        LogSetupCheckpoint("BMP585 init complete");
        BootLedSlowBlink();

    }

    LogSetupCheckpoint("loading CFD table");
    BootLedSlowBlink();
    RetryCfdTableUntilLoaded(&g_cfdTable);
    LogSetupCheckpoint("CFD table loaded");
    BootLedSlowBlink();

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
    g_autoActuationFirstMotionReleased = false;
    g_autoActuationSafetyLatched = false;
    ResetPredictorHorizontalVelocityTracker(g_actuationPredictorHorizontalVelocity);
    g_lastTimingLogMs = 0;
    g_timingStats = TimingStats{};
    LogSetupCheckpoint("playing startup buzzer");
    PlayStartupMarch();
    g_setupComplete = true;
    LogSetupCheckpoint("setup complete");

    // Boot successful - turn LED solid on
    BootLedSolidOn();
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
        if (kBnoEnabled) {
            ServiceRetry(nowMs, g_bnoRetry, BnoSensorIsInitialized(), &StartBnoDuringSetup, "bno");
            if (!BnoSensorIsInitialized() && g_bnoRetry.attempts >= kCriticalStartupResetAttempts) {
                ForceTeensyRebootIfSafe("bno");
            }
        }
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
        if (kWt901Enabled && !Wt901SensorIsInitialized()) {
            Wt901SensorBegin();
        }
        if (kLsmEnabled && !Lsm9ds1SensorIsInitialized()) {
            Lsm9ds1SensorBegin();
        }
        ServiceRetry(nowMs, g_bmpRetry, Bmp585SensorIsInitialized(), &Bmp585SensorBegin, "bmp585");
        if (!Bmp585SensorIsInitialized() && g_bmpRetry.attempts >= kCriticalStartupResetAttempts) {
            ForceTeensyRebootIfSafe("bmp585");
        }
    }

    Icm20948SensorSetFlightStatus(flightComputer.Status());
    // Pass burnout timestamp and current timestamp for burnout correction burst
    const float currentTimeSeconds = static_cast<float>(nowMs) * 0.001f;
    const float burnoutTimeSeconds = static_cast<float>(flightComputer.BurnoutTime());
    Icm20948SensorSetBurnoutTimestamp(burnoutTimeSeconds);
    Icm20948SensorSetCurrentTimestamp(currentTimeSeconds);
    if (kLsmEnabled) {
        Lsm9ds1SensorSetFlightStatus(flightComputer.Status());
        Lsm9ds1SensorSetBurnoutTimestamp(burnoutTimeSeconds);
        Lsm9ds1SensorSetCurrentTimestamp(currentTimeSeconds);
    }
    if (kPulseEnabled) {
        Ellipse20SensorSetFlightStatus(flightComputer.Status());
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

    g_healthPhase = flightComputer.Status();
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
    const bool hasBaroAgl = g_hasPadAltitude;
    const float altitudeAglMeters = altitudeAglFeet * 0.3048f;

    const bool baroTransientActive =
        g_flapActuator.IsSettling() || (nowMs < g_altimeterTransientUntilMs);
    data.altimeterGateSigma = baroTransientActive
        ? settings::actuation::kBaroInnovationGateSigmaTransient
        : settings::actuation::kBaroInnovationGateSigmaNominal;
    data.altimeterSigmaScale = baroTransientActive
        ? settings::actuation::kBaroDeweightSigmaScale
        : 1.0f;
    data.flapCommandDeg = g_servoCommandDeg;
    data.flapEffectiveDeg = g_servoEffectiveDeg;
    data.actuationIsSettling = g_flapActuator.IsSettling() ? 1.0f : 0.0f;

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
        const FlightStatus flightStatus = flightComputer.Status();
        if (flightStatus == FlightStatus::Ground) {
            g_autoActuationFirstMotionReleased = false;
            g_autoActuationSafetyLatched = false;
        }
        if (ShouldLatchAutoActuationSafety(state, flightStatus, hasBaroAgl, altitudeAglMeters)) {
            g_autoActuationSafetyLatched = true;
        }

        if (!g_autoActuationSafetyLatched) {
            autoCommandDeg =
                ComputeAutoActuationCommandDeg(nowMs, state, flightStatus, g_servoEffectiveDeg, &autoTelemetry);
        }
        const bool firstMotionRequested =
            autoCommandDeg > settings::actuation::kServoSettlingAngleEpsilonDeg;
        if (firstMotionRequested &&
            ShouldBlockFirstAutoActuationMotion(state, flightStatus, hasBaroAgl, altitudeAglMeters)) {
            autoCommandDeg = 0.0f;
            g_actuationLastCommandDeg = 0.0f;
            g_actuationHasLastControlUpdate = false;
            autoTelemetry.autoCommandDeg = 0.0f;
        } else if (g_autoActuationSafetyLatched) {
            autoCommandDeg = 0.0f;
            g_actuationLastCommandDeg = 0.0f;
            g_actuationHasLastControlUpdate = false;
            autoTelemetry.autoCommandDeg = 0.0f;
        } else if (firstMotionRequested) {
            g_autoActuationFirstMotionReleased = true;
        }
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
    data.flapCommandDeg = g_servoCommandDeg;
    data.flapEffectiveDeg = g_servoEffectiveDeg;
    data.actuationIsSettling = g_flapActuator.IsSettling() ? 1.0f : 0.0f;
    data.predictorSeedHorizontalSpeedMps = autoTelemetry.predictorSeedHorizontalSpeedMps;
    data.predictorSeedClampedZenithRad = autoTelemetry.predictorSeedClampedZenithRad;
    data.predictorSeedClampedAngularRateRadPerSec = autoTelemetry.predictorSeedClampedAngularRateRadPerSec;
    data.predictorSeedConfidenceFlags = autoTelemetry.predictorSeedConfidenceFlags;

    if (!g_csvReplay.enabled) {
        DataLoggerLogTelemetry(
            data, flightComputer.Status(), hasFilteredState ? &state : nullptr, g_servoCommandDeg, g_servoEffectiveDeg);
    }

    if (hasFilteredState) {
        const FlightStatus status = flightComputer.Status();
        if (!g_hasLoggedStatus || status != g_lastLoggedStatus) {
            DataLoggerLogEvent(FlightEventType::StageChange,
                               status,
                               state.time,
                               state.position[2] * constants::kMetersToFeet,
                               state.velocity[2] * constants::kMetersToFeet,
                               state.apogeeEstimate * constants::kMetersToFeet,
                               g_servoCommandDeg,
                               g_servoEffectiveDeg);
            g_lastLoggedStatus = status;
            g_hasLoggedStatus = true;
        }
    }

    const float eventTimestamp = hasFilteredState ? state.time : data.timestamp;
    const float eventAltitudeFeet = hasFilteredState ? (state.position[2] * constants::kMetersToFeet) : 0.0f;
    const float eventVerticalVelocityFps = hasFilteredState ? (state.velocity[2] * constants::kMetersToFeet) : 0.0f;
    const float eventApogeeEstimateFeet =
        hasFilteredState ? (state.apogeeEstimate * constants::kMetersToFeet) : 0.0f;
    if (g_flapActuator.ConsumeActuationEvent()) {
        DataLoggerLogEvent(FlightEventType::FlapActuated,
                           flightComputer.Status(),
                           eventTimestamp,
                           eventAltitudeFeet,
                           eventVerticalVelocityFps,
                           eventApogeeEstimateFeet,
                           g_servoCommandDeg,
                           g_servoEffectiveDeg);
    }
    if (g_flapActuator.ConsumeSettlingTimerFiredEvent()) {
        DataLoggerLogEvent(FlightEventType::FlapSettlingTimerFired,
                           flightComputer.Status(),
                           eventTimestamp,
                           eventAltitudeFeet,
                           eventVerticalVelocityFps,
                           eventApogeeEstimateFeet,
                           g_servoCommandDeg,
                           g_servoEffectiveDeg);
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
        LOG_PRINT(" zft=");
        LOG_PRINT(state.position[2] * constants::kMetersToFeet, 2);
        LOG_PRINT(" vzfps=");
        LOG_PRINT(state.velocity[2] * constants::kMetersToFeet, 2);
        LOG_PRINT(" az=");
        LOG_PRINT(state.acceleration[2], 2);
        LOG_PRINT(" iaz=");
        LOG_PRINT(state.inertialAcceleration[2], 2);
        LOG_PRINT(" apgft=");
        LOG_PRINT(state.apogeeEstimate * constants::kMetersToFeet, 2);
        LOG_PRINT(" cmd=");
        LOG_PRINT(g_servoCommandDeg, 1);
        LOG_PRINT(" eff=");
        LOG_PRINT(g_servoEffectiveDeg, 1);
        LOG_PRINT(" mode=");
        LOG_PRINT(manualOverrideActive ? "manual" : "auto");
        if (flightComputer.Status() == FlightStatus::Ground) {
            LOG_PRINT(" pad=");
            LOG_PRINT(state.padReferenceSettled > 0.5f ? "ready" : "settling");
            LOG_PRINT(" driftfpm=");
            LOG_PRINT(state.padReferenceDriftMps * constants::kMetersToFeet * 60.0f, 2);
        }
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
