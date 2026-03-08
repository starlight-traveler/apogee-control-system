#pragma once

#include <Arduino.h>

#include <stddef.h>
#include <stdint.h>

#ifndef DATA_LOGGER_BUFFER_SIZE
#define DATA_LOGGER_BUFFER_SIZE 4096
#endif

#ifndef DATA_LOGGER_FLUSH_INTERVAL_US
#define DATA_LOGGER_FLUSH_INTERVAL_US 50000
#endif

#ifndef DATA_LOGGER_SYNC_INTERVAL_US
#define DATA_LOGGER_SYNC_INTERVAL_US 1000000
#endif

#ifndef APOGEE_PREDICTOR_MAX_STEPS
#define APOGEE_PREDICTOR_MAX_STEPS 256
#endif

namespace settings {
// ---------------------------------------------------------------------------
// Build/Profile Settings
// These are controlled by platformio build flags and affect runtime behavior.
// ---------------------------------------------------------------------------
namespace build {
// Byte size of SD log staging buffer before writes are flushed to the card.
constexpr size_t kDataLoggerBufferSize = DATA_LOGGER_BUFFER_SIZE;
// Minimum elapsed time between forced log buffer flushes (microseconds).
constexpr uint32_t kDataLoggerFlushIntervalUs = DATA_LOGGER_FLUSH_INTERVAL_US;
// Minimum elapsed time between SD metadata sync calls during nominal flight.
constexpr uint32_t kDataLoggerSyncIntervalUs = DATA_LOGGER_SYNC_INTERVAL_US;
}

// ---------------------------------------------------------------------------
// Hardware Settings
// Physical pin assignments and actuator positions.
// ---------------------------------------------------------------------------
namespace hardware {
constexpr uint8_t kStatusLedPin = -1;
constexpr uint8_t kBuzzerPin = 8;
constexpr uint8_t kTopServoPin = 9;
constexpr uint8_t kBottomServoPin = 10;
constexpr int kServoExtendAngle = 60;
constexpr int kServoRetractAngle = 0;
}

// ---------------------------------------------------------------------------
// Network / Telemetry Settings
// WiFi AirLift mapping and UDP stream controls.
// ---------------------------------------------------------------------------
namespace network {
constexpr bool kEnableTelemetry = true;
// `true` = Teensy hosts AP; `false` = Teensy joins existing WiFi as station.
constexpr bool kUseAccessPointMode = true;
constexpr const char *kSsid = "Hi_Madelyn";
constexpr const char *kPassword = "11112222";
// WiFiNINA `setPins()` arguments for the AirLift coprocessor.
constexpr int8_t kAirliftSsPin = 34;
constexpr int8_t kAirliftAckPin = 31;
constexpr int8_t kAirliftResetPin = 32;
constexpr int8_t kAirliftGpio0Pin = -1;
// Stream packet cadence. 20 ms = 50 Hz.
constexpr uint32_t kTelemetryIntervalMs = 250;
// How often to refresh WiFi link status checks in telemetry service.
constexpr uint32_t kWiFiStatusCheckIntervalMs = 5000;
constexpr uint16_t kTelemetryUdpLocalPort = 5006;
constexpr uint16_t kTelemetryUdpRemotePort = 5005;
constexpr bool kRequireSubscriberHeartbeat = true;
constexpr uint32_t kSubscriberHeartbeatTimeoutMs = 600;
// Ground-station receiver target (default 192.168.4.2 on AP subnet).
constexpr uint8_t kTelemetryRemoteIp0 = 192;
constexpr uint8_t kTelemetryRemoteIp1 = 168;
constexpr uint8_t kTelemetryRemoteIp2 = 4;
constexpr uint8_t kTelemetryRemoteIp3 = 2;
}

// ---------------------------------------------------------------------------
// Actuation Settings
// Flap actuation angle-to-PWM calibration table.
// ---------------------------------------------------------------------------
namespace actuation {
// Physical full-deployment angle (degrees). 0 deg is fully retracted.
constexpr float kServoMaxActuationDeg = 60.0f;
struct ServoCalibrationPoint {
    float angleDeg;
    int topPwmUs;
    int bottomPwmUs;
};
constexpr size_t kServoCalibrationPointCount = 20;
constexpr ServoCalibrationPoint kServoCalibrationTable[kServoCalibrationPointCount] = {
    {0.0f, 1090, 1967},  {3.2f, 1122, 1935},  {6.3f, 1154, 1903},  {9.5f, 1186, 1871},
    {12.6f, 1218, 1839}, {15.8f, 1250, 1807}, {18.9f, 1282, 1775}, {22.1f, 1315, 1742},
    {25.3f, 1347, 1710}, {28.4f, 1379, 1678}, {31.6f, 1411, 1646}, {34.7f, 1443, 1614},
    {37.9f, 1475, 1582}, {41.1f, 1507, 1550}, {44.2f, 1539, 1518}, {47.4f, 1571, 1486},
    {50.5f, 1603, 1454}, {53.7f, 1636, 1421}, {56.8f, 1668, 1389}, {60.0f, 1700, 1355},
};
// First-order servo/flap response time constant (seconds).
constexpr float kServoLatencySeconds = 0.20f;
// Minimum time between commanded PWM table step changes.
constexpr uint32_t kServoMinStepIntervalMs = 150;
// Settling window after a PWM step is applied.
constexpr uint32_t kServoSettlingDurationMs = 250;
// Consider actuator settled when command and effective are within this error.
constexpr float kServoSettlingAngleEpsilonDeg = 0.5f;
// Controller update period for angle optimization.
constexpr uint32_t kControlUpdateIntervalMs = 40;
// Ignore tiny command changes to reduce chatter (degrees).
constexpr float kAngleCommandDeadbandDeg = 1.0f;
// Do not add drag when within this apogee error band (meters).
constexpr float kApogeeErrorDeadbandMeters = 6.0f;
// Cost weight for command slew (penalizes large step changes).
constexpr float kRatePenalty = 0.15f;
// Cost weight for high actuation angles (conserves control authority).
constexpr float kEffortPenalty = 0.20f;
// Extra cost multiplier when predicted apogee falls below target.
constexpr float kUndershootPenalty = 2.0f;
// During flap settling, inflate baro measurement sigma by this multiplier.
constexpr float kBaroDeweightSigmaScale = 12.0f;
// Keep baro in deweighted mode for at least this long after a flap transient.
constexpr uint32_t kBaroDeweightDurationMs = 300;
// Nominal/settling innovation gates for baro fusion.
constexpr float kBaroInnovationGateSigmaNominal = 3.5f;
constexpr float kBaroInnovationGateSigmaTransient = 2.5f;
// Late-coast soft-disable window: taper max commanded angle down as time-to-apogee shrinks.
constexpr float kCoastSoftDisableStartTimeToApogeeS = 2.5f;
// Hard-disable window: command 0 deg at/inside this time-to-apogee threshold.
constexpr float kCoastHardDisableTimeToApogeeS = 1.0f;
// Hard-disable when vertical speed gets this low in coast.
constexpr float kCoastHardDisableVelocityMps = 25.0f;
// Integration step cap for the actuation-side predictor (kept at flight default for accuracy).
constexpr int kActuationPredictorMaxSteps = APOGEE_PREDICTOR_MAX_STEPS;
}

// ---------------------------------------------------------------------------
// Status LED Settings (AirLift RGB LEDs)
// Low-rate, non-blocking status indication.
// ---------------------------------------------------------------------------
namespace status_leds {
// LED service period. 500 ms = 2 Hz blink cadence.
constexpr uint32_t kUpdateIntervalMs = 500;

// Fault (highest priority): RED
constexpr uint8_t kFaultR = 140;
constexpr uint8_t kFaultG = 0;
constexpr uint8_t kFaultB = 0;

// WiFi disconnected: AMBER/ORANGE
constexpr uint8_t kWifiDownR = 80;
constexpr uint8_t kWifiDownG = 24;
constexpr uint8_t kWifiDownB = 0;

// Manual override active: MAGENTA/PURPLE
constexpr uint8_t kManualR = 120;
constexpr uint8_t kManualG = 0;
constexpr uint8_t kManualB = 120;

// WiFi up, no subscriber heartbeat: BLUE BLINK
constexpr uint8_t kNoSubscriberR = 0;
constexpr uint8_t kNoSubscriberG = 0;
constexpr uint8_t kNoSubscriberB = 96;

// Flight phase colors when subscriber is active:
// Ground = BLUE
constexpr uint8_t kGroundR = 0;
constexpr uint8_t kGroundG = 0;
constexpr uint8_t kGroundB = 96;
// Burn = ORANGE
constexpr uint8_t kBurnR = 128;
constexpr uint8_t kBurnG = 48;
constexpr uint8_t kBurnB = 0;
// Coast = GREEN
constexpr uint8_t kCoastR = 0;
constexpr uint8_t kCoastG = 120;
constexpr uint8_t kCoastB = 0;
// Overshoot = YELLOW
constexpr uint8_t kOvershootR = 120;
constexpr uint8_t kOvershootG = 120;
constexpr uint8_t kOvershootB = 0;
// Descent = CYAN
constexpr uint8_t kDescentR = 0;
constexpr uint8_t kDescentG = 96;
constexpr uint8_t kDescentB = 96;
}

// ---------------------------------------------------------------------------
// Replay Settings
// Controls local CSV replay mode and parser buffer sizing.
// ---------------------------------------------------------------------------
namespace replay {
constexpr bool kEnableCsvReplay = false;
constexpr const char *kCsvReplayPath = "shortened.csv";
constexpr size_t kCsvLineBufferSize = 768;
}

// ---------------------------------------------------------------------------
// Environment Model Settings
// Default atmospheric and wind parameters used by EnvironmentModel::Config.
// ---------------------------------------------------------------------------
namespace environment {
// Ground temperature used as altitude=0 reference in Fahrenheit.
constexpr float kGroundTemperatureF = 32.0f;
// Measured surface wind speed in miles per hour.
constexpr float kWindSpeedMph = 8.0f;
// Meteorological wind direction in degrees.
constexpr float kWindDirectionDeg = 63.0f;
// Launch rail azimuth direction in degrees.
constexpr float kLaunchDirectionDeg = 63.0f;
// Terrain roughness length (meters) for log wind profile.
constexpr float kRoughnessLengthMeters = 0.075f;
// Height where gradient wind is modeled (meters).
constexpr float kGradientHeightMeters = 300.0f;
// Height of wind measurement input (meters).
constexpr float kMeasurementHeightMeters = 10.0f;
}

// ---------------------------------------------------------------------------
// Vehicle Model Settings
// Parameters used by the apogee predictor dynamics.
// ---------------------------------------------------------------------------
namespace vehicle {
// Aerodynamic moment arm CP-CG during coast/burnout [m].
// CP from tip: 1.7537 m, CG from tip: 1.31 m.
constexpr double kCenterOfPressureOffsetMeters = 0.4389;
// Longitudinal moment of inertia during coast [kg*m^2].
constexpr double kMomentOfInertiaKgM2 = 8.28;
// Rocket dry mass / burnout mass [kg].
constexpr double kDryMassKg = 18.09975;
}

// ---------------------------------------------------------------------------
// Flight/Estimator Settings
// Core thresholds and filter/prediction tuning values.
// ---------------------------------------------------------------------------
namespace flight {
constexpr uint32_t kErrorBlinkIntervalMs = 120;
constexpr uint32_t kRecoveryBlinkIntervalMs = 60;
constexpr uint32_t kDebugHeartbeatIntervalMs = 1000;
constexpr uint32_t kStateLogIntervalMs = 250;
constexpr uint32_t kRecoveryRetryInitialMs = 200;
constexpr uint32_t kRecoveryRetryStepMs = 200;
constexpr uint32_t kRecoveryRetryMaxMs = 2000;
constexpr uint32_t kTimingLogIntervalMs = 1000;

constexpr float kDefaultDtSeconds = 0.03f;
constexpr float kLiftoffAccelerationThresholdMps2 = 20.0f;
constexpr float kLiftoffAltitudeThresholdM = 40.0f;
constexpr float kLiftoffVelocityThresholdMps = 10.0f;
constexpr uint8_t kLiftoffConfirmSamples = 3;
// Burnout confirmation requires sustained low/negative accel while still ascending.
constexpr float kBurnoutAccelerationThresholdMps2 = 2.0f;
constexpr float kBurnoutVelocityThresholdMps = 5.0f;
constexpr float kBurnoutMinDurationSeconds = 1.0f;
constexpr uint8_t kBurnoutConfirmSamples = 10;
constexpr float kDescentVelocityThresholdMps = 0.0f;
constexpr float kDescentAccelerationThresholdMps2 = 0.0f;

// Fast-tracking defaults: prioritize responsiveness over smoothness.
constexpr double kSigmaAccelXY = 0.8;
constexpr double kSigmaAccelZ = 0.7;
constexpr double kSigmaAltimeter = 1.0;
constexpr double kProcessNoiseXY = 0.6;
constexpr double kProcessNoiseZ = 1.2;
constexpr double kApogeeTargetMeters = 1700;
// Predictor-only horizontal speed seed tuning. These values intentionally keep
// XY speed conservative because the estimator does not have a horizontal
// position/velocity measurement update.
constexpr float kPredictorHorizontalAccelLimitMps2 = 12.0f;
constexpr float kPredictorHorizontalDecayTauSeconds = 1.75f;
constexpr float kPredictorMaxSeedZenithDeg = 20.0f;
constexpr float kPredictorMaxSeedAngularRateRadPerSec = 1.5f;
constexpr float kPredictorMaxHorizontalSpeedMps = 65.0f;
constexpr float kPredictorMinHorizontalSpeedCapMps = 6.0f;
constexpr float kPredictorHorizontalSpeedMarginMps = 3.0f;

constexpr int kApogeePredictorMaxSteps = APOGEE_PREDICTOR_MAX_STEPS;
}

// ---------------------------------------------------------------------------
// Sensor Settings
// Sensor-specific configuration and runtime sanity checks.
// ---------------------------------------------------------------------------
namespace sensors {
namespace bno085 {
// BNO055 I2C address.
constexpr uint8_t kI2cAddress = 0x28;
// Optional BNO055 reset pin. Set to -1 if reset is not wired.
constexpr int8_t kResetPin = -1;
// Poll interval for consuming queued sensor events.
constexpr uint32_t kSampleIntervalUs = 10000;
// Number of full startup attempts before giving up to the caller.
constexpr uint8_t kInitializationAttempts = 5;
// Delay between failed startup attempts.
constexpr uint32_t kRetryDelayMs = 80;
// If no complete sample arrives for this long, force a full reinit.
constexpr uint32_t kDataTimeoutUs = 250000;
}

namespace bmp585 {
// Pressure reference used by barometric altitude conversion.
constexpr float kSeaLevelPressureHpa = 1032.2f;
// Reject altitude jumps that imply faster vertical motion than this rate.
constexpr float kMaxAltitudeRateFeetPerSecond = 2500.0f;
// Minimum single-sample jump (feet) required before classifying as a spike.
constexpr float kMinSpikeJumpFeet = 500.0f;
// Absolute altitude magnitude limit for invalid sample rejection.
constexpr float kMaxValidAltitudeFeet = 120000.0f;
}

namespace ms5611 {
// SPI chip-select pin for the MS5611 breakout.
constexpr uint8_t kChipSelectPin = 36;
// Pressure reference used by barometric altitude conversion.
constexpr float kSeaLevelPressureHpa = 1018.8f;
// Minimum spacing between blocking reads.
constexpr uint32_t kMinReadSpacingUs = 1000;
// Absolute altitude magnitude limit for invalid sample rejection.
constexpr float kMaxValidAltitudeFeet = 120000.0f;
// Maximum disagreement before flagging the barometers as mismatched.
constexpr float kAgreementThresholdFeet = 150.0f;
}

namespace icm20948 {
// ICM-20948 SPI chip-select pin.
constexpr uint8_t kChipSelectPin = 25;
// Local magnetic declination used for compass yaw correction.
constexpr float kMagDeclinationDeg = -14.84f;
// Mahony filter proportional and integral gains.
constexpr float kMahonyKp = 50.0f;
constexpr float kMahonyKi = 0.0f;
// Gyro conversion scale used by the reference Mahony implementation.
constexpr float kGyroScaleRadPerSecPerLsb = 0.000133168788f;  // (PI/180)*0.00763
// Calibrated gyro zero-rate offsets.
constexpr float kGyroOffset[3] = {74.3f, 153.8f, -5.5f};
// Calibrated accelerometer hard-iron offsets.
constexpr float kAccelBias[3] = {79.60f, -18.56f, 383.31f};
// Calibrated accelerometer soft-iron inverse matrix.
constexpr float kAccelAinv[3][3] = {
    {1.00847f, 0.00470f, -0.00428f},
    {0.00470f, 1.00846f, -0.00328f},
    {-0.00428f, -0.00328f, 0.99559f},
};
// Calibrated magnetometer hard-iron offsets.
constexpr float kMagBias[3] = {-156.70f, -52.79f, -141.07f};
// Calibrated magnetometer soft-iron inverse matrix.
constexpr float kMagAinv[3][3] = {
    {1.12823f, -0.01142f, 0.00980f},
    {-0.01142f, 1.09539f, 0.00927f},
    {0.00980f, 0.00927f, 1.10625f},
};
}
}
}  // namespace settings
