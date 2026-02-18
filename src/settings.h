#pragma once

#include <Arduino.h>

#include <stddef.h>
#include <stdint.h>

#ifndef ENABLE_SERIAL_TELEMETRY
#define ENABLE_SERIAL_TELEMETRY 1
#endif

#ifndef DATA_LOGGER_BUFFER_SIZE
#define DATA_LOGGER_BUFFER_SIZE 4096
#endif

#ifndef DATA_LOGGER_FLUSH_INTERVAL_US
#define DATA_LOGGER_FLUSH_INTERVAL_US 50000
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
// Enables/disables serial telemetry printing in main loop and init paths.
constexpr bool kEnableSerialTelemetry = (ENABLE_SERIAL_TELEMETRY != 0);
// Byte size of SD log staging buffer before writes are flushed to the card.
constexpr size_t kDataLoggerBufferSize = DATA_LOGGER_BUFFER_SIZE;
// Minimum elapsed time between forced log buffer flushes (microseconds).
constexpr uint32_t kDataLoggerFlushIntervalUs = DATA_LOGGER_FLUSH_INTERVAL_US;
}

// ---------------------------------------------------------------------------
// Hardware Settings
// Physical pin assignments and actuator positions.
// ---------------------------------------------------------------------------
namespace hardware {
constexpr uint8_t kStatusLedPin = LED_BUILTIN;
constexpr uint8_t kServoPin = 18;
constexpr int kServoExtendAngle = 60;
constexpr int kServoRetractAngle = 0;
}

// ---------------------------------------------------------------------------
// Actuation Settings
// Deployment gating thresholds for the ACS servo logic.
// ---------------------------------------------------------------------------
namespace actuation {
// Minimum altitude above pad reference (feet AGL) before coast-phase extension.
constexpr float kServoMinExtendAltitudeFeet = 1000.0f;
// Physical full-deployment angle (degrees). 0 deg is fully retracted.
constexpr float kServoMaxActuationDeg = 60.0f;
// First-order servo/flap response time constant (seconds).
constexpr float kServoLatencySeconds = 0.20f;
// Controller update period for angle optimization.
constexpr uint32_t kControlUpdateIntervalMs = 40;
// Candidate angle spacing used by the optimizer search (degrees).
constexpr float kAngleStepDeg = 2.0f;
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
// Integration step cap for the actuation-side predictor (kept at flight default for accuracy).
constexpr int kActuationPredictorMaxSteps = APOGEE_PREDICTOR_MAX_STEPS;
// Coarse search spacing for two-stage command optimization (degrees).
constexpr float kCoarseAngleStepDeg = 8.0f;
// If top coarse candidates are too close in cost, use full-resolution sweep.
constexpr float kCoarseAmbiguityCostThreshold = 2.0f;
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
constexpr float kGroundTemperatureF = 44.0f;
// Measured surface wind speed in miles per hour.
constexpr float kWindSpeedMph = 6.0f;
// Meteorological wind direction in degrees.
constexpr float kWindDirectionDeg = 200.0f;
// Launch rail azimuth direction in degrees.
constexpr float kLaunchDirectionDeg = 200.0f;
// Terrain roughness length (meters) for log wind profile.
constexpr float kRoughnessLengthMeters = 0.075f;
// Height where gradient wind is modeled (meters).
constexpr float kGradientHeightMeters = 300.0f;
// Height of wind measurement input (meters).
constexpr float kMeasurementHeightMeters = 10.0f;
}

// ---------------------------------------------------------------------------
// Flight/Estimator Settings
// Core thresholds and filter/prediction tuning values.
// ---------------------------------------------------------------------------
namespace flight {
constexpr uint32_t kErrorBlinkIntervalMs = 120;
constexpr uint32_t kRecoveryBlinkIntervalMs = 60;

constexpr float kDefaultDtSeconds = 0.03f;
constexpr float kLiftoffAccelerationThresholdMps2 = 20.0f;
constexpr float kLiftoffAltitudeThresholdM = 40.0f;
constexpr float kBurnoutAccelerationThresholdMps2 = 0.0f;
constexpr float kBurnoutVelocityThresholdMps = 0.0f;
constexpr float kDescentVelocityThresholdMps = 0.0f;
constexpr float kDescentAccelerationThresholdMps2 = 0.0f;

constexpr double kSigmaAccelXY = 0.5;
constexpr double kSigmaAccelZ = 0.5;
constexpr double kSigmaAltimeter = 0.5;
constexpr double kProcessNoiseXY = 0.5;
constexpr double kProcessNoiseZ = 1.0;
constexpr double kApogeeTargetMeters = 1711.;

constexpr int kApogeePredictorMaxSteps = APOGEE_PREDICTOR_MAX_STEPS;
}

// ---------------------------------------------------------------------------
// Sensor Settings
// Sensor-specific configuration and runtime sanity checks.
// ---------------------------------------------------------------------------
namespace sensors {
namespace bmp581 {
// Pressure reference used by barometric altitude conversion.
constexpr float kSeaLevelPressureHpa = 1018.8f;
// Reject altitude jumps that imply faster vertical motion than this rate.
constexpr float kMaxAltitudeRateFeetPerSecond = 2500.0f;
// Minimum single-sample jump (feet) required before classifying as a spike.
constexpr float kMinSpikeJumpFeet = 500.0f;
// Absolute altitude magnitude limit for invalid sample rejection.
constexpr float kMaxValidAltitudeFeet = 120000.0f;
}
}
}  // namespace settings
