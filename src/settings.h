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
constexpr int kServoExtendAngle = 180;
constexpr int kServoRetractAngle = 0;
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
// Test Settings
// Optional bench-test behaviors that bypass normal flight logic.
// ---------------------------------------------------------------------------
namespace test {
// When true, firmware runs only a servo extend/retract bench test.
constexpr bool kEnableServoCycleTest = true;
// Total test runtime for servo cycling (milliseconds).
constexpr uint32_t kServoCycleDurationMs = 5000;
// Delay between each servo state toggle (milliseconds).
constexpr uint32_t kServoCycleToggleIntervalMs = 250;
}

// ---------------------------------------------------------------------------
// Environment Model Settings
// Default atmospheric and wind parameters used by EnvironmentModel::Config.
// ---------------------------------------------------------------------------
namespace environment {
// Ground temperature used as altitude=0 reference in Fahrenheit.
constexpr float kGroundTemperatureF = 50.0f;
// Measured surface wind speed in miles per hour.
constexpr float kWindSpeedMph = 10.0f;
// Meteorological wind direction in degrees.
constexpr float kWindDirectionDeg = 270.0f;
// Launch rail azimuth direction in degrees.
constexpr float kLaunchDirectionDeg = 260.0f;
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
constexpr double kApogeeTargetMeters = 1569.72;

constexpr int kApogeePredictorMaxSteps = APOGEE_PREDICTOR_MAX_STEPS;
}
}  // namespace settings
