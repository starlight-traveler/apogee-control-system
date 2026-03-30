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

#ifndef DATA_LOGGER_MIN_FLUSH_BYTES
#define DATA_LOGGER_MIN_FLUSH_BYTES 4096
#endif

#ifndef DATA_LOGGER_PREALLOCATE_BYTES
#define DATA_LOGGER_PREALLOCATE_BYTES 1048576
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
// Prefer writing at least this many bytes per flush when timing allows.
constexpr size_t kDataLoggerMinFlushBytes = DATA_LOGGER_MIN_FLUSH_BYTES;
// Best-effort initial file preallocation for less fragmented SD writes.
constexpr uint32_t kDataLoggerPreallocateBytes = DATA_LOGGER_PREALLOCATE_BYTES;
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
// Flap actuation angle-to-PWM calibration endpoints.
// ---------------------------------------------------------------------------
namespace actuation {
// Physical full-deployment angle (degrees). 0 deg is fully retracted.
constexpr float kServoMaxActuationDeg = 45.0f;
constexpr int kServoAttachMinPulseUs = 400;
constexpr int kServoAttachMaxPulseUs = 2700;
constexpr int kTopServoClosedPwmUs = 1150;
constexpr int kTopServoOpenPwmUs = 1744;
constexpr int kBottomServoClosedPwmUs = 2169;
constexpr int kBottomServoOpenPwmUs = 1585;
// First-order servo/flap response time constant (seconds).
constexpr float kServoLatencySeconds = 0.66f;
// Minimum time between commanded PWM updates.
constexpr uint32_t kServoMinStepIntervalMs = 150;
// Settling window after a PWM step is applied.
constexpr uint32_t kServoSettlingDurationMs = 250;
// Consider actuator settled when command and effective are within this error.
constexpr float kServoSettlingAngleEpsilonDeg = 0.5f;
// Controller update period for angle optimization.
constexpr uint32_t kControlUpdateIntervalMs = 40;
// Continuous angle sweep resolution for automatic apogee control.
constexpr float kControlSweepStepDeg = 0.5f;
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
constexpr const char *kCsvReplayPath = "tools/replay/data/output.csv";
constexpr size_t kCsvLineBufferSize = 2048;
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
constexpr uint32_t kStateLogIntervalMs = 50;
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
constexpr double kApogeeTargetMeters = 1100;
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
// Adaptive axial-drag correction, tuned from replay scripts. This stays
// single-state on purpose so the estimator path remains lightweight enough for
// the flight controller hot loop.
constexpr float kAdaptiveAxialDragScaleMin = 1.00f;
constexpr float kAdaptiveAxialDragScaleMax = 1.35f;
constexpr float kAdaptiveAxialDragResidualClampMps2 = 8.0f;
constexpr float kAdaptiveAxialDragTauSeconds = 0.45f;
constexpr float kAdaptiveAxialAccelMinAbsMps2 = 0.75f;

constexpr int kApogeePredictorMaxSteps = APOGEE_PREDICTOR_MAX_STEPS;
}

// ---------------------------------------------------------------------------
// Sensor Settings
// Sensor-specific configuration and runtime sanity checks.
// ---------------------------------------------------------------------------
namespace sensors {
namespace bno {
enum class Model : uint8_t { Bno055 = 0, Bno085 = 1 };
enum class Transport : uint8_t { I2c = 0, Spi = 1 };

// Master enable for the entire BNO sensor tree.
constexpr bool kEnabled = true;
// Select the active BNO-family device used by the firmware.
constexpr Model kModel = Model::Bno085;
// Select the transport used by the active BNO-family device.
constexpr Transport kTransport = Transport::I2c;
}

namespace bno055 {
// BNO055 I2C address.
constexpr uint8_t kI2cAddress = 0x28;
// Optional BNO055 reset pin. Set to -1 if reset is not wired.
constexpr int8_t kResetPin = -1;
// Poll interval for consuming queued sensor events.
constexpr uint32_t kSampleIntervalUs = 10000;
// If no complete sample arrives for this long, force a full reinit.
constexpr uint32_t kDataTimeoutUs = 250000;
}

namespace bno085 {
// BNO085 I2C address.
constexpr uint8_t kI2cAddress = 0x4B;
// Fast-mode I2C clock for the BNO085 sidecar path.
constexpr uint32_t kI2cClockHz = 400000UL;
// SPI chip-select pin for the BNO085 when SPI transport is selected.
constexpr uint8_t kChipSelectPin = 4;
// Optional interrupt pin for the BNO085 SPI transport. Set to -1 if unused.
constexpr int8_t kInterruptPin = 2;
// Optional BNO085 reset pin. Set to -1 if reset is not wired.
constexpr int8_t kResetPin = 3;
// Initial delay before the first startup attempt so the sensor hub can finish booting.
constexpr uint32_t kStartupSettleDelayMs = 250;
// Duration of the active-low reset pulse when reset wiring is available.
constexpr uint32_t kResetPulseDelayMs = 10;
// Delay after releasing reset before starting SPI traffic.
constexpr uint32_t kPostResetBootDelayMs = 150;
// Number of bounded startup retries performed during setup.
constexpr uint8_t kStartupRetryCount = 4;
// Delay between startup retries.
constexpr uint32_t kStartupRetryDelayMs = 200;
// Poll interval for consuming queued sensor events.
constexpr uint32_t kSampleIntervalUs = 10000;
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

namespace lsm9ds1 {
constexpr bool kEnabled = false;
constexpr uint8_t kAccelGyroChipSelectPin = 15;
constexpr uint8_t kMagChipSelectPin = 14;
// Route the LSM9DS1 accel/gyro data-ready output (INT1) to this pin.
// Set to -1 to disable interrupt-driven acquisition and fall back to polling.
constexpr int8_t kInterruptPin = -1;
constexpr uint32_t kSampleIntervalUs = 5000;
constexpr uint8_t kCalibrationAccelRangeG = 2;
constexpr uint16_t kCalibrationGyroRangeDps = 245;
constexpr uint8_t kCalibrationMagRangeGauss = 4;
constexpr uint8_t kAccelRangeG = 16;
constexpr uint16_t kGyroRangeDps = 2000;
constexpr uint8_t kMagRangeGauss = 12;
constexpr uint8_t kGyroSampleRateSetting = 6;
constexpr uint8_t kAccelSampleRateSetting = 6;
constexpr uint8_t kMagSampleRateSetting = 7;
// Library bandwidth/high-resolution settings:
// gyro bandwidth: 0..3, accel bandwidth: -1..3, accel HR bandwidth: 0..3.
constexpr uint8_t kGyroBandwidthSetting = 2;
constexpr int8_t kAccelBandwidthSetting = 2;
constexpr bool kAccelHighResolutionEnable = true;
constexpr uint8_t kAccelHighResolutionBandwidthSetting = 2;
// Magnetometer performance tuning: performance 0..3, mode 0..2.
constexpr bool kMagTemperatureCompensationEnable = true;
constexpr uint8_t kMagXyPerformanceSetting = 3;
constexpr uint8_t kMagZPerformanceSetting = 3;
constexpr bool kMagLowPowerEnable = false;
constexpr uint8_t kMagOperatingModeSetting = 0;
// Optional FIFO use for accel/gyro backlog absorption. Leave off unless needed.
constexpr bool kUseFifo = false;
constexpr uint8_t kFifoThresholdSamples = 8;
constexpr uint8_t kFifoMaxBurstSamplesPerAcquire = 4;
// Optional library-side hard-iron offset load for sanity checks.
// Keep false for the normal path; the firmware calibration model remains primary.
constexpr bool kUseLibraryMagOffsets = false;
constexpr uint8_t kAxisMap[3] = {0, 1, 2};
constexpr int8_t kAxisSign[3] = {-1, 1, 1};
constexpr float kMagDeclinationDeg = -14.84f;
constexpr float kAccelCorrectionGainGround = 18.0f;
constexpr float kAccelCorrectionGainDescent = 9.0f;
constexpr float kMagCorrectionGainGround = 7.0f;
constexpr float kMagCorrectionGainFlight = 2.4f;
constexpr float kMagCorrectionGainDescent = 5.0f;
constexpr float kMagTrustGround = 1.0f;
constexpr float kMagTrustBurn = 0.0f;
constexpr float kMagTrustCoast = 0.35f;
constexpr float kMagTrustOvershoot = 0.45f;
constexpr float kMagTrustDescent = 0.8f;
constexpr float kMagTrustGyroFadeStartRadPerSec = 0.6f;
constexpr float kMagTrustGyroFadeEndRadPerSec = 4.0f;
constexpr float kGyroBiasLearningRate = 0.08f;
constexpr float kGyroBiasMaxRadPerSec = 0.35f;
constexpr float kStationaryGyroMaxRadPerSec = 0.35f;
constexpr float kGyroReferenceTemperatureC = 21.0f;
constexpr float kGyroTempBiasSlopeRadPerSecPerC[3] = {0.0f, 0.0f, 0.0f};
constexpr uint16_t kGroundAlignmentMinSamples = 40;
constexpr float kGroundAlignmentAccelTrustMin = 0.75f;
constexpr float kGroundAlignmentMagTrustMin = 0.20f;
constexpr float kAccelCorrectionMinG = 0.8f;
constexpr float kAccelCorrectionMaxG = 1.2f;
constexpr float kAccelCorrectionGyroFadeStartRadPerSec = 0.4f;
constexpr float kAccelCorrectionGyroFadeEndRadPerSec = 3.0f;
constexpr float kMagCorrectionMaxRelativeError = 0.35f;
constexpr float kMagReferenceBlend = 0.02f;
constexpr float kAccelCorrectionMaxRateRadPerSec = 6.0f;
constexpr float kMagCorrectionMaxRateRadPerSec = 2.5f;
constexpr float kTotalCorrectionMaxRateRadPerSec = 7.0f;
constexpr float kGyroOffset[3] = {0.0f, 0.0f, 0.0f};
constexpr float kAccelBias[3] = {0.0f, 0.0f, 0.0f};
constexpr float kAccelAinv[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
};
// Fixed rotation from calibrated sensor axes into the rocket body frame.
constexpr float kMountRotation[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
};
constexpr float kMagBias[3] = {0.0f, 0.0f, 0.0f};
constexpr float kMagAinv[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
};
}

namespace icm20948 {
// ICM-20948 SPI chip-select pin.
constexpr uint8_t kChipSelectPin = 25;
// Route the ICM-20948 INT pin here for raw-data-ready interrupt driven reads.
// Set to -1 to leave the driver in polling mode.
constexpr int8_t kInterruptPin = -1;
// Fresh-sample pacing used by the firmware's ICM acquisition path.
constexpr uint32_t kSampleIntervalUs = 5000;
// Stored calibration constants were fit at the library default ranges below.
// If you update these references, the hard-coded bias terms must match.
constexpr uint8_t kCalibrationAccelRangeG = 2;
constexpr uint16_t kCalibrationGyroRangeDps = 250;
// Active full-scale ranges used in flight. Higher ranges avoid saturation
// during launch transients but require the calibration bias terms to be
// rescaled before use.
constexpr uint8_t kAccelRangeG = 16;
constexpr uint16_t kGyroRangeDps = 2000;
// Hardware sample-rate dividers: accel ODR = 1125 / (1 + a), gyro ODR = 1100 / (1 + g).
constexpr uint16_t kAccelSampleRateDivider = 4;
constexpr uint8_t kGyroSampleRateDivider = 4;
// Enable the internal digital low-pass filters and choose the bandwidth bins.
// Values map to the SparkFun ICM-20948 enum constants:
// accel: 0=246Hz, 2=111Hz, 3=50Hz ...
// gyro:  0=196Hz, 2=119Hz, 3=51Hz ...
constexpr bool kEnableDlpFilter = true;
constexpr uint8_t kAccelDlpFilterSetting = 2;
constexpr uint8_t kGyroDlpFilterSetting = 2;
// Local magnetic declination used for compass yaw correction.
constexpr float kMagDeclinationDeg = -14.84f;
// Hold the last trusted pad attitude for a short bounded window after burn
// starts to approximate rail guidance before the airframe is fully free.
constexpr float kRailConstraintDurationSeconds = 0.25f;
constexpr float kRailConstraintGain = 10.0f;
// Adaptive AHRS feedback gains (rad/s per unit vector error).
constexpr float kAccelCorrectionGainGround = 18.0f;
constexpr float kAccelCorrectionGainDescent = 9.0f;
constexpr float kMagCorrectionGainGround = 7.0f;
constexpr float kMagCorrectionGainFlight = 2.4f;
constexpr float kMagCorrectionGainDescent = 5.0f;
// Additional phase multipliers applied to magnetometer trust.
constexpr float kMagTrustGround = 1.0f;
constexpr float kMagTrustBurn = 0.0f;
constexpr float kMagTrustCoast = 0.35f;
constexpr float kMagTrustOvershoot = 0.45f;
constexpr float kMagTrustDescent = 0.8f;
constexpr float kMagTrustGyroFadeStartRadPerSec = 0.6f;
constexpr float kMagTrustGyroFadeEndRadPerSec = 4.0f;
// Learn residual gyro bias only during low-dynamic phases.
constexpr float kGyroBiasLearningRate = 0.08f;
constexpr float kGyroBiasMaxRadPerSec = 0.35f;
constexpr float kStationaryGyroMaxRadPerSec = 0.35f;
// Optional linear gyro-bias temperature compensation. Leave zero until the
// active-range IMU has been characterized against temperature.
constexpr float kGyroReferenceTemperatureC = 21.0f;
constexpr float kGyroTempBiasSlopeRadPerSecPerC[3] = {0.0f, 0.0f, 0.0f};
// Require a short stable ground window before publishing the first quaternion.
constexpr uint16_t kGroundAlignmentMinSamples = 40;
constexpr float kGroundAlignmentAccelTrustMin = 0.75f;
constexpr float kGroundAlignmentMagTrustMin = 0.20f;
// Only trust accel correction when the calibrated magnitude is near 1 g.
constexpr float kAccelCorrectionMinG = 0.8f;
constexpr float kAccelCorrectionMaxG = 1.2f;
// Fade accel correction as angular rate rises, even inside the 1 g window.
constexpr float kAccelCorrectionGyroFadeStartRadPerSec = 0.4f;
constexpr float kAccelCorrectionGyroFadeEndRadPerSec = 3.0f;
// Only trust mag correction when the calibrated field stays near its learned baseline.
constexpr float kMagCorrectionMaxRelativeError = 0.35f;
constexpr float kMagReferenceBlend = 0.02f;
// Treat values this close to full-scale as clipped and reject their correction use.
constexpr float kAccelSaturationFraction = 0.97f;
constexpr float kGyroSaturationFraction = 0.97f;
// Bound observer feedback so vibration or transients do not whip the estimate.
constexpr float kAccelCorrectionMaxRateRadPerSec = 6.0f;
constexpr float kMagCorrectionMaxRateRadPerSec = 2.5f;
constexpr float kTotalCorrectionMaxRateRadPerSec = 7.0f;
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
// Fixed rotation from calibrated sensor axes into the rocket body frame.
constexpr float kMountRotation[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
};
// Calibrated magnetometer hard-iron offsets.
constexpr float kMagBias[3] = {-156.70f, -52.79f, -141.07f};
// Calibrated magnetometer soft-iron inverse matrix.
constexpr float kMagAinv[3][3] = {
    {1.12823f, -0.01142f, 0.00980f},
    {-0.01142f, 1.09539f, 0.00927f},
    {0.00980f, 0.00927f, 1.10625f},
};

namespace crosscheck {
// Agreement thresholds used to gate sensor correction trust between the ICM and LSM AHRS paths.
constexpr float kAccelDiffFullTrustMps2 = 0.75f;
constexpr float kAccelDiffZeroTrustMps2 = 3.0f;
constexpr float kGyroDiffFullTrustRadPerSec = 0.15f;
constexpr float kGyroDiffZeroTrustRadPerSec = 1.0f;
constexpr float kQuaternionDiffFullTrustDeg = 5.0f;
constexpr float kQuaternionDiffZeroTrustDeg = 25.0f;
constexpr float kTrustBlend = 0.2f;
constexpr float kTrustRecoveryPerLoop = 0.02f;
// Compare the fast IMU rails only when their cached samples are both recent
// and closely time-aligned.
constexpr uint32_t kFastSampleMaxAgeUs = 20000;
constexpr uint32_t kFastPairMaxSkewUs = 10000;
// The BNO085 runs as a slower I2C reference rail, so allow more age/skew
// before discarding a comparison.
constexpr uint32_t kBnoSampleMaxAgeUs = 50000;
constexpr uint32_t kBnoPairMaxSkewUs = 30000;
}
}
}

// ---------------------------------------------------------------------------
// AHRS Improvement Settings
// Configuration for advanced AHRS features including exponential map integration,
// quaternion blending, soft saturation, and outlier detection.
// ---------------------------------------------------------------------------
namespace ahrs {
// Feature flags - enable/disable individual improvements.
constexpr bool kEnableExponentialMap = true;
constexpr bool kEnableQuaternionBlending = true;
constexpr bool kEnableSoftSaturation = true;
constexpr bool kEnableOutlierDetection = true;
constexpr bool kEnableBnoCoastBlending = true;
constexpr bool kEnableBnoReferenceCorrection = true;
constexpr bool kEnableTrustHysteresis = true;

// Soft saturation: ramp trust from 1.0 to 0.0 between this fraction and 1.0 of saturation.
constexpr float kSoftSaturationWarningFraction = 0.85f;

// Quaternion blending: minimum trust required to include a source in the blend.
constexpr float kMinBlendTrust = 0.1f;

// BNO085 coast blending: blend factor applied to BNO quaternion during coast phase.
constexpr float kBnoCoastBlendFactor = 0.1f;
// Slow BNO reference correction: small slerp toward the healthy BNO-backed
// reference quaternion after gyro propagation.
constexpr float kBnoReferenceCorrectionBlendFactor = 0.03f;
// Rail health thresholds derived from cross-check trust.
constexpr float kHealthyRailTrust = 0.65f;
constexpr float kDegradedRailTrust = 0.35f;

// Outlier detection: maximum allowed rate of change for gyro and accel.
constexpr float kGyroMaxRateChangeRadPerSecSq = 100.0f;
constexpr float kAccelMaxRateChangeMps3 = 500.0f;

// Trust hysteresis: reduce blend rate by this factor when trust direction changes.
constexpr float kTrustHysteresisReductionFactor = 0.25f;

// Exponential map: threshold for switching between small-angle Taylor series and full Rodrigues.
constexpr float kExpMapSmallAngleThreshold = 0.01f;
}

// ---------------------------------------------------------------------------
// Apogee Predictor Improvement Settings
// Advanced predictor features: Mach-dependent drag, uncertainty bounds,
// density modeling, and wind estimation.
// ---------------------------------------------------------------------------
namespace predictor {
// Feature flags - enable/disable individual improvements.
constexpr bool kEnableMachDependentDrag = true;
constexpr bool kEnableUncertaintyBounds = true;
constexpr bool kEnableDensityScaling = true;
constexpr bool kEnableWindEstimation = true;

// Mach-dependent drag adaptation: bin edges for piecewise-linear interpolation.
// Scales are learned independently in each bin during coast phase.
constexpr int kMachBinCount = 4;
constexpr float kMachBinEdges[kMachBinCount] = {0.3f, 0.6f, 0.9f, 1.2f};
constexpr float kMachDragScaleMin = 0.85f;
constexpr float kMachDragScaleMax = 1.50f;
constexpr float kMachDragAdaptTauSeconds = 0.6f;

// Prediction uncertainty bounds: perturbation factors for confidence interval.
constexpr float kUncertaintyDragPerturbFraction = 0.12f;  // +/- 12% drag variation
constexpr float kUncertaintyWindPerturbMps = 3.0f;        // +/- 3 m/s wind variation

// Atmospheric density model: ISA reference values.
constexpr float kSeaLevelPressurePa = 101325.0f;
constexpr float kSeaLevelTemperatureK = 288.15f;
constexpr float kTemperatureLapseRateKPerM = 0.0065f;
constexpr float kReferenceDensityKgPerM3 = 1.225f;

// Wind estimation: low-pass filter for horizontal acceleration residual.
constexpr float kWindEstimateBlendRate = 0.03f;
constexpr float kWindEstimateMaxMps = 25.0f;
constexpr float kWindEstimateMinCoastTimeSec = 0.5f;  // Wait before starting estimation
}
}  // namespace settings
