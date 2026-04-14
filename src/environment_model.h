#pragma once

#include <cmath>

#include "constants.h"
#include "math_utils.h"
#include "settings.h"

// Encapsulates the logic in environment.py for use at runtime.
class EnvironmentModel {
  public:
    struct Config {
        float groundTemperatureF = settings::environment::kGroundTemperatureF;
        float seaLevelPressureHpa = settings::sensors::bmp585::kSeaLevelPressureHpa;
        float windSpeedMph = settings::environment::kWindSpeedMph;
        float windDirectionDeg = settings::environment::kWindDirectionDeg;
        float launchDirectionDeg = settings::environment::kLaunchDirectionDeg;
        float roughnessLengthMeters = settings::environment::kRoughnessLengthMeters;
        float gradientHeightMeters = settings::environment::kGradientHeightMeters;
        float measurementHeightMeters = settings::environment::kMeasurementHeightMeters;
    };

    EnvironmentModel() { Configure(Config{}); }

    explicit EnvironmentModel(const Config &config) { Configure(config); }

    void Configure(const Config &config) {
        config_ = config;
        initialiseWind();
    }

    double SeaLevelPressurePa() const {
        return static_cast<double>(config_.seaLevelPressureHpa) * 100.0;
    }

    double SeaLevelTemperatureKelvin() const {
        return TemperatureKelvin(0.0);
    }

    // Temperature as a function of altitude (meters).
    double TemperatureKelvin(double altitudeMeters) const {
        const double altitudeFeet = altitudeMeters * constants::kMetersToFeet;
        const double temperatureF = static_cast<double>(config_.groundTemperatureF) - 0.00356 * altitudeFeet;
        return (temperatureF - 32.0) / 1.8 + 273.15;
    }

    // Atmospheric pressure at altitude using barometric formula (Pa).
    double PressurePa(double altitudeMeters) const {
        if (!settings::predictor::kEnableDensityScaling) {
            return SeaLevelPressurePa();
        }
        const double T = TemperatureKelvin(altitudeMeters);
        const double T0 = SeaLevelTemperatureKelvin();
        const double P0 = SeaLevelPressurePa();
        // Barometric formula: P = P0 * (T / T0)^(g / (L * R))
        // With g = 9.80665, R = 287.05, L = 0.0065, exponent ≈ 5.2561 for ISA
        constexpr double kBarometricExponent = 5.2561;
        if (T <= 0.0 || T0 <= 0.0) {
            return P0;
        }
        return P0 * std::pow(T / T0, kBarometricExponent);
    }

    // Atmospheric density at altitude using ideal gas law (kg/m³).
    double DensityKgPerM3(double altitudeMeters) const {
        if (!settings::predictor::kEnableDensityScaling) {
            return static_cast<double>(settings::predictor::kReferenceDensityKgPerM3);
        }
        const double T = TemperatureKelvin(altitudeMeters);
        const double P = PressurePa(altitudeMeters);
        constexpr double kGasConstant = 287.05;  // J/(kg·K) for dry air
        if (T <= 0.0) {
            return static_cast<double>(settings::predictor::kReferenceDensityKgPerM3);
        }
        return P / (kGasConstant * T);
    }

    // Density ratio relative to the CFD table's reference density (dimensionless).
    // Applies pre-flight calibration correction if CalibrateFromMeasurements() was called.
    double DensityRatio(double altitudeMeters) const {
        const double density = DensityKgPerM3(altitudeMeters);
        const double refDensity = static_cast<double>(settings::predictor::kReferenceDensityKgPerM3);
        const double ratio = (refDensity > 0.0) ? (density / refDensity) : 1.0;
        return ratio * static_cast<double>(densityCorrection_);
    }

    // Gradient wind experienced above the boundary layer (m/s).
    math_utils::Vec3 GradientWind() const { return gradientWind_; }

    // Runtime-adjustable wind offset for wind estimation feedback.
    void SetWindOffset(const math_utils::Vec3 &offset) { windOffset_ = offset; }
    math_utils::Vec3 WindOffset() const { return windOffset_; }

    // Total effective wind including runtime offset.
    math_utils::Vec3 EffectiveWind() const {
        return math_utils::MakeVec3(
            gradientWind_.x + windOffset_.x,
            gradientWind_.y + windOffset_.y,
            gradientWind_.z + windOffset_.z);
    }

    // Pre-flight density calibration using actual barometer/temperature readings.
    // Call this on the pad before launch to correct ISA model assumptions.
    void CalibrateFromMeasurements(float measuredPressurePa, float measuredTemperatureK,
                                   float currentAltitudeMeters) {
        constexpr float kGasConstant = 287.05f;
        if (measuredTemperatureK <= 0.0f) {
            return;
        }
        const float actualDensity = measuredPressurePa / (kGasConstant * measuredTemperatureK);
        const float modelDensity = static_cast<float>(DensityKgPerM3(currentAltitudeMeters));
        if (modelDensity > 0.0f && actualDensity > 0.0f) {
            densityCorrection_ = actualDensity / modelDensity;
        }
    }

    // Returns the current density correction factor (1.0 = uncalibrated).
    float DensityCorrection() const { return densityCorrection_; }

    // Resets density calibration to nominal ISA model.
    void ResetDensityCalibration() { densityCorrection_ = 1.0f; }

  private:
    void initialiseWind() {
        const double windSpeedMs = static_cast<double>(config_.windSpeedMph) * constants::kMphToMs;
        const double windDirectionRad = ToRadians(config_.windDirectionDeg);
        const double launchDirectionRad = ToRadians(config_.launchDirectionDeg);
        const double windSin = std::sin(windDirectionRad);
        const double windCos = std::cos(windDirectionRad);
        const double launchSin = std::sin(launchDirectionRad);
        const double launchCos = std::cos(launchDirectionRad);

        const double windVectorX = windSpeedMs * windCos;
        const double windVectorY = windSpeedMs * windSin;
        const double launchUnitX = launchCos;
        const double launchUnitY = launchSin;
        const double windDownrange = windVectorX * launchUnitX + windVectorY * launchUnitY;

        const double numerator = std::log(static_cast<double>(config_.gradientHeightMeters) /
                                          static_cast<double>(config_.roughnessLengthMeters));
        const double denominator = std::log(static_cast<double>(config_.measurementHeightMeters) /
                                            static_cast<double>(config_.roughnessLengthMeters));
        double gradientSpeed = 0.0;
        if (denominator != 0.0) {
            gradientSpeed = windDownrange * numerator / denominator;
        }

        gradientWind_ = math_utils::MakeVec3(0.0f, static_cast<float>(gradientSpeed), 0.0f);
    }

    static double ToRadians(double degrees) {
        return degrees * 0.017453292519943295;
    }

    Config config_;
    math_utils::Vec3 gradientWind_ = math_utils::MakeVec3(0.0f, 0.0f, 0.0f);
    math_utils::Vec3 windOffset_ = math_utils::MakeVec3(0.0f, 0.0f, 0.0f);
    float densityCorrection_ = 1.0f;  // Pre-flight calibration factor (1.0 = ISA model)
};
