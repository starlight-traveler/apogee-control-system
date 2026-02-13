#pragma once

#include <cmath>

#include "constants.h"
#include "math_utils.h"

// Encapsulates the logic in environment.py for use at runtime.
class EnvironmentModel {
  public:
    struct Config {
        float groundTemperatureF = 50.0f;
        float windSpeedMph = 10.0f;
        float windDirectionDeg = 270.0f;
        float launchDirectionDeg = 260.0f;
        float roughnessLengthMeters = 0.075f;
        float gradientHeightMeters = 300.0f;
        float measurementHeightMeters = 10.0f;
    };

    EnvironmentModel() { Configure(Config{}); }

    explicit EnvironmentModel(const Config &config) { Configure(config); }

    void Configure(const Config &config) {
        config_ = config;
        initialiseWind();
    }

    // Temperature as a function of altitude (meters).
    double TemperatureKelvin(double altitudeMeters) const {
        const double altitudeFeet = altitudeMeters * constants::kMetersToFeet;
        const double temperatureF = static_cast<double>(config_.groundTemperatureF) - 0.00356 * altitudeFeet;
        return (temperatureF - 32.0) / 1.8 + 273.15;
    }

    // Gradient wind experienced above the boundary layer (m/s).
    math_utils::Vec3 GradientWind() const { return gradientWind_; }

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
};
