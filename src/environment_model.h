#pragma once

#include "constants.h"
#include "math_utils.h"

// Encapsulates the logic in environment.py for use at runtime.
class EnvironmentModel {
  public:
    struct Config {
        float groundTemperatureF = 45.0f;
        float windSpeedMph = 25.0f;
        float windDirectionDeg = 310.0f;
        float launchDirectionDeg = 130.0f;
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
    float TemperatureKelvin(float altitudeMeters) const {
        const float altitudeFeet = altitudeMeters * constants::kMetersToFeet;
        const float temperatureF = config_.groundTemperatureF - 0.00356f * altitudeFeet;
        return constants::FahrenheitToKelvin(temperatureF);
    }

    // Gradient wind experienced above the boundary layer (m/s).
    math_utils::Vec3 GradientWind() const { return gradientWind_; }

  private:
    void initialiseWind() {
        const float windSpeedMs = config_.windSpeedMph * constants::kMphToMs;
        const float windDirectionRad = ToRadians(config_.windDirectionDeg);
        const float launchDirectionRad = ToRadians(config_.launchDirectionDeg);

        const float windVectorX = windSpeedMs * cosf(windDirectionRad);
        const float windVectorY = windSpeedMs * sinf(windDirectionRad);
        const float launchUnitX = cosf(launchDirectionRad);
        const float launchUnitY = sinf(launchDirectionRad);
        const float windDownrange = windVectorX * launchUnitX + windVectorY * launchUnitY;

        const float numerator = logf(config_.gradientHeightMeters / config_.roughnessLengthMeters);
        const float denominator = logf(config_.measurementHeightMeters / config_.roughnessLengthMeters);
        float gradientSpeed = 0.0f;
        if (denominator != 0.0f) {
            gradientSpeed = windDownrange * numerator / denominator;
        }

        gradientWind_ = math_utils::MakeVec3(0.0f, gradientSpeed, 0.0f);
    }

    static float ToRadians(float degrees) {
        return degrees * 0.017453292519943295f;
    }

    Config config_;
    math_utils::Vec3 gradientWind_ = math_utils::MakeVec3(0.0f, 0.0f, 0.0f);
};
