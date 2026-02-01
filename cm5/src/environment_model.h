#pragma once

#include "constants.h"
#include "math_utils.h"

/// \file environment_model.h
/// \brief Environment model helpers for wind and temperature.

/// \brief Encapsulates the logic in environment.py for use at runtime.
class EnvironmentModel {
  public:
    /// \brief Configuration values for the environment model.
    struct Config {
        /// \brief Ground temperature in Fahrenheit.
        float groundTemperatureF = 50.0f;
        /// \brief Wind speed in miles per hour.
        float windSpeedMph = 10.0f;
        /// \brief Wind direction in degrees.
        float windDirectionDeg = 270.0f;
        /// \brief Launch direction in degrees.
        float launchDirectionDeg = 260.0f;
        /// \brief Surface roughness length in meters.
        float roughnessLengthMeters = 0.075f;
        /// \brief Gradient height in meters.
        float gradientHeightMeters = 300.0f;
        /// \brief Height of wind measurement in meters.
        float measurementHeightMeters = 10.0f;
    };

    /// \brief Construct with default configuration.
    EnvironmentModel() { Configure(Config{}); }

    /// \brief Construct with a specific configuration.
    /// \param config Environment parameters to apply.
    explicit EnvironmentModel(const Config &config) { Configure(config); }

    /// \brief Apply a new configuration and recompute wind parameters.
    /// \param config Environment parameters to apply.
    void Configure(const Config &config) {
        config_ = config;
        initialiseWind();
    }

    /// \brief Temperature as a function of altitude.
    /// \param altitudeMeters Altitude above ground level in meters.
    /// \return Temperature in Kelvin.
    float TemperatureKelvin(float altitudeMeters) const {
        const float altitudeFeet = altitudeMeters * constants::kMetersToFeet;
        const float temperatureF = config_.groundTemperatureF - 0.00356f * altitudeFeet;
        return constants::FahrenheitToKelvin(temperatureF);
    }

    /// \brief Gradient wind experienced above the boundary layer.
    /// \return Wind vector in meters per second.
    math_utils::Vec3 GradientWind() const { return gradientWind_; }

  private:
    /// \brief Compute gradient wind based on configuration values.
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

    /// \brief Convert degrees to radians.
    /// \param degrees Angle in degrees.
    /// \return Angle in radians.
    static float ToRadians(float degrees) {
        return degrees * 0.017453292519943295f;
    }

    /// \brief Cached configuration.
    Config config_;
    /// \brief Cached gradient wind vector.
    math_utils::Vec3 gradientWind_ = math_utils::MakeVec3(0.0f, 0.0f, 0.0f);
};
