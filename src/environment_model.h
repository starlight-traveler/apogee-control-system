#pragma once

#include <cmath>

#include "constants.h"
#include "math_utils.h"
#include "settings.h"

/*
 * This model is intentionally lightweight. It is not trying to be a full weather
 * forecast; it gives the apogee predictor the pieces that most affect drag:
 *
 *   - air density changes with altitude,
 *   - speed of sound changes with temperature,
 *   - wind changes the rocket's air-relative velocity,
 *   - launch-day pressure/temperature can scale density away from nominal.
 *
 * The predictor cares about air-relative motion, not GPS/ground-relative motion,
 * because drag is produced by motion through the air.
 */

/**
 * @brief runtime atmosphere and wind model used by the apogee predictor.
 *
 * this is the firmware version of the old python environment model. it turns
 * pad weather and wind settings into density, pressure, temperature, and a
 * downrange wind vector that the predictor can use while integrating coast.
 */
class EnvironmentModel {
  public:
    /**
     * @brief pad/weather configuration for one flight.
     *
     * these values can come from flashed defaults or runtime telemetry
     * settings. they are intentionally simple because they are used onboard.
     */
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

    /// @brief constructs the model using compile-time defaults.
    EnvironmentModel() { Configure(Config{}); }

    /// @brief constructs the model using an explicit pad/weather config.
    explicit EnvironmentModel(const Config &config) { Configure(config); }

    /**
     * @brief applies a new environment config and recomputes wind.
     *
     * changing wind/weather here changes what the predictor thinks the air is
     * doing, but it does not reset the flight computer.
     */
    void Configure(const Config &config) {
        config_ = config;
        initialiseWind();
    }

    /// @brief sea-level pressure reference in pascals.
    double SeaLevelPressurePa() const {
        return static_cast<double>(config_.seaLevelPressureHpa) * 100.0;
    }

    /// @brief model temperature at zero altitude in kelvin.
    double SeaLevelTemperatureKelvin() const {
        return TemperatureKelvin(0.0);
    }

    /// @brief temperature as a function of altitude in meters.
    double TemperatureKelvin(double altitudeMeters) const {
        const double altitudeFeet = altitudeMeters * constants::kMetersToFeet;
        const double temperatureF = static_cast<double>(config_.groundTemperatureF) - 0.00356 * altitudeFeet;
        return (temperatureF - 32.0) / 1.8 + 273.15;
    }

    /// @brief atmospheric pressure at altitude using the barometric formula.
    double PressurePa(double altitudeMeters) const {
        if (!settings::predictor::kEnableDensityScaling) {
            return SeaLevelPressurePa();
        }
        const double T = TemperatureKelvin(altitudeMeters);
        const double T0 = SeaLevelTemperatureKelvin();
        const double P0 = SeaLevelPressurePa();
        // pressure drops as altitude rises; this exponent is the compact
        // standard-atmosphere approximation for that curve.
        constexpr double kBarometricExponent = 5.2561;
        if (T <= 0.0 || T0 <= 0.0) {
            return P0;
        }
        return P0 * std::pow(T / T0, kBarometricExponent);
    }

    /// @brief atmospheric density at altitude using the ideal gas law.
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

    /**
     * @brief density ratio relative to the CFD table reference density.
     *
     * the cfd table is built for one reference air density. this scales the
     * table up or down for the actual air on launch day.
     */
    double DensityRatio(double altitudeMeters) const {
        /*
         * CFD force tables are usually generated at a reference density. Dynamic
         * pressure, and therefore aerodynamic force, scales roughly with density,
         * so this ratio lets the same table work on a hotter/colder launch day.
         */
        const double density = DensityKgPerM3(altitudeMeters);
        const double refDensity = static_cast<double>(settings::predictor::kReferenceDensityKgPerM3);
        const double ratio = (refDensity > 0.0) ? (density / refDensity) : 1.0;
        return ratio * static_cast<double>(densityCorrection_);
    }

    /// @brief modeled downrange wind at gradient height in meters per second.
    math_utils::Vec3 GradientWind() const { return gradientWind_; }

    /// @brief stores a runtime wind correction learned during coast.
    void SetWindOffset(const math_utils::Vec3 &offset) { windOffset_ = offset; }
    /// @brief returns the runtime wind correction learned during coast.
    math_utils::Vec3 WindOffset() const { return windOffset_; }

    /// @brief total wind used by the predictor.
    math_utils::Vec3 EffectiveWind() const {
        return math_utils::MakeVec3(
            gradientWind_.x + windOffset_.x,
            gradientWind_.y + windOffset_.y,
            gradientWind_.z + windOffset_.z);
    }

    /**
     * @brief calibrates density from measured pressure and temperature.
     *
     * if the real air is thinner or thicker than the simple model says, this
     * stores one correction factor so drag predictions start closer.
     */
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

    /// @brief returns the current density correction factor.
    float DensityCorrection() const { return densityCorrection_; }

    /// @brief resets density calibration to the nominal atmosphere model.
    void ResetDensityCalibration() { densityCorrection_ = 1.0f; }

  private:
    /// @brief computes the downrange wind component seen by the predictor.
    void initialiseWind() {
        /*
         * The onboard apogee model is 2D: vertical plus one horizontal/downrange
         * axis. A real wind has compass direction, so project it onto the launch
         * direction and ignore crossrange wind. That keeps the predictor small
         * while still capturing headwind/tailwind effects on drag and AoA.
         */
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
        // only the wind along the launch/downrange plane matters to this 2d
        // predictor, so project the measured wind onto that line.
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

    /// @brief converts degrees to radians.
    static double ToRadians(double degrees) {
        return degrees * 0.017453292519943295;
    }

    Config config_;
    math_utils::Vec3 gradientWind_ = math_utils::MakeVec3(0.0f, 0.0f, 0.0f);
    math_utils::Vec3 windOffset_ = math_utils::MakeVec3(0.0f, 0.0f, 0.0f);
    float densityCorrection_ = 1.0f;  // Pre-flight calibration factor (1.0 = ISA model)
};
