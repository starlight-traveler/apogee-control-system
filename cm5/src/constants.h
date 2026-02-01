#pragma once

/// \file constants.h
/// \brief Physical constants and unit conversions shared across cm5.

/// \brief Lightweight equivalents to constants.py for use on the Teensy flight computer.
namespace constants {

/// \brief Standard gravity in meters per second squared.
constexpr float kGravity = 9.8067f;  // [m/s^2]
/// \brief Ratio of specific heats for air.
constexpr float kGamma = 1.4f;
/// \brief Specific gas constant for air.
constexpr float kGasConstant = 287.05f;  // [J/(kg*K)]
/// \brief Conversion multiplier from meters to feet.
constexpr float kMetersToFeet = 3.28083989501f;
/// \brief Conversion multiplier from feet to meters.
constexpr float kFeetToMeters = 1.0f / kMetersToFeet;
/// \brief Scaling factor for Fahrenheit to Kelvin conversion.
constexpr float kFahrenheitToKelvinScale = 5.0f / 9.0f;
/// \brief Offset for Fahrenheit to Kelvin conversion.
constexpr float kFahrenheitToKelvinOffset = 273.15f - 32.0f * kFahrenheitToKelvinScale;
/// \brief Conversion multiplier from miles per hour to meters per second.
constexpr float kMphToMs = 0.44704f;

/// \brief Convert Fahrenheit temperature to Kelvin.
/// \param temperatureF Temperature in degrees Fahrenheit.
/// \return Temperature in Kelvin.
inline float FahrenheitToKelvin(float temperatureF) {
    return temperatureF * kFahrenheitToKelvinScale + kFahrenheitToKelvinOffset;
}

}  // namespace constants
