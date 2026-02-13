#pragma once

// Lightweight equivalents to constants.py for use on the Teensy flight computer.
namespace constants {

constexpr double kGravity = 9.8067;            // [m/s^2]
constexpr double kGamma = 1.4;                 // Ratio of specific heats for air
constexpr double kGasConstant = 287.05;        // [J/(kg·K)] specific gas constant
constexpr double kMetersToFeet = 3.28083989501;
constexpr double kFeetToMeters = 1.0 / kMetersToFeet;
constexpr double kFahrenheitToKelvinScale = 5.0 / 9.0;
constexpr double kFahrenheitToKelvinOffset = 273.15 - 32.0 * kFahrenheitToKelvinScale;
constexpr double kMphToMs = 0.44704;

// Convert Fahrenheit to Kelvin.
inline double FahrenheitToKelvin(double temperatureF) {
    return temperatureF * kFahrenheitToKelvinScale + kFahrenheitToKelvinOffset;
}

}  // namespace constants
