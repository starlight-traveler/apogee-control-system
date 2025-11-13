#pragma once

// Lightweight equivalents to constants.py for use on the Teensy flight computer.
namespace constants {

constexpr float kGravity = 9.8067f;            // [m/s^2]
constexpr float kGamma = 1.4f;                 // Ratio of specific heats for air
constexpr float kGasConstant = 287.05f;        // [J/(kg·K)] specific gas constant
constexpr float kMetersToFeet = 3.28083989501f;
constexpr float kFeetToMeters = 1.0f / kMetersToFeet;
constexpr float kFahrenheitToKelvinScale = 5.0f / 9.0f;
constexpr float kFahrenheitToKelvinOffset = 273.15f - 32.0f * kFahrenheitToKelvinScale;
constexpr float kMphToMs = 0.44704f;

// Convert Fahrenheit to Kelvin.
inline float FahrenheitToKelvin(float temperatureF) {
    return temperatureF * kFahrenheitToKelvinScale + kFahrenheitToKelvinOffset;
}

}  // namespace constants
