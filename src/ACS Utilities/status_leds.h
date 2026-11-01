#pragma once

#include <Arduino.h>

#include "flight_computer.h"

/**
 * @brief small status-light facade for launch-day visibility.
 *
 * The LEDs are intentionally driven from summarized state instead of reaching
 * into sensors directly. That keeps indicator behavior from changing flight
 * logic, and it makes LED failures a diagnostic problem instead of a control
 * problem.
 */
/// @brief initializes status led hardware.
void StatusLedsBegin();
/// @brief marks whether the system should show a fault pattern.
void StatusLedsSetFault(bool active);
/// @brief updates the flight-phase color source.
void StatusLedsSetFlightStatus(FlightStatus status);
/// @brief updates wifi/subscriber indicators.
void StatusLedsSetComms(bool wifiConnected, bool subscriberActive);
/// @brief updates the manual override indicator.
void StatusLedsSetManualOverride(bool active);
/// @brief advances blinking/fade behavior.
void StatusLedsService(uint32_t nowMs);
