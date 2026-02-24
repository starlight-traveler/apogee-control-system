#pragma once

#include <Arduino.h>

#include "flight_computer.h"

void StatusLedsBegin();
void StatusLedsSetFault(bool active);
void StatusLedsSetFlightStatus(FlightStatus status);
void StatusLedsSetComms(bool wifiConnected, bool subscriberActive);
void StatusLedsSetManualOverride(bool active);
void StatusLedsService(uint32_t nowMs);
