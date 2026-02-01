#pragma once

#include <Arduino.h>

#include "telemetry_types.h"

bool SerialLinkBegin(uint32_t baud);
void SerialLinkProcessInput();
void SerialLinkSetStreamRateHz(float rateHz);
void SerialLinkSetStreamEnabled(bool enabled);
bool SerialLinkShouldSend(uint32_t nowMicros);
bool SerialLinkSendTelemetry(const SensorData &data);
bool SerialLinkHandleCustomCommand(uint8_t commandId, const uint8_t *payload, uint16_t length);
