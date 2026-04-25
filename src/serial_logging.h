#pragma once

#include <Arduino.h>

/*
 * Serial logging facade.
 *
 * Flight code calls LOG_* instead of Serial.* directly so debug printing can be
 * compiled out with one flag. That is useful when timing is tight or when USB
 * serial is not connected during launch operations.
 */
// Set to 0 (or pass -DENABLE_SERIAL_LOGGING=0 in build flags) to strip all debug logs.
#ifndef ENABLE_SERIAL_LOGGING
#define ENABLE_SERIAL_LOGGING 1
#endif

#if ENABLE_SERIAL_LOGGING
#define LOG_BEGIN(...) Serial.begin(__VA_ARGS__)
#define LOG_PRINT(...) Serial.print(__VA_ARGS__)
#define LOG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define LOG_BEGIN(...) \
    do {               \
    } while (0)
#define LOG_PRINT(...) \
    do {               \
    } while (0)
#define LOG_PRINTLN(...) \
    do {                 \
    } while (0)
#endif
