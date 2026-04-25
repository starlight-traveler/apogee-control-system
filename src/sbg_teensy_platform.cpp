#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "serial_logging.h"

extern "C" {
#include <sbgCommon.h>
}

namespace {

SbgCommonLibOnLogFunc g_logCallback = nullptr;
uint32_t g_suppressedParserLogCount = 0;
uint32_t g_lastSuppressedParserLogMs = 0;

const char *LogTypePrefix(SbgDebugLogType logType) {
    switch (logType) {
        case SBG_DEBUG_LOG_TYPE_ERROR:
            return "ERR";
        case SBG_DEBUG_LOG_TYPE_WARNING:
            return "WRN";
        case SBG_DEBUG_LOG_TYPE_INFO:
            return "INF";
        case SBG_DEBUG_LOG_TYPE_DEBUG:
            return "DBG";
    }
    return "LOG";
}

bool StartsWith(const char *text, const char *prefix) {
    if (text == nullptr || prefix == nullptr) {
        return false;
    }
    const size_t prefixLen = strlen(prefix);
    return strncmp(text, prefix, prefixLen) == 0;
}

bool IsNoisyProtocolLog(const char *pFileName, SbgDebugLogType logType, const char *message) {
    if (logType != SBG_DEBUG_LOG_TYPE_ERROR && logType != SBG_DEBUG_LOG_TYPE_WARNING) {
        return false;
    }
    if (pFileName == nullptr || message == nullptr) {
        return false;
    }
    if (strstr(pFileName, "sbgEComProtocol.c") == nullptr) {
        return false;
    }
    return StartsWith(message, "invalid end-of-frame") ||
           StartsWith(message, "invalid payload size") ||
           StartsWith(message, "invalid CRC") ||
           StartsWith(message, "reserved bits set in extended headers") ||
           StartsWith(message, "invalid page information");
}

void FlushSuppressedParserLogSummary(uint32_t nowMs) {
    if (g_suppressedParserLogCount == 0) {
        return;
    }
    if ((nowMs - g_lastSuppressedParserLogMs) < 1000u) {
        return;
    }
    LOG_PRINT("SBG WRN: suppressed ");
    LOG_PRINT(g_suppressedParserLogCount);
    LOG_PRINTLN(" protocol parser errors");
    g_suppressedParserLogCount = 0;
    g_lastSuppressedParserLogMs = nowMs;
}

}  // namespace

extern "C" uint32_t sbgGetTime(void) {
    return millis();
}

extern "C" void sbgSleep(uint32_t ms) {
    delay(ms);
}

extern "C" void sbgCommonLibSetLogCallback(SbgCommonLibOnLogFunc logCallback) {
    g_logCallback = logCallback;
}

extern "C" void sbgPlatformDebugLogMsg(const char *pFileName,
                                        const char *pFunctionName,
                                        uint32_t line,
                                        const char *pCategory,
                                        SbgDebugLogType logType,
                                        SbgErrorCode errorCode,
                                        const char *pFormat,
                                        ...) {
    char message[192];
    va_list args;
    va_start(args, pFormat);
    vsnprintf(message, sizeof(message), pFormat, args);
    va_end(args);

    const uint32_t nowMs = millis();

    if (IsNoisyProtocolLog(pFileName, logType, message)) {
        ++g_suppressedParserLogCount;
        FlushSuppressedParserLogSummary(nowMs);
        return;
    }

    if (g_logCallback) {
        g_logCallback(pFileName, pFunctionName, line, pCategory, logType, errorCode, message);
        return;
    }

    FlushSuppressedParserLogSummary(nowMs);

    LOG_PRINT("SBG ");
    LOG_PRINT(LogTypePrefix(logType));
    LOG_PRINT(": ");
    LOG_PRINT(message);
    if (errorCode != SBG_NO_ERROR) {
        LOG_PRINT(" (");
        LOG_PRINT(static_cast<int>(errorCode));
        LOG_PRINT(")");
    }
    LOG_PRINTLN("");
}
