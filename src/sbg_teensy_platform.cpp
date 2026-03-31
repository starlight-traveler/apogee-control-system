#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

#include "serial_logging.h"

extern "C" {
#include <sbgCommon.h>
}

namespace {

SbgCommonLibOnLogFunc g_logCallback = nullptr;

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

    if (g_logCallback) {
        g_logCallback(pFileName, pFunctionName, line, pCategory, logType, errorCode, message);
        return;
    }

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
