#pragma once

#include <Arduino.h>

#if defined(DEBUG) || defined(VERBOSE)
#define DBG(msg) Serial.print(msg)
#define DBG_FMT(fmt, ...) Serial.printf((fmt), ##__VA_ARGS__)
#else
#define DBG(msg)
#define DBG_FMT(fmt, ...)
#endif

#if defined(CRITICAL_DEBUG) || defined(DEBUG) || defined(VERBOSE)
#define DBG_BEGIN(speed) Serial.begin(speed)
#define DBGCRT(msg) Serial.print(msg)
#define DBGCRT_FMT(fmt, ...) Serial.printf((fmt), ##__VA_ARGS__)
#else
#define DBG_BEGIN(speed)
#define DBGCRT(msg)
#define DBGCRT_FMT(fmt, ...)
#endif
