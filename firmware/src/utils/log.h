#pragma once

#include <Arduino.h>

#if defined(DEBUG) || defined(VERBOSE)
#define DBG_BEGIN(speed) Serial.begin(speed)
#define DBG(msg) Serial.print(msg)
#define DBG_FMT(fmt, ...) Serial.printf((fmt), ##__VA_ARGS__)
#else
#define DBG_BEGIN(speed)
#define DBG(msg)
#define DBG_FMT(fmt, ...)
#endif
