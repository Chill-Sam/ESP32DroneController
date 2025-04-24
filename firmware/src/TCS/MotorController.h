#pragma once

#include "ESCDriver.h"
#include <cstdint>

class MCU {
  public:
    MCU(uint8_t pin, uint8_t channel, uint16_t minPulsewidth_us,
        uint16_t maxPulsewidth_us);

    const bool &armed;
    const float &currentThrottle;

    void begin();
    void arm();
    void disarm();
    void setThrottle(float throttle); // [0.0 - 100.0]
    void stop();

  private:
    bool _armed = false;
    float _currentThrottle = 0.0F;

    const uint16_t minPulsewidth_us;
    const uint16_t maxPulsewidth_us;
    const uint8_t channel;

    ESCDriver esc;
};
