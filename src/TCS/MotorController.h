#pragma once

#include "ESCDriver.h"

class MCU {
  public:
    MCU(ESCDriver *esc, float minThrottle = 0.05F, float maxThrottle = 1.0F);

    void begin();
    void arm();
    void disarm();
    void setThrottle(float value);
    float getThrottle() const;
    bool isArmed() const;

  private:
    ESCDriver *esc;
    float currentThrottle = 0.0F;
    float minThrottle;
    float maxThrottle;
    bool armed = false;

    int minPulseWidth = 1000;
    int maxPulseWidth = 2000;
};
