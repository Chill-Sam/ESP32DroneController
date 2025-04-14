#pragma once

#include "ESCDriver.h"

class MCU {
  public:
    MCU(int pin, int channel, int minPulsewidth, int maxPulsewidth);

    void arm();
    void disarm();
    void setThrottle(float throttle);
    void stop();
    void test();

    bool armed = false;

  private:
    int minPulsewidth;
    int maxPulsewidth;

    int channel;

    ESCDriver esc;
};
