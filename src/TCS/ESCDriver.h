#pragma once

#include <cstdint>

class ESCDriver {
  public:
    ESCDriver(int pwmPin, uint8_t channel);

    void begin();
    void arm();
    void disarm();
    void write(float pulseWidth);

  private:
    int pwmPin;
    uint8_t channel;
    bool isArmed = false;

    // NOTE: Switch to DShot ??
    int frequency = 50;  ///< PWM frequency.
    int resolution = 16; ///< PWM resolution.

    float pulseWidthToDutyCycle(float pulseWidth) const;
};
