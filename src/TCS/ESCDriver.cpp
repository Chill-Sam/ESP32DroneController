#include "ESCDriver.h"
#include <Arduino.h>

ESCDriver::ESCDriver(int pwmPin, uint8_t channel)
    : pwmPin(pwmPin), channel(channel) {};

float ESCDriver::pulseWidthToDutyCycle(float pulseWidth) const {
    auto maxDutyCycle = static_cast<float>(pow(2, resolution) - 1);
    return static_cast<float>(map(static_cast<long>(pulseWidth), 0, 20000, 0,
                                  static_cast<long>(maxDutyCycle)));
}

void ESCDriver::begin() const {
    ledcSetup(channel, frequency, resolution);
    ledcAttachPin(pwmPin, channel);
}

void ESCDriver::write(float pulseWidth) {
    ledcWrite(channel,
              static_cast<uint32_t>(pulseWidthToDutyCycle(pulseWidth)));
}

void ESCDriver::arm() { isArmed = true; }

void ESCDriver::disarm() { isArmed = false; }
