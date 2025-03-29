#include "Arduino.h"
#include "ESCDriver.h"
#include "MotorController.h"

MCU::MCU(ESCDriver *esc, float minThrottle, float maxThrottle)
    : esc(esc), minThrottle(minThrottle), maxThrottle(maxThrottle) {}

void MCU::begin() {
    esc->begin();
    disarm();
}

void MCU::arm() {
    esc->arm();
    esc->write(minThrottle);
    armed = true;
}

void MCU::disarm() {
    esc->disarm();
    currentThrottle = 0.0F;
    armed = false;
}

void MCU::setThrottle(float value) {
    if (!armed) {
        return;
    }
    value = constrain(value, 0.0F, 1.0F);
    value = minThrottle + value * (maxThrottle - minThrottle);
    currentThrottle = value;
    auto pulseWidth = static_cast<float>(
        map(static_cast<long>(value), 0, 100, minPulseWidth, maxPulseWidth));
    esc->write(pulseWidth);
}

float MCU::getThrottle() const { return currentThrottle; }

bool MCU::isArmed() const { return armed; }
