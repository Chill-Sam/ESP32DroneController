#include "MotorController.h"
#include "utils/log.h"
#include <cstdint>

MCU::MCU(uint8_t pin, uint8_t channel, uint16_t minPulsewidth_us,
         uint16_t maxPulsewidth_us)
    : esc(pin, channel, minPulsewidth_us), channel(channel),
      minPulsewidth_us(minPulsewidth_us), maxPulsewidth_us(maxPulsewidth_us),
      armed(_armed), currentThrottle(_currentThrottle) {}

void MCU::arm() {
    DBG_FMT("[MCU] Arming engine %d\n", channel);
    esc.stop();
    _armed = true;
}

void MCU::disarm() {
    DBG_FMT("[MCU] Disarming engine %d\n", channel);
    esc.stop();
    _armed = false;
}

void MCU::setThrottle(float throttle) {
    if (!armed) {
        DBG_FMT("[MCU] cannot throttle engine %d (Disarmed)\n", channel);
        stop();
        return;
    }

    DBG_FMT("[MCU] Setting engine %d to %f% throttle\n", channel, throttle);
    uint32_t pulsewidth =
        map(throttle, 0, 100, minPulsewidth_us, maxPulsewidth_us);
    esc.write(pulsewidth);
    _currentThrottle = throttle;
}

void MCU::stop() {
    DBG_FMT("[MCU] Stopping engine %d\n", channel);
    esc.stop();
    _currentThrottle = 0.0F;
}
