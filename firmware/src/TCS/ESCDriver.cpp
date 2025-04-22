#include "ESCDriver.h"
#include "utils/log.h"
#include <cstdint>

ESCDriver::ESCDriver(uint8_t pwmPin, uint8_t channel, uint16_t minPulsewidth)
    : pwmPin(pwmPin), channel(channel), minPulsewidth(minPulsewidth) {

    ledcSetup(channel, frequency, resolution);
    ledcAttachPin(pwmPin, channel);
};

void ESCDriver::write(uint16_t pulsewidth) const {
    DBG_FMT("[ESC] Writing pulsewidth %f to channel %d\n", pulsewidth, channel);
    ledcWrite(channel, pulseWidthToDutyCycle(pulsewidth));
}

void ESCDriver::stop() const {
    DBG_FMT("[ESC] Stopping channel %d\n", channel);
    ledcWrite(channel, pulseWidthToDutyCycle(minPulsewidth));
}

uint32_t ESCDriver::pulseWidthToDutyCycle(uint16_t pulsewidth_us) {
    uint32_t maxDuty = (1UL << resolution) - 1; // 2^resolution - 1
    return map(pulsewidth_us, 0, 20000, 0, maxDuty);
}
