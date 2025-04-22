#pragma once

#include <cstdint>

class ESCDriver {
  public:
    ESCDriver(uint8_t pwmPin, uint8_t channel, uint16_t minPulsewidth_us);

    void write(uint16_t pulsewidth_us) const;
    void stop() const;

  private:
    const uint8_t pwmPin;
    const uint8_t channel;
    const uint16_t minPulsewidth_us;

    static constexpr uint16_t frequency = 50;
    static constexpr uint8_t resolution = 16;

    static uint32_t pulseWidthToDutyCycle(uint16_t pulsewidth_us);
};
