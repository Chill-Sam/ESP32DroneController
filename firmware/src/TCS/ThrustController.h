#pragma once

#include "MotorController.h"
#include <cstdint>

class TCS {
  public:
    TCS(uint8_t pinA, uint8_t pinB, uint8_t pinC, uint8_t pinD,
        uint16_t minPulsewidth_us, uint16_t maxPulsewidth_us);

    void arm(int8_t motor = -1);
    void disarm(int8_t motor = -1);

    void throttle(uint8_t motor, float throttle);
    void stop();
    void test();

    const MCU &motor(uint8_t motor) const;

  private:
    MCU motors[4];
};
