#pragma once

#include "MotorController.h"
#include "types/FlightData.h"
#include <cstdint>

class TCS {
  public:
    TCS(uint8_t pinA, uint8_t pinB, uint8_t pinC, uint8_t pinD,
        uint16_t minPulsewidth_us, uint16_t maxPulsewidth_us);

    void begin();

    void arm(int8_t motor = -1);
    void disarm(int8_t motor = -1);

    void throttle(uint8_t motor, float throttle);
    void stop();
    void test();

    const ThrottleState &throttleState;
    const ArmState &armState;

  private:
    ThrottleState _throttleState;
    ArmState _armState;

    void updateState();

    MCU motors[4];
};
