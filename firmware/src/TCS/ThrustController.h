#pragma once

#include "MotorController.h"

class TCS {
  public:
    TCS(int pinA, int pinB, int pinC, int pinD, int minPulsewidth,
        int maxPulsewidth);

    void arm();
    void armMotor(int motor);
    void disarm();
    void disarmMotor(int motor);
    void throttle(int motor, float throttle);
    void stop();
    void test();

    float getThrottle(int motor);
    bool isArmed(int motor);

  private:
    MCU engineA;
    MCU engineB;
    MCU engineC;
    MCU engineD;

    MCU intToEngine(int motor);
};
