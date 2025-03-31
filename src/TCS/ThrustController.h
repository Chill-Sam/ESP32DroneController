#pragma once

#include "MotorController.h"

struct TCSState {
    bool armed;

    float throttleA;
    float throttleB;
    float throttleC;
    float throttleD;
};

class TCS {
  public:
    TCS(int pinA, int pinB, int pinC, int pinD);

    void begin();
    void arm();
    void disarm();
    int setThrottle(int motor, float throttle);

    void testEngines(float throttle = 0.2);

    TCSState getTCSState();

  private:
    bool armed = false;

    int pinA;
    int pinB;
    int pinC;
    int pinD;

    MCU m1 = nullptr;
    MCU m2 = nullptr;
    MCU m3 = nullptr;
    MCU m4 = nullptr;

    float throttleA = 0;
    float throttleB = 0;
    float throttleC = 0;
    float throttleD = 0;
};
