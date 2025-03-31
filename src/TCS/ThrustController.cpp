#include "ESCDriver.h"
#include "ThrustController.h"

TCS::TCS(int pinA, int pinB, int pinC, int pinD)
    : pinA(pinA), pinB(pinB), pinC(pinC), pinD(pinD) {};

void TCS::begin() {
    ESCDriver escA(pinA, 1);
    ESCDriver escB(pinB, 2);
    ESCDriver escC(pinC, 3);
    ESCDriver escD(pinD, 4);

    m1 = MCU(&escA);
    m2 = MCU(&escB);
    m3 = MCU(&escC);
    m4 = MCU(&escD);

    disarm();
}

void TCS::arm() {
    m1.arm();
    m2.arm();
    m3.arm();
    m4.arm();

    armed = true;
}

void TCS::disarm() {
    m1.disarm();
    m2.disarm();
    m3.disarm();
    m4.disarm();

    armed = false;
}

int TCS::setThrottle(int motor, float throttle) {
    MCU engine = nullptr;

    switch (motor) {
    case 1:
        engine = m1;
        break;
    case 2:
        engine = m2;
        break;
    case 3:
        engine = m3;
        break;
    case 4:
        engine = m4;
        break;
    default:
        return -1; // Invalid motor selection
    }

    engine.setThrottle(throttle);
    return 0; // SUCCESS
}

TCSState TCS::getTCSState() {
    TCSState state{};
    state.armed = armed;

    state.throttleA = m1.getThrottle();
    state.throttleB = m2.getThrottle();
    state.throttleC = m3.getThrottle();
    state.throttleD = m4.getThrottle();

    return state;
};
