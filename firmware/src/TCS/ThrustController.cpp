#include "Arduino.h"
#include "ThrustController.h"
#include "utils/log.h"

TCS::TCS(int pinA, int pinB, int pinC, int pinD, int minPulsewidth,
         int maxPulsewidth)
    : engineA(pinA, 1, minPulsewidth, maxPulsewidth),
      engineB(pinB, 2, minPulsewidth, maxPulsewidth),
      engineC(pinC, 3, minPulsewidth, maxPulsewidth),
      engineD(pinD, 4, minPulsewidth, maxPulsewidth) {}

void TCS::arm() {
    DBG("[TCS] Arming all engines\n");
    engineA.arm();
    engineB.arm();
    engineC.arm();
    engineD.arm();
}

void TCS::armMotor(int motor) {
    DBG_FMT("[TCS] Arming engine %d\n", motor);
    MCU engine = intToEngine(motor);
    engine.arm();
}

void TCS::disarm() {
    DBG("[TCS] Disarming all engines\n");
    engineA.disarm();
    engineB.disarm();
    engineC.disarm();
    engineD.disarm();
}

void TCS::disarmMotor(int motor) {
    DBG_FMT("[TCS] Disarming engine %d\n", motor);
    MCU engine = intToEngine(motor);
    engine.disarm();
}

void TCS::throttle(int motor, float throttle) {
    DBG_FMT("[TCS] Setting %f% throttle for engine %d", throttle, motor);
    MCU engine = intToEngine(motor);
    engine.setThrottle(throttle);
}

void TCS::stop() {
    engineA.stop();
    engineB.stop();
    engineC.stop();
    engineD.stop();
}

void TCS::test() {
    DBG("[TCS] Testing\n");
    arm();
    delay(5000);
    throttle(1, 20);
    delay(1000);
    throttle(2, 20);
    delay(1000);
    throttle(3, 20);
    delay(1000);
    throttle(4, 20);
    delay(1000);
    stop();
    delay(2500);
    throttle(1, 20);
    throttle(2, 20);
    throttle(3, 20);
    throttle(4, 20);
    delay(5000);
    disarm();
    throttle(1, 20);
    throttle(2, 20);
    throttle(3, 20);
    throttle(4, 20);
    delay(1500);

    DBG("[TCS] Testing complete\n");
}

float TCS::getThrottle(int motor) {
    MCU engine = intToEngine(motor);
    return engine.currentThrottle;
}

bool TCS::isArmed(int motor) {
    MCU engine = intToEngine(motor);
    return engine.armed;
}

MCU TCS::intToEngine(int motor) {
    switch (motor) {
    case 1:
        return engineA;
    case 2:
        return engineB;
    case 3:
        return engineC;
    case 4:
        return engineD;
    default:
        return engineA;
    }
}
