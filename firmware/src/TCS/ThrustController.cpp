#include "Arduino.h"
#include "ThrustController.h"

TCS::TCS(int pinA, int pinB, int pinC, int pinD, int minPulsewidth,
         int maxPulsewidth)
    : engineA(pinA, 1, minPulsewidth, maxPulsewidth),
      engineB(pinB, 2, minPulsewidth, maxPulsewidth),
      engineC(pinC, 3, minPulsewidth, maxPulsewidth),
      engineD(pinD, 4, minPulsewidth, maxPulsewidth) {}

void TCS::arm() {
    Serial.println("Arming all engines");
    engineA.arm();
    engineB.arm();
    engineC.arm();
    engineD.arm();
}

void TCS::armMotor(int motor) {
    Serial.println("TCS arming engine " + String(motor));
    MCU engine = intToEngine(motor);
    engine.arm();
}

void TCS::disarm() {
    Serial.println("Disarming all engines");
    engineA.disarm();
    engineB.disarm();
    engineC.disarm();
    engineD.disarm();
}

void TCS::disarmMotor(int motor) {
    Serial.println("TCS disarming engine " + String(motor));
    MCU engine = intToEngine(motor);
    engine.disarm();
}

void TCS::throttle(int motor, float throttle) {
    Serial.println("TCS setting " + String(throttle) +
                   "% throttle for engine " + String(motor));
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
    Serial.println("Testing TCS");
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

    Serial.println("Testing for TCS finished!");
}

void TCS::testMotor(int motor) {
    MCU engine = intToEngine(motor);
    engine.test();
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
