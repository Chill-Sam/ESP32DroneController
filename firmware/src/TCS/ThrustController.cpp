#include "Arduino.h"
#include "ThrustController.h"
#include "utils/log.h"
#include <cstdint>

TCS::TCS(uint8_t pinA, uint8_t pinB, uint8_t pinC, uint8_t pinD,
         uint16_t minPulsewidth, uint16_t maxPulsewidth)
    : motors{MCU(pinA, 1, minPulsewidth, maxPulsewidth),
             MCU(pinB, 2, minPulsewidth, maxPulsewidth),
             MCU(pinC, 3, minPulsewidth, maxPulsewidth),
             MCU(pinD, 4, minPulsewidth, maxPulsewidth)},
      throttleState(_throttleState), armState(_armState) {}

void TCS::begin() {
    for (auto &m : motors) {
        m.begin();
    }
    updateState();
}

void TCS::arm(int8_t motor) {
    if (motor < 0) {
        DBG("[TCS] Arming all engines\n");
        for (auto &m : motors) {
            m.arm();
        }
    } else {
        DBG_FMT("[TCS] Arming engine %d\n", motor);
        motors[constrain(motor, 0, 3)].arm();
    }
    updateState();
}

void TCS::disarm(int8_t motor) {
    if (motor < 0) {
        DBG("[TCS] Disarming all engines\n");
        for (auto &m : motors) {
            m.disarm();
        }
    } else {
        DBG_FMT("[TCS] Disarming engine %d\n", motor);
        motors[constrain(motor, 0, 3)].arm();
    }
    updateState();
}

void TCS::throttle(uint8_t motor, float throttle) {
    DBG_FMT("[TCS] Setting %f% throttle for engine %u", throttle, motor);
    motors[constrain(motor, 0, 3)].setThrottle(throttle);
    updateState();
}

void TCS::stop() {
    DBG("[TCS] Stopping all motors");
    for (auto &m : motors) {
        m.stop();
    }
    updateState();
}

void TCS::updateState() {
    _throttleState.ThrottleA = motors[0].currentThrottle;
    _throttleState.ThrottleB = motors[1].currentThrottle;
    _throttleState.ThrottleC = motors[2].currentThrottle;
    _throttleState.ThrottleD = motors[3].currentThrottle;

    _armState.ArmedA = motors[0].armed;
    _armState.ArmedB = motors[1].armed;
    _armState.ArmedC = motors[2].armed;
    _armState.ArmedD = motors[3].armed;
}

void TCS::test() {
    DBG("[TCS] Testing\n");
    arm();
    delay(5000);
    throttle(0, 20);
    delay(1000);
    throttle(1, 20);
    delay(1000);
    throttle(2, 20);
    delay(1000);
    throttle(3, 20);
    delay(1000);
    stop();
    delay(2500);
    throttle(0, 20);
    throttle(1, 20);
    throttle(2, 20);
    throttle(3, 20);
    delay(5000);
    disarm();
    throttle(0, 20);
    throttle(1, 20);
    throttle(2, 20);
    throttle(3, 20);
    delay(1500);

    DBG("[TCS] Testing complete\n");
}
