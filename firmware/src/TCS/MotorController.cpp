#include "Arduino.h"
#include "ESCDriver.h"
#include "MotorController.h"

MCU::MCU(int pin, int channel, int minPulsewidth, int maxPulsewidth)
    : esc(pin, channel, minPulsewidth), channel(channel),
      minPulsewidth(minPulsewidth), maxPulsewidth(maxPulsewidth) {}

void MCU::arm() {
    Serial.println("Arming engine " + String(channel));
    esc.stop();
    armed = true;
}

void MCU::disarm() {
    Serial.println("Disarming engine " + String(channel));
    esc.stop();
    armed = false;
}

void MCU::setThrottle(float throttle) {
    if (!armed) {
        Serial.println("Engine " + String(channel) + " is disarmed!");
        return;
    }

    Serial.println("Setting engine " + String(channel) + " to " +
                   String(throttle) + "% throttle");
    float pulsewidth = map(throttle, 0, 100, minPulsewidth, maxPulsewidth);
    esc.write(pulsewidth);
}

void MCU::stop() {
    Serial.println("Stopping engine " + String(channel));
    esc.stop();
}

void MCU::test() {
    Serial.println("Testing engine " + String(channel));
    arm();
    delay(5000);
    setThrottle(20);
    delay(5000);
    stop();
    delay(5000);
    setThrottle(20);
    delay(5000);
    disarm();
    setThrottle(20);
    delay(1500);
    Serial.println("Testing for engine " + String(channel) + " finished!");
}
