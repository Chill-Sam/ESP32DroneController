#include "Arduino.h"
#include "ESCDriver.h"

ESCDriver::ESCDriver(int pwmPin, int channel, int minPulsewidth)
    : pwmPin(pwmPin), channel(channel), minPulsewidth(minPulsewidth) {
    ledcSetup(channel, frequency, resolution);
    ledcAttachPin(pwmPin, channel);
};

void ESCDriver::write(float pulsewidth) {
    Serial.println("Writing pulsewidth " + String(pulsewidth) + " to ESC " +
                   String(channel));
    ledcWrite(channel, pulseWidthToDutyCycle(pulsewidth));
}

void ESCDriver::stop() {
    Serial.println("Stopping ESC " + String(channel));
    ledcWrite(channel, pulseWidthToDutyCycle(minPulsewidth));
}

float ESCDriver::pulseWidthToDutyCycle(float pulsewidth) const {
    float maxDutyCycle = pow(2, resolution) - 1;
    return map(pulsewidth, 0, 20000, 0, maxDutyCycle);
}
