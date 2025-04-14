#include "PID.h"
#include <Arduino.h>

PID::PID(float p_, float i_, float d_) : p(p_), i(i_), d(d_) {};

void PID::tune(float p_, float i_, float d_) {
    p = p_;
    i = i_;
    d = d_;
}

void PID::limit(float min_, float max_) {
    min = min_;
    max = max_;
}

void PID::reset() {
    integral = 0.0F;
    previousError = 0.0F;
    lastMicros = micros();
}

// NOLINTBEGIN(readability-identifier-naming)
float PID::calc(float setpoint, float input) {
    unsigned long now = micros();
    long dt = (now - lastMicros) / 1e6F;
    lastMicros = now;

    if (dt <= 0.0F || dt > 1.0F) {
        return 0.0F; // prevent crazy values
    }

    float error = setpoint - input;

    float P = p * error;

    integral += error * dt;
    float I = i * integral;

    float derivative = (error - previousError) / dt;
    float D = d * derivative;

    previousError = error;

    float output = P + I + D;

    return constrain(output, min, max);
}
// NOLINTEND(readability-identifier-naming)
