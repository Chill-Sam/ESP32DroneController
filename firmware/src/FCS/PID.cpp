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

    // First‐call guard
    if (lastMicros == 0UL) {
        lastMicros = now;
        previousError = setpoint - input;
        return 0.0F;
    }

    // dt in _seconds_
    float dt = (now - lastMicros) * 1e-6F;
    lastMicros = now;

    // Basic sanity check: skip I & D if dt is bogus,
    // but still apply P so the loop doesn't go dead.
    bool valid = (dt > 0.0F && dt < 1.0F);

    float error = setpoint - input;
    float P = p * error; // proportional term

    // Integral term (only if dt sane)
    if (valid) {
        integral += error * dt;
        // anti‑windup: clamp integral within output bounds
        if (integral * i > max) {
            integral = max / i;
        } else if (integral * i < min) {
            integral = min / i;
        }
    }
    float I = i * integral;

    // Derivative term (only if dt sane)
    float D = 0.0F;
    if (valid) {
        float derivative = (error - previousError) / dt;
        D = d * derivative;
    }

    previousError = error;

    // Compute and constrain final output
    float output = P + I + D;
    output = constrain(output, min, max);
    return output;
}
// NOLINTEND(readability-identifier-naming)
