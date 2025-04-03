#include "PID.h"
#include <Arduino.h>

PID::PID(float p_, float i_, float d_) : p(p_), i(i_), d(d_) {};

void PID::begin() {
    xTaskCreatePinnedToCore(pidTask,   // task function
                            "pidTask", // name
                            4096,      // stack size
                            this,      // parameters
                            1,         // priority
                            nullptr,   // task handle
                            1          // core (1 = avoid Wi-Fi core)
    );
}

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
    auto dt = static_cast<float>((now - lastMicros)) / 1e6F;
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

void PID::pidTask(void *pvParameters) {
    PID *self = static_cast<PID *>(pvParameters); // cast back to instance

    const TickType_t xFrequency = pdMS_TO_TICKS(2);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        self->output = self->calc(self->setpoint, self->input);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
