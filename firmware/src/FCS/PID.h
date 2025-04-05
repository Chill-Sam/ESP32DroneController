// TODO: Add PID class
#pragma once

class PID {
  public:
    volatile float output = 0;
    float input = 0;
    float setpoint = 0;

    PID(float p_, float i_, float d_);

    float calc(float setpoint, float input);

    void begin();
    void tune(float p_, float i_, float d_);
    void limit(float min_, float max_);
    void reset();

  private:
    static void pidTask(void *pvParameters);
    float p, i, d;

    float integral = 0.0F;
    float previousError = 0.0F;

    float min = -20.0F;
    float max = 20.0F;

    unsigned long lastMicros = 0;
};
