#pragma once

class ESCDriver {
  public:
    ESCDriver(int pwmPin, int channel, int minPulsewidth);

    void write(float pulsewidth);
    void stop();

  private:
    int pwmPin;
    int channel;

    int frequency = 50;
    int resolution = 16;

    int minPulsewidth;

    float pulseWidthToDutyCycle(float pulsewidth) const;
};
