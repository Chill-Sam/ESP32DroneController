#pragma once

#include "MPU6000.h"
#include <MadgwickAHRS.h>
#include <QMC5883LCompass.h>

struct Orientation {
    float pitch = 0.0F;
    float roll = 0.0F;
    float yaw = 0.0F;
};

class AHRS {
  public:
    volatile float pitch = 0, roll = 0, yaw = 0;
    void init();

  private:
    QMC5883LCompass compass;
    MPU6000 mpu;
    Madgwick filter;

    static void sensorTask(void *pvParameters);
};
