#pragma once

#include "MPU6000.h"
#include <MadgwickAHRS.h>
#include <QMC5883LCompass.h>

class AHRS {
  public:
    void begin();

    volatile float pitch = 0, roll = 0, yaw = 0;
    volatile float gx = 0, gy = 0, gz = 0;

  private:
    QMC5883LCompass compass;
    MPU6000 mpu;
    Madgwick filter;

    static void sensorTask(void *pvParameters);
};
