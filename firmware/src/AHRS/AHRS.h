#pragma once

#include "MPU6000.h"
#include "types/FlightData.h"
#include <MadgwickAHRS.h>
#include <QMC5883LCompass.h>

class AHRS {
  public:
    AHRS();

    void begin();

    const Orientation &orientation;
    const SpeedData &speedData;

  private:
    QMC5883LCompass compass;
    MPU6000 mpu;
    Madgwick filter;

    Orientation _orientation;
    SpeedData _speedData;

    static void sensorTask(void *pvParameters);
};
