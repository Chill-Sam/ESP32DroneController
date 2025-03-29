#include "MPU6000.h"
#include <MadgwickAHRS.h>
#include <QMC5883LCompass.h>

class AHRS {
  public:
    volatile float pitch, roll, yaw;

    AHRS();

    void init();

  private:
    QMC5883LCompass compass;
    MPU6000 mpu;
    Madgwick filter;

    void sensorTask(void *pvParameters);
}
