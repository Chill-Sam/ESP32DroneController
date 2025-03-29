#include "MPU6000.h"
#include <MadgwickAHRS.h>
#include <QMC5883LCompass.h>

class AHRS {
  public:
    volatile float pitch, roll, yaw;
    void init();

  private:
    QMC5883LCompass compass;
    MPU6000 mpu;
    Madgwick filter;

    static void sensorTask(void *pvParameters);
};
