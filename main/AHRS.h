#include "MPUHandler.h"
#include <QMC5883LCompass.h>
#include <MadgwickAHRS.h>

class AHRS {
public:
  volatile float pitch, roll, yaw;

  AHRS();

  void init();

private:
  QMC5883LCompass compass;
  MPUHandler mpu;
  Madgwick filter;

  void sensorTask(void* pvParameters);
}