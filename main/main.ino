#include "MPUHandler.h"
#include <QMC5883LCompass.h>
#include <MadgwickAHRS.h>
QMC5883LCompass compass;
Madgwick filter;


MPUHandler MPU;

volatile float roll, pitch, yaw = 0.0f;

void sensorTask(void* pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(2);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true) {
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    float ax_ned, ay_ned, az_ned, gx_ned, gy_ned, gz_ned, mx_ned, my_ned, mz_ned;

    MPU.getData(ax, ay, az, gx, gy, gz);

    compass.read();

    mx = compass.getX();
    my = compass.getY();
    mz = compass.getZ();

    ax_ned = -ax;
    ay_ned = ay;
    az_ned = az;

    gx_ned = gx;
    gy_ned = -gy;
    gz_ned = -gz;

    mx_ned = my;
    my_ned = -mx;
    mz_ned = -mz;

    filter.update(gx_ned, gy_ned, gz_ned, ax_ned, ay_ned, az_ned, mx_ned, my_ned, mz_ned);

    // === GET ORIENTATION ===
    roll  = filter.getRoll();
    pitch = filter.getPitch();
    yaw   = filter.getYaw();

    // === DEBUG OUTPUT ===
    Serial.print("Roll: "); Serial.print(roll, 2);
    Serial.print("  Pitch: "); Serial.print(pitch, 2);
    Serial.print("  Yaw: "); Serial.println(yaw, 2);
  }
}

void setup() {
  Serial.begin(115200);
  MPU.initializeMPU();
  MPU.calibrateAccelerometer();
  delay(2500);
  MPU.calibrateGyroscope();
  delay(2500);

  compass.init();
  compass.setMode(0x01, 0x0C, 0x00, 0x00);
  compass.setCalibrationOffsets(417.00, 499.00, 308.00);
  compass.setCalibrationScales(1.05, 0.98, 0.98);
  compass.setMagneticDeclination(6, 59);

  filter.begin(500.0f);

  xTaskCreatePinnedToCore(
    sensorTask,          // task function
    "SensorFusion",      // name
    4096,                // stack size
    NULL,                // parameters
    1,                   // priority
    NULL,                // task handle
    1                    // core (1 = avoid Wi-Fi core)
  );
}

void loop() {
    /*MPU.getData(ax, ay, az, gx, gy, gz);

    compass.read();

    mx = compass.getX();
    my = compass.getY();
    mz = compass.getZ();

    ax_ned = -ax;
    ay_ned = ay;
    az_ned = az;

    gx_ned = gx;
    gy_ned = -gy;
    gz_ned = -gz;

    mx_ned = my;
    my_ned = -mx;
    mz_ned = -mz;

    filter.update(gx_ned, gy_ned, gz_ned, ax_ned, ay_ned, az_ned, mx_ned, my_ned, mz_ned);

    // === GET ORIENTATION ===
    float roll  = filter.getRoll();
    float pitch = filter.getPitch();
    float yaw   = filter.getYaw();

    // === DEBUG OUTPUT ===
    Serial.print("Roll: "); Serial.print(roll, 2);
    Serial.print("  Pitch: "); Serial.print(pitch, 2);
    Serial.print("  Yaw: "); Serial.println(yaw, 2);

  //Serial.println("ax:" + String(ax_ned) + "," + "ay:" + String(ay_ned) + "," + "az:" + String(az_ned) + "," + "gx:" + String(gx_ned) + "," + "gy:" + String(gy_ned) + "," + "gz:" + String(gz_ned));
  //Serial.println("mx:" + String(mx) + "," + "my:" + String(my) + "," + "mz:" + String(mz));
  //Serial.println(String(mx) + " " + String(my) + " " + String(mz));*/
}