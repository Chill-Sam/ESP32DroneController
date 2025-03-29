#include "AHRS.h"

void AHRS::init() {
    MPU6000::initializeMPU();
    mpu.calibrateAccelerometer();
    delay(1000);
    mpu.calibrateGyroscope();
    delay(1000);

    compass.init();
    compass.setMode(0x01, 0x0C, 0x00, 0x00);
    compass.setCalibrationOffsets(417.00, 499.00, 308.00);
    compass.setCalibrationScales(1.05, 0.98, 0.98);
    compass.setMagneticDeclination(6, 59);

    filter.begin(500.0F);

    xTaskCreatePinnedToCore(sensorTask,     // task function
                            "SensorFusion", // name
                            4096,           // stack size
                            this,           // parameters
                            1,              // priority
                            nullptr,        // task handle
                            1               // core (1 = avoid Wi-Fi core)
    );
}

void AHRS::sensorTask(void *pvParameters) {
    AHRS *self = static_cast<AHRS *>(pvParameters); // cast back to instance
                                                    //
    const TickType_t xFrequency = pdMS_TO_TICKS(2);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0, mx = 0, my = 0,
              mz = 0;
        float axNed = 0, ayNed = 0, azNed = 0, gxNed = 0, gyNed = 0, gzNed = 0,
              mxNed = 0, myNed = 0, mzNed = 0;

        self->mpu.getData(ax, ay, az, gx, gy, gz);

        self->compass.read();
        mx = static_cast<float>(self->compass.getX());
        my = static_cast<float>(self->compass.getY());
        mz = static_cast<float>(self->compass.getZ());

        // Convert sensor output to NED (North-East-Down)
        axNed = -ax;
        ayNed = ay;
        azNed = az;

        gxNed = gx;
        gyNed = -gy;
        gzNed = -gz;

        mxNed = my;
        myNed = -mx;
        mzNed = -mz;

        self->filter.update(gxNed, gyNed, gzNed, axNed, ayNed, azNed, mxNed,
                            myNed, mzNed);

        // === GET ORIENTATION ===
        self->roll = self->filter.getRoll();
        self->pitch = self->filter.getPitch();
        self->yaw = self->filter.getYaw();
    }
}
