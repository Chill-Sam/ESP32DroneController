#include "AHRS.h"

void AHRS::init() {
    mpu.initializeMPU();
    mpu.calibrateAccelerometer();
    delay(1000);
    mpu.calibrateGyroscope();
    delay(1000);

    compass.init();
    compass.setMode(0x01, 0x0C, 0x00, 0x00);
    compass.setCalibrationOffsets(417.00, 499.00, 308.00);
    compass.setCalibrationScales(1.05, 0.98, 0.98);
    compass.setMagneticDeclination(6, 59);

    filter.begin(500.0f);

    xTaskCreatePinnedToCore(sensorTask,     // task function
                            "SensorFusion", // name
                            4096,           // stack size
                            this,           // parameters
                            1,              // priority
                            NULL,           // task handle
                            1               // core (1 = avoid Wi-Fi core)
    );
}

void AHRS::sensorTask(void *pvParameters) {
    AHRS *self = static_cast<AHRS *>(pvParameters); // cast back to instance
                                                    //
    const TickType_t xFrequency = pdMS_TO_TICKS(2);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        float ax, ay, az, gx, gy, gz, mx, my, mz;
        float ax_ned, ay_ned, az_ned, gx_ned, gy_ned, gz_ned, mx_ned, my_ned,
            mz_ned;

        self->mpu.getData(ax, ay, az, gx, gy, gz);

        self->compass.read();
        mx = self->compass.getX();
        my = self->compass.getY();
        mz = self->compass.getZ();

        // Convert sensor output to NED (North-East-Down)
        ax_ned = -ax;
        ay_ned = ay;
        az_ned = az;

        gx_ned = gx;
        gy_ned = -gy;
        gz_ned = -gz;

        mx_ned = my;
        my_ned = -mx;
        mz_ned = -mz;

        self->filter.update(gx_ned, gy_ned, gz_ned, ax_ned, ay_ned, az_ned,
                            mx_ned, my_ned, mz_ned);

        // === GET ORIENTATION ===
        self->roll = self->filter.getRoll();
        self->pitch = self->filter.getPitch();
        self->yaw = self->filter.getYaw();
    }
}
