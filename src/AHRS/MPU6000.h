#include <stdint.h>

class MPU6000 {
  private:
    float accelXOffset = 0, accelYOffset = 0, accelZOffset = 0, gyroXOffset = 0,
          gyroYOffset = 0, gyroZOffset = 0; //< Variables for offsets

    static void writeRegister(uint8_t reg, uint8_t value);
    static uint8_t readRegister(uint8_t reg);
    static int16_t read16(uint8_t highReg, uint8_t lowReg);

  public:
    MPU6000();

    static void initializeMPU();

    static void getRawData(int16_t &rawAccelX, int16_t &rawAccelY,
                           int16_t &rawAccelZ, int16_t &rawGyroX,
                           int16_t &rawGyroY, int16_t &rawGyroZ);

    void getData(float &ax, float &ay, float &az, float &gx, float &gy,
                 float &gz) const;

    void calibrateAccelerometer();
    void calibrateGyroscope();
};
