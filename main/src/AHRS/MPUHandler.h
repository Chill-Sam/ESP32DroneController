#include <cstdint>
#include <array>

class MPUHandler {
private:
  float accelXOffset, accelYOffset, accelZOffset, gyroXOffset, gyroYOffset, gyroZOffset = 0; //< Variables for offsets
  
  void writeRegister(uint8_t reg, uint8_t value); 
  uint8_t readRegister(uint8_t reg);

public:
  MPUHandler();

  void initializeMPU();
  
  void getRawData(int16_t &rawAccelX, int16_t &rawAccelY, int16_t &rawAccelZ, int16_t &rawGyroX, int16_t &rawGyroY, int16_t &rawGyroZ);
  void getData(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);

  void calibrateAccelerometer();
  void calibrateGyroscope();
};
