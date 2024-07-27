/**
 * @file MPUHandler.h
 *
 * @brief Interfaces with the MPU6000 sensor to read accelerometer and gyroscope data.
 *
 * This class encapsulates methods to initialize the MPU6000 sensor, read its accelerometer and gyroscope
 * data, and handle SPI communication with the sensor.
 *
 * Dependencies:
 * - Wire.h: Manages I2C communication.
 * - SPI.h: Handles SPI communication.
 * - math.h: Provides mathematical constants and functions.
 *
 * Example Usage:
 * ```cpp
 * MPUHandler mpu;
 * mpu.initializeMPU();
 * std::array<float, 6> sensorData = mpu.getData();
 * Serial.println("Accel X: " + String(sensorData[0]));
 * Serial.println("Accel Y: " + String(sensorData[1]));
 * Serial.println("Accel Z: " + String(sensorData[2]));
 * Serial.println("Gyro X: " + String(sensorData[3]));
 * Serial.println("Gyro Y: " + String(sensorData[4]));
 * Serial.println("Gyro Z: " + String(sensorData[5]));
 * ```
 */

#include <Wire.h>
#include <SPI.h>
#include <array>
#include <cstdint>
#include <math.h>

// Define the pin numbers for the MPU6000 connection
#define VSPI_CS_PIN 5    ///< Chip Select pin
#define VSPI_SCK_PIN 18  ///< Serial Clock pin
#define VSPI_SDI_PIN 19  ///< Serial Data In pin
#define VSPI_SDO_PIN 23  ///< Serial Data Out pin

#define MPU6000_ADDRESS 0x68   ///< MPU6000 I2C address

// Define MPU6000 register addresses
#define ACCEL_XOUT_H 0x3B      ///< Accelerometer X-axis high byte
#define ACCEL_XOUT_L 0x3C      ///< Accelerometer X-axis low byte
#define ACCEL_YOUT_H 0x3D      ///< Accelerometer Y-axis high byte
#define ACCEL_YOUT_L 0x3E      ///< Accelerometer Y-axis low byte
#define ACCEL_ZOUT_H 0x3F      ///< Accelerometer Z-axis high byte
#define ACCEL_ZOUT_L 0x40      ///< Accelerometer Z-axis low byte
#define GYRO_XOUT_H 0x43       ///< Gyroscope X-axis high byte
#define GYRO_XOUT_L 0x44       ///< Gyroscope X-axis low byte
#define GYRO_YOUT_H 0x45       ///< Gyroscope Y-axis high byte
#define GYRO_YOUT_L 0x46       ///< Gyroscope Y-axis low byte
#define GYRO_ZOUT_H 0x47       ///< Gyroscope Z-axis high byte
#define GYRO_ZOUT_L 0x48       ///< Gyroscope Z-axis low byte
#define PWR_MGMT_1 0x6B        ///< Power management register
#define USER_CTRL 0x6A         ///< User control register

// Define constants for calculations
#define GRAVITY 9.80665              ///< Gravity constant in m/s^2
#define DEG_TO_RAD (M_PI / 180)      ///< Conversion factor from degrees to radians
#define ACCEL_SENSITIVITY 16384.0    ///< Accelerometer sensitivity (LSB/g)
#define GYRO_SENSITIVITY 131.0       ///< Gyroscope sensitivity (LSB/(deg/s))

SPIClass vspi(VSPI);  ///< Create an SPIClass instance for VSPI

/**
 * @class MPUHandler
 *
 * @brief Handles communication with the MPU6000 sensor.
 *
 * This class provides comprehensive management of the MPU6000 sensor, including initialization
 * and reading of accelerometer and gyroscope data. It uses the Wire and SPI libraries to communicate
 * with the sensor over I2C and SPI, respectively.
 *
 * Features:
 * - Initialize the MPU6000 sensor.
 * - Read accelerometer and gyroscope data.
 *
 * @note The device must support I2C and SPI communication.
 */
class MPUHandler {
public:
  /**
   * @brief Constructor for the MPUHandler class.
   */
  MPUHandler() {}

  /**
   * @brief Initializes the MPU6000 sensor.
   *
   * Sets up SPI communication and configures the sensor.
   */
  void initializeMPU() {
    // Start VSPI communication
    vspi.begin(VSPI_SCK_PIN, VSPI_SDI_PIN, VSPI_SDO_PIN, VSPI_CS_PIN);

    pinMode(VSPI_CS_PIN, OUTPUT); // Set Chip Select pin to output
    digitalWrite(VSPI_CS_PIN, HIGH); // Deselect VSPI

    delay(100); // Wait for hardware initialization

    writeRegister(PWR_MGMT_1, 0x00); // Sets register 0x6B to empty, clearing sleep bit causing chip to wake up.

    delay(100); // Wait for MPU6000 to wake up

    // Set I2C_IF_DIS bit to 1 to disable I2C communication and force chip to use SPI
    uint8_t userCtrl = readRegister(USER_CTRL);
    userCtrl |= (1 << 4); // Sets bit 4 to 1 (I2C_IF_DIS)
    writeRegister(USER_CTRL, userCtrl);

    // Confirm register data
    userCtrl = readRegister(USER_CTRL);
    if (userCtrl == 0x10) {
      Serial.println("USER_CTRL Register properly set, I2C_IF_DIS bit is 1.");
    } else {
      Serial.print("Error setting USER_CTRL Register: ");
      Serial.print(userCtrl, HEX);
      while (1);
    }
  }

  /**
   * @brief Reads accelerometer and gyroscope data from the MPU6000 sensor.
   *
   * @return An array containing accelerometer and gyroscope data: [accelX_mps2, accelY_mps2, accelZ_mps2, gyroX_rads, gyroY_rads, gyroZ_rads].
   */
  std::array<float, 6> getData() {
    // Get accelerometer data
    // Get AccelX Data
    uint8_t accelX_H = readRegister(ACCEL_XOUT_H); // Get high bits
    uint8_t accelX_L = readRegister(ACCEL_XOUT_L); // Get low bits
    int16_t accelX = (accelX_H << 8) | accelX_L; // Combine
    float accelX_mps2 = (accelX / ACCEL_SENSITIVITY) * GRAVITY; // Convert to m/s^2
    
    // Get AccelY Data
    uint8_t accelY_H = readRegister(ACCEL_YOUT_H); // Get high bits
    uint8_t accelY_L = readRegister(ACCEL_YOUT_L); // Get low bits
    int16_t accelY = (accelY_H << 8) | accelY_L; // Combine
    float accelY_mps2 = (accelY / ACCEL_SENSITIVITY) * GRAVITY; // Convert to m/s^2
    
    // Get AccelZ Data
    uint8_t accelZ_H = readRegister(ACCEL_ZOUT_H); // Get high bits
    uint8_t accelZ_L = readRegister(ACCEL_ZOUT_L); // Get low bits
    int16_t accelZ = (accelZ_H << 8) | accelZ_L; // Combine
    float accelZ_mps2 = (accelZ / ACCEL_SENSITIVITY) * GRAVITY; // Convert to m/s^2

    // Get gyroscope data
    // Get GyroX Data
    uint8_t gyroX_H = readRegister(GYRO_XOUT_H); // Get high bits
    uint8_t gyroX_L = readRegister(GYRO_XOUT_L); // Get low bits
    int16_t gyroX = (gyroX_H << 8) | gyroX_L; // Combine
    float gyroX_rads = (gyroX / GYRO_SENSITIVITY) * DEG_TO_RAD; // Convert to rad/s

    // Get GyroY Data
    uint8_t gyroY_H = readRegister(GYRO_YOUT_H); // Get high bits
    uint8_t gyroY_L = readRegister(GYRO_YOUT_L); // Get low bits
    int16_t gyroY = (gyroY_H << 8) | gyroY_L; // Combine
    float gyroY_rads = (gyroY / GYRO_SENSITIVITY) * DEG_TO_RAD; // Convert to rad/s
    
    // Get GyroZ Data
    uint8_t gyroZ_H = readRegister(GYRO_ZOUT_H); // Get high bits
    uint8_t gyroZ_L = readRegister(GYRO_ZOUT_L); // Get low bits
    int16_t gyroZ = (gyroZ_H << 8) | gyroZ_L; // Combine
    float gyroZ_rads = (gyroZ / GYRO_SENSITIVITY) * DEG_TO_RAD; // Convert to rad/s

    return std::array<float, 6> {accelX_mps2, accelY_mps2, accelZ_mps2, gyroX_rads, gyroY_rads, gyroZ_rads};
  }

private:
  /**
   * @brief Writes a byte of data to a specific register of the MPU6000.
   *
   * @param reg The register address to write to.
   * @param value The data byte to write to the register.
   */
  void writeRegister(uint8_t reg, uint8_t value) {
    vspi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));  ///< Adjust SPI settings if needed
    digitalWrite(VSPI_CS_PIN, LOW);  ///< Select the slave device
    vspi.transfer(reg);  ///< Register address
    vspi.transfer(value);  ///< Register value
    digitalWrite(VSPI_CS_PIN, HIGH);  ///< Deselect the slave device
    vspi.endTransaction();
  }

  /**
   * @brief Reads a byte of data from a specific register of the MPU6000.
   *
   * @param reg The register address to read from.
   * @return The byte value read from the register.
   */
  uint8_t readRegister(uint8_t reg) {
    uint8_t value;
    vspi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));  ///< Adjust SPI settings if needed
    digitalWrite(VSPI_CS_PIN, LOW);  ///< Select the slave device
    vspi.transfer(reg | 0x80);  ///< Register address with read flag
    value = vspi.transfer(0x00);  ///< Dummy write to read the register value
    digitalWrite(VSPI_CS_PIN, HIGH);  ///< Deselect the slave device
    vspi.endTransaction();
    return value;
  }
};