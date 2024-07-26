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

// Define the pin numbers for the MPU6000 connection
#define VSPI_CS_PIN 5    ///< Chip Select pin
#define VSPI_SCK_PIN 18  ///< Serial Clock pin
#define VSPI_SDI_PIN 19  ///< Serial Data In pin
#define VSPI_SDO_PIN 23  ///< Serial Data Out pin

// Define MPU6000 I2C address and register addresses
#define MPU6000_ADDRESS 0x68   ///< MPU6000 I2C address
#define ACCEL_XOUT_H 0x3B      ///< Accelerometer X-axis high byte
#define ACCEL_YOUT_H 0x3D      ///< Accelerometer Y-axis high byte
#define ACCEL_ZOUT_H 0x3F      ///< Accelerometer Z-axis high byte
#define GYRO_XOUT_H 0x43       ///< Gyroscope X-axis high byte
#define GYRO_YOUT_H 0x45       ///< Gyroscope Y-axis high byte
#define GYRO_ZOUT_H 0x47       ///< Gyroscope Z-axis high byte
#define PWR_MGMT_1 0x6B        ///< Power management register

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
     * Sets up I2C and SPI communication and configures the sensor.
     */
    void initializeMPU() {
        Wire.begin();  ///< Initialize SPI communication
        SPI.begin(VSPI_SCK_PIN, VSPI_SDI_PIN, VSPI_SDO_PIN, VSPI_CS_PIN);  ///< Initialize SPI communication
        pinMode(VSPI_CS_PIN, OUTPUT);  ///< Set Chip Select pin as output
        digitalWrite(VSPI_CS_PIN, HIGH);  ///< Set Chip Select pin high (inactive)
        writeRegister(MPU6000_ADDRESS, PWR_MGMT_1, 0x00);  ///< Wake up the MPU6000 by writing 0 to the power management register
    }

    /**
     * @brief Reads accelerometer and gyroscope data from the MPU6000 sensor.
     *
     * @return An array containing accelerometer and gyroscope data: [accelX, accelY, accelZ, gyroX, gyroY, gyroZ].
     */
    std::array<float, 6> getData() {
        int16_t ax, ay, az, gx, gy, gz;
        ax = readRegister16(MPU6000_ADDRESS, ACCEL_XOUT_H);  ///< Read X-axis accelerometer data
        ay = readRegister16(MPU6000_ADDRESS, ACCEL_YOUT_H);  ///< Read Y-axis accelerometer data
        az = readRegister16(MPU6000_ADDRESS, ACCEL_ZOUT_H);  ///< Read Z-axis accelerometer data
        gx = readRegister16(MPU6000_ADDRESS, GYRO_XOUT_H);   ///< Read X-axis gyroscope data
        gy = readRegister16(MPU6000_ADDRESS, GYRO_YOUT_H);   ///< Read Y-axis gyroscope data
        gz = readRegister16(MPU6000_ADDRESS, GYRO_ZOUT_H);   ///< Read Z-axis gyroscope data

        // Convert raw data to physical values
        float accelX = ax / 16384.0;
        float accelY = ay / 16384.0;
        float accelZ = az / 16384.0;
        float gyroX = gx / 131.0;
        float gyroY = gy / 131.0;
        float gyroZ = gz / 131.0;

        std::array<float, 6> data{accelX, accelY, accelZ, gyroX, gyroY, gyroZ};  ///< Store data in an array
        return data;
    }

private:
    /**
     * @brief Writes a byte of data to a specific register of the MPU6000.
     *
     * @param address The I2C address of the MPU6000.
     * @param reg The register address to write to.
     * @param data The data byte to write to the register.
     */
    void writeRegister(uint8_t address, uint8_t reg, uint8_t data) {
        digitalWrite(VSPI_CS_PIN, LOW);  ///< Activate the MPU6000
        SPI.transfer(reg);  ///< Send register address
        SPI.transfer(data);  ///< Send data
        digitalWrite(VSPI_CS_PIN, HIGH);  ///< Deactivate the MPU6000
    }

    /**
     * @brief Reads a 16-bit value from a specific register of the MPU6000.
     *
     * @param address The SPI address of the MPU6000.
     * @param reg The register address to read from.
     * @return The 16-bit value read from the register.
     */
    int16_t readRegister16(uint8_t address, uint8_t reg) {
        digitalWrite(VSPI_CS_PIN, LOW);  ///< Activate the MPU6000
        SPI.transfer(reg | 0x80);  ///< Send register address with read flag
        int16_t value = SPI.transfer(0x00) << 8 | SPI.transfer(0x00);  ///< Read high and low bytes
        digitalWrite(VSPI_CS_PIN, HIGH);  ///< Deactivate the MPU6000
        return value;  ///< Return the 16-bit value
    }
};
