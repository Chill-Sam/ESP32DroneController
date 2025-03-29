#include "MPU6000.h"
#include <Arduino.h>
#include <SPI.h> // SPI communication library

// Define the pin numbers for the MPU6000 connection
#define MPU_CS_PIN 5 ///< Chip Select pin
#define SCK_PIN 18   ///< Serial Clock pin
#define SDI_PIN 19   ///< Serial Data In pin

#define SDO_PIN 23 ///< Serial Data Out pin

// Define MPU6000 register addresses
#define ACCEL_XOUT_H 0x3B ///< Accelerometer X-axis high byte
#define ACCEL_XOUT_L 0x3C ///< Accelerometer X-axis low byte
#define ACCEL_YOUT_H 0x3D ///< Accelerometer Y-axis high byte
#define ACCEL_YOUT_L 0x3E ///< Accelerometer Y-axis low byte
#define ACCEL_ZOUT_H 0x3F ///< Accelerometer Z-axis high byte
#define ACCEL_ZOUT_L 0x40 ///< Accelerometer Z-axis low byte

#define GYRO_XOUT_H 0x43 ///< Gyroscope X-axis high byte
#define GYRO_XOUT_L 0x44 ///< Gyroscope X-axis low byte
#define GYRO_YOUT_H 0x45 ///< Gyroscope Y-axis high byte
#define GYRO_YOUT_L 0x46 ///< Gyroscope Y-axis low byte
#define GYRO_ZOUT_H 0x47 ///< Gyroscope Z-axis high byte
#define GYRO_ZOUT_L 0x48 ///< Gyroscope Z-axis low byte

#define PWR_MGMT_1 0x6B   ///< Power management register
#define USER_CTRL 0x6A    ///< User control register
#define ACCEL_CONFIG 0x1C ///< Accelerometer configuration
#define GYRO_CONFIG 0x1B  ///< Gyrometer configuration

// Define constants for calculations
// #define GRAVITY 9.80665              ///< Gravity constant in m/s^2
#define GRAVITY 1 ///< Measure in g

#define ACCEL_SENSITIVITY 8192.0 ///< Accelerometer sensitivity (LSB/g)
#define GYRO_SENSITIVITY 65.5    ///< Gyroscope sensitivity (LSB/(deg/s))

MPU6000::MPU6000() {}

void MPU6000::initializeMPU() {
    SPI.begin(SCK_PIN, SDI_PIN, SDO_PIN, MPU_CS_PIN);

    pinMode(MPU_CS_PIN, OUTPUT);
    digitalWrite(MPU_CS_PIN, HIGH);

    delay(100);

    writeRegister(PWR_MGMT_1, 0x00); // Sets register 0x6B to empty, clearing
                                     // sleep bit causing chip to wake up.

    delay(100);

    // Set AFS_SEL to 1 for +- 4g range
    uint8_t accelConfig = readRegister(ACCEL_CONFIG);
    accelConfig |= (1 << 3);
    writeRegister(ACCEL_CONFIG, accelConfig);

    // Set FS_SEL for +- 500 deg/s range
    uint8_t gyroConfig = readRegister(GYRO_CONFIG);
    gyroConfig |= (1 << 3);
    writeRegister(GYRO_CONFIG, gyroConfig);

    // Set I2C_IF_DIS bit to 1 to disable I2C communication and force chip to
    // use SPI
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
        while (1)
            ;
    }
}

void MPU6000::getData(float &ax, float &ay, float &az, float &gx, float &gy,
                      float &gz) {
    // Get raw values first
    int16_t rax, ray, raz, rgx, rgy, rgz;
    getRawData(rax, ray, raz, rgx, rgy, rgz);
    ax = rax / ACCEL_SENSITIVITY * GRAVITY - accelXOffset;
    ay = ray / ACCEL_SENSITIVITY * GRAVITY - accelYOffset;
    az = raz / ACCEL_SENSITIVITY * GRAVITY - accelZOffset;

    gx = rgx / GYRO_SENSITIVITY - gyroXOffset;
    gy = rgy / GYRO_SENSITIVITY - gyroYOffset;
    gz = rgz / GYRO_SENSITIVITY - gyroZOffset;
}

void MPU6000::calibrateAccelerometer() {
    Serial.println("Calibrating Accelerometer...");
    float ax, ay, az;
    int numSamples = 500;

    double sumXOffset, sumYOffset, sumZOffset = 0;

    for (int i = 0; i < numSamples; i++) {
        int16_t rax, ray, raz, rgx, rgy, rgz;
        getRawData(rax, ray, raz, rgx, rgy, rgz);
        ax = rax / ACCEL_SENSITIVITY * GRAVITY;
        ay = ray / ACCEL_SENSITIVITY * GRAVITY;
        az = raz / ACCEL_SENSITIVITY * GRAVITY;

        sumXOffset += ax;
        sumYOffset += ay;
        sumZOffset += az;

        Serial.println(
            "ax: " + String(ax) + " | sum: " + String(sumXOffset) +
            " | ay: " + String(ay) + " | sum: " + String(sumYOffset) +
            " | az: " + String(az) + " | sum: " + String(sumZOffset));
        delay(10);
    }

    sumXOffset /= numSamples;
    sumYOffset /= numSamples;
    sumZOffset /= numSamples;

    // Adjust for gravity
    sumZOffset -= GRAVITY;

    accelXOffset = sumXOffset;
    accelYOffset = sumYOffset;
    accelZOffset = sumZOffset;

    Serial.println("X: " + String(accelXOffset) + " | Y: " +
                   String(accelYOffset) + " | Z: " + String(accelZOffset));
    Serial.println("Accelerometer Calibration Complete.");
}

void MPU6000::calibrateGyroscope() {
    Serial.println("Calibrating gyroscope...");
    float gx, gy, gz;
    int numSamples = 500;

    double sumXOffset, sumYOffset, sumZOffset = 0;

    for (int i = 0; i < numSamples; i++) {
        int16_t rax, ray, raz, rgx, rgy, rgz;
        getRawData(rax, ray, raz, rgx, rgy, rgz);
        gx = rgx / GYRO_SENSITIVITY;
        gy = rgy / GYRO_SENSITIVITY;
        gz = rgz / GYRO_SENSITIVITY;

        sumXOffset += gx;
        sumYOffset += gy;
        sumZOffset += gz;

        Serial.println(
            "gx: " + String(gx) + " | sum: " + String(sumXOffset) +
            " | gy: " + String(gy) + " | sum: " + String(sumYOffset) +
            " | gz: " + String(gz) + " | sum: " + String(sumZOffset));
        delay(10);
    }

    sumXOffset /= numSamples;
    sumYOffset /= numSamples;
    sumZOffset /= numSamples;

    gyroXOffset = sumXOffset;
    gyroYOffset = sumYOffset;
    gyroZOffset = sumZOffset;
    Serial.println("X: " + String(gyroXOffset) + " | Y: " +
                   String(gyroYOffset) + " | Z: " + String(gyroZOffset));
    Serial.println("Gyroscope Calibration Complete.");
}

void MPU6000::writeRegister(uint8_t reg, uint8_t value) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
    digitalWrite(MPU_CS_PIN, LOW);
    SPI.transfer(reg);
    SPI.transfer(value);
    digitalWrite(MPU_CS_PIN, HIGH);
    SPI.endTransaction();
}

uint8_t MPU6000::readRegister(uint8_t reg) {
    uint8_t value;
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
    digitalWrite(MPU_CS_PIN, LOW);
    SPI.transfer(reg | 0x80);

    value = SPI.transfer(0x00);
    digitalWrite(MPU_CS_PIN, HIGH);
    SPI.endTransaction();
    return value;
}

void MPU6000::getRawData(int16_t &rawAccelX, int16_t &rawAccelY,
                         int16_t &rawAccelZ, int16_t &rawGyroX,
                         int16_t &rawGyroY, int16_t &rawGyroZ) {
    rawAccelX = (readRegister(ACCEL_XOUT_H) << 8) | readRegister(ACCEL_XOUT_L);
    rawAccelY = (readRegister(ACCEL_YOUT_H) << 8) | readRegister(ACCEL_YOUT_L);
    rawAccelZ = (readRegister(ACCEL_ZOUT_H) << 8) | readRegister(ACCEL_ZOUT_L);
    rawGyroX = (readRegister(GYRO_XOUT_H) << 8) | readRegister(GYRO_XOUT_L);
    rawGyroY = (readRegister(GYRO_YOUT_H) << 8) | readRegister(GYRO_YOUT_L);
    rawGyroZ = (readRegister(GYRO_ZOUT_H) << 8) | readRegister(GYRO_ZOUT_L);
}
