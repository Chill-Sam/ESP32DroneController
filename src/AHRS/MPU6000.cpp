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

MPU6000::MPU6000() = default;

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
        while (true) {
        }
    }
}

void MPU6000::getData(float &ax, float &ay, float &az, float &gx, float &gy,
                      float &gz) const {
    // Get raw values first
    int16_t rax = 0, ray = 0, raz = 0, rgx = 0, rgy = 0, rgz = 0;
    getRawData(rax, ray, raz, rgx, rgy, rgz);
    ax = static_cast<float>((rax / ACCEL_SENSITIVITY * GRAVITY) - accelXOffset);
    ay = static_cast<float>((ray / ACCEL_SENSITIVITY * GRAVITY) - accelYOffset);
    az = static_cast<float>((raz / ACCEL_SENSITIVITY * GRAVITY) - accelZOffset);

    gx = static_cast<float>((rgx / GYRO_SENSITIVITY) - gyroXOffset);
    gy = static_cast<float>((rgy / GYRO_SENSITIVITY) - gyroYOffset);
    gz = static_cast<float>((rgz / GYRO_SENSITIVITY) - gyroZOffset);
}

void MPU6000::calibrateAccelerometer() {
    Serial.println("Calibrating Accelerometer...");
    int numSamples = 500;

    float sumXOffset = 0, sumYOffset = 0, sumZOffset = 0;

    for (int i = 0; i < numSamples; i++) {
        float ax = 0, ay = 0, az = 0;
        int16_t rax = 0, ray = 0, raz = 0, rgx = 0, rgy = 0, rgz = 0;
        getRawData(rax, ray, raz, rgx, rgy, rgz);
        ax = static_cast<float>(rax / ACCEL_SENSITIVITY * GRAVITY);
        ay = static_cast<float>(ray / ACCEL_SENSITIVITY * GRAVITY);
        az = static_cast<float>(raz / ACCEL_SENSITIVITY * GRAVITY);

        sumXOffset += ax;
        sumYOffset += ay;
        sumZOffset += az;

        Serial.println(
            "ax: " + String(ax) + " | sum: " + String(sumXOffset) +
            " | ay: " + String(ay) + " | sum: " + String(sumYOffset) +
            " | az: " + String(az) + " | sum: " + String(sumZOffset));
        delay(10);
    }

    sumXOffset /= static_cast<float>(numSamples);
    sumYOffset /= static_cast<float>(numSamples);
    sumZOffset /= static_cast<float>(numSamples);

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
    int numSamples = 500;

    float sumXOffset = 0, sumYOffset = 0, sumZOffset = 0;

    for (int i = 0; i < numSamples; i++) {
        float gx = 0, gy = 0, gz = 0;
        int16_t rax = 0, ray = 0, raz = 0, rgx = 0, rgy = 0, rgz = 0;
        getRawData(rax, ray, raz, rgx, rgy, rgz);
        gx = static_cast<float>(rgx / GYRO_SENSITIVITY);
        gy = static_cast<float>(rgy / GYRO_SENSITIVITY);
        gz = static_cast<float>(rgz / GYRO_SENSITIVITY);

        sumXOffset += gx;
        sumYOffset += gy;
        sumZOffset += gz;

        Serial.println(
            "gx: " + String(gx) + " | sum: " + String(sumXOffset) +
            " | gy: " + String(gy) + " | sum: " + String(sumYOffset) +
            " | gz: " + String(gz) + " | sum: " + String(sumZOffset));
        delay(10);
    }

    sumXOffset /= static_cast<float>(numSamples);
    sumYOffset /= static_cast<float>(numSamples);
    sumZOffset /= static_cast<float>(numSamples);

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
    uint8_t value = 0;
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
    rawAccelX = read16(ACCEL_XOUT_H, ACCEL_XOUT_L);
    rawAccelY = read16(ACCEL_YOUT_H, ACCEL_YOUT_L);
    rawAccelZ = read16(ACCEL_ZOUT_H, ACCEL_ZOUT_L);
    rawGyroX = read16(GYRO_XOUT_H, GYRO_XOUT_L);
    rawGyroY = read16(GYRO_YOUT_H, GYRO_YOUT_L);
    rawGyroZ = read16(GYRO_ZOUT_H, GYRO_ZOUT_L);
}

int16_t MPU6000::read16(uint8_t highReg, uint8_t lowReg) {
    uint8_t high = readRegister(highReg);
    uint8_t low = readRegister(lowReg);
    return static_cast<int16_t>(static_cast<uint16_t>(high) << 8 | low);
}
