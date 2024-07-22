#include <Wire.h>
#include <SPI.h>
#include <array>
#include <cstdint>

#define MPU6000_CS_PIN 29
#define MPU6000_SCK_PIN 30
#define MPU6000_MISO_PIN 31
#define MPU6000_MOSI_PIN 37

#define MPU6000_ADDRESS 0x68
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H 0x43
#define GYRO_YOUT_H 0x45
#define GYRO_ZOUT_H 0x47
#define PWR_MGMT_1 0x6B

class MPUHandler {
public:
    MPUHandler() {}

    void initializeMPU() {
        Wire.begin();
        SPI.begin(MPU6000_SCK_PIN, MPU6000_MISO_PIN, MPU6000_MOSI_PIN, MPU6000_CS_PIN);
        pinMode(MPU6000_CS_PIN, OUTPUT);
        digitalWrite(MPU6000_CS_PIN, HIGH);
        writeRegister(MPU6000_ADDRESS, PWR_MGMT_1, 0x00);
    }

    std::array<float, 6> getData() {
        int16_t ax, ay, az, gx, gy, gz;
        ax = readRegister16(MPU6000_ADDRESS, ACCEL_XOUT_H);
        ay = readRegister16(MPU6000_ADDRESS, ACCEL_YOUT_H);
        az = readRegister16(MPU6000_ADDRESS, ACCEL_ZOUT_H);
        gx = readRegister16(MPU6000_ADDRESS, GYRO_XOUT_H);
        gy = readRegister16(MPU6000_ADDRESS, GYRO_YOUT_H);
        gz = readRegister16(MPU6000_ADDRESS, GYRO_ZOUT_H);

        float accelX = ax / 16384.0;
        float accelY = ay / 16384.0;
        float accelZ = az / 16384.0;
        float gyroX = gx / 131.0;
        float gyroY = gy / 131.0;
        float gyroZ = gz / 131.0;

        std::array<float, 6> data{accelX, accelY, accelZ, gyroX, gyroY, gyroZ};
        return data;
    }

private:
    void writeRegister(uint8_t address, uint8_t reg, uint8_t data) {
        digitalWrite(MPU6000_CS_PIN, LOW);
        SPI.transfer(reg);
        SPI.transfer(data);
        digitalWrite(MPU6000_CS_PIN, HIGH);
    }

    int16_t readRegister16(uint8_t address, uint8_t reg) {
        digitalWrite(MPU6000_CS_PIN, LOW);
        SPI.transfer(reg | 0x80);
        int16_t value = SPI.transfer(0x00) << 8 | SPI.transfer(0x00);
        digitalWrite(MPU6000_CS_PIN, HIGH);
        return value;
    }
};
