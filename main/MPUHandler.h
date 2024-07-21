#include <Wire.h>
#include <SPI.h>
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
        digitalWrite(MPU6000_CS_PIN, HIGH)
        writeRegister(MPU6000_ADDRESS, PWR_MGMT_1, 0x00);
    }

private:
    void writeRegister(uint8_t address, uint8_t reg, uint8_t data) {
        digitalWrite(MPU6000_CS_PIN, LOW);
        SPI.transfer(reg);
        SPI.transfer(data);
        digitalWrite(MPU6000_CS_PIN, HIGH);
    }
};
