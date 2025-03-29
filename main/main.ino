#include "AHRS.h"

AHRS ahrs;

volatile float pitch, roll, yaw;

void setup() {
    Serial.begin(115200);
    Serial.println("Starting");
    ahrs.init();
}

void getOrientation() {
    pitch = ahrs.pitch;
    roll = ahrs.roll;
    yaw = ahrs.yaw;
}

void loop() {
    getOrientation();

    Serial.println("pitch:" + String(pitch) + ",roll:" + String(roll) +
                   ",yaw:" + String(yaw));
}
