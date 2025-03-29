#include "AHRS/AHRS.h"

namespace {
struct Orientation {
    float pitch = 0.0F;
    float roll = 0.0F;
    float yaw = 0.0F;
};

AHRS ahrs;
Orientation orientation;

void getOrientation() {
    orientation.pitch = ahrs.pitch;
    orientation.roll = ahrs.roll;
    orientation.yaw = ahrs.yaw;
}
} // namespace

void setup() {
    Serial.begin(115200);
    Serial.println("Starting");
    ahrs.init();
}

void loop() {
    getOrientation();

    Serial.println("pitch:" + String(orientation.pitch) +
                   ",roll:" + String(orientation.roll) +
                   ",yaw:" + String(orientation.yaw));
}
