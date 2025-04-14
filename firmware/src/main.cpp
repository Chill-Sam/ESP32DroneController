#include "Arduino.h"
#include "TCS/ThrustController.h"

TCS tcs(26, 25, 33, 32, 1000, 2000);

void setup() {
    Serial.begin(115200);
    Serial.println("Starting");
    tcs.test();
}

void loop() {}
