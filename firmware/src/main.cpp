#include "Arduino.h"
#include "FCS/FlightController.h"

FCS *fcs;

void setup() {
    Serial.begin(115200);
    Serial.println("Starting");
    fcs = new FCS(26, 25, 33, 32, 1000, 2000);
    fcs->begin();
}

void loop() {
    fcs->loopWS();
    delay(20);
}
