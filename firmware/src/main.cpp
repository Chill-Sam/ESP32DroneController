#include "Arduino.h"
#include "FCS/FlightController.h"
#include "utils/log.h"

FCS *fcs;

void setup() {
    DBG_BEGIN(115200);
    DBG("Starting");
    fcs = new FCS(26, 25, 33, 32, 1000, 2000);
}

void loop() {}
