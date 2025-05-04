#include "Arduino.h"
#include "FCS/FlightController.h"
#include "utils/log.h"

void setup() {
    DBG_BEGIN(115200);
    DBG("Starting");
    FCS fcs(26, 25, 33, 32, 1000, 2000);
    fcs.begin();
}

void loop() {}
