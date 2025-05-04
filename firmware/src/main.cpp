#define DEBUG 1

#include "Arduino.h"
#include "FCS/FlightController.h"
#include "utils/log.h"

namespace {
FCS fcs(26, 25, 33, 32, 1000, 2000);
}

void setup() {
    DBG_BEGIN(115200);
    DBG("Starting");
    fcs.begin();
}

void loop() {}
