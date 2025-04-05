#include "FCS/FlightController.h"
#include "Network/ControllerLink.h"
#include "Safety/DroneState.h"

#define WIFI_SSID ""
#define PASSWORD ""
#define WEBSOCKET ""

namespace {

DroneState state;
ControllerLink rc(WIFI_SSID, PASSWORD, WEBSOCKET);
FCS fcs(26, 25, 33, 32, &state, &rc);

} // namespace

void setup() {
    Serial.begin(115200);
    Serial.println("Starting");
    fcs.begin();
}

void loop() {
    if (!state.shouldFly()) {
        return;
    }

    // TODO: Run safety system
    // TODO: Get controls from Network
    // TODO: Update FCS
}
