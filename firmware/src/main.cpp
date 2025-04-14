#include "Arduino.h"
#include "FCS/FlightController.h"

#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#define WEBSOCKET ""

FCS fcs(26, 25, 33, 32, 1000, 2000, WIFI_SSID, WIFI_PASSWORD, WEBSOCKET);

void setup() {
    Serial.begin(115200);
    Serial.println("Starting");
    fcs.begin();
}
