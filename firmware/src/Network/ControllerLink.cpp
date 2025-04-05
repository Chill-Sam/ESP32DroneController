#include "ControllerLink.h"
#include "Safety/DroneState.h"
#include "TCS/ThrustController.h"
#include <WiFi.h>

ControllerLink::ControllerLink(const char *ssid, const char *password,
                               const char *websocket)
    : ssid(ssid), password(password), websocket(websocket) {}

void ControllerLink::begin() {
    WiFi.begin(ssid, password);

    Serial.print("Connecting to " + String(ssid));

    // NOLINTNEXTLINE(readability-static-accessed-through-instance)
    while (WiFi.status() != WL_CONNECTED) {
        delay(100);
        Serial.print(".");
    }

    Serial.println(" Connected!");
}

void ControllerLink::sendTelemetry(DroneState state, TCSState thrustState) {
    return;
}
