// NOTE: Class that connects, sends and recieves commands via internet

// TODO:
// - Recieve data from websocket
// - Failsafe on network error
// - Able to arm / disarm
// - Update joystick control
// - Send telemetery (throttle, arming state)

#pragma once

#include "Safety/DroneState.h"
#include "TCS/ThrustController.h"

struct Input {
    float pitch;
    float roll;
    float yaw;
    float alt;
};

class ControllerLink {
  public:
    ControllerLink(const char *ssid, const char *password,
                   const char *websocket);

    void begin();
    void onRecieve();
    void sendTelemetry(DroneState state, TCSState thrustState);

  private:
    const char *ssid;
    const char *password;
    const char *websocket;
};
