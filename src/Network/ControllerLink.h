// NOTE: Class that connects, sends and recieves commands via internet

// TODO:
// - Connect to internet
// - Recieve data from websocket
// - Failsafe on network error
// - Able to arm / disarm
// - Update joystick control
// - Send telemetery (throttle, arming state)

#pragma once

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

  private:
    const char *ssid;
    const char *password;
    const char *websocket;
};
