#pragma once

class FCS {
  public:
    FCS(int pinA, int pinB, int pinC, int pinD, const char *ssid,
        const char *password, const char *websocket);

    void begin();
    void armMotors();
    void disarmMotors();
};
