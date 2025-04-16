#pragma once

#include "AHRS/AHRS.h"
#include "TCS/ThrustController.h"
#include <ArduinoJson.h>
#include <WebSocketsClient.h>

struct SetpointState {
    float setpointPitch = 0.0F;
    float setpointRoll = 0.0F;
    float setpointYaw = 0.0F;
    float setpointAltitude = 0.0F;
};

struct RCArmingState {
    bool RCArmedA = false;
    bool RCArmedB = false;
    bool RCArmedC = false;
    bool RCArmedD = false;
};

class ControllerLink {
  public:
    ControllerLink();

    void loopWS() { client.loop(); }
    void sendTelemetry(FlightMode flightMode, ThrottleState throttleState,
                       Orientation orientation);

    SetpointState setpointState;
    RCArmingState armingState;

  private:
    WebSocketsClient client;
    bool authenticated = false;

    void handlePayload(uint8_t *payload);
    void webSocketEvent(WStype_t type, uint8_t *payload, size_t length);

    static String computeHMACSHA256(const String &key, const String &data);
};
