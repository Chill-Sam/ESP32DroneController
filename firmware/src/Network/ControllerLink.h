#pragma once

#include "types/ControlData.h"
#include "types/DroneState.h"
#include <ArduinoJson.h>
#include <WebSocketsClient.h>

class ControllerLink {
  public:
    ControllerLink();

    void begin();
    void updateTelemetry(const DroneState &state);

    const SetpointState &setpointState;
    const RCArmingState &armingState;

    std::function<void()> onAuthenticateSuccess = []() {};
    std::function<void()> onTestRequest = []() {};
    std::function<void()> onDisconnect = []() {};
    std::function<void(const PIDTuningState &tuning)> onPIDTune =
        [](const PIDTuningState &tuning) {};

  private:
    const bool &isAuthenticated;
    bool _authenticated = false;
    SetpointState _setpointState{};
    RCArmingState _armingState{};

    WebSocketsClient client;
    JsonDocument telemetryBuffer;
    bool telemetryReady = false;

    void beginConnection();
    void webSocketEvent(WStype_t type, uint8_t *payload, size_t length);
    void handlePayload(uint8_t *payload);
    void authenticate(const char *nonce);
    void onAuthenticate();
    void handleCommand(JsonObjectConst command);
    void calculateSetpoints(Joystick left, Joystick right);
    void updatePIDTuning(JsonObjectConst tuning) const;

    static void webSocketTask(void *pvParameters);
    static String computeHMACSHA256(const String &key, const String &data);
};
