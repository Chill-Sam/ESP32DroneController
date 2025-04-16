#pragma once

#include "mbedtls/md.h"
#include <ArduinoJson.h>
#include <WebSocketsClient.h>

class ControllerLink {
  public:
    ControllerLink();

    void loopWS() { client.loop(); }

  private:
    WebSocketsClient client;

    void handlePayload(uint8_t *payload);
    void webSocketEvent(WStype_t type, uint8_t *payload, size_t length);

    static String computeHMACSHA256(const String &key, const String &data);
};
