#include "Arduino.h"
#include "ControllerLink.h"
#include "TCS/ThrustController.h"
#include "WebSocketsClient.h"
#include "WiFi.h"
#include "mbedtls/md.h"

#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#define WEBSOCKET "EXAMPLE.COM"
#define PORT 0000
#define URI "/EXAMPLE"
#define SECRET_PASS ""

#define CONTROL_ANGLE 20

ControllerLink::ControllerLink() {
    xTaskCreatePinnedToCore(webSocketTask, "websocket", 4096, this, 1, nullptr,
                            0);
}

void ControllerLink::webSocketTask(void *pvParameters) {
    auto *rc = static_cast<ControllerLink *>(pvParameters);
    const TickType_t rate = pdMS_TO_TICKS(20); // 50 Hz
    TickType_t last = xTaskGetTickCount();

    rc->begin();

    unsigned long lastTelemetry = 0;
    const unsigned long telemetryInterval = 100; // 10 Hz

    while (true) {
        rc->client.loop();

        unsigned long now = millis();
        if (now - lastTelemetry >= telemetryInterval && rc->authenticated) {
            lastTelemetry = now;
            rc->client.sendTXT(rc->tlm);
        }

        vTaskDelayUntil(&last, rate);
    }
}

void ControllerLink::begin() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");

    // NOLINTNEXTLINE(readability-static-accessed-through-instance)
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected");
    delay(500);

    client.beginSSL(WEBSOCKET, PORT, URI);
    client.onEvent([this](WStype_t type, uint8_t *payload, size_t length) {
        this->webSocketEvent(type, payload, length);
    });
}

void ControllerLink::handlePayload(uint8_t *payload) {
    JsonDocument json;
    deserializeJson(json, payload);

    const char *nonce = json["nonce"];
    const char *authstatus = json["status"];
    const char *command = json["type"];

    if (nonce != nullptr) {
        Serial.println("Attempting authentication");
        String reply = computeHMACSHA256(SECRET_PASS, nonce);
        JsonDocument response;
        response["role"] = "drone";
        response["signature"] = reply;

        String output;
        serializeJson(response, output);

        client.sendTXT(output);

    } else if (authstatus != nullptr) {
        Serial.println("Successfully authenticated!");
        authenticated = true;

    } else if (command != nullptr) {
        int test = json["payload"]["test"];
        if (test == 1 && !shouldTest) {
            Serial.println("Test command recieved");
            shouldTest = true;
        }

        float lx = json["payload"]["joysticks"]["left"][0];
        float ly = json["payload"]["joysticks"]["left"][1];

        float rx = json["payload"]["joysticks"]["right"][0];
        float ry = json["payload"]["joysticks"]["right"][1];

        setpointState.setpointPitch = map(constrain(ry, -100, 100), -100, 100,
                                          -CONTROL_ANGLE, CONTROL_ANGLE);
        setpointState.setpointRoll = map(constrain(rx, -100, 100), -100, 100,
                                         -CONTROL_ANGLE, CONTROL_ANGLE);
        setpointState.setpointYaw = map(constrain(lx, -100, 100), -100, 100,
                                        -CONTROL_ANGLE, CONTROL_ANGLE);

        JsonArray payloadArming = json["payload"]["arming"];
        armingState.RCArmedFCU = payloadArming[0];
        armingState.RCArmedA = payloadArming[1];
        armingState.RCArmedB = payloadArming[2];
        armingState.RCArmedC = payloadArming[3];
        armingState.RCArmedD = payloadArming[4];
    }
}

void ControllerLink::webSocketEvent(WStype_t type, uint8_t *payload,
                                    size_t length) {
    switch (type) {
    case WStype_DISCONNECTED:
        Serial.println("[WSS] Disconnected");
        break;
    case WStype_CONNECTED:
        Serial.println("[WSS] Connected");
        break;
    case WStype_TEXT:
        // Serial.printf("[WSS] Got message: %s\n", payload);
        handlePayload(payload);
        break;
    default:
        break;
    }
}

void ControllerLink::updateTelemetry(FlightMode flightMode,
                                     ThrottleState throttleState,
                                     Orientation orientation) {
    if (!authenticated) {
        return;
    }

    String mode;
    switch (flightMode) {
    case FlightMode::DISARMED:
        mode = "DISARMED";
    case FlightMode::ARMED:
        mode = "ARMED";
    case FlightMode::FAILSAFE:
        mode = "FAILSAFE";
    default:
        mode = "UNKNOWN";
    }

    JsonDocument msg;

    msg["type"] = "telemetry";

    JsonObject payload = msg["payload"].to<JsonObject>();
    payload["FlightMode"] = mode;

    JsonArray jsonThrottleState = payload["ThrottleState"].to<JsonArray>();
    jsonThrottleState.add(throttleState.ThrottleA);
    jsonThrottleState.add(throttleState.ThrottleB);
    jsonThrottleState.add(throttleState.ThrottleC);
    jsonThrottleState.add(throttleState.ThrottleD);

    JsonArray jsonOrientation = payload["Orientation"].to<JsonArray>();
    jsonOrientation.add(orientation.pitch);
    jsonOrientation.add(orientation.roll);
    jsonOrientation.add(orientation.yaw);
    jsonOrientation.add(orientation.alt);

    String output;
    serializeJson(msg, output);

    tlm = output;
}

String ControllerLink::computeHMACSHA256(const String &key,
                                         const String &data) {
    // Retrieve the SHA256 info structure.
    const mbedtls_md_info_t *mdInfo =
        mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);

    if (mdInfo == nullptr) {
        Serial.println("Failed to retrieve md info for SHA256.");
        return "";
    }

    // Output buffer (SHA256 produces 32 bytes)
    unsigned char hmacOutput[32];

    // Initialize the message digest context
    mbedtls_md_context_t ctx;
    mbedtls_md_init(&ctx);

    // Setup the context for HMAC operation (1 indicates HMAC is enabled)
    int ret = mbedtls_md_setup(&ctx, mdInfo, 1);

    // Start HMAC with the provided key
    ret = mbedtls_md_hmac_starts(
        &ctx, reinterpret_cast<const unsigned char *>(key.c_str()),
        key.length());

    // Process the input data
    ret = mbedtls_md_hmac_update(
        &ctx, reinterpret_cast<const unsigned char *>(data.c_str()),
        data.length());

    // Finalize the HMAC operation and write the result to hmac_output
    ret = mbedtls_md_hmac_finish(&ctx, hmacOutput);

    if (ret != 0) {
        mbedtls_md_free(&ctx);
        return "";
    }

    // Always free the context to avoid memory leaks.
    mbedtls_md_free(&ctx);

    // Convert the binary HMAC output to a hex-encoded string.
    String hexDigest = "";
    char hexBuffer[3]; // Two hex digits plus null terminator
    for (unsigned char i : hmacOutput) {
        sprintf(hexBuffer, "%02x", i);
        hexDigest += hexBuffer;
    }

    return hexDigest;
}
