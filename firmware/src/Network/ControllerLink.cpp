#include "ArduinoJson/Object/JsonObjectConst.hpp"
#include "ArduinoJson/Variant/JsonVariant.hpp"
#include "ArduinoJson/Variant/JsonVariantConst.hpp"
#include "ControllerLink.h"
#include "utils/log.h"
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <WiFi.h>
#include <cstring>
#include <mbedtls/md.h>

#define WIFI_SSID "Xiaomi11Net"
#define WIFI_PASSWORD "safepass"
#define WEBSOCKET "chillsam.ddns.net"
#define PORT 8012
#define URI "/echo"
#define SECRET_PASS "dronesecret"

#define CONTROL_ANGLE 20

ControllerLink::ControllerLink()
    : isAuthenticated(_authenticated), setpointState(_setpointState),
      armingState(_armingState) {
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
        if (now - lastTelemetry >= telemetryInterval && rc->isAuthenticated) {
            lastTelemetry = now;
            rc->client.sendTXT(rc->telemetryBuffer);
        }

        vTaskDelayUntil(&last, rate);
    }
}

void ControllerLink::begin() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    DBG("Connecting to WiFi");

    // NOLINTNEXTLINE(readability-static-accessed-through-instance)
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        DBG(".");
    }
    DBG("\nWiFi Connected\n");
    delay(500);

    client.beginSSL(WEBSOCKET, PORT, URI);
    client.onEvent([this](WStype_t type, uint8_t *payload, size_t length) {
        this->webSocketEvent(type, payload, length);
    });
}

void ControllerLink::webSocketEvent(WStype_t type, uint8_t *payload,
                                    size_t /*length*/) {
    switch (type) {
    case WStype_DISCONNECTED:
        DBG("[WSS] Disconnected\n");
        onDisconnect();
        break;
    case WStype_CONNECTED:
        DBG("[WSS] Connected\n");
        break;
    case WStype_TEXT:
        DBG_FMT("[WSS] Got message: %s\n", payload);
        handlePayload(payload);
        break;
    default:
        break;
    }
}

void ControllerLink::handlePayload(uint8_t *payload) {
    JsonDocument json;
    deserializeJson(json, payload);

    const char *nonce = json["nonce"];
    const char *authstatus = json["status"];
    const char *command = json["type"];

    if (nonce != nullptr) {
        authenticate(nonce);
    } else if (authstatus != nullptr) {
        onAuthenticate();
    } else if (command != nullptr && strcmp(command, "command") == 0) {
        handleCommand(json["payload"].as<JsonObjectConst>());
    }
}

void ControllerLink::authenticate(const char *nonce) {
    DBG("Attempting authentication\n");
    String reply = computeHMACSHA256(SECRET_PASS, nonce);
    DBG_FMT("Computed hash: %s\n", reply);

    JsonDocument response;
    response["role"] = "drone";
    response["signature"] = reply;

    String output;
    serializeJson(response, output);

    client.sendTXT(output);
}

void ControllerLink::onAuthenticate() {
    DBG("Successfully authenticated");
    _authenticated = true;
}

void ControllerLink::handleCommand(JsonObjectConst command) {
    // Handle a test commnad
    JsonVariantConst testFlag = command["test"];
    if (testFlag != nullptr && testFlag.as<bool>() && onTestRequest) {
        onTestRequest();
    }

    JsonObjectConst joysticks = command["joysticks"];
    if (joysticks != nullptr) {

        auto parseJoystick = [](JsonVariantConst v) -> Joystick {
            Joystick j;
            JsonArrayConst arr = v.as<JsonArrayConst>();
            if (!arr.isNull() && arr.size() == 2) {
                j.x = arr[0].as<float>();
                j.y = arr[1].as<float>();
            }
            return j;
        };

        Joystick leftStick = parseJoystick(joysticks["left"]);
        Joystick rightStick = parseJoystick(joysticks["right"]);

        calculateSetpoints(leftStick, rightStick);
    }

    JsonArrayConst arming = command["arming"];
    if (arming != nullptr && arming.size() == 5) {
        _armingState.RCArmedFCU = arming[0];
        _armingState.RCArmedA = arming[1];
        _armingState.RCArmedB = arming[2];
        _armingState.RCArmedC = arming[3];
        _armingState.RCArmedD = arming[4];
    }
}

void ControllerLink::calculateSetpoints(Joystick left, Joystick right) {
    static uint32_t lastMicros = micros();
    uint32_t now = micros();
    float dt = (now - lastMicros) / 1000000.0f; // convert µs to seconds
    dt = constrain(dt, 0.0f, 0.1f);

    lastMicros = now;

    auto scaleInput = [](float val) {
        return map(constrain(val, -100.0f, 100.0f), -100.0f, 100.0f,
                   -CONTROL_ANGLE, CONTROL_ANGLE);
    };

    auto scaleAltitudeRate = [](float val) {
        return map(constrain(val, -100.0f, 100.0f), -100.0f, 100.0f, -1.0f,
                   1.0f); // m/s
    };

    _setpointState.setpointPitch = scaleInput(right.y);
    _setpointState.setpointRoll = scaleInput(right.x);
    _setpointState.setpointYaw = scaleInput(left.x);

    float verticalVelocity = scaleAltitudeRate(left.y);
    _setpointState.setpointAltitude += verticalVelocity * dt;
}

void ControllerLink::updateTelemetry(const DroneState &state) {
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
