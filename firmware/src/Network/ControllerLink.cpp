#include "ControllerLink.h"
#include "utils/log.h"
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <WiFi.h>
#include <cstring>
#include <mbedtls/md.h>

#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#define WEBSOCKET "EXAMPLE.COM"
#define PORT 0000
#define URI "/EXAMPLE"
#define SECRET_PASS ""

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

        const unsigned long now = millis();
        const bool shouldSendTelemetry =
            rc->isAuthenticated && (now - lastTelemetry >= 100);

        if (shouldSendTelemetry) {
            lastTelemetry = now;
            String telemetry;
            serializeJson(rc->telemetryBuffer, telemetry);
            rc->client.sendTXT(telemetry);
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
    float dt = (now - lastMicros) / 1000000.0F; // convert µs to seconds
    dt = constrain(dt, 0.0F, 0.1F);

    lastMicros = now;

    auto scaleInput = [](float val) {
        return map(constrain(val, -100.0F, 100.0F), -100.0F, 100.0F,
                   -CONTROL_ANGLE, CONTROL_ANGLE);
    };

    auto scaleAltitudeRate = [](float val) {
        return map(constrain(val, -100.0F, 100.0F), -100.0F, 100.0F, -1.0F,
                   1.0F); // m/s
    };

    _setpointState.setpointPitch = scaleInput(right.y);
    _setpointState.setpointRoll = scaleInput(right.x);
    _setpointState.setpointYaw = scaleInput(left.x);

    float verticalVelocity = scaleAltitudeRate(left.y);
    _setpointState.setpointAltitude += verticalVelocity * dt;
}

void ControllerLink::updateTelemetry(const DroneState &state) {
    if (!isAuthenticated) {
        return;
    }

    telemetryBuffer.clear();
    telemetryBuffer["type"] = "telemetry";

    const char *modeStrings[] = {"DISARMED", "ARMED", "FAILSAFE", "UNKNOWN"};
    telemetryBuffer["payload"]["FlightMode"] =
        modeStrings[static_cast<uint8_t>(state.flightMode)];

    JsonArray t = telemetryBuffer["payload"]["ThrottleState"].to<JsonArray>();
    t.add(state.throttleState.ThrottleA);
    t.add(state.throttleState.ThrottleB);
    t.add(state.throttleState.ThrottleC);
    t.add(state.throttleState.ThrottleD);

    JsonArray o = telemetryBuffer["payload"]["Orientation"].to<JsonArray>();
    o.add(state.orientation.pitch);
    o.add(state.orientation.roll);
    o.add(state.orientation.yaw);
    o.add(state.orientation.alt);
}

String ControllerLink::computeHMACSHA256(const String &key,
                                         const String &data) {
    const mbedtls_md_info_t *mdInfo =
        mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (mdInfo != nullptr) {
        DBG("[HMAC] Failed to get SHA256 md info.");

        return "";
    }

    unsigned char hmac[32];
    mbedtls_md_context_t ctx;
    mbedtls_md_init(&ctx);

    // NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)
    if (mbedtls_md_setup(&ctx, mdInfo, 1) != 0 ||
        mbedtls_md_hmac_starts(
            &ctx, reinterpret_cast<const unsigned char *>(key.c_str()),
            key.length()) != 0 ||
        mbedtls_md_hmac_update(
            &ctx, reinterpret_cast<const unsigned char *>(data.c_str()),
            data.length()) != 0 ||
        mbedtls_md_hmac_finish(&ctx, &hmac[0]) != 0) {
        DBG("[HMAC] HMAC computation failed.");
        mbedtls_md_free(&ctx);
        return "";
    }
    // NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)

    mbedtls_md_free(&ctx);

    // Convert to hex
    char hexBuffer[65]; // 64 chars + null terminator
    for (size_t i = 0; i < sizeof(hmac); ++i) {
        sprintf(&hexBuffer[i * 2], "%02x", hmac[i]);
    }
    return {static_cast<const char *>(hexBuffer)};
}
