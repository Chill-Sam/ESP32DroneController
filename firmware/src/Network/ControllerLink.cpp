#include "ControllerLink.h"
#include "WebSocketsClient.h"
#include "WiFi.h"

#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#define WEBSOCKET "EXAMPLE.COM"
#define PORT 0000
#define URI "/EXAMPLE"
#define SECRET_PASS ""

ControllerLink::ControllerLink() {
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

    if (nonce != nullptr) {
        Serial.println("Attempting authentication");
        String reply = computeHMACSHA256(SECRET_PASS, nonce);
        client.sendTXT(R"({"role":"drone","signature":")" + reply + "\"}");
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
        Serial.printf("[WSS] Got message: %s\n", payload);
        handlePayload(payload);
        break;
    default:
        break;
    }
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
