/**
 * @file ConnectionManager.h
 *
 * @brief Manages WiFi and HTTP connections for Arduino devices.
 *
 * This class encapsulates methods to manage WiFi connections and HTTP requests in
 * Arduino sketches. It provides functionalities to connect to a WiFi network,
 * connect to a website via HTTP, and retrieve content from an HTTP server.
 *
 * Dependencies:
 * - WiFi.h: Manages WiFi connections.
 * - HTTPClient.h: Handles HTTP requests.
 *
 * Example Usage:
 * ```cpp
 * ConnectionManager connManager("yourSSID", "yourPassword", "http://example.com/api/data");
 * connManager.connectToWifi();
 * connManager.connectToWebsite();
 * String payload = connManager.getPayloadFromAddress();
 * Serial.println(payload);
 * ```
 */

#include <WiFi.h>
#include <HTTPClient.h>

/**
 * @class ConnectionManager
 *
 * @brief Manages network connections for Arduino-based devices.
 *
 * This class provides comprehensive management of network connectivity for Arduino
 * devices. It encapsulates the complexity of handling WiFi connections and performing
 * HTTP requests. The class uses the Arduino standard libraries WiFi.h and HTTPClient.h
 * to establish WiFi connections and to send HTTP requests, respectively.
 *
 * The class supports connecting to a specified WiFi network, initializing a connection
 * to a web server, and retrieving data from a web address using HTTP GET requests.
 * It ensures that all network interactions are robustly handled, reporting the status
 * and errors through the Serial interface.
 *
 * @note The device must be equipped with networking capabilities (e.g., ESP32 or any
 * Arduino module with WiFi capabilities).
 *
 * Features:
 * - Connect to WiFi using specified SSID and password.
 * - Initialize HTTP connections to a specified server URL.
 * - Fetch and return payloads from the server as strings.
 *
 * How to use:
 * 1. Instantiate ConnectionManager with network and server details.
 * 2. Call connectToWifi() to establish a WiFi connection.
 * 3. Call connectToWebsite() to prepare the HTTP client for requests.
 * 4. Use getPayloadFromAddress() to retrieve data from the configured server.
 *
 * The functionality is demonstrated in the example provided in the header comment.
 *
 * @param ssid The SSID of the WiFi network to connect to.
 * @param password The password for the WiFi network.
 * @param address The URL of the HTTP server to which the HTTP requests are sent.
 */
class ConnectionManager {
public:
    /**
     * @brief Constructor to initialize ConnectionManager with WiFi and server details.
     * 
     * @param ssid The WiFi network SSID.
     * @param password The WiFi network password.
     * @param address The URL of the HTTP server to connect to.
     */
    ConnectionManager(const char* ssid, const char* password, const char* address) :
        ssid(ssid), password(password), address(address) {}

    /**
     * @brief Connects to the specified WiFi network.
     *
     * This method attempts to connect to the WiFi network specified during the class instantiation.
     * It will print the connection status to the Serial monitor.
     */
    void connectToWifi() {
        Serial.print("Attempting WiFi connection to " + String(ssid));
        WiFi.begin(ssid, password);

        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }

        Serial.println("\nWiFi connected");
    }

    /**
     * @brief Initializes the HTTP client with the server address.
     *
     * Prepares the HTTPClient object to connect to the provided server URL.
     */
    void connectToWebsite() {
        http.begin(address);
    }

    /**
     * @brief Fetches the payload from the server address.
     *
     * Sends an HTTP GET request to the configured server address. It checks the connection
     * status before attempting the request and reports any errors.
     *
     * @return String The response from the server as a string. Returns an empty string if the request fails.
     */
    String getPayloadFromAddress() {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi not connected.");
            return "";
        }

        int http_code = http.GET();
        if (http_code != HTTP_CODE_OK) {
            Serial.println("HTTP Get Request failed. Code: " + http_code);
            return "";
        }

        return http.getString();
    }
    
private:
    const char* ssid;      ///< WiFi SSID to connect to
    const char* password;  ///< WiFi password
    const char* address;   ///< HTTP server URL
    HTTPClient http;       ///< HTTP client instance
};
