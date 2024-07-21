/**
 * @file main.ino
 *
 * @brief Drone Control System Initialization and Configuration.
 *
 * This script initializes and configures a drone control system using various components including
 * a WiFi module, motor controllers, sensors, and PID controllers for stability. The configuration includes
 * network settings for retrieving joystick data, motor control via ESCs, and sensor data processing
 * with Kalman and complementary filters.
 *?
 * Dependencies:
 * - Arduino.h: Core Arduino library.
 * - WiFi.h: Handles WiFi operations.
 * - HTTPClient.h: Facilitates HTTP communications.
 * - ArduinoJson.h: Parses JSON formatted data.
 * - Wire.h: Communication interface library for I2C devices.
 * - MPU6050_light.h: Lightweight library for the MPU6050 sensor.
 * - PIDController.h: Library for PID control implementation.
 * - KalmanFilter.h, ConnectionManager.h, EngineController.h: Custom libraries/classes for specific system functionalities.
 *
 * System Components:
 * - WiFi and HTTP communication for remote data fetching.
 * - MPU6050 sensor for motion tracking.
 * - PID controllers for adjusting the pitch and roll.
 * - Kalman filters for noise reduction in sensor data.
 * - Motor controllers for executing flight commands based on control inputs.
 *
 * Configuration Details:
 * - Network credentials and endpoint URL for fetching control settings.
 * - ESC pin assignments and PWM settings for motor control.
 * - Initial PID tuning parameters for flight control stability.
 * - MPU6050 and Kalman filter setup for accurate angle measurements.
 *
 * Initialization Steps:
 * 1. Configure WiFi and HTTP connections for data reception.
 * 2. Initialize motor control settings with specific ESC pinouts and PWM specifications.
 * 3. Set up the MPU6050 sensor on the I2C bus.
 * 4. Configure Kalman filters for pitch and roll measurements.
 * 5. Initialize PID controllers with starting parameters for pitch and roll control.
 *
 * Example Usage:
 * The setup allows the drone to receive joystick commands over WiFi, process these commands to adjust motor speeds,
 * and use sensor data to maintain balance and orientation. The system is designed to operate within a loop where it continually
 * updates its control outputs based on sensor readings and external commands.
 *
 * ```cpp
 * void setup() {
 *     Serial.begin(115200);
 *     connection.connectToWifi();
 *     controller.initializeESC();
 *     mpu.begin();
 * }
 *
 * void loop() {
 *     connection.getPayloadFromAddress();  // Fetch joystick data if available
 *     calculateAngles();  // Update pitch and roll based on sensor data
 *     pidControl();  // Adjust motor outputs based on PID calculations
 *     delay(10);
 * }
 * ```
 *
 * @note Ensure that all external libraries are correctly installed and the hardware is properly connected as per the specified pin assignments and configurations.
 */

/*
* TODO: Remove MPU6050 and replace with MPU6000
* TODO: Add magnetometer and barometer measurements
* TODO: Replace KalmanFilter/Complementary with MahonyFilter
* TODO: Add PID for yaw and altitude
* TODO: Update documentation
*/

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <PIDController.h>

#include "KalmanFilter.h"
#include "ConnectionManager.h"
#include "EngineController.h"

// Network settings
const char* ssid = "";
const char* password = "";
const char* address = "";

// Initialize WiFi and HTTP connection manager
ConnectionManager connection(ssid, password, address);

// JSON document for parsing incoming data
StaticJsonDocument<1024> doc;

// Joystick positions and switch state
float joystick1[2], joystick2[2];
bool isON;

// ESC control settings
std::array<const int, 4> escPins = {16, 17, 18, 19};
const int frequency = 50;
const int resolution = 16;
const int minPulseWidth = 1000;
const int maxPulseWidth = 2000;

// Initialize motor control
EngineController controller(escPins, frequency, resolution, minPulseWidth, maxPulseWidth);

MPU6050 mpu(Wire);
KalmanFilter kalmanPitch(0.02, 0.01, 1.0, 0.0, 1.0);
KalmanFilter kalmanRoll(0.02, 0.01, 1.0, 0.0, 1.0);

float alpha = 0.96;
float compPitch = 0, compRoll = 0; // Complementary filter output
unsigned long lastTime = 0; // Time of the last measurement

float dronePitch = 0;
float droneRoll = 0;

PIDController pidPitch, pidRoll;
double Kp = 0.7, Ki = 0.7, Kd = 0.3; // Initial PID parameters, need tuning

int liftOffValue = 10;

/**
 * @brief Decodes JSON data to extract joystick positions and a switch state.
 *
 * This function parses a JSON string to update the positions of two joysticks and the state of a switch.
 * It assumes that global variables for storing these values (joystick1, joystick2, isON) are pre-defined.
 * The joystick values are expected to be floats within the range of -1 to 1.
 * The function checks for errors during JSON parsing and prints an error message to the Serial console if
 * the parsing fails.
 *
 * Expected JSON structure:
 * {
 *   "Values": {
 *     "Joy1": {"x": float, "y": float},
 *     "Joy2": {"x": float, "y": float},
 *     "Switch": bool
 *   }
 * }
 *
 * If the JSON structure is incorrect or parsing fails, the function will print "Parse Failure" and no global
 * variables will be updated.
 *
 * Usage:
 * Call this function with a JSON string that adheres to the expected structure.
 *
 * Example Usage:
 * ```cpp
 * String json = "{\"Values\":{\"Joy1\":{\"x\":-0.5,\"y\":0.75},\"Joy2\":{\"x\":0.3,\"y\":-0.3},\"Switch\":true}}";
 * decodeJson(json);
 * ```
 *
 * @param payload String containing the JSON data to be parsed.
 */
void decodeJson(String payload) {
    StaticJsonDocument<256> doc;  // Adjust size based on JSON complexity

    // Attempt to parse the JSON payload
    DeserializationError error = deserializeJson(doc, payload);
    if (error) {
        Serial.println("Parse Failure");
        return; // Exit if there is a parsing error
    }

    // Extract joystick and switch states from the JSON object
    joystick1[0] = doc["Values"]["Joy1"]["x"].as<float>(); // X coordinate of the first joystick
    joystick1[1] = doc["Values"]["Joy1"]["y"].as<float>(); // Y coordinate of the first joystick
    joystick2[0] = doc["Values"]["Joy2"]["x"].as<float>(); // X coordinate of the second joystick
    joystick2[1] = doc["Values"]["Joy2"]["y"].as<float>(); // Y coordinate of the second joystick
    isON         = doc["Values"]["Switch"];                // State of the switch
}

/**
 * @brief Reads throttle settings from serial input and sets the engine speed accordingly.
 *
 * This function listens for incoming serial data, expected in the format "engineNumber throttle".
 * It parses the input to extract the engine number and throttle percentage, and then applies these
 * settings using the `setEngineSpeed` method of a previously instantiated `controller` object.
 *
 * Expected Serial Input Format:
 * "engineNumber throttle"
 * - engineNumber: An integer identifying the engine (1-4 for individual engines, 5 for all engines).
 * - throttle: A float value representing the throttle percentage (0-100).
 *
 * Operation:
 * - Waits for a complete line of input terminated by a newline character.
 * - Trims any leading or trailing whitespace from the input.
 * - Parses the input to separate the engine number from the throttle value.
 * - If the input format is correct, it updates the engine speed as specified.
 * 
 * Example Serial Input:
 * "2 75"
 * This will set engine number 2 to 75% throttle.
 *
 * Usage:
 * Call this function repeatedly within the main loop to continuously monitor and respond to serial input.
 * Ensure that a valid `controller` instance of an `EngineController` class is accessible globally.
 *
 * Notes:
 * - The function assumes that serial data arrives in a correct and timely manner without errors.
 * - If there is no serial data or the format is incorrect, no action is taken.
 */
void setMotorThrottleManual() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // Remove any extraneous whitespace.

    // Parse input assuming the format "engineNumber throttle"
    int separatorIndex = input.indexOf(" ");
    if (separatorIndex > 0) {  // Ensure there's a space separating the components.
      int engineNumber = input.substring(0, separatorIndex).toInt();  // Convert the engine number to an integer.
      float throttle = input.substring(separatorIndex + 1).toFloat();  // Convert the throttle value to a float.

      // Use the controller object to set the engine speed.
      controller.setEngineSpeed(engineNumber, throttle);
    }
  }
}

/**
 * @brief Computes and filters pitch and roll angles using sensor data.
 *
 * This function updates pitch and roll measurements from an MPU6050 sensor and applies
 * complementary and Kalman filters to smooth out the measurements. This is crucial for
 * applications such as drone flight control where accurate and stable angle readings are required.
 *
 * Process:
 * 1. Update sensor readings from the MPU6050.
 * 2. Calculate the time interval since the last update to get accurate dt for the filters.
 * 3. Read the current pitch and roll angles, as well as gyroscope rates from the sensor.
 * 4. Apply a complementary filter to merge the accelerometer (angle) and gyroscope (rate) data.
 * 5. Update Kalman filters based on the output from the complementary filter.
 * 6. Store and print the filtered pitch and roll values for use in controlling the drone.
 *
 * Variables:
 * - `compPitch` and `compRoll`: Variables storing the angle values from the complementary filter.
 * - `kalmanPitch` and `kalmanRoll`: Instances of KalmanFilter class used for further filtering of angles.
 * - `dronePitch` and `droneRoll`: Final filtered angles used for drone control.
 * - `alpha`: Tuning parameter for the complementary filter, balancing accelerometer and gyroscope data.
 *
 * @note This function is designed to be called in a loop to continuously update and filter the angles.
 *       A fixed delay at the end of the function helps in managing the update frequency.
 *
 * Example Usage:
 * Ensure that `alpha`, `compPitch`, `compRoll`, `kalmanPitch`, and `kalmanRoll` are properly initialized before calling this function.
 *
 * ```cpp
 * // Assuming `alpha`, `compPitch`, `compRoll`, `kalmanPitch`, and `kalmanRoll` are defined elsewhere
 * while (true) {
 *     calculateAngles();
 *     delay(10); // Delay to control loop frequency
 * }
 * ```
 */
void calculateAngles() {
  mpu.update();  // Update sensor readings from MPU6050

  unsigned long currentTime = millis(); // Get current time
  float dt = (currentTime - lastTime) / 1000.0; // Calculate the time interval in seconds
  lastTime = currentTime; // Update lastTime for the next cycle

  float pitch = mpu.getAngleX(); // Get current pitch angle from accelerometer
  float roll = mpu.getAngleY(); // Get current roll angle from accelerometer
  float gyroXrate = mpu.getGyroX(); // Get current X-axis rotation rate from gyroscope
  float gyroYrate = mpu.getGyroY(); // Get current Y-axis rotation rate from gyroscope

  // Complementary filter calculation
  compPitch = alpha * (compPitch + gyroXrate * dt) + (1 - alpha) * pitch;
  compRoll = alpha * (compRoll + gyroYrate * dt) + (1 - alpha) * roll;

  // Kalman filter prediction and update
  kalmanPitch.predict();
  kalmanRoll.predict();
  kalmanPitch.update(compPitch, compPitch - kalmanPitch.getState());
  kalmanRoll.update(compRoll, compRoll - kalmanRoll.getState());

  // Retrieve the filtered angles from Kalman filters
  dronePitch = kalmanPitch.getState();
  droneRoll = kalmanRoll.getState();

  // Output the filtered angles for monitoring
  Serial.print("Filtered Pitch: ");
  Serial.print(dronePitch);
  Serial.print(" Filtered Roll: ");
  Serial.println(droneRoll);

  delay(10); // Short delay to stabilize loop frequency
}

/**
 * @brief Performs PID control to stabilize the drone's pitch and roll.
 *
 * This function initializes and runs PID controllers for pitch and roll to stabilize the drone.
 * It sets the desired setpoints, tunes the PID parameters, computes the control outputs based on
 * the current angles, and adjusts the engine speeds accordingly to maintain stability.
 *
 * Process Overview:
 * 1. Initializes the PID controllers for pitch and roll.
 * 2. Sets the desired setpoints (0 for both pitch and roll).
 * 3. Tunes the PID controllers with the specified gains (Kp, Ki, Kd).
 * 4. Limits the output values to prevent excessive control signals.
 * 5. Computes the control outputs based on the current pitch and roll angles.
 * 6. Sets the engine speeds using the computed PID outputs, ensuring the values are within valid limits.
 *
 * Variables and Parameters:
 * - `pidPitch`, `pidRoll`: Instances of PID controller objects for pitch and roll control.
 * - `Kp`, `Ki`, `Kd`: PID tuning parameters (proportional, integral, derivative gains).
 * - `dronePitch`, `droneRoll`: Current pitch and roll angles obtained from the sensor data.
 * - `liftOffValue`: Base throttle value for the engines.
 * - `outputPitch`, `outputRoll`: Computed control outputs for pitch and roll.
 *
 * Engine Control Logic:
 * - Engine 1: Adjusted by the difference between lift-off value, pitch, and roll.
 * - Engine 2: Adjusted by the difference between lift-off value, pitch, and roll, inverted.
 * - Engine 3: Adjusted by the sum of lift-off value, pitch, and roll.
 * - Engine 4: Adjusted by the sum of lift-off value, pitch, and roll, inverted.
 *
 * Example Usage:
 * This function should be called in the main control loop, repeatedly updating the PID control
 * and engine speeds based on the latest sensor readings.
 *
 * ```cpp
 * void loop() {
 *     calculateAngles();
 *     pidControl();
 *     delay(10); // Delay to control loop frequency
 * }
 * ```
 */
void pidControl() {
  // Initialize PID controllers for pitch and roll
  pidPitch.begin();
  pidPitch.setpoint(0); // Desired pitch setpoint
  pidPitch.tune(Kp, Ki, Kd);
  pidPitch.limit(-10, 10); // Set output limits for pitch

  pidRoll.begin();
  pidRoll.setpoint(0); // Desired roll setpoint
  pidRoll.tune(Kp, Ki, Kd);
  pidRoll.limit(-10, 10); // Set output limits for roll

  // Compute PID control outputs
  float outputPitch = pidPitch.compute(dronePitch);
  float outputRoll = pidRoll.compute(droneRoll);

  // Print the control outputs for debugging
  Serial.print("outputPitch: ");
  Serial.println(outputPitch);
  Serial.print("outputRoll: ");
  Serial.println(outputRoll);

  // Set engine speeds based on PID outputs, ensuring values are within 0-100 range
  controller.setEngineSpeed(1, constrain(liftOffValue + outputPitch - outputRoll, 0, 100));
  controller.setEngineSpeed(2, constrain(liftOffValue - outputPitch - outputRoll, 0, 100));
  controller.setEngineSpeed(3, constrain(liftOffValue + outputPitch + outputRoll, 0, 100));
  controller.setEngineSpeed(4, constrain(liftOffValue - outputPitch + outputRoll, 0, 100));
}

/**
 * @brief Adjusts PID controller settings or the lift-off value based on serial input.
 *
 * This function listens for incoming serial data formatted to specify a control parameter 
 * and its new value. It allows dynamic adjustment of the PID gains (Kp, Ki, Kd) and the lift-off
 * throttle setting during runtime, which is useful for tuning controller settings without
 * needing to recompile the code.
 *
 * Expected Serial Input Format:
 * "<parameterIndex> <value>"
 * - parameterIndex: An integer (0, 1, 2, or 3) specifying which parameter to set:
 *   - 0: Proportional gain (Kp)
 *   - 1: Integral gain (Ki)
 *   - 2: Derivative gain (Kd)
 *   - 3: Lift-off value
 * - value: A float specifying the new value for the selected parameter.
 *
 * Operation:
 * 1. Checks if there is serial data available.
 * 2. Reads the input until a newline character is encountered.
 * 3. Trims any whitespace from the ends of the input string.
 * 4. Parses the input to extract the parameter index and its new value.
 * 5. Sets the specified parameter to the new value and confirms the action via serial output.
 *
 * Examples of Serial Input:
 * - "0 0.5" sets Kp to 0.5
 * - "1 0.1" sets Ki to 0.1
 * - "2 0.05" sets Kd to 0.05
 * - "3 50" sets the lift-off value to 50
 *
 * Usage:
 * This function should be called within the main loop to continuously monitor and respond to
 * serial input for real-time PID tuning.
 *
 * ```cpp
 * void loop() {
 *   setPID();
 *   // Other operations such as pidControl();
 * }
 * ```
 *
 * @note Ensure global variables for Kp, Ki, Kd, and liftOffValue are defined and accessible.
 */
void setPID(){
    // Check if there is incoming serial data
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n'); // Read the input until newline
      input.trim(); // Trim whitespace from the input

      // Find the index of the first space character
      int separatorIndex = input.indexOf(" ");
      if (separatorIndex > 0) { // Ensure there's a space character in the input
        int value = input.substring(0, separatorIndex).toInt(); // Parse the parameter index
        float value2 = input.substring(separatorIndex + 1).toFloat(); // Parse the value

        // Set the corresponding PID parameter based on the index
        if (value == 0){
          Kp = value2;
          Serial.println("Set p value to "+String(Kp));
        }
        else if (value == 1){
          Ki = value2;
          Serial.println("Set i value to "+String(Ki));
        }
        else if (value == 2){
          Kd = value2;
          Serial.println("Set d value to "+String(Kd));
        }
        else if (value == 3 ){
          liftOffValue = value2;
          Serial.println("Set lift-off value to "+String(liftOffValue));
        }
      }
    }
}


void setup()
{
  Serial.begin(115200);
  Serial.println("System Online");

  Wire.begin();
  byte status = mpu.begin();
  if (status != 0) {
    Serial.println("MPU connection failed. Check your connections.");
    while(1);
  }

  mpu.calcOffsets(); // Calibrate MPU6050
  Serial.println("MPU6050 ready!");

  connection.connectToWifi();
  connection.connectToWebsite();

  controller.initializeESC();
  controller.stopEngines(3000); // Resets engines to 0% throttle to initialize them
}


void loop()
{
  String payload = connection.getPayloadFromAddress();
  decodeJson(payload);

  if (isON) {
    calculateAngles();
    setPID();
    pidControl();
    // setMotorThrottleManual();
  }
  else {
    controller.stopEngines(1000);
    Serial.println("Off");
  }
}
