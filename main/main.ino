/**
 * @file main.ino
 *
 * @brief Manages the control system of a drone, including sensor fusion, PID control, and motor throttle.
 *
 * This program initializes and controls various components of a drone, including the MPU6000
 * accelerometer and gyroscope, DPS310 pressure sensor, QMC5883L magnetometer, and motor control via ESCs.
 *
 * Dependencies:
 * - Arduino.h: Core Arduino functions.
 * - cmath.h: Math library
 * - WiFi.h: WiFi connectivity.
 * - HTTPClient.h: HTTP client for web communication.
 * - ArduinoJson.h: JSON parsing library.
 * - Wire.h: I2C communication.
 * - PIDController.h: PID control algorithms.
 * - Adafruit_Sensor.h: Unified sensor library.
 * - QMC5883LCompass.h: Magnetometer library.
 * - MahonyAHRS.h: Sensor fusion algorithm.
 * - ConnectionManager.h: Manages WiFi and HTTP connections.
 * - EngineController.h: Manages motor control via ESCs.
 * - MPUHandler.h: Interfaces with the MPU6000 sensor.
 * - DPSHandler.h: Interfaces with the DPS310 sensor.
 */

#include <Arduino.h>
#include <cmath>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <PIDController.h>
#include <Adafruit_Sensor.h>
#include <QMC5883LCompass.h>
#include <MahonyAHRS.h>
#include <Adafruit_DPS310.h>

#include <ConnectionManager.h>
#include <EngineController.h>
#include <MPUHandler.h>

// Sensor and filter instances
QMC5883LCompass compass;
Adafruit_DPS310 dps;
MPUHandler mpu;
Mahony filter;

// Network settings
const char* ssid = "";
const char* password = "";
const char* address = "";

// Connection manager instance
ConnectionManager connection(ssid, password, address);

// JSON document for parsing incoming data
StaticJsonDocument<2048> doc;

// Joystick positions and switch state
float joystick1[2], joystick2[2];
bool isON;

// Timing variables
unsigned long previousTime = 0;

// ESC control settings
std::array<const int, 4> escPins = {16, 17, 18, 19};
const int frequency = 50;
const int resolution = 16;
const int minPulseWidth = 1000;
const int maxPulseWidth = 2000;

// Engine controller instance
EngineController controller(escPins, frequency, resolution, minPulseWidth, maxPulseWidth);

// Drone state variables
float dronePitch = 0;
float droneRoll = 0;
float droneYaw = 0;
float dronePressure = 0; // Relative pressure change
float startPressure;

// PID controllers and tuning parameters
PIDController pidPitch, pidRoll, pidYaw, pidAlt;
double pitchKp = 0.7, pitchKi = 0.7, pitchKd = 0.1;
double rollKp = 0.7, rollKi = 0.7, rollKd = 0.1;
double yawKp = 0.7, yawKi = 0.7, yawKd = 0.1;
double altKp = 0.7, altKi = 0.7, altKd = 0.1;
double pitchSetPoint, rollSetPoint, yawSetPoint, pressureSetPoint = 0;
double maxPitch, maxRoll, maxYaw = 15;
double maxPressure = 2;

// Initial lift-off value
int liftOffValue = 10;

/**
 * @brief Parses a JSON payload to extract joystick and switch states.
 *
 * @param payload The JSON payload to decode.
 */
void decodeJson(String payload) {
    //StaticJsonDocument<1024> doc;  // Adjust size based on JSON complexity

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
 * @brief Manually sets the motor throttle based on serial input.
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
 * @brief Calculates the drone's orientation angles and relative altitude using sensor data and the Mahony filter.
 */
void calculateAngles() {
    // Read MPU6000 data
    std::array<float, 6> data = mpu.getData();
    float accelX = data[0];
    float accelY = data[1];
    float accelZ = data[2];
    float gyroX = data[3];
    float gyroY = data[4];
    float gyroZ = data[5];

    // Read QMC5883L data
    float magX = compass.getX();
    float magY = compass.getY();
    float magZ = compass.getZ();

    // Update the filter
    filter.update(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, magX, magY, magZ);

    // Get orientation in terms of pitch, roll, and yaw
    dronePitch = filter.getPitch();
    droneRoll = filter.getRoll();
    droneYaw = filter.getYaw();

    // Get altitude
    sensors_event_t temp_event, pressure_event;
  
    while (!dps.temperatureAvailable() || !dps.pressureAvailable()) {
      yield(); // wait until there's something to read
    }

    dps.getEvents(&temp_event, &pressure_event);
    Serial.print(F("Pressure = "));
    Serial.print(pressure_event.pressure);
    Serial.println(" hPa"); 

    Serial.println();
    dronePressure = pressure_event.pressure - startPressure; 
}

/**
 * @brief Performs PID control to adjust motor speeds based on desired setpoints and sensor data.
 */
void pidControl() {
  float MAX_PID = 10.0;
  float MIN_PID = -10.0;

  // Initialize PID controllers for pitch and roll
  pidPitch.begin();
  pidPitch.setpoint(pitchSetPoint); // Desired pitch setpoint
  pidPitch.tune(pitchKp, pitchKi, pitchKd);
  pidPitch.limit(MIN_PID, MAX_PID); // Set output limits for pitch

  pidRoll.begin();
  pidRoll.setpoint(rollSetPoint); // Desired roll setpoint
  pidRoll.tune(rollKp, rollKi, rollKd);
  pidRoll.limit(MIN_PID, MAX_PID); // Set output limits for roll

  pidYaw.begin();
  pidYaw.setpoint(yawSetPoint); // Desired yaw setpoint
  pidYaw.tune(yawKp, yawKi, yawKd);
  pidYaw.limit(MIN_PID, MAX_PID); // Set output limits for yaw

  pidAlt.begin();
  pidAlt.setpoint(pressureSetPoint);
  pidAlt.tune(altKp, altKi, altKd);
  pidAlt.limit(MIN_PID, MAX_PID);

  // Compute PID control outputs
  float outputPitch = pidPitch.compute(dronePitch);
  float outputRoll = pidRoll.compute(droneRoll);
  float outputYaw = pidYaw.compute(droneYaw);
  float outputPressure = pidAlt.compute(dronePressure);

  // Print the control outputs for debugging
  Serial.print("outputPitch: ");
  Serial.println(outputPitch);
  Serial.print("outputRoll: ");
  Serial.println(outputRoll);

  Serial.print("outputYaw: ");
  Serial.println(outputYaw);
  Serial.print("outputPressure: ");
  Serial.println(outputPressure);

  // Set engine speeds based on PID outputs, ensuring values are within 0-100 range
  controller.setEngineSpeed(1, constrain(liftOffValue + outputPitch - outputRoll, 0, 100));
  controller.setEngineSpeed(2, constrain(liftOffValue - outputPitch - outputRoll, 0, 100));
  controller.setEngineSpeed(3, constrain(liftOffValue + outputPitch + outputRoll, 0, 100));
  controller.setEngineSpeed(4, constrain(liftOffValue - outputPitch + outputRoll, 0, 100));
}

/**
 * @brief Adjusts PID tuning parameters based on serial input for one-time tuning.
 */
void setPID() {
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
          pitchKp = value2;
          Serial.println("Set p value to " + String(pitchKp));
        }
        else if (value == 1){
          pitchKi = value2;
          Serial.println("Set i value to " + String(pitchKi));
        }
        else if (value == 2){
          pitchKd = value2;
          Serial.println("Set d value to " + String(pitchKd));
        }
        else if (value == 3){
          liftOffValue = value2;
          Serial.println("Set lift-off value to " + String(liftOffValue));
        }
      }
    }
}

/**
 * @brief Updates the control setpoints based on joystick inputs.
 */
void updateSetpoints() {
  float MAX_PITCH_ANGLE = 15.0;
  float MAX_ROLL_ANGLE = 15.0;
  float YAW_COEFFICIENT = 0.001;
  float PRESSURE_COEFFICIENT = 0.001; 

  pitchSetPoint = joystick1[1] * MAX_PITCH_ANGLE;
  rollSetPoint = joystick1[0] * MAX_ROLL_ANGLE;

  unsigned long currentTime = millis();  // Get the current time
  float dt = (currentTime - previousTime);
  if (dt > 0) {
    if (abs(joystick2[1]) >= abs(joystick2[0])) {
      pressureSetPoint += joystick2[1] * PRESSURE_COEFFICIENT * dt;
      pressureSetPoint = constrain(pressureSetPoint, 0, maxPressure);
    } else {
      yawSetPoint += joystick2[0] * YAW_COEFFICIENT * dt;
    }
  }
  previousTime = currentTime;
}

/**
 * @brief Initializes the drone control system, including sensors, network connection, and ESCs.
 */
void setup() {
  Serial.begin(115200);
  Serial.println("System Online");

  mpu.initializeMPU();
  mpu.calculateOffsets();

  if (! dps.begin_SPI(15)) {  // If you want to use SPI
    Serial.println("Failed to find DPS");
    while (1) yield();
  }
  Serial.println("DPS OK!");

  dps.configurePressure(DPS310_128HZ, DPS310_128SAMPLES);
  dps.configureTemperature(DPS310_128HZ, DPS310_128SAMPLES);

  delay(1500);

  sensors_event_t temp_event, pressure_event;
  
  while (!dps.temperatureAvailable() || !dps.pressureAvailable()) {
    return; // wait until there's something to read
  }

  dps.getEvents(&temp_event, &pressure_event);

  startPressure = pressure_event.pressure;

  connection.connectToWifi();
  connection.connectToWebsite();

  controller.initializeESC();
  controller.stopEngines(3000); // Resets engines to 0% throttle to initialize them
}

/**
 * @brief Main loop to handle drone control, including sensor updates, PID control, and motor throttle adjustments.
 */
void loop() {
  String payload = connection.getPayloadFromAddress();
  decodeJson(payload);

  if (isON) {
    calculateAngles();
    updateSetpoints();
    // setPID();
    pidControl();
    // setMotorThrottleManual();
    delay(10);
  }
  else {
    controller.stopEngines(1000);
    Serial.println("Off");
  }
}
