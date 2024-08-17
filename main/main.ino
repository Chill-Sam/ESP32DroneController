#include <Arduino.h>
#include <ArduinoJson.h>
#include <PIDController.h>

#include "ConnectionManager.h"
#include "EngineController.h"
#include "AHRS.h"

// Attitude and heading reference system
AHRS ahrs;

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
std::array<const int, 4> escPins = {26, 25, 33, 32};
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
double rollKp = 0.0, rollKi = 0.0, rollKd = 0.0;
double yawKp = 0.7, yawKi = 0.7, yawKd = 0.1;
double altKp = 0.7, altKi = 0.7, altKd = 0.1;
double pitchSetPoint, rollSetPoint, yawSetPoint, pressureSetPoint = 0;
double maxPitch, maxRoll, maxYaw = 15;
double maxPressure = 2;

// Initial lift-off value
int liftOffValue = 15;

void decodeJson(String payload) {
  StaticJsonDocument<1024> doc;  // Adjust size based on JSON complexity

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


void pidControl() {
  float MAX_PID = 15.0;
  float MIN_PID = -15.0;

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
  //Serial.print("outputPitch: ");
  //Serial.println(outputPitch);
  //Serial.print("outputRoll:");
  //Serial.println(outputRoll);

  //Serial.print("outputYaw: ");
  //Serial.println(outputYaw);
  //Serial.print("outputPressure: ");
  //Serial.println(outputPressure);

  // Set engine speeds based on PID outputs, ensuring values are within 0-100 range
  controller.setEngineSpeed(1, constrain(liftOffValue - outputRoll, 0, 100));
  controller.setEngineSpeed(2, constrain(liftOffValue - outputRoll, 0, 100));
  controller.setEngineSpeed(3, constrain(liftOffValue + outputRoll, 0, 100));
  controller.setEngineSpeed(4, constrain(liftOffValue + outputRoll, 0, 100));
}


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
        rollKp = value2;
        Serial.println("Set p value to " + String(rollKp));
      }
      else if (value == 1){
        rollKi = value2;
        Serial.println("Set i value to " + String(rollKi));
      }
      else if (value == 2){
        rollKd = value2;
        Serial.println("Set d value to " + String(rollKd));
      }
      else if (value == 3){
        liftOffValue = value2;
        Serial.println("Set lift-off value to " + String(liftOffValue));
      }
    }
  }
}


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


void setup() {
  Serial.begin(115200);
  Serial.println("System Online");

  ahrs.init();

  connection.connectToWifi();
  connection.connectToWebsite();

  controller.initializeESC();
  controller.stopEngines(3000); // Resets engines to 0% throttle to initialize them

  delay(1000);
}


void loop() {
  static unsigned long nowMillis;
  static unsigned long thenMillis;
  String payload = connection.getPayloadFromAddress();
  decodeJson(payload);
  
  if (isON) {
    nowMillis = millis();
    Serial.println(nowMillis - thenMillis);

    ahrs.readData();
    ahrs.update();
    dronePitch = ahrs.pitch;
    droneRoll = ahrs.roll;
    droneYaw = ahrs.yaw;
    Serial.print("droneRoll:");
    Serial.println(droneRoll);
    Serial.print("dronePitch:");
    Serial.println(dronePitch);
    Serial.print("droneYaw:");
    Serial.println(droneYaw);

    updateSetpoints();
    //setPID();
    pidControl();
    //setMotorThrottleManual();

    thenMillis = nowMillis;
  }
  else {
    controller.stopEngines(1000);
    Serial.println("Off");
  }
}
