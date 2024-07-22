/* 
* TODO: Finalize sensor fusion
* TODO: Add barometer sensor handling
* TODO: Add PIDs for altitude and yaw 
* TODO: Turn joystick input into pitch, roll, yaw and altitude setpoints
* TODO: Add documentation
*/

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <PIDController.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <MahonyAHRS.h>

#include "KalmanFilter.h"
#include "ConnectionManager.h"
#include "EngineController.h"
#include "MPUHandler.h"

// Barometer settings
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_BMP3XX bmp; // For BMP310

// Initialize mahony filter
Mahony filter;

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

// Initialize MPUHandler
MPUHandler mpu;

// Float for pitch, roll, yaw and relative altitude of drone
float dronePitch = 0;
float droneRoll = 0;
float droneYaw = 0;
float droneAltitude = 0; // NOTE: This is relative altitude

// NOTE: IMPORTANT: NEEDS TO BE TUNED BEFORE FLYING
PIDController pidPitch, pidRoll, pidYaw, pidAlt;
double pitchKp = 0.7, pitchKi = 0.7, pitchKd;
double rollKp = 0.7, rollKi = 0.7, rollKd;
double yawKp = 0.7, yawKi = 0.7, yawKd;
double altKp = 0.7, altKi = 0.7, altKd;
double pitchSetPoint, rollSetPoint, yawSetPoint, altSetPoint = 0;
double maxPitch, maxRoll, maxYaw = 15;
double maxAlt = 1;

// NOTE: Needs to be tuned properly
int liftOffValue = 10;

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

// NOTE: Using mahony filter for drone sensor fusion
void calculateAngles() {
    std::array<float, 6> data = mpu.getData();  // Update sensor readings from MPU6000
    
    float accelX = data[0];
    float accelY = data[1];
    float accelZ = data[2];
    float gyroX = data[3];
    float gyroY = data[4];
    float gyroZ = data[5];
}

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

// NOTE: Made for one time tuning use.
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

  mpu.initializeMPU();

  // Initialize HMC5883L
  if (!mag.begin()) {
    Serial.println("HMC5883L not detected");
    while (1);
  }

  // Initialize BMP310
  if (!bmp.begin_I2C()) { // Default I2C address is 0x76
    Serial.println("BMP310 not detected");
    while (1);
  }

  // Set up BMP310 sensor settings
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

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
