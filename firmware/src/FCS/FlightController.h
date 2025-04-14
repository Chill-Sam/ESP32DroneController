#pragma once

#include "AHRS/AHRS.h"
#include "PID.h"
#include "TCS/ThrustController.h"

enum class FlightMode : uint8_t { DISARMED, ARMED, FAILSAFE };

struct ThrottleState {
    float ThrottleA = 0.0F;
    float ThrottleB = 0.0F;
    float ThrottleC = 0.0F;
    float ThrottleD = 0.0F;
};

struct ArmState {
    bool ArmedA = false;
    bool ArmedB = false;
    bool ArmedC = false;
    bool ArmedD = false;
};

struct SetpointState {
    float setpointPitch = 0.0F;
    float setpointRoll = 0.0F;
    float setpointYaw = 0.0F;
    float setpointAltitude = 0.0F;
};

struct RCArmingState {
    bool RCArmedA = false;
    bool RCArmedB = false;
    bool RCArmedC = false;
    bool RCArmedD = false;
};

struct DroneState {
    // Onboard Controlled
    FlightMode flightMode = FlightMode::DISARMED;
    Orientation orientation;
    ThrottleState throttleState;
    ArmState armState;

    // Remote Controlled
    SetpointState setpointState;
    RCArmingState rcArmingState;
};

class FCS {
  public:
    FCS(int pinA, int pinB, int pinC, int pinD, int minPulsewidth,
        int maxPulsewidth, const char *ssid, const char *password,
        const char *websocket);

    void begin();

  private:
    DroneState state;
    AHRS ahrs;
    TCS tcs;

    PID pidPitch{1.0F, 1.0F, 1.0F};
    PID pidRoll{1.0F, 1.0F, 1.0F};
    PID pidYaw{1.0F, 1.0F, 1.0F};

    void updateOrientation();
    void updateThrottleState();
    void updateArmState();

    void arm();
    void disarm();

    static void update(void *pvParameters);
};
