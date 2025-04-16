#pragma once

#include "AHRS/AHRS.h"
#include "Network/ControllerLink.h"
#include "PID.h"
#include "TCS/ThrustController.h"

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
        int maxPulsewidth);

    void begin();
    void loopWS() { rc.loopWS(); }

  private:
    DroneState state;
    DroneState oldState;
    AHRS ahrs;
    TCS tcs;
    ControllerLink rc;

    PID pidPitch{1.0F, 1.0F, 1.0F};
    PID pidRoll{1.0F, 1.0F, 1.0F};
    PID pidYaw{1.0F, 1.0F, 1.0F};

    void updateOrientation();
    void updateThrottleState();
    void updateArmState();
    void updateControls();

    void arm();
    void disarm();

    static void update(void *pvParameters);
    static void telemetry(void *pvParameters);
};
