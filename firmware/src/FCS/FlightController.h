#pragma once

#include "AHRS/AHRS.h"
#include "Network/ControllerLink.h"
#include "PID.h"
#include "TCS/ThrustController.h"
#include "types/DroneState.h"

class FCS {
  public:
    FCS(int pinA, int pinB, int pinC, int pinD, int minPulsewidth,
        int maxPulsewidth);

    void begin();

  private:
    DroneState state;
    AHRS ahrs;
    TCS tcs;
    ControllerLink rc;

    PID outerPitch{1.0F, 1.0F, 1.0F};
    PID outerRoll{1.0F, 1.0F, 1.0F};
    PID outerYaw{1.0F, 1.0F, 1.0F};
    PID outerAlt{1.0F, 1.0F, 1.0F};

    PID innerPitch{1.0F, 1.0F, 1.0F};
    PID innerRoll{1.0F, 1.0F, 1.0F};
    PID innerYaw{1.0F, 1.0F, 1.0F};
    PID innerAlt{1.0F, 1.0F, 1.0F};

    float pitchCommand = 0;
    float rollCommand = 0;
    float yawCommand = 0;
    float altCommand = 0;

    void updateMotorSpeed();
    void updateMotorArming();
    void updatePID();
    void updateState();
    void arm();
    void disarm();
    void failsafe();
    void startLoop();

    static void control(void *pvParameters);
};
