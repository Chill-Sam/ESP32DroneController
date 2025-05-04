#pragma once

#include "AHRS/AHRS.h"
#include "Network/ControllerLink.h"
#include "PID.h"
#include "TCS/ThrustController.h"
#include "types/ControlData.h"
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

    PID outerPitch{0.0F, 0.0F, 0.0F};
    PID outerRoll{0.0F, 0.0F, 0.0F};
    PID outerYaw{0.0F, 0.0F, 0.0F};
    PID outerAlt{0.0F, 0.0F, 0.0F};

    PID innerPitch{0.0F, 0.0F, 0.0F};
    PID innerRoll{0.0F, 0.0F, 0.0F};
    PID innerYaw{0.0F, 0.0F, 0.0F};
    PID innerAlt{0.0F, 0.0F, 0.0F};

    float pitchCommand = 0.0F;
    float rollCommand = 0.0F;
    float yawCommand = 0.0F;
    float altCommand = 0.0F;

    void updateMotorSpeed();
    void updateMotorArming();
    void updatePID();
    void tunePID(const PIDTuningState &tuning);
    void updateState();
    void arm();
    void disarm();
    void failsafe();
    void startLoop();

    static void control(void *pvParameters);
};
