#pragma once

#include "AHRS/AHRS.h"
#include "Network/ControllerLink.h"
#include "PID.h"
#include "Safety/DroneState.h"
#include "TCS/ThrustController.h"

class FCS {
  public:
    FCS(int pinA, int pinB, int pinC, int pinD, DroneState *state,
        ControllerLink *rc);

    void begin();
    void arm();
    void disarm();
    void update();

  private:
    void updateOrientation();
    static void fcsTask(void *pvParameters);

    AHRS ahrs;
    TCS tcs;
    bool armed = false;

    Orientation orientation;
    DroneState *state;
    ControllerLink *rc;

    PID pidPitch{1.0, 1.0, 1.0};
    PID pidRoll{1.0, 1.0, 1.0};
    PID pidYaw{1.0, 1.0, 1.0};
};
