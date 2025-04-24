#pragma once

#include "AHRS/AHRS.h"
#include "Network/ControllerLink.h"
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

    static void control(void *pvParameters);
};
