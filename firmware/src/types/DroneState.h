#pragma once

#include "ControlData.h"
#include "FlightData.h"

struct DroneState {
    // Onboard Controlled
    FlightMode flightMode = FlightMode::DISARMED;
    Orientation orientation;
    SpeedData speedData;
    ThrottleState throttleState;
    ArmState armState;

    // Remote Controlled
    SetpointState setpointState;
    RCArmingState rcArmingState;
};
