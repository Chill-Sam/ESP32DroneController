#include "DroneState.h"

void DroneState::arm() {
    if (!isFailsafe()) {
        mode = FlightMode::ARMED;
    }
}

void DroneState::disarm() {
    if (!isFailsafe()) {
        mode = FlightMode::DISARMED;
    }
}

void DroneState::enterFailsafe() { mode = FlightMode::FAILSAFE; }
