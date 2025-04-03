#include "FlightController.h"
#include "Network/ControllerLink.h"
#include "Safety/DroneState.h"

FCS::FCS(int pinA, int pinB, int pinC, int pinD, DroneState *state,
         ControllerLink *rc)
    : tcs(pinA, pinB, pinC, pinD), state(state), rc(rc) {}

void FCS::begin() {
    tcs.begin();
    ahrs.init();
    pidPitch.begin();
    pidRoll.begin();
    pidYaw.begin();

    xTaskCreatePinnedToCore(fcsTask, "FCU", 4096, this, 1, nullptr, 1);
}

void FCS::arm() {
    tcs.arm();
    armed = true;
}

void FCS::disarm() {
    tcs.disarm();
    armed = false;
}

void FCS::updateOrientation() {
    orientation.pitch = ahrs.pitch;
    orientation.roll = ahrs.roll;
    orientation.yaw = ahrs.yaw;
}

void FCS::update() {
    if (state->isFailsafe() || !state->isArmed()) {
        return;
    }

    TCSState thrustState = tcs.getTCSState();

    // TODO:
    // Calculate setpoints
    // Set PID to setpoints
    // Calculate PID based on orientation
    // Set speed of engines via TCS

    rc->sendTelemetry(*state, thrustState);
}

void FCS::fcsTask(void *pvParameters) {
    FCS *fcs = static_cast<FCS *>(pvParameters);

    const TickType_t rate = pdMS_TO_TICKS(2); // 500 Hz
    TickType_t last = xTaskGetTickCount();

    while (true) {
        fcs->update(); // routes AHRS data to PIDs, processes controller input
        vTaskDelayUntil(&last, rate);
    }
}
