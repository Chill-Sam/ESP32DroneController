#include "Arduino.h"
#include "FlightController.h"
#include "freertos/portmacro.h"
#include "freertos/projdefs.h"

FCS::FCS(int pinA, int pinB, int pinC, int pinD, int minPulseWidth,
         int maxPulsewidth)
    : tcs(pinA, pinB, pinC, pinD, minPulseWidth, maxPulsewidth) {
    //    ahrs.init();
}

void FCS::updateOrientation() {
    state.orientation.pitch = ahrs.pitch;
    state.orientation.roll = ahrs.roll;
    state.orientation.yaw = ahrs.yaw;
}

void FCS::updateThrottleState() {
    state.throttleState.ThrottleA = tcs.getThrottle(1);
    state.throttleState.ThrottleB = tcs.getThrottle(2);
    state.throttleState.ThrottleC = tcs.getThrottle(3);
    state.throttleState.ThrottleD = tcs.getThrottle(4);
}

void FCS::updateArmState() {
    state.armState.ArmedA = tcs.isArmed(1);
    state.armState.ArmedB = tcs.isArmed(2);
    state.armState.ArmedC = tcs.isArmed(3);
    state.armState.ArmedD = tcs.isArmed(4);
}

void FCS::updateControls() {
    state.rcArmingState = rc.armingState;
    state.setpointState = rc.setpointState;
}

void FCS::begin() {
    arm();
    xTaskCreatePinnedToCore(update, "FCU", 4096, this, 2, nullptr, 1);
    xTaskCreatePinnedToCore(telemetry, "telemetry", 4096, this, 0, nullptr, 0);
}
void FCS::arm() {
    if (state.flightMode == FlightMode::FAILSAFE) {
        Serial.println("DRONE IS FAILSAFED");
        return;
    }

    state.flightMode = FlightMode::ARMED;
}
void FCS::disarm() {
    if (state.flightMode == FlightMode::FAILSAFE) {
        Serial.println("DRONE IS FAILSAFED");
        return;
    }

    state.flightMode = FlightMode::DISARMED;
}

void FCS::update(void *pvParameters) {
    FCS *fcs = static_cast<FCS *>(pvParameters);

    const TickType_t rate = pdMS_TO_TICKS(2); // 500 Hz
    TickType_t last = xTaskGetTickCount();

    while (true) {
        if (fcs->state.flightMode == FlightMode::FAILSAFE) {
            Serial.println("DRONE IS FAILSAFED");
            fcs->tcs.disarm();
            vTaskDelayUntil(&last, rate);
            continue;
        }

        // Update State
        fcs->updateOrientation();
        fcs->updateThrottleState();
        fcs->updateArmState();

        if (fcs->rc.shouldTest) {
            Serial.println("Should Test");
            fcs->tcs.test();
            fcs->rc.shouldTest = false;
        }

        // TODO:
        // Update PIDs
        // Decide on action based on state
        // Execute action

        vTaskDelayUntil(&last, rate);
    }
}

void FCS::telemetry(void *pvParameters) {
    FCS *fcs = static_cast<FCS *>(pvParameters);

    const TickType_t rate = pdMS_TO_TICKS(1000); // 1 Hz
    TickType_t last = xTaskGetTickCount();

    while (true) {
        fcs->rc.sendTelemetry(fcs->state.flightMode, fcs->state.throttleState,
                              fcs->state.orientation);
        vTaskDelayUntil(&last, rate);
    }
}
