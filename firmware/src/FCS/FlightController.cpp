#include "Arduino.h"
#include "FlightController.h"
#include "TCS/ThrustController.h"

FCS::FCS(int pinA, int pinB, int pinC, int pinD, int minPulseWidth,
         int maxPulsewidth)
    : tcs(pinA, pinB, pinC, pinD, minPulseWidth, maxPulsewidth) {
    // ahrs.init();
    arm();
    xTaskCreatePinnedToCore(update, "FCU", 4096, this, 2, nullptr, 1);
    xTaskCreatePinnedToCore(rateLoop, "RateLoop", 4096, this, 2, nullptr, 1);
    xTaskCreatePinnedToCore(angleLoop, "AngleLoop", 4096, this, 1, nullptr, 1);
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

void FCS::arm() {
    if (state.flightMode == FlightMode::FAILSAFE) {
        Serial.println("DRONE IS FAILSAFED");
        tcs.disarm();
        return;
    }

    state.flightMode = FlightMode::ARMED;
}
void FCS::disarm() {
    if (state.flightMode == FlightMode::FAILSAFE) {
        Serial.println("DRONE IS FAILSAFED");
        tcs.disarm();
        return;
    }

    state.flightMode = FlightMode::DISARMED;
    tcs.disarm();
}

void FCS::failsafe() {
    state.flightMode = FlightMode::FAILSAFE;
    tcs.disarm();
}

void FCS::angleLoop(void *pvParameters) {
    FCS *fcs = static_cast<FCS *>(pvParameters);
    const TickType_t rate = pdMS_TO_TICKS(10); // 100 Hz
    TickType_t last = xTaskGetTickCount();

    while (true) {
        fcs->updateOuterLoop();
        vTaskDelayUntil(&last, rate);
    }
}

void FCS::rateLoop(void *pvParameters) {
    FCS *fcs = static_cast<FCS *>(pvParameters);
    const TickType_t rate = pdMS_TO_TICKS(2); // 500 Hz
    TickType_t last = xTaskGetTickCount();

    while (true) {
        fcs->updateInnerLoop();
        vTaskDelayUntil(&last, rate);
    }
}

void FCS::updateOuterLoop() {
    pidPitchAngle.calc(state.setpointState.setpointPitch,
                       state.orientation.pitch);
}

void FCS::updateInnerLoop() {
    pidPitchRate.calc(pidPitchAngle.output, ahrs.gx);
}

void FCS::update(void *pvParameters) {
    FCS *fcs = static_cast<FCS *>(pvParameters);

    const TickType_t rate = pdMS_TO_TICKS(2); // 500 Hz
    TickType_t last = xTaskGetTickCount();

    while (true) {
        if (!fcs->rc.authenticated) {
            vTaskDelayUntil(&last, rate);
            continue;
        }
        // Update telemetry in the RC
        fcs->rc.updateTelemetry(fcs->state.flightMode, fcs->state.throttleState,
                                fcs->state.orientation);

        // Update State
        fcs->updateOrientation();
        fcs->updateThrottleState();
        fcs->updateArmState();
        fcs->updateControls();

        // Dont do anything if FAILSAFED
        if (fcs->state.flightMode == FlightMode::FAILSAFE) {
            Serial.println("DRONE IS FAILSAFED");
            fcs->disarm();
            vTaskDelayUntil(&last, rate);
            continue;
        }

        // Test the drone
        if (fcs->rc.shouldTest) {
            Serial.println("Should Test");
            fcs->tcs.test();
            fcs->rc.shouldTest = false;
        }

        // Update PIDs
        // fcs->pidPitch.calc(fcs->state.setpointState.setpointPitch,
        // fcs->state.orientation.pitch);
        // fcs->pidRoll.calc(fcs->state.setpointState.setpointRoll,
        // fcs->state.orientation.roll);
        // fcs->pidYaw.calc(fcs->state.setpointState.setpointYaw,
        // fcs->state.orientation.yaw);

        // Should FCS be armed?
        if (fcs->state.rcArmingState.RCArmedFCU &&
            fcs->state.flightMode == FlightMode::DISARMED) {
            Serial.println("Arming FCU");
            fcs->arm();
        } else if (!fcs->state.rcArmingState.RCArmedFCU &&
                   fcs->state.flightMode == FlightMode::ARMED) {
            Serial.println("I am now disarming FCU");
            fcs->disarm();
        }

        // Is FCS disarmed?
        if (fcs->state.flightMode == FlightMode::DISARMED) {
            Serial.println("FCS is disarmed");
            // fcs->tcs.disarm();
            vTaskDelayUntil(&last, rate);
            continue;
        }

        // Arm / Disarm motors
        std::array<bool, 4> arming = {fcs->state.rcArmingState.RCArmedA,
                                      fcs->state.rcArmingState.RCArmedB,
                                      fcs->state.rcArmingState.RCArmedC,
                                      fcs->state.rcArmingState.RCArmedD};

        for (std::size_t i = 0; i < arming.size(); ++i) {
            bool armed = arming[i];
            int motorId = static_cast<int>(i) + 1; // yields 1,2,3,4
            if (armed && fcs->tcs.isArmed(motorId)) {
                fcs->tcs.armMotor(motorId);
            } else if (!armed && fcs->tcs.isArmed(motorId)) {
                fcs->tcs.disarmMotor(motorId);
            }
        }

        // Set speeds
        Serial.println("Pitch PID: " + String(fcs->pidPitchRate.output));

        vTaskDelayUntil(&last, rate);
    }
}
