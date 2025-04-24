#include "FlightController.h"
#include "types/FlightData.h"
#include "utils/log.h"

FCS::FCS(int pinA, int pinB, int pinC, int pinD, int minPulseWidth,
         int maxPulsewidth)
    : tcs(pinA, pinB, pinC, pinD, minPulseWidth, maxPulsewidth) {}

void FCS::begin() {
    ahrs.begin();
    tcs.begin();

    rc.begin();

    rc.onDisconnect = [this]() { this->failsafe(); };
    rc.onTestRequest = [this]() { this->tcs.test(); };
    rc.onAuthenticateSuccess = [this]() { this->startLoop(); };
}

void FCS::startLoop() {
    // Create main control loop task on core 1 with medium priority
    xTaskCreatePinnedToCore(control, "FCU Control", 4092, this, 2, nullptr, 1);
}

void FCS::control(void *pvParameters) {
    FCS *fcs = static_cast<FCS *>(pvParameters);

    const TickType_t rate = pdMS_TO_TICKS(2); // 500 Hz
    TickType_t last = xTaskGetTickCount();

    while (true) {
        fcs->updateState();

        // Prevent further action if drone is in FAILSAFE
        if (fcs->state.flightMode == FlightMode::FAILSAFE) {
            DBG("[FCS] FAILSAFE")
            vTaskDelayUntil(&last, rate);
            continue;
        }

        // We update PIDs before ARM check to keep the PIDs consistent.
        fcs->updatePID();

        // Arm/Disarm FCS first before DISARM/ARM check
        if (fcs->state.rcArmingState.RCArmedFCU) {
            fcs->arm();
        } else {
            fcs->disarm();
        }

        if (fcs->state.flightMode == FlightMode::DISARMED) {
            DBG("[FCS] Currently disarmed");
            fcs->tcs.disarm();
            fcs->tcs.stop();
            vTaskDelayUntil(&last, rate);
            continue;
        } // Drone should be armed if it passes this

        if (fcs->state.flightMode != FlightMode::ARMED) {
            DBG("[FCS] PANIC: unreachable");
            fcs->failsafe();
            vTaskDelayUntil(&last, rate);
            continue;
        }

        // Arm motors based on rc
        fcs->updateMotorArming();

        // Set motor speeds based on PID
        fcs->updateMotorSpeed();

        vTaskDelayUntil(&last, rate);
    }
}

void FCS::updateMotorSpeed() {
    // Mixer
    float tlThrust = altCommand + pitchCommand + rollCommand -
                     yawCommand; // Front Left  (Motor 1)
    float trThrust = altCommand + pitchCommand - rollCommand +
                     yawCommand; // Front Right (Motor 0)
    float blThrust = altCommand - pitchCommand + rollCommand +
                     yawCommand; // Back Left   (Motor 2)
    float brThrust = altCommand - pitchCommand - rollCommand -
                     yawCommand; // Back Right  (Motor 3)

    tcs.throttle(0, 0);
    tcs.throttle(1, 0);
    tcs.throttle(2, 0);
    tcs.throttle(3, 0);
    // Actually throttle them here
}

void FCS::updateMotorArming() {
    const bool armStates[] = {
        state.rcArmingState.RCArmedA,
        state.rcArmingState.RCArmedB,
        state.rcArmingState.RCArmedC,
        state.rcArmingState.RCArmedD,
    };

    for (uint8_t i = 0; i < 4; ++i) {
        if (armStates[i]) {
            tcs.arm(i);
        } else {
            tcs.disarm(i);
        }
    }
}

void FCS::updatePID() {
    // Outer PIDs
    float pitchRateSetpoint = outerPitch.calc(state.setpointState.setpointPitch,
                                              state.orientation.pitch);
    float rollRateSetpoint = outerPitch.calc(state.setpointState.setpointRoll,
                                             state.orientation.roll);
    float yawRateSetpoint =
        outerPitch.calc(state.setpointState.setpointYaw, state.orientation.yaw);
    float altRateSetpoint = outerPitch.calc(
        state.setpointState.setpointAltitude, state.orientation.alt);

    // Inner PIDs
    pitchCommand = innerPitch.calc(pitchRateSetpoint, state.speedData.gx);
    rollCommand = innerPitch.calc(rollRateSetpoint, state.speedData.gy);
    yawCommand = innerPitch.calc(yawRateSetpoint, state.speedData.gz);
    altCommand = innerPitch.calc(altRateSetpoint, state.speedData.alt);

    DBG_FMT("Pitch: %f | Roll: %f | Yaw: %f | Alt: %f\n", pitchCommand,
            rollCommand, yawCommand, altCommand);
}

void FCS::updateState() {
    DBG("[FCS] Updating state");
    // Update AHRS states
    state.orientation = ahrs.getOrientation();
    state.speedData = ahrs.getSpeedData();

    // Update TCS states
    state.throttleState = tcs.throttleState;
    state.armState = tcs.armState;

    // Update RC states
    state.setpointState = rc.getSetpointState();
    state.rcArmingState = rc.getArmingState();
}

void FCS::arm() {
    switch (state.flightMode) {
    case FlightMode::FAILSAFE:
        DBG("[FCS] Cannot arm, FAILSAFE");
        return;
    case FlightMode::ARMED:
        return;
    case FlightMode::DISARMED:
        DBG("[FCS] Arming");
        state.flightMode = FlightMode::ARMED;
        tcs.stop();
        return;
    }
}

void FCS::disarm() {
    switch (state.flightMode) {
    case FlightMode::FAILSAFE:
        DBG("[FCS] Cannot disarm, FAILSAFE");
        return;
    case FlightMode::ARMED:
        DBG("[FCS] Disarming");
        state.flightMode = FlightMode::DISARMED;
        tcs.stop();
        return;
    case FlightMode::DISARMED:
        return;
    }
}

void FCS::failsafe() {
    DBG("[FCS] FAILSAFE ACTIVE");
    state.flightMode = FlightMode::FAILSAFE;
    tcs.disarm();
}
