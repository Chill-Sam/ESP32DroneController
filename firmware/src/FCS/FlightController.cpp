#include "FlightController.h"
#include "types/ControlData.h"
#include "types/FlightData.h"
#include "utils/log.h"

#define BASE_SPEED 20

FCS::FCS(int pinA, int pinB, int pinC, int pinD, int minPulseWidth,
         int maxPulsewidth)
    : tcs(pinA, pinB, pinC, pinD, minPulseWidth, maxPulsewidth) {}

void FCS::begin() {
    ahrs.begin();
    tcs.begin();

    rc.begin();

    rc.onDisconnect = [this]() { this->failsafe(); };
    rc.onTestRequest = [this]() {
        this->testing = true;
        this->tcs.test();
        this->testing = false;
    };
    rc.onAuthenticateSuccess = [this]() { this->startLoop(); };
    rc.onPIDTune = [this](const PIDTuningState &tuning) {
        this->tunePID(tuning);
    };
}

void FCS::startLoop() {
    // Create main control loop task on core 1 with medium priority
    xTaskCreatePinnedToCore(control, "FCU Control", 4096, this, 2, nullptr, 1);

    // Create telemetry update task on core 0 with low priority
    xTaskCreatePinnedToCore(telemetry, "Update Telemetry", 4096, this, 0,
                            nullptr, 0);
}

void FCS::telemetry(void *pvParameters) {
    FCS *fcs = static_cast<FCS *>(pvParameters);

    const TickType_t rate = pdMS_TO_TICKS(100); // 10 Hz
    TickType_t last = xTaskGetTickCount();

    while (true) {
        fcs->rc.updateTelemetry(fcs->state);
        vTaskDelayUntil(&last, rate);
    }
}

void FCS::control(void *pvParameters) {
    FCS *fcs = static_cast<FCS *>(pvParameters);

#ifdef DEBUG
#define FCS_LOOP_PERIOD_MS 100
#else
#define FCS_LOOP_PERIOD_MS 2
#endif // !DEBUG

    const TickType_t rate = pdMS_TO_TICKS(FCS_LOOP_PERIOD_MS); // 500 Hz
    TickType_t last = xTaskGetTickCount();

    while (true) {
        fcs->updateState();

        // Prevent further action if drone is in FAILSAFE
        if (fcs->state.flightMode == FlightMode::FAILSAFE) {
            DBG("[FCS] FAILSAFE\n");
            vTaskDelayUntil(&last, rate);
            continue;
        }

        // We update PIDs before ARM check to keep the PIDs consistent.
        fcs->updatePID();

        if (fcs->testing) {
            vTaskDelayUntil(&last, rate);
            continue;
        }

        // Arm/Disarm FCS first before DISARM/ARM check
        if (fcs->state.rcArmingState.RCArmedFCU) {
            fcs->arm();
        } else {
            fcs->disarm();
        }

        if (fcs->state.flightMode == FlightMode::DISARMED) {
            DBG("[FCS] Currently disarmed\n");
            fcs->tcs.disarm();
            fcs->tcs.stop();
            vTaskDelayUntil(&last, rate);
            continue;
        } // Drone should be armed if it passes this

        if (fcs->state.flightMode != FlightMode::ARMED) {
            DBGCRT("[FCS] PANIC: unreachable\n");
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
    float tlThrust = BASE_SPEED + altCommand + pitchCommand + rollCommand -
                     yawCommand; // Front Left  (Motor 1)
    float trThrust = BASE_SPEED + altCommand + pitchCommand - rollCommand +
                     yawCommand; // Front Right (Motor 0)
    float blThrust = BASE_SPEED + altCommand - pitchCommand + rollCommand +
                     yawCommand; // Back Left   (Motor 2)
    float brThrust = BASE_SPEED + altCommand - pitchCommand - rollCommand -
                     yawCommand; // Back Right  (Motor 3)

    tcs.throttle(0, blThrust);
    tcs.throttle(1, trThrust);
    tcs.throttle(2, blThrust);
    tcs.throttle(3, tlThrust);
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
    float rollRateSetpoint = outerRoll.calc(state.setpointState.setpointRoll,
                                            state.orientation.roll);
    float yawRateSetpoint =
        outerYaw.calc(state.setpointState.setpointYaw, state.orientation.yaw);
    float altRateSetpoint = outerAlt.calc(state.setpointState.setpointAltitude,
                                          state.orientation.alt);

    // Inner PIDs
    pitchCommand = innerPitch.calc(pitchRateSetpoint, state.speedData.gx);
    rollCommand = innerRoll.calc(rollRateSetpoint, state.speedData.gy);
    yawCommand = innerYaw.calc(yawRateSetpoint, state.speedData.gz);
    altCommand = innerAlt.calc(altRateSetpoint, state.speedData.alt);

    DBG_FMT("[FCS] Pitch: %f | Roll: %f | Yaw: %f | Alt: %f\n", pitchCommand,
            rollCommand, yawCommand, altCommand);
}

void FCS::tunePID(const PIDTuningState &tuning) {
    String axis = tuning.axis;
    if (axis == "Pitch") {
        DBGCRT("[FCS] Tuning pitch\n");
        innerPitch.tune(tuning.innerP, tuning.innerI, tuning.innerD);
        outerPitch.tune(tuning.outerP, tuning.outerI, tuning.outerD);
    }
    if (axis == "Roll") {
        DBGCRT("[FCS] Tuning roll\n");
        innerRoll.tune(tuning.innerP, tuning.innerI, tuning.innerD);
        outerRoll.tune(tuning.outerP, tuning.outerI, tuning.outerD);
    }
    if (axis == "Yaw") {
        DBGCRT("[FCS] Tuning yaw\n");
        innerYaw.tune(tuning.innerP, tuning.innerI, tuning.innerD);
        outerYaw.tune(tuning.outerP, tuning.outerI, tuning.outerD);
    }
    if (axis == "Alt") {
        DBGCRT("[FCS] Tuning alt\n");
        innerAlt.tune(tuning.innerP, tuning.innerI, tuning.innerD);
        outerAlt.tune(tuning.outerP, tuning.outerI, tuning.outerD);
    }
}

void FCS::updateState() {
    DBG("[FCS] Updating state\n");
    // Update AHRS states
    state.orientation = ahrs.orientation;
    state.speedData = ahrs.speedData;

    // Update TCS states
    state.throttleState = tcs.throttleState;
    state.armState = tcs.armState;

    // Update RC states
    state.setpointState = rc.setpointState;
    state.rcArmingState = rc.armingState;
}

void FCS::arm() {
    switch (state.flightMode) {
    case FlightMode::FAILSAFE:
        DBGCRT("[FCS] Cannot arm, FAILSAFE\n");
        return;
    case FlightMode::ARMED:
        return;
    case FlightMode::DISARMED:
        DBGCRT("[FCS] Arming\n");
        state.flightMode = FlightMode::ARMED;
        tcs.stop();
        return;
    }
}

void FCS::disarm() {
    switch (state.flightMode) {
    case FlightMode::FAILSAFE:
        DBGCRT("[FCS] Cannot disarm, FAILSAFE\n");
        return;
    case FlightMode::ARMED:
        DBGCRT("[FCS] Disarming\n");
        state.flightMode = FlightMode::DISARMED;
        tcs.stop();
        return;
    case FlightMode::DISARMED:
        return;
    }
}

void FCS::failsafe() {
    DBGCRT("[FCS] FAILSAFE ACTIVE\n");
    state.flightMode = FlightMode::FAILSAFE;
    tcs.disarm();
}
