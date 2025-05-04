#pragma once

#include <Arduino.h>

struct Joystick {
    float x = 0.0F;
    float y = 0.0F;
};

struct SetpointState {
    float setpointPitch = 0.0F;
    float setpointRoll = 0.0F;
    float setpointYaw = 0.0F;
    float setpointAltitude = 0.0F;
};

struct RCArmingState {
    bool RCArmedFCU = false;
    bool RCArmedA = false;
    bool RCArmedB = false;
    bool RCArmedC = false;
    bool RCArmedD = false;
};

struct PIDTuningState {
    String axis = "NONE";
    float innerP = 0.0F;
    float innerI = 0.0F;
    float innerD = 0.0F;
    float outerP = 0.0F;
    float outerI = 0.0F;
    float outerD = 0.0F;
};
