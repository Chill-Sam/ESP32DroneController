#pragma once

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
