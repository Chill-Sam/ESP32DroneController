#include <cstdint>

enum class FlightMode : uint8_t { DISARMED, ARMED, FAILSAFE };

struct ThrottleState {
    float ThrottleA = 0.0F;
    float ThrottleB = 0.0F;
    float ThrottleC = 0.0F;
    float ThrottleD = 0.0F;
};

struct ArmState {
    bool ArmedA = false;
    bool ArmedB = false;
    bool ArmedC = false;
    bool ArmedD = false;
};
