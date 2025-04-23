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

struct Orientation {
    float pitch = 0.0F;
    float roll = 0.0F;
    float yaw = 0.0F;
    float alt = 0.0F;
};
