#include <cstdint>

enum class FlightMode : uint8_t { DISARMED, ARMED, FAILSAFE };

class DroneState {
  public:
    FlightMode mode = FlightMode::DISARMED;

    bool isArmed() const { return mode == FlightMode::ARMED; }
    bool isFailsafe() const { return mode == FlightMode::FAILSAFE; }
    bool shouldFly() const { return isArmed() && !isFailsafe(); }

    void arm();
    void disarm();
    void enterFailsafe();
};
