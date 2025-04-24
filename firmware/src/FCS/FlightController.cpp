#include "FlightController.h"
#include "utils/log.h"

FCS::FCS(int pinA, int pinB, int pinC, int pinD, int minPulseWidth,
         int maxPulsewidth)
    : tcs(pinA, pinB, pinC, pinD, minPulseWidth, maxPulsewidth) {}

void FCS::begin() {
    ahrs.begin();
    tcs.begin();
    rc.begin();

    // Create main control loop task on core 1 with medium priority
    xTaskCreatePinnedToCore(control, "FCU Control", 4092, this, 2, nullptr, 1);
}

void FCS::control(void *pvParameters) {
    FCS *fcs = static_cast<FCS *>(pvParameters);

    const TickType_t rate = pdMS_TO_TICKS(2); // 500 Hz
    TickType_t last = xTaskGetTickCount();

    while (true) {
        if (!fcs->rc.isAuthenticated) {
            DBG("[FCS] Waiting for RC authentication");
        }
    }
}
