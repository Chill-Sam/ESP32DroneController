#pragma once

#include <cstdint>

#include "driver/ledc.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "hal/gpio_types.h"
#include "hal/ledc_types.h"

/**
 * Zero-overhead, compile-time-configurable ESC driver using LEDC.
 *
 * @tparam PWM_PIN       GPIO pin for the PWM output
 * @tparam CHANNEL       LEDC channel (0-7)
 * @tparam TIMER         LEDC timer number
 * @tparam FREQ_HZ       PWM frequency (1-40000 Hz)
 * @tparam RES_BITS      Resolution in bits (1-16)
 * @tparam MIN_PULSE_US  Minimum pulse width in microseconds (e.g. 1000)
 * @tparam MAX_PULSE_US  Maximum pulse width in microseconds (e.g. 2000)
 */
template <gpio_num_t PWM_PIN, ledc_channel_t CHANNEL,
          ledc_timer_t TIMER = LEDC_TIMER_0, uint16_t MIN_PULSE_US = 1000,
          uint16_t MAX_PULSE_US = 2000, uint32_t FREQ_HZ = 50,
          uint8_t RES_BITS = 16>
class ESCDriver {
    /// Number of discrete duty steps (2^RES_BITS - 1)
    static constexpr uint32_t maxDuty = (1UL << RES_BITS) - 1;
    static constexpr uint32_t pulseRange = MAX_PULSE_US - MIN_PULSE_US;
    static constexpr uint32_t offset = (MIN_PULSE_US * maxDuty / pulseRange);
    static uint32_t lastDuty;

    static_assert(FREQ_HZ >= 1 && FREQ_HZ <= 40000,
                  "FREQ_HZ out of range (1-40000)");
    static_assert(RES_BITS >= 1 && RES_BITS <= 16,
                  "RES_BITS out of range (1-16)");
    static_assert(MIN_PULSE_US < MAX_PULSE_US,
                  "MIN_PULSE_US must be less than MAX_PULSE_US");
    static_assert(maxDuty <= (UINT32_MAX / pulseRange),
                  "maxDuty*(MAX–MIN) must fit in 32 bits");

  public:
    /**
     * @brief  Configure LEDC timer & channel
     * @return true on success, false on error
     */
    static bool begin() noexcept {
        ledc_timer_config_t timerCfg = {
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .duty_resolution = static_cast<ledc_timer_bit_t>(RES_BITS),
            .timer_num = TIMER,
            .freq_hz = FREQ_HZ,
            .clk_cfg = LEDC_AUTO_CLK};
        if (ledc_timer_config(&timerCfg) != ESP_OK) {
            return false;
        }

        ledc_channel_config_t chanCfg = {.gpio_num = PWM_PIN,
                                         .speed_mode = LEDC_HIGH_SPEED_MODE,
                                         .channel = CHANNEL,
                                         .intr_type = LEDC_INTR_DISABLE,
                                         .timer_sel = TIMER,
                                         .duty = 0,
                                         .hpoint = 0};
        return (ledc_channel_config(&chanCfg) == ESP_OK);
    }

    /**
     * Write an ESC pulse width.
     * @param pulse_us Pulse width in microseconds (clamped to MIN/MAX)
     */
    static void write(uint16_t pulse_us) noexcept IRAM_ATTR {
        const uint32_t duty = pulseToDuty(pulse_us);

        // We only call the ledc functions when necessary to minimize overhead
        // from calling the functions. We exchange 4 bytes of memory for this
        // gain.
        if (duty == lastDuty) {
            return;
        }
        lastDuty = duty;

        ledc_set_duty(LEDC_HIGH_SPEED_MODE, CHANNEL, duty);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, CHANNEL);
    }

    /// Stop the ESC (send minimum pulse)
    static void stop() noexcept { write(MIN_PULSE_US); }

  private:
    /// Map [MIN_PULSE_US .. MAX_PULSE_US] → [0..MAX_DUTY]
    static uint32_t pulseToDuty(uint16_t us) noexcept {
        // We ensure that us is clamped to MIN and MAX pulsewidth for safety.
        if (us < MIN_PULSE_US) {
            us = MIN_PULSE_US;
        } else if (us > MAX_PULSE_US) {
            us = MAX_PULSE_US;
        }

        return ((uint32_t(us) * maxDuty) / pulseRange) - offset;
    }
};

template <gpio_num_t P, ledc_channel_t C, ledc_timer_t T, uint16_t MIN,
          uint16_t MAX, uint32_t F, uint8_t R>
uint32_t ESCDriver<P, C, T, MIN, MAX, F, R>::lastDuty = UINT32_MAX;
