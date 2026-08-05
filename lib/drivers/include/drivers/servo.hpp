#pragma once

#include "esp32-hal-ledc.h"
#include <Arduino.h>

#include <cstdint>

namespace driver {
class ServoDriver {
  public:
    struct Config {
        gpio_num_t gpio = GPIO_NUM_NC;
        uint8_t channel = 0;

        uint32_t freq_hz = 50;
        uint8_t duty_res_bits = 16;

        uint32_t min_pulse_us = 1000;
        uint32_t max_pulse_us = 2000;

        float min_pulse_deg = -90.0f; // maps to min_pulse_us
        float max_pulse_deg = 90.0f;  // maps to max_pulse_us

        // Software command limits. Keep these inside min_pulse_deg..max_pulse_deg.
        float min_clamp_deg = -90.0f;
        float max_clamp_deg = 90.0f;
        float bias_deg = 0.0f;

        bool reversed = false;
    };

    explicit ServoDriver(const Config &config);

    bool init();

    bool set_deg(float deg);
    bool sweep_to_deg(float target_deg, uint32_t duration_ms);
    bool set_us(uint32_t us);
    bool attach();
    bool detach();
    inline bool is_attached() const { return _attached; }

    bool center();

  private:
    Config _config;
    bool _initialized = false;
    bool _attached = false;
    float _current_deg = 0.0f;

    uint32_t deg_to_us(float deg) const;
    uint32_t us_to_duty(uint32_t us) const;
    inline uint32_t duty_levels() const { return 1UL << _config.duty_res_bits; }
    inline uint32_t max_duty() const { return duty_levels() - 1; }
};
} // namespace driver
