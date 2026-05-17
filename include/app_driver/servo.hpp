#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

#include <cstdint>

class ServoDriver {
  public:
    struct Config {
        gpio_num_t gpio;
        ledc_timer_t timer = LEDC_TIMER_0;
        ledc_channel_t channel = LEDC_CHANNEL_0;
        ledc_mode_t speed_mode = LEDC_LOW_SPEED_MODE;

        uint32_t freq_hz = 50;
        ledc_timer_bit_t duty_res = LEDC_TIMER_16_BIT;

        uint32_t min_pulse_us = 1000;
        uint32_t max_pulse_us = 2000;

        float min_pulse_deg = -90.0f; // maps to min_pulse_us
        float max_pulse_deg = 90.0f;  // maps to max_pulse_us

        // Software command limits. Keep these inside min_pulse_deg..max_pulse_deg.
        float min_clamp_deg = -90.0f;
        float max_clamp_deg = 90.0f;
        float bias_deg = 0.0f;

        bool reversed = false;

        bool configure_timer = true;
    };

    explicit ServoDriver(const Config &config);

    esp_err_t init();

    esp_err_t set_deg(float deg);
    esp_err_t set_us(uint32_t us);

    esp_err_t center();
    esp_err_t stop();

  private:
    Config _config;
    bool _initialized = false;

    uint32_t deg_to_us(float deg) const;
    uint32_t us_to_duty(uint32_t us) const;
    uint32_t duty_levels() const;
    uint32_t max_duty() const;
};
