#pragma once
#include <Arduino.h>
#include "FastAccelStepper.h"
#include "actuators/limit.hpp"
#include "actuators/servo.hpp"

namespace control {

class WormSpear {
  public:

    struct Config {
        FastAccelStepperEngine *engine = nullptr;
        gpio_num_t worm_step_pin = GPIO_NUM_NC;
        gpio_num_t worm_dir_pin = GPIO_NUM_NC;
        gpio_num_t worm_calibration_switch_pin = GPIO_NUM_NC;
        int32_t speed_hz = 500;
        int32_t acceleration_hz_per_s = 100;
        TickType_t calibration_max_delay = pdMS_TO_TICKS(5000);
        int direction = 1; //1 for forward, -1 for backward

        driver::ServoDriver ::Config spear_servo_config = {
            .gpio = GPIO_NUM_NC,
            .channel = 1,
            .freq_hz = 50,
            .duty_res_bits = 14,
            .min_pulse_us = 500,
            .max_pulse_us = 2400,
            .min_pulse_deg = 0.0f,
            .max_pulse_deg = 180.0f,
            .min_clamp_deg = 0.0f,
            .max_clamp_deg = 180.0f
        };
;
    };

    explicit WormSpear(const Config &config);

    esp_err_t init();

    void calibrate();
    void move_to_position(float step);
    void spear_angle(float deg);

    inline void stop_worm() { _stepper->forceStop(); }

    bool worm_done() const;

    // ONLY FOR TUNING PURPOSES
    void set_speed(int32_t speed_hz, unsigned int acceleration_hz_per_s);
    void set_home_position();
    int32_t get_current_position();

  private:
    Config _config;
    FastAccelStepper *_stepper = nullptr;
    driver::ServoDriver _spear_servo;

    DebouncedLimitSwitch _limit_switch;
};
}
