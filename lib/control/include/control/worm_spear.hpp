#pragma once
#include <Arduino.h>
#include "FastAccelStepper.h"
#include "actuators/limit.hpp"
#include "actuators/servo.hpp"

namespace control {

class WormSpear {
    static constexpr uint32_t DEFAULT_SPEAR_SWEEP_TIME = 600.0f;

  public:
    struct Config {
        FastAccelStepperEngine *engine = nullptr;

        gpio_num_t worm_step_pin = GPIO_NUM_NC;
        gpio_num_t worm_dir_pin = GPIO_NUM_NC;
        gpio_num_t worm_calibration_switch_pin = GPIO_NUM_NC;
        gpio_num_t crescent_moon_limit_switch_pin = GPIO_NUM_NC;
        int32_t speed_hz = 500;
        int32_t acceleration_hz_per_s = 100;

        TickType_t calibration_max_delay = pdMS_TO_TICKS(5000);
        bool reversed = false;

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
            .max_clamp_deg = 180.0f,
            .bias_deg = -145.0f,

        };
    };

    explicit WormSpear(const Config &config);

    esp_err_t init();

    void calibrate();

    inline void move_to_position(int32_t step, bool blocking = true)
    {
        if (_worm_limit_switch.is_pressed() && step < 0) return;
        _stepper->moveTo(_config.reversed ? -step : step, blocking);
    }

    inline void set_spear(float deg, uint32_t time = DEFAULT_SPEAR_SWEEP_TIME) { _spear_servo.sweep_to_deg(deg, time); };

    inline void stop_worm() { _stepper->forceStop(); }

    inline void disable_worm() { _stepper->disableOutputs(); }

    inline void enable_worm() { _stepper->enableOutputs(); }

    inline bool worm_done() const { return !_stepper->isRunning(); }

    inline bool crescent_moon_limit_is_pressed() const { return _crescent_moon_limit_switch.is_pressed(); }

    // ONLY FOR TUNING PURPOSES
    void set_speed(int32_t speed_hz, unsigned int acceleration_hz_per_s);
    void set_home_position();
    int32_t get_current_position();
    inline bool limit_is_pressed() const { return _worm_limit_switch.is_pressed(); }

  private:
    Config _config;
    FastAccelStepper *_stepper = nullptr;
    driver::ServoDriver _spear_servo;

    DebouncedLimitSwitch _worm_limit_switch;
    DebouncedLimitSwitch _crescent_moon_limit_switch;
};
} // namespace control
