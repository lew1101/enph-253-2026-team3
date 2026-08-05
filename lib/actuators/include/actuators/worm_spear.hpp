#pragma once
#include <Arduino.h>
#include "FastAccelStepper.h"
#include "FastAccelStepperEngine.h"
#include "drivers/limit.hpp"
#include "drivers/servo.hpp"

namespace control {

class WormSpear {
    static constexpr uint32_t DEFAULT_SPEAR_SWEEP_TIME = 600.0f;

  public:
    struct Config {
        FastAccelStepperEngine *engine = nullptr;

        gpio_num_t worm_step_pin = GPIO_NUM_NC;
        gpio_num_t worm_dir_pin = GPIO_NUM_NC;
        gpio_num_t worm_calibration_switch_pin = GPIO_NUM_NC;
        gpio_num_t worm_en_pin = GPIO_NUM_NC;

        int32_t speed_hz = 500;
        int32_t acceleration_hz_per_s = 100;

        TickType_t calibration_max_delay = pdMS_TO_TICKS(5000);
        // FasDriver driver_type = DRIVER_MCPWM_PCNT;
        FasDriver driver_type = DRIVER_RMT;
        bool reversed = false;

        driver::ServoDriver::Config spear_servo_config = {
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
    esp_err_t calibrate();

    inline void move_to_position(int32_t step, bool blocking = true)
    {
        if (_worm_limit_switch.is_pressed() && step < 0) return;
        _stepper->moveTo(_config.reversed ? -step : step, blocking);
    }

    inline void set_spear(float deg, uint32_t time = DEFAULT_SPEAR_SWEEP_TIME)
    {
        _spear_servo.sweep_to_deg(deg, time);
    };
    inline void stop_worm() { _stepper->forceStop(); }
    inline bool enable()
    {
        const bool servo_enabled = _spear_servo.attach();
        const bool stepper_enabled = _stepper->enableOutputs();
        return servo_enabled && stepper_enabled;
    }
    inline bool disable()
    {
        _stepper->forceStop();
        vTaskDelay(pdMS_TO_TICKS(10));
        const bool servo_disabled = _spear_servo.detach();
        const bool stepper_disabled = _stepper->disableOutputs();
        return servo_disabled && stepper_disabled;
    }
    inline bool worm_done() const { return !_stepper->isRunning(); }

    // ONLY FOR TUNING PURPOSES
    void set_speed(int32_t speed_hz, unsigned int acceleration_hz_per_s);
    void set_home_position();
    int32_t get_current_position();
    inline bool limit_is_pressed() const { return _worm_limit_switch.is_pressed(); }

  private:
    Config _config;
    FastAccelStepper *_stepper = nullptr;
    driver::ServoDriver _spear_servo;

    driver::DebouncedLimitSwitch _worm_limit_switch;
};
} // namespace control
