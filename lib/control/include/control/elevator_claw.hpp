#pragma once
#include "FastAccelStepper.h"

#include "soc/gpio_num.h"
#include <cstdint>
#include <esp_err.h>

#include "actuators/limit.hpp"
#include "actuators/servo.hpp"

namespace control {

class ElevatorClaw {
  public:
    struct Config {
        FastAccelStepperEngine *engine = nullptr;
        gpio_num_t elevator_step_pin = GPIO_NUM_NC;
        gpio_num_t elevator_dir_pin = GPIO_NUM_NC;
        gpio_num_t elevator_calibration_switch_pin = GPIO_NUM_NC;
        int32_t speed_hz = 500;
        int32_t acceleration_hz_per_s = 100;
        bool reversed = false;
        TickType_t calibration_max_delay = pdMS_TO_TICKS(5000);

        driver::ServoDriver ::Config claw_servo_config = {.gpio = GPIO_NUM_NC,
                                                          .channel = 1,
                                                          .freq_hz = 50,
                                                          .duty_res_bits = 14,
                                                          .min_pulse_us = 500,
                                                          .max_pulse_us = 2400,
                                                          .min_pulse_deg = 0.0f,
                                                          .max_pulse_deg = 180.0f,
                                                          .min_clamp_deg = 0.0f,
                                                          .max_clamp_deg = 180.0f};
    };

    explicit ElevatorClaw(const Config &config);

    esp_err_t init();

    void calibrate();

    inline void move_to_position(int32_t step)
    {
        if (_limit_switch.is_pressed() && step < 0) return;
        _stepper->moveTo(_config.reversed ? -step : step);
    }

    inline void set_claw(float deg) { _claw_servo.set_deg(deg); }

    inline void stop_elevator() { _stepper->forceStop(); }

    inline bool elevator_done() const { return !_stepper->isRunning(); }

    // ONLY FOR TUNING PURPOSES
    void set_speed(int32_t speed_hz, unsigned int acceleration_hz_per_s);
    void set_home_position();
    int32_t get_current_position();
    inline bool limit_is_pressed() const { return _limit_switch.is_pressed(); }

  private:
    Config _config;

    FastAccelStepper *_stepper = nullptr;
    driver::ServoDriver _claw_servo;

    DebouncedLimitSwitch _limit_switch;
};
} // namespace control
