#pragma once
#include "FastAccelStepper.h"

#include "soc/gpio_num.h"
#include <cstdint>
#include <esp_err.h>

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

        driver::ServoDriver ::Config claw_servo_config = {
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

    ElevatorClaw() = default;

    explicit ElevatorClaw(const Config &config);

    esp_err_t init();

    void calibrate();
    void move_to_position(int32_t step);
    void open_claw_tower();
    void open_claw_rock();
    void close_claw();

    bool elevator_done() const;

  private:
    Config _config;
    FastAccelStepper *_stepper = nullptr;
    driver::ServoDriver _claw_servo{_config.claw_servo_config};
};
}
