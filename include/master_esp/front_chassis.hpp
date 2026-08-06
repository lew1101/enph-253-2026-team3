#pragma once

#include "soc/gpio_num.h"
#include "actuators/elevator_claw.hpp"
#include "actuators/worm_spear.hpp"

#include "FastAccelStepperEngine.h"

extern control::ElevatorClaw elevator_claw;
extern control::WormSpear worm_spear;
extern FastAccelStepperEngine stepper_engine;

constexpr gpio_num_t SPEAR_SERVO_PIN = GPIO_NUM_7;
constexpr gpio_num_t CLAW_SERVO_PIN = GPIO_NUM_6;

constexpr gpio_num_t WORM_STEP_PIN = GPIO_NUM_15;
constexpr gpio_num_t WORM_DIR_PIN = GPIO_NUM_16;
constexpr gpio_num_t WORM_LIMIT_SWITCH_PIN = GPIO_NUM_10;
constexpr gpio_num_t WORM_EN_PIN = GPIO_NUM_4;

constexpr gpio_num_t ELEVATOR_STEP_PIN = GPIO_NUM_42;
constexpr gpio_num_t ELEVATOR_DIR_PIN = GPIO_NUM_41;
constexpr gpio_num_t ELEVATOR_LIMIT_SWITCH_PIN = GPIO_NUM_8;
constexpr gpio_num_t ELEVATOR_EN_PIN = GPIO_NUM_2;

constexpr control::WormSpear::Config worm_config{
    .engine = &stepper_engine,

    .worm_step_pin = WORM_STEP_PIN,
    .worm_dir_pin = WORM_DIR_PIN,
    .worm_calibration_switch_pin = WORM_LIMIT_SWITCH_PIN,
    .worm_en_pin = WORM_EN_PIN,

    .speed_hz = 18'000,
    .acceleration_hz_per_s = 6'000,
    .calibration_max_delay = pdMS_TO_TICKS(10000),
    .reversed = true,

    .spear_servo_config{
        .gpio = SPEAR_SERVO_PIN,

        .channel = 1,
        .freq_hz = 50,

        .duty_res_bits = 14,
        .min_pulse_us = 500,
        .max_pulse_us = 2500,
        .min_pulse_deg = 0.0f,
        .max_pulse_deg = 180.0f,
        .min_clamp_deg = 0.0f,
        .max_clamp_deg = 180.0f,
        .bias_deg = 150.0f,
        .reversed = true,
    },
};

constexpr control::ElevatorClaw::Config elevator_config{
    .engine = &stepper_engine,

    .elevator_step_pin = ELEVATOR_STEP_PIN,
    .elevator_dir_pin = ELEVATOR_DIR_PIN,
    .elevator_calibration_switch_pin = ELEVATOR_LIMIT_SWITCH_PIN,
    .elevator_en_pin = ELEVATOR_EN_PIN,

    .speed_hz = 2250,
    .acceleration_hz_per_s = 200,
    .reversed = true,

    .claw_servo_config{
        .gpio = CLAW_SERVO_PIN, // Specify your pin here

        .channel = 2,
        .freq_hz = 50,
        .duty_res_bits = 14,

        .min_pulse_us = 500,
        .max_pulse_us = 2400,
        .min_pulse_deg = 0.0f,
        .max_pulse_deg = 180.0f,
        .min_clamp_deg = 0.0f,
        .max_clamp_deg = 180.0f,
    },
};

esp_err_t front_chassis_init();
