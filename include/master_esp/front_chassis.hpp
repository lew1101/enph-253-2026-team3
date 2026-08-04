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

constexpr gpio_num_t ELEVATOR_STEP_PIN = GPIO_NUM_42;
constexpr gpio_num_t ELEVATOR_DIR_PIN = GPIO_NUM_41;
constexpr gpio_num_t ELEVATOR_LIMIT_SWITCH_PIN = GPIO_NUM_8;

constexpr driver::ServoDriver::Config SPEAR_SERVO_CFG{
    .gpio = SPEAR_SERVO_PIN, // Specify your pin here
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
};

constexpr driver::ServoDriver::Config CLAW_SERVO_CFG{
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
};

constexpr control::WormSpear::Config worm_config{
    .engine = &stepper_engine, // Set this to your FastAccelStepperEngine instance
    .worm_step_pin = WORM_STEP_PIN,
    .worm_dir_pin = WORM_DIR_PIN,
    .worm_calibration_switch_pin = WORM_LIMIT_SWITCH_PIN,
    .speed_hz = 9000,
    .acceleration_hz_per_s = 1600,
    .calibration_max_delay = pdMS_TO_TICKS(10000),
    .reversed = true,
    .spear_servo_config = SPEAR_SERVO_CFG,
};

constexpr control::ElevatorClaw::Config elevator_config{
    .engine = &stepper_engine, // Set this to your FastAccelStepperEngine instance
    .elevator_step_pin = ELEVATOR_STEP_PIN,
    .elevator_dir_pin = ELEVATOR_DIR_PIN,
    .elevator_calibration_switch_pin = ELEVATOR_LIMIT_SWITCH_PIN,
    .speed_hz = 6500,
    .acceleration_hz_per_s = 1200,
    .reversed = true,
    .claw_servo_config = CLAW_SERVO_CFG,
};

esp_err_t front_chassis_init();
