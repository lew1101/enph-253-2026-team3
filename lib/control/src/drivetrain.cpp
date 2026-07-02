#include <Arduino.h>
#include "control/drivetrain.hpp"

control::Drivetrain::Drivetrain(const Config &config)
    : _config(config)
{
}

bool control::Drivetrain::init()
{
    // Initialize timer 0 and timer 1
    mcpwm_timer_config_t timer_0_config = {};
    timer_0_config.group_id = 0;
    timer_0_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_0_config.resolution_hz = timer_resolution_hz;
    timer_0_config.period_ticks = period_tick;
    timer_0_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN;
    if (mcpwm_new_timer(&timer_0_config, &_config.timer_0) == ESP_OK) {
        Serial.println("Timer 0 initialized");
    } else {
        Serial.println("Failed to initialize Timer 0");
        return false;
    }
 
    mcpwm_timer_config_t timer_1_config = {};
    timer_1_config.group_id = 1;
    timer_1_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_1_config.resolution_hz = timer_resolution_hz;
    timer_1_config.period_ticks = period_tick;
    timer_1_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN;
    timer_0_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN;
    if (mcpwm_new_timer(&timer_1_config, &_config.timer_1) == ESP_OK) {
        Serial.println("Timer 1 initialized");
    } else {
        Serial.println("Failed to initialize Timer 1");
        return false;
    }

    // Fleshing out the configs for the motors
    _config.front_right_motor_config.group_id = 0;
    _config.front_right_motor_config.timer = _config.timer_0;
    _config.front_right_motor_config.period_ticks = period_tick;
    _config.back_right_motor_config.group_id = 0;
    _config.back_right_motor_config.timer = _config.timer_0;
    _config.back_right_motor_config.period_ticks = period_tick;

    _config.front_left_motor_config.group_id = 1;
    _config.front_left_motor_config.timer = _config.timer_1;
    _config.front_left_motor_config.period_ticks = period_tick;
    _config.back_left_motor_config.group_id = 1;
    _config.back_left_motor_config.timer = _config.timer_1;
    _config.back_left_motor_config.period_ticks = period_tick;

    // Setting up config for each motor
    front_right_motor = driver::DCDriver(_config.front_right_motor_config);
    front_left_motor = driver::DCDriver(_config.front_left_motor_config);
    back_right_motor = driver::DCDriver(_config.back_right_motor_config);
    back_left_motor = driver::DCDriver(_config.back_left_motor_config);

    if (!front_right_motor.init()) return false;
    Serial.println("Front right motor initialized");
    if (!front_left_motor.init()) return false;
    Serial.println("Front left motor initialized");
    if (!back_right_motor.init()) return false;
    Serial.println("Back right motor initialized");
    if (!back_left_motor.init()) return false;
    Serial.println("Back left motor initialized");

    ESP_ERROR_CHECK(mcpwm_timer_enable(_config.timer_0));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(_config.timer_0, MCPWM_TIMER_START_NO_STOP));
    ESP_ERROR_CHECK(mcpwm_timer_enable(_config.timer_1));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(_config.timer_1, MCPWM_TIMER_START_NO_STOP));

    return true;
}

bool control::Drivetrain::forward(float speed_percentage)
{
    if (speed_percentage > 0.0) {
        front_right_motor.turn_c_clockwise();
        back_right_motor.turn_c_clockwise();
        front_left_motor.turn_clockwise();
        back_left_motor.turn_clockwise();
    }

    else {
        front_right_motor.turn_clockwise();
        back_right_motor.turn_clockwise();
        front_left_motor.turn_c_clockwise();
        back_left_motor.turn_c_clockwise();
    }

    speed_percentage = std::abs(speed_percentage);
    front_right_motor.set_speed(speed_percentage);
    back_right_motor.set_speed(speed_percentage);
    front_left_motor.set_speed(speed_percentage);
    back_left_motor.set_speed(speed_percentage);

    return true;
}

bool control::Drivetrain::strafe(float speed_percentage) // right is positive, left is negative
{
    if (speed_percentage > 0.0) {
        front_right_motor.turn_c_clockwise();
        back_right_motor.turn_clockwise();
        front_left_motor.turn_c_clockwise();
        back_left_motor.turn_clockwise();
    }

    else {
        front_right_motor.turn_clockwise();
        back_right_motor.turn_c_clockwise();
        front_left_motor.turn_clockwise();
        back_left_motor.turn_c_clockwise();
    }

    speed_percentage = std::abs(speed_percentage);
    front_right_motor.set_speed(speed_percentage);
    back_right_motor.set_speed(speed_percentage);
    front_left_motor.set_speed(speed_percentage);
    back_left_motor.set_speed(speed_percentage);

    return true;
}

bool control::Drivetrain::stop()
{
    if (!front_right_motor.stop()) return false;
    if (!front_left_motor.stop()) return false;
    if (!back_right_motor.stop()) return false;
    if (!back_left_motor.stop()) return false;

    return true;
}

bool control::Drivetrain::turn(float speed_percentage)
{
    if (speed_percentage > 0.0) {
        front_right_motor.turn_c_clockwise();
        back_right_motor.turn_c_clockwise();
        front_left_motor.turn_clockwise();
        back_left_motor.turn_clockwise();
    }

    else {
        front_right_motor.turn_clockwise();
        back_right_motor.turn_clockwise();
        front_left_motor.turn_c_clockwise();
        back_left_motor.turn_c_clockwise();
    }

    speed_percentage = std::abs(speed_percentage);
    front_right_motor.set_speed(speed_percentage);
    back_right_motor.set_speed(speed_percentage);
    front_left_motor.set_speed(speed_percentage);
    back_left_motor.set_speed(speed_percentage);

    return true;
}