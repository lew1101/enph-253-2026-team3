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
    target_speed[0] = speed_percentage;
    target_speed[1] = speed_percentage;
    target_speed[2] = -speed_percentage;
    target_speed[3] = -speed_percentage;

    return true;
}

bool control::Drivetrain::strafe(float speed_percentage) // right is positive, left is negative
{
    target_speed[0] = speed_percentage;
    target_speed[1] = -speed_percentage;
    target_speed[2] = speed_percentage;
    target_speed[3] = -speed_percentage;

    return true;
}

bool control::Drivetrain::turn(float speed_percentage) // positive is clockwise, negative is counter-clockwise
{
    target_speed[0] = -speed_percentage;
    target_speed[1] = -speed_percentage;
    target_speed[2] = -speed_percentage;
    target_speed[3] = -speed_percentage;

    return true;
}

bool control::Drivetrain::stop()
{
    target_speed[0] = 0.0;
    target_speed[1] = 0.0;
    target_speed[2] = 0.0;
    target_speed[3] = 0.0;

    stopped = true;
    return true;
}

bool control::Drivetrain::update()
{
    uint32_t current_time = millis();
    float time_elapsed = current_time - previous_time;
    previous_time = current_time;

    for (int i = 0; i < 4; i++) {
        if (current_speed[i] < target_speed[i]) {
            current_speed[i] += acceleration_rate * time_elapsed;
            if (current_speed[i] > target_speed[i]) {
                current_speed[i] = target_speed[i];
            }
        } else if (current_speed[i] > target_speed[i]) {
            current_speed[i] -= acceleration_rate * time_elapsed;
            if (current_speed[i] < target_speed[i]) {
                current_speed[i] = target_speed[i];
            }
        }
    }

    if (current_speed[0] > 0.0) {
        front_right_motor.turn_clockwise();
    } else if (current_speed[0] < 0.0) {
        front_right_motor.turn_c_clockwise();
    } else {
        front_right_motor.stop();
    }
    front_right_motor.set_speed(std::abs(current_speed[0]));

    if (current_speed[1] > 0.0) {
        back_right_motor.turn_clockwise();
    } else if (current_speed[1] < 0.0) {
        back_right_motor.turn_c_clockwise();
    } else {
        back_right_motor.stop();
    }
    back_right_motor.set_speed(std::abs(current_speed[1]));

    if (current_speed[2] > 0.0) {
        front_left_motor.turn_clockwise();
    } else if (current_speed[2] < 0.0) {
        front_left_motor.turn_c_clockwise();
    } else {
        front_left_motor.stop();
    }
    front_left_motor.set_speed(std::abs(current_speed[2]));

    if (current_speed[3] > 0.0) {
        back_left_motor.turn_clockwise();
    } else if (current_speed[3] < 0.0) {
        back_left_motor.turn_c_clockwise();
    } else {
        back_left_motor.stop();
    }
    back_left_motor.set_speed(std::abs(current_speed[3]));
    
    return true;
}