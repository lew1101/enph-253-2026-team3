#include <Arduino.h>
#include "control/drivetrain.hpp"
#include "esp_err.h"

static constexpr char TAG[] = "drivetrain";

using control::Drivetrain;

Drivetrain::Drivetrain(const Config &config)
    : _config(config)
{
}

esp_err_t Drivetrain::init()
{
    // Initialize timer 0 and timer 1
    mcpwm_timer_config_t timer_0_config = {};
    timer_0_config.group_id = 0;
    timer_0_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_0_config.resolution_hz = timer_resolution_hz;
    timer_0_config.period_ticks = period_tick;
    timer_0_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN;

    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_0_config, &_config.timer_0));
    ESP_LOGD(TAG, "Timer 0 initialized.");

    mcpwm_timer_config_t timer_1_config = {};
    timer_1_config.group_id = 1;
    timer_1_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_1_config.resolution_hz = timer_resolution_hz;
    timer_1_config.period_ticks = period_tick;
    timer_1_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN;

    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_1_config, &_config.timer_1));
    ESP_LOGD(TAG, "Timer 1 initialized.");

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

    ESP_ERROR_CHECK(front_right_motor.init());
    ESP_LOGI(TAG, "Front right motor initialized");

    ESP_ERROR_CHECK(front_left_motor.init());
    ESP_LOGI(TAG, "Front left motor initialized");

    ESP_ERROR_CHECK(back_right_motor.init());
    ESP_LOGI(TAG, "Back right motor initialized");

    ESP_ERROR_CHECK(back_left_motor.init());
    ESP_LOGI(TAG, "Back left motor initialized");

    ESP_ERROR_CHECK(mcpwm_timer_enable(_config.timer_0));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(_config.timer_0, MCPWM_TIMER_START_NO_STOP));
    ESP_ERROR_CHECK(mcpwm_timer_enable(_config.timer_1));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(_config.timer_1, MCPWM_TIMER_START_NO_STOP));

    ESP_LOGI(TAG, "Drivetrain ok.");

    return ESP_OK;
}

bool Drivetrain::forward(float speed_percentage)
{
    target_speed[0] = speed_percentage;
    target_speed[1] = speed_percentage;
    target_speed[2] = -speed_percentage;
    target_speed[3] = -speed_percentage;

    return true;
}

bool Drivetrain::strafe(float speed_percentage) // right is positive, left is negative
{
    target_speed[0] = speed_percentage;
    target_speed[1] = -speed_percentage;
    target_speed[2] = speed_percentage;
    target_speed[3] = -speed_percentage;

    return true;
}

// positive is clockwise, negative is counter-clockwise
bool Drivetrain::turn(float speed_percentage)
{
    target_speed[0] = -speed_percentage * _config.rot_scalar;
    target_speed[1] = -speed_percentage * _config.rot_scalar;
    target_speed[2] = -speed_percentage * _config.rot_scalar;
    target_speed[3] = -speed_percentage * _config.rot_scalar;

    return true;
}

bool Drivetrain::move_vector(float vx, float vy, float omega)
{
    target_speed[0] = vy - vx + omega * _config.rot_scalar;
    target_speed[1] = vy - vx + omega * _config.rot_scalar;
    target_speed[2] = -(vy - vx + omega * _config.rot_scalar);
    target_speed[3] = -(vy - vx + omega * _config.rot_scalar);

    float max_mag = std::max({std::fabs(target_speed[0]),
                              std::fabs(target_speed[1]),
                              std::fabs(target_speed[2]),
                              std::fabs(target_speed[3])});

    if (max_mag > 1.0f) {
        target_speed[0] /= max_mag;
        target_speed[1] /= max_mag;
        target_speed[2] /= max_mag;
        target_speed[3] /= max_mag;
    }

    return true;
}

bool Drivetrain::stop()
{
    target_speed[0] = 0.0;
    target_speed[1] = 0.0;
    target_speed[2] = 0.0;
    target_speed[3] = 0.0;

    stopped = true;
    return true;
}

bool Drivetrain::update()
{
    uint32_t current_time = millis();
    float time_elapsed = current_time - previous_time;
    previous_time = current_time;

    for (int i = 0; i < 4; i++) {
        if (current_speed[i] < target_speed[i]) {
            current_speed[i] += _config.acceleration_rate * time_elapsed;
            if (current_speed[i] > target_speed[i]) {
                current_speed[i] = target_speed[i];
            }
        } else if (current_speed[i] > target_speed[i]) {
            current_speed[i] -= _config.acceleration_rate * time_elapsed;
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
