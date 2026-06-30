#include <Arduino.h>

#include "actuators/dc_driver.hpp"

#include <algorithm>

namespace driver {
using std::clamp;
// --- Constructor ---

DCDriver::DCDriver(const Config &config)
    : _config(config)
{
}

// --- Public API ---

bool DCDriver::init()
{
    if (_config.clockwise_pwm_output == GPIO_NUM_NC || _config.c_clockwise_pwm_output == GPIO_NUM_NC) return false;
    if (_config.timer == nullptr) return false;

    // Set up MCPWM
    mcpwm_operator_config_t operator_config = {};
    operator_config.group_id = _config.group_id;
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &motor_operator));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(motor_operator, _config.timer));

    mcpwm_comparator_config_t comparator_config = {};
    comparator_config.flags.update_cmp_on_tez = true;
    ESP_ERROR_CHECK(mcpwm_new_comparator(motor_operator, &comparator_config, &motor_comparator));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motor_comparator, 0));

    mcpwm_generator_config_t generator_a_config = {};
    generator_a_config.gen_gpio_num = _config.clockwise_pwm_output;
    ESP_ERROR_CHECK(mcpwm_new_generator(motor_operator, &generator_a_config, &motor_generator_a));
    mcpwm_generator_config_t generator_b_config = {};
    generator_b_config.gen_gpio_num = _config.c_clockwise_pwm_output;
    ESP_ERROR_CHECK(mcpwm_new_generator(motor_operator, &generator_b_config, &motor_generator_b));

    // Event timer
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(motor_generator_a, 
    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(motor_generator_a, 
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor_comparator, MCPWM_GEN_ACTION_LOW), 
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, motor_comparator, MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END()));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(motor_generator_b, 
    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(motor_generator_b, 
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor_comparator, MCPWM_GEN_ACTION_LOW), 
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, motor_comparator, MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END()));

    stop();
    return true;
}

bool DCDriver::set_speed(float percentage) // 0% to 100%
{
    percentage = clamp(percentage, 0.0f, 100.0f);
    uint32_t compare_value = percentage / 100.0f * _config.period_ticks;

    return (mcpwm_comparator_set_compare_value(motor_comparator, compare_value) == ESP_OK);
}

bool DCDriver::turn_clockwise()
{
    if (_clockwise) return true; // already turning clockwise

    else if (_clockwise == false) {
        // If currently turning counterclockwise, stop first and wait for motor momentum to dissipate
        stop();
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // wait for 100ms
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(motor_generator_a, -1, true)); // -1 means no forcing
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(motor_generator_b, 0, true));
    _clockwise = true;
    return true;
}

bool DCDriver::turn_c_clockwise()
{
    if (!_clockwise) return true; // already turning counterclockwise

    else if (_clockwise == true) {
        // If currently turning clockwise, stop first and wait for motor momentum to dissipate
        stop();
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // wait for 100ms
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(motor_generator_a, 0, true));
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(motor_generator_b, -1, true));
    _clockwise = false;
    return true;
}

bool DCDriver::stop()
{
    // Implementation for stopping
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(motor_generator_a, 0, true));
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(motor_generator_b, 0, true));
    return true;
}
}