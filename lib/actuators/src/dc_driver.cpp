#include <algorithm>

#include "actuators/dc_driver.hpp"
#include "esp_check.h"

#include <cmath>
#include <esp32-hal-log.h>

static constexpr char TAG[] = "dc_driver";

namespace driver {
// --- Constructor ---

using std::clamp;
DCDriver::DCDriver(const Config &config)
    : _config(config)
{
}

// --- Public API ---

esp_err_t DCDriver::init()
{
    if (_config.clockwise_pwm_output == GPIO_NUM_NC ||
        _config.c_clockwise_pwm_output == GPIO_NUM_NC)
        return ESP_ERR_INVALID_ARG;

    if (_config.timer == nullptr) return ESP_ERR_INVALID_ARG;

    /*
     * operator
     * dead-time changes are synchronized to timer zero.
     */
    ESP_LOGD(TAG, "Set up MCPWM operator");

    mcpwm_operator_config_t operator_config = {};
    operator_config.group_id = _config.group_id;
    operator_config.flags.update_dead_time_on_tez = true;

    ESP_RETURN_ON_ERROR(mcpwm_new_operator(&operator_config, &motor_operator),
                        TAG,
                        "failed to create MCPWM operator");

    ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(motor_operator, _config.timer),
                        TAG,
                        "failed to connect MCPWM timer");

    /*
     * comparator
     * PWM updates occur at timer zero to avoid malformed partial pulses.
     */
    ESP_LOGD(TAG, "Set up MCPWM comparator");

    mcpwm_comparator_config_t comparator_config = {};
    comparator_config.flags.update_cmp_on_tez = true;

    ESP_RETURN_ON_ERROR(mcpwm_new_comparator(motor_operator, &comparator_config, &motor_comparator),
                        TAG,
                        "failed to create MCPWM comparator");

    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(motor_comparator, 0),
                        TAG,
                        "failed to initialize comparator");

    /*
     * generators
     *
     * generator A is normal, but generator B is inverted at the GPIO output.
     *
     * Its internal waveform will
     * also be the inverse of generator A. After the GPIO inversion, both
     * physical pins produce the same active-high PWM waveform.
     *
     * The reason for this arrangement is that MCPWM only has one rising-edge
     * delay path and one falling-edge delay path per operator:
     *
     *   A uses rising-edge delay.
     *   B uses falling-edge delay on an internally inverted waveform.
     *
     * After GPIO inversion, B's physical rising edge is delayed too.
     */

    ESP_LOGD(TAG, "Set up MCPWM generators");

    mcpwm_generator_config_t generator_a_config = {};
    generator_a_config.gen_gpio_num = _config.clockwise_pwm_output;

    ESP_RETURN_ON_ERROR(
        mcpwm_new_generator(motor_operator, &generator_a_config, &motor_generator_a),
        TAG,
        "failed to create clockwise generator");

    mcpwm_generator_config_t generator_b_config = {};
    generator_b_config.gen_gpio_num = _config.c_clockwise_pwm_output;
    generator_b_config.flags.invert_pwm = true; // INVERT GEN B PWM

    ESP_RETURN_ON_ERROR(
        mcpwm_new_generator(motor_operator, &generator_b_config, &motor_generator_b),
        TAG,
        "failed to create counter-clockwise generator");

    // force both outputs low.
    ESP_RETURN_ON_ERROR(mcpwm_generator_set_force_level(motor_generator_a, 0, true),
                        TAG,
                        "failed to disable generator A");

    ESP_RETURN_ON_ERROR(mcpwm_generator_set_force_level(motor_generator_b, 1, true),
                        TAG,
                        "failed to disable generator B");

    /*
     * Generator A raw waveform:
     *
     *   EMPTY/up:    HIGH
     *   CMP/up:      LOW
     *   CMP/down:    HIGH
     *
     * This creates an active-high, center-aligned PWM waveform.
     */
    ESP_RETURN_ON_ERROR(mcpwm_generator_set_actions_on_timer_event(
                            motor_generator_a,
                            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                         MCPWM_TIMER_EVENT_EMPTY,
                                                         MCPWM_GEN_ACTION_HIGH),
                            MCPWM_GEN_TIMER_EVENT_ACTION_END()),
                        TAG,
                        "failed to configure generator A timer action");

    ESP_RETURN_ON_ERROR(mcpwm_generator_set_actions_on_compare_event(
                            motor_generator_a,
                            MCPWM_GEN_COMPARE_EVENT_ACTION(
                                MCPWM_TIMER_DIRECTION_UP, motor_comparator, MCPWM_GEN_ACTION_LOW),
                            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN,
                                                           motor_comparator,
                                                           MCPWM_GEN_ACTION_HIGH),
                            MCPWM_GEN_COMPARE_EVENT_ACTION_END()),
                        TAG,
                        "failed to configure generator A compare actions");

    /*
     * Generator B raw waveform is the inverse of A:
     *
     *   EMPTY/up:    LOW
     *   CMP/up:      HIGH
     *   CMP/down:    LOW
     *
     * Its GPIO is inverted, so the physical waveform matches generator A.
     */
    ESP_RETURN_ON_ERROR(mcpwm_generator_set_actions_on_timer_event(
                            motor_generator_b,
                            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                         MCPWM_TIMER_EVENT_EMPTY,
                                                         MCPWM_GEN_ACTION_LOW),
                            MCPWM_GEN_TIMER_EVENT_ACTION_END()),
                        TAG,
                        "failed to configure generator B timer action");

    ESP_RETURN_ON_ERROR(mcpwm_generator_set_actions_on_compare_event(
                            motor_generator_b,
                            MCPWM_GEN_COMPARE_EVENT_ACTION(
                                MCPWM_TIMER_DIRECTION_UP, motor_comparator, MCPWM_GEN_ACTION_HIGH),
                            MCPWM_GEN_COMPARE_EVENT_ACTION(
                                MCPWM_TIMER_DIRECTION_DOWN, motor_comparator, MCPWM_GEN_ACTION_LOW),
                            MCPWM_GEN_COMPARE_EVENT_ACTION_END()),
                        TAG,
                        "failed to configure generator B compare actions");

    /*
     * Hardware dead-time
     *
     * A: delay internal rising edge.
     * B: delay internal falling edge.
     *
     * Because B's physical output is inverted, delaying its internal falling
     * edge delays its physical rising edge.
     */
    ESP_LOGD(TAG, "Configure MCPWM dead-time");

    mcpwm_dead_time_config_t dead_time_a = {};
    dead_time_a.posedge_delay_ticks = _config.dead_time_ticks;
    dead_time_a.negedge_delay_ticks = 0;

    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_dead_time(motor_generator_a, motor_generator_a, &dead_time_a),
        TAG,
        "failed to configure generator A dead-time");

    mcpwm_dead_time_config_t dead_time_b = {};
    dead_time_b.posedge_delay_ticks = 0;
    dead_time_b.negedge_delay_ticks = _config.dead_time_ticks;

    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_dead_time(motor_generator_b, motor_generator_b, &dead_time_b),
        TAG,
        "failed to configure generator B dead-time");

    motor_state = MOTOR_STOPPED;

    return ESP_OK;
}

bool DCDriver::set_output(float signed_percentage)
{
    signed_percentage = std::clamp(signed_percentage, -100.0f, 100.0f);

    constexpr float stop_threshold = 0.001f;

    if (std::fabs(signed_percentage) <= stop_threshold) {
        return stop();
    }

    const MotorState requested_direction =
        signed_percentage > 0.0f ? MOTOR_CLOCKWISE : MOTOR_COUNTER_CLOCKWISE;

    const float magnitude = std::fabs(signed_percentage);

    /*
     * When direction changes, force both physical outputs LOW first.
     *
     * Releasing the requested generator then produces a rising edge through
     * the MCPWM dead-time hardware. There is no blocking vTaskDelay().
     */
    if (requested_direction != motor_state) {
        if (!_force_all_off()) {
            return false;
        }

        if (!_set_pwm(magnitude)) {
            return false;
        }

        if (!_set_direction(requested_direction)) {
            return false;
        }

        motor_state = requested_direction;
        return true;
    }

    return _set_pwm(magnitude);
}

bool DCDriver::_set_pwm(float magnitude_percentage)
{
    magnitude_percentage = std::clamp(magnitude_percentage, 0.0f, 100.0f);

    const float output_scale = std::max(_config.output_scale, 0.0f);
    const float calibrated_percentage =
        std::clamp(magnitude_percentage * output_scale, 0.0f, 100.0f);

    const float max_duty_percentage = std::clamp(_config.max_duty_percentage, 0.0f, 100.0f);
    const float duty_fraction = (calibrated_percentage / 100.0f) * (max_duty_percentage / 100.0f);

    const uint32_t peak_ticks = _config.period_ticks / 2U;

    const uint32_t compare_value =
        static_cast<uint32_t>(std::lround(duty_fraction * static_cast<float>(peak_ticks)));

    return mcpwm_comparator_set_compare_value(motor_comparator, compare_value) == ESP_OK;
}

bool DCDriver::_force_all_off()
{
    /*
     * Physical output A LOW:
     *     force internal A LOW.
     *
     * Physical output B LOW:
     *     force internal B HIGH because B's GPIO is inverted.
     */
    const esp_err_t result_a = mcpwm_generator_set_force_level(motor_generator_a, 0, true);
    const esp_err_t result_b = mcpwm_generator_set_force_level(motor_generator_b, 1, true);

    return result_a == ESP_OK && result_b == ESP_OK;
}

bool DCDriver::_set_direction(MotorState direction)
{
    /*
     * Start from the safe state before enabling either direction.
     */
    if (!_force_all_off()) {
        return false;
    }

    switch (direction) {
        case MOTOR_CLOCKWISE:
            /*
             * Keep B physically LOW, then release A.
             * A's physical rising edge receives hardware dead-time.
             */
            return mcpwm_generator_set_force_level(motor_generator_a, -1, true) == ESP_OK;

        case MOTOR_COUNTER_CLOCKWISE:
            /*
             * Keep A physically LOW, then release B.
             * B's internal falling edge is delayed, which becomes a delayed
             * physical rising edge after GPIO inversion.
             */
            return mcpwm_generator_set_force_level(motor_generator_b, -1, true) == ESP_OK;

        default:
            return false;
    }
}

bool DCDriver::stop()
{
    const bool pwm_ok = _set_pwm(0.0f);
    const bool force_ok = _force_all_off();

    motor_state = MOTOR_STOPPED;

    return pwm_ok && force_ok;
}
} // namespace driver
