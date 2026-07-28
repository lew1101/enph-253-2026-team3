#pragma once

#include <cstdint>

#include "esp_err.h"

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"

namespace driver {
class DCDriver {
  private:
    enum MotorState { MOTOR_STOPPED, MOTOR_CLOCKWISE, MOTOR_COUNTER_CLOCKWISE };

  public:
    struct Config {
        gpio_num_t clockwise_pwm_output = GPIO_NUM_NC;
        gpio_num_t c_clockwise_pwm_output = GPIO_NUM_NC;

        // MCPWM Setup
        int group_id = 0;
        mcpwm_timer_handle_t timer = nullptr;
        uint32_t period_ticks = 0;
        uint32_t dead_time_ticks = 100;

        // Maximum physical duty.
        float max_duty_percentage = 100.0f;
        float output_scale = 1.0f;

        float min_percentage;
        float bias_percentage;
    };

    static constexpr inline uint32_t us_to_mcpwm_ticks(uint32_t us, uint32_t resolution_hz)
    {
        return static_cast<uint32_t>((static_cast<uint64_t>(us) * resolution_hz + 999'999ULL) /
                                     1'000'000ULL);
    }

    DCDriver() = default;

    explicit DCDriver(const Config &config);

    esp_err_t init();
    bool set_output(float percentage); // -100% to 100%
    bool stop();

  private:
    bool _set_pwm(float magnitude_percentage);
    bool _set_direction(MotorState direction);
    bool _force_all_off();

  private:
    Config _config;

    MotorState motor_state = MOTOR_STOPPED;
    mcpwm_oper_handle_t motor_operator = nullptr;
    mcpwm_cmpr_handle_t motor_comparator = nullptr;
    mcpwm_gen_handle_t motor_generator_a = nullptr;
    mcpwm_gen_handle_t motor_generator_b = nullptr;
};

} // namespace driver
