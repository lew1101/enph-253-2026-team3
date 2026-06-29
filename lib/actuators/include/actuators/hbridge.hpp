#pragma once

#include <cstdint>

#include "esp_err.h"

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"

namespace driver{
class DCDriver {
    public:
        struct Config{
            gpio_num_t clockwise_pwm = GPIO_NUM_NC;
            gpio_num_t c_clockwise_pwm = GPIO_NUM_NC;

            // MCPWM Setup

            uint32_t freq_hz = 50;
            uint8_t duty_res_bits = 16;

            mcpwm_timer_handle_t timer = nullptr;
            mcpwm_cmpr_handle_t comparator_a = nullptr;
            mcpwm_cmpr_handle_t comparator_b = nullptr;
            mcpwm_gen_handle_t gen_a = nullptr;
            mcpwm_gen_handle_t gen_b = nullptr;
        };

        explicit DCDriver(const Config &config); 

        bool init();
        
        bool set_speed(float percentage); // -100.0% to 100%, neg is counterclockwise and pos is clockwise
        
    };
} // namespace driver