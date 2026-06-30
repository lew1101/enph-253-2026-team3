#pragma once

#include <cstdint>

#include "esp_err.h" 

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"

namespace driver{
class DCDriver {
    public:
        struct Config{
            gpio_num_t clockwise_pwm_output = GPIO_NUM_NC;
            gpio_num_t c_clockwise_pwm_output = GPIO_NUM_NC;

            // MCPWM Setup
            int group_id = 0;
            mcpwm_timer_handle_t timer = nullptr;
            uint32_t period_ticks = 0;
            float clamp_percentage = 100.0f;
        };

        explicit DCDriver(const Config &config); 

        bool init();
        
        bool set_speed(float percentage); // 0% to 100%
        bool turn_clockwise();
        bool turn_c_clockwise();
        bool stop();
        
    
    private:
        Config _config;

        bool _clockwise = false;
        mcpwm_oper_handle_t motor_operator = nullptr;
        mcpwm_cmpr_handle_t motor_comparator = nullptr;
        mcpwm_gen_handle_t motor_generator_a = nullptr;
        mcpwm_gen_handle_t motor_generator_b = nullptr;
    };

} // namespace driver