#pragma once

#include <cstdint>

#include "esp_err.h" 

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "AccelStepper.h"

namespace driver{
class StepperDriver {
    public:
        struct Config{
            gpio_num_t dir_1 = GPIO_NUM_NC;
            gpio_num_t dir_2 = GPIO_NUM_NC;
        };

        StepperDriver() = default;

        explicit StepperDriver(const Config &config); 

        bool init();
        
        bool set_speed(float percentage); // 0% to 100%
        bool turn_clockwise();
        bool turn_c_clockwise();
        bool stop();
        
    
    private:
        Config _config;

        int motor_interface_type = 1; // 1 for DRIVER, 4 for FULL4WIRE, 8 for HALF4WIRE
    };
}