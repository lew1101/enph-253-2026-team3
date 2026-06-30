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

    // Set up MCPWM
    
}
}