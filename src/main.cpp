#include <Arduino.h>
#include "robot_state.hpp"

static constexpr char TAG[] = "main";

std::atomic<RobotState> robot_state{RobotState::ROBOT_IDLE};

void setup()
{
    esp_log_level_set("metal_detector", ESP_LOG_VERBOSE);

    vTaskDelete(nullptr); // delete the superloop task
}

void loop() {}
