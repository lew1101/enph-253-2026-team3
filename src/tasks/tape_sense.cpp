#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tasks/tape_sense.hpp"
#include "esp_err.h"
#include "supervisor.hpp"
#include "tasks/drive.hpp"
#include "control/pid.hpp"
#include "robot_states.hpp"

using namespace supervisor;
using namespace state;

namespace {
    TapeSenseTaskConfig s_task_config;

    bool FL_sees_tape = false;
    bool FR_sees_tape = false;

    float left_sensor_value = 0.0f;
    float right_sensor_value = 0.0f;
    float error = 0.0f;
    float prev_error = 0.0f;
}

void tape_task(void *arg)
{
    (void)arg;

    while (true) {
        xEventGroupWaitBits(
            g_robot_flags, RobotFlag::ROBOT_FLAG_TAPE_ACTIVE, pdFALSE, pdTRUE, portMAX_DELAY);

        // Define the GPIO pins for the tape sensors and read them
        gpio_set_direction(s_task_config.fl_tape_pin, GPIO_MODE_INPUT);
        gpio_set_direction(s_task_config.fr_tape_pin, GPIO_MODE_INPUT);

        if (analogRead(s_task_config.fl_tape_pin) )

        left_sensor_value = analogRead(s_task_config.fl_tape_pin);
        right_sensor_value = analogRead(s_task_config.fr_tape_pin);

        // hystersis and tape detection logic
        if (FL_sees_tape) {
            if (left_sensor_value < 1500.0) {
                FL_sees_tape = false;
            }
        }
        else if (left_sensor_value > 2000.0) {
            FL_sees_tape = true;
        }

        if (FR_sees_tape) {
            if (right_sensor_value < 1500.0) {
                FR_sees_tape = false;
            }
        }
        else if (right_sensor_value > 2000.0) {
            FR_sees_tape = true;
        }

        error = get_tape_error(FL_sees_tape, FR_sees_tape, prev_error);
        prev_error = error;
        g_tape_error.store(error);
        ESP_LOGI("TapeSense", "FL: %d, FR: %d, Error: %.2f", FL_sees_tape, FR_sees_tape, error);

        vTaskDelay(pdMS_TO_TICKS(static_cast<uint32_t>(s_task_config.period_ms))); 
    }
}

esp_err_t start_tape_sense_task(const TapeSenseTaskConfig &task_config, TaskHandle_t *out_handle)
{
    s_task_config = task_config;
    xTaskCreatePinnedToCore(tape_task, "tape_task", s_task_config.stack_depth, nullptr, s_task_config.priority, out_handle, s_task_config.core_id);
    return ESP_OK;
}

float get_tape_error(bool FL_sees_tape, bool FR_sees_tape, float prev_error)
{
    if (FL_sees_tape && FR_sees_tape) {
        return 0.0f; // Both sensors see tape, no error
    }
    else if (FL_sees_tape && !FR_sees_tape) {
        return 1.0f; // Left sensor sees tape, robot is too far right
    }
    else if (!FL_sees_tape && FR_sees_tape) {
        return -1.0f; // Right sensor sees tape, robot is too far left
    }
    else if (!FL_sees_tape && !FR_sees_tape) {
        // Both sensors do not see tape, use previous error to determine direction
        if (prev_error > 0.0f) {
            return 5.0f; // Last known position was to the right
        }
        else if (prev_error < 0.0f) {
            return -5.0f; // Last known position was to the left
        }
    }

    return error;
}