#pragma once
#include "arduino.h"
#include "freertos/idf_additions.h"
#include "portmacro.h"

struct TapeSenseTaskConfig {
    uint32_t stack_depth = 4096;
    UBaseType_t priority = 4;
    BaseType_t core_id = 1;
    float period_ms = 20.0f; // 50 Hz control loop
    gpio_num_t fl_tape_pin = GPIO_NUM_1;
    gpio_num_t fr_tape_pin = GPIO_NUM_3;
};

// 
esp_err_t start_tape_sense_task(const TapeSenseTaskConfig &task_config,
                                TaskHandle_t *out_handle);

// private
float get_tape_error(bool FL_sees_tape, bool FR_sees_tape, float prev_error);