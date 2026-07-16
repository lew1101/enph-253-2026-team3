#pragma once
#include "Arduino.h"

#include "freertos/idf_additions.h"
#include "portmacro.h"

struct TapeSenseTaskConfig {
    gpio_num_t fl_tape_pin = GPIO_NUM_NC;
    gpio_num_t fm_tape_pin = GPIO_NUM_NC;
    gpio_num_t fr_tape_pin = GPIO_NUM_NC;

    gpio_num_t bl_tape_pin = GPIO_NUM_NC;
    gpio_num_t bm_tape_pin = GPIO_NUM_NC;
    gpio_num_t br_tape_pin = GPIO_NUM_NC;

    gpio_num_t l1_tape_pin = GPIO_NUM_NC;
    gpio_num_t l2_tape_pin = GPIO_NUM_NC;

    uint16_t high_threshold = 2000;
    uint16_t low_threshold = 1500;

    uint32_t stack_depth = 4096;
    UBaseType_t priority = 4;
    BaseType_t core_id = 1;
    float period_ms = 20.0f; // 50 Hz control loop
};

struct TapeSnapshot {
    // front
    bool tape_fl = false;
    bool tape_fm = false;
    bool tape_fr = false;
    float front_err = 0.0f;

    // back
    bool tape_bl = false;
    bool tape_bm = false;
    bool tape_br = false;
    float back_err = 0.0f;

    // left
    bool tape_l1 = false;
    bool tape_l2 = false;
    float left_err = 0.0f;
};

//
esp_err_t start_tape_sense_task(const TapeSenseTaskConfig &task_config, TaskHandle_t *out_handle);
bool get_tape_snapshot(TapeSnapshot *out, TickType_t timeout);
