#pragma once
#include "Arduino.h"

#include "freertos/idf_additions.h"
#include "portmacro.h"

namespace TapeSenseTaskConfig {
constexpr uint32_t TASK_STACK_DEPTH = 4096;
constexpr UBaseType_t TASK_PRIORITY = 4;
constexpr BaseType_t TASK_CORE_ID = 1;
constexpr float TASK_PERIOD_MS = 20.0f; // 50 Hz control loop

constexpr gpio_num_t FL_TAPE_PIN = GPIO_NUM_1;
constexpr gpio_num_t FM_TAPE_PIN = GPIO_NUM_2;
constexpr gpio_num_t FR_TAPE_PIN = GPIO_NUM_3;

constexpr gpio_num_t BL_TAPE_PIN = GPIO_NUM_4;
constexpr gpio_num_t BM_TAPE_PIN = GPIO_NUM_5;
constexpr gpio_num_t BR_TAPE_PIN = GPIO_NUM_6;

constexpr gpio_num_t L1_TAPE_PIN = GPIO_NUM_7;
constexpr gpio_num_t L2_TAPE_PIN = GPIO_NUM_8;

constexpr uint16_t TAPE_HIGH_THRESHOLD = 2000;
constexpr uint16_t TAPE_LOW_THRESHOLD = 1500;
}; // namespace TapeSenseTaskConfig

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

    TickType_t tick;
    bool valid;
};

//
esp_err_t start_tape_sense_task(TaskHandle_t *out_handle);
bool get_tape_snapshot(TapeSnapshot *out, TickType_t timeout);
