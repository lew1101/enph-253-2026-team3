#pragma once

#include "freertos/idf_additions.h"
#include "soc/gpio_num.h"
#include <cstdint>

constexpr uint8_t IMU_I2C_ADDRESS = 0x4B;
constexpr uint32_t IMU_I2C_FREQ = 400'000ul;

struct ImuTaskConfig {
    uint32_t stack_depth = 4096;
    UBaseType_t priority = 4;
    BaseType_t core_id = 1;

    int scl_pin = GPIO_NUM_9;
    int sda_pin = GPIO_NUM_10;
    int int_pin = GPIO_NUM_11;
    int rst_pin = GPIO_NUM_12;

    uint8_t i2c_address = 0x4B;
    uint32_t i2c_frequency_hz = 400000;

    uint16_t report_period_ms = 10; // 100 Hz
};

struct ImuSnapshot {
    float yaw;
    float pitch;
    float roll;

    TickType_t tick;
    bool valid;
};

esp_err_t start_imu_task(const ImuTaskConfig &task_cfg, TaskHandle_t *out_handle);
bool get_imu_snapshot(ImuSnapshot *out, TickType_t timeout);

