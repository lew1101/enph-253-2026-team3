#pragma once

#include "freertos/idf_additions.h"
#include <cstdint>

constexpr uint8_t IMU_I2C_ADDRESS = 0x4B;
constexpr uint32_t IMU_I2C_FREQ = 400'000ul;

struct ImuTaskConfig {
    uint32_t stack_depth = 4096;
    UBaseType_t priority = 4;
    BaseType_t core_id = 1;

    int sda_pin;
    int scl_pin;
    int int_pin;
    int rst_pin;

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

esp_err_t start_tape_sense_task(const ImuTaskConfig &task_cfg, TaskHandle_t *out_handle);
bool get_imu_snapshot(ImuSnapshot *out, TickType_t timeout);

