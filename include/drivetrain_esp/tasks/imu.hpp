#pragma once

#include "freertos/idf_additions.h"
#include "soc/gpio_num.h"
#include <cstdint>

namespace ImuTaskConfig {
    constexpr uint32_t TASK_STACK_DEPTH = 4096;
    constexpr UBaseType_t TASK_PRIORITY = 4;
    constexpr BaseType_t TASK_CORE_ID = 1;

    constexpr int SCL_PIN = GPIO_NUM_9;
    constexpr int SDA_PIN = GPIO_NUM_10;
    constexpr int INT_PIN = GPIO_NUM_11;
    constexpr int RST_PIN = GPIO_NUM_12;

    constexpr uint8_t IMU_I2C_ADDRESS = 0x4B;
    constexpr uint32_t IMU_I2C_FREQ_HZ = 400'000ul;

    constexpr uint16_t REPORT_PERIOD_MS = 10; // 100 Hz
};

// =============================

struct ImuSnapshot {
    float yaw;
    float pitch;
    float roll;

    TickType_t tick;
    uint32_t reset_count;
    bool valid;
};

esp_err_t start_imu_task(TaskHandle_t *out_handle);
bool get_imu_snapshot(ImuSnapshot *out, TickType_t timeout);
