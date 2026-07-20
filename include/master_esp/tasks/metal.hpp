#pragma once
#include "freertos/idf_additions.h"

#include "sensors/metal_detector.hpp"

namespace MetalTaskConfig {
constexpr uint32_t TIMER_FREQ = 1000000;

constexpr uint32_t MD_START_DELAY_US = 1000;
constexpr uint32_t MD_DEADTIME_US = 30000;

constexpr uint32_t TASK_STACK_DEPTH = 4096;
constexpr UBaseType_t TASK_PRIORITY = 3;
constexpr BaseType_t TASK_CORE_ID = 1;

constexpr metal_detector::MetalDetector::Config METAL_CFG{
    .gpio_md_pulse = GPIO_NUM_NC,
    .gpio_md_adc_in = GPIO_NUM_NC,

    .md_pulse_us = 100,
    .md_blank_us = 50,

    .detect_count_required = 4,
    .clear_count_required = 8,

    .detect_threshold = 7.0f,
    .clear_threshold = 4.0f,

    .baseline_samples = 400,
    .baseline_discard = 70,

    .output_alpha = 0.06f,
    .baseline_alpha = 0.00005f,
};
}; // namespace MetalTaskConfig

bool get_metal_detector_snapshot(metal_detector::MetalDetector::Snapshot &snapshot);
esp_err_t start_metal_detector_task(TaskHandle_t *out_handle = nullptr);
