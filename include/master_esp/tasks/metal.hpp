#pragma once
#include "freertos/idf_additions.h"

#include "sensors/metal_detector.hpp"

namespace MetalTaskConfig {
constexpr bool MD_1_ENABLE = true;
constexpr bool MD_2_ENABLE = false;

constexpr gpio_num_t METAL_1_PULSE_PIN = GPIO_NUM_5;
constexpr gpio_num_t METAL_1_ADC_PIN = GPIO_NUM_1;
constexpr gpio_num_t METAL_2_PULSE_PIN = GPIO_NUM_21;
constexpr gpio_num_t METAL_2_ADC_PIN = GPIO_NUM_3;

constexpr uint32_t TIMER_FREQ = 1000000;

constexpr uint32_t MD_START_DELAY_US = 1000;
constexpr uint32_t MD_SAMPLE_PERIOD_US = 30000;
constexpr uint32_t MD_STAGGER_US = MD_SAMPLE_PERIOD_US / 2;

constexpr uint32_t TASK_STACK_DEPTH = 4096;
constexpr UBaseType_t TASK_PRIORITY = 3;
constexpr BaseType_t TASK_CORE_ID = 1;

constexpr metal_detector::MetalDetector::Config METAL_1_CFG{
    .gpio_md_pulse = METAL_1_PULSE_PIN,
    .gpio_md_adc_in = METAL_1_ADC_PIN,

    .md_pulse_us = 20,
    .md_blank_us = 5,

    .detect_count_required = 4,
    .clear_count_required = 8,

    .detect_threshold = 2.0f,
    .clear_threshold = 1.0f,

    .baseline_samples = 400,
    .baseline_discard = 70,

    .output_alpha = 0.06f,
    .baseline_alpha = 0.00005f,
};

constexpr metal_detector::MetalDetector::Config METAL_2_CFG{
    .gpio_md_pulse = METAL_2_PULSE_PIN,
    .gpio_md_adc_in = METAL_2_ADC_PIN,

    .md_pulse_us = 20,
    .md_blank_us = 5,

    .detect_count_required = 4,
    .clear_count_required = 8,

    .detect_threshold = 2.0f,
    .clear_threshold = 1.0f,

    .baseline_samples = 400,
    .baseline_discard = 70,

    .output_alpha = 0.06f,
    .baseline_alpha = 0.00005f,
};
}; // namespace MetalTaskConfig

struct MetalDetectorSnapshots {
    metal_detector::MetalDetector::Snapshot detector_1;
    metal_detector::MetalDetector::Snapshot detector_2;
};

bool get_metal_detector_snapshots(MetalDetectorSnapshots *snapshots);
esp_err_t start_metal_detector_task(TaskHandle_t *out_handle = nullptr);
