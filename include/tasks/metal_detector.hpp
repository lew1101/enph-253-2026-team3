#pragma once
#include "freertos/idf_additions.h"
#include "portmacro.h"
#include "driver/gpio.h"

namespace metal_detector {

enum class MetalState : uint8_t {
    METAL_CALIBRATION,
    METAL_NONE,
    METAL_DETECTED
};

struct MetalConfig {
    gpio_num_t gpio_md_pulse = GPIO_NUM_NC;
    gpio_num_t gpio_md_adc_in = GPIO_NUM_NC;

    uint32_t md_pulse_us = 80;
    uint32_t md_blank_us = 50;
    uint32_t md_deadtime_us = 30000;

    uint32_t start_delay_us = 1000;

    uint32_t detect_count_required = 4;
    uint32_t clear_count_required = 8;

    float detect_threshold = 7.0f;
    float clear_threshold = 4.0f;

    int baseline_samples = 400;
    int baseline_discard = 70;

    float output_alpha = 0.06f;
    float baseline_alpha = 0.00005f;
};

struct MetalTaskConfig {
    uint32_t stack_depth = 4096;
    UBaseType_t priority = 3;
    BaseType_t core_id = 1;
};

struct MetalSnapshot {
    int raw = 0;
    float sensor = 0.0f;
    float baseline = 0.0f;
    bool baseline_ready = false;
    MetalState state = MetalState::METAL_NONE;
};

esp_err_t start_metal_detector_task(const MetalTaskConfig &task_cfg,
                                    const MetalConfig &detector_cfg,
                                    TaskHandle_t *out_handle = nullptr);

void get_snapshot(MetalSnapshot &out);
void get_state(MetalState &out);

} // namespace metal_detector
