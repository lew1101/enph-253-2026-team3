#pragma once
#include "freertos/idf_additions.h"

#include "sensors/metal_detector.hpp"

struct MetalTaskConfig {
    uint32_t start_delay_us = 1000;
    uint32_t md_deadtime_us = 30000;
    uint32_t stack_depth = 4096;
    UBaseType_t priority = 3;
    BaseType_t core_id = 1;
};

bool get_metal_detector_snapshot(const metal_detector::MetalDetector::Snapshot &snapshot);
esp_err_t start_metal_detector_task(const metal_detector::MetalDetector::Config &cfg,
                                    const MetalTaskConfig &task_cfg,
                                    TaskHandle_t *out_handle = nullptr);


