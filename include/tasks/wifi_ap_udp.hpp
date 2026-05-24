#pragma once

#include <Arduino.h>

#include "comms/comms.hpp"

using comms::RobotCallbacks;

struct WifiUdpConfig {
    uint32_t stack_depth = 4096;
    UBaseType_t priority = 2;
    BaseType_t core_id = 1;
    uint32_t period_ms = 50;
};

struct WifiUdpContext {
    WifiUdpConfig config {};
    RobotCallbacks callbacks {};
};

esp_err_t start_wifi_udp_task(WifiUdpContext &context, TaskHandle_t *out_handle);
