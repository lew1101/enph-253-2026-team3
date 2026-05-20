#pragma once

#include <Arduino.h>

#include "comms/handler.hpp"

using telemetry::RobotCallbacks;

static constexpr uint32_t PERIOD_MS = 50;
static constexpr uint32_t TASK_STACK_SIZE = 4096;
static constexpr uint32_t TASK_PRIORITY = 5;

struct WifiUdpConfig {
    uint32_t stack_depth;
    UBaseType_t priority;
    BaseType_t core_id;
    uint32_t period_ms;
};

struct WifiUdpContext {
    WifiUdpConfig config;
    RobotCallbacks robot_cbs;
};

void start_wifi_udp_task(const WifiUdpContext &context);
