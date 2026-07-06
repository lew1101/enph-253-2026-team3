#pragma once

#include "portmacro.h"
#include "state.hpp"

namespace supervisor {
struct SupervisorConfig {
    uint32_t event_queue_len = 16;
};

void init(SupervisorConfig &cfg, void (*event_handler)(const RobotEvent &));
void update();

bool peek_state(RobotState &out, TickType_t timeout = 0);
bool post_event(RobotEventType type, TickType_t timeout = 0);
bool post_event_from_isr(RobotEventType type, BaseType_t *higher_priority_task_woken);

} // namespace supervisor
