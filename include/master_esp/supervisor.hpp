#pragma once

#include "shared/robot_flags.hpp"
#include "freertos/idf_additions.h"

#include <cstdint>

namespace supervisor {
extern TaskHandle_t* g_main_handle_ptr;
extern EventGroupHandle_t g_robot_status_flags;
extern EventGroupHandle_t g_robot_control_flags;

void init();
inline void attach_main_loop(TaskHandle_t* task_handle) { g_main_handle_ptr = task_handle; }
esp_err_t notify_main(uint32_t notification);
bool wait_for_notification(uint32_t mask, TickType_t timeout = portMAX_DELAY);
} // namespace supervisor
