#include "freertos/idf_additions.h"
#include "supervisor.hpp"

#include "esp_check.h"
#include "esp_err.h"
static constexpr char TAG[] = "supervisor";

namespace supervisor {
TaskHandle_t* g_main_handle_ptr = nullptr;
EventGroupHandle_t g_robot_status_flags = nullptr;
EventGroupHandle_t g_robot_control_flags = nullptr;

void init()
{
    g_robot_status_flags = xEventGroupCreate();
    configASSERT(g_robot_status_flags != nullptr);

    g_robot_control_flags = xEventGroupCreate();
    configASSERT(g_robot_control_flags != nullptr);
}

esp_err_t notify_main(uint32_t notification)
{
    ESP_RETURN_ON_FALSE(g_main_handle_ptr != nullptr && *g_main_handle_ptr != nullptr,
                        ESP_ERR_INVALID_STATE,
                        TAG,
                        "cannot notify supervisor because no autonomous task is running");

    return xTaskNotify(*g_main_handle_ptr, notification, eSetBits) == pdPASS ? ESP_OK : ESP_FAIL;
}

bool wait_for_notification(uint32_t mask, TickType_t timeout)
{
    TimeOut_t timeout_state;
    TickType_t remaining = timeout;

    vTaskSetTimeOutState(&timeout_state);

    while (true) {
        uint32_t notification_value = 0;

        const BaseType_t notified = xTaskNotifyWait(0,    // Clear nothing on entry
                                                    mask, // Clear requested bits on exit
                                                    &notification_value,
                                                    remaining);

        if (notified == pdFALSE) return false; // Timed out without receiving a notification
        if ((notification_value & mask) != 0) return true; // Requested notification bit received

        // A different notification woke the task. Update the remaining time.
        if (xTaskCheckForTimeOut(&timeout_state, &remaining) == pdTRUE) return false;
    }
}

} // namespace supervisor
