#include "freertos/idf_additions.h"
#include "supervisor.hpp"

#include "esp_check.h"
#include "esp_err.h"
static constexpr char TAG[] = "supervisor";

namespace supervisor {
TaskHandle_t g_task_handle = nullptr;
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
    ESP_RETURN_ON_FALSE(g_task_handle != nullptr,
                        ESP_ERR_INVALID_STATE,
                        TAG,
                        "cannot notify supervisor because it is not initialized");

    return xTaskNotify(g_task_handle, notification, eSetBits) == pdPASS ? ESP_OK : ESP_FAIL;
}

void wait_for_notification(uint32_t mask, TickType_t timeout)
{
    uint32_t notif_val = 0;
    do {
        xTaskNotifyWait(0,    // Clear nothing on entry
                        mask, // Clear target bit on exit
                        &notif_val,
                        timeout);
    } while ((notif_val & mask) == 0);
}

} // namespace supervisor
