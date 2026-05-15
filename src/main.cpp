#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static constexpr char TAG[] = "main";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Robot starting");

    while (1) {
        ESP_LOGI(TAG, "Heartbeat");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
