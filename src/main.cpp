#include <Arduino.h>

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static constexpr char TAG[] = "main";

void setUp() {
    Serial.begin(SERIAL_BAUD);
}

void loop()
{
    ESP_LOGI(TAG, "Robot starting");

    while (1) {
        ESP_LOGI(TAG, "Heartbeat");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
