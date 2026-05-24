#include <Arduino.h>

#include "comms/handler.hpp"
#include "esp_log.h"

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

#include "tasks/wifi_ap_udp.hpp"

static constexpr char TAG[] = "main";

static WifiUdpContext udp_context{};

void setup()
{
    Serial.begin(SERIAL_BAUD);

    start_wifi_udp_task(udp_context, nullptr);
}

void loop()
{
    ESP_LOGI(TAG, "Heartbeat");
    vTaskDelay(pdMS_TO_TICKS(1000));
}
