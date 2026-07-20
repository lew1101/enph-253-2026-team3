#include <Arduino.h>

#include "freertos/idf_additions.h"
#include "projdefs.h"
#include "tasks/metal.hpp"
#include "tasks/uart.hpp"
#include "supervisor.hpp"

#include "shared/robot_flags.hpp"

void setup()
{
    supervisor::init();
    supervisor::attach_main_loop();

    ESP_ERROR_CHECK(start_master_uart_tasks());
    ESP_ERROR_CHECK(start_metal_detector_task());
}

void loop() {
    xEventGroupSetBits(supervisor::g_robot_control_flags, robot_flags::CONTROL_DRIVE_ENABLED);


    vTaskDelay(portMAX_DELAY);

}
