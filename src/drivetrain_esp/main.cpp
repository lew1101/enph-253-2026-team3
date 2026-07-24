#include "freertos/idf_additions.h"
#include "tasks/drive.hpp"
#include "tasks/tape_sense.hpp"
#include "tasks/imu.hpp"
#include "tasks/uart.hpp"

TaskHandle_t uart_tx_handle;
TaskHandle_t uart_rx_handle;
TaskHandle_t drive_handle;
TaskHandle_t imu_sensor_handle;
TaskHandle_t tape_handle;

void setup() {
    ESP_ERROR_CHECK_WITHOUT_ABORT(start_uart_tasks(&uart_tx_handle, &uart_rx_handle));
    ESP_ERROR_CHECK_WITHOUT_ABORT(start_imu_task(&imu_sensor_handle));
    ESP_ERROR_CHECK_WITHOUT_ABORT(start_drive_task(&drive_handle));
    ESP_ERROR_CHECK_WITHOUT_ABORT(start_tape_sense_task(&tape_handle));

    vTaskDelete(nullptr);
}

