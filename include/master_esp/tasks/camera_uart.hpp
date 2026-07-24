#pragma once

#include "comms/uart_link.hpp"
#include "vision.pb.h"

namespace CameraUartTaskConfig {
constexpr gpio_num_t TX_PIN = GPIO_NUM_39;
constexpr gpio_num_t RX_PIN = GPIO_NUM_38;

constexpr uint32_t TASK_RX_STACK_DEPTH = 4096;
constexpr UBaseType_t TASK_RX_PRIORITY = 4;
constexpr BaseType_t TASK_RX_CORE_ID = 0;

constexpr TickType_t RX_TIMEOUT = pdMS_TO_TICKS(20);
constexpr TickType_t UART_LINK_TIMEOUT = pdMS_TO_TICKS(500);

constexpr comms::UartLink::Config UART_LINK_CFG{
    .port = UART_NUM_2,
    .tx_pin = TX_PIN,
    .rx_pin = RX_PIN,
    .baud_rate = 115'200,
    .rx_buffer_size = 1024,
    .tx_buffer_size = 0,
};
} // namespace CameraUartTaskConfig

esp_err_t start_camera_uart_task(TaskHandle_t *rx_handle_out = nullptr);
esp_err_t get_teletubby_detection(vision_TeletubbyDetection *out, TickType_t timeout = 0);
bool camera_uart_connected();
