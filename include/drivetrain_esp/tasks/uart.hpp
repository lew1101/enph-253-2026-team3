#pragma once

#include "hal/gpio_types.h"
#include "hal/uart_types.h"

#include "comms/uart_link.hpp"

#include "portmacro.h"
#include "robot_message.pb.h"
#include "drive_message.pb.h"

using UartRxHandler = void (*)(const robot_RobotUartMessage &message, uint16_t sequence);

namespace UartTaskConfig {
// RX task
constexpr uint32_t TASK_RX_STACK_DEPTH = 4096;
constexpr UBaseType_t TASK_RX_PRIORITY = 4;
constexpr BaseType_t TASK_RX_CORE_ID = 0;

// TX task
constexpr uint32_t TASK_TX_STACK_DEPTH = 4096;
constexpr UBaseType_t TASK_TX_PRIORITY = 3;
constexpr BaseType_t TASK_TX_CORE_ID = 0;

// UART receive behavior
constexpr TickType_t RX_TIMEOUT = pdMS_TO_TICKS(20);
constexpr TickType_t TX_SEND_SYNC_PERIOD = pdMS_TO_TICKS(100);
// Periodic state synchronization

// Connection supervision
constexpr TickType_t UART_LINK_TIMEOUT = pdMS_TO_TICKS(500);

constexpr comms::UartLink::Config UART_LINK_CFG{
    .port = UART_NUM_1,

    .tx_pin = GPIO_NUM_14,
    .rx_pin = GPIO_NUM_13,

    .baud_rate = 460'800,

    .rx_buffer_size = 2048,
    .tx_buffer_size = 1024,
};
} // namespace UartTaskConfig

esp_err_t start_uart_tasks(TaskHandle_t *tx_handle_out, TaskHandle_t *rx_handle_out);
esp_err_t get_latest_message(robot_RobotUartMessage *message_out, TickType_t timeout);
