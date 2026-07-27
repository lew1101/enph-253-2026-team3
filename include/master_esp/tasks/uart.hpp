#pragma once

#include "portmacro.h"
#include "shared/robot_flags.hpp"
#include "comms/uart_link.hpp"
#include "drive_message.pb.h"
#include "robot_message.pb.h"

namespace MasterUartTaskConfig {
constexpr gpio_num_t RX_PIN = GPIO_NUM_48;
constexpr gpio_num_t TX_PIN = GPIO_NUM_47;

constexpr uint32_t TASK_RX_STACK_DEPTH = 4096;
constexpr UBaseType_t TASK_RX_PRIORITY = 4;
constexpr BaseType_t TASK_RX_CORE_ID = 0;

constexpr uint32_t TASK_TX_STACK_DEPTH = 4096;
constexpr UBaseType_t TASK_TX_PRIORITY = 3;
constexpr BaseType_t TASK_TX_CORE_ID = 0;

constexpr TickType_t RX_TIMEOUT = pdMS_TO_TICKS(20);
constexpr TickType_t STATE_SYNC_PERIOD = pdMS_TO_TICKS(100);
constexpr TickType_t UART_LINK_TIMEOUT = pdMS_TO_TICKS(500);
constexpr TickType_t COMMAND_ACK_TIMEOUT = pdMS_TO_TICKS(100);

constexpr TickType_t TX_POLL_PERIOD = pdMS_TO_TICKS(10);
constexpr uint32_t COMMAND_MAX_RETRIES = 5;
constexpr size_t TX_QUEUE_LENGTH = 8;

constexpr comms::UartLink::Config UART_LINK_CFG{
    .port = UART_NUM_1,
    .tx_pin = TX_PIN,
    .rx_pin = RX_PIN,
    .baud_rate = 460'800,
    .rx_buffer_size = 2048,
    .tx_buffer_size = 1024,
};
} // namespace MasterUartTaskConfig

esp_err_t start_master_uart_tasks(TaskHandle_t *tx_handle_out = nullptr,
                                  TaskHandle_t *rx_handle_out = nullptr);
esp_err_t send_robot_message(const robot_RobotUartMessage &message, TickType_t timeout = 0);
esp_err_t send_drive_command(const robot_DriveCommand &command, TickType_t timeout = portMAX_DELAY);

esp_err_t get_drive_status(drive_DriveUartMessage *out, TickType_t timeout = 0);

bool drive_uart_link_connected();
bool drive_command_pending();
bool drive_command_retry_failed();
