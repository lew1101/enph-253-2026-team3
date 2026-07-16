#pragma once

#include "freertos/idf_additions.h"

#include "comms/uart_link.hpp"

#include "portmacro.h"
#include "robot_message.pb.h"
#include "drive_message.pb.h"

using UartRxHandler = void (*)(const robot_RobotUartMessage &message, uint16_t sequence);

struct UartTaskConfig {
    // RX task
    uint32_t rx_stack_depth = 4096;
    UBaseType_t rx_priority = 4;
    BaseType_t rx_core_id = 0;

    // TX task
    uint32_t tx_stack_depth = 4096;
    UBaseType_t tx_priority = 3;
    BaseType_t tx_core_id = 0;

    // UART receive behavior
    TickType_t rx_timeout = pdMS_TO_TICKS(20);
    TickType_t tx_send_sync_period = pdMS_TO_TICKS(100);
    // Periodic state synchronization

    // Connection supervision
    TickType_t link_timeout = pdMS_TO_TICKS(500);
};

esp_err_t start_uart_tasks(const UartTaskConfig &task_cfg,
                           const comms::UartLink::Config &uart_link_cfg,
                           TaskHandle_t *out_handle);

esp_err_t get_latest_message(robot_RobotUartMessage* message_out, TickType_t timeout);
