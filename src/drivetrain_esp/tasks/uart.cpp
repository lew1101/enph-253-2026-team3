#include <Arduino.h>
#include <array>
#include <atomic>

#include "comms/pb_codec.hpp"
#include "drive_message.pb.h"
#include "drive_handler.hpp"
#include "portmacro.h"
#include "tasks/uart.hpp"

static constexpr char TAG[] = "drive_uart";
using namespace UartTaskConfig;

namespace {
TaskHandle_t s_tx_task_handle = nullptr;
TaskHandle_t s_rx_task_handle = nullptr;

comms::UartLink *s_uart_link = nullptr;

std::atomic_bool s_link_connected{false};

DriveMessageHandler s_handler;

void _set_link_connected(bool connected)
{
    const bool previous = s_link_connected.exchange(connected, std::memory_order_acq_rel);
    if (previous == connected) return;

    ESP_LOGI(TAG, "UART link %s", connected ? "connected" : "disconnected");
    if (!connected) s_handler.on_link_disconnected();
}

void _tx_task(void *)
{
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        const drive_DriveUartMessage status_message =
            s_handler.make_status(s_link_connected.load(std::memory_order_acquire), millis());

        std::array<uint8_t, drive_DriveUartMessage_size> payload{};
        size_t encoded_size = 0;

        const esp_err_t encode_err =
            comms::pbcodec::encode<drive_DriveUartMessage, drive_DriveUartMessage_fields>(
                status_message, payload.data(), payload.size(), &encoded_size);

        if (encode_err != ESP_OK) {
            ESP_LOGE(TAG, "status encode failed");
        } else {
            uint16_t sequence;
            const esp_err_t send_err = s_uart_link->send(payload.data(), encoded_size, &sequence);

            if (send_err != ESP_OK)
                ESP_LOGW(TAG, "status send failed: %s", esp_err_to_name(send_err));
        }

        vTaskDelayUntil(&last_wake, TX_SEND_SYNC_PERIOD);
    }
}

void _rx_task(void *)
{
    std::array<uint8_t, robot_RobotUartMessage_size> payload{};
    TickType_t last_receive = xTaskGetTickCount();

    while (true) {
        uint16_t size = 0, sequence = 0;

        // blocking
        const esp_err_t receive_err =
            s_uart_link->receive(payload.data(), payload.size(), &size, &sequence, RX_TIMEOUT);

        if (receive_err == ESP_OK) {
            robot_RobotUartMessage message = robot_RobotUartMessage_init_zero;
            const esp_err_t decode_err =
                comms::pbcodec::decode<robot_RobotUartMessage, robot_RobotUartMessage_fields>(
                    payload.data(), size, &message);

            if (decode_err == ESP_OK) {
                const esp_err_t handle_err = s_handler.handle(message);

                if (handle_err != ESP_OK)
                    ESP_LOGW(TAG, "message rejected: %s", esp_err_to_name(handle_err));

                last_receive = xTaskGetTickCount();
                _set_link_connected(true);

            } else {
                ESP_LOGW(TAG, "message decode failed: %s", esp_err_to_name(decode_err));
                continue;
            }
        } else if (receive_err != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "UART receive failed: %s", esp_err_to_name(receive_err));
        }

        const TickType_t now = xTaskGetTickCount();
        const bool is_connected = s_link_connected.load(std::memory_order_acquire);

        if (is_connected && now - last_receive >= UART_LINK_TIMEOUT) _set_link_connected(false);
    }
}
} // namespace

bool uart_link_connected() { return s_link_connected.load(std::memory_order_acquire); }

esp_err_t start_uart_tasks(TaskHandle_t *tx_out, TaskHandle_t *rx_out)
{
    if (s_tx_task_handle != nullptr || s_rx_task_handle != nullptr) return ESP_ERR_INVALID_STATE;

    static comms::UartLink link;
    ESP_RETURN_ON_ERROR(link.init(UART_LINK_CFG), TAG, "UART init failed");

    s_uart_link = &link;

    if (xTaskCreatePinnedToCore(_rx_task,
                                "drive_uart_rx",
                                TASK_RX_STACK_DEPTH,
                                nullptr,
                                TASK_RX_PRIORITY,
                                &s_rx_task_handle,
                                TASK_RX_CORE_ID) != pdPASS)
        return ESP_FAIL;

    if (xTaskCreatePinnedToCore(_tx_task,
                                "drive_uart_tx",
                                TASK_TX_STACK_DEPTH,
                                nullptr,
                                TASK_TX_PRIORITY,
                                &s_tx_task_handle,
                                TASK_TX_CORE_ID) != pdPASS) {
        vTaskDelete(s_rx_task_handle);
        s_rx_task_handle = nullptr;
        return ESP_FAIL;
    }

    if (tx_out) *tx_out = s_tx_task_handle;
    if (rx_out) *rx_out = s_rx_task_handle;
    return ESP_OK;
}
