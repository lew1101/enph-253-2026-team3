#include "freertos/idf_additions.h"
#include "esp_log.h"
#include <atomic>

#include "comms/uart_link.hpp"
#include "comms/pb_codec.hpp"
#include "portmacro.h"

#include "robot_message.pb.h"
#include "drive_message.pb.h"

#include "tasks/uart.hpp"

static constexpr char TAG[] = "uart_link";

using namespace UartTaskConfig;
using namespace comms;

namespace {
TaskHandle_t s_tx_task_handle = nullptr;
TaskHandle_t s_rx_task_handle = nullptr;

UartLink *s_uart_link;

std::atomic_bool s_link_connected{false};
QueueHandle_t s_rx_latest_queue;

void _set_link_connected(bool connected)
{
    const bool previous = s_link_connected.exchange(connected, std::memory_order_acq_rel);

    if (previous == connected) return;
    ESP_LOGI(TAG, "UART link %s", connected ? "connected" : "disconnected");
}

esp_err_t _send_message(drive_DriveUartMessage &message)
{
    std::array<uint8_t, drive_DriveUartMessage_size> payload{};

    size_t encoded_size = 0;
    esp_err_t err = comms::pbcodec::encode<drive_DriveUartMessage, drive_DriveUartMessage_fields>(
        message, payload.data(), payload.size(), &encoded_size);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to encode UART message: %s", esp_err_to_name(err));

        return err;
    }

    uint16_t sequence = 0;

    err = s_uart_link->send(payload.data(), encoded_size, sequence);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to send UART message: %s", esp_err_to_name(err));

        return err;
    }

    ESP_LOGV(TAG,
             "sent UART message: sequence=%u, payload_size=%u",
             static_cast<unsigned>(sequence),
             static_cast<unsigned>(encoded_size));

    return ESP_OK;
}

void _handle_message(robot_RobotUartMessage &master_messsage) {}

void _tx_task(void *arg)
{
    (void)arg;

    std::array<uint8_t, drive_DriveUartMessage_size> payload{};
    TickType_t last_state_sync = xTaskGetTickCount();

    while (true) {
        vTaskDelay(TX_SEND_SYNC_PERIOD);
    }
}

void _rx_task(void *arg)
{
    (void)arg;

    std::array<uint8_t, robot_RobotUartMessage_size> payload{};
    TickType_t last_valid_receive = xTaskGetTickCount();

    while (true) {
        uint16_t payload_size = 0;
        uint16_t sequence = 0;

        auto receive_err = s_uart_link->receive( //
            payload.data(),
            payload.size(),
            &payload_size,
            &sequence,
            RX_TIMEOUT);

        if (receive_err == ESP_OK) {
            robot_RobotUartMessage mcu_message = robot_RobotUartMessage_init_zero;

            auto decode_err =
                comms::pbcodec::decode<robot_RobotUartMessage, robot_RobotUartMessage_fields>(
                    payload.data(), payload_size, &mcu_message);

            if (decode_err != ESP_OK) {
                ESP_LOGW(TAG,
                         "failed to decode UART payload: sequence=%u, error=%s",
                         static_cast<unsigned>(sequence),
                         esp_err_to_name(decode_err));

                continue;
            }

            xQueueOverwrite(s_rx_latest_queue, &mcu_message);

            last_valid_receive = xTaskGetTickCount();
            _set_link_connected(true);

            ESP_LOGV(TAG,
                     "received UART message: sequence=%u, payload_size=%u",
                     static_cast<unsigned>(sequence),
                     static_cast<unsigned>(payload_size));
        } else if (receive_err != ESP_ERR_TIMEOUT) {
            /*
             * Framing, CRC, UART, or other receive error. The task continues
             * looking for the next valid packet.
             */
            ESP_LOGW(TAG, "UART receive failed: %s", esp_err_to_name(receive_err));
            vTaskDelay(1); // avoid tight error loop
        }

        if (UART_LINK_TIMEOUT == 0) {
            continue;
        } else {
            const TickType_t now = xTaskGetTickCount();

            if (s_link_connected.load(std::memory_order_acquire) &&
                (now - last_valid_receive) >= UART_LINK_TIMEOUT) {
                _set_link_connected(false);
            }
        }
    }
}
} // namespace

esp_err_t get_latest_message(robot_RobotUartMessage *message_out, TickType_t timeout)
{
    return xQueuePeek(s_rx_latest_queue, message_out, timeout) == pdTRUE ? ESP_OK : ESP_FAIL;
}

esp_err_t start_uart_tasks(TaskHandle_t *tx_handle_out, TaskHandle_t *rx_handle_out)
{
    if (s_rx_task_handle != nullptr) {
        if (rx_handle_out != nullptr) {
            *rx_handle_out = s_rx_task_handle;
        }

        return ESP_ERR_INVALID_STATE;
    }

    if (s_tx_task_handle != nullptr) {
        if (tx_handle_out != nullptr) {
            *tx_handle_out = s_tx_task_handle;
        }

        return ESP_ERR_INVALID_STATE;
    }

    s_rx_latest_queue = xQueueCreate(1, sizeof(robot_RobotUartMessage));
    configASSERT(s_rx_latest_queue != nullptr);

    static UartLink uart_link{};
    ESP_ERROR_CHECK(uart_link.init(UART_LINK_CFG));

    s_uart_link = &uart_link;

    auto rx_ok = xTaskCreatePinnedToCore(_rx_task,
                                         "uart_rx_task",
                                         TASK_RX_STACK_DEPTH,
                                         nullptr,
                                         TASK_RX_PRIORITY,
                                         &s_rx_task_handle,
                                         TASK_RX_CORE_ID);

    if (rx_ok != pdPASS) {
        ESP_LOGE(TAG, "failed to instantiate uart rx task");

        s_rx_task_handle = nullptr;
        return ESP_FAIL;
    }

    auto tx_ok = xTaskCreatePinnedToCore(_rx_task,
                                         "uart_tx_task",
                                         TASK_TX_STACK_DEPTH,
                                         nullptr,
                                         TASK_RX_PRIORITY,
                                         &s_tx_task_handle,
                                         TASK_TX_CORE_ID);

    if (rx_ok != pdPASS) {
        ESP_LOGE(TAG, "failed to instantiate uart tx task");

        vTaskDelete(s_tx_task_handle);

        s_tx_task_handle = nullptr;
        s_rx_task_handle = nullptr;
        return ESP_FAIL;
    }

    if (rx_handle_out != nullptr) {
        *rx_handle_out = s_rx_task_handle;
    }

    if (tx_handle_out != nullptr) {
        *tx_handle_out = s_tx_task_handle;
    }

    return ESP_OK;
}
