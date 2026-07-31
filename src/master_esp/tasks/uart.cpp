#include <array>
#include <atomic>

#include "freertos/FreeRTOS.h"
#include "comms/pb_codec.hpp"
#include "drive_command_handler.hpp"
#include "esp_random.h"
#include "freertos/idf_additions.h"
#include "supervisor.hpp"
#include "tasks/uart.hpp"

static constexpr char TAG[] = "master_uart";

using namespace MasterUartTaskConfig;
using comms::UartLink;

namespace {
TaskHandle_t s_tx_task_handle = nullptr;
TaskHandle_t s_rx_task_handle = nullptr;

QueueHandle_t s_tx_queue = nullptr;
QueueHandle_t s_drive_status_queue = nullptr;

UartLink s_uart_link{};

std::atomic_bool s_link_connected{false};
std::atomic_uint32_t s_next_sequence{1};
std::atomic_uint32_t s_last_completed_sequence{0};
uint32_t s_session_id = 0;

DriveCommandHandler s_drive_command_handler{COMMAND_ACK_TIMEOUT, COMMAND_MAX_RETRIES};

esp_err_t _transmit(const robot_RobotUartMessage &message)
{
    std::array<uint8_t, robot_RobotUartMessage_size> payload{};
    size_t encoded_size = 0;

    ESP_RETURN_ON_ERROR(
        (comms::pbcodec::encode<robot_RobotUartMessage, robot_RobotUartMessage_fields>(
            message, payload.data(), payload.size(), &encoded_size)),
        TAG,
        "failed to encode message");

    uint16_t packet_sequence;
    return s_uart_link.send(payload.data(), encoded_size, &packet_sequence);
}

robot_RobotUartMessage _make_state_sync()
{
    robot_RobotUartMessage message = robot_RobotUartMessage_init_zero;

    message.which_payload = robot_RobotUartMessage_state_tag;

    message.payload.state.sequence = s_next_sequence.fetch_add(1, std::memory_order_relaxed);
    message.payload.state.control_flags = supervisor::g_robot_control_flags == nullptr
                                              ? 0
                                              : xEventGroupGetBits(
                                                    supervisor::g_robot_control_flags);
    message.payload.state.session_id = s_session_id;

    return message;
}

void _tx_task(void *)
{
    TickType_t last_state_sync = xTaskGetTickCount();

    while (true) {
        robot_RobotUartMessage message;

        // block for polling period until next transmission message
        if (xQueueReceive(s_tx_queue, &message, TX_POLL_PERIOD) == pdTRUE) {
            if (message.which_payload == robot_RobotUartMessage_drive_command_tag) {
                s_drive_command_handler.submit(message, xTaskGetTickCount());

                const esp_err_t err = _transmit(message);
                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "drive command send failed: %s", esp_err_to_name(err));
                }
            } else {
                const esp_err_t err = _transmit(message);
                if (err != ESP_OK) ESP_LOGW(TAG, "UART send failed: %s", esp_err_to_name(err));
            }
        }

        const TickType_t now = xTaskGetTickCount();

        // did not receive acknowledgement, retry sending same command
        robot_RobotUartMessage retry;
        if (s_drive_command_handler.retry_due(now, &retry)) {
            const esp_err_t err = _transmit(retry);
            if (err != ESP_OK)
                ESP_LOGW(TAG, "drive command retry failed: %s", esp_err_to_name(err));
        }

        // periodic state sync
        if (now - last_state_sync >= STATE_SYNC_PERIOD) {
            const robot_RobotUartMessage state = _make_state_sync();
            const esp_err_t err = _transmit(state);
            if (err != ESP_OK) ESP_LOGW(TAG, "state sync send failed: %s", esp_err_to_name(err));
            last_state_sync = now;
        }
    }
}

void _rx_task(void *)
{
    std::array<uint8_t, drive_DriveUartMessage_size> payload{};
    TickType_t last_valid_receive = xTaskGetTickCount();
    uint32_t consecutive_timeouts = 0;
    uint32_t previous_fault = 0;

    ESP_LOGI(TAG, "rx uart started");

    while (true) {
        uint16_t payload_size = 0;
        uint16_t packet_sequence = 0;

        // block for timeout period until new messsage
        const esp_err_t receive_err = s_uart_link.receive(
            payload.data(), payload.size(), &payload_size, &packet_sequence, RX_TIMEOUT);

        if (receive_err == ESP_OK) {
            consecutive_timeouts = 0;
            drive_DriveUartMessage drivetrain_status = drive_DriveUartMessage_init_zero;

            // decode message
            const esp_err_t decode_err =
                comms::pbcodec::decode<drive_DriveUartMessage, drive_DriveUartMessage_fields>(
                    payload.data(), payload_size, &drivetrain_status);

            if (decode_err == ESP_OK) {
                // update drivetrain latest status
                xQueueOverwrite(s_drive_status_queue, &drivetrain_status);

                // drivetrain acknowledge new command
                s_drive_command_handler.acknowledge(drivetrain_status.last_command_sequence);
                last_valid_receive = xTaskGetTickCount();

                const bool was_connected =
                    s_link_connected.exchange(true, std::memory_order_acq_rel);
                if (!was_connected) {
                    ESP_LOGI(TAG, "drivetrain UART connected");
                    xEventGroupSetBits(supervisor::g_robot_status_flags,
                                       robot_flags::STATUS_DRIVE_CONNECTED);
                }

                const uint32_t flags = drivetrain_status.flags;
                if ((flags & robot_flags::DRIVE_STATUS_ESTOP_LATCHED) != 0) {
                    xEventGroupSetBits(supervisor::g_robot_status_flags,
                                       robot_flags::STATUS_ESTOP_ACTIVE);
                } else {
                    xEventGroupClearBits(supervisor::g_robot_status_flags,
                                         robot_flags::STATUS_ESTOP_ACTIVE);
                }

                if (drivetrain_status.fault != 0) {
                    xEventGroupSetBits(supervisor::g_robot_status_flags,
                                       robot_flags::STATUS_FAULT_ACTIVE |
                                           robot_flags::STATUS_DRIVE_FAULT_ACTIVE);
                    if (previous_fault == 0)
                        supervisor::notify_main(robot_flags::NOTIFY_FAULT);
                } else {
                    xEventGroupClearBits(supervisor::g_robot_status_flags,
                                         robot_flags::STATUS_DRIVE_FAULT_ACTIVE);
                }

                const bool target_reached =
                    (flags & robot_flags::DRIVE_STATUS_TARGET_REACHED) != 0;
                const uint32_t completed_sequence = drivetrain_status.last_command_sequence;
                const uint32_t previously_completed =
                    s_last_completed_sequence.load(std::memory_order_acquire);
                if (target_reached && completed_sequence != 0 &&
                    completed_sequence != previously_completed) {
                    s_last_completed_sequence.store(completed_sequence, std::memory_order_release);
                    supervisor::notify_main(robot_flags::NOTIFY_DRIVE_TARGET_REACHED);
                }

                previous_fault = drivetrain_status.fault;

            } else {
                ESP_LOGW(TAG, "status protobuf decode failed");
            }
        } else if (receive_err == ESP_ERR_TIMEOUT) {
            ++consecutive_timeouts;
            if (consecutive_timeouts % 50 == 0)
                ESP_LOGI(TAG,
                         "waiting for UART frames (%u consecutive timeouts)",
                         static_cast<unsigned>(consecutive_timeouts));
        } else {
            consecutive_timeouts = 0;
            // some uart error... not timeout
            ESP_LOGW(TAG, "UART receive failed: %s", esp_err_to_name(receive_err));
        }

        const TickType_t now = xTaskGetTickCount();

        const bool is_connected = s_link_connected.load(std::memory_order_acquire);
        const bool is_link_timedout = now - last_valid_receive >= UART_LINK_TIMEOUT;

        if (is_connected && is_link_timedout) {
            s_link_connected.store(false, std::memory_order_release);
            previous_fault = 0;
            xEventGroupClearBits(supervisor::g_robot_status_flags,
                                 robot_flags::STATUS_DRIVE_CONNECTED);
            xEventGroupSetBits(supervisor::g_robot_status_flags,
                               robot_flags::STATUS_FAULT_ACTIVE |
                                   robot_flags::STATUS_DRIVE_FAULT_ACTIVE);

            supervisor::notify_main(robot_flags::NOTIFY_FAULT);

            ESP_LOGW(TAG, "drivetrain UART disconnected");
        }
    }
}
} // namespace

esp_err_t send_robot_message(const robot_RobotUartMessage &message, TickType_t timeout)
{
    if (s_tx_queue == nullptr) return ESP_ERR_INVALID_STATE;
    return xQueueSend(s_tx_queue, &message, timeout) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t send_drive_command(const robot_DriveCommand &command,
                             TickType_t timeout,
                             uint32_t *out_sequence)
{
    if (supervisor::g_robot_control_flags == nullptr) return ESP_ERR_INVALID_STATE;

    // Put the current flags ahead of every command in the same TX queue. This
    // guarantees the drivetrain evaluates the command against current control state.
    const esp_err_t state_err = send_robot_message(_make_state_sync(), timeout);
    if (state_err != ESP_OK) return state_err;

    robot_RobotUartMessage message = robot_RobotUartMessage_init_zero;

    message.which_payload = robot_RobotUartMessage_drive_command_tag;
    message.payload.drive_command = command;

    if (message.payload.drive_command.sequence == 0) {
        message.payload.drive_command.sequence =
            s_next_sequence.fetch_add(1, std::memory_order_relaxed);
    }

    const esp_err_t err = send_robot_message(message, timeout);
    if (err == ESP_OK && out_sequence != nullptr) {
        *out_sequence = message.payload.drive_command.sequence;
    }
    return err;
}

esp_err_t get_drive_status(drive_DriveUartMessage *out, TickType_t timeout)
{
    if (out == nullptr) return ESP_ERR_INVALID_ARG;
    if (s_drive_status_queue == nullptr) return ESP_ERR_INVALID_STATE;
    return xQueuePeek(s_drive_status_queue, out, timeout) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

bool drive_uart_link_connected() { return s_link_connected.load(std::memory_order_acquire); }

bool drive_command_pending() { return s_drive_command_handler.pending(); }

bool drive_command_retry_failed() { return s_drive_command_handler.failed(); }

bool drive_command_completed(uint32_t sequence)
{
    return sequence != 0 &&
           s_last_completed_sequence.load(std::memory_order_acquire) == sequence;
}

esp_err_t start_master_uart_tasks(TaskHandle_t *tx_out, TaskHandle_t *rx_out)
{
    if (s_tx_task_handle != nullptr || s_rx_task_handle != nullptr) return ESP_ERR_INVALID_STATE;

    do {
        s_session_id = esp_random();
    } while (s_session_id == 0);

    s_tx_queue = xQueueCreate(TX_QUEUE_LENGTH, sizeof(robot_RobotUartMessage));
    configASSERT(s_tx_queue != nullptr);

    s_drive_status_queue = xQueueCreate(1, sizeof(drive_DriveUartMessage));
    configASSERT(s_drive_status_queue != nullptr);

    ESP_RETURN_ON_ERROR(s_uart_link.init(UART_LINK_CFG), TAG, "UART init failed");

    BaseType_t ok = xTaskCreatePinnedToCore(_rx_task,
                                            "master_uart_rx",
                                            TASK_RX_STACK_DEPTH,
                                            nullptr,
                                            TASK_RX_PRIORITY,
                                            &s_rx_task_handle,
                                            TASK_RX_CORE_ID);
    if (ok != pdPASS) return ESP_FAIL;

    ok = xTaskCreatePinnedToCore(_tx_task,
                                 "master_uart_tx",
                                 TASK_TX_STACK_DEPTH,
                                 nullptr,
                                 TASK_TX_PRIORITY,
                                 &s_tx_task_handle,
                                 TASK_TX_CORE_ID);
    if (ok != pdPASS) {
        vTaskDelete(s_rx_task_handle);
        s_rx_task_handle = nullptr;
        return ESP_FAIL;
    }

    if (tx_out != nullptr) *tx_out = s_tx_task_handle;
    if (rx_out != nullptr) *rx_out = s_rx_task_handle;
    return ESP_OK;
}
