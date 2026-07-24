#include "tasks/camera_uart.hpp"

#include <array>
#include <atomic>
#include <cmath>

#include "comms/pb_codec.hpp"
#include "esp_check.h"

namespace {
static constexpr char TAG[] = "camera_uart";
using namespace CameraUartTaskConfig;

TaskHandle_t s_rx_task_handle = nullptr;
QueueHandle_t s_detection_queue = nullptr;

comms::UartLink s_uart_link{};

std::atomic_bool s_link_connected{false};

bool _is_valid_detection(const vision_TeletubbyDetection &detection)
{
    if (detection.image_width == 0 || detection.image_height == 0) return false;
    if (!detection.detected)
        return detection.teletubby_type == vision_TeletubbyType_TELETUBBY_TYPE_UNSPECIFIED;

    if (!std::isfinite(detection.confidence) || detection.confidence < 0.0f ||
        detection.confidence > 1.0f)
        return false;
    if (!detection.has_bounding_box) return false;
    if (detection.teletubby_type < vision_TeletubbyType_TELETUBBY_TYPE_YELLOW ||
        detection.teletubby_type > vision_TeletubbyType_TELETUBBY_TYPE_PURPLE)
        return false;

    const auto &box = detection.bounding_box;
    if (box.width == 0 || box.height == 0) return false;
    if (box.x >= detection.image_width || box.y >= detection.image_height) return false;
    if (box.width > detection.image_width - box.x) return false;
    if (box.height > detection.image_height - box.y) return false;

    return true;
}

void _set_link_connected(bool connected)
{
    const bool previous = s_link_connected.exchange(connected, std::memory_order_acq_rel);
    if (previous != connected)
        ESP_LOGI(TAG, "camera UART %s", connected ? "connected" : "disconnected");
}

void _rx_task(void *)
{
    std::array<uint8_t, vision_TeletubbyDetection_size> payload{};
    TickType_t last_valid_receive = xTaskGetTickCount();

    while (true) {
        uint16_t payload_size = 0;
        uint16_t packet_sequence = 0;
        const esp_err_t receive_err = s_uart_link.receive(payload.data(),
                                                         payload.size(),
                                                         &payload_size,
                                                         &packet_sequence,
                                                         RX_TIMEOUT);

        if (receive_err == ESP_OK) {
            vision_TeletubbyDetection detection = vision_TeletubbyDetection_init_zero;
            const esp_err_t decode_err =
                comms::pbcodec::decode<vision_TeletubbyDetection,
                                       vision_TeletubbyDetection_fields>(
                    payload.data(), payload_size, &detection);

            if (decode_err == ESP_OK && _is_valid_detection(detection)) {
                xQueueOverwrite(s_detection_queue, &detection);
                last_valid_receive = xTaskGetTickCount();
                _set_link_connected(true);
            } else {
                ESP_LOGW(TAG, "invalid camera detection payload");
            }
        } else if (receive_err != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "camera UART receive failed: %s", esp_err_to_name(receive_err));
        }

        if (s_link_connected.load(std::memory_order_acquire) &&
            xTaskGetTickCount() - last_valid_receive >= UART_LINK_TIMEOUT)
            _set_link_connected(false);
    }
}
} // namespace

esp_err_t start_camera_uart_task(TaskHandle_t *rx_handle_out)
{
    if (s_rx_task_handle != nullptr) return ESP_ERR_INVALID_STATE;

    s_detection_queue = xQueueCreate(1, sizeof(vision_TeletubbyDetection));
    ESP_RETURN_ON_FALSE(
        s_detection_queue != nullptr, ESP_ERR_NO_MEM, TAG, "failed to create detection queue");

    esp_err_t err = s_uart_link.init(CameraUartTaskConfig::UART_LINK_CFG);
    if (err != ESP_OK) {
        vQueueDelete(s_detection_queue);
        s_detection_queue = nullptr;
        return err;
    }

    const BaseType_t task_created = xTaskCreatePinnedToCore(_rx_task,
                                                            "camera_uart_rx",
                                                            CameraUartTaskConfig::TASK_RX_STACK_DEPTH,
                                                            nullptr,
                                                            CameraUartTaskConfig::TASK_RX_PRIORITY,
                                                            &s_rx_task_handle,
                                                            CameraUartTaskConfig::TASK_RX_CORE_ID);
    if (task_created != pdPASS) {
        s_rx_task_handle = nullptr;
        s_uart_link.deinit();
        vQueueDelete(s_detection_queue);
        s_detection_queue = nullptr;
        return ESP_FAIL;
    }

    if (rx_handle_out != nullptr) *rx_handle_out = s_rx_task_handle;
    return ESP_OK;
}

esp_err_t get_teletubby_detection(vision_TeletubbyDetection *out, TickType_t timeout)
{
    if (out == nullptr) return ESP_ERR_INVALID_ARG;
    if (s_detection_queue == nullptr) return ESP_ERR_INVALID_STATE;
    return xQueuePeek(s_detection_queue, out, timeout) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

bool camera_uart_connected()
{
    return s_link_connected.load(std::memory_order_acquire);
}
