#include "tasks/camera_uart.hpp"

#include <Arduino.h>
#include "esp_err.h"
#include "freertos/idf_additions.h"
#include "shared/robot_flags.hpp"

#include "drivers/pulse_tx.hpp"

#include <array>
#include <atomic>
#include <cmath>

#include "comms/pb_codec.hpp"
#include "esp_check.h"
#include "supervisor.hpp"

namespace {
static constexpr char TAG[] = "camera_uart";
using namespace CameraUartTaskConfig;
using driver::PulseTx;

PulseTx s_teletubby_led_pulse_tx{TELETUBBY_LED_PIN};

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

    const float width_ratio =
        static_cast<float>(box.width) / static_cast<float>(detection.image_width);
    const float height_ratio =
        static_cast<float>(box.height) / static_cast<float>(detection.image_height);
    const float area_ratio = width_ratio * height_ratio;

    return width_ratio >= MIN_BOX_WIDTH_RATIO && height_ratio >= MIN_BOX_HEIGHT_RATIO &&
           area_ratio >= MIN_BOX_AREA_RATIO && area_ratio <= MAX_BOX_AREA_RATIO;
}

void _set_link_connected(bool connected)
{
    const bool previous = s_link_connected.exchange(connected, std::memory_order_acq_rel);
    if (previous != connected)
        log_i("%s: camera UART %s", TAG, connected ? "connected" : "disconnected");
}

void _rx_task(void *)
{
    log_i("%s: starting camera task", TAG);

    std::array<uint8_t, vision_TeletubbyDetection_size> payload{};
    TickType_t last_valid_receive = xTaskGetTickCount();
    TickType_t last_positive_detection = 0;
    uint32_t consecutive_timeouts = 0;
    uint8_t consecutive_detections = 0;
    bool teletubby_seen = false;
    bool prev_teletubby_seen = false;

    static constexpr PulseTx::Word TELETUBBY_BLINK_MSG[]{
        {1, 200'000},
        {0, 180'000},
        {1, 200'000},
        {0, 180'000},
        {1, 200'000},
        {0, 0},
    };

    while (true) {
        uint16_t payload_size = 0;
        uint16_t packet_sequence = 0;

        const esp_err_t receive_err = s_uart_link.receive(
            payload.data(), payload.size(), &payload_size, &packet_sequence, RX_TIMEOUT);

        if (receive_err == ESP_OK) {
            consecutive_timeouts = 0;
            // log_v("%s: received frame: packet_sequence=%u payload_size=%u",
            //       TAG,
            //       static_cast<unsigned>(packet_sequence),
            //       static_cast<unsigned>(payload_size));

            vision_TeletubbyDetection msg = vision_TeletubbyDetection_init_zero;
            const esp_err_t decode_err =
                comms::pbcodec::decode<vision_TeletubbyDetection, vision_TeletubbyDetection_fields>(
                    payload.data(), payload_size, &msg);

            if (decode_err == ESP_OK && _is_valid_detection(msg)) {
                xQueueOverwrite(s_detection_queue, &msg);
                last_valid_receive = xTaskGetTickCount();
                _set_link_connected(true);

                if (msg.detected) {
                    if (consecutive_detections < REQUIRED_CONSECUTIVE_DETECTIONS)
                        ++consecutive_detections;

                    last_positive_detection = last_valid_receive;
                    if (consecutive_detections >= REQUIRED_CONSECUTIVE_DETECTIONS)
                        teletubby_seen = true;
                } else {
                    consecutive_detections = 0;
                }

                switch (msg.teletubby_type) {
                    case vision_TeletubbyType_TELETUBBY_TYPE_YELLOW:
                        log_v("%s: teletubby: yellow", TAG);
                        break;

                    case vision_TeletubbyType_TELETUBBY_TYPE_GREEN:
                        log_v("%s: teletubby: green", TAG);
                        break;
                    case vision_TeletubbyType_TELETUBBY_TYPE_RED:
                        log_v("%s: teletubby: red", TAG);
                        break;
                    case vision_TeletubbyType_TELETUBBY_TYPE_PURPLE:
                        log_v("%s: teletubby: purple", TAG);
                        break;
                    default:
                        break;
                }

            } else {
                consecutive_detections = 0;
                log_w("%s: invalid detection: decode=%s detected=%d type=%d image=%ux%u box=%d",
                      TAG,
                      esp_err_to_name(decode_err),
                      msg.detected,
                      static_cast<int>(msg.teletubby_type),
                      static_cast<unsigned>(msg.image_width),
                      static_cast<unsigned>(msg.image_height),
                      msg.has_bounding_box);
            }
        } else if (receive_err == ESP_ERR_TIMEOUT) {
            ++consecutive_timeouts;
            if (consecutive_timeouts % 50 == 0)
                log_i("%s: waiting for UART frames (%u consecutive timeouts)",
                      TAG,
                      static_cast<unsigned>(consecutive_timeouts));
        } else if (receive_err != ESP_ERR_TIMEOUT) {
            consecutive_timeouts = 0;
            log_w("%s: camera UART receive failed: %s", TAG, esp_err_to_name(receive_err));
        }

        if (s_link_connected.load(std::memory_order_acquire) &&
            xTaskGetTickCount() - last_valid_receive >= UART_LINK_TIMEOUT)
            _set_link_connected(false);

        const TickType_t now = xTaskGetTickCount();
        if (!s_link_connected.load(std::memory_order_acquire) ||
            (teletubby_seen && now - last_positive_detection >= TELETUBBY_LED_HOLD_TIME)) {
            teletubby_seen = false;
        }

        const auto should_flash_led =
            robot_flags::has_flag(xEventGroupGetBits(supervisor::g_robot_control_flags),
                                  robot_flags::CONTROL_CAMERA_ENABLE);

        if (should_flash_led) {
            if (!prev_teletubby_seen && teletubby_seen) {
                log_i("%s: teletubby detected; flashing LED", TAG);
                const esp_err_t flash_err = s_teletubby_led_pulse_tx.transmit(TELETUBBY_BLINK_MSG);
                if (flash_err != ESP_OK)
                    log_w("%s: failed to transmit teletubby flash: %s",
                          TAG,
                          esp_err_to_name(flash_err));
            }
        }

        if (teletubby_seen) {
            xEventGroupSetBits(supervisor::g_robot_status_flags,
                               robot_flags::STATUS_TELETUBBY_SEEN);
        } else {
            xEventGroupClearBits(supervisor::g_robot_status_flags,
                                 robot_flags::STATUS_TELETUBBY_SEEN);
        }

        prev_teletubby_seen = teletubby_seen;
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

    err = s_teletubby_led_pulse_tx.init();
    log_i("%s: teletubby LED RMT init: %s", TAG, esp_err_to_name(err));

    if (err != ESP_OK) {
        s_uart_link.deinit();
        vQueueDelete(s_detection_queue);
        s_detection_queue = nullptr;
        return err;
    }

    const BaseType_t task_created =
        xTaskCreatePinnedToCore(_rx_task,
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

bool camera_uart_connected() { return s_link_connected.load(std::memory_order_acquire); }
