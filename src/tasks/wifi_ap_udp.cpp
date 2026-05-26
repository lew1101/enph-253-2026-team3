#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "esp_err.h"

#include "comms/comms.hpp"

#include "tasks/wifi_ap_udp.hpp"

static constexpr char TAG[] = "wifi_ap_udp";

static constexpr char WIFI_AP_SSID[] = "enph-robot-team3";
static constexpr char WIFI_AP_PASSWORD[] = "robot1234";
static constexpr uint16_t UDP_PORT = 4210;

static constexpr size_t TX_BUFFER_SIZE = 256;
static constexpr size_t RX_BUFFER_SIZE = 256;

static constexpr size_t MAX_RX_PACKETS_PER_CYCLE = 4;
static constexpr TickType_t CONTROLLER_TIMEOUT = pdMS_TO_TICKS(5000);

bool send_udp_packet(
    WiFiUDP &udp, const IPAddress &ip, uint16_t port, const uint8_t *buffer, size_t length)
{
    if (!udp.beginPacket(ip, port)) {
        ESP_LOGW(TAG, "UDP beginPacket failed");
        return false;
    }

    const size_t bytes_written = udp.write(buffer, length);

    if (bytes_written != length) {
        ESP_LOGW(TAG,
                 "UDP write failed: wrote %u/%u bytes",
                 static_cast<unsigned>(bytes_written),
                 static_cast<unsigned>(length));
        return false;
    }

    if (!udp.endPacket()) {
        ESP_LOGW(TAG, "UDP endPacket failed");
        return false;
    }

    ESP_LOGV(TAG, "Packet Sent");

    return true;
}

static void wifi_udp_task(void *arg)
{
    using comms::PacketHandler;
    using comms::PacketHandlerResult;

    const auto *context = reinterpret_cast<const WifiUdpContext *>(arg);

    const WifiUdpConfig config = context->config;
    const RobotCallbacks robot_cbs = context->callbacks;

    WiFiUDP udp;
    uint16_t sequence = 0;

    WiFi.mode(WIFI_AP);

    if (!WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD)) {
        ESP_LOGE(TAG, "Failed to start Wi-Fi AP");
        vTaskDelete(nullptr);
        return;
    }

    const IPAddress ap_ip = WiFi.softAPIP();

    ESP_LOGI(TAG, "ESP Wi-Fi AP IP: %s", ap_ip.toString().c_str());

    if (!udp.begin(UDP_PORT)) {
        ESP_LOGE(TAG, "Failed to begin UDP socket");
        vTaskDelete(nullptr);
        return;
    }

    ESP_LOGI(TAG, "UDP listening on port %u", static_cast<unsigned>(LOCAL_UDP_PORT));

    uint8_t rx_buffer[RX_BUFFER_SIZE];
    uint8_t tx_buffer[TX_BUFFER_SIZE];

    PacketHandler packet_handler{
        robot_cbs, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer)};

    IPAddress controller_ip;
    uint16_t controller_port = 0;
    bool has_controller = false;
    TickType_t last_controller_tick = 0;

    while (true) {
        const TickType_t cycle_tick = xTaskGetTickCount();

        // expire controllers that are past timeout
        if (has_controller && cycle_tick - last_controller_tick > CONTROLLER_TIMEOUT) {
            ESP_LOGI(TAG, "Controller timed out");
            has_controller = false;
        }

        for (size_t i = 0; i < MAX_RX_PACKETS_PER_CYCLE; ++i) {
            const int packet_len = udp.parsePacket();

            if (packet_len <= 0) {
                break;
            }

            const IPAddress sender_ip = udp.remoteIP();
            const uint16_t sender_port = udp.remotePort();

            if (packet_len > static_cast<int>(sizeof(rx_buffer))) {
                ESP_LOGW(TAG, "Dropping oversized UDP packet: %d bytes", packet_len);

                udp.clear();
                continue;
            }

            const int bytes_read = udp.read(rx_buffer, sizeof(rx_buffer));

            if (bytes_read != packet_len) {
                ESP_LOGW(
                    TAG, "UDP read length mismatch: expected %d, got %d", packet_len, bytes_read);

                udp.clear();
                continue;
            }

            const TickType_t now = xTaskGetTickCount();

            const bool from_controller =
                has_controller && sender_ip == controller_ip && sender_port == controller_port;

            if (!has_controller) {
                controller_ip = sender_ip;
                controller_port = sender_port;
                has_controller = true;

                ESP_LOGI(TAG,
                         "Controller registered: %s:%u",
                         controller_ip.toString().c_str(),
                         static_cast<unsigned>(controller_port));
            } else if (!from_controller) {
                ESP_LOGW(TAG, "Ignoring packet from non-controller client");
                continue;
            }

            // Only the registered controller reaches this point.
            PacketHandlerResult result =
                packet_handler.handle_received_packet(static_cast<size_t>(bytes_read));

            if (!result.ok()) {
                ESP_LOGW(TAG, "Packet error code %u", static_cast<unsigned>(result.error));

                continue;
            }

            // Only valid controller packets keep the session alive.
            last_controller_tick = now;

            if (result.should_send() &&
                send_udp_packet(udp, controller_ip, controller_port, tx_buffer, result.tx_len)) {
                ++sequence;
            }
        }

        const TickType_t now = xTaskGetTickCount();

        if (has_controller && now - last_controller_tick <= CONTROLLER_TIMEOUT) {
            PacketHandlerResult result = packet_handler.build_telemetry_packet(sequence);

            if (!result.ok()) {
                ESP_LOGW(
                    TAG, "Telemetry packet error code %u", static_cast<unsigned>(result.error));
            } else if (result.should_send() &&
                       send_udp_packet(
                           udp, controller_ip, controller_port, tx_buffer, result.tx_len)) {
                ++sequence;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(config.period_ms));
    }
}

esp_err_t start_wifi_udp_task(WifiUdpContext &context, TaskHandle_t *out_handle)
{
    static TaskHandle_t handle = nullptr;

    if (handle != nullptr) {
        ESP_LOGE(TAG, "wifi udp task already initialized");
        return ESP_ERR_NOT_SUPPORTED;
    }

    const WifiUdpConfig config = context.config;
    if (config.period_ms <= 0) return ESP_ERR_INVALID_ARG;

    const BaseType_t ok = xTaskCreatePinnedToCore(wifi_udp_task,
                                                  TAG,
                                                  config.stack_depth,
                                                  reinterpret_cast<void *>(&context),
                                                  config.priority,
                                                  &handle,
                                                  config.core_id // core 0 or 1
    );

    if (ok != pdPASS) {
        return ESP_FAIL;
    }

    if (out_handle != nullptr) {
        *out_handle = handle;
    }

    return ESP_OK;
}
