#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "comms/handler.hpp"
#include "tasks/wifi_ap_udp.hpp"

#include "esp_err.h"

static constexpr char TAG[] = "wifi_ap_udp";

static constexpr char AP_SSID[] = "ESP32-Robot";
static constexpr char AP_PASS[] = "robot1234";

static constexpr uint16_t LOCAL_UDP_PORT = 4210;

static constexpr size_t TX_BUFFER_SIZE = 256;
static constexpr size_t RX_BUFFER_SIZE = 256;

using telemetry::PacketError;
using telemetry::PacketHandler;
using telemetry::PacketHandlerResult;
using telemetry::ResponseType;

static void wifi_udp_task(void *arg)
{
    const auto *context = reinterpret_cast<const WifiUdpContext *>(arg);

    const WifiUdpConfig config = context->config;
    const RobotCallbacks robot_cbs = context->robot_cbs;

    WiFiUDP udp;
    uint16_t sequence = 0;

    // init wifi in AP mode
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);

    const IPAddress ap_ip = WiFi.softAPIP();

    ESP_LOGI(TAG, "ESP Wifi AP IP: %s", ap_ip.toString());

    // begin udp socket
    udp.begin(LOCAL_UDP_PORT);

    ESP_LOGI(TAG, "UDP listening on port %u", static_cast<unsigned>(LOCAL_UDP_PORT));

    uint8_t rx_buffer[RX_BUFFER_SIZE];
    uint8_t tx_buffer[TX_BUFFER_SIZE];

    PacketHandler packet_handler{
        robot_cbs, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer)};

    while (true) {
        // Parse Packet
        while (true) {
            int rx_len = udp.parsePacket();

            if (rx_len <= 0) break;

            const IPAddress remote_ip = udp.remoteIP();
            const uint16_t remote_port = udp.remotePort();

            // Receive and parse packet, if avaliable
            rx_len = udp.read(rx_buffer, RX_BUFFER_SIZE);

            if (rx_len > 0) {
                PacketHandlerResult result = packet_handler.handle_received_packet(rx_len);

                if (!result.ok()) {
                    ESP_LOGW(TAG, "Packet error code %u", static_cast<unsigned>{result.error});
                    continue;
                }

                if (result.should_send()) {
                    udp.beginPacket(udp.remoteIP(), udp.remotePort());
                    udp.write(reinterpret_cast<const uint8_t *>(tx_buffer), result.tx_len);
                    udp.endPacket();
                }
            }
        }

        const IPAddress remote_ip = udp.remoteIP();
        const uint16_t remote_port = udp.remotePort();

        PacketHandlerResult result = packet_handler.build_telemetry_packet(sequence);

        if (!result.ok()) {
            ESP_LOGW(TAG, "Packet error code %u", static_cast<unsigned>{result.error});
            continue;
        }

        if (result.should_send()) {
            udp.beginPacket(udp.remoteIP(), udp.remotePort());
            udp.write(reinterpret_cast<const uint8_t *>(tx_buffer), result.tx_len);
            udp.endPacket();
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

    const BaseType_t ok = xTaskCreatePinnedToCore(wifi_udp_task,
                                                  TAG,
                                                  config.stack_depth,
                                                  reinterpret_cast<void *>(&context),
                                                  config.priority,
                                                  nullptr,
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
