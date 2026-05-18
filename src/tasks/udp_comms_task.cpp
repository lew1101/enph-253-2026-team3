#include "tasks/udp_comms_task.hpp"

#include "comms/udp_socket.hpp"
#include "comms/tlv_writer.hpp"
#include "comms/tlv_parser.hpp"

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/task.h"

static constexpr const char TAG[] = "udp_comms_task";

namespace telemetry {
static constexpr size_t TX_BUFFER_SIZE = 256;
static constexpr size_t RX_BUFFER_SIZE = 256;

struct UdpCommsTaskContext {
    UdpCommsConfig config;
    UdpCommsCallbacks callbacks;
};

static esp_err_t write_telemetry_packet(uint8_t *buffer,
                                        size_t capacity,
                                        const RobotTelemetry &telemetry,
                                        size_t &out_len)
{
    out_len = 0;

    TlvPacketWriter writer(buffer, capacity);

    writer.begin(PacketType::TELEMETRY, telemetry.sequence, telemetry.tick);

    writer.add<TlvType::BATT_V>(telemetry.battery_v)
        .add<TlvType::DRIVETRAIN_L>(telemetry.drivetrain_l)
        .add<TlvType::DRIVETRAIN_R>(telemetry.drivetrain_r)
        .add<TlvType::ERR>(telemetry.err);

    esp_err_t err = writer.finish();
    if (err != ESP_OK) return err;

    out_len = writer.size();
    return ESP_OK;
}

static esp_err_t parse_command_packet(const uint8_t *buffer, size_t len, RobotCommand &command)
{
    command = {};

    TlvPacketParser parser(buffer, len);
    if (!parser.ok()) {
        return ESP_ERR_INVALID_STATE;
    }

    TlvPacketHeader header = parser.parse_header();

    // check if header type is correct.
    if (header.packet_type != PacketType::COMMAND) return ESP_ERR_INVALID_ARG;

    esp_err_t err;
    while (parser.has_next()) {
        switch (parser.peek_type()) {
            case TlvType::CMD_DRIVETRAIN_L:
                err = parser.read<TlvType::CMD_DRIVETRAIN_L>(command.drivetrain_l);
                break;

            case TlvType::CMD_DRIVETRAIN_R:
                err = parser.read<TlvType::CMD_DRIVETRAIN_R>(command.drivetrain_r);
                break;

            case TlvType::CMD_ESTOP:
                err = parser.read<TlvType::CMD_ESTOP>(command.estop);
                break;

            default:
                err = parser.skip();
                break;
        }

        if (err != ESP_OK) {
            command.valid = false;
            return err;
        }
    }

    command.sequence = header.packet_seq;
    command.tick = header.tick;
    command.valid = true;

    return ESP_OK;
}

static void send_telemetry(const UdpSocket &sock,
                           const UdpCommsCallbacks &callbacks,
                           uint8_t *tx_buffer,
                           size_t tx_buffer_capacity)
{
    if (callbacks.read_telemetry == nullptr) return;

    RobotTelemetry telemetry{};

    esp_err_t err = callbacks.read_telemetry(telemetry, callbacks.state);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Unable to read telemetry from callback");
        return;
    }

    size_t tx_len = 0;

    err = write_telemetry_packet(tx_buffer, tx_buffer_capacity, telemetry, tx_len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Unable to send telemetry packet");
        return;
    }

    if (sock.send(tx_buffer, tx_len) != ESP_OK) {
        ESP_LOGW(TAG, "UDP telemetry send failed");
    }
}

static void receive_commands(const UdpSocket &sock,
                             const UdpCommsCallbacks &callbacks,
                             uint8_t *rx_buffer,
                             size_t rx_buffer_capacity)
{
    while (true) {
        size_t rx_len = 0;

        esp_err_t err = sock.receive(rx_buffer, rx_buffer_capacity, rx_len);
        if (err == ESP_ERR_NOT_FOUND) return; // up to date, read all packets.

        if (err != ESP_OK) {
            ESP_LOGW(TAG, "UDP command receive failed");
            return;
        }

        RobotCommand command{};

        err = parse_command_packet(rx_buffer, rx_len, command);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to parse packet");
            continue; // corrupted packet, skip it
        };

        if (callbacks.handle_command != nullptr) {
            esp_err_t err = callbacks.handle_command(command, callbacks.state);

            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Command callback failed");
            }
        }
    }
}

static void udp_comms_task(void *arg)
{
    const auto *context = static_cast<const UdpCommsTaskContext *>(arg);

    const UdpCommsConfig &config = context->config;
    const UdpCommsCallbacks &callbacks = context->callbacks;

    UdpSocket sock;

    esp_err_t err = sock.open(config.dest_ip, config.port);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open UDP socket");
        vTaskDelete(nullptr);
        return;
    }

    uint8_t rx_buffer[RX_BUFFER_SIZE];
    uint8_t tx_buffer[TX_BUFFER_SIZE];

    while (true) {
        receive_commands(sock, callbacks, rx_buffer, RX_BUFFER_SIZE);
        send_telemetry(sock, callbacks, tx_buffer, TX_BUFFER_SIZE);

        vTaskDelay(pdMS_TO_TICKS(config.period_ms));
    }
}

esp_err_t start_udp_comms_task(const UdpCommsConfig &config, const UdpCommsCallbacks &callbacks)
{
    static UdpCommsTaskContext context;
    static bool task_has_started = false;

    if (task_has_started) return ESP_ERR_INVALID_STATE;

    if (config.dest_ip == nullptr) return ESP_ERR_INVALID_ARG;
    if (config.port == 0) return ESP_ERR_INVALID_ARG;
    if (config.period_ms == 0) return ESP_ERR_INVALID_ARG;

    context.config = config;
    context.callbacks = callbacks;

    BaseType_t result = xTaskCreate(
        udp_comms_task, "udp_comms", config.stack_size, &context, config.priority, nullptr);

    if (result != pdPASS) {
        return ESP_FAIL;
    }

    task_has_started = true;
    return ESP_OK;
}
} // namespace telemetry
