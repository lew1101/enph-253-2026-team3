#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "esp_err.h"

#include "comms/comms.hpp"
#include "comms/tlv_writer.hpp"
#include "comms/tlv_parser.hpp"

#include "freertos/task.h"

#include <cstdint>

static constexpr const char TAG[] = "udp_comms_task";

static constexpr inline auto get_current_tick = xTaskGetTickCount;

namespace comms {
PacketHandlerResult PacketHandler::handle_received_packet(size_t rx_len)
{
    TlvPacketParser parser(_rx_buf, rx_len);

    if (!parser.ok()) {
        ESP_LOGW(TAG, "Failed to parse packet");
        return PacketHandlerResult{.error = PacketError::ParseFail}; // corrupted packet, skip it
    }

    TlvPacketHeader header = parser.header();

    PacketHandlerResult resp;
    switch (header.packet_type) {
        case PacketType::COMMAND: {
            RobotCommand command{};

            resp = handle_command(parser);
            break;
        }

        case PacketType::PING: {
            resp = handle_ping(parser, header.packet_seq); // mirror the ping sequence number
            break;
        }

        case PacketType::TELEMETRY:
        case PacketType::PONG: {
            break;
        }

        default: {
            ESP_LOGW(TAG, "Unknown packet type: %u", header.packet_type);
            break;
        }
    }

    return resp;
}

PacketHandlerResult PacketHandler::build_telemetry_packet(uint16_t &sequence)
{
    if (_robot_cbs.read_telemetry == nullptr) {
        return PacketHandlerResult{
            .error = PacketError::None,
            .response_type = ResponseType::None //
        };
    }

    RobotTelemetry telemetry{};
    esp_err_t err;

    if (_robot_cbs.handle_command != nullptr) {
        err = _robot_cbs.read_telemetry(telemetry, _robot_cbs.param);

        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Unable to read telemetry from callback");
            return PacketHandlerResult{.error = PacketError::InternalError};
        }
    }

    TlvPacketWriter writer(_tx_buf, _tx_cap);

    err = telemetry.to_tlv(writer, sequence);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Unable to construct telemetry packet");
        return PacketHandlerResult{.error = PacketError::InternalError};
    }

    sequence++; // increment only if everything ok

    return PacketHandlerResult{
        .error = PacketError::None,
        .response_type = ResponseType::Reply,
        .tx_len = writer.size() //
    };
}

PacketHandlerResult PacketHandler::build_pong_packet(uint16_t ping_seq)
{
    size_t tx_len = 0;

    TlvPacketWriter writer(_tx_buf, _tx_cap);

    uint16_t tick = get_current_tick();
    esp_err_t err = writer.begin(PacketType::PONG, ping_seq, tick);

    ESP_LOGI(TAG, "building pong packet");

    if (err != ESP_OK) return PacketHandlerResult{.error = PacketError::InternalError};
    err = writer.finish();
    if (err != ESP_OK) return PacketHandlerResult{.error = PacketError::InternalError};

    tx_len = writer.size();

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Unable to construct PONG packet");
        return PacketHandlerResult{.error = PacketError::InternalError};
    }

    return PacketHandlerResult{
        .error = PacketError::None,
        .response_type = ResponseType::Reply,
        .tx_len = writer.size() //
    };
}

PacketHandlerResult PacketHandler::handle_command(TlvPacketParser &parser)
{
    if (!parser.ok()) return PacketHandlerResult{.error = PacketError::ParseFail};

    TlvPacketHeader header = parser.header();
    // check if header type is correct.
    if (header.packet_type != PacketType::COMMAND)
        return PacketHandlerResult{.error = PacketError::InvalidPacket};

    RobotCommand command{};
    esp_err_t err = command.from_tlv(parser);

    if (err != ESP_OK) {
        return PacketHandlerResult{.error = PacketError::ParseFail};
    }

    if (_robot_cbs.handle_command != nullptr) {
        err = _robot_cbs.handle_command(command, _robot_cbs.param);

        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Command callback failed");
        }
    }

    return PacketHandlerResult{
        .error = PacketError::None,
        .response_type = ResponseType::None,
        .tx_len = 0 //
    };
}

PacketHandlerResult PacketHandler::handle_ping(TlvPacketParser &parser, uint16_t sequence)
{
    if (!parser.ok()) return PacketHandlerResult{.error = PacketError::ParseFail};

    // ESP_LOGI(TAG, "PING");

    TlvPacketHeader header = parser.header();
    // check if header type is correct.
    if (header.packet_type != PacketType::PING)
        return PacketHandlerResult{.error = PacketError::InvalidPacket};
    return build_pong_packet(sequence);
}

} // namespace comms
