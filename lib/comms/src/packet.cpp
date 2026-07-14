#include "comms/packet.hpp"
#include "comms/crc16.hpp"

#include "esp_err.h"
#include "esp_check.h"

constexpr const char TAG[] = "packet";

namespace comms {
esp_err_t make_packet(const uint8_t *payload,
                      size_t payload_size,
                      uint16_t sequence,
                      uint8_t *packet,
                      size_t packet_capacity,
                      size_t &packet_size)
{
    packet_size = 0;

    ESP_RETURN_ON_FALSE(payload != nullptr, ESP_ERR_INVALID_ARG, TAG, "payload cannot be nullptr");

    ESP_RETURN_ON_FALSE(
        packet != nullptr, ESP_ERR_INVALID_ARG, TAG, "packet buffer cannot be nullptr");

    ESP_RETURN_ON_FALSE(payload_size > 0 && payload_size <= UINT16_MAX,
                        ESP_ERR_INVALID_SIZE,
                        TAG,
                        "invalid payload size");

    const size_t required_size = PACKET_HEADER_SIZE + payload_size + PACKET_TRAILER_SIZE;

    ESP_RETURN_ON_FALSE(packet_capacity >= required_size,
                        ESP_ERR_INVALID_SIZE,
                        TAG,
                        "packet buffer too small: need %u, have %u",
                        static_cast<unsigned>(required_size),
                        static_cast<unsigned>(packet_capacity));

    const PacketHeader header{
        .magic = PACKET_MAGIC,
        .sequence = sequence,
        .payload_length = static_cast<uint16_t>(payload_size),
    };

    memcpy(packet, &header, PACKET_HEADER_SIZE);
    memcpy(packet + PACKET_HEADER_SIZE, payload, payload_size);

    const size_t trailer_offset = PACKET_HEADER_SIZE + payload_size;

    const PacketTrailer trailer{
        .crc16 = crc16(packet, trailer_offset),
    };

    memcpy(packet + trailer_offset, &trailer, PACKET_TRAILER_SIZE);

    packet_size = required_size;
    return ESP_OK;
}
} // namespace comms
