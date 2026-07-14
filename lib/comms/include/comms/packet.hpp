#pragma once

#include <cstdint>
#include <cstddef>

#include "esp_err.h"

namespace comms {

// packet layout: header | protobuf payload | trailer

constexpr uint16_t PACKET_MAGIC = 0xA55A;

struct __attribute__((packed)) PacketHeader {
    uint16_t magic;
    uint16_t sequence;
    uint16_t payload_length;
};

struct __attribute__((packed)) PacketTrailer {
    uint16_t crc16;
};

constexpr const size_t PACKET_HEADER_SIZE = sizeof(PacketHeader);
constexpr const size_t PACKET_TRAILER_SIZE = sizeof(PacketTrailer);

static_assert(PACKET_HEADER_SIZE == 6);
static_assert(PACKET_TRAILER_SIZE == 2);

esp_err_t make_packet(const uint8_t *payload,
                      size_t payload_size,
                      uint16_t sequence,
                      uint8_t *packet,
                      size_t packet_capacity,
                      size_t &packet_size);

} // namespace comms
