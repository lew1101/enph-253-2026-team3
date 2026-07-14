#pragma once

#include <cstdint>
#include <cstddef>

namespace comms {
// crc16_ccitt

constexpr uint16_t CRC16_INITIAL = 0xFFFF;
constexpr uint16_t CRC16_POLYNOMIAL = 0x1021;

inline uint16_t crc16_update(uint16_t crc, const uint8_t *data, size_t size)
{
    for (size_t i = 0; i < size; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;

        for (uint8_t bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000) {
                crc = static_cast<uint16_t>((crc << 1) ^ CRC16_POLYNOMIAL);
            } else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }

    return crc;
}

inline uint16_t crc16(const uint8_t *data, size_t size)
{
    return crc16_update(CRC16_INITIAL, data, size);
}
} // namespace comms
