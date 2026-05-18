#pragma once

#include "esp_err.h"

#include "comms/tlv_fields.hpp"

#include <cstdint>
#include <cstring>

namespace telemetry {
class TlvPacketWriter {
  public:
    static constexpr uint16_t MAGIC_NUMBER = 0xABCD;
    static constexpr uint8_t VERSION = 1;
    static constexpr size_t HEADER_SIZE = sizeof(TlvPacketHeader);

  private:
    uint8_t *_begin; // Adjust size as needed
    uint8_t *_it;
    uint8_t *_end;

    TlvPacketHeader *_header;
    uint8_t *_payload;
    bool _ok;

  public:
    explicit TlvPacketWriter(uint8_t *buffer, size_t capacity)
        : _begin(buffer)
        , _it(nullptr)
        , _end(nullptr)
        , _header(nullptr)
        , _payload(nullptr)
        , _ok(buffer != nullptr && capacity >= HEADER_SIZE && capacity <= UINT16_MAX)
    {
        if (_ok) {
            _end = _begin + capacity;
            _header = reinterpret_cast<TlvPacketHeader *>(_begin);
            _payload = _begin + HEADER_SIZE;
            _it = _payload;
        }
    }

    esp_err_t begin(uint8_t packet_type, uint16_t packet_seq, uint32_t tick)
    {
        if (!_ok) return ESP_ERR_INVALID_ARG;

        _header->magic = MAGIC_NUMBER;
        _header->version = VERSION;
        _header->packet_type = packet_type;
        _header->packet_len = HEADER_SIZE; // placeholder
        _header->packet_seq = packet_seq;
        _header->tick = tick;

        return ESP_OK;
    }

    esp_err_t finish()
    {
        if (!_ok) return ESP_ERR_INVALID_ARG;
        if (_it > _end) {
            _ok = false;
            return ESP_ERR_INVALID_STATE;
        };
        // should NEVER hit
        _header->packet_len = size(); // add actual packet length to header
        return ESP_OK;
    }

    template <uint8_t TlvType, typename T>
    TlvPacketWriter &add(const TlvField<TlvType, T> &field)
    {
        if (_ok) {
            constexpr uint8_t type = field.type;
            constexpr uint16_t length = field.payload_len;

            write_raw(&type, sizeof(type));
            write_raw(&length, sizeof(length));
            write_raw(&field.payload, sizeof(field.payload));
        }

        return *this;
    }

    template <uint8_t TlvType, typename T>
    inline TlvPacketWriter &add(const T &value)
    {
        return add(TlvField<TlvType, T>{value});
    }

    inline bool ok() const { return _ok; }
    inline size_t size() const { return _it - _begin; } // header + payload
    inline size_t remaining() const { return _end - _it; }

    inline const TlvPacketHeader &header() const { return *_header; }

    inline const uint8_t *begin() const { return _begin; }
    inline const uint8_t *end() const { return _end; }
    inline const uint8_t *payload() const { return _payload; }

    inline size_t payload_size() const { return _it - _payload; }

  private:
    bool write_raw(const void *data, size_t n)
    {
        if (!_ok || data == nullptr) return false;
        if (remaining() < n) {
            _ok = false;
            return false;
        }

        std::memcpy(_it, data, n);
        _it += n;

        return true;
    }
};
} // namespace telemetry
