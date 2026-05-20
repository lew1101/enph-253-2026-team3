#pragma once

#include <Arduino.h>

#include "comms/tlv_fields.hpp"

namespace telemetry {
class TlvPacketParser {
  public:
    static constexpr uint16_t MAGIC_NUMBER = 0xABCD;
    static constexpr uint8_t VERSION = 1;
    static constexpr size_t HEADER_SIZE = sizeof(TlvPacketHeader);

  private:
    const uint8_t *_begin;
    const uint8_t *_it;
    const uint8_t *_end;

    const TlvPacketHeader *_header;
    const uint8_t *_payload;

    bool _ok;

  public:
    explicit TlvPacketParser(const uint8_t *buffer, size_t len)
        : _begin(buffer)
        , _it(nullptr)
        , _end(nullptr)
        , _header(nullptr)
        , _payload(nullptr)
        , _ok(buffer != nullptr && len >= HEADER_SIZE && len <= UINT16_MAX)
    {
        if (_ok) {
            _end = _begin + len;
            _header = reinterpret_cast<const TlvPacketHeader *>(_begin);
            _payload = _begin + HEADER_SIZE;
            _it = _payload;
        }
    }

    template <uint8_t TlvType, typename T>
    esp_err_t read(TlvField<TlvType, T> &out)
    {
        if (!_ok) return ESP_ERR_INVALID_STATE;

        if (out == nullptr) return ESP_ERR_INVALID_ARG;
        out = {};

        if (!read_raw(&out.type, sizeof(out.type)) ||
            !read_raw(&out.payload_len, sizeof(out.payload_len))) {
            _ok = false;
            return ESP_ERR_INVALID_SIZE;
        }

        if (out.type != TlvType) return ESP_ERR_INVALID_RESPONSE;
        if (out.payload_len != sizeof(T)) return ESP_ERR_INVALID_SIZE;

        if (!read_raw(&out.payload, sizeof(out.payload))) {
            _ok = false;
            return ESP_ERR_INVALID_SIZE;
        }

        return ESP_OK;
    }

    template <uint8_t TlvType, typename T>
    inline esp_err_t read(T &out)
    {
        if (!_ok) return ESP_ERR_INVALID_STATE;
        if (out == nullptr) return ESP_ERR_INVALID_ARG;

        TlvField<TlvType, T> field;
        esp_err_t result = read(&field);

        if (result == ESP_OK) {
            *out = field.payload;
        }
        return result;
    }

    template <uint8_t TlvType>
    esp_err_t read_string(char* out, size_t buf_capacity)
    {
        if (!_ok) return ESP_ERR_INVALID_STATE;
        if (out == nullptr) return ESP_ERR_INVALID_ARG;

        TlvType_t type;
        TlvPayloadLen_t payload_len;

        if (!read_raw(&type, sizeof(type)) || !read_raw(&payload_len, sizeof(payload_len))) {
            _ok = false;
            return ESP_ERR_INVALID_SIZE;
        }

        if (type != TlvType) return ESP_ERR_INVALID_RESPONSE;
        if (payload_len > buf_capacity) return ESP_ERR_INVALID_SIZE;

        if (!read_raw(out, payload_len)) {
            _ok = false;
            return ESP_ERR_INVALID_SIZE;
        }

        return ESP_OK;
    }

    esp_err_t skip()
    {
        if (!_ok) return ESP_ERR_INVALID_STATE;

        TlvType_t type;
        TlvPayloadLen_t payload_len;

        if (!read_raw(&type, sizeof(type)) || !read_raw(&payload_len, sizeof(payload_len))) {
            _ok = false;
            return ESP_ERR_INVALID_SIZE;
        }

        if (remaining() < payload_len) {
            _ok = false;
            return ESP_ERR_INVALID_SIZE;
        }

        _it += payload_len;
        return ESP_OK;
    }

    inline uint8_t peek_type() const
    {
        if (!_ok || remaining() < sizeof(uint8_t)) return 0;
        return *_it;
    }

    inline bool ok() const { return _ok; }
    inline size_t remaining() const { return _end - _it; }
    inline bool has_next() const { return _ok && remaining() > 0; }

    inline const TlvPacketHeader &header() const { return *_header; }

    inline const uint8_t *begin() const { return _begin; }
    inline const uint8_t *end() const { return _end; }
    inline const uint8_t *payload() const { return _payload; }

  private:
    bool read_raw(void *out, size_t n)
    {
        if (!_ok || out == nullptr || remaining() < n) return false;
        memcpy(out, _it, n);
        _it += n;
        return true;
    }
};
} // namespace telemetry
