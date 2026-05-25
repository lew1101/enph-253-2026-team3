#pragma once

#include <Arduino.h>

#include <type_traits>

namespace comms {
struct __attribute__((packed)) TlvPacketHeader {
    uint16_t magic; //
    uint8_t version;
    uint8_t packet_type; // Telemetry type (e.g., 0 for sensor data, 1 for status update, etc.)
    uint16_t packet_len; // length of the total packet (including header) in bytes
    uint16_t packet_seq;
    uint32_t tick; // system tick at the time of packet creation
};

template <typename T>
struct IsTlvValue : std::false_type {};

template <>
struct IsTlvValue<uint8_t> : std::true_type {};

template <>
struct IsTlvValue<uint16_t> : std::true_type {};

template <>
struct IsTlvValue<uint32_t> : std::true_type {};

template <>
struct IsTlvValue<int32_t> : std::true_type {};

template <>
struct IsTlvValue<float> : std::true_type {};

using TlvType_t = uint8_t;
using TlvPayloadLen_t = uint16_t;

template <typename T>
struct TlvField {
    static_assert(IsTlvValue<T>::value, "Unsupported TLV value type");

    static constexpr TlvPayloadLen_t payload_len = sizeof(T);

    TlvType_t type;
    T payload;
};

struct TlvStringField {
    TlvType_t type;
    const TlvPayloadLen_t payload_len;
    const char *payload_buf;
};
} // namespace comms
