#pragma once

#include <cstddef>
#include <cstdint>

#include "pb_decode.h"
#include "pb_encode.h"

#include "esp_err.h"
#include "esp_check.h"

namespace comms::pbcodec {
template <typename Message, const pb_msgdesc_t *Descriptor>
esp_err_t encode(const Message &message, pb_byte_t *buf, size_t buf_cap, size_t *encoded_size)
{
    ESP_RETURN_ON_FALSE(
        buf != nullptr, ESP_ERR_INVALID_ARG, "pb_codec", "buffer cannot be nullptr");
    ESP_RETURN_ON_FALSE(
        encoded_size != nullptr, ESP_ERR_INVALID_ARG, "pb_codec", "encoded_size cannot be nullptr");
    ESP_RETURN_ON_FALSE(
        buf_cap > 0, ESP_ERR_INVALID_ARG, "pb_codec", "output capacity cannot be 0");

    *encoded_size = 0;

    pb_ostream_t ostream = pb_ostream_from_buffer(buf, buf_cap);

    if (!pb_encode(&ostream, Descriptor, &message)) {
        ESP_LOGE("pb_codec", "Protobuf encode failed: %s", PB_GET_ERROR(&ostream));
        return ESP_FAIL;
    }

    *encoded_size = ostream.bytes_written;
    return ESP_OK;
}

template <typename Message, const pb_msgdesc_t *Descriptor>
esp_err_t decode(const pb_byte_t *buf, const size_t msg_len, Message *message_out)
{
    ESP_RETURN_ON_FALSE(
        buf != nullptr, ESP_ERR_INVALID_ARG, "pb_codec", "istream cannot be nullptr");

    *message_out = {};

    pb_istream_t istream = pb_istream_from_buffer(buf, msg_len);

    if (!pb_decode(&istream, Descriptor, message_out)) {
        ESP_LOGE("pb_codec", "Protobuf decode failed: %s", PB_GET_ERROR(&istream));
        *message_out = {};
        return ESP_FAIL;
    }

    return ESP_OK;
}
} // namespace comms::pbcodec
