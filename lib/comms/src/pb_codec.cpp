#include "comms/pb_codec.hpp"

#include "esp_check.h"
#include "pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

static constexpr char TAG[] = "pb_codec";

namespace comms::pbcodec {
template <typename Message, const pb_msgdesc_t* Descriptor>
esp_err_t encode(const Message &message,
                 uint8_t *buf,
                 size_t buf_cap,
                 size_t &encoded_size)
{
    ESP_RETURN_ON_FALSE(buf != nullptr, ESP_ERR_INVALID_ARG, TAG, "buffer cannot be nullptr");
    ESP_RETURN_ON_FALSE(buf_cap > 0, ESP_ERR_INVALID_ARG, TAG, "output capacity cannot be 0");

    encoded_size = 0;

    pb_ostream_t ostream = pb_ostream_from_buffer(buf, buf_cap);

    if (!pb_encode(&ostream, Descriptor, &message)) {
        ESP_LOGE(TAG, "Protobuf encode failed: %s", PB_GET_ERROR(&ostream));
        return ESP_FAIL;
    }

    encoded_size = ostream.bytes_written;
    return ESP_OK;
}

template <typename Message, const pb_msgdesc_t* Descriptor>
esp_err_t decode(const uint8_t *buf, size_t &msg_len, Message &message)
{
    ESP_RETURN_ON_FALSE(buf != nullptr, ESP_ERR_INVALID_ARG, TAG, "istream cannot be nullptr");

    message = {};

    pb_istream_t istream = pb_istream_from_buffer(buf, msg_len);

    if (!pb_decode(&istream, Descriptor, &message)) {
        ESP_LOGE(TAG, "Protobuf decode failed: %s", PB_GET_ERROR(&istream));
        message = {};
        return ESP_FAIL;
    }

    return ESP_OK;
}
} // namespace comms::pbcodec
