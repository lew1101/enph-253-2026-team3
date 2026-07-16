#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

#include "robot_message.pb.h"

namespace comms::pbcodec {
template <typename Message, const pb_msgdesc_t *Descriptor>
esp_err_t encode(const Message &message, uint8_t *output, size_t output_capacity);

template <typename Message, const pb_msgdesc_t *Descriptor>
esp_err_t decode(const uint8_t *input, size_t input_size, Message* message_out);
} // namespace comms::pbcodec
