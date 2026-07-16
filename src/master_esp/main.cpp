#include <Arduino.h>

#include <array>
#include <cstring>

#include "comms/uart_link.hpp"

namespace {

comms::UartLink uart_link;

} // namespace

void setup()
{
    Serial.begin(115200);
    delay(1000);

    constexpr comms::UartLink::Config config{
        .port = UART_NUM_1,
        .tx_pin = GPIO_NUM_17,
        .rx_pin = GPIO_NUM_39,
        .baud_rate = 460800,
        .rx_buffer_size = 2048,
        .tx_buffer_size = 1024,
    };

    const esp_err_t init_err = uart_link.init(config);

    if (init_err != ESP_OK) {
        Serial.printf(
            "UART init failed: %s\n",
            esp_err_to_name(init_err)
        );
        return;
    }

    const char sent_message[] =
        "fart";

    Serial.printf(
        "Sent message:     \"%s\"\n",
        sent_message
    );

    uint16_t sent_sequence = 0;

    const esp_err_t send_err = uart_link.send(
        reinterpret_cast<const uint8_t*>(sent_message),
        std::strlen(sent_message),
        sent_sequence
    );

    if (send_err != ESP_OK) {
        Serial.printf(
            "UART send failed: %s\n",
            esp_err_to_name(send_err)
        );
        return;
    }

    std::array<char, 128> received_message{};

    size_t received_size = 0;
    uint16_t received_sequence = 0;

    // Leave one byte free for the terminating '\0'.
    const esp_err_t receive_err = uart_link.receive(
        reinterpret_cast<uint8_t*>(
            received_message.data()
        ),
        received_message.size() - 1,
        received_size,
        received_sequence,
        pdMS_TO_TICKS(1000)
    );

    if (receive_err != ESP_OK) {
        Serial.printf(
            "UART receive failed: %s\n",
            esp_err_to_name(receive_err)
        );
        return;
    }

    received_message[received_size] = '\0';

    Serial.printf(
        "Received message: \"%s\"\n",
        received_message.data()
    );

    Serial.printf(
        "Sent sequence:     %u\n",
        static_cast<unsigned>(sent_sequence)
    );

    Serial.printf(
        "Received sequence: %u\n",
        static_cast<unsigned>(received_sequence)
    );

    const bool message_matches =
        received_size == std::strlen(sent_message) &&
        std::memcmp(
            sent_message,
            received_message.data(),
            received_size
        ) == 0;

    const bool sequence_matches =
        sent_sequence == received_sequence;

    Serial.printf(
        "UART loopback test: %s\n",
        message_matches && sequence_matches
            ? "PASSED"
            : "FAILED"
    );
}

void loop()
{
}
