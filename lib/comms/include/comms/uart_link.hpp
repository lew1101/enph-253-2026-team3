#pragma once

#include <cstdint>
#include <array>

#include "freertos/idf_additions.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "packet.hpp"
#include "portmacro.h"

namespace comms {
class UartLink {
  public:
    struct Config {
        uart_port_t port = UART_NUM_1;

        gpio_num_t tx_pin = GPIO_NUM_NC;
        gpio_num_t rx_pin = GPIO_NUM_NC;

        int baud_rate = 460800;

        int rx_buffer_size = 2048;
        int tx_buffer_size = 1024;
    };

    static constexpr size_t MAX_PAYLOAD_SIZE = 512;
    static constexpr size_t MAX_PACKET_SIZE =
        PACKET_HEADER_SIZE + MAX_PAYLOAD_SIZE + PACKET_TRAILER_SIZE;

    UartLink() = default;

    UartLink(const UartLink &) = delete;
    UartLink &operator=(const UartLink &) = delete;

    esp_err_t init(const Config &config);
    esp_err_t deinit();

    esp_err_t send(const uint8_t *payload, size_t payload_size, uint16_t &sequence);
    esp_err_t receive(uint8_t *payload,
                      size_t payload_capacity,
                      size_t &payload_size,
                      uint16_t &sequence,
                      TickType_t timeout);

    [[nodiscard]] inline bool initialized() const { return _initialized; }
    [[nodiscard]] inline uart_port_t port() const { return _port; }

  private:
    static TickType_t _remaining_timeout(TickType_t start, TickType_t timeout);
    esp_err_t _find_magic(TickType_t start, TickType_t timeout);
    esp_err_t _read_exact(uint8_t *output,
                          size_t required_size,
                          TickType_t start,
                          TickType_t timeout);

  private:
    SemaphoreHandle_t _tx_mutex = nullptr;
    SemaphoreHandle_t _rx_mutex = nullptr;

    uart_port_t _port = UART_NUM_MAX;
    uint16_t _tx_sequence = 0;
    bool _initialized = false;

    std::array<uint8_t, MAX_PACKET_SIZE> _tx_buf{};
    std::array<uint8_t, MAX_PAYLOAD_SIZE> _rx_buf{};
};
} // namespace comms
