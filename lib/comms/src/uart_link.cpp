#include "comms/uart_link.hpp"
#include "comms/crc16.hpp"
#include "comms/packet.hpp"

#include "esp_err.h"
#include "esp_check.h"

static constexpr const char TAG[] = "uart_link";

namespace comms {
namespace {
constexpr uint8_t MAGIC_BYTE_0 = static_cast<uint8_t>(PACKET_MAGIC & 0xFFU);
constexpr uint8_t MAGIC_BYTE_1 = static_cast<uint8_t>((PACKET_MAGIC >> 8U) & 0xFFU);
} // namespace

esp_err_t UartLink::init(const Config &config)
{
    ESP_RETURN_ON_FALSE(
        !_initialized, ESP_ERR_INVALID_STATE, TAG, "UART link is already initialized");

    ESP_RETURN_ON_FALSE(config.port >= UART_NUM_0 && config.port < UART_NUM_MAX,
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "invalid UART port");

    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(config.tx_pin),
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "invalid TX pin: %d",
                        static_cast<int>(config.tx_pin));

    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(config.rx_pin),
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "invalid RX pin: %d",
                        static_cast<int>(config.rx_pin));

    const int fifo_size = UART_HW_FIFO_LEN(config.port);

    ESP_RETURN_ON_FALSE(config.rx_buffer_size > fifo_size,
                        ESP_ERR_INVALID_SIZE,
                        TAG,
                        "RX buffer must exceed UART FIFO size %d",
                        fifo_size);
    ESP_RETURN_ON_FALSE(config.tx_buffer_size == 0 || config.tx_buffer_size > fifo_size,
                        ESP_ERR_INVALID_SIZE,
                        TAG,
                        "TX buffer must either be zero or exceed UART FIFO size %d",
                        fifo_size);

    _tx_mutex = xSemaphoreCreateMutex();
    configASSERT(_tx_mutex != nullptr);

    _rx_mutex = xSemaphoreCreateMutex();
    configASSERT(_rx_mutex != nullptr);

    uart_config_t uart_cfg{
        .baud_rate = config.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(config.port, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin( //
        config.port,
        config.tx_pin,
        config.rx_pin,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE));

    ESP_ERROR_CHECK(uart_driver_install( //
        config.port,
        config.rx_buffer_size,
        config.tx_buffer_size,
        0,
        nullptr,
        0));

    _port = config.port;
    _tx_sequence = 0;
    _initialized = true;

    esp_err_t err = uart_flush_input(_port);
    if (err != ESP_OK) {
        deinit();
        return err;
    }

    return ESP_OK;
}

esp_err_t UartLink::deinit()
{
    if (!_initialized) {
        return ESP_OK;
    }

    _initialized = false;

    const uart_port_t port = _port;

    _port = UART_NUM_MAX;
    _tx_sequence = 0;

    const esp_err_t err = uart_driver_delete(port);

    if (_tx_mutex != nullptr) {
        vSemaphoreDelete(_tx_mutex);
        _tx_mutex = nullptr;
    }

    if (_rx_mutex != nullptr) {
        vSemaphoreDelete(_rx_mutex);
        _rx_mutex = nullptr;
    }

    return err;
}

esp_err_t UartLink::send(const uint8_t *payload, size_t payload_size, uint16_t &sequence)
{
    sequence = 0;

    ESP_RETURN_ON_FALSE(_initialized, ESP_ERR_INVALID_STATE, TAG, "UART link is not initialized");
    ESP_RETURN_ON_FALSE(payload != nullptr, ESP_ERR_INVALID_ARG, TAG, "payload cannot be nullptr");
    ESP_RETURN_ON_FALSE(payload_size > 0 && payload_size <= MAX_PAYLOAD_SIZE,
                        ESP_ERR_INVALID_SIZE,
                        TAG,
                        "invalid payload size: %u",
                        static_cast<unsigned>(payload_size));

    if (xSemaphoreTake(_tx_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    const uint16_t packet_sequence = _tx_sequence;

    size_t packet_size = 0;

    esp_err_t err = make_packet(
        payload, payload_size, packet_sequence, _tx_buf.data(), _tx_buf.size(), packet_size);

    if (err == ESP_OK) {
        const int written = uart_write_bytes(_port, _tx_buf.data(), packet_size);

        if (written < 0) {
            err = ESP_FAIL;
        } else if (static_cast<size_t>(written) != packet_size) {
            err = ESP_ERR_INVALID_SIZE;
        }
    }

    if (err == ESP_OK) {
        sequence = packet_sequence;
        ++_tx_sequence;
    }

    xSemaphoreGive(_tx_mutex);

    return err;
}

esp_err_t UartLink::receive(uint8_t *payload,
                            size_t payload_capacity,
                            size_t &payload_size,
                            uint16_t &sequence,
                            TickType_t timeout)
{
    payload_size = 0;
    sequence = 0;

    ESP_RETURN_ON_FALSE(_initialized, ESP_ERR_INVALID_STATE, TAG, "UART link is not initialized");
    ESP_RETURN_ON_FALSE(payload != nullptr, ESP_ERR_INVALID_ARG, TAG, "payload cannot be nullptr");
    ESP_RETURN_ON_FALSE(
        payload_capacity > 0, ESP_ERR_INVALID_SIZE, TAG, "payload capacity must greater than zero");

    /*
     * Start before acquiring the mutex so the timeout applies
     * to the complete receive call, including mutex waiting.
     */
    const TickType_t start = xTaskGetTickCount();

    if (xSemaphoreTake(_rx_mutex, timeout) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = _find_magic(start, timeout);
    if (err != ESP_OK) goto cleanup;

    {
        PacketHeader header{};
        header.magic = PACKET_MAGIC;

        auto *header_bytes = reinterpret_cast<uint8_t *>(&header);

        // populate
        err = _read_exact(header_bytes + sizeof(header.magic),
                          PACKET_HEADER_SIZE - sizeof(header.magic),
                          start,
                          timeout);
        if (err != ESP_OK) goto cleanup;

        uint16_t crc = crc16_update(CRC16_INITIAL, header_bytes, PACKET_HEADER_SIZE);

        /*
         * The header is not trusted until the CRC is checked, but
         * the length still has to be bounded before using it.
         */
        if (header.payload_length == 0 || header.payload_length > MAX_PAYLOAD_SIZE) {
            ESP_LOGW(TAG,
                     "invalid packet payload length: %u",
                     static_cast<unsigned>(header.payload_length));

            err = ESP_ERR_INVALID_SIZE;
            goto cleanup;
        }

        err = _read_exact(_rx_buf.data(), header.payload_length, start, timeout);
        if (err != ESP_OK) goto cleanup;

        PacketTrailer trailer{};

        err = _read_exact( //
            reinterpret_cast<uint8_t *>(&trailer),
            PACKET_TRAILER_SIZE,
            start,
            timeout);
        if (err != ESP_OK) goto cleanup;

        crc = crc16_update(crc, _rx_buf.data(), header.payload_length);

        if (trailer.crc16 != crc) {
            ESP_LOGW(TAG,
                     "packet CRC mismatch: received=%04X calculated=%04X",
                     static_cast<unsigned>(trailer.crc16),
                     static_cast<unsigned>(crc));

            err = ESP_ERR_INVALID_CRC;
            goto cleanup;
        }

        /*
         * Check caller capacity after consuming and validating the
         * complete packet. This keeps the UART stream aligned even
         * if the caller supplied an undersized buffer.
         */
        if (header.payload_length > payload_capacity) {
            ESP_LOGW(TAG,
                     "payload length %u exceeds capacity %u",
                     static_cast<unsigned>(header.payload_length),
                     static_cast<unsigned>(payload_capacity));

            err = ESP_ERR_INVALID_SIZE;
            goto cleanup;
        }

        memcpy(payload, _rx_buf.data(), header.payload_length);

        payload_size = header.payload_length;
        sequence = header.sequence;
    }

cleanup:
    xSemaphoreGive(_rx_mutex);
    return err;
}

esp_err_t UartLink::_read_exact(uint8_t *output,
                                size_t required_size,
                                TickType_t start,
                                TickType_t timeout)
{
    if (required_size == 0) return ESP_OK;
    if (output == nullptr) return ESP_ERR_INVALID_ARG;

    size_t total_received = 0;

    while (total_received < required_size) {
        const TickType_t remaining = _remaining_timeout(start, timeout);

        /*
         * Call uart_read_bytes() even when remaining == 0.
         * This allows a nonblocking receive to consume bytes
         * that are already buffered.
         */
        const int received = uart_read_bytes(_port,
                                             output + total_received,
                                             static_cast<uint32_t>(required_size - total_received),
                                             remaining);

        if (received < 0) {
            return ESP_FAIL;
        } else if (received == 0) {
            return ESP_ERR_TIMEOUT;
        }

        total_received += static_cast<size_t>(received);
    }

    return ESP_OK;
}

esp_err_t UartLink::_find_magic(TickType_t start, TickType_t timeout)
{
    bool first_byte_seen = false;

    while (true) {
        uint8_t byte = 0;

        const esp_err_t err = _read_exact(&byte, 1, start, timeout);
        if (err != ESP_OK) return err;

        if (!first_byte_seen) {
            first_byte_seen = byte == MAGIC_BYTE_0;

            continue;
        }

        if (byte == MAGIC_BYTE_1) {
            return ESP_OK;
        }

        first_byte_seen = byte == MAGIC_BYTE_0;
    }
}

TickType_t UartLink::_remaining_timeout(TickType_t start, TickType_t timeout)
{
    if (timeout == portMAX_DELAY) return portMAX_DELAY;

    const TickType_t elapsed = xTaskGetTickCount() - start;

    if (elapsed >= timeout) return 0;
    return timeout - elapsed;
}

} // namespace comms
