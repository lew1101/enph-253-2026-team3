#pragma once

#include "portmacro.h"
#include "driver/rmt_tx.h"

#include "esp_err.h"
#include "esp_check.h"

namespace driver {

class RmtTx {
    static constexpr char LOG_TAG[] = "rmt_tx";

  public:
    struct Config {
        gpio_num_t gpio = GPIO_NUM_NC;
        uint32_t resolution_hz = 1'000'000;
        size_t memory_symbols = 64;
        size_t queue_depth = 1;
        uint8_t interrupt_priority = 0;

        bool invert_output = false;
        bool open_drain = false; //
    };

    explicit RmtTx(const Config &config)
        : _cfg(config)
    {
    }

    RmtTx(const RmtTx &) = delete;
    RmtTx &operator=(const RmtTx &) = delete;

    ~RmtTx() { deinit(); }

    esp_err_t init()
    {
        ESP_RETURN_ON_FALSE(!_initialized, ESP_ERR_INVALID_STATE, LOG_TAG, "already initialized");
        ESP_RETURN_ON_FALSE(_cfg.gpio != GPIO_NUM_NC && _cfg.resolution_hz != 0 &&
                                _cfg.memory_symbols != 0 && _cfg.queue_depth != 0,
                            ESP_ERR_INVALID_ARG,
                            LOG_TAG,
                            "invalid rmt tx config args");

        const rmt_tx_channel_config_t channel_config = {
            .gpio_num = _cfg.gpio,
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = _cfg.resolution_hz,
            .mem_block_symbols = _cfg.memory_symbols,
            .trans_queue_depth = _cfg.queue_depth,
            .intr_priority = _cfg.interrupt_priority,
            .flags =
                {
                    .invert_out = _cfg.invert_output,
                    .with_dma = false,
                    .io_loop_back = false,
                    .io_od_mode = _cfg.open_drain,
                    .allow_pd = false,
                },
        };

        ESP_RETURN_ON_ERROR(
            rmt_new_tx_channel(&channel_config, &_channel), LOG_TAG, "failed to start tx channel");

        const rmt_copy_encoder_config_t encoder_config = {};

        esp_err_t err = rmt_new_copy_encoder(&encoder_config, &_encoder);
        if (err != ESP_OK) {
            ESP_LOGE(LOG_TAG, "failed to start copy encoder");

            rmt_del_channel(_channel);
            _channel = nullptr;
            return err;
        }

        err = rmt_enable(_channel);
        if (err != ESP_OK) {
            ESP_LOGE(LOG_TAG, "failed to enable rmt");

            rmt_del_encoder(_encoder);
            rmt_del_channel(_channel);
            _encoder = nullptr;
            _channel = nullptr;
            return err;
        }

        _initialized = true;
        return ESP_OK;
    }

    void deinit()
    {
        if (!_initialized) return;

        rmt_disable(_channel);
        rmt_del_encoder(_encoder);
        rmt_del_channel(_channel);

        _encoder = nullptr;
        _channel = nullptr;
        _initialized = false;
    }

    esp_err_t transmit(const rmt_symbol_word_t *symbols,
                       size_t symbol_count,
                       int loop_count = 0,
                       bool queue_nonblocking = true,
                       bool end_level = false)
    {
        ESP_RETURN_ON_FALSE(_initialized, ESP_ERR_INVALID_STATE, LOG_TAG, "not initialized");
        ESP_RETURN_ON_FALSE(symbols != nullptr && symbol_count != 0,
                            ESP_ERR_INVALID_ARG,
                            LOG_TAG,
                            "invalid rmt transmit call params");

        const rmt_transmit_config_t tx_config = {
            .loop_count = loop_count,
            .flags =
                {
                    .eot_level = end_level,
                    .queue_nonblocking = queue_nonblocking,
                },
        };

        return rmt_transmit(
            _channel, _encoder, symbols, symbol_count * sizeof(rmt_symbol_word_t), &tx_config);
    }

    template <size_t N>
    inline esp_err_t transmit(const rmt_symbol_word_t (&symbols)[N],
                              int loop_count = 0,
                              bool queue_nonblocking = true,
                              bool end_level = false)
    {
        return transmit(symbols, N, loop_count, queue_nonblocking, end_level);
    }

    inline esp_err_t wait(TickType_t timeout = portMAX_DELAY)
    {
        ESP_RETURN_ON_FALSE(_initialized, ESP_ERR_INVALID_STATE, LOG_TAG, "not initialized");
        return rmt_tx_wait_all_done(_channel, timeout);
    }

    inline esp_err_t transmit_and_wait(const rmt_symbol_word_t *symbols,
                                       size_t symbol_count,
                                       int loop_count = 0,
                                       TickType_t timeout = portMAX_DELAY,
                                       bool end_level = false)
    {
        ESP_RETURN_ON_ERROR(transmit(symbols, symbol_count, loop_count, false, end_level),
                            LOG_TAG,
                            "failed to transmit rmt message");
        return wait(timeout);
    }

    inline uint32_t resolution_hz() const { return _cfg.resolution_hz; }
    inline uint32_t us_to_ticks(uint32_t duration_us) const
    {
        const uint64_t ticks =
            static_cast<uint64_t>(duration_us) * _cfg.resolution_hz / 1'000'000ULL;

        return static_cast<uint32_t>(ticks);
    }

  private:
    Config _cfg;

    rmt_channel_handle_t _channel = nullptr;
    rmt_encoder_handle_t _encoder = nullptr;

    bool _initialized = false;
};

} // namespace driver
