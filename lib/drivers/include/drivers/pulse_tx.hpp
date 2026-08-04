#pragma once

#include "driver/gpio.h"
#include "esp_timer.h"

namespace driver {

class PulseTx {
  public:
    struct Word {
        uint8_t level;
        uint32_t duration_us;
    };

    explicit PulseTx(gpio_num_t gpio) : _gpio(gpio) {}

    esp_err_t init()
    {
        gpio_config_t io_cfg{
            .pin_bit_mask = 1ULL << _gpio,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };

        esp_err_t err = gpio_config(&io_cfg);
        if (err != ESP_OK) return err;

        esp_timer_create_args_t timer_config{
            .callback = _s_timer_callback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "pulse_tx",
            .skip_unhandled_events = true,
        };

        return esp_timer_create(&timer_config, &_timer);
    }

    esp_err_t transmit(const Word *words, size_t count)
    {
        if (words == nullptr || count == 0) return ESP_ERR_INVALID_ARG;
        if (_active) return ESP_ERR_INVALID_STATE;

        _words = words;
        _count = count;
        _index = 0;
        _active = true;

        _output_current_word();
        return ESP_OK;
    }

    template <size_t N>
    inline esp_err_t transmit(const Word (&words)[N])
    {
        return transmit(words, N);
    }

    inline bool active() const { return _active; }

  private:
    static void _s_timer_callback(void *context)
    {
        auto *self = static_cast<PulseTx *>(context);
        self->_advance();
    }

    void _output_current_word()
    {
        const Word &word = _words[_index];
        gpio_set_level(_gpio, word.level);

        if (word.duration_us == 0) {
            _finish();
            return;
        }

        esp_err_t err = esp_timer_start_once(_timer, word.duration_us);
        if (err != ESP_OK) {
            _finish();
        }
    }

    void _advance()
    {
        ++_index;

        if (_index >= _count) {
            _finish();
            return;
        }

        _output_current_word();
    }

    void _finish()
    {
        _active = false;
        _words = nullptr;
        _count = 0;
        _index = 0;
    }

    gpio_num_t _gpio = GPIO_NUM_NC;
    esp_timer_handle_t _timer = nullptr;

    const Word *_words = nullptr;
    size_t _count = 0;
    size_t _index = 0;
    bool _active = false;
};
} // namespace driver
