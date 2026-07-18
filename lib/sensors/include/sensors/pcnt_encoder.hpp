#pragma once

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"

namespace sensors {
class PcntEncoder {
  private:
    static constexpr int kHighLimit = 30000;
    static constexpr int kLowLimit = -30000;

  public:
    struct Config {
        gpio_num_t gpio_a;
        gpio_num_t gpio_b;

        uint32_t glitch_filter_ns = 1000; // Ignore pulses shorter than this.
        // Flip the reported direction without changing wiring.
        bool invert_direction = false;
    };

    PcntEncoder() = default;
    ~PcntEncoder();

    PcntEncoder(const PcntEncoder &) = delete;
    PcntEncoder &operator=(const PcntEncoder &) = delete;

    esp_err_t init(const Config &cfg);
    esp_err_t get_count(int *count) const;
    esp_err_t clear();
    esp_err_t deinit();

  private:
    pcnt_unit_handle_t _unit = nullptr;
    pcnt_channel_handle_t _chan_a = nullptr;
    pcnt_channel_handle_t _chan_b = nullptr;

    int _dir_sign = 1;
    bool _enabled = false;
    bool _initialized = false;
};
} // namespace sensors
