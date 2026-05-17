#include "app_driver/servo.hpp"

#include <algorithm>
#include <cstdint>

namespace driver {
using std::clamp;

// --- Constructor ---

ServoDriver::ServoDriver(const Config &config)
    : _config(config)
{
}

// --- Public API ---

esp_err_t ServoDriver::init()
{
    if (_initialized) return ESP_ERR_INVALID_STATE; // user error
    if (_config.gpio == GPIO_NUM_NC) return ESP_ERR_INVALID_ARG;
    if (_config.min_pulse_us >= _config.max_pulse_us) return ESP_ERR_INVALID_ARG;
    if (_config.freq_hz == 0) return ESP_ERR_INVALID_ARG;
    if (_config.min_pulse_deg >= _config.max_pulse_deg) return ESP_ERR_INVALID_ARG;
    if (_config.min_clamp_deg >= _config.max_clamp_deg) return ESP_ERR_INVALID_ARG;

    if (_config.min_clamp_deg < _config.min_pulse_deg ||
        _config.max_clamp_deg > _config.max_pulse_deg)
        return ESP_ERR_INVALID_ARG;

    if (_config.max_pulse_us > (1000000UL / _config.freq_hz)) return ESP_ERR_INVALID_ARG;

    esp_err_t err;

    if (_config.configure_timer) {
        // configure ledc timer
        ledc_timer_config_t timer_config = {};
        timer_config.speed_mode = _config.speed_mode;
        timer_config.duty_resolution = _config.duty_res;
        timer_config.timer_num = _config.timer;
        timer_config.freq_hz = _config.freq_hz;
        timer_config.clk_cfg = LEDC_AUTO_CLK;

        err = ledc_timer_config(&timer_config);
        if (err != ESP_OK) return err;
    }

    // now configure ledc channel
    ledc_channel_config_t channel_config = {};
    channel_config.gpio_num = _config.gpio;
    channel_config.speed_mode = _config.speed_mode;
    channel_config.channel = _config.channel;
    channel_config.timer_sel = _config.timer;
    channel_config.duty = 0; // off
    channel_config.hpoint = 0;

    err = ledc_channel_config(&channel_config);
    if (err != ESP_OK) return err;

    _initialized = true;

    // set to center position
    err = center();
    if (err != ESP_OK) _initialized = false;

    return err;
}

esp_err_t ServoDriver::set_deg(float deg)
{
    if (!_initialized) return ESP_ERR_INVALID_STATE;

    const uint32_t us = deg_to_us(deg);
    return set_us(us);
}

esp_err_t ServoDriver::set_us(uint32_t us)
{
    if (!_initialized) return ESP_ERR_INVALID_STATE;

    us = clamp(us, _config.min_pulse_us, _config.max_pulse_us);
    const uint32_t duty = us_to_duty(us);

    esp_err_t err = ledc_set_duty(_config.speed_mode, _config.channel, duty);
    if (err != ESP_OK) return err;

    return ledc_update_duty(_config.speed_mode, _config.channel);
}

esp_err_t ServoDriver::center()
{
    if (!_initialized) return ESP_ERR_INVALID_STATE;

    const float center_deg = 0.5f * (_config.min_clamp_deg + _config.max_clamp_deg);
    return set_deg(center_deg);
}

esp_err_t ServoDriver::stop()
{
    if (!_initialized) return ESP_ERR_INVALID_STATE;

    return ledc_stop(_config.speed_mode, _config.channel, 0);
}

// --- Private helper methods ---

uint32_t ServoDriver::deg_to_us(float deg) const
{
    const float scale = static_cast<float>(_config.max_pulse_us - _config.min_pulse_us) /
                        (_config.max_pulse_deg - _config.min_pulse_deg);

    deg = (_config.reversed ? -deg : deg) + _config.bias_deg;
    deg = clamp(deg, _config.min_clamp_deg, _config.max_clamp_deg);

    const float us = _config.min_pulse_us + (deg - _config.min_pulse_deg) * scale;
    return static_cast<uint32_t>(us + 0.5f); // round to nearest microsecond
}

uint32_t ServoDriver::us_to_duty(uint32_t us) const
{
    const float period_us = 1000000.0f / static_cast<float>(_config.freq_hz);
    const float duty_f = static_cast<float>(us) / period_us;

    uint32_t ticks_high =
        static_cast<uint32_t>(duty_f * duty_levels() + 0.5f); // round to nearest tick

    return clamp<uint32_t>(ticks_high, 0UL, max_duty());
}
} // namespace driver
