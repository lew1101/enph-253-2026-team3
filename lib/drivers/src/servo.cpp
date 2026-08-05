#include <Arduino.h>

#include "drivers/servo.hpp"

#include <algorithm>

namespace driver {
using std::clamp;
// --- Constructor ---

ServoDriver::ServoDriver(const Config &config)
    : _config(config)
{
}

// --- Public API ---

bool ServoDriver::init()
{
    if (_initialized) return false; // user error
    if (_config.gpio == GPIO_NUM_NC) return false;
    if (_config.min_pulse_us >= _config.max_pulse_us) return false;
    if (_config.freq_hz == 0) return false;
    if (_config.min_pulse_deg >= _config.max_pulse_deg) return false;
    if (_config.min_clamp_deg >= _config.max_clamp_deg) return false;

    // if (_config.min_clamp_deg < _config.min_pulse_deg ||
    //     _config.max_clamp_deg > _config.max_pulse_deg)
    //     return false;

    if (_config.max_pulse_us > (1000000UL / _config.freq_hz)) return false;

    _initialized = true;
    _current_deg = 0.5f * (_config.min_clamp_deg + _config.max_clamp_deg);

    if (!attach()) {
        _initialized = false;
        return false;
    }

    return true;
}

bool ServoDriver::attach()
{
    if (!_initialized) return false;
    if (_attached) return true;

    if (!ledcAttachChannel(
            _config.gpio, _config.freq_hz, _config.duty_res_bits, _config.channel)) {
        return false;
    }

    _attached = true;
    if (!set_deg(_current_deg)) {
        ledcDetach(_config.gpio);
        _attached = false;
        return false;
    }

    return true;
}

bool ServoDriver::detach()
{
    if (!_initialized) return false;

    bool detached = true;
    if (_attached) {
        detached = ledcDetach(_config.gpio);
    }

    pinMode(_config.gpio, OUTPUT);
    digitalWrite(_config.gpio, LOW);
    _attached = false;
    return detached;
}

bool ServoDriver::set_deg(float deg)
{
    if (!_initialized || !_attached) return false;

    _current_deg = clamp(deg, _config.min_clamp_deg, _config.max_clamp_deg);

    const uint32_t us = deg_to_us(deg);
    return set_us(us);
}

bool ServoDriver::sweep_to_deg(float target_deg, uint32_t duration_ms)
{
    if (!_initialized || !_attached) return false;

    float distance = target_deg - _current_deg;
    if (std::abs(distance) < 0.1f) return true; // Already there

    int num_steps = duration_ms / (1000 / _config.freq_hz);

    if (num_steps <= 0) {
        return set_deg(target_deg); // Just snap to position if duration is too short
    }

    float deg_per_step = distance / num_steps;
    float starting_deg = _current_deg;

    for (int i = 1; i <= num_steps; i++) {
        set_deg(starting_deg + (deg_per_step * i));
        vTaskDelay(pdMS_TO_TICKS(1000 / _config.freq_hz));
    }

    return set_deg(target_deg);
}

bool ServoDriver::set_us(uint32_t us)
{
    if (!_initialized || !_attached) return false;

    us = clamp(us, _config.min_pulse_us, _config.max_pulse_us);
    const uint32_t duty = us_to_duty(us);

    ledcWriteChannel(_config.channel, duty);
    return true;
}

bool ServoDriver::center()
{
    if (!_initialized) return false;

    const float center_deg = 0.5f * (_config.min_clamp_deg + _config.max_clamp_deg);
    return set_deg(center_deg);
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
    // const float period_us = 1000000.0f / static_cast<float>(_config.freq_hz);
    // const float duty_f = static_cast<float>(us) / period_us;

    // uint32_t ticks_high =
    //     static_cast<uint32_t>(duty_f * duty_levels() + 0.5f); // round to nearest tick

    // return clamp<uint32_t>(ticks_high, 0UL, max_duty());

    // better implementation
    const uint64_t numerator = static_cast<uint64_t>(us) * max_duty();
    const uint64_t denominator = 1000000UL / _config.freq_hz;

    // divide towards nearest whole integer
    return static_cast<uint32_t>((numerator + denominator / 2UL) / denominator);
}
} // namespace driver
