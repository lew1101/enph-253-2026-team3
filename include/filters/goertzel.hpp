#pragma once

#include <cmath>
#include <numbers>

#include "ring_buffer/ring_buffer.hpp"

template <size_t N>
class GoertzelDetector {
    static_assert(N > 0, "Goertzel window size must be greater than zero.");

  private:
    static constexpr float _PI = std::numbers::pi_v<float>;
    static constexpr float _TWO_PI = 2.0f * _PI;

    RingBuffer<float, N> _buf;
    const float _omega;
    const float _coeff;

  public:
    GoertzelDetector(float sample_rate_hz, float target_freq_hz, size_t window_size)
        : _omega{_TWO_PI * target_freq_hz / sample_rate_hz}
        , _coeff{2.0f * std::cosf(_omega)}
    {
    }

    [[nodiscard]] bool add_sample(float sample, float &power_out)
    {
        _buf.push(sample);

        if (!_buf.full()) {
            return false;
        }

        power_out = compute_power();
        return true;
    }

  private:
    [[nodiscard]] float compute_power() const
    {
        float s_prev = 0.0f;
        float s_prev2 = 0.0f;

        for (size_t i = 0; i < N; ++i) {
            const float s = _buf[i] + _coeff * s_prev - s_prev2;

            s_prev2 = s_prev;
            s_prev = s;
        }

        return s_prev * s_prev + s_prev2 * s_prev2 - _coeff * s_prev * s_prev2;
    }

    inline bool valid() { return 0 < _omega && _omega < _PI; }
};
