#pragma once

#include <cstddef>

template <typename T, size_t N>
class RingBuffer {
    static_assert(N > 0, "Goertzel window size must be greater than zero.");

  private:
    T _buf[N]{};
    size_t _write_idx = 0;
    size_t _count = 0;

  public:
    void push(T sample)
    {
        _buf[_write_idx] = sample;
        _write_idx = (_write_idx + 1) % N;

        if (_count < N) {
            ++_count;
        }
    }

    [[nodiscard]] inline const T &at(size_t i) const
    {
        const size_t start = full() ? _write_idx : 0;
        return _buf[(start + i) % N];
    }

    [[nodiscard]] inline const T &reverse_at(size_t i) const
    {
        return (_write_idx + N - 1 - i) % N;
    }

    [[nodiscard]] inline const T &operator[](size_t i) const { return at(i); }

    [[nodiscard]] inline bool full() const { return _count == N; }
    [[nodiscard]] inline size_t size() const { return _count; }
};
