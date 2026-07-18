#pragma once

#include "freertos/idf_additions.h"
#include <math.h>

namespace control {
inline constexpr float wrap_angle_2pi(float angle)
{
    constexpr float two_pi = 2.0f * static_cast<float>(M_PI);

    angle = fmodf(angle, two_pi);
    if (angle < 0.0f) angle += two_pi;
    return angle;
}

inline constexpr float wrap_angle_pi(float angle)
{
    constexpr float pi = static_cast<float>(M_PI);

    return wrap_angle_2pi(angle + pi) - pi;
}

struct PoseSnapshot {
    float x_m = 0.0f;
    float y_m = 0.0f;
    float heading_rad = 0.0f;

    TickType_t tick = 0;
    bool valid = false;
};

class PoseEstimator {
  public:
    struct Config {
        float deadwheel_x_count_to_m_scale = 0.0f;
        float deadwheel_y_count_to_m_scale = 0.0f;

        float deadwheel_x_y_offset_m = 0.0f;
        float deadwheel_y_x_offset_m = 0.0f;
    };

    explicit PoseEstimator(const Config &cfg)
        : _cfg{cfg} {};

    const PoseSnapshot &update(int32_t x_count,
                               int32_t y_count,
                               float imu_heading_rad,
                               TickType_t tick);

    /*
     * Reset position. The next update establishes new encoder and
     * heading references.
     */
    void reset(float x_m = 0.0f, float y_m = 0.0f, float heading_rad = 0.0f);

    inline const PoseSnapshot &pose() const { return _pose; }
    inline bool initialized() const { return _initialized; }

  private:
    Config _cfg;
    PoseSnapshot _pose{};

    int32_t _prev_x_count = 0;
    int32_t _prev_y_count = 0;

    float _prev_heading_rad = 0.0f;
    float _heading_offset_rad = 0.0f;

    bool _initialized = false;
};
} // namespace control
