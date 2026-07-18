#include "control/pose_estimator.hpp"

namespace control {
const PoseSnapshot &PoseEstimator::update(int x_count,
                                          int y_count,
                                          float imu_heading_rad,
                                          TickType_t tick)
{
    if (!_initialized) {
        _prev_x_count = x_count;
        _prev_y_count = y_count;

        _heading_offset_rad = imu_heading_rad;
        _prev_heading_rad = 0.0f;

        _pose.heading_rad = _prev_heading_rad;
        _pose.x_m = 0.0f;
        _pose.y_m = 0.0f;

        _pose.tick = tick;
        _pose.valid = true;

        _initialized = true;

        return _pose;
    }

    const float heading_rad = wrap_angle_2pi(imu_heading_rad - _heading_offset_rad);
    const float delta_heading_rad = wrap_angle_pi(heading_rad - _prev_heading_rad);

    const int64_t delta_x_count =
        static_cast<int64_t>(x_count) - static_cast<int64_t>(_prev_x_count);

    const int64_t delta_y_count =
        static_cast<int64_t>(y_count) - static_cast<int64_t>(_prev_y_count);

    _prev_x_count = x_count;
    _prev_y_count = y_count;

    const float meas_delta_x_m =
        static_cast<float>(delta_x_count) * _cfg.deadwheel_x_count_to_m_scale;
    const float meas_delta_y_m =
        static_cast<float>(delta_y_count) * _cfg.deadwheel_y_count_to_m_scale;

    const float delta_x_robot_m = meas_delta_x_m + _cfg.deadwheel_x_y_offset_m * delta_heading_rad;
    const float delta_y_robot_m = meas_delta_y_m - _cfg.deadwheel_y_x_offset_m * delta_heading_rad;

    const float midpoint_heading_rad = _prev_heading_rad + 0.5f * delta_heading_rad;

    const float sin_h = sinf(midpoint_heading_rad);
    const float cos_h = cosf(midpoint_heading_rad);

    const float delta_x_field_m = cos_h * delta_x_robot_m - sin_h * delta_y_robot_m;
    const float delta_y_field_m = sin_h * delta_x_robot_m + cos_h * delta_y_robot_m;

    _pose.x_m += delta_x_field_m;
    _pose.y_m += delta_y_field_m;
    _pose.heading_rad = heading_rad;
    _pose.tick = tick;
    _pose.valid = true;

    _prev_heading_rad = heading_rad;

    return _pose;
}

void PoseEstimator::reset(float x_m, float y_m, float heading_rad)
{
    _pose.x_m = x_m;
    _pose.y_m = y_m;
    _pose.heading_rad = heading_rad;
}

} // namespace control
