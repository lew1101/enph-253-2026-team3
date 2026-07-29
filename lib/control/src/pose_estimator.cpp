#include "control/pose_estimator.hpp"

namespace control {
const PoseSnapshot &PoseEstimator::update(int32_t x_count,
                                          int32_t y_count,
                                          float imu_heading_rad,
                                          TickType_t tick)
{
    if (!std::isfinite(imu_heading_rad) || !std::isfinite(_snapshot.pose.x_m) ||
        !std::isfinite(_snapshot.pose.y_m) || !std::isfinite(_snapshot.pose.heading_rad) ||
        !std::isfinite(_cfg.deadwheel_x_count_to_m_scale) ||
        !std::isfinite(_cfg.deadwheel_y_count_to_m_scale) ||
        !std::isfinite(_cfg.deadwheel_x_y_offset_m) ||
        !std::isfinite(_cfg.deadwheel_y_x_offset_m)) {
        _snapshot.tick = tick;
        _snapshot.valid = false;
        return _snapshot;
    }

    if (!_initialized) {
        _prev_x_count = x_count;
        _prev_y_count = y_count;

        _snapshot.pose.heading_rad = wrap_angle_pi(_snapshot.pose.heading_rad);
        _heading_offset_rad = wrap_angle_pi(imu_heading_rad - _snapshot.pose.heading_rad);
        _prev_heading_rad = _snapshot.pose.heading_rad;

        _snapshot.tick = tick;
        _snapshot.valid = true;

        _initialized = true;

        return _snapshot;
    }

    const float heading_rad = wrap_angle_pi(imu_heading_rad - _heading_offset_rad);
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

    // trapezoidal approx:
    // should be good enough for a high enough control update rate
    // dx_meas ~= dx_centre + y_offset * dtheta
    // dy_meas ~= dy_centre - x_offset * dtheta
    // theta_mid ~= theta_prev + 1/2*dtheta
    const float delta_x_robot_m = meas_delta_x_m + _cfg.deadwheel_x_y_offset_m * delta_heading_rad;
    const float delta_y_robot_m = meas_delta_y_m - _cfg.deadwheel_y_x_offset_m * delta_heading_rad;
    const float mid_heading_rad = wrap_angle_pi(_prev_heading_rad + 0.5f * delta_heading_rad);

    float sin_h, cos_h;
    sincosf(mid_heading_rad, &sin_h, &cos_h);

    const float delta_x_field_m = cos_h * delta_x_robot_m - sin_h * delta_y_robot_m;
    const float delta_y_field_m = sin_h * delta_x_robot_m + cos_h * delta_y_robot_m;

    _snapshot.pose.x_m += delta_x_field_m;
    _snapshot.pose.y_m += delta_y_field_m;
    _snapshot.pose.heading_rad = heading_rad;
    _snapshot.tick = tick;
    _snapshot.valid = true;

    _prev_heading_rad = heading_rad;

    return _snapshot;
}

void PoseEstimator::reset(float x_m, float y_m, float heading_rad)
{
    _snapshot.pose.x_m = x_m;
    _snapshot.pose.y_m = y_m;
    _snapshot.pose.heading_rad = wrap_angle_pi(heading_rad);
    _snapshot.valid = false;
    _initialized = false;
}

} // namespace control
