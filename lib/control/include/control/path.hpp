#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

#include "pose_estimator.hpp"

namespace control {

inline Pose lerp_pose(const Pose &start, const Pose &end, float t)
{
    t = std::clamp(t, 0.0f, 1.0f);

    const float heading_delta = wrap_angle_pi(end.heading_rad - start.heading_rad);

    return {
        .x_m = std::lerp(start.x_m, end.x_m, t),
        .y_m = std::lerp(start.y_m, end.y_m, t),
        .heading_rad = wrap_angle_pi(start.heading_rad + t * heading_delta),
    };
}

inline std::optional<Pose> get_next_lerp_pose(const Pose &start,
                                              const Pose &end,
                                              float spacing_m,
                                              uint32_t count)
{
    const float dx = end.x_m - start.x_m;
    const float dy = end.y_m - start.y_m;
    const float distance_m = std::hypot(dx, dy);

    if (spacing_m <= 0.0f) return std::nullopt;

    const uint32_t segment_count =
        std::max<uint32_t>(1, static_cast<uint32_t>(std::ceil(distance_m / spacing_m)));

    // count = 0 returns start; count = segment_count returns end.
    if (count > segment_count) return std::nullopt;

    const float t = static_cast<float>(count) / static_cast<float>(segment_count);
    return lerp_pose(start, end, t);
}

} // namespace control
