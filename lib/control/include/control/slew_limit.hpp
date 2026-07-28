#pragma once

#include <algorithm>
#include <cmath>

namespace control {

inline float slew_limit(float target,
                        float current,
                        float max_rate_per_s,
                        float dt_s)
{
    if (dt_s <= 0.0f || max_rate_per_s <= 0.0f) {
        return current;
    }

    const float max_delta = max_rate_per_s * dt_s;
    const float delta = std::clamp(
        target - current,
        -max_delta,
        max_delta
    );

    return current + delta;
}

} // namespace control
