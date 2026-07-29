#pragma once

#include <Arduino.h>
#include <array>

#include "control/pose_estimator.hpp"

using control::Pose;

#define WAYPOINT_LIST(X)                                                                           \
    X(ROCK_1, 0.1272f, 0.8245f, 0.0f)                                                              \
    X(ROCK_2, 0.205f, 0.13963f, 0.0f)                                                              \
    X(ROCK_3, 0.0594f, 0.16843f, radians(23.85f))                                                  \
    X(ROCK_4, -0.0123f, 0.16555f, radians(110.0f))                                                 \
    X(ROCK_5, -0.027f, 0.00295f, radians(180.0f))                                                  \
    X(ROCK_6, -0.06046f, -0.01179f, radians(90.0f))                                                \
    X(TOWER_STACK, -0.14528f, 0.00453f, radians(210.0f))                                                \
    X(TOWER_ASSEM, -0.1503f, 0.01217f, radians(210.0f))                                            \
    X(SOLAR, -0.10891f, 0.11402f, 0.0f)

enum WaypointIndex : std::size_t {
#define X(name, x, y, heading) name,
    WAYPOINT_LIST(X)
#undef X

        COUNT
};

constexpr std::array<Pose, WaypointIndex::COUNT> WAYPOINTS{{
#define X(name, x, y, heading) Pose{.x_m = x, .y_m = y, .heading_rad = heading},
    WAYPOINT_LIST(X)
#undef X
}};
