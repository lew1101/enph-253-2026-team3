#pragma once

#include <Arduino.h>
#include <array>

#include "control/pose_estimator.hpp"

using control::Pose;

// IN METERS
#define WAYPOINT_LIST(X)                                                                           \
    X(POSE_HOME, 0.0f, 0.0f, 0.0f)                                                                 \
    X(POSE_ROCK_SCAN_1, 0.1172f, 0.84156f, 0.0f)                                                   \
    X(POSE_ROCK_PICKUP_1, 0.0f, 0.0f, 0.0f)                                                                 \
    X(POSE_ROCK_SCAN_2, 0.1948f, 1.3323f, 0.0f)                                                    \
    X(POSE_ROCK_PICKUP_2, 0.0f, 0.0f, 0.0f)                                                                 \
    X(POSE_ROCK_INTER_23, 0.2378f, 1.6306f, 0.0f)                                                  \
    X(POSE_ROCK_SCAN_3, 0.1869f, 1.7180f, 45.0f)                                                   \
    X(POSE_ROCK_PICKUP_3, 0.0f, 0.0f, 0.0f)                                                                 \
    X(POSE_ROCK_INTER_34_1, -0.1411f, 1.9427f, 65.0f)                                              \
    X(POSE_ROCK_SCAN_4, -0.2526f, 1.9280f, 100.0f)                                                 \
    X(POSE_ROCK_PICKUP_4, 0.0f, 0.0f, 0.0f)                                                                 \
    X(POSE_ROCK_INTER_45_1, -0.4305f, 1.5521f, 180.0f)                                             \
    X(POSE_ROCK_INTER_45_2, -0.4305f, 0.6295, 180.0f)                                              \
    X(POSE_ROCK_SCAN_5, -0.5664f, 0.2476f, 180.0f)                                                 \
    X(POSE_ROCK_PICKUP_5, 0.0f, 0.0f, 0.0f)                                                                 \
    X(POSE_ROCK_SCAN_6, -0.7105f, 0.1723f, 90.0f)                                                  \
    X(POSE_ROCK_PICKUP_6, 0.0f, 0.0f, 0.0f)                                                                 \
    X(POSE_INTER_ROCK_TOWER, -1.0153f, 0.1675f, 92.0f)                                             \
    X(POSE_TOWER_BUILD, -1.4137f, 0.4173f, 123.5f)                                                 \
    X(POSE_TOWER_STACK, -1.5310f, 0.5400f, 62.1f)                                                  \
    X(POSE_SOLAR_ALIGN, -1.1735f, 1.038f, 0.0f)                                                    \
    X(POSE_SOLAR_PULL, -1.1735f, 1.168f, 0.0f)

enum WaypointIndex : std::size_t {
#define X(name, x, y, heading) name,
    WAYPOINT_LIST(X)
#undef X

        COUNT
};

constexpr std::array<Pose, WaypointIndex::COUNT> WAYPOINTS{{
#define X(name, x, y, heading) Pose{.x_m = x, .y_m = y, .heading_rad = radians(heading)},
    WAYPOINT_LIST(X)
#undef X
}};
