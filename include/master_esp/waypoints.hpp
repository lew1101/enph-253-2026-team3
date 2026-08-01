#pragma once

#include <Arduino.h>
#include <array>

#include "control/pose_estimator.hpp"

using control::Pose;

// IN METERS
#define WAYPOINT_LIST(X)                                                                           \
    X(POSE_HOME, 0.0f, 0.0f, 0.0f)                                                                 \
    X(POSE_ROCK_SCAN_1, 0.1922f, 0.81156f, 0.0f)                                                   \
    X(POSE_ROCK_PICKUP_1_1, 0.05291f, 0.8179f, -18.5f)                                             \
    X(POSE_ROCK_PICKUP_1_2, 0.03494f, 1.0198f, -90.0f)                                             \
    X(POSE_ROCK_PICKUP_1_3, 0.1609f, 1.0255f, -90.0f)                                              \
    X(POSE_ROCK_SCAN_2, 0.1948f, 1.3623f, 0.0f)                                                    \
    X(POSE_ROCK_PICKUP_2_1, 0.1896f, 1.2715f, 0.0f)                                                \
    X(POSE_ROCK_PICKUP_2_2, 0.1680f, 1.3046f, 30.9f)                                               \
    X(POSE_ROCK_PICKUP_2_3, 0.1135f, 1.3904f, 25.9f)                                               \
    X(POSE_ROCK_INTER_23, 0.2378f, 1.6306f, 0.0f)                                                  \
    X(POSE_ROCK_SCAN_3, 0.2969f, 1.6980f, 45.0f)                                                   \
    X(POSE_ROCK_PICKUP_3_1, 0.1853f, 1.6468f, -4.4f)                                               \
    X(POSE_ROCK_PICKUP_3_2, 0.2382f, 1.8785f, -12.4f)                                              \
    X(POSE_ROCK_INTER_34_1, -0.1411f, 1.9427f, 65.0f)                                              \
    X(POSE_ROCK_SCAN_4, -0.2126f, 1.9480f, 100.0f)                                                 \
    X(POSE_ROCK_PICKUP_4_1, -0.2374f, 1.9205f, 64.4f)                                              \
    X(POSE_ROCK_PICKUP_4_2, -0.35736f, 1.9833f, 64.4f)                                               \
    X(POSE_ROCK_INTER_45_1, -0.4305f, 1.5521f, 180.0f)                                             \
    X(POSE_ROCK_INTER_45_2, -0.4305f, 0.6295, 180.0f)                                              \
    X(POSE_ROCK_SCAN_5, -0.5664f, 0.2476f, 180.0f)                                                 \
    X(POSE_ROCK_PICKUP_5_1, -0.641f, 0.2968f, 209.4f)                                              \
    X(POSE_ROCK_PICKUP_5_2, -0.5646f, 0.3116f, 209.4f)                                             \
    X(POSE_ROCK_PICKUP_5_3, -0.4813f, 0.1955f, 209.4f)                                             \
    X(POSE_ROCK_SCAN_6, -0.7105f, 0.1723f, 90.0f)                                                  \
    X(POSE_ROCK_PICKUP_6_1, -0.557f, 0.1792f, 90.0f)                                               \
    X(POSE_ROCK_PICKUP_6_2, -0.5986f, 0.1822f, 59.9f)                                              \
    X(POSE_ROCK_PICKUP_6_3, -0.7320f, 0.2652f, 64.9f)                                              \
    X(POSE_INTER_ROCK_TOWER, -1.0153f, 0.1675f, 92.0f)                                             \
    X(POSE_TOWER_BUILD_A, -1.4137f, 0.4373f, 123.5f)                                                 \
    X(POSE_TOWER_STACK_A, -1.5310f, 0.6000f, 62.1f)                                                  \
    X(POSE_TOWER_BUILD_B, -1.4137f, 0.4373f, 123.5f)                                                 \
    X(POSE_TOWER_STACK_B, -1.5310f, 0.6000f, 62.1f)                                                  \
    X(POSE_SOLAR_ALIGN, -1.1735f, 1.038f, 0.0f)                                                    \
    X(POSE_SOLAR_PULL, -1.1735f, 1.468f, 0.0f)                                                     \
    X(POSE_SOLAR_TURN, -1.1735f, 1.468f, 90.0f)

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
