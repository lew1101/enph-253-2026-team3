#pragma once

#include <Arduino.h>
#include <array>

#include "control/pose_estimator.hpp"

using control::Pose;

// IN METERS
#define WAYPOINT_LIST(X)                                                                           \
    X(POSE_HOME, 0.0f, 0.0f, 0.0f)                                                                 \
    X(POSE_ROCK_SCAN_1, 0.1822f, 0.80656f, 0.0f)                                                   \
    X(POSE_ROCK_PICKUP_1_1, 0.09237f, 0.8138f, -15.6f)                                             \
    X(POSE_ROCK_PICKUP_1_2, 0.0324243f, 0.8876759f, -65.4f)                                             \
    X(POSE_ROCK_PICKUP_1_3, 0.1895774f, 0.95653f, -65.4f)                                              \
    X(POSE_ROCK_SCAN_2, 0.1638f, 1.3733f, 0.0f)                                                    \
    X(POSE_ROCK_PICKUP_2_1, 0.1896f, 1.2715f, 0.0f)                                                \
    X(POSE_ROCK_PICKUP_2_2, 0.1680f, 1.3046f, 30.9f)                                               \
    X(POSE_ROCK_PICKUP_2_3, 0.0935f, 1.4204f, 28.9f)                                               \
    X(POSE_ROCK_INTER_23, 0.2378f, 1.6306f, 0.0f)                                                  \
    X(POSE_ROCK_SCAN_3, 0.2349f, 1.7450f, 45.0f)                                                   \
    X(POSE_ROCK_PICKUP_3_1, 0.1653f, 1.6368f, -4.4f)                                               \
    X(POSE_ROCK_PICKUP_3_2, 0.2182f, 1.8785f, -12.4f)                                              \
    X(POSE_ROCK_INTER_34_1, -0.1411f, 1.9427f, 65.0f)                                              \
    X(POSE_ROCK_SCAN_4, -0.2789f, 1.9580f, 100.0f)                                                 \
    X(POSE_ROCK_PICKUP_4_1, -0.2489f, 1.9580f, 100.0f)                                              \
    X(POSE_ROCK_PICKUP_4_2, -0.2374f, 1.9205f, 64.4f)                                              \
    X(POSE_ROCK_PICKUP_4_3, -0.35736f, 1.9833f, 64.4f)                                               \
    X(POSE_ROCK_INTER_45_1, -0.4305f, 1.5521f, 180.0f)                                             \
    X(POSE_ROCK_INTER_45_2, -0.4305f, 0.6295, 180.0f)                                              \
    X(POSE_ROCK_SCAN_5, -0.5464f, 0.2426f, 180.0f)                                                 \
    X(POSE_ROCK_PICKUP_5_1, -0.641ff, 0.2968f, 180.0f)                                              \
    X(POSE_ROCK_PICKUP_5_2, -0.641f, 0.2968f, 219.4f)                                              \
    X(POSE_ROCK_PICKUP_5_3, -0.5446f, 0.3116f, 219.4f)                                             \
    X(POSE_ROCK_PICKUP_5_4, -0.4763f, 0.1859f, 219.4f)                                             \
    X(POSE_ROCK_SCAN_6, -0.7120f, 0.1723f, 90.0f)                                                  \
    X(POSE_ROCK_PICKUP_6_1, -0.557f, 0.1792f, 90.0f)                                               \
    X(POSE_ROCK_PICKUP_6_2, -0.5986f, 0.1822f, 59.9f)                                              \
    X(POSE_ROCK_PICKUP_6_3, -0.7320f, 0.2652f, 59.9f)                                              \
    X(POSE_INTER_ROCK_TOWER, -1.0153f, 0.1675f, 92.0f)                                             \
    X(POSE_TOWER_BUILD_A, -1.4137f, 0.4373f, 123.5f)                                                 \
    X(POSE_TOWER_STACK_A, -1.5310f, 0.6000f, 62.1f)                                                  \
    X(POSE_TOWER_BUILD_B, -1.3904f, 0.4058f, 123.5f)                                                 \
    X(POSE_TOWER_STACK_B, -1.5110f, 0.5200f, 62.1f)                                                  \
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
