#pragma once

#include <Arduino.h>
#include <array>

#include "control/pose_estimator.hpp"

using control::Pose;

// IN METERS
#define WAYPOINT_LIST(X)                                                                           \
    X(POSE_HOME, 0.0f, 0.0f, 0.0f)                                                                 \
    X(POSE_ROCK_SCAN_1, 0.1772f, 0.81656f, 0.0f)                                                   \
    X(POSE_ROCK_PICKUP_1_1, 0.0872f, 0.80656f, 0.0f)                                             \
    X(POSE_ROCK_PICKUP_1_2, -0.0124243f, 0.9076759f, -67.4f)                                             \
    X(POSE_ROCK_PICKUP_1_3, 0.1895774f, 0.95653f, -67.4f)                                              \
    X(POSE_ROCK_SCAN_2, 0.1188f, 1.3633f, 0.0f)                                                    \
    X(POSE_ROCK_PICKUP_2_1, 0.1896f, 1.2715f, 0.0f)                                                \
    X(POSE_ROCK_PICKUP_2_2, 0.1680f, 1.3046f, 30.9f)                                               \
    X(POSE_ROCK_PICKUP_2_3, 0.0935f, 1.4204f, 31.4f)                                               \
    X(POSE_ROCK_INTER_23, 0.2378f, 1.6306f, 0.0f)                                                  \
    X(POSE_ROCK_SCAN_3, 0.2149f, 1.7650f, 45.0f)                                                   \
    X(POSE_ROCK_PICKUP_3_1, 0.1653f, 1.6368f, -4.4f)                                               \
    X(POSE_ROCK_PICKUP_3_2, 0.1682f, 1.8785f, -12.4f)                                              \
    X(POSE_ROCK_INTER_34_1, -0.1411f, 1.9427f, 65.0f)                                              \
    X(POSE_ROCK_SCAN_4, -0.2789f, 1.9680f, 90.0f)                                                 \
    X(POSE_ROCK_PICKUP_4_1, -0.1989f, 1.9680f, 90.0f)                                              \
    X(POSE_ROCK_PICKUP_4_2, -0.1989f, 1.9680f, 64.4f)                                              \
    X(POSE_ROCK_PICKUP_4_3, -0.3574, 1.9980f, 180.0f)                                               \
    X(POSE_ROCK_INTER_45_1, -0.4705f, 1.8340f, 180.0f)                                             \
    X(POSE_ROCK_INTER_45_2, -0.4305f, 1.5521f, 180.0f)                                             \
    X(POSE_ROCK_INTER_45_3, -0.4305f, 0.6295, 180.0f)                                              \
    X(POSE_ROCK_SCAN_5, -0.5264f, 0.2376f, 180.0f)                                                 \
    X(POSE_ROCK_PICKUP_5_1, -0.5264f, 0.4076f, 180.0f)                                              \
    X(POSE_ROCK_PICKUP_5_2, -0.3664f, 0.4076f, 180.0f)                                              \
    X(POSE_ROCK_PICKUP_5_3, -0.3664f, 0.2376f, 180.0f)                                             \
    X(POSE_ROCK_SCAN_6, -0.7120f, 0.1723f, 90.0f)                                                  \
    X(POSE_ROCK_PICKUP_6_1, -0.557f, 0.1792f, 90.0f)                                               \
    X(POSE_ROCK_PICKUP_6_2, -0.5986f, 0.1822f, 56.9f)                                              \
    X(POSE_ROCK_PICKUP_6_3, -0.7320f, 0.2652f, 56.9f)                                              \
    X(POSE_INTER_ROCK_TOWER, -1.0153f, 0.1675f, 92.0f)                                             \
    X(POSE_TOWER_BUILD_A, -1.4137f, 0.4373f, 123.5f)                                                 \
    X(POSE_TOWER_STACK_A, -1.5310f, 0.6000f, 62.1f)                                                  \
    X(POSE_TOWER_BUILD_B, -1.3904f, 0.4058f, 124.5f)                                                 \
    X(POSE_TOWER_STACK_B, -1.5110f, 0.5200f, 64.1f)                                                  \
    X(POSE_SOLAR_ALIGN, -1.0035f, 0.538f, 0.0f)                                                    \
    X(POSE_SOLAR_PULL, -1.0035f, 1.468f, 0.0f)                                                     \
    X(POSE_SOLAR_TURN, -1.0035f, 1.468f, 90.0f)


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
