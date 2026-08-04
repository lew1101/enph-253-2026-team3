#pragma once

#include <Arduino.h>
#include <array>

#include "control/pose_estimator.hpp"

using control::Pose;

// IN METERS, (x_pos_m, y_pos_m, heading_deg)
#define WAYPOINT_LIST(X)                                                                           \
    X(POSE_HOME, 0.0f, -0.0f, 0.0f)                                                         \
    X(POSE_ROCK_SCAN_1, -0.1200f, 0.8097f, 0.0f)                                                   \
    X(POSE_ROCK_PICKUP_1_1, -0.2100f, 0.7997f, 0.0f)                                               \
    X(POSE_ROCK_PICKUP_1_2, -0.3096f, 0.9008f, -67.4f)                                             \
    X(POSE_ROCK_PICKUP_1_3, -0.1076f, 0.9497f, -67.4f)                                             \
    X(POSE_ROCK_SCAN_2, -0.1784f, 1.3564f, 0.0f)                                                   \
    X(POSE_ROCK_PICKUP_2_1, -0.1076f, 1.2646f, 0.0f)                                               \
    X(POSE_ROCK_PICKUP_2_2, -0.1292f, 1.2977f, 30.9f)                                              \
    X(POSE_ROCK_PICKUP_2_3, -0.2037f, 1.4135f, 31.4f)                                              \
    X(POSE_ROCK_INTER_23, -0.0594f, 1.6237f, 0.0f)                                                 \
    X(POSE_ROCK_SCAN_3, -0.0823f, 1.7581f, 45.0f)                                                  \
    X(POSE_ROCK_PICKUP_3_1, -0.1319f, 1.6299f, -4.4f)                                              \
    X(POSE_ROCK_PICKUP_3_2, -0.1290f, 1.8716f, -12.4f)                                             \
    X(POSE_ROCK_INTER_34_1, -0.4383f, 1.9358f, 65.0f)                                              \
    X(POSE_ROCK_SCAN_4, -0.5761f, 1.9611f, 90.0f)                                                  \
    X(POSE_ROCK_PICKUP_4_1, -0.4961f, 1.9611f, 90.0f)                                              \
    X(POSE_ROCK_PICKUP_4_2, -0.4961f, 1.9611f, 64.4f)                                              \
    X(POSE_ROCK_PICKUP_4_3, -0.6546f, 1.9911f, 180.0f)                                             \
    X(POSE_ROCK_INTER_45_1, -0.7677f, 1.8271f, 180.0f)                                             \
    X(POSE_ROCK_INTER_45_2, -0.7277f, 1.5452f, 180.0f)                                             \
    X(POSE_ROCK_INTER_45_3, -0.7277f, 0.6226f, 180.0f)                                             \
    X(POSE_ROCK_SCAN_5, -0.8236f, 0.2307f, 180.0f)                                                 \
    X(POSE_ROCK_PICKUP_5_1, -0.8236f, 0.4007f, 180.0f)                                             \
    X(POSE_ROCK_PICKUP_5_2, -0.6636f, 0.4007f, 180.0f)                                             \
    X(POSE_ROCK_PICKUP_5_3, -0.6636f, 0.2307f, 180.0f)                                             \
    X(POSE_ROCK_SCAN_6, -1.0092f, 0.1654f, 90.0f)                                                  \
    X(POSE_ROCK_PICKUP_6_1, -0.8542f, 0.1723f, 90.0f)                                              \
    X(POSE_ROCK_PICKUP_6_2, -0.8958f, 0.1753f, 56.9f)                                              \
    X(POSE_ROCK_PICKUP_6_3, -1.0292f, 0.2583f, 56.9f)                                              \
    X(POSE_INTER_ROCK_TOWER, -1.3125f, 0.1606f, 92.0f)                                             \
    X(POSE_TOWER_BUILD_A, -1.7109f, 0.4304f, 123.5f)                                               \
    X(POSE_TOWER_STACK_A, -1.8282f, 0.5931f, 64.1f)                                                \
    X(POSE_TOWER_BUILD_B, -1.6976f, 0.3889f, 124.5f)                                               \
    X(POSE_TOWER_STACK_B, -1.8082f, 0.5131f, 64.1f)                                                \
    X(POSE_SOLAR_ALIGN, -1.3007f, 0.5311f, 0.0f)                                                   \
    X(POSE_SOLAR_PULL, -1.3007f, 1.4611f, 0.0f)                                                    \
    X(POSE_SOLAR_TURN, -1.3007f, 1.4611f, 90.0f)


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
