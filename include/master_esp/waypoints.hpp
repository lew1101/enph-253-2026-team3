#pragma once

#include <Arduino.h>
#include <array>

#include "control/pose_estimator.hpp"

using control::Pose;

// IN METERS, (x_pos_m, y_pos_m, heading_deg)
// IN METERS, (x_pos_m, y_pos_m, heading_deg)
#define WAYPOINT_LIST(X)                                                                           \
    X(POSE_HOME, 0.0f, 0.0f, 0.0f)                                                           \
    X(POSE_ROCK_SCAN_1, 0.1902f, 0.8366f, 0.0f)                                                    \
    X(POSE_ROCK_PICKUP_1_1, 0.0872f, 0.8066f, 0.0f)                                                \
    X(POSE_ROCK_PICKUP_1_2, -0.0124f, 0.9077f, -67.4f)                                             \
    X(POSE_ROCK_PICKUP_1_3, 0.1896f, 0.9566f, -67.4f)                                              \
    X(POSE_ROCK_SCAN_2, 0.1088f, 1.4233f, 90.0f)                                                   \
    X(POSE_ROCK_PICKUP_2_1, 0.2588f, 1.4433f, 90.0f)                                               \
    X(POSE_ROCK_PICKUP_2_2, 0.2588f, 1.5683f, 90.0f)                                               \
    X(POSE_ROCK_PICKUP_2_3, 0.1088f, 1.5683f, 90.0f)                                               \
    X(POSE_ROCK_INTER_23, 0.2378f, 1.6306f, 0.0f)                                                  \
    X(POSE_ROCK_SCAN_3, 0.2449f, 1.8050f, 45.0f)                                                   \
    X(POSE_ROCK_PICKUP_3_1, 0.1653f, 1.6368f, 20.0f)                                               \
    X(POSE_ROCK_PICKUP_3_2, 0.1652f, 1.8785f, 0.0f)                                              \
    X(POSE_ROCK_INTER_34_1, -0.1411f, 1.9427f, 65.0f)                                              \
    X(POSE_ROCK_SCAN_4, -0.2989f, 1.9480f, 90.0f)                                                  \
    X(POSE_ROCK_PICKUP_4_1, -0.1789f, 1.9680f, 90.0f)                                              \
    X(POSE_ROCK_PICKUP_4_2, -0.1789f, 2.0880f, 90.0f)                                              \
    X(POSE_ROCK_PICKUP_4_3, -0.2989f, 2.0880f, 90.0f)                                              \
    X(POSE_ROCK_INTER_45_1, -0.4705f, 1.8340f, 180.0f)                                             \
    X(POSE_ROCK_INTER_45_2, -0.4305f, 1.5521f, 180.0f)                                             \
    X(POSE_ROCK_INTER_45_3, -0.4305f, 0.6295f, 180.0f)                                             \
    X(POSE_ROCK_SCAN_5, -0.5264f, 0.2376f, 180.0f)                                                 \
    X(POSE_ROCK_PICKUP_5_1, -0.5264f, 0.4076f, 180.0f)                                             \
    X(POSE_ROCK_PICKUP_5_2, -0.3664f, 0.4076f, 180.0f)                                             \
    X(POSE_ROCK_PICKUP_5_3, -0.3664f, 0.2376f, 180.0f)                                             \
    X(POSE_ROCK_SCAN_6, -0.6620f, 0.2423f, 90.0f)                                                  \
    X(POSE_ROCK_PICKUP_6_1, -0.5570f, 0.1792f, 90.0f)                                              \
    X(POSE_ROCK_PICKUP_6_2, -0.5986f, 0.1822f, 56.9f)                                              \
    X(POSE_ROCK_PICKUP_6_3, -0.7320f, 0.2652f, 56.9f)                                              \
    X(POSE_INTER_ROCK_TOWER, -1.0153f, 0.1675f, 92.0f)                                             \
    X(POSE_TOWER_BUILD_A, -1.4137f, 0.4373f, 123.5f)                                               \
    X(POSE_TOWER_STACK_A, -1.5310f, 0.6000f, 64.1f)                                                \
    X(POSE_TOWER_BUILD_B, -1.4004f, 0.3958f, 124.5f)                                               \
    X(POSE_TOWER_STACK_B, -1.5110f, 0.5200f, 64.1f)                                                \
    X(POSE_SOLAR_ALIGN, -1.0035f, 0.5380f, 0.0f)                                                   \
    X(POSE_SOLAR_PULL, -1.0035f, 1.4680f, 0.0f)                                                    \
    X(POSE_SOLAR_TURN, -1.0035f, 1.4680f, 90.0f)



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
