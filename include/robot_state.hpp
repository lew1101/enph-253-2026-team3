#pragma once

#include <atomic>

#define ROBOT_STATE_LIST(X)                         \
    X(ROBOT_CALIBRATE, "CALIBRATE")                 \
    X(ROBOT_IDLE, "IDLE")                           \
    X(ROBOT_FOLLOW_TAPE, "FOLLOW_TAPE")             \
    X(ROBOT_FIND_ROCK, "FIND_ROCK")                 \
    X(ROBOT_DETECT_METAL, "DETECT_METAL")           \
    X(ROBOT_ERROR, "ERROR")

enum class RobotState : uint8_t {
#define X(name, str) name,
    ROBOT_STATE_LIST(X)
#undef X
};

extern std::atomic<RobotState> robot_state;

constexpr const char* robot_state_to_string(RobotState state)
{
    switch (state) {
#define X(name, str)                                \
        case RobotState::name:                      \
            return str;
        ROBOT_STATE_LIST(X)
#undef X

        default:
            return "UNKNOWN";
    }
}
