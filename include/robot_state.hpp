#pragma once

#include "freertos/idf_additions.h"
#include <cstdint>

#define ROBOT_MODE_LIST(X)                                                                         \
    X(ROBOT_IDLE)                                                                                  \
    X(ROBOT_CALIBRATE_METAL)                                                                       \
    X(ROBOT_FOLLOW_TAPE)                                                                           \
    X(ROBOT_FIND_ROCK)                                                                             \
    X(ROBOT_DETECT_METAL)                                                                          \
    X(ROBOT_ESTOP)                                                                                 \
    X(ROBOT_ERROR)

#define ROBOT_FLAG_LIST(X)                                                                         \
    X(ROBOT_FLAG_NONE, 0)                                                                        \
    X(ROBOT_FLAG_FAULT_ACTIVE, 1)                                                                  \
    X(ROBOT_FLAG_DRIVE_ENABLED, 2)                                                                 \
    X(ROBOT_FLAG_METAL_RUNNING, 3)                                                                 \
    X(ROBOT_FLAG_TAPE_ACTIVE, 4)                                                                   \
    X(ROBOT_FLAG_METAL_SEEN, 5)                                                                    \
    X(ROBOT_FLAG_TAPE_SEEN, 6)

#define ROBOT_EVENT_LIST(X)                                                                        \
    X(START)                                                                                       \
    X(STOP)                                                                                      \
    X(METAL_CALIBRATE_START)                                                                       \
    X(METAL_CALIBRATE_DONE)                                                                        \
    X(TAPE_SEEN)                                                                                   \
    X(TAPE_LOST)                                                                                   \
    X(METAL_SEEN)                                                                                  \
    X(METAL_LOST)                                                                                  \
    X(ROCK_FOUND)

enum class RobotMode : uint8_t {
#define X(name) name,
    ROBOT_MODE_LIST(X)
#undef X
};

enum RobotFlag : uint32_t {
#define X(name, bit) name = 1u << bit,
    ROBOT_FLAG_LIST(X)
#undef X
};

enum class RobotEventType : uint8_t {
#define X(name) name,
    ROBOT_EVENT_LIST(X)
#undef X
};

inline constexpr const char *to_string(RobotMode mode)
{
    switch (mode) {
#define X(name)                                                                                    \
    case RobotMode::name:                                                                          \
        return #name;
        ROBOT_MODE_LIST(X)
#undef X
        default:
            return "ROBOT_UNKNOWN";
    }
}

inline constexpr const char *to_string(RobotFlag flag)
{
    switch (flag) {
#define X(name, bit)                                                                               \
    case name:                                                                                     \
        return #name;
        ROBOT_FLAG_LIST(X)
#undef X

        default:
            return "ROBOT_FLAG_UNKNOWN";
    }
}

inline const char *to_string(RobotEventType event)
{
    switch (event) {
#define X(name)                                                                                    \
    case RobotEventType::name:                                                                     \
        return #name;
        ROBOT_EVENT_LIST(X)
#undef X
        default:
            return "UNKNOWN_EVENT";
    }
}

struct RobotState {
    RobotMode mode = RobotMode::ROBOT_IDLE;
    uint32_t flags = 0;
    uint32_t sequence = 0;
};

constexpr inline bool has_flag(uint32_t flags, RobotFlag f) { return (flags & f) != 0; }
constexpr inline void set_flag(uint32_t &flags, RobotFlag f) { flags |= f; }
constexpr inline void clear_flag(uint32_t &flags, RobotFlag f) { flags &= ~f; }
constexpr inline void write_flag(uint32_t &flags, RobotFlag f, bool enabled)
{
    if (enabled) {
        set_flag(flags, f);
    } else {
        clear_flag(flags, f);
    }
}
