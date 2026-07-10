#pragma once

#include "freertos/idf_additions.h"
#include "portmacro.h"

#include <cstdint>

namespace supervisor {

constexpr uint32_t EVENT_QUEUE_LEN = 32;

extern EventGroupHandle_t g_robot_flags;

#define ROBOT_JOB_LIST(X)                                                                        \
    X(ROBOT_IDLE)                                                                                  \
    X(ROBOT_CALIBRATE)                                                                             \
    X(ROBOT_FOLLOW_TAPE)                                                                           \
    X(ROBOT_DRIVE_TO_TARGET)                                                                       \
    X(ROBOT_FIND_ROCK)                                                                             \
    X(ROBOT_DETECT_METAL)                                                                          \
    X(ROBOT_ESTOP)                                                                                 \
    X(ROBOT_ERROR)

#define ROBOT_EVENT_LIST(X)                                                                        \
    X(START_REQUESTED)                                                                             \
    X(STOP_REQUESTED)                                                                              \
    X(RESET_REQUESTED)                                                                             \
    X(TAPE_SEEN)                                                                                   \
    X(TAPE_LOST)                                                                                   \
    X(METAL_SEEN)                                                                                  \
    X(ROCK_FOUND)                                                                                  \
    X(CALIBRATION_DONE)                                                                            \
    X(LIFT_AT_TARGET)                                                                              \
    X(DRIVE_TARGET_REACHED)                                                                        \
    X(DRIVE_FAULT)                                                                                 \
    X(SENSOR_FAULT)                                                                                \
    X(TIMEOUT)

#define ROBOT_FLAG_LIST(X)                                                                         \
    X(ROBOT_FLAG_NONE, 0)                                                                          \
    X(ROBOT_FLAG_FAULT_ACTIVE, 1)                                                                  \
    X(ROBOT_FLAG_DRIVE_ENABLED, 2)                                                                 \
    X(ROBOT_FLAG_METAL_ENABLED, 3)                                                                 \
    X(ROBOT_FLAG_METAL_CALIBRATING, 4)                                                              \
    X(ROBOT_FLAG_METAL_RUNNING, 5)                                                              \
    X(ROBOT_FLAG_TAPE_ACTIVE, 6)                                                                   \
    X(ROBOT_FLAG_METAL_SEEN, 7)                                                                    \
    X(ROBOT_FLAG_TAPE_SEEN, 8)

enum class RobotJob : uint8_t {
#define X(name) name,
    ROBOT_JOB_LIST(X)
#undef X
};

enum class RobotEvent : uint8_t {
#define X(name) name,
    ROBOT_EVENT_LIST(X)
#undef X
};

enum RobotFlag : EventBits_t {
#define X(name, bit) name = EventBits_t{1} << bit,
    ROBOT_FLAG_LIST(X)
#undef X
};

void init();

// Call this every loop to process events and update the robot job
void update();

//
bool send_event(RobotEvent event, TickType_t timeout = 0);
bool send_event_from_isr(RobotEvent event, BaseType_t *higher_priority_task_woken);
bool get_job(RobotJob &out, TickType_t timeout = 0);

inline constexpr const char *to_string(RobotJob mode)
{
    switch (mode) {
#define X(name)                                                                                    \
    case RobotJob::name:                                                                         \
        return #name;
        ROBOT_JOB_LIST(X)
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
} // namespace supervisor
