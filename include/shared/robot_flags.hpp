#pragma once

#include "freertos/idf_additions.h"
#include <cstdint>

namespace robot_flags {
// DriveUartMessage.flag
enum DriveStatusFlag : uint32_t {
    DRIVE_STATUS_LINK_CONNECTED = 1U << 0,
    DRIVE_STATUS_POSE_VALID = 1U << 1,
    DRIVE_STATUS_TARGET_REACHED = 1U << 2,
    DRIVE_STATUS_ESTOP_LATCHED = 1U << 3,
};

// RobotStateSync.flags
enum RobotControlFlag : EventBits_t {
    CONTROL_DRIVE_ENABLED = 1U << 0,
    CONTROL_TAPE_ENABLED = 1U << 1,
    CONTROL_BEACON_ENABLED = 1U << 2,
    CONTROL_METAL_ENABLED = 1U << 3,
    CONTROL_CAMERA_ENABLE = 1U << 4,
    CONTROL_ESTOP_ACTIVE = 1U << 5,
    CONTROL_CLEAR_DRIVE_FAULT = 1U << 6,
    CONTROL_ACTUATORS = CONTROL_DRIVE_ENABLED | CONTROL_TAPE_ENABLED | CONTROL_BEACON_ENABLED |
                        CONTROL_CAMERA_ENABLE | CONTROL_METAL_ENABLED,
    CONTROL_ALL = CONTROL_DRIVE_ENABLED | CONTROL_TAPE_ENABLED | CONTROL_BEACON_ENABLED |
                  CONTROL_CAMERA_ENABLE | CONTROL_METAL_ENABLED | CONTROL_ESTOP_ACTIVE |
                  CONTROL_CLEAR_DRIVE_FAULT
};

// Master supervisor status event-group bits.
enum RobotStatusFlag : EventBits_t {
    STATUS_ESTOP_ACTIVE = 1U << 0,
    STATUS_FAULT_ACTIVE = 1U << 1,
    STATUS_DRIVE_CONNECTED = 1U << 2,
    STATUS_METAL_1_CALIBRATED = 1U << 3,
    STATUS_TAPE_SEEN = 1U << 4,
    STATUS_BEACON_SEEN = 1U << 5,
    STATUS_METAL_1_SEEN = 1U << 6,
    STATUS_TELETUBBY_SEEN = 1U << 7,
    STATUS_DRIVE_FAULT_ACTIVE = 1U << 8,
    STATUS_METAL_2_CALIBRATED = 1U << 9,
    STATUS_METAL_2_SEEN = 1U << 10,
};

constexpr EventBits_t STATUS_METAL_CALIBRATED_MASK =
    STATUS_METAL_1_CALIBRATED | STATUS_METAL_2_CALIBRATED;
constexpr EventBits_t STATUS_METAL_SEEN_MASK = STATUS_METAL_1_SEEN | STATUS_METAL_2_SEEN;

// Master task-notification bits. These are one-shot notifications, not UART state.
enum RobotNotification : uint32_t {
    NOTIFY_START = 1U << 0,
    NOTIFY_STOP = 1U << 1,
    NOTIFY_RESET = 1U << 2,
    NOTIFY_TAPE_COMPLETE = 1U << 3,
    NOTIFY_BEACON_FOUND = 1U << 4,
    NOTIFY_BEACON_REACHED = 1U << 5,
    NOTIFY_METAL_CALIBRATED = 1U << 6,
    NOTIFY_METAL_FOUND = 1U << 7,
    NOTIFY_FAULT = 1U << 8,
    NOTIFY_DRIVE_TARGET_REACHED = 1U << 9,
};

inline bool has_flag(EventBits_t flags, EventBits_t flag) { return (flags & flag) != 0; }
inline bool has_any_flag(EventBits_t flags, EventBits_t mask) { return (flags & mask) != 0; }
inline bool has_all_flags(EventBits_t flags, EventBits_t mask) { return (flags & mask) == mask; }
} // namespace robot_flags
