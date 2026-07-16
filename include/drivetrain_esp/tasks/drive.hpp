#pragma once
#include "control/drivetrain.hpp"

enum class DriveMode : uint8_t {
    STOP,
    SET_SPEED, // vx, vy, omega
    TAPE_FOLLOW,
    DRIVE_TO_POSITION, // x, y, theta target
};

struct DriveCommand {
    DriveMode mode;

    float x_speed = 0.0f; // strafe velocity
    float y_speed = 0.0f; // forward/backward velocity
    float rot_speed = 0.0f;

    float tape_follow_speed = 0.0f;

    float target_x;
    float target_y;
    float target_theta_rad;
};

struct DriveTaskConfig {
    uint32_t stack_depth = 4096;
    UBaseType_t priority = 4;
    BaseType_t core_id = 1;
    uint32_t period_ms = 20ul; // 50 Hz control loop
};

//
esp_err_t send_drive_cmd(const DriveCommand &cmd);
esp_err_t start_drive_task(const DriveTaskConfig &task_cfg,
                           const control::Drivetrain::Config &drivetrain_cfg,
                           TaskHandle_t *out_handle);
