#pragma once
#include "Arduino.h"
#include "control/drivetrain.hpp"
#include "sensors/pcnt_encoder.hpp"

#include "control/pose_estimator.hpp"

constexpr float DEADWHEEL_DIAMETER_M = 0.0508f;
constexpr int32_t COUNTS_PER_REV = 4096;

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

    float target_x_m;
    float target_y_m;
    float target_heading_rad; // [-pi, pi]
};

struct DriveTaskConfig {
    uint32_t stack_depth = 4096;
    UBaseType_t priority = 4;
    BaseType_t core_id = 1;
    uint32_t period_ms = 20ul; // 50 Hz control loop

    sensors::PcntEncoderConfig deadwheel_x_cfg{
        .gpio_a = GPIO_NUM_21,
        .gpio_b = GPIO_NUM_40,
        .glitch_filter_ns = 1000,
        .invert_direction = false,
    };

    sensors::PcntEncoderConfig deadwheel_y_cfg{
        .gpio_a = GPIO_NUM_39,
        .gpio_b = GPIO_NUM_38,
        .glitch_filter_ns = 1000,
        .invert_direction = false,
    };

    control::PoseEstimator::Config pose_estimator_cfg{
        .deadwheel_x_count_to_m_scale =
            PI * DEADWHEEL_DIAMETER_M / static_cast<float>(COUNTS_PER_REV),
        .deadwheel_y_count_to_m_scale =
            PI * DEADWHEEL_DIAMETER_M / static_cast<float>(COUNTS_PER_REV),
        .deadwheel_x_y_offset_m = 0.0f,
        .deadwheel_y_x_offset_m = 0.0f,
    };

    float position_tolerance_m = 0.02f;
    float heading_tolerance_rad = 0.05f;

    float imu_timeout_ms = 100;
};

//
esp_err_t send_drive_cmd(const DriveCommand &cmd);
esp_err_t start_drive_task(const DriveTaskConfig &task_cfg,
                           const control::Drivetrain::Config &drivetrain_cfg,
                           TaskHandle_t *out_handle);
