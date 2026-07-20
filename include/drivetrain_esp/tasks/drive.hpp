#pragma once
#include "Arduino.h"
#include "control/drivetrain.hpp"
#include "portmacro.h"
#include "projdefs.h"

#include "sensors/pcnt_encoder.hpp"

#include "control/pose_estimator.hpp"

namespace DriveTaskConfig {

constexpr uint32_t TASK_STACK_DEPTH = 4096;
constexpr UBaseType_t TASK_PRIORITY = 4;
constexpr BaseType_t TASK_CORE_ID = 1;
constexpr float TASK_PERIOD_MS = 20.0f; // 50 Hz control loop

constexpr gpio_num_t FR_MOTOR_CW_PIN = GPIO_NUM_45;
constexpr gpio_num_t FR_MOTOR_CCW_PIN = GPIO_NUM_46;
constexpr gpio_num_t BR_MOTOR_CW_PIN = GPIO_NUM_42;
constexpr gpio_num_t BR_MOTOR_CCW_PIN = GPIO_NUM_41;
constexpr gpio_num_t FL_MOTOR_CW_PIN = GPIO_NUM_16;
constexpr gpio_num_t FL_MOTOR_CCW_PIN = GPIO_NUM_15;
constexpr gpio_num_t BL_MOTOR_CW_PIN = GPIO_NUM_17;
constexpr gpio_num_t BL_MOTOR_CCW_PIN = GPIO_NUM_18;

constexpr control::Drivetrain::Config DRIVETRAIN_CFG = {
    .timer_0 = nullptr,
    .timer_1 = nullptr,
    .front_right_motor_config{.clockwise_pwm_output = FR_MOTOR_CW_PIN,
                              .c_clockwise_pwm_output = FR_MOTOR_CCW_PIN,
                              .clamp_percentage = 100.0f},
    .back_right_motor_config{.clockwise_pwm_output = BR_MOTOR_CW_PIN,
                             .c_clockwise_pwm_output = BR_MOTOR_CCW_PIN,
                             .clamp_percentage = 100.0f},
    .front_left_motor_config{.clockwise_pwm_output = FL_MOTOR_CW_PIN,
                             .c_clockwise_pwm_output = FL_MOTOR_CCW_PIN,
                             .clamp_percentage = 95.0f},
    .back_left_motor_config{.clockwise_pwm_output = BL_MOTOR_CW_PIN,
                            .c_clockwise_pwm_output = BL_MOTOR_CCW_PIN,
                            .clamp_percentage = 88.0f}};

constexpr sensors::PcntEncoder::Config DEADWHL_X_CFG{
    .gpio_a = GPIO_NUM_21,
    .gpio_b = GPIO_NUM_40,
    .glitch_filter_ns = 1000,
    .invert_direction = false,
};

constexpr sensors::PcntEncoder::Config DEADWHL_Y_CFG{
    .gpio_a = GPIO_NUM_39,
    .gpio_b = GPIO_NUM_38,
    .glitch_filter_ns = 1000,
    .invert_direction = false,
};

constexpr float DEADWHEEL_DIAMETER_M = 0.0508f;
constexpr int32_t COUNTS_PER_REV = 4096;

constexpr control::PoseEstimator::Config POSE_ESTIMATION_CFG{
    .deadwheel_x_count_to_m_scale = PI * DEADWHEEL_DIAMETER_M / static_cast<float>(COUNTS_PER_REV),
    .deadwheel_y_count_to_m_scale = PI * DEADWHEEL_DIAMETER_M / static_cast<float>(COUNTS_PER_REV),
    .deadwheel_x_y_offset_m = 0.0f,
    .deadwheel_y_x_offset_m = 0.0f,
};

constexpr float POS_TOLERANCE_M = 0.02f;
constexpr float HEADING_TOLERANCE_RAD = 0.05f;

constexpr uint32_t IMU_TIMEOUT_MS = 100;
}; // namespace DriveTaskConfig

/// ========================================

enum class DriveMode : uint8_t {
    STOP,
    SET_SPEED, // vx, vy, omega
    TAPE_FOLLOW,
    DRIVE_TO_POSITION, // x, y, theta target
};

struct DriveCommand {
    DriveMode mode;
    uint32_t sequence = 0;

    float x_speed = 0.0f; // strafe velocity
    float y_speed = 0.0f; // forward/backward velocity
    float rot_speed = 0.0f;

    float tape_follow_speed = 0.0f;

    float target_x_m;
    float target_y_m;
    float target_heading_rad; // [-pi, pi]
};

//
esp_err_t send_drive_cmd(const DriveCommand &cmd);
esp_err_t start_drive_task(TaskHandle_t *out_handle);
esp_err_t get_pose(control::PoseSnapshot* out);
bool reached_pose();
