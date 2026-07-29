#pragma once
#include "Arduino.h"
#include "drive.pb.h"
#include "portmacro.h"
#include "projdefs.h"

#include "sensors/pcnt_encoder.hpp"

#include "control/drivetrain.hpp"
#include "control/tape_pid.hpp"
#include "control/pid.hpp"
#include "control/pose_estimator.hpp"

extern control::TapePID tape_pid;
extern control::PID x_pid;
extern control::PID y_pid;
extern control::PID heading_pid;

namespace DriveTaskConfig {

constexpr uint32_t TASK_STACK_DEPTH = 4096;
constexpr UBaseType_t TASK_PRIORITY = 4;
constexpr BaseType_t TASK_CORE_ID = 1;
constexpr float TASK_PERIOD_MS = 10.0f; // 100 Hz control loop

constexpr gpio_num_t FR_MOTOR_CW_PIN = GPIO_NUM_45;
constexpr gpio_num_t FR_MOTOR_CCW_PIN = GPIO_NUM_46;
constexpr gpio_num_t BR_MOTOR_CW_PIN = GPIO_NUM_41;
constexpr gpio_num_t BR_MOTOR_CCW_PIN = GPIO_NUM_42;
constexpr gpio_num_t FL_MOTOR_CW_PIN = GPIO_NUM_17;
constexpr gpio_num_t FL_MOTOR_CCW_PIN = GPIO_NUM_18;
constexpr gpio_num_t BL_MOTOR_CW_PIN = GPIO_NUM_15;
constexpr gpio_num_t BL_MOTOR_CCW_PIN = GPIO_NUM_16;

constexpr uint32_t DEAD_TIME_TICKS = 10ul; // ~10us
constexpr float DEAD_BAND_PERCENTAGE = 0.8f;

constexpr control::Drivetrain::Config DRIVETRAIN_CFG = {
    .front_right_motor_config{
        .clockwise_pwm_output = FR_MOTOR_CW_PIN,
        .c_clockwise_pwm_output = FR_MOTOR_CCW_PIN,
        .dead_time_ticks = DEAD_TIME_TICKS,
        .bias_percentage = 13.0f,
        .output_scale = 1.08f,
        .deadband_percentage = DEAD_BAND_PERCENTAGE,
        .max_duty_percentage = 100.0f,
    },

    .back_right_motor_config{
        .clockwise_pwm_output = BR_MOTOR_CW_PIN,
        .c_clockwise_pwm_output = BR_MOTOR_CCW_PIN,
        .dead_time_ticks = DEAD_TIME_TICKS,
        .bias_percentage = 13.5f,
        .output_scale = 1.1f,
        .deadband_percentage = DEAD_BAND_PERCENTAGE,
        .max_duty_percentage = 100.0f,
    },

    .front_left_motor_config{
        .clockwise_pwm_output = FL_MOTOR_CW_PIN,
        .c_clockwise_pwm_output = FL_MOTOR_CCW_PIN,
        .dead_time_ticks = DEAD_TIME_TICKS,
        .bias_percentage = 11.0f,
        .output_scale = 1.15f,
        .deadband_percentage = DEAD_BAND_PERCENTAGE,
        .max_duty_percentage = 100.0f,

    },

    .back_left_motor_config{
        .clockwise_pwm_output = BL_MOTOR_CW_PIN,
        .c_clockwise_pwm_output = BL_MOTOR_CCW_PIN,
        .dead_time_ticks = DEAD_TIME_TICKS,
        .bias_percentage = 11.5f,
        .output_scale = 1.1f,
        .deadband_percentage = DEAD_BAND_PERCENTAGE,
        .max_duty_percentage = 100.0f,
    },
};

constexpr sensors::PcntEncoder::Config DEADWHL_X_CFG{
    .gpio_a = GPIO_NUM_21,
    .gpio_b = GPIO_NUM_40,
    .glitch_filter_ns = 1000,
    .invert_direction = false,
};

constexpr sensors::PcntEncoder::Config DEADWHL_Y_CFG{
    .gpio_a = GPIO_NUM_38,
    .gpio_b = GPIO_NUM_47,
    .glitch_filter_ns = 1000,
    .invert_direction = true,
};

constexpr float DEADWHEEL_DIAMETER_M = 0.0505f;
constexpr int32_t COUNTS_PER_REV = 4096;

constexpr control::PoseEstimator::Config POSE_ESTIMATION_CFG{
    .deadwheel_x_count_to_m_scale = PI * DEADWHEEL_DIAMETER_M / static_cast<float>(COUNTS_PER_REV),
    .deadwheel_y_count_to_m_scale = PI * DEADWHEEL_DIAMETER_M / static_cast<float>(COUNTS_PER_REV),
    .deadwheel_x_y_offset_m = 0.2444f / 1000.0f,
    .deadwheel_y_x_offset_m = 42.62525f / 1000.0f,
};

constexpr float X_TOLERANCE_M = 0.008f;
constexpr float X_TOLERANCE_EXIT_M = 0.011f;
constexpr float Y_TOLERANCE_M = 0.008f;
constexpr float Y_TOLERANCE_EXIT_M = 0.011f;

constexpr float HEADING_TOLERANCE_RAD = radians(1.0f);
constexpr float HEADING_TOLERANCE_EXIT_RAD = radians(1.5f);

constexpr float POSE_PATH_WAYPOINT_SPACING_M = 0.15f;
constexpr float POSE_PATH_LOOKAHEAD_TOLERANCE_M = 0.7f * POSE_PATH_WAYPOINT_SPACING_M;

constexpr uint32_t IMU_TIMEOUT_MS = 100;
}; // namespace DriveTaskConfig

/// ========================================

//
esp_err_t send_drive_cmd(const robot_DriveCommand &cmd);
esp_err_t start_drive_task(TaskHandle_t *out_handle);
esp_err_t get_pose(control::PoseSnapshot *out);
bool reached_pose();
