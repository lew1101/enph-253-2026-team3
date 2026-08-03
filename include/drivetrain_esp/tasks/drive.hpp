#pragma once
#include "Arduino.h"
#include "drive.pb.h"
#include "portmacro.h"
#include "projdefs.h"

#include "sensors/pcnt_encoder.hpp"

#include "control/drivetrain.hpp"
#include "control/pose_estimator.hpp"

namespace DriveTaskConfig {
constexpr bool LOGGING_ENABLED = true;

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

constexpr float X_PID_STOP_TOLERANCE_M = 0.005f;
constexpr float X_PID_STOP_TOLERANCE_EXIT_M = 0.008f;
constexpr float Y_PID_STOP_TOLERANCE_M = 0.005f;
constexpr float Y_PID_STOP_TOLERANCE_EXIT_M = 0.008f;

constexpr float HEADING_PID_STOP_TOLERANCE_RAD = radians(0.7f);
constexpr float HEADING_PID_STOP_TOLERANCE_EXIT_RAD = radians(1.2f);

constexpr float TARGET_REACHED_X_TOLERANCE_M = 0.03f;
constexpr float TARGET_REACHED_Y_TOLERANCE_M = 0.03f;
constexpr float TARGET_REACHED_HEADING_TOLERANCE_RAD = radians(4.5f);

constexpr float TARGET_SETTLED_MAX_POSITION_ERROR_M = 0.05f;
constexpr float TARGET_SETTLED_MAX_HEADING_ERROR_RAD = radians(5.0f);
constexpr float TARGET_SETTLED_TRANSLATION_DELTA_M = 0.002f;
constexpr float TARGET_SETTLED_HEADING_DELTA_RAD = radians(0.5f);
constexpr uint32_t TARGET_SETTLED_TIME_MS = 500;

// Position-reference lead along a pose path. This approximates the
// aggressiveness of the previous 0.15 m waypoint spacing.
constexpr float DEFAULT_POSE_PATH_LOOKAHEAD_M = 0.15f;

// Physical scaling for the percentage-based UART velocity command. Velocity
// mode integrates a moving pose reference and tracks it with the pose PIDs.
constexpr float VELOCITY_COMMAND_MAX_TRANSLATION_MPS = 0.5f;
constexpr float VELOCITY_COMMAND_MAX_HEADING_RATE_RAD_S = radians(180.0f);
constexpr float VELOCITY_REFERENCE_ADVANCE_TOLERANCE_M = 0.105f;
constexpr float VELOCITY_REFERENCE_ADVANCE_TOLERANCE_RAD = radians(15.0f);

constexpr uint32_t IMU_TIMEOUT_MS = 100;
constexpr uint32_t TAPE_SNAPSHOT_TIMEOUT_MS = 100;
constexpr uint32_t TAPE_SNAPSHOT_WARNING_PERIOD_MS = 1000;
}; // namespace DriveTaskConfig

/// ========================================

//
esp_err_t send_drive_cmd(const robot_DriveCommand &cmd);
esp_err_t start_drive_task(TaskHandle_t *out_handle);
esp_err_t get_pose(control::PoseSnapshot *out);
bool reached_pose();
void clear_reached_pose();
uint32_t get_drive_task_fault();
