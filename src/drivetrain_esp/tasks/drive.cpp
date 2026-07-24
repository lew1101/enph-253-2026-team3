#include "drive.pb.h"
#include "freertos/idf_additions.h"
#include <atomic>
#include <cmath>

#include "control/pose_estimator.hpp"

#include "esp_check.h"
#include "esp_err.h"

#include "tasks/drive.hpp"
#include "portmacro.h"
#include "projdefs.h"
#include "control/tape_pid.hpp"
#include "tasks/tape_sense.hpp"
#include "tasks/imu.hpp"
#include "sensors/pcnt_encoder.hpp"
#include "control/pid.hpp"

using namespace DriveTaskConfig;

using control::Drivetrain;
using control::PID;
using control::PoseEstimator;
using control::PoseSnapshot;

using sensors::PcntEncoder;

static constexpr char TAG[] = "drive_task";

control::TapePID tape_pid(16.0f, 0.0f, 450.0f);

namespace {
TaskHandle_t s_task_handle = nullptr;

QueueHandle_t s_drive_cmd_queue = nullptr;
QueueHandle_t s_pose_queue = nullptr;

PcntEncoder s_encoder_x;
PcntEncoder s_encoder_y;

// right strafe is +x, forward is +y, CCW rotation is +\theta
PID s_tape_pid(15.0f, 0.0f, 1.2f);
PID s_x_pid(15.0f, 0.0f, 0.0f, 15.0f, -70.0f, 70.0f);
PID s_y_pid(15.0f, 0.0f, 0.0f, 15.0f, -70.0f, 70.0f);
PID s_heading_pid(15.0f, 0.0f, 1.2f, 0.4f, -0.5f, 0.5f);

std::atomic_bool s_reached_pose{false};

esp_err_t _initialize_deadwheels()
{
    esp_err_t err = s_encoder_x.init(DEADWHL_X_CFG);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "x deadwheel startup failed");
        s_encoder_x.deinit();
        return err;
    }

    err = s_encoder_y.init(DEADWHL_Y_CFG);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "x deadwheel startup failed");
        s_encoder_x.deinit();
        s_encoder_y.deinit();
        return err;
    }

    return ESP_OK;
}

esp_err_t _update_and_get_pose_estimation(PoseEstimator &pose_estimator, PoseSnapshot *out)
{
    static uint32_t last_imu_reset_count = 0;
    static bool have_imu_reset_count = false;

    ImuSnapshot imu_snapshot;
    int x_count, y_count;

    ESP_RETURN_ON_FALSE(
        get_imu_snapshot(&imu_snapshot, 0), ESP_FAIL, TAG, "failed to get imu data");

    const TickType_t now = xTaskGetTickCount();
    const TickType_t max_imu_age = pdMS_TO_TICKS(IMU_TIMEOUT_MS);

    ESP_RETURN_ON_FALSE(imu_snapshot.valid && std::isfinite(imu_snapshot.yaw),
                        ESP_ERR_INVALID_STATE,
                        TAG,
                        "invalid imu data");
    ESP_RETURN_ON_FALSE(
        (now - imu_snapshot.tick) <= max_imu_age, ESP_ERR_TIMEOUT, TAG, "stale imu data");
    ESP_RETURN_ON_ERROR(s_encoder_x.get_count(&x_count), TAG, "failed to get x encoder count");
    ESP_RETURN_ON_ERROR(s_encoder_y.get_count(&y_count), TAG, "failed to get y encoder count;");

    if (have_imu_reset_count && imu_snapshot.reset_count != last_imu_reset_count) {
        const PoseSnapshot &pose = pose_estimator.pose();
        pose_estimator.reset(pose.x_m, pose.y_m, pose.heading_rad);
    }

    last_imu_reset_count = imu_snapshot.reset_count;
    have_imu_reset_count = true;

    *out = pose_estimator.update(x_count, y_count, imu_snapshot.yaw, now);

    return ESP_OK;
}

void _drive_task(void *arg)
{
    (void)arg;
    Drivetrain drivetrain{DRIVETRAIN_CFG};

    esp_err_t err = drivetrain.init();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "drivetrain init failed, deleting task");
        drivetrain.stop();
        s_task_handle = nullptr;
        vTaskDelete(nullptr);
    }

    drivetrain.stop();

    err = _initialize_deadwheels();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "deadwheel init failed, deleting task");
        s_task_handle = nullptr;
        vTaskDelete(nullptr);
    }

    PoseEstimator pose_estimator{POSE_ESTIMATION_CFG};

    const float DT_S = static_cast<float>(TASK_PERIOD_MS) / 1000;

    PoseSnapshot pose_snapshot{};
    TapeSnapshot tape_snapshot;

    float target_x_m = 0.0f;
    float target_y_m = 0.0f;
    float target_heading_rad = 0.0f;
    bool pose_target_valid = false;

    pb_size_t s_command_tag = robot_DriveCommand_stop_tag;

    robot_DriveCommand cmd = robot_DriveCommand_init_zero; // most recent command;
    uint32_t previous_pose_sequence = 0;

    TickType_t last_wake_tick = xTaskGetTickCount();

    while (true) {
        esp_err_t err = _update_and_get_pose_estimation(pose_estimator, &pose_snapshot);
        if (err != ESP_OK) {
            pose_snapshot.valid = false;
            pose_snapshot.tick = xTaskGetTickCount();
            s_reached_pose.store(false, std::memory_order_release);
        }

        xQueueOverwrite(s_pose_queue, &pose_snapshot);

        if (xQueuePeek(s_drive_cmd_queue, &cmd, 0) == pdTRUE) {
            // run drivetrain command
            const bool mode_changed = s_command_tag != cmd.which_command;
            const bool new_pose_command = cmd.which_command == robot_DriveCommand_pose_tag &&
                                          (mode_changed || cmd.sequence != previous_pose_sequence);

            if (mode_changed) {
                s_command_tag = cmd.which_command;

                switch (s_command_tag) {
                    case robot_DriveCommand_stop_tag:
                        break;
                    case robot_DriveCommand_velocity_tag:
                        break;
                    case robot_DriveCommand_tape_follow_tag:
                        s_tape_pid.reset();
                        break;
                    case robot_DriveCommand_pose_tag:
                        break;
                }
            }

            if (new_pose_command) {
                const auto &pose_command = cmd.command.pose;

                // Resolve relative commands once, against the measured pose at receipt.
                if (!pose_command.relative || pose_snapshot.valid) {
                    target_x_m = pose_command.relative ? pose_snapshot.x_m + pose_command.x_m
                                                       : pose_command.x_m;
                    target_y_m = pose_command.relative ? pose_snapshot.y_m + pose_command.y_m
                                                       : pose_command.y_m;
                    target_heading_rad = control::wrap_angle_pi(
                        pose_command.relative
                            ? pose_snapshot.heading_rad + pose_command.theta_rad
                            : pose_command.theta_rad);
                    pose_target_valid = true;

                    s_x_pid.reset();
                    s_y_pid.reset();
                    s_heading_pid.reset();
                    previous_pose_sequence = cmd.sequence;
                    s_reached_pose.store(false, std::memory_order_release);
                } else {
                    pose_target_valid = false;
                }
            }

            switch (cmd.which_command) {
                case robot_DriveCommand_stop_tag:
                    ESP_LOGD(TAG, "drivetrain stop");
                    drivetrain.stop();
                    break;

                case robot_DriveCommand_velocity_tag: {
                    const auto &velocity = cmd.command.velocity;
                    ESP_LOGD(TAG,
                             "moving at speed: x=%.2f, y=%.2f, rot=%.2f",
                             velocity.vx_percent,
                             velocity.vy_percent,
                             velocity.omega_percent);
                    drivetrain.move_vector(
                        velocity.vx_percent, velocity.vy_percent, velocity.omega_percent);
                    break;
                }

                case robot_DriveCommand_pose_tag: {
                    if (!pose_snapshot.valid || !pose_target_valid) {
                        drivetrain.stop();
                        break;
                    }

                    // calculate pos error in field coords
                    const float error_x_field = target_x_m - pose_snapshot.x_m;
                    const float error_y_field = target_y_m - pose_snapshot.y_m;
                    const float position_error = hypotf(error_x_field, error_y_field);

                    // calculate heading error
                    const float heading_error =
                        control::wrap_angle_pi(target_heading_rad - pose_snapshot.heading_rad);

                    // allow for small tolerance in pos/heading error
                    // to avoid jitter near setpoint.
                    const bool position_reached = position_error <= POS_TOLERANCE_M;
                    const bool heading_reached = fabsf(heading_error) <= HEADING_TOLERANCE_RAD;

                    if (position_reached && heading_reached) {
                        drivetrain.stop();
                        s_reached_pose.store(true, std::memory_order_relaxed);
                        break;
                        // position reached... break
                    }

                    s_reached_pose.store(false, std::memory_order_relaxed);

                    // get both sin and cos at the same time
                    float sin_h, cos_h;
                    sincosf(pose_snapshot.heading_rad, &sin_h, &cos_h);

                    float error_x_robot = 0.0f, error_y_robot = 0.0f;
                    if (!position_reached) {
                        error_x_robot = error_x_field * cos_h + error_y_field * sin_h;
                        error_y_robot = -error_x_field * sin_h + error_y_field * cos_h;
                    }
                    // update position pid
                    // negative sign on error necessary for robot to move in the right direction
                    const float x_cmd =
                        position_reached ? 0.0f : s_x_pid.update(0.0f, -error_x_robot, DT_S);
                    const float y_cmd =
                        position_reached ? 0.0f : s_y_pid.update(0.0f, -error_y_robot, DT_S);
                    const float rot_cmd = heading_reached //
                                              ? 0.0f
                                              : s_heading_pid.update(0.0f, -heading_error, DT_S);

                    drivetrain.move_vector(x_cmd, y_cmd, rot_cmd);
                    break;
                }

                case robot_DriveCommand_tape_follow_tag: {
                    auto& m = cmd.command.tape_follow;

                    ESP_LOGD(TAG, "tape following");
                    bool success = get_tape_snapshot(&tape_snapshot, 0);
                    if (!success) {
                        ESP_LOGW(TAG, "failed to get tape snapshot");
                        drivetrain.stop();
                        break;
                    }

                    float correction = tape_pid.update(0.0f, tape_snapshot.front_err, DT_S);
                    float left_speed = m.forward_speed_percent + correction;
                    float right_speed = m.forward_speed_percent - correction;
                    drivetrain.move_rear(left_speed, right_speed);

                    break;
                }
              default:
                break;
            }
        }

        drivetrain.update();
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(TASK_PERIOD_MS));
    }
}
} // namespace

esp_err_t start_drive_task(TaskHandle_t *out_handle)
{
    if (s_task_handle != nullptr) {
        if (out_handle != nullptr) {
            *out_handle = s_task_handle;
        }

        return ESP_ERR_INVALID_STATE;
    }

    s_drive_cmd_queue = xQueueCreate(1, sizeof(robot_DriveCommand));
    configASSERT(s_drive_cmd_queue != nullptr);

    s_pose_queue = xQueueCreate(1, sizeof(PoseSnapshot));
    configASSERT(s_pose_queue != nullptr);

    auto ok = xTaskCreatePinnedToCore(_drive_task,
                                      "drive_task",
                                      TASK_STACK_DEPTH,
                                      nullptr,
                                      TASK_PRIORITY,
                                      &s_task_handle,
                                      TASK_CORE_ID);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to instantiate drive task");

        s_task_handle = nullptr;
        return ESP_FAIL;
    }

    if (out_handle != nullptr) {
        *out_handle = s_task_handle;
    }

    return ESP_OK;
}

esp_err_t send_drive_cmd(const robot_DriveCommand &cmd)
{
    ESP_RETURN_ON_FALSE(
        s_drive_cmd_queue != nullptr, ESP_ERR_INVALID_STATE, TAG, "drive queue not initialized");

    s_reached_pose.store(false, std::memory_order_release);
    ESP_RETURN_ON_FALSE(xQueueOverwrite(s_drive_cmd_queue, &cmd) == pdTRUE,
                        ESP_ERR_INVALID_STATE,
                        TAG,
                        "failed to write drive command");
    return ESP_OK;
}

esp_err_t get_pose(PoseSnapshot *out)
{
    ESP_RETURN_ON_FALSE(out != nullptr, ESP_ERR_INVALID_ARG, TAG, "pose output is null");
    ESP_RETURN_ON_FALSE(
        s_pose_queue != nullptr, ESP_ERR_INVALID_STATE, TAG, "pose queue not initialized!");
    return xQueuePeek(s_pose_queue, out, 0) == pdTRUE ? ESP_OK : ESP_FAIL;
}

bool reached_pose() { return s_reached_pose.load(std::memory_order_relaxed); }
