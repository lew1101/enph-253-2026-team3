#include "control/pose_estimator.hpp"
#include "freertos/idf_additions.h"

#include "esp_check.h"
#include "esp_err.h"

#include "tasks/drive.hpp"
#include "portmacro.h"
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

namespace {
TaskHandle_t s_task_handle = nullptr;

QueueHandle_t s_drive_cmd_queue = nullptr;
QueueHandle_t s_pose_queue = nullptr;

DriveMode s_mode = DriveMode::STOP;

PcntEncoder s_encoder_x;
PcntEncoder s_encoder_y;

PID tape_pid(15.0f, 0.0f, 1.2f);
PID x_pid(15.0f, 0.0f, 1.2f, 15.0f, -70.0f, 70.0f);
PID y_pid(15.0f, 0.0f, 1.2f, 15.0f, -70.0f, 70.0f);
PID heading_pid(15.0f, 0.0f, 1.2f, 0.4f, -0.5f, 0.5f);

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
    ImuSnapshot imu_snapshot;
    int x_count, y_count;

    ESP_RETURN_ON_FALSE(
        get_imu_snapshot(&imu_snapshot, 0), ESP_FAIL, TAG, "failed to get imu data");
    ESP_RETURN_ON_ERROR(s_encoder_x.get_count(&x_count), TAG, "failed to get x encoder count");
    ESP_RETURN_ON_ERROR(s_encoder_y.get_count(&y_count), TAG, "failed to get y encoder count;");

    *out = pose_estimator.update(x_count, y_count, imu_snapshot.yaw, imu_snapshot.tick);
    xQueueOverwrite(s_pose_queue, out);

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

    PoseSnapshot pose_snapshot;
    TapeSnapshot tape_snapshot;

    DriveCommand cmd;

    TickType_t last_wake_tick = xTaskGetTickCount();
    while (true) {
        esp_err_t err = _update_and_get_pose_estimation(pose_estimator, &pose_snapshot);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "failed to get pose snapshot");
        }

        if (xQueuePeek(s_drive_cmd_queue, &cmd, 0) == pdTRUE) {
            // run drivetrain command
            if (s_mode != cmd.mode) {
                s_mode = cmd.mode;

                switch (s_mode) {
                    case DriveMode::STOP:
                        break;
                    case DriveMode::SET_SPEED:
                        break;
                    case DriveMode::TAPE_FOLLOW:
                        tape_pid.reset();
                        break;
                    case DriveMode::DRIVE_TO_POSITION:
                        x_pid.reset();
                        y_pid.reset();
                        heading_pid.reset();
                        break;
                }
            }

            switch (cmd.mode) {
                case DriveMode::STOP:
                    drivetrain.stop();
                    break;

                case DriveMode::SET_SPEED:
                    drivetrain.move_vector(cmd.x_speed, cmd.y_speed, cmd.rot_speed);
                    break;

                case DriveMode::DRIVE_TO_POSITION: {
                    if (!pose_snapshot.valid) {
                        drivetrain.stop();
                        ESP_LOGE(TAG, "invalid pose");
                        break;
                    }

                    const float error_x_field = cmd.target_x_m - pose_snapshot.x_m;
                    const float error_y_field = cmd.target_y_m - pose_snapshot.y_m;
                    const float position_error = hypotf(error_x_field, error_y_field);

                    const float heading_error =
                        control::wrap_angle_pi(cmd.target_heading_rad - pose_snapshot.heading_rad);

                    const bool position_reached = position_error <= POS_TOLERANCE_M;
                    const bool heading_reached = fabs(heading_error) <= HEADING_TOLERANCE_RAD;

                    if (position_reached && heading_reached) {
                        drivetrain.stop();
                        break;
                    }

                    const float x_cmd_field =
                        position_reached ? 0.0f
                                         : x_pid.update(cmd.target_x_m, pose_snapshot.x_m, DT_S);

                    const float y_cmd_field =
                        position_reached ? 0.0f
                                         : y_pid.update(cmd.target_y_m, pose_snapshot.y_m, DT_S);

                    const float rotation_cmd = heading_reached //
                                                   ? 0.0f
                                                   : heading_pid.update(0.0f, -heading_error, DT_S);

                    const float sin_h = sinf(pose_snapshot.heading_rad);
                    const float cos_h = cosf(pose_snapshot.heading_rad);

                    const float x_cmd_robot = x_cmd_field * cos_h + y_cmd_field * sin_h;
                    const float y_cmd_robot = -x_cmd_field * sin_h + y_cmd_field * cos_h;

                    drivetrain.move_vector(x_cmd_robot, y_cmd_robot, rotation_cmd);
                    break;
                }

                case DriveMode::TAPE_FOLLOW: {
                    bool success = get_tape_snapshot(&tape_snapshot, 0);
                    if (!success) {
                        ESP_LOGW(TAG, "failed to get tape snapshot");
                        break;
                    }

                    float rot_correction = tape_pid.update(0.0f, tape_snapshot.front_err, DT_S);
                    drivetrain.move_vector(0.0f, cmd.tape_follow_speed, rot_correction);
                    break;
                }
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

    s_drive_cmd_queue = xQueueCreate(1, sizeof(DriveCommand));
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

esp_err_t send_drive_cmd(const DriveCommand &cmd)
{
    ESP_RETURN_ON_FALSE(xQueueOverwrite(s_drive_cmd_queue, &cmd) == pdTRUE,
                        ESP_ERR_INVALID_STATE,
                        TAG,
                        "failed to write drive command");
    return ESP_OK;
}
