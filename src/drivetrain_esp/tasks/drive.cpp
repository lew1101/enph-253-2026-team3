#include "drive.pb.h"
#include "freertos/idf_additions.h"
#include <cmath>

#include "control/pose_estimator.hpp"
#include "control/drive_controller.hpp"

#include "esp_check.h"
#include "esp_err.h"

#include "tasks/drive.hpp"
#include "tasks/imu.hpp"
#include "sensors/pcnt_encoder.hpp"

using namespace DriveTaskConfig;

using control::Drivetrain;
using control::PoseEstimator;
using control::PoseSnapshot;

using sensors::PcntEncoder;

static constexpr char TAG[] = "drive_task";

namespace {
TaskHandle_t s_task_handle = nullptr;

QueueHandle_t s_drive_cmd_queue = nullptr;
QueueHandle_t s_pose_queue = nullptr;

PcntEncoder s_encoder_x;
PcntEncoder s_encoder_y;

DriveController s_drive_controller;

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
        const PoseSnapshot &snapshot = pose_estimator.pose();
        pose_estimator.reset(snapshot.pose.x_m, snapshot.pose.y_m, snapshot.pose.heading_rad);
    }

    last_imu_reset_count = imu_snapshot.reset_count;
    have_imu_reset_count = true;

    if (LOGGING_ENABLED) {
        Serial.printf(">x_cnt:%d\n"
                      ">ycnt:%d\n",
                      x_count,
                      y_count);
    }

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

    ESP_LOGI(TAG, "sucessfully initialized deadwheels");

    delay(1000);

    PoseEstimator pose_estimator{POSE_ESTIMATION_CFG};
    const float dt_s = static_cast<float>(TASK_PERIOD_MS) / 1000.0f;

    PoseSnapshot pose_snapshot{};
    robot_DriveCommand cmd = robot_DriveCommand_init_zero;
    TickType_t last_wake_tick = xTaskGetTickCount();

    while (true) {
        const esp_err_t pose_err = _update_and_get_pose_estimation(pose_estimator, &pose_snapshot);

        if (pose_err != ESP_OK) {
            pose_snapshot.valid = false;
            pose_snapshot.tick = xTaskGetTickCount();
            s_drive_controller.clear_reached_pose();
        }

        xQueueOverwrite(s_pose_queue, &pose_snapshot);

        if (LOGGING_ENABLED) {
            Serial.printf(">trajectory:%.4f:%.4f|xy\n"
                          ">heading:%.4f\n"
                          ">valid:%s|t\n",
                          pose_snapshot.pose.x_m,
                          pose_snapshot.pose.y_m,
                          degrees(pose_snapshot.pose.heading_rad),
                          pose_snapshot.valid ? "true" : "false");
        }

        if (xQueuePeek(s_drive_cmd_queue, &cmd, 0) == pdTRUE) {
            s_drive_controller.update(drivetrain, cmd, pose_snapshot, dt_s);
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

    s_drive_controller.clear_reached_pose();
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

bool reached_pose() { return s_drive_controller.reached_pose(); }

void clear_reached_pose() { s_drive_controller.clear_reached_pose(); }

uint32_t get_drive_task_fault() { return s_drive_controller.fault(); }
