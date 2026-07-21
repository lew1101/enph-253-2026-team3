#include "tasks/drive.hpp"
#include "esp_check.h"
#include "esp_err.h"
#include "freertos/idf_additions.h"
#include "portmacro.h"
#include "projdefs.h"
#include "control/tape_pid.hpp"
#include "tasks/tape_sense.hpp"

using namespace control;

static constexpr char TAG[] = "drive_task";

control::TapePID tape_pid(22.0f, 0.0f, 460.0f);

namespace {
TaskHandle_t s_task_handle = nullptr;
QueueHandle_t s_drive_cmd_queue = nullptr;
bool is_stopped = true;
TickType_t stop_timestamp = 0;

float max_vy = 0.0f;
float min_vy = 10.0f;
float current_vy = 0.0f;
float speed_penalty_multiplier = 0.5f; // Multiplier for speed penalty when tape is lost

DriveMode s_mode;

DriveTaskConfig s_task_cfg;
Drivetrain::Config s_drive_cfg;

void _drive_task(void *arg)
{
    (void)arg;
    Drivetrain drivetrain{s_drive_cfg};

    esp_err_t err = drivetrain.init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "drivetrain init failed, deleting task");
        vTaskDelete(nullptr);
    }

    const float DT_S = static_cast<float>(s_task_cfg.period_ms) / 1000;
    TapeSnapshot tape_snapshot{};

    while (true) {
        //         xEventGroupWaitBits(
        //             g_robot_flags, RobotFlag::ROBOT_FLAG_DRIVE_ENABLED, pdFALSE, pdTRUE,
        //             portMAX_DELAY);

        DriveCommand cmd;
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
                        break;
                }
            }

            switch (cmd.mode) {
                case DriveMode::STOP:
                {
                    if (is_stopped != true) {
                        ESP_LOGI(TAG, "Stopping");
                        is_stopped = true;
                        stop_timestamp = xTaskGetTickCount();
                    }
                    bool stop_success = get_tape_snapshot(&tape_snapshot, 0);
                    if (!stop_success) {
                        ESP_LOGW(TAG, "unable to get tape snapshot");
                        continue;
                    }
                    float stop_rot_correction = tape_pid.update(0.0f, tape_snapshot.front_err, DT_S);
                    float stop_rot_speed = -stop_rot_correction / 50; // Apply correction to rotation
                    
                    if (xTaskGetTickCount() - stop_timestamp > pdMS_TO_TICKS(1000)) {
                        drivetrain.stop();
                    }
                    else {
                        drivetrain.move_vector(0.0f, 0.0f, stop_rot_speed);
                    }
                    break;
                }
                case DriveMode::SET_SPEED:
                {
                    drivetrain.move_vector(cmd.x_speed, cmd.y_speed, cmd.rot_speed);
                    break;
                }
                case DriveMode::DRIVE_TO_POSITION:
                {
                    ESP_LOGW(TAG, "DriveMode::DRIVE_TO_POSE not implemented");
                    break;
                }
                case DriveMode::TAPE_FOLLOW:
                {
                    is_stopped = false;
                    bool success = get_tape_snapshot(&tape_snapshot, 0);
                    if (!success) {
                        ESP_LOGW(TAG, "unable to get tape snapshot");
                        continue;
                    }
                    max_vy = cmd.tape_follow_speed;
                    float correction = tape_pid.update(0.0f, tape_snapshot.front_err, DT_S);
                    float rot_speed = -correction; // Apply correction to rotation
                    float speed_penalty = abs(correction) * speed_penalty_multiplier;
                    current_vy = max_vy - speed_penalty;
                    if (current_vy < min_vy) {
                        current_vy = min_vy; // Ensure we don't go below the minimum speed
                    }
                    // ESP_LOGI(TAG, "Tape snapshot: front_err = %f, rot speed: %f", tape_snapshot.front_err, rot_speed);
                    drivetrain.move_vector(0.0f, current_vy, rot_speed);
                    break;
                }
            }
        }

        drivetrain.update();
        vTaskDelay(pdMS_TO_TICKS(static_cast<uint32_t>(s_task_cfg.period_ms)));
    }
}
} // namespace

esp_err_t start_drive_task(const DriveTaskConfig &task_cfg,
                           const Drivetrain::Config &drivetrain_cfg,
                           TaskHandle_t *out_handle)
{
    if (s_task_handle != nullptr) {
        if (out_handle != nullptr) {
            *out_handle = s_task_handle;
        }

        return ESP_ERR_INVALID_STATE;
    }

    s_task_cfg = task_cfg;
    s_drive_cfg = drivetrain_cfg;

    s_drive_cmd_queue = xQueueCreate(1, sizeof(DriveCommand));
    configASSERT(s_drive_cmd_queue != nullptr);

    auto ok = xTaskCreatePinnedToCore(_drive_task,
                                      "drive_task",
                                      s_task_cfg.stack_depth,
                                      nullptr,
                                      s_task_cfg.priority,
                                      &s_task_handle,
                                      s_task_cfg.core_id);
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
