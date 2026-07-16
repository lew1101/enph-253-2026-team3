#include "tasks/drive.hpp"
#include "esp_check.h"
#include "esp_err.h"
#include "freertos/idf_additions.h"
#include "portmacro.h"
#include "projdefs.h"
#include "control/PID.hpp"
#include "tasks/tape_sense.hpp"

using namespace control;

static constexpr char TAG[] = "drive_task";

namespace {
TaskHandle_t s_task_handle = nullptr;
QueueHandle_t s_drive_cmd_queue = nullptr;

DriveMode s_mode;

DriveTaskConfig s_task_cfg;
Drivetrain::Config s_drive_cfg;

control::PID tape_pid(15.0f, 0.0f, 1.2f);

void _drive_task(void *arg)
{
    (void)arg;
    Drivetrain drivetrain{s_drive_cfg};

    ESP_ERROR_CHECK(drivetrain.init());

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
                    drivetrain.stop();
                    break;

                case DriveMode::SET_SPEED:
                    drivetrain.move_vector(cmd.x_speed, cmd.y_speed, cmd.rot_speed);
                    break;

                case DriveMode::DRIVE_TO_POSITION:
                    ESP_LOGW(TAG, "DriveMode::DRIVE_TO_POSE not implemented");
                    break;

                case DriveMode::TAPE_FOLLOW:
                    bool success = get_tape_snapshot(&tape_snapshot, 0);
                    if (!success) {
                        ESP_LOGW(TAG, "unable to get tape snapshot");
                        continue;
                    }

                    float rot_correction = -tape_pid.update(0.0f, tape_snapshot.front_err, DT_S);
                    drivetrain.move_vector(0.0f, cmd.tape_follow_speed, rot_correction);
                    break;
            }
        }
    }

    drivetrain.update();
    vTaskDelay(pdMS_TO_TICKS(static_cast<uint32_t>(s_task_cfg.period_ms)));
}
} // namespace

esp_err_t start_drive_task(const DriveTaskConfig &task_cfg,
                           Drivetrain::Config &drivetrain_cfg,
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
