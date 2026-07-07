#include "tasks/drive.hpp"
#include "esp_check.h"
#include "esp_err.h"
#include "freertos/idf_additions.h"
#include "portmacro.h"
#include "projdefs.h"
#include "supervisor.hpp"

using namespace control;
using namespace supervisor;

static constexpr char TAG[] = "drive_task";

namespace {
QueueHandle_t s_drive_cmd_queue = nullptr;
TaskHandle_t s_task_handle = nullptr;

Drivetrain *s_drivetrain = nullptr;
DriveTaskConfig s_task_cfg;

void drive_task(void *arg)
{
    (void)arg;

    while (true) {
        xEventGroupWaitBits(
            g_robot_flags, RobotFlag::ROBOT_FLAG_DRIVE_ENABLED, pdFALSE, pdTRUE, portMAX_DELAY);

        DriveCommand cmd;
        if (xQueuePeek(s_drive_cmd_queue, &cmd, pdMS_TO_TICKS(20)) == pdTRUE) {
            // run drivetrain command
            switch (cmd.mode) {
                case DriveMode::STOP:
                    s_drivetrain->stop();
                    break;

                case DriveMode::SET_SPEED:
                    s_drivetrain->move_vector(cmd.x_speed, cmd.y_speed, cmd.rot_speed);
                    break;

                case DriveMode::DRIVE_TO_POSE:
                    ESP_LOGW(TAG, "DriveMode::DRIVE_TO_POSE not implemented");
                    break;

                case DriveMode::TAPE_FOLLOW:
                    ESP_LOGW(TAG, "DriveMode::TAPE_FOLLOW not implemented");
                    break;
            }
        }

        EventBits_t flags = xEventGroupGetBits(g_robot_flags);
        if (!(flags & RobotFlag::ROBOT_FLAG_DRIVE_ENABLED)) {
            s_drivetrain->stop();
        }
    }
}
} // namespace

esp_err_t start_drive_task(const control::Drivetrain::Config &drivetrain_cfg,
                           const DriveTaskConfig &task_cfg,
                           TaskHandle_t *out_handle)
{
    s_task_cfg = task_cfg;

    if (s_task_handle != nullptr) {
        if (out_handle != nullptr) {
            *out_handle = s_task_handle;
        }

        return ESP_ERR_INVALID_STATE;
    }

    s_drive_cmd_queue = xQueueCreate(1, sizeof(DriveCommand));
    configASSERT(s_drive_cmd_queue != nullptr);

    static Drivetrain drive{drivetrain_cfg};
    s_drivetrain = &drive;

    ESP_RETURN_ON_ERROR(s_drivetrain->init(), TAG, "drivetrain setup failed");

    BaseType_t ok = xTaskCreatePinnedToCore(drive_task,
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
    ESP_RETURN_ON_FALSE(xQueueOverwrite(s_drive_cmd_queue, &cmd),
                        ESP_ERR_INVALID_STATE,
                        TAG,
                        "failed to write drive command");
    return ESP_OK;
}
